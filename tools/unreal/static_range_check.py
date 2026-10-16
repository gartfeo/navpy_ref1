r"""Static range check of the Unreal native camera on the dock's ChArUco board: the vehicle held still at fixed poses
whose camera looks straight at the board centre from given slant ranges, ChArUco run on every frame of each pose.

No motion, so no motion blur or pose timing: what is left is resolution (pixels per square) and the detector. A bench
stand-in for the HITL bridge like npyc_live_check --pose-at: this tool sends the vehicle_state.v1 poses itself
(unreal_pose.encode_vehicle_state) and takes only frames rendered from the current pose (NPYC v2 pose sequence, exact
flag). The truth is the pose it sends; nothing reaches navpy's navigation.

  .venv\Scripts\python.exe -m tools.unreal.static_range_check --out C:\tune_hitl\static_zr10
  .venv\Scripts\python.exe -m tools.unreal.static_range_check --camera fov90 --out C:\tune_hitl\static_fov90

Unreal as in hitl_unreal_camera.md "Running it" step 1 (header v2, exact pose, fixed body camera). --camera only
selects the scoring geometry and must match that launch: zr10 (and --zoom) with -NavPyNativeCameraIntrinsics=1
(and -NavPyNativeCameraZoom), fov90 without it.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import socket
import statistics
import threading
import time
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

from navpy.core.ports.frame_capture_ports import CaptureStamp
from navpy.plugins.camera.unreal.frame_capture_npyc import NpycFrameCapture
from navpy.plugins.camera.unreal.npyc_codec import NpycHeader
from tools.tuning.dock_pass import DOCK_BOARD_YEGHVARD
from tools.tuning.hitl_bridge.physics import PlantState
from tools.tuning.hitl_bridge.unreal_pose import POSE_PERIOD_US, encode_vehicle_state, parse_endpoint
from tools.unreal.dock_pass_geometry import (BOARD_HALF_EAST_M, BOARD_HALF_NORTH_M, CAMERA_OFFSET_BODY_M, CAMERAS,
                                             MOUNT_PITCH_DEG, CameraModel, Truth, View, board_view, body_to_ned,
                                             camera_named, detected_pose, metres_per_degree)
from tools.unreal.npyc_live_check import _Logger

DEFAULT_RANGES_M = (10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 80.0)
# The geoid undulation at Yeghvard (hitl_unreal_camera.md "Rules"): the poses go to Unreal as ellipsoid heights.
YEGHVARD_UNDULATION_M = 23.5
SQUARE_M = 0.4                        # the Unreal board's squares (dock_pass_geometry.BOARD_SPEC)
# A pose's frames must arrive within this: Unreal renders at 15-20 fps, so a pose that yields none in this long means
# Unreal is not rendering the poses this tool sends (wrong port, not the exact-pose receiver).
POSE_FRAMES_TIMEOUT_S = 30.0
POSE_EXACT_FLAGS = 3                  # NPYC v2 pose_flags: stamp valid | exact


def pose_for_range(board: tuple[float, float, float], slant_m: float, heading_deg: float) -> Truth:
    """A level vehicle on this heading whose fixed camera's optical axis passes through the board centre at
    ``slant_m`` (the camera's depression is the mount pitch)."""
    blat, blon, bmsl = board
    down = math.radians(-MOUNT_PITCH_DEG)
    heading = math.radians(heading_deg)
    axis_ned = np.array([math.cos(heading) * math.cos(down), math.sin(heading) * math.cos(down), math.sin(down)])
    camera_ned = -slant_m * axis_ned                                    # from the board centre
    vehicle_ned = camera_ned - body_to_ned(0.0, 0.0, heading_deg) @ np.array(CAMERA_OFFSET_BODY_M)
    north_per_deg, east_per_deg = metres_per_degree(blat)
    return Truth(blat + vehicle_ned[0] / north_per_deg, blon + vehicle_ned[1] / east_per_deg,
                 bmsl - vehicle_ned[2], 0.0, 0.0, heading_deg)


def pixels_per_square(corners_px: np.ndarray) -> tuple[float, float]:
    """Mean square size on the picture across (east-west, 6 squares with the margin) and along (north-south, 8)
    the board, from its projected outline (NW, NE, SE, SW)."""
    across = 0.5 * (np.linalg.norm(corners_px[1] - corners_px[0]) + np.linalg.norm(corners_px[2] - corners_px[3]))
    along = 0.5 * (np.linalg.norm(corners_px[3] - corners_px[0]) + np.linalg.norm(corners_px[2] - corners_px[1]))
    return float(across / (2 * BOARD_HALF_EAST_M / SQUARE_M)), float(along / (2 * BOARD_HALF_NORTH_M / SQUARE_M))


class PoseSender:
    """Sends the current pose every 20 ms of its own board clock; ``set`` returns the first sequence of the new pose.

    Unreal stamps a frame with the sequence of the pose it applied, from whichever sender, and keeps showing the last
    one when the poses stop. The sequence therefore starts at the wall clock in microseconds: above every sequence an
    earlier run of this tool sent (one per 20 ms, far fewer than a microsecond count), so a frame still rendered from
    an earlier pose cannot pass for this run's."""

    def __init__(self, endpoint: tuple[str, int], undulation_m: float, first_sequence: Optional[int] = None) -> None:
        self._endpoint = endpoint
        self._undulation = undulation_m
        self._lock = threading.Lock()
        self._state: Optional[PlantState] = None
        self._sequence = time.time_ns() // 1000 if first_sequence is None else first_sequence
        self._start = time.monotonic()

    def set(self, truth: Truth) -> int:
        with self._lock:
            self._state = PlantState(truth.lat, truth.lon, truth.alt_msl, math.radians(truth.roll),
                                     math.radians(truth.pitch), math.radians(truth.yaw), *([0.0] * 11))
            return self._sequence

    def run(self, stop: threading.Event) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not stop.wait(POSE_PERIOD_US / 1e6):
            with self._lock:
                state, sequence = self._state, self._sequence
                if state is not None:
                    self._sequence += 1
            if state is not None:
                board_us = int((time.monotonic() - self._start) * 1e6)
                sock.sendto(encode_vehicle_state(state, board_us, sequence, "static-range-check", self._undulation, 1),
                            self._endpoint)
        sock.close()


def frames_of_pose(frames: queue.Queue, first_sequence: int, count: int, timeout_s: float,
                   clock: Callable[[], float] = time.monotonic) -> list[np.ndarray]:
    """Up to ``count`` frames rendered from the pose whose first sequence is ``first_sequence``, waiting at most
    ``timeout_s``; frames of earlier poses still in the queue are dropped."""
    got: list[np.ndarray] = []
    deadline = clock() + timeout_s
    while len(got) < count and clock() < deadline:
        try:
            sequence, frame = frames.get(timeout=max(0.0, deadline - clock()))
        except queue.Empty:
            break
        if sequence >= first_sequence:
            got.append(frame)
    return got


def measure_range(frames: list[np.ndarray], view: View, camera: CameraModel, slant_m: float) -> dict:
    """Detection and PnP range error over one pose's frames."""
    poses = [detected_pose(frame, camera) for frame in frames]
    hits = [p for p in poses if p is not None]
    across, along = pixels_per_square(view.corners_px)
    range_errors = [float(np.linalg.norm(p["centre"]) - np.linalg.norm(view.centre_cam)) for p in hits]
    return {"slant_m": slant_m, "in_view": view.in_view, "px_per_square_across": round(across, 2),
            "px_per_square_along": round(along, 2), "frames": len(frames), "detected": len(hits),
            "corners_median": statistics.median([p["count"] for p in hits]) if hits else 0,
            "range_error_m_median": round(statistics.median(range_errors), 3) if hits else None}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--source", default="npyc://127.0.0.1:18110")
    parser.add_argument("--pose-endpoint", default="127.0.0.1:17110")
    parser.add_argument("--camera", choices=CAMERAS, default="zr10", help="the native camera Unreal renders with")
    parser.add_argument("--zoom", type=float, default=1.0, help="zr10 only: Unreal's -NavPyNativeCameraZoom")
    parser.add_argument("--ranges", default=",".join(f"{r:g}" for r in DEFAULT_RANGES_M), help="slant ranges in m")
    parser.add_argument("--heading", type=float, default=0.0, help="vehicle heading, deg from north")
    parser.add_argument("--frames", type=int, default=10, help="frames per pose")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        camera = camera_named(args.camera, args.zoom)
    except ValueError as error:
        parser.error(str(error))
    args.out.mkdir(parents=True, exist_ok=True)
    board = tuple(float(v) for v in DOCK_BOARD_YEGHVARD.split(","))
    ranges = [float(v) for v in args.ranges.split(",")]

    sender = PoseSender(parse_endpoint(args.pose_endpoint), YEGHVARD_UNDULATION_M)
    frames: queue.Queue = queue.Queue()        # (pose sequence, frame) of every exact-pose frame
    headers: dict[int, NpycHeader] = {}

    def observe(header: NpycHeader, read_at_s: float, stamp: CaptureStamp) -> None:
        headers[threading.get_ident()] = header

    def publish(frame: np.ndarray, stamp: Optional[CaptureStamp]) -> None:
        header = headers.get(threading.get_ident())
        if header is not None and header.pose_flags & POSE_EXACT_FLAGS == POSE_EXACT_FLAGS:
            frames.put((header.pose_sequence, frame))

    backend = NpycFrameCapture.open(args.source, _Logger(), None, observe)
    if backend is None:
        return 2
    stop = threading.Event()
    threads = [threading.Thread(target=sender.run, args=(stop,), daemon=True),
               threading.Thread(target=backend.run, args=(publish, lambda: not stop.is_set()), daemon=True)]
    for thread in threads:
        thread.start()
    rows = []
    try:
        for slant in ranges:
            truth = pose_for_range(board, slant, args.heading)
            got = frames_of_pose(frames, sender.set(truth), args.frames, POSE_FRAMES_TIMEOUT_S)
            row = measure_range(got, board_view(truth, board, camera), camera, slant)
            rows.append(row)
            print(json.dumps(row), flush=True)
            for i, frame in enumerate(got):
                cv2.imwrite(str(args.out / f"range{slant:05.1f}m_{i:02d}.png"), frame)
            if not got:
                print(f"no frame of the {slant:g} m pose in {POSE_FRAMES_TIMEOUT_S:g} s", flush=True)
                break
    finally:
        stop.set()
        backend.close()
    summary = {"camera": camera.name, "fx": camera.fx, "fy": camera.fy, "heading_deg": args.heading,
               "mount_pitch_deg": MOUNT_PITCH_DEG, "ranges": rows}
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1))
    return 0 if rows and all(r["frames"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
