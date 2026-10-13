r"""Live check of the Unreal camera stream through navpy's npyc:// frame-capture backend.

Listens like navpy does (the Unreal camera connects), stamps every frame with the backend's own
rules, and writes one CSV row per frame plus sample PNGs; the ChArUco detector (navpy's default
board) runs on every --detect-every-th frame in a worker, so reading never waits for it.

Board clock (what turns a frame's board time into an EXPOSURE stamp):
  --mavlink URL     ATTITUDE from the autopilot (SITL or the board), exactly as navpy maps it
                    (navpy.core.perception.camera.board_clock_map.AttitudeBoardClock);
  --pose-at ...     bench stand-in for the HITL bridge: this tool sends a fixed vehicle_state.v1
                    (unreal_pose.encode_vehicle_state) every 20 ms with its own board clock, and
                    records each (board time, send time) as the clock's sample, so latency is
                    "pose sent -> frame received".

  .venv\Scripts\python.exe -m tools.unreal.npyc_live_check --mavlink tcp:127.0.0.1:5762 --seconds 30 --out C:\tune_hitl\cam_check
  .venv\Scripts\python.exe -m tools.unreal.npyc_live_check --pose-at 40.31205,44.45800,1335.4,0,-35,90 --seconds 20 --out C:\tune_hitl\cam_bench

Latency in the CSV: frame receipt minus the mapped board time of its pose (EXPOSURE frames only).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import statistics
import threading
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import cv2
import numpy as np

from navpy.core.perception.camera.board_clock_map import BOARD_CLOCK_HISTORY_COUNT, AttitudeBoardClock
from navpy.core.ports.frame_capture_ports import CaptureStamp, CaptureStampKind
from navpy.plugins.camera.unreal.frame_capture_npyc import NpycFrameCapture
from navpy.plugins.camera.unreal.npyc_codec import NpycHeader
from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec
from navpy.plugins.detection.charuco.charuco_detector import CharucoBoardDetector
from tools.tuning.hitl_bridge.physics import PlantState
from tools.tuning.hitl_bridge.unreal_pose import POSE_PERIOD_US, encode_vehicle_state, parse_endpoint

COLUMNS = ("frame_index", "version", "board_time_us", "pose_sequence", "pose_flags", "unreal_capture_s",
           "read_at_s", "stamp_kind", "stamp_s", "latency_ms", "detected", "confidence")


class _Logger:
    def info(self, message: str) -> None:
        print(message, flush=True)

    warning = error = info


class ClockSamples:
    """ATTITUDE-like (time_boot_s, receipt_time_s) samples for AttitudeBoardClock."""

    def __init__(self) -> None:
        self._samples: deque = deque(maxlen=BOARD_CLOCK_HISTORY_COUNT)
        self._lock = threading.Lock()

    def add(self, boot_s: float, receipt_s: float) -> None:
        with self._lock:
            self._samples.append(SimpleNamespace(time_boot_s=boot_s, receipt_time_s=receipt_s))

    def history(self, count: int) -> list:
        with self._lock:
            return list(self._samples)[-count:]


def read_attitude(url: str, samples: ClockSamples, stop: threading.Event) -> None:
    from pymavlink import mavutil   # noqa: PLC0415 - only with --mavlink
    link = mavutil.mavlink_connection(url, source_system=254)
    link.wait_heartbeat(timeout=30)
    # ATTITUDE every 20 ms on this link (MAV_CMD_SET_MESSAGE_INTERVAL; per link, nothing else changes)
    link.mav.command_long_send(link.target_system, link.target_component, 511, 0, 30, 20000, 0, 0, 0, 0, 0)
    while not stop.is_set():
        message = link.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
        if message is not None:
            samples.add(message.time_boot_ms * 1e-3, time.time())
    link.close()


def send_fixed_pose(pose: list[float], endpoint: tuple[str, int], samples: ClockSamples, stop: threading.Event) -> None:
    lat, lon, alt, roll, pitch, yaw = pose
    state = PlantState(lat, lon, alt, math.radians(roll), math.radians(pitch), math.radians(yaw),
                       *([0.0] * 11))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    sequence = 0
    while not stop.wait(POSE_PERIOD_US / 1e6):
        board_us = int((time.monotonic() - start) * 1e6)
        datagram = encode_vehicle_state(state, board_us, sequence, "npyc-live-check", 0.0, 1)
        sent_at = time.time()
        sock.sendto(datagram, endpoint)
        samples.add(board_us / 1e6, sent_at)
        sequence += 1
    sock.close()


class Recorder:
    def __init__(self, out: Path, save_every: int, detect_every: int) -> None:
        self.out = out
        self.save_every = save_every
        self.detect_every = detect_every
        self.rows: dict[int, dict] = {}
        self.detector = CharucoBoardDetector(CharucoBoardSpec())
        self.work: deque = deque(maxlen=4)      # newest frames only: detection must never slow reading
        self.wake = threading.Event()
        self.count = 0
        self.lock = threading.Lock()

    def observe(self, header: NpycHeader, read_at_s: float, stamp: CaptureStamp) -> None:
        exposure = stamp.kind is CaptureStampKind.EXPOSURE
        with self.lock:
            self.rows[header.frame_index] = {
                "frame_index": header.frame_index, "version": header.version, "board_time_us": header.board_time_us,
                "pose_sequence": header.pose_sequence, "pose_flags": header.pose_flags,
                "unreal_capture_s": f"{header.capture_time_s:.6f}", "read_at_s": f"{read_at_s:.6f}",
                "stamp_kind": stamp.kind.value, "stamp_s": f"{stamp.captured_at_s:.6f}",
                "latency_ms": f"{(read_at_s - stamp.captured_at_s) * 1e3:.2f}" if exposure else "",
                "detected": "", "confidence": ""}
        self._last_header = header

    def publish(self, frame: np.ndarray, stamp: Optional[CaptureStamp]) -> None:
        index = self._last_header.frame_index
        self.count += 1
        if self.count % self.detect_every == 0 or self.count % self.save_every == 0:
            self.work.append((index, frame, self.count % self.save_every == 0))
            self.wake.set()

    def detect_loop(self, stop: threading.Event) -> None:
        while not stop.is_set() or self.work:
            self.wake.wait(0.2)
            self.wake.clear()
            while self.work:
                index, frame, save = self.work.popleft()
                found = self.detector.detect(frame)
                with self.lock:
                    row = self.rows[index]
                    row["detected"] = int(bool(found))
                    row["confidence"] = f"{found[0].confidence:.2f}" if found else ""
                if save or found:
                    image = frame.copy()
                    for det in found:
                        cv2.rectangle(image, (int(det.cx - det.w / 2), int(det.cy - det.h / 2)),
                                      (int(det.cx + det.w / 2), int(det.cy + det.h / 2)), (0, 0, 255), 2)
                    name = f"frame_{index:06d}{'_charuco' if found else ''}.png"
                    if save or not any(self.out.glob("*_charuco.png")) or index % 30 == 0:
                        cv2.imwrite(str(self.out / name), image)


def summarize(rows: list[dict], seconds: float) -> dict:
    indices = sorted(int(r["frame_index"]) for r in rows)
    gaps = sum(b - a - 1 for a, b in zip(indices, indices[1:]) if b > a + 1)
    latencies = sorted(float(r["latency_ms"]) for r in rows if r["latency_ms"])

    def pct(p: float) -> Optional[float]:
        return round(latencies[min(len(latencies) - 1, int(p * len(latencies)))], 2) if latencies else None
    return {
        "frames": len(rows), "seconds": round(seconds, 2), "fps": round(len(rows) / seconds, 2) if seconds else 0,
        "frame_index_gaps": gaps,
        "exposure_stamped": sum(r["stamp_kind"] == "exposure" for r in rows),
        "latency_ms": {"n": len(latencies), "min": pct(0.0), "p50": pct(0.5), "p90": pct(0.9), "p99": pct(0.99),
                       "max": latencies[-1] if latencies else None,
                       "mean": round(statistics.fmean(latencies), 2) if latencies else None},
        "detected": sum(r["detected"] == 1 for r in rows),
        "detection_runs": sum(r["detected"] != "" for r in rows),
    }


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--source", default="npyc://127.0.0.1:18110")
    clock = parser.add_mutually_exclusive_group()
    clock.add_argument("--mavlink", default=None, help="pymavlink URL with ATTITUDE (the board clock)")
    clock.add_argument("--pose-at", default=None, help="LAT,LON,ELLIPSOID_ALT,ROLL,PITCH,YAW: bench stand-in sender")
    parser.add_argument("--pose-endpoint", default="127.0.0.1:17110")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--save-every", type=int, default=150, help="save every Nth frame as PNG")
    parser.add_argument("--detect-every", type=int, default=3, help="run the ChArUco detector on every Nth frame")
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    samples = ClockSamples()
    stop = threading.Event()
    threads = []
    if args.mavlink:
        threads.append(threading.Thread(target=read_attitude, args=(args.mavlink, samples, stop), daemon=True))
    elif args.pose_at:
        pose = [float(v) for v in args.pose_at.split(",")]
        if len(pose) != 6:
            parser.error("--pose-at needs 6 numbers")
        threads.append(threading.Thread(target=send_fixed_pose,
                                        args=(pose, parse_endpoint(args.pose_endpoint), samples, stop), daemon=True))
    recorder = Recorder(args.out, args.save_every, args.detect_every)
    board_clock = AttitudeBoardClock(samples.history) if threads else None
    backend = NpycFrameCapture.open(args.source, _Logger(), board_clock, recorder.observe)
    if backend is None:
        return 2
    detector_thread = threading.Thread(target=recorder.detect_loop, args=(stop,), daemon=True)
    for thread in (*threads, detector_thread):
        thread.start()
    reader = threading.Thread(target=backend.run, args=(recorder.publish, lambda: not stop.is_set()), daemon=True)
    reader.start()
    first_frame_wait = time.monotonic()
    started: Optional[float] = None
    while not stop.is_set():
        time.sleep(0.1)
        if started is None and recorder.count:
            started = time.monotonic()
        if started is not None and time.monotonic() - started >= args.seconds:
            break
        if started is None and time.monotonic() - first_frame_wait > max(120.0, args.seconds):
            print("no frame arrived", flush=True)
            break
    elapsed = time.monotonic() - started if started is not None else 0.0
    stop.set()
    backend.close()
    reader.join(2.0)
    detector_thread.join(5.0)
    rows = [recorder.rows[k] for k in sorted(recorder.rows)]
    with (args.out / "frames.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows, elapsed)
    summary["connections"] = backend.connections
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
