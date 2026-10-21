"""Recorder for the capture-cadence check: listen like navpy, keep every NPYC frame's header, the
CRC32 of its pixels and the ChArUco corners detected in it (JSONL)."""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
import zlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from navpy.plugins.camera.unreal.npyc_codec import (NPYC_PREFIX_SIZE, NPYC_PIXEL_FORMAT_BGR8, NpycHeader,
                                                    decode_header, decode_prefix)
from tools.unreal.dock_pass_geometry import BOARD_SPEC

DEFAULT_LISTEN = "127.0.0.1:18110"    # NavPyGimbalCameraActor NativeCameraPort


# --- record --------------------------------------------------------------------------------------------------------
class _Deadline(Exception):
    """The recording time ran out; ``partial`` when bytes of the read were already taken."""

    def __init__(self, partial: bool) -> None:
        super().__init__()
        self.partial = partial


def _read_exact(conn: socket.socket, size: int, deadline: float,
                clock: Callable[[], float] = time.monotonic) -> Optional[bytes]:
    """Exactly size bytes, waiting across receive timeouts (the bytes so far are kept) until the deadline, checked
    before every read; None at the end of the stream."""
    chunks, have = [], 0
    while have < size:
        if clock() >= deadline:
            raise _Deadline(have > 0)
        try:
            chunk = conn.recv(min(size - have, 1 << 20))
        except socket.timeout:
            continue
        if not chunk:
            return None
        chunks.append(chunk)
        have += len(chunk)
    return b"".join(chunks)


def read_frames(conn: socket.socket, deadline: float, on_frame: Callable,
                clock: Callable[[], float] = time.monotonic) -> str:
    """Calls on_frame(header, pixels) for each whole NPYC frame; returns why it stopped: "closed", "deadline"
    (between frames) or "cut" (the deadline inside a frame, which is not reported)."""
    while True:
        try:
            prefix = _read_exact(conn, NPYC_PREFIX_SIZE, deadline, clock)
        except _Deadline as stop:
            return "cut" if stop.partial else "deadline"
        if prefix is None:
            return "closed"
        try:
            _version, header_size = decode_prefix(prefix)
            rest = (_read_exact(conn, header_size - NPYC_PREFIX_SIZE, deadline, clock)
                    if header_size > NPYC_PREFIX_SIZE else b"")
            if rest is None:
                return "closed"
            header = decode_header(prefix + rest)
            pixels = _read_exact(conn, header.payload_size, deadline, clock)
        except _Deadline:
            return "cut"
        if pixels is None:
            return "closed"
        on_frame(header, pixels)


def detect_corners(pixels: bytes, width: int, height: int) -> list[list[float]]:
    import cv2
    from navpy.plugins.detection.charuco.charuco_detector import build_charuco_board, build_charuco_detector

    image = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = build_charuco_detector(build_charuco_board(BOARD_SPEC)).detectBoard(gray)
    if ids is None:
        return []
    return [[int(i), float(c[0]), float(c[1])] for i, c in zip(np.asarray(ids).reshape(-1),
                                                               np.asarray(corners).reshape(-1, 2))]


def record(listen: tuple[str, int], out: Path, seconds: float, workers: int = 4,
           listening: Optional[threading.Event] = None) -> dict:
    """Writes one JSON line per frame; corners null when the detector pool was behind (reading never waits).
    ``listening`` is set once the port accepts connections."""
    stats = {"frames": 0, "undetected_busy": 0, "connections": 0, "stops": []}
    lock = threading.Lock()
    pool = ThreadPoolExecutor(max_workers=workers)
    pending: queue.Queue = queue.Queue(maxsize=4 * workers)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as sink, socket.create_server(listen) as server:
        def write(row: dict) -> None:
            with lock:
                sink.write(json.dumps(row, separators=(",", ":")) + "\n")

        def finish(row: dict, pixels: bytes, width: int, height: int) -> None:
            try:
                row["corners"] = detect_corners(pixels, width, height)
            finally:
                pending.get_nowait()
                write(row)

        def on_frame(header: NpycHeader, pixels: bytes) -> None:
            stats["frames"] += 1
            row = {"conn": stats["connections"], "frame": header.frame_index, "board_us": header.board_time_us,
                   "seq": header.pose_sequence, "flags": header.pose_flags, "crc": zlib.crc32(pixels),
                   "w": header.width, "h": header.height, "corners": None}
            if header.pixel_format == NPYC_PIXEL_FORMAT_BGR8:
                try:
                    pending.put_nowait(True)
                    pool.submit(finish, row, pixels, header.width, header.height)
                    return
                except queue.Full:
                    stats["undetected_busy"] += 1
            write(row)

        deadline = time.monotonic() + seconds
        server.settimeout(1.0)
        if listening is not None:
            listening.set()
        while time.monotonic() < deadline:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            stats["connections"] += 1
            with conn:
                conn.settimeout(1.0)
                stats["stops"].append(read_frames(conn, deadline, on_frame))
        pool.shutdown(wait=True)
    return stats


def parse_endpoint(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    return host, int(port)
