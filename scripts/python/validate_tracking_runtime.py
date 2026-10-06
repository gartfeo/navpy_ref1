"""Production tracking stack and replay operations for SIYI validation."""

from __future__ import annotations

import os
import sys
import time
from collections.abc import Callable
from typing import Any

import cv2
import numpy as np

from navpy.modules.vision.appearance import create_appearance_embedder
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.geometry import cxcywh_to_xyxy
from navpy.modules.vision.lost_target_bridge import LostTargetBridge
from navpy.modules.vision.target_lock import TargetLock
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import create_tracker_backend
from navpy.modules.vision.yolo_detector import YoloDetector
from scripts.python.validate_tracking_config import BUS, CAR, TRUCK


EmitLine = Callable[[str], None]
TrackingStack = tuple[Any, Any, Any, Any, Any, Any]


def build_stack(model: str, logger: Any) -> TrackingStack:
    yolo = YoloDetector(
        model,
        imgsz=640,
        conf=0.25,
        device="auto",
        classes=[CAR, BUS, TRUCK],
        logger=logger,
    )
    mot = create_tracker_backend(
        {
            "backend": "botsort",
            "with_reid": False,
            "reid_device": "auto",
            "reid_weights": "clip_veri.pt",
            "cmc_method": "sof",
            "max_age": 90,
            "min_hits": 3,
            "conf": 0.25,
        },
        detector_device=yolo.device,
        detector_conf=0.25,
        logger=logger,
    )
    identity = TrackIdentityResolver()
    embedder = create_appearance_embedder(
        {
            "enabled": True,
            "reid_device": "auto",
            "half": True,
            "weights": "clip_veri.pt",
        },
        detector_device=yolo.device,
        logger=logger,
    )
    lock = TargetLock(max_lost_frames=120, auto_lock=False)
    bridge = LostTargetBridge()
    return yolo, mot, identity, embedder, lock, bridge


def reset_all(components: TrackingStack) -> None:
    components[1].reset()
    components[2].reset()
    components[4].reset()
    components[5].reset()


def step(
    frame: np.ndarray,
    now: float,
    components: TrackingStack,
    lock_armed: bool,
) -> tuple[list[Any], Any | None]:
    yolo, mot, identity, embedder, lock, bridge = components
    height, width = frame.shape[:2]
    raw = mot.update(
        yolo.detect(frame),
        width,
        height,
        frame=frame,
    )
    embeddings = None
    if embedder is not None and raw:
        values = embedder.embed(
            frame,
            [
                cxcywh_to_xyxy(
                    (track.cx, track.cy, track.w, track.h)
                )
                for track in raw
            ],
        )
        if values is not None and len(values) == len(raw):
            embeddings = {
                int(track.id): values[index]
                for index, track in enumerate(raw)
            }
    if lock_armed and lock.locked_id is not None:
        locked_id = lock.locked_id
        current = next(
            (
                track
                for track in raw
                if identity.stable_of(int(track.id)) == locked_id
            ),
            None,
        )
        if current is not None:
            bridge.observe(
                frame,
                (current.cx, current.cy, current.w, current.h),
                now,
            )
        else:
            hit = bridge.search(frame, now)
            if hit is not None:
                identity.hint_position(
                    locked_id,
                    hit.cx / width,
                    hit.cy / height,
                    now,
                )
    tracks = identity.update(
        raw,
        width,
        height,
        now=now,
        embeddings=embeddings,
    )
    locked = lock.select(tracks, width, height) if lock_armed else None
    if lock_armed:
        identity.pin(lock.locked_id)
    return tracks, locked


def record_clip(
    url: str,
    clip: str,
    record_seconds: float,
    logger: Any,
    emit: EmitLine,
) -> tuple[int, int]:
    provider = FrameProvider(url, logger)
    provider.start()
    started_at = time.time()
    frame = None
    while time.time() - started_at < 15:
        frame, width, height, sequence = provider.get_frame_state()
        if frame is not None:
            break
        time.sleep(0.2)
    if frame is None:
        emit("RECORD FAILED: no frames")
        provider.stop()
        sys.exit(1)
    height, width = frame.shape[:2]
    emit(
        f"recording {width}x{height} for {record_seconds:.0f}s "
        f"-> {os.path.basename(clip)}"
    )
    writer = cv2.VideoWriter(
        clip,
        cv2.VideoWriter_fourcc(*"mp4v"),
        25,
        (width, height),
    )
    count, last_sequence = 0, -1
    started_at = time.time()
    while time.time() - started_at < record_seconds:
        frame, width, height, sequence = provider.get_frame_state()
        if frame is None or sequence == last_sequence:
            time.sleep(0.005)
            continue
        last_sequence = sequence
        writer.write(frame)
        count += 1
    writer.release()
    provider.stop()
    emit(
        f"recorded {count} frames "
        f"({count / record_seconds:.1f} fps)"
    )
    return width, height


__all__ = [
    "build_stack",
    "record_clip",
    "reset_all",
    "step",
]
