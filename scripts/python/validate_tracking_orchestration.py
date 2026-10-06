"""Top-level sequencing for comprehensive SIYI tracking validation."""

from __future__ import annotations

import traceback
from functools import partial
from typing import Any

import cv2

from scripts.python.validate_tracking_camera_motion import camera_motion
from scripts.python.validate_tracking_config import BUS, CAR, TRUCK
from scripts.python.validate_tracking_metrics import (
    append_metric_line,
    associate,
    find_occ,
    report_identity_stability,
    report_recognition,
)
from scripts.python.validate_tracking_occlusion import report_occurrence
from scripts.python.validate_tracking_runtime import (
    TrackingStack,
    build_stack,
    record_clip,
    step,
)


def replay_clip(
    clip: str,
    components: TrackingStack,
    emit: Any,
) -> tuple[
    float,
    list[list[tuple[int, tuple[float, ...], int]]],
    list[list[tuple[float, ...]]],
    list[int],
]:
    capture = cv2.VideoCapture(clip)
    fps = capture.get(cv2.CAP_PROP_FPS) or 25.0
    frames: list[list[tuple[int, tuple[float, ...], int]]] = []
    bus_boxes: list[list[tuple[float, ...]]] = []
    car_counts: list[int] = []
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        tracks, _ = step(
            frame,
            frame_index / fps,
            components,
            lock_armed=False,
        )
        frames.append(
            [
                (
                    int(track.id),
                    (track.cx, track.cy, track.w, track.h),
                    int(track.class_id),
                )
                for track in tracks
            ]
        )
        bus_boxes.append(
            [
                (track.cx, track.cy, track.w, track.h)
                for track in tracks
                if track.class_id in (BUS, TRUCK)
            ]
        )
        car_counts.append(
            sum(1 for track in tracks if track.class_id == CAR)
        )
        frame_index += 1
        if frame_index % 300 == 0:
            emit(f"  ...replayed {frame_index} frames")
    capture.release()
    return fps, frames, bus_boxes, car_counts


def run_validation(
    record_seconds: float,
    metrics_path: str,
    url: str,
    model: str,
    clip: str,
    evidence_dir: str,
    logger: Any,
) -> None:
    open(metrics_path, "w", encoding="utf-8").close()
    emit = partial(append_metric_line, metrics_path=metrics_path)
    emit("=" * 70)
    emit("COMPREHENSIVE REAL-SIYI TRACKING VALIDATION")
    emit("=" * 70)
    frame_width, frame_height = record_clip(
        url,
        clip,
        record_seconds,
        logger,
        emit,
    )
    components = build_stack(model, logger)
    fps, frames, bus_boxes, car_counts = replay_clip(
        clip,
        components,
        emit,
    )
    frame_count = len(frames)
    report_recognition(car_counts, frame_count, emit)
    tracklets = associate(frames, coast=int(fps))
    report_identity_stability(
        tracklets,
        fps,
        frame_width,
        frame_height,
        emit,
    )

    emit("\n[3] BUS / LARGE-VEHICLE OCCLUSION (locked id survival)")
    try:
        event = find_occ(tracklets, bus_boxes, fps)
        if event is None:
            present_frames = sum(1 for boxes in bus_boxes if boxes)
            emit(
                "  no bus-occludes-car event captured in this clip "
                f"(bus/truck present in {present_frames} frames)"
            )
        else:
            report_occurrence(
                event,
                components,
                fps,
                clip,
                evidence_dir,
                emit,
            )
    except Exception:
        emit("  SECTION FAILED:\n" + traceback.format_exc())

    emit(
        "\n[4] CAMERA MOTION "
        "(virtual gimbal pan/tilt/zoom over the real clip)"
    )
    try:
        camera_motion(
            build_stack(model, logger),
            fps,
            clip,
            emit,
        )
    except Exception:
        emit("  SECTION FAILED:\n" + traceback.format_exc())

    emit("\n" + "=" * 70)
    emit("evidence saved under validation_evidence/")


__all__ = ["replay_clip", "run_validation"]
