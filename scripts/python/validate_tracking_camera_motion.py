"""Virtual-gimbal camera-motion validation over recorded SIYI footage."""

from __future__ import annotations

from collections.abc import Callable

import cv2
import numpy as np

from scripts.python.validate_tracking_config import CAR
from scripts.python.validate_tracking_metrics import associate
from scripts.python.validate_tracking_runtime import (
    TrackingStack,
    reset_all,
    step,
)


EmitLine = Callable[[str], None]


def camera_motion(
    components: TrackingStack,
    fps: float,
    clip: str,
    emit: EmitLine,
) -> None:
    output_width, output_height = 1280, 720
    base_width, base_height = 1280, 720
    segments = [
        ("static", 0, 3, lambda value: (1280, 480, 1.0)),
        (
            "pan",
            3,
            6,
            lambda value: (1280 + 170 * value, 480 - 80 * value, 1.0),
        ),
        (
            "tilt",
            6,
            9,
            lambda value: (1450, 400 - 60 * value, 1.0),
        ),
        (
            "zoom_in",
            9,
            12,
            lambda value: (1450, 340, 1.0 + value),
        ),
        (
            "zoom_out",
            12,
            15,
            lambda value: (1450, 340, 2.0 - value),
        ),
    ]

    def view(frame: np.ndarray, timestamp: float) -> tuple[np.ndarray, str]:
        for name, start, end, transform in segments:
            if start <= timestamp < end:
                fraction = (timestamp - start) / max(
                    1e-9,
                    end - start,
                )
                center_x, center_y, zoom = transform(fraction)
                break
        else:
            name = segments[-1][0]
            center_x, center_y, zoom = segments[-1][3](1.0)
        width, height = base_width / zoom, base_height / zoom
        x1 = int(
            np.clip(
                center_x - width / 2,
                0,
                frame.shape[1] - width,
            )
        )
        y1 = int(
            np.clip(
                center_y - height / 2,
                0,
                frame.shape[0] - height,
            )
        )
        crop = frame[
            y1 : y1 + int(height),
            x1 : x1 + int(width),
        ]
        return (
            cv2.resize(crop, (output_width, output_height)),
            name,
        )

    reset_all(components)
    capture = cv2.VideoCapture(clip)
    frames: list[list[tuple[int, tuple[float, ...], int]]] = []
    segment_of: list[str] = []
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        timestamp = frame_index / fps
        if timestamp >= segments[-1][2]:
            break
        virtual_frame, segment_name = view(frame, timestamp)
        tracks, _ = step(
            virtual_frame,
            timestamp,
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
        segment_of.append(segment_name)
        frame_index += 1
    capture.release()
    tracklets = associate(frames, coast=int(fps))
    margin = 40
    probes = [
        tracklet
        for tracklet in tracklets
        if tracklet["cls"] == CAR
        and len(tracklet["pts"]) >= 0.9 * len(frames)
        and all(
            point[2][0] - point[2][2] / 2 > margin
            and point[2][0] + point[2][2] / 2 < output_width - margin
            and point[2][1] - point[2][3] / 2 > margin
            and point[2][1] + point[2][3] / 2 < output_height - margin
            for point in tracklet["pts"]
        )
    ]
    switches = 0
    for tracklet in probes:
        ids = [point[1] for point in tracklet["pts"]]
        switches += sum(
            1
            for index in range(1, len(ids))
            if ids[index] != ids[index - 1]
        )
    emit(
        "  segments: static->pan->tilt->zoom_in->zoom_out "
        "over real footage"
    )
    emit(
        f"  interior probe cars (full-span, always in view): "
        f"n={len(probes)}"
    )
    emit(
        f"  id switches on probes during camera motion: {switches} "
        f"-> {'STABLE' if switches == 0 else 'some churn'}"
    )
    emit(f"  resolver counters: {components[2].counters}")


__all__ = ["camera_motion"]
