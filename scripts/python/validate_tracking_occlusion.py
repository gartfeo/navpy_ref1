"""Locked-POI bus-occlusion replay and proof-frame rendering."""

from __future__ import annotations

import os
from collections.abc import Callable
from typing import Any

import cv2
import numpy as np

from scripts.python.validate_tracking_config import BUS, TRUCK
from scripts.python.validate_tracking_runtime import (
    TrackingStack,
    reset_all,
    step,
)


EmitLine = Callable[[str], None]


def report_occurrence(
    event: dict,
    components: TrackingStack,
    fps: float,
    clip: str,
    evidence_dir: str,
    emit: EmitLine,
) -> None:
    emit(
        f"  car visible until frame {event['fb']} (id {event['ib']}), "
        f"occluded {event['gap']} frames, reappears frame {event['fa']} "
        f"(id {event['ia']})"
    )
    outcome = (
        "SURVIVED"
        if event["ib"] == event["ia"]
        else "changed %d->%d" % (event["ib"], event["ia"])
    )
    emit(f"  WITHOUT lock (resolver only): id {outcome}")
    reset_all(components)
    capture = cv2.VideoCapture(clip)
    lock_frame = max(0, event["fb"] - int(0.5 * fps))
    end_frame = event["fa"] + int(1.0 * fps)
    locked_stable_id = None
    present, samples = 0, 0
    ids: set[int] = set()
    frame_index = 0
    saved: dict[int, np.ndarray] = {}
    poi_stable_id = event["ib"]
    while frame_index <= end_frame:
        ok, frame = capture.read()
        if not ok:
            break
        tracks, locked = step(
            frame,
            frame_index / fps,
            components,
            lock_armed=(locked_stable_id is not None),
        )
        if locked_stable_id is None and frame_index >= lock_frame:
            if any(track.id == poi_stable_id for track in tracks):
                locked_stable_id = poi_stable_id
                components[4].force_lock(locked_stable_id)
                emit(
                    f"  [pass2] locked the occluded car id "
                    f"{locked_stable_id}"
                )
        if (
            locked_stable_id is not None
            and frame_index >= event["fa"]
        ):
            samples += 1
            if locked is not None:
                present += 1
                ids.add(locked.id)
        if frame_index in (
            lock_frame,
            (event["fb"] + event["fa"]) // 2,
            event["fa"],
            end_frame,
        ):
            saved[frame_index] = annotate(
                frame,
                tracks,
                locked,
                frame_index,
            )
        frame_index += 1
    capture.release()
    for saved_frame, image in saved.items():
        cv2.imwrite(
            os.path.join(
                evidence_dir,
                f"bus_occlusion_f{saved_frame}.jpg",
            ),
            image,
        )
    emit(
        f"  WITH lock+bridge+pin: after reappearance locked present "
        f"{present}/{samples} frames; ids seen {sorted(ids)}"
    )
    verdict = (
        "KEPT id %d across the bus occlusion" % locked_stable_id
        if locked_stable_id in ids and len(ids) <= 1
        else f"ids={sorted(ids)}"
    )
    emit(f"  -> RESULT: {verdict}")
    emit(
        "  proof frames: "
        + ", ".join(
            f"bus_occlusion_f{saved_frame}.jpg"
            for saved_frame in sorted(saved)
        )
    )


def annotate(
    frame: np.ndarray,
    tracks: list[Any],
    locked: Any | None,
    frame_index: int,
) -> np.ndarray:
    image = frame.copy()
    for track in tracks:
        x1, y1 = int(track.cx - track.w / 2), int(track.cy - track.h / 2)
        x2, y2 = int(track.cx + track.w / 2), int(track.cy + track.h / 2)
        is_locked = locked is not None and track.id == locked.id
        color = (
            (0, 0, 255)
            if is_locked
            else (
                (0, 200, 255)
                if track.class_id in (BUS, TRUCK)
                else (0, 255, 0)
            )
        )
        cv2.rectangle(
            image,
            (x1, y1),
            (x2, y2),
            color,
            3 if is_locked else 1,
        )
        cv2.putText(
            image,
            str(track.id),
            (x1, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
    status = "LOCKED=%d" % locked.id if locked else "lock=lost"
    cv2.putText(
        image,
        f"f{frame_index} " + status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )
    return image


__all__ = ["annotate", "report_occurrence"]
