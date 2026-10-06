"""Typed adapter from grouped detections to production angular samples."""

from __future__ import annotations

import math
from typing import Protocol

from navpy.modules.vision.gimbal_rate_types import (
    GimbalRateUpdate,
    GimbalTrackResult,
)
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_components import BoundingBox

from .gimbal_tuning_geometry import TrackingCommand, normalize_bbox
from .gimbal_tuning_overlay import OverlayTarget


class TuningTrackerPort(Protocol):
    def update(self, sample: GimbalAngularSample) -> GimbalRateUpdate: ...

    def lose_target(self) -> GimbalTrackResult: ...

    def stop(self) -> GimbalTrackResult: ...


def tracking_bbox(target: DetectedObject | None) -> BoundingBox | None:
    if target is None:
        return None
    return target.tracking.bbox_cxcywh or target.confirmation.bbox_cxcywh


def source_timestamp(target: DetectedObject | None) -> float | None:
    if target is None:
        return None
    value = target.timing.detection_timestamp_s
    if value is None or isinstance(value, bool):
        return None
    try:
        timestamp_s = float(value)
    except (TypeError, ValueError):
        return None
    return timestamp_s if math.isfinite(timestamp_s) else None


def build_overlay_target(
    target: DetectedObject | None,
) -> OverlayTarget | None:
    if target is None:
        return None
    return OverlayTarget(
        target.identity.obj_id,
        normalize_bbox(tracking_bbox(target)),
    )


def build_tracker_target(
    command: TrackingCommand,
    source_target: DetectedObject | None = None,
) -> GimbalAngularSample | None:
    timestamp_s = source_timestamp(source_target)
    if timestamp_s is None:
        return None
    return GimbalAngularSample(
        yaw_error_rad=math.radians(command.delta_yaw_deg),
        pitch_error_rad=math.radians(command.delta_pitch_deg),
        source_timestamp_s=timestamp_s,
    )


def tick_gimbal_tracker(
    tracker: TuningTrackerPort,
    command: TrackingCommand | None,
    source_target: DetectedObject | None,
) -> GimbalTrackResult:
    if command is None:
        return tracker.lose_target()
    sample = build_tracker_target(command, source_target)
    if sample is None:
        return tracker.lose_target()
    return tracker.update(sample).result


__all__ = [
    "TuningTrackerPort",
    "build_overlay_target",
    "build_tracker_target",
    "source_timestamp",
    "tick_gimbal_tracker",
    "tracking_bbox",
]
