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
from .gimbal_tuning_overlay import OverlayPoi


class TuningTrackerPort(Protocol):
    def update(self, sample: GimbalAngularSample) -> GimbalRateUpdate: ...

    def lose_poi(self) -> GimbalTrackResult: ...

    def stop(self) -> GimbalTrackResult: ...


def tracking_bbox(poi: DetectedObject | None) -> BoundingBox | None:
    if poi is None:
        return None
    return poi.tracking.bbox_cxcywh or poi.confirmation.bbox_cxcywh


def source_timestamp(poi: DetectedObject | None) -> float | None:
    if poi is None:
        return None
    value = poi.timing.detection_timestamp_s
    if value is None or isinstance(value, bool):
        return None
    try:
        timestamp_s = float(value)
    except (TypeError, ValueError):
        return None
    return timestamp_s if math.isfinite(timestamp_s) else None


def build_overlay_poi(
    poi: DetectedObject | None,
) -> OverlayPoi | None:
    if poi is None:
        return None
    return OverlayPoi(
        poi.identity.obj_id,
        normalize_bbox(tracking_bbox(poi)),
    )


def build_tracker_poi(
    command: TrackingCommand,
    source_poi: DetectedObject | None = None,
) -> GimbalAngularSample | None:
    timestamp_s = source_timestamp(source_poi)
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
    source_poi: DetectedObject | None,
) -> GimbalTrackResult:
    if command is None:
        return tracker.lose_poi()
    sample = build_tracker_poi(command, source_poi)
    if sample is None:
        return tracker.lose_poi()
    return tracker.update(sample).result


__all__ = [
    "TuningTrackerPort",
    "build_overlay_poi",
    "build_tracker_poi",
    "source_timestamp",
    "tick_gimbal_tracker",
    "tracking_bbox",
]
