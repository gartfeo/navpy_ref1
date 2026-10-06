"""Pure image geometry for target containment and zoom entry."""

from __future__ import annotations

import math

from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult, TrackingState
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_size import characteristic_pixels
from navpy.modules.vision.target_zoom_types import ZoomGeometry, ZoomObservation


FRAME_MARGIN_RATIO = 0.08


def extract_bbox(target: object) -> tuple[float, float, float, float] | None:
    if target is None:
        return None
    if isinstance(target, DetectedObject):
        bbox = target.tracking.bbox_cxcywh
        if bbox is None:
            bbox = target.confirmation.bbox_cxcywh
    else:
        bbox = getattr(target, "tracking_bbox_cxcywh", None)
        if bbox is None:
            bbox = getattr(target, "bbox_cxcywh", None)
    if bbox is None or len(bbox) < 4:
        return None
    try:
        values = tuple(float(value) for value in bbox[:4])
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    if values[2] <= 0.0 or values[3] <= 0.0:
        return None
    return values  # type: ignore[return-value]


def target_size(bbox: tuple[float, float, float, float]) -> float:
    return characteristic_pixels(bbox[2], bbox[3])


def _scale_limits(
    bbox: tuple[float, float, float, float],
    geometry: ZoomGeometry | None,
) -> tuple[float, ...] | None:
    if geometry is None:
        return None
    margin = min(geometry.width, geometry.height) * FRAME_MARGIN_RATIO
    center_x = geometry.width / 2.0
    center_y = geometry.height / 2.0
    cx, cy, width, height = bbox
    rays_and_edges = (
        (cx - center_x - width / 2.0, margin - center_x, -1),
        (cx - center_x + width / 2.0, geometry.width - margin - center_x, 1),
        (cy - center_y - height / 2.0, margin - center_y, -1),
        (cy - center_y + height / 2.0, geometry.height - margin - center_y, 1),
    )
    limits = [edge / ray for ray, edge, sign in rays_and_edges if ray * sign > 0.0]
    return tuple(limits)


def containment_scale(
    bbox: tuple[float, float, float, float],
    geometry: ZoomGeometry | None,
) -> float | None:
    limits = _scale_limits(bbox, geometry)
    return None if limits is None else max(0.0, min((1.0, *limits)))


def reserve_scale(
    bbox: tuple[float, float, float, float],
    geometry: ZoomGeometry | None,
) -> float | None:
    limits = _scale_limits(bbox, geometry)
    return None if limits is None else max(0.0, min(limits))


def zoom_cone(
    size_px: float,
    target_pixels: float,
    geometry: ZoomGeometry | None,
) -> float | None:
    if geometry is None or geometry.fy is None:
        return None
    half_extent = geometry.height * (0.5 - FRAME_MARGIN_RATIO)
    if half_extent <= 0.0:
        return None
    ratio = max(target_pixels / size_px, 1.0)
    return math.atan2(half_extent, geometry.fy) / ratio


def pointing_centered(
    pointing: GimbalTrackResult | None,
    cone: float | None,
) -> bool:
    if pointing is None:
        return True
    if pointing.state is not TrackingState.TRACKING or not pointing.mature:
        return False
    errors = (pointing.yaw_error, pointing.pitch_error)
    if any(value is None or not math.isfinite(value) for value in errors):
        return False
    if cone is None:
        return True
    return all(abs(value) < cone for value in errors if value is not None)


def build_observation(
    bbox: tuple[float, float, float, float],
    target_pixels: float,
    pointing: GimbalTrackResult | None,
    geometry: ZoomGeometry | None,
    hold_on_decenter: bool,
) -> ZoomObservation:
    size_px = target_size(bbox)
    cone = zoom_cone(size_px, target_pixels, geometry)
    return ZoomObservation(
        bbox=bbox,
        size_px=size_px,
        target_pixels=target_pixels,
        containment_scale=containment_scale(bbox, geometry),
        reserve_scale=reserve_scale(bbox, geometry),
        centered=pointing_centered(pointing, cone),
        optical_fresh=(
            pointing is None or pointing.state is not TrackingState.COASTING
        ),
        hold_on_decenter=hold_on_decenter,
    )


__all__ = [
    "FRAME_MARGIN_RATIO",
    "build_observation",
    "containment_scale",
    "extract_bbox",
    "pointing_centered",
    "reserve_scale",
    "target_size",
    "zoom_cone",
]
