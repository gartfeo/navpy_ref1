"""Pixel, anchor, zoom, and calibrated-intrinsics geometry for gimbal tuning."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from .gimbal_tuning_args import MAX_ZOOM, MIN_ZOOM


ANCHOR_NAMES = {
    1: "BL", 2: "BC", 3: "BR",
    4: "ML", 5: "C", 6: "MR",
    7: "TL", 8: "TC", 9: "TR",
}


@dataclass(frozen=True)
class TrackingIntrinsics:
    zoom: float
    fx: float
    fy: float
    cx: float
    cy: float
    k: np.ndarray
    dist_coeffs: np.ndarray
    source: str
    reference_zoom: float


@dataclass(frozen=True)
class TrackingCommand:
    bbox_center: tuple[float, float]
    bbox_size: tuple[float, float]
    anchor_pixel: tuple[float, float]
    undistorted_bbox_center: tuple[float, float]
    undistorted_anchor: tuple[float, float]
    delta_yaw_deg: float
    delta_pitch_deg: float
    tracking_k: np.ndarray


def pixel_to_delta(
    px: float,
    py: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float]:
    return (
        math.degrees(math.atan2(px - cx, fx)),
        math.degrees(math.atan2(py - cy, fy)),
    )


def undistort_point(
    px: float,
    py: float,
    k: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[float, float]:
    points = np.asarray([[[float(px), float(py)]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(points, k, dist_coeffs, P=k)
    return (
        float(undistorted[0, 0, 0]),
        float(undistorted[0, 0, 1]),
    )


def normalize_bbox(
    bbox_cxcywh: Any,
) -> tuple[float, float, float, float] | None:
    if bbox_cxcywh is None:
        return None
    try:
        if len(bbox_cxcywh) < 4:
            return None
        bbox = tuple(float(bbox_cxcywh[index]) for index in range(4))
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in bbox):
        return None
    if bbox[2] <= 0.0 or bbox[3] <= 0.0:
        return None
    return bbox


def bbox_center(bbox_cxcywh: Any) -> tuple[float, float] | None:
    bbox = normalize_bbox(bbox_cxcywh)
    return None if bbox is None else bbox[:2]


def anchor_pixel(
    mode: int,
    image_width: int,
    image_height: int,
    center_x: float,
    center_y: float,
) -> tuple[float, float]:
    return {
        1: (0.0, float(image_height)),
        2: (image_width / 2.0, float(image_height)),
        3: (float(image_width), float(image_height)),
        4: (0.0, image_height / 2.0),
        5: (center_x, center_y),
        6: (float(image_width), image_height / 2.0),
        7: (0.0, 0.0),
        8: (image_width / 2.0, 0.0),
        9: (float(image_width), 0.0),
    }[mode]


def resolve_tracking_intrinsics(
    calibrated_intrinsics: dict[float, TrackingIntrinsics],
    zoom_level: float,
) -> TrackingIntrinsics:
    zoom = float(zoom_level)
    if zoom in calibrated_intrinsics:
        return calibrated_intrinsics[zoom]
    if not calibrated_intrinsics:
        raise ValueError("No calibrated intrinsics available")
    reference_zoom = min(
        calibrated_intrinsics,
        key=lambda value: abs(value - zoom),
    )
    reference = calibrated_intrinsics[reference_zoom]
    k = reference.k.copy()
    scale = zoom / reference_zoom
    k[0, 0] *= scale
    k[1, 1] *= scale
    return TrackingIntrinsics(
        zoom=zoom,
        fx=float(k[0, 0]),
        fy=float(k[1, 1]),
        cx=float(k[0, 2]),
        cy=float(k[1, 2]),
        k=k,
        dist_coeffs=reference.dist_coeffs.copy(),
        source="estimated",
        reference_zoom=reference_zoom,
    )


def build_bbox_tracking_command(
    bbox_cxcywh: Any,
    intrinsics: TrackingIntrinsics,
    anchor_mode: int,
    image_width: int,
    image_height: int,
) -> TrackingCommand | None:
    bbox = normalize_bbox(bbox_cxcywh)
    if bbox is None:
        return None
    bbox_x, bbox_y, bbox_width, bbox_height = bbox
    anchor = anchor_pixel(
        anchor_mode,
        image_width,
        image_height,
        intrinsics.cx,
        intrinsics.cy,
    )
    undistorted_bbox = undistort_point(
        bbox_x,
        bbox_y,
        intrinsics.k,
        intrinsics.dist_coeffs,
    )
    undistorted_anchor = undistort_point(
        anchor[0],
        anchor[1],
        intrinsics.k,
        intrinsics.dist_coeffs,
    )
    bbox_delta = pixel_to_delta(
        *undistorted_bbox,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.cx,
        intrinsics.cy,
    )
    anchor_delta = pixel_to_delta(
        *undistorted_anchor,
        intrinsics.fx,
        intrinsics.fy,
        intrinsics.cx,
        intrinsics.cy,
    )
    tracking_k = intrinsics.k.copy()
    tracking_k[0, 2], tracking_k[1, 2] = undistorted_anchor
    return TrackingCommand(
        (bbox_x, bbox_y),
        (bbox_width, bbox_height),
        anchor,
        undistorted_bbox,
        undistorted_anchor,
        bbox_delta[0] - anchor_delta[0],
        bbox_delta[1] - anchor_delta[1],
        tracking_k,
    )


def format_zoom(zoom_level: float) -> str:
    zoom = float(zoom_level)
    return str(int(zoom)) if zoom.is_integer() else f"{zoom:g}"


def zoom_sequence(
    calibrated_intrinsics: dict[float, TrackingIntrinsics],
) -> list[float]:
    levels = sorted(calibrated_intrinsics)
    if len(levels) > 1:
        return levels
    return [float(level) for level in range(int(MIN_ZOOM), int(MAX_ZOOM) + 1)]


def step_zoom(current_zoom: float, sequence: list[float], direction: int) -> float:
    if not sequence:
        return current_zoom
    ordered = sorted(sequence)
    if direction > 0:
        return next(
            (level for level in ordered if level > current_zoom),
            ordered[-1],
        )
    return next(
        (level for level in reversed(ordered) if level < current_zoom),
        ordered[0],
    )


__all__ = [
    "ANCHOR_NAMES",
    "TrackingCommand",
    "TrackingIntrinsics",
    "anchor_pixel",
    "bbox_center",
    "build_bbox_tracking_command",
    "format_zoom",
    "normalize_bbox",
    "pixel_to_delta",
    "resolve_tracking_intrinsics",
    "step_zoom",
    "undistort_point",
    "zoom_sequence",
]
