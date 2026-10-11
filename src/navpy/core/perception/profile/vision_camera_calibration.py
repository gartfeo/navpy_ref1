"""Narrow immutable read model for calibrated camera zoom intrinsics."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np


def _finite_number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number) or (positive and number <= 0.0):
        qualifier = "positive finite" if positive else "finite"
        raise ValueError(f"{name} must be a {qualifier} number")
    return number


@dataclass(frozen=True)
class CameraZoomCalibration:
    """Immutable optical calibration for one commanded zoom level."""

    zoom: str
    fx: float
    fy: float
    cx: float
    cy: float
    skew: float = 0.0

    def intrinsic_matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, self.skew, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )


def parse_zoom_calibration(
    zoom_map: Mapping[object, object],
) -> tuple[CameraZoomCalibration, ...]:
    """Validate and freeze a JSON-shaped zoom calibration mapping."""
    if not isinstance(zoom_map, Mapping) or not zoom_map:
        raise ValueError("camera.intrinsics.zooms must be a non-empty mapping")

    entries: list[CameraZoomCalibration] = []
    seen_zooms: set[float] = set()
    for raw_zoom, raw_entry in zoom_map.items():
        zoom_value = _finite_number(raw_zoom, "camera zoom", positive=True)
        if zoom_value in seen_zooms:
            raise ValueError(f"duplicate numeric camera zoom {zoom_value}")
        seen_zooms.add(zoom_value)
        if not isinstance(raw_entry, Mapping):
            raise ValueError(f"camera zoom {raw_zoom!r} must be a mapping")
        cx = _finite_number(raw_entry.get("cx"), f"zoom {raw_zoom} cx")
        cy = _finite_number(raw_entry.get("cy"), f"zoom {raw_zoom} cy")
        if cx < 0.0 or cy < 0.0:
            raise ValueError(f"zoom {raw_zoom} cx/cy must be non-negative")
        entries.append(
            CameraZoomCalibration(
                zoom=str(raw_zoom),
                fx=_finite_number(
                    raw_entry.get("fx"),
                    f"zoom {raw_zoom} fx",
                    positive=True,
                ),
                fy=_finite_number(
                    raw_entry.get("fy"),
                    f"zoom {raw_zoom} fy",
                    positive=True,
                ),
                cx=cx,
                cy=cy,
                skew=_finite_number(
                    raw_entry.get(
                        "skew",
                        raw_entry.get("alpha", raw_entry.get("alpha_c", 0.0)),
                    ),
                    f"zoom {raw_zoom} skew",
                ),
            )
        )
    entries.sort(key=lambda entry: float(entry.zoom))
    return tuple(entries)


def read_camera_zoom_calibration(camera: object) -> tuple[CameraZoomCalibration, ...]:
    """Read a camera's private storage once and expose immutable values.

    This adapter is the sole compatibility boundary around the legacy
    ``CameraIntrinsics`` storage layout. Navigation consumes only this public,
    immutable read model.
    """
    raw_zoom_map = getattr(camera, "_zoom_map", None)
    if not isinstance(raw_zoom_map, Mapping) or not raw_zoom_map:
        return ()
    entries: list[CameraZoomCalibration] = []
    for zoom, raw_entry in raw_zoom_map.items():
        try:
            entries.extend(parse_zoom_calibration({zoom: raw_entry}))
        except ValueError:
            # Legacy runtime tables historically skipped malformed candidates;
            # strict profile validation remains in parse_zoom_calibration().
            continue
    entries.sort(key=lambda entry: float(entry.zoom))
    return tuple(entries)


__all__ = [
    "CameraZoomCalibration",
    "parse_zoom_calibration",
    "read_camera_zoom_calibration",
]
