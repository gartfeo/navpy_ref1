"""Immutable pixels and transforms allowed across the navigation boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np


class PixelProjectionKind(Enum):
    PINHOLE = "pinhole"
    SPHERICAL_EQUIANGULAR = "spherical_equiangular"


@dataclass(frozen=True)
class PixelCalibration:
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pixel calibration must be finite")
        if self.fx_px <= 0.0 or self.fy_px <= 0.0:
            raise ValueError("pixel focal lengths must be positive")

    @classmethod
    def from_matrix(cls, value: object) -> "PixelCalibration":
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("camera matrix must be finite 3x3")
        return cls(
            float(matrix[0, 0]),
            float(matrix[1, 1]),
            float(matrix[0, 2]),
            float(matrix[1, 2]),
        )

    def matrix(self) -> np.ndarray:
        return np.asarray(
            [
                [self.fx_px, 0.0, self.cx_px],
                [0.0, self.fy_px, self.cy_px],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )


@dataclass(frozen=True)
class CameraToBodyTransform:
    elements: tuple[float, float, float, float, float, float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.elements) != 9 or not all(
            math.isfinite(float(value)) for value in self.elements
        ):
            raise ValueError("camera-to-body transform must be a finite 3x3 matrix")

    @classmethod
    def from_matrix(cls, value: Sequence[Sequence[float]]) -> "CameraToBodyTransform":
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (3, 3):
            raise ValueError("camera-to-body transform must be 3x3")
        return cls(tuple(float(value) for value in matrix.reshape(9)))

    def matrix(self) -> np.ndarray:
        return np.asarray(self.elements, dtype=float).reshape(3, 3)


@dataclass(frozen=True)
class PixelObservation:
    u_px: float
    v_px: float
    calibration: PixelCalibration
    camera_to_body: CameraToBodyTransform
    projection: PixelProjectionKind
    aircraft_pitch_deg: float
    aircraft_roll_deg: float
    source_timestamp_s: float | None
    pose_is_frame_atomic: bool | None
    source_name: str
    aircraft_yaw_rate_rad_s: float | None = None


@dataclass(frozen=True)
class VisualDetection:
    task_id: int
    obj_id: int
    observation: PixelObservation


__all__ = [
    "CameraToBodyTransform",
    "PixelCalibration",
    "PixelObservation",
    "PixelProjectionKind",
    "VisualDetection",
    "aircraft_yaw_rate_rad_s",
]


def aircraft_yaw_rate_rad_s(
    pitch_deg: float,
    roll_deg: float,
    body_rates_rad_s: Sequence[float] | None,
) -> float | None:
    """Convert body q/r gyros to yaw rate without compass heading."""
    if body_rates_rad_s is None:
        return None
    try:
        pitch = math.radians(float(pitch_deg))
        roll = math.radians(float(roll_deg))
        q_rad_s = float(body_rates_rad_s[1])
        r_rad_s = float(body_rates_rad_s[2])
        denominator = math.cos(pitch)
        rate = (q_rad_s * math.sin(roll) + r_rad_s * math.cos(roll)) / denominator
    except (IndexError, TypeError, ValueError, ZeroDivisionError):
        return None
    return rate if math.isfinite(rate) and abs(denominator) > 1e-6 else None
