"""Frame-associated camera state for the gimbal feedforward (HITL step 24).

Only camera state and the aircraft's gyro turn rate: the calibrated
camera-to-body transform at capture (mount and gimbal readback), aircraft
pitch/roll, and the turn rate from body q/r gyros (owner ruling 2026-11-06:
gyro allowed for gimbal pointing only). Never compass heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.core.ports.pixel_observation import (
    PixelObservation,
    PixelProjectionKind,
)


@dataclass(frozen=True)
class CameraMotionState:
    """One frame's camera pose relative to the aircraft, plus gyro turn rate."""

    camera_to_body: tuple[float, ...]
    aircraft_pitch_deg: float
    aircraft_roll_deg: float
    aircraft_turn_rate_rad_s: float

    def __post_init__(self) -> None:
        values = (
            *self.camera_to_body,
            self.aircraft_pitch_deg,
            self.aircraft_roll_deg,
            self.aircraft_turn_rate_rad_s,
        )
        if len(self.camera_to_body) != 9 or not all(
            math.isfinite(value) for value in values
        ):
            raise ValueError("camera motion state must be finite")

    def camera_to_body_matrix(self) -> np.ndarray:
        return np.asarray(self.camera_to_body, dtype=float).reshape(3, 3)


def camera_motion_from_observation(
    observation: PixelObservation,
) -> CameraMotionState | None:
    """Camera state of one frame-atomic pinhole observation, else None."""
    if (
        observation.pose_is_frame_atomic is not True
        or observation.projection is not PixelProjectionKind.PINHOLE
        or observation.aircraft_yaw_rate_rad_s is None
    ):
        return None
    try:
        return CameraMotionState(
            tuple(float(value) for value in observation.camera_to_body.elements),
            float(observation.aircraft_pitch_deg),
            float(observation.aircraft_roll_deg),
            float(observation.aircraft_yaw_rate_rad_s),
        )
    except (AttributeError, TypeError, ValueError):
        return None


__all__ = ["CameraMotionState", "camera_motion_from_observation"]
