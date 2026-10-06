"""Immutable snapshots consumed by legacy Roll-L1 navigation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RollL1MotionSnapshot:
    yaw_deg: float
    heading_deg: float
    velocity_ne_mps: tuple[float, float]


@dataclass(frozen=True)
class RollL1AttitudeSnapshot:
    pitch_deg: float
    roll_deg: float


@dataclass(frozen=True)
class RollL1AirframeLimits:
    pitch_min_deg: float
    pitch_max_deg: float
    trim_throttle_percent: float


@dataclass(frozen=True)
class PitchLockSettings:
    termination_angle_deg: float
    lock_distance_m: float
    roll_difference_deg: float


__all__ = [
    "PitchLockSettings",
    "RollL1AirframeLimits",
    "RollL1AttitudeSnapshot",
    "RollL1MotionSnapshot",
]
