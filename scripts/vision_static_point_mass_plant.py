"""Point-mass plant integration and miss geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.utils.math_utils import GRAVITY_MSS
from scripts.vision_static_point_mass_types import (
    StaticPointMassCase,
    StaticPointMassMiss,
)


@dataclass(frozen=True)
class PointMassState:
    position_ned_m: np.ndarray
    pitch_deg: float
    roll_deg: float
    yaw_deg: float
    t_s: float


@dataclass(frozen=True)
class PointMassStep:
    state: PointMassState
    velocity_ned_mps: np.ndarray
    segment_start_ned_m: np.ndarray
    segment_ned_m: np.ndarray


def initial_state(case: StaticPointMassCase) -> PointMassState:
    return PointMassState(
        np.asarray(case.start_ned_m, dtype=float),
        float(case.pitch_deg),
        float(case.roll_deg),
        float(case.yaw_deg),
        0.0,
    )


def wind_ned_mps(case: StaticPointMassCase) -> np.ndarray:
    if case.wind_speed_mps <= 0.0:
        return np.zeros(3, dtype=float)
    wind_to_rad = math.radians((case.wind_dir_from_deg + 180.0) % 360.0)
    return np.asarray(
        [
            case.wind_speed_mps * math.cos(wind_to_rad),
            case.wind_speed_mps * math.sin(wind_to_rad),
            0.0,
        ],
        dtype=float,
    )


def integrate_plant(
    state: PointMassState,
    case: StaticPointMassCase,
    target_pitch_deg: float,
    target_roll_deg: float,
    wind: np.ndarray,
) -> PointMassStep:
    alpha = (
        min(1.0, case.dt_s / case.attitude_time_constant_s)
        if case.attitude_time_constant_s > 0.0
        else 1.0
    )
    pitch_deg = state.pitch_deg + (target_pitch_deg - state.pitch_deg) * alpha
    roll_deg = state.roll_deg + (target_roll_deg - state.roll_deg) * alpha
    air_velocity = air_velocity_ned_mps(
        case.airspeed_mps,
        state.yaw_deg,
        pitch_deg + case.flight_path_pitch_offset_deg,
    )
    yaw_deg = state.yaw_deg + coordinated_turn_rate_deg_s(
        air_velocity,
        roll_deg,
    ) * case.dt_s
    velocity = air_velocity + wind
    previous_position = state.position_ned_m.copy()
    position = previous_position + velocity * case.dt_s
    return PointMassStep(
        PointMassState(
            position,
            pitch_deg,
            roll_deg,
            yaw_deg,
            state.t_s + case.dt_s,
        ),
        velocity,
        previous_position,
        position - previous_position,
    )


def air_velocity_ned_mps(
    airspeed_mps: float,
    yaw_deg: float,
    pitch_deg: float,
) -> np.ndarray:
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    horizontal_mps = airspeed_mps * math.cos(pitch_rad)
    return np.asarray(
        [
            horizontal_mps * math.cos(yaw_rad),
            horizontal_mps * math.sin(yaw_rad),
            airspeed_mps * math.sin(-pitch_rad),
        ],
        dtype=float,
    )


def coordinated_turn_rate_deg_s(
    velocity_ned_mps: np.ndarray,
    roll_deg: float,
) -> float:
    horizontal_mps = max(float(np.linalg.norm(velocity_ned_mps[:2])), 1e-6)
    return math.degrees(
        GRAVITY_MSS * math.tan(math.radians(roll_deg)) / horizontal_mps
    )


def closest_point_to_target(
    segment_start_ned_m: np.ndarray,
    segment_ned_m: np.ndarray,
) -> np.ndarray:
    segment_len2 = float(np.dot(segment_ned_m, segment_ned_m))
    if segment_len2 <= 0.0:
        return segment_start_ned_m
    fraction = float(
        np.clip(
            -float(np.dot(segment_start_ned_m, segment_ned_m)) / segment_len2,
            0.0,
            1.0,
        )
    )
    return segment_start_ned_m + fraction * segment_ned_m


def component_miss(
    closest_ned_m: np.ndarray,
    segment_ned_m: np.ndarray,
    t_s: float,
) -> StaticPointMassMiss:
    track_horizontal = np.asarray(segment_ned_m[:2], dtype=float)
    track_norm = float(np.linalg.norm(track_horizontal))
    track_unit = (
        np.asarray([1.0, 0.0], dtype=float)
        if track_norm <= 0.0
        else track_horizontal / track_norm
    )
    lateral_unit = np.asarray([-track_unit[1], track_unit[0]], dtype=float)
    target_from_closest = -closest_ned_m
    return StaticPointMassMiss(
        lateral_m=abs(float(np.dot(target_from_closest[:2], lateral_unit))),
        longitudinal_m=abs(float(np.dot(target_from_closest[:2], track_unit))),
        vertical_m=abs(float(target_from_closest[2])),
        slant_m=float(np.linalg.norm(target_from_closest)),
        t_s=t_s,
    )


def has_passed_target(step: PointMassStep, dt_s: float) -> bool:
    return (
        step.state.t_s > dt_s
        and float(
            np.dot(-step.state.position_ned_m, step.velocity_ned_mps)
        ) < 0.0
    )


def is_approaching_target(step: PointMassStep) -> bool:
    return float(
        np.dot(-step.state.position_ned_m, step.velocity_ned_mps)
    ) > 0.0


__all__ = [
    "PointMassState",
    "PointMassStep",
    "air_velocity_ned_mps",
    "closest_point_to_target",
    "component_miss",
    "coordinated_turn_rate_deg_s",
    "has_passed_target",
    "initial_state",
    "integrate_plant",
    "is_approaching_target",
    "wind_ned_mps",
]
