"""Immutable scenario and result types for the static point-mass diagnostic."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class StaticPointMassCase:
    name: str
    wind_dir_from_deg: float
    wind_speed_mps: float
    start_ned_m: tuple[float, float, float] = (-1650.0, 0.0, -140.0)
    yaw_deg: float = 0.0
    pitch_deg: float = -1.5
    roll_deg: float = 0.0
    airspeed_mps: float = 25.0
    roll_limit_deg: float = 60.0
    attitude_time_constant_s: float = 0.45
    dt_s: float = 0.02
    command_interval_s: float = 0.1
    max_t_s: float = 90.0
    bearing_noise_deg: float = 0.0
    noise_seed: int = 1
    flight_path_pitch_offset_deg: float = 0.0


@dataclass(frozen=True)
class StaticPointMassMiss:
    lateral_m: float
    longitudinal_m: float
    vertical_m: float
    slant_m: float
    t_s: float
    passed_target: bool = False
    timed_out: bool = False


@dataclass(frozen=True)
class StaticPointMassRun:
    miss: StaticPointMassMiss
    command_timestamps_s: tuple[float, ...]


__all__ = [
    "StaticPointMassCase",
    "StaticPointMassMiss",
    "StaticPointMassRun",
]
