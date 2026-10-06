"""Deterministic static point-mass scenario runner."""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Iterable, Protocol

import numpy as np

from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    TerminalFrameProjector,
    TerminalProjectionConfig,
)
from navpy.modules.navigation.nav.vision_nav.law import (
    FixedTerminalLawConfigProvider,
    TerminalLawConfig,
    VisionNavLaw,
)
from navpy.modules.vision.models.pixel_observation import VisualDetection
from scripts.vision_static_point_mass_plant import (
    air_velocity_ned_mps,
    closest_point_to_target,
    component_miss,
    has_passed_target,
    initial_state,
    integrate_plant,
    is_approaching_target,
    wind_ned_mps,
    coordinated_turn_rate_deg_s,
)
from scripts.vision_static_point_mass_sensor import (
    build_static_detection,
    undelayed_truth_gyro,
)
from scripts.vision_static_point_mass_types import (
    StaticPointMassCase,
    StaticPointMassMiss,
    StaticPointMassRun,
)


class StaticDetectionFactory(Protocol):
    def __call__(
        self,
        *,
        position_ned_m: np.ndarray,
        t_s: float,
        pitch_deg: float,
        roll_deg: float,
        yaw_deg: float,
        bearing_noise_rad: float = 0.0,
        aircraft_yaw_rate_rad_s: float = 0.0,
    ) -> VisualDetection: ...


_TERMINAL_PROJECTOR = TerminalFrameProjector(
    TerminalProjectionConfig("ZYX", True),
)


def run_case(case: StaticPointMassCase) -> StaticPointMassMiss:
    return run_case_analysis(case).miss


def run_case_analysis(
    case: StaticPointMassCase,
    *,
    detection_factory: StaticDetectionFactory = build_static_detection,
) -> StaticPointMassRun:
    with undelayed_truth_gyro():
        law = VisionNavLaw(
            FixedTerminalLawConfigProvider(
                TerminalLawConfig(
                    pitch_min_deg=-55.0,
                    pitch_max_deg=30.0,
                    roll_limit_deg=case.roll_limit_deg,
                    pitch_time_constant_s=case.attitude_time_constant_s,
                    throttle=None,
                )
            )
        )
    state = initial_state(case)
    wind = wind_ned_mps(case)
    next_command_t_s = 0.0
    target_pitch_deg = state.pitch_deg
    target_roll_deg = state.roll_deg
    best_miss = StaticPointMassMiss(
        math.inf,
        math.inf,
        math.inf,
        math.inf,
        0.0,
    )
    command_timestamps: list[float] = []
    approach_observed = False
    noise_rng = (
        np.random.default_rng(case.noise_seed)
        if case.bearing_noise_deg > 0.0
        else None
    )

    for _ in range(int(case.max_t_s / case.dt_s)):
        if state.t_s + 1e-12 >= next_command_t_s:
            noise_rad = _bearing_noise_rad(case, noise_rng)
            detection = detection_factory(
                position_ned_m=state.position_ned_m,
                t_s=state.t_s,
                pitch_deg=state.pitch_deg,
                roll_deg=state.roll_deg,
                yaw_deg=state.yaw_deg,
                bearing_noise_rad=noise_rad,
                aircraft_yaw_rate_rad_s=math.radians(
                    coordinated_turn_rate_deg_s(
                        air_velocity_ned_mps(
                            case.airspeed_mps,
                            state.yaw_deg,
                            state.pitch_deg + case.flight_path_pitch_offset_deg,
                        ),
                        state.roll_deg,
                    )
                ),
            )
            if not isinstance(detection, VisualDetection):
                raise TypeError("point-mass sensor must publish VisualDetection")
            observation = _TERMINAL_PROJECTOR.project(
                detection,
                source_generation=0,
                air_speed_mps=case.airspeed_mps,
            )
            if observation is None:
                raise ValueError("static visual detection was rejected")
            command_timestamps.append(observation.source_timestamp_s)
            plan = law.plan(observation)
            target_pitch_deg = plan.command.cmd_pitch_deg
            target_roll_deg = plan.command.cmd_roll_deg
            law.commit(plan)
            next_command_t_s += _command_interval(case)

        step = integrate_plant(
            state,
            case,
            target_pitch_deg,
            target_roll_deg,
            wind,
        )
        state = step.state
        closest = closest_point_to_target(
            step.segment_start_ned_m,
            step.segment_ned_m,
        )
        miss = component_miss(closest, step.segment_ned_m, state.t_s)
        if miss.slant_m < best_miss.slant_m:
            best_miss = miss
        approach_observed = approach_observed or is_approaching_target(step)
        if approach_observed and has_passed_target(step, case.dt_s):
            return StaticPointMassRun(
                replace(best_miss, passed_target=True),
                tuple(command_timestamps),
            )

    return StaticPointMassRun(
        replace(best_miss, timed_out=True),
        tuple(command_timestamps),
    )


def default_cases(
    wind_speeds: Iterable[float],
    wind_dirs: Iterable[float],
    *,
    dt_s: float | None = None,
    command_interval_s: float | None = None,
    max_t_s: float | None = None,
    bearing_noise_deg: float | None = None,
    noise_seed: int | None = None,
) -> list[StaticPointMassCase]:
    defaults = StaticPointMassCase("defaults", 0.0, 0.0)
    return [
        StaticPointMassCase(
            name=f"w{wind_speed:g}-d{wind_dir:g}",
            wind_speed_mps=float(wind_speed),
            wind_dir_from_deg=float(wind_dir),
            dt_s=(
                float(dt_s) if dt_s is not None else defaults.dt_s
            ),
            command_interval_s=(
                float(command_interval_s)
                if command_interval_s is not None
                else defaults.command_interval_s
            ),
            max_t_s=(
                float(max_t_s)
                if max_t_s is not None
                else defaults.max_t_s
            ),
            bearing_noise_deg=(
                float(bearing_noise_deg)
                if bearing_noise_deg is not None
                else defaults.bearing_noise_deg
            ),
            noise_seed=(
                int(noise_seed)
                if noise_seed is not None
                else defaults.noise_seed
            ),
        )
        for wind_speed in wind_speeds
        for wind_dir in wind_dirs
    ]


def _command_interval(case: StaticPointMassCase) -> float:
    configured = (
        case.command_interval_s
        if case.command_interval_s > 0.0
        else case.dt_s
    )
    return max(case.dt_s, configured)


def _bearing_noise_rad(
    case: StaticPointMassCase,
    noise_rng: np.random.Generator | None,
) -> float:
    if noise_rng is None or case.bearing_noise_deg <= 0.0:
        return 0.0
    return math.radians(case.bearing_noise_deg) * float(
        noise_rng.standard_normal()
    )


__all__ = ["default_cases", "run_case", "run_case_analysis"]
