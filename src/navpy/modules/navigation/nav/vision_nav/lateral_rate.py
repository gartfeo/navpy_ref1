"""Transactional horizontal visual-rate estimator."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.rate_filter import _low_pass


# Which gyro sample cancels own-yaw motion out of the two-point bearing
# derivative. The bearing difference is the MEAN rate over the frame
# interval — its effective time is the interval midpoint — while every
# gyro sample is INS_GYRO_FILTER-delayed (~11.7 ms measured, 2026-08-27
# A/B, p=0.0007). ``avg``: 0.5*(g[k-1]+g[k]) — effective time = midpoint
# minus the filter delay, so the compensation describes the aircraft
# ~11.7 ms before the motion it cancels, at any frame rate. ``endpoint``:
# the newest sample alone — aligned only near 27 ms frames, WORSE than
# ``avg`` at intervals over 4x the delay. ``blend`` (default): weight the
# two samples to aim at the midpoint for the ACTUAL interval —
# (0.5 - d/T)*g0 + (0.5 + d/T)*g1 — cadence-independent by construction
# (measured: residual mismatch +1.5 ms vs avg's -11.7; above-floor error
# 0.115 -> 0.020 deg/s). Unknown values fail loud, like
# AAS_TRUTH_POSE_TIME_AXIS.
LAT_GYRO_TERM_ENV = "AAS_LAT_GYRO_TERM"
GYRO_TERM_AVG = "avg"
GYRO_TERM_ENDPOINT = "endpoint"
GYRO_TERM_BLEND = "blend"
GYRO_TERMS = (GYRO_TERM_AVG, GYRO_TERM_ENDPOINT, GYRO_TERM_BLEND)

# Filter group delay used by the ``blend`` arm. Default = sqrt(2)/(2*pi*20)
# for the fleet-wide INS_GYRO_FILTER=20 Hz 2-pole Butterworth (verified on
# every real BIN, both airframes, 2025-01..2026-01). Follow-ups replace the
# constant with the runtime INS_GYRO_FILTER param and an online roll/pitch
# estimate; the env override exists so an A/B can probe other values.
LAT_GYRO_DELAY_ENV = "AAS_LAT_GYRO_DELAY_S"
DEFAULT_GYRO_DELAY_S = 0.01125
# Aiming past the newest sample is linear extrapolation; cap it at one full
# sample step so a pathologically short frame interval cannot amplify noise.
MAX_DELAY_TO_INTERVAL_RATIO = 1.0


def resolve_lat_gyro_term() -> str:
    raw = os.environ.get(LAT_GYRO_TERM_ENV, GYRO_TERM_BLEND)
    value = str(raw).strip().lower()
    if value not in GYRO_TERMS:
        raise ValueError(
            f"{LAT_GYRO_TERM_ENV} must be one of {GYRO_TERMS}, got {raw!r}"
        )
    return value


def resolve_lat_gyro_delay_s() -> float:
    raw = os.environ.get(LAT_GYRO_DELAY_ENV, "")
    if not str(raw).strip():
        return DEFAULT_GYRO_DELAY_S
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError(
            f"{LAT_GYRO_DELAY_ENV} must be a number in seconds, got {raw!r}"
        ) from error
    if not 0.0 <= value <= 0.05:
        raise ValueError(
            f"{LAT_GYRO_DELAY_ENV} must be within [0, 0.05] s, got {value}"
        )
    return value


@dataclass(frozen=True)
class LateralRateState:
    continuity_key: tuple[str, int, int, int]
    timestamp_s: float
    bearing_rad: float
    yaw_rate_rad_s: float
    filtered_rate_rad_s: float


@dataclass(frozen=True)
class LateralRatePlan:
    visual_rate_rad_s: float
    raw_inertial_rate_rad_s: float
    rate_rad_s: float
    next_state: LateralRateState


class LateralRateFilter:
    """Estimate measured LOS bearing rate without a wall clock."""

    def __init__(self) -> None:
        self._state: LateralRateState | None = None
        self._gyro_term = resolve_lat_gyro_term()
        self._gyro_delay_s = resolve_lat_gyro_delay_s()

    def reset(self) -> None:
        self._state = None

    def seed(self, frame: TerminalVisionFrame) -> None:
        self._state = LateralRateState(
            frame.continuity_key,
            frame.source_timestamp_s,
            _bearing(frame),
            frame.aircraft_yaw_rate_rad_s,
            0.0,
        )

    def plan(
        self,
        frame: TerminalVisionFrame,
        tau_s: float | None = None,
    ) -> LateralRatePlan:
        bearing = _bearing(frame)
        previous = self._state
        visual_rate = 0.0
        raw_inertial_rate = 0.0
        filtered = 0.0
        if previous is not None and previous.continuity_key == frame.continuity_key:
            dt_s = frame.source_timestamp_s - previous.timestamp_s
            if dt_s <= 0.0:
                return LateralRatePlan(0.0, 0.0, 0.0, previous)
            delta = math.atan2(
                math.sin(bearing - previous.bearing_rad),
                math.cos(bearing - previous.bearing_rad),
            )
            visual_rate = delta / dt_s
            if self._gyro_term == GYRO_TERM_ENDPOINT:
                raw_inertial_rate = visual_rate + frame.aircraft_yaw_rate_rad_s
            elif self._gyro_term == GYRO_TERM_BLEND:
                ratio = min(
                    self._gyro_delay_s / dt_s, MAX_DELAY_TO_INTERVAL_RATIO
                )
                raw_inertial_rate = visual_rate + (
                    (0.5 - ratio) * previous.yaw_rate_rad_s
                    + (0.5 + ratio) * frame.aircraft_yaw_rate_rad_s
                )
            else:
                raw_inertial_rate = visual_rate + 0.5 * (
                    previous.yaw_rate_rad_s + frame.aircraft_yaw_rate_rad_s
                )
            # Mirror the vertical channel: the roll command is proportional to
            # this rate, so the raw two-point bearing derivative fed it whole
            # noise * N*V/g. tau_s None keeps the old raw passthrough; a tau
            # low-passes the SAME quantity the command consumes, exactly as
            # VerticalRateFilter does for pitch.
            filtered = (
                raw_inertial_rate
                if not tau_s
                else _low_pass(
                    raw_inertial_rate, previous.filtered_rate_rad_s, dt_s, tau_s
                )
            )
        return LateralRatePlan(
            visual_rate,
            raw_inertial_rate,
            filtered,
            LateralRateState(
                frame.continuity_key,
                frame.source_timestamp_s,
                bearing,
                frame.aircraft_yaw_rate_rad_s,
                filtered,
            ),
        )

    def commit(self, plan: LateralRatePlan) -> None:
        self._state = plan.next_state


def _bearing(frame: TerminalVisionFrame) -> float:
    return math.atan2(frame.control_y, frame.control_x)


__all__ = [
    "LateralRateFilter",
    "LateralRatePlan",
    "LateralRateState",
]
