"""Immutable configuration and results for gimbal rate tracking."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from navpy.modules.vision.poi_angle_estimator import (
    PoiAngleEstimatorConfig,
)


class TrackingState(Enum):
    IDLE = "idle"
    TRACKING = "tracking"
    COASTING = "coasting"
    HOLDING = "holding"


class GimbalObservationDisposition(Enum):
    """How one visual observation affected navigation state."""

    ACCEPTED = "accepted"
    STALE_NOOP = "stale_noop"
    NO_OBSERVATION = "no_observation"


@dataclass(frozen=True)
class GimbalRateTrackerConfig:
    correction_bw: float = 2.5
    max_rate: float = 100.0
    max_slew_dps: float = 90.0
    settle_tolerance_deg: float = 0.3
    command_lead_time: float = 0.15
    estimator: PoiAngleEstimatorConfig = field(
        default_factory=PoiAngleEstimatorConfig
    )
    rate_clamp_slew_multiple: float = 3.0

    def __post_init__(self) -> None:
        _require_nonnegative("correction_bw", self.correction_bw)
        _require_nonnegative("max_rate", self.max_rate)
        _require_nonnegative(
            "settle_tolerance_deg",
            self.settle_tolerance_deg,
        )
        _require_nonnegative("command_lead_time", self.command_lead_time)
        if not math.isfinite(self.max_slew_dps) or self.max_slew_dps <= 0.0:
            raise ValueError("max_slew_dps must be finite and > 0")
        if (
            not math.isfinite(self.rate_clamp_slew_multiple)
            or self.rate_clamp_slew_multiple <= 0.0
        ):
            raise ValueError(
                "rate_clamp_slew_multiple must be finite and > 0"
            )


def _require_nonnegative(name: str, value: float) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and >= 0")


@dataclass(frozen=True)
class GimbalTrackResult:
    state: TrackingState
    has_poi: bool
    yaw_error: float | None = None
    pitch_error: float | None = None
    yaw_rate: float | None = None
    pitch_rate: float | None = None
    yaw_rate_estimate: float | None = None
    pitch_rate_estimate: float | None = None
    mature: bool = False


@dataclass(frozen=True)
class GimbalRateUpdate:
    result: GimbalTrackResult
    disposition: GimbalObservationDisposition


def idle_result() -> GimbalTrackResult:
    return GimbalTrackResult(TrackingState.IDLE, False)


__all__ = [
    "GimbalRateTrackerConfig",
    "GimbalRateUpdate",
    "GimbalTrackResult",
    "GimbalObservationDisposition",
    "TrackingState",
    "idle_result",
]
