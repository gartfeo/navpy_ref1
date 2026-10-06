"""Focused immutable limit groups for three-UAV demo certification."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from scripts.eval_gcs_demo_constants import SIM_SPEEDUP


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return number


def _non_negative_int(name: str, value: object) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    if value < 0:
        raise ValueError(f"{name} must be non-negative")
    return value


@dataclass(frozen=True)
class CadenceLimits:
    """Final-approach-event count, wall cadence, and clock-ratio gates."""

    min_observations: int = 10
    max_median_wall_gap_s: float = 0.20
    max_wall_gap_s: float = 0.50
    min_speedup_ratio: float = 0.80
    max_speedup_ratio: float = 1.20

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "min_observations",
            _non_negative_int("min_observations", self.min_observations),
        )
        for name in (
            "max_median_wall_gap_s",
            "max_wall_gap_s",
            "min_speedup_ratio",
            "max_speedup_ratio",
        ):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))
        if self.min_speedup_ratio > self.max_speedup_ratio:
            raise ValueError("minimum observed speedup ratio exceeds maximum")

    def speedup_bounds(self, expected_speedup: float) -> tuple[float, float]:
        expected = _finite("expected speedup", expected_speedup)
        if expected <= 0.0:
            raise ValueError("expected speedup must be positive")
        return (
            expected * self.min_speedup_ratio,
            expected * self.max_speedup_ratio,
        )


@dataclass(frozen=True)
class RollQualityLimits:
    """Roll reversal, saturation, and command-step quality gates."""

    significant_roll_deg: float = 10.0
    max_significant_reversals: int = 1
    saturation_margin_deg: float = 0.5
    max_saturation_fraction: float = 0.20
    max_saturation_run: int = 3
    max_roll_step_deg: float = 50.0

    def __post_init__(self) -> None:
        for name in ("max_significant_reversals", "max_saturation_run"):
            object.__setattr__(
                self,
                name,
                _non_negative_int(name, getattr(self, name)),
            )
        for name in (
            "significant_roll_deg",
            "saturation_margin_deg",
            "max_saturation_fraction",
            "max_roll_step_deg",
        ):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))
        if self.max_saturation_fraction > 1.0:
            raise ValueError("max_saturation_fraction must be <= 1")


@dataclass(frozen=True)
class TruthLimits:
    """Simulation-truth scoring limits kept outside the command path."""

    max_snap_distance_m: float = 0.5

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_snap_distance_m",
            _finite("max_snap_distance_m", self.max_snap_distance_m),
        )


@dataclass(frozen=True)
class GateLimits:
    """Read-only view composed from focused certification policies."""

    cadence: CadenceLimits = field(default_factory=CadenceLimits)
    roll_quality: RollQualityLimits = field(default_factory=RollQualityLimits)
    truth: TruthLimits = field(default_factory=TruthLimits)

    @property
    def min_observations(self) -> int:
        return self.cadence.min_observations

    @property
    def max_median_wall_gap_s(self) -> float:
        return self.cadence.max_median_wall_gap_s

    @property
    def max_wall_gap_s(self) -> float:
        return self.cadence.max_wall_gap_s

    @property
    def min_observed_speedup(self) -> float:
        return self.cadence.min_speedup_ratio * SIM_SPEEDUP

    @property
    def max_observed_speedup(self) -> float:
        return self.cadence.max_speedup_ratio * SIM_SPEEDUP

    @property
    def significant_roll_deg(self) -> float:
        return self.roll_quality.significant_roll_deg

    @property
    def max_significant_reversals(self) -> int:
        return self.roll_quality.max_significant_reversals

    @property
    def saturation_margin_deg(self) -> float:
        return self.roll_quality.saturation_margin_deg

    @property
    def max_saturation_fraction(self) -> float:
        return self.roll_quality.max_saturation_fraction

    @property
    def max_saturation_run(self) -> int:
        return self.roll_quality.max_saturation_run

    @property
    def max_roll_step_deg(self) -> float:
        return self.roll_quality.max_roll_step_deg

    @property
    def max_snap_distance_m(self) -> float:
        return self.truth.max_snap_distance_m

    def speedup_bounds(self, expected_speedup: float) -> tuple[float, float]:
        return self.cadence.speedup_bounds(expected_speedup)


__all__ = [
    "CadenceLimits",
    "GateLimits",
    "RollQualityLimits",
    "TruthLimits",
]
