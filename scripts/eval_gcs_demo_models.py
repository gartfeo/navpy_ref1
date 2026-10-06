"""Validated data contracts for the exact three-UAV demo evaluator."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field

from scripts.eval_gcs_demo_ports import JsonApiPort, JsonValue, TailPort
from scripts.eval_gcs_demo_limits import (
    CadenceLimits,
    GateLimits,
    RollQualityLimits,
    TruthLimits,
)


class RegressionError(RuntimeError):
    """A deterministic setup, workflow, or evidence gate failed."""


def finite_number(name: str, value: object, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or number < minimum:
        raise ValueError(f"{name} must be finite and >= {minimum}")
    return number


def positive_int(name: str, value: object) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


@dataclass(frozen=True)
class ThreeUavIds:
    values: tuple[int, int, int]

    @classmethod
    def from_values(cls, values: Iterable[object]) -> ThreeUavIds:
        items = tuple(values)
        if len(items) != 3:
            raise ValueError(f"exactly three UAV ids are required, got {items!r}")
        checked = tuple(positive_int("sys_id", value) for value in items)
        if len(set(checked)) != 3:
            raise ValueError(f"UAV ids must be unique, got {checked!r}")
        return cls(checked)  # type: ignore[arg-type]


@dataclass(frozen=True)
class FinalApproachCommand:
    """Immutable passive copy of one atomic TERMINAL_CMD payload."""

    wall_s: float
    source: str
    generation: int
    task: int
    obj: int
    obs_ts: float
    dt_wall_ms: float | None
    body_bearing_deg: float
    cmd_roll: float | None
    cmd_pitch: float | None
    cmd_thr: float | None
    issued: bool
    passed: bool

    @property
    def source_key(self) -> tuple[str, int, int, int]:
        return self.source, self.generation, self.task, self.obj


@dataclass(frozen=True)
class FinalApproachTiming:
    median_wall_gap_s: float | None = None
    max_wall_gap_s: float | None = None
    max_source_gap_s: float | None = None
    observed_speedup: float | None = None


@dataclass(frozen=True)
class FinalApproachScore:
    sample_count: int = 0
    significant_reversals: int = 0
    saturation_fraction: float = 0.0
    max_saturation_run: int = 0
    max_roll_step_deg: float = 0.0


@dataclass
class EpisodeMetrics:
    sample_count: int = 0
    snap_distance_m: float | None = None
    median_wall_gap_s: float | None = None
    max_wall_gap_s: float | None = None
    max_source_gap_s: float | None = None
    observed_speedup: float | None = None
    significant_reversals: int = 0
    saturation_fraction: float = 0.0
    max_saturation_run: int = 0
    max_roll_step_deg: float = 0.0
    confirmation_source_size_px: float | None = None
    confirmation_required_size_px: float | None = None
    source_process_pid: int | None = None
    scheduler_rate_hz: float | None = None
    attitude_source_count: int | None = None
    attitude_source_gap_p99_ms: float | None = None
    frame_source_gap_p99_ms: float | None = None
    observation_source_count: int | None = None
    observation_source_gap_p99_ms: float | None = None
    observation_age_p95_ms: float | None = None
    command_source_count: int | None = None
    command_wall_gap_p50_ms: float | None = None
    command_wall_gap_p99_ms: float | None = None
    command_age_p95_ms: float | None = None
    command_age_max_ms: float | None = None


@dataclass
class VehicleReport:
    sys_id: int
    role: str
    passed: bool
    metrics: EpisodeMetrics
    errors: list[str] = field(default_factory=list)


@dataclass
class RunReport:
    passed: bool
    log_dir: str
    sys_ids: list[int]
    sim_speedup: float
    navigation_speedup_override: float
    approvals: list[dict[str, JsonValue]]
    global_assignments: dict[int, int]
    vehicles: list[VehicleReport]
    errors: list[str] = field(default_factory=list)


@dataclass
class StackContext:
    api: JsonApiPort
    sys_ids: tuple[int, int, int]
    backend_tail: TailPort
    launch_evidence: dict[str, JsonValue] | None = None

    def __post_init__(self) -> None:
        self.sys_ids = ThreeUavIds.from_values(self.sys_ids).values


@dataclass
class MissionArtifacts:
    sys_ids: tuple[int, int, int]
    approvals: list[dict[str, JsonValue]]
    backend_log_text: str
    plan_path: str | None = None

    def __post_init__(self) -> None:
        self.sys_ids = ThreeUavIds.from_values(self.sys_ids).values


__all__ = [
    "CadenceLimits",
    "EpisodeMetrics",
    "GateLimits",
    "MissionArtifacts",
    "RegressionError",
    "RunReport",
    "RollQualityLimits",
    "StackContext",
    "FinalApproachCommand",
    "FinalApproachScore",
    "FinalApproachTiming",
    "ThreeUavIds",
    "TruthLimits",
    "VehicleReport",
    "finite_number",
    "positive_int",
]
