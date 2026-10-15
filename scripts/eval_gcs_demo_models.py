"""Validated data contracts for the scenario-driven GCS demo evaluator."""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from scripts.eval_gcs_demo_ports import (
    ClockPort,
    EventStreamPort,
    JsonApiPort,
    JsonValue,
    TailPort,
)
from scripts.eval_gcs_demo_limits import (
    CadenceLimits,
    GateLimits,
    RollQualityLimits,
)

if TYPE_CHECKING:
    # The scenario module imports this one; the annotation stays a string.
    from scripts.eval_gcs_demo_scenario import DemoScenario


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
class UavIds:
    """Unique positive sysids, exactly as many as the scenario has slots."""

    values: tuple[int, ...]

    @classmethod
    def from_values(cls, values: Iterable[object], count: int) -> UavIds:
        expected = positive_int("UAV id count", count)
        items = tuple(values)
        if len(items) != expected:
            raise ValueError(f"exactly {expected} UAV ids are required, got {items!r}")
        checked = tuple(positive_int("sys_id", value) for value in items)
        if len(set(checked)) != expected:
            raise ValueError(f"UAV ids must be unique, got {checked!r}")
        return cls(checked)


@dataclass(frozen=True)
class FinalApproachCommand:
    """Immutable passive copy of one atomic FINAL_APPROACH_CMD payload."""

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
    # The aft gate declared the dock missed (NAV_FAIL: missed_dock).
    missed: bool

    @property
    def source_key(self) -> tuple[str, int, int, int]:
        return self.source, self.generation, self.task, self.obj


@dataclass(frozen=True)
class FinalApproachResponse:
    """Pairing key and achieved roll of one FINAL_APPROACH_RESPONSE_STATE row.

    Parsed leniently: the achieved-roll gate, not the parser, rejects bad
    evidence, so ``None`` means absent or non-numeric and a non-finite roll
    is kept as read. ``repeated_field`` marks a row naming a field twice.
    """

    source: str
    obs_ts: float | None
    aircraft_roll_deg: float | None
    repeated_field: bool = False


@dataclass(frozen=True)
class FinalApproachEpisode:
    """One episode's command rows and the response rows in its span."""

    commands: tuple[FinalApproachCommand, ...]
    responses: tuple[FinalApproachResponse, ...]
    #: A valid DOCKED row before the episode's SNAP; its absence is the
    #: docking gate's failure, not an evidence error.
    docked: bool


@dataclass(frozen=True)
class DockArmed:
    """One DOCK_ARMED row: the episode the contact sensor was armed for."""

    vehicle: int
    task: int
    dock: int
    episode: int

    @property
    def identity(self) -> tuple[int, int, int, int]:
        return self.vehicle, self.task, self.dock, self.episode


@dataclass(frozen=True)
class DockedEvidence:
    """One DOCKED row: the sim contact sensor's success event."""

    wall_s: float
    vehicle: int
    task: int
    dock: int
    episode: int
    aircraft_ts: float
    closest_3d_m: float
    closest_h_m: float
    closest_v_m: float

    @property
    def identity(self) -> tuple[int, int, int, int]:
        return self.vehicle, self.task, self.dock, self.episode


@dataclass(frozen=True)
class FinalApproachTiming:
    median_wall_gap_s: float | None = None
    max_wall_gap_s: float | None = None
    max_source_gap_s: float | None = None
    observed_speedup: float | None = None


@dataclass(frozen=True)
class FinalApproachScore:
    sample_count: int = 0
    # Commanded-roll reversals: a diagnostic only, not gated.
    significant_reversals: int = 0
    saturation_fraction: float = 0.0
    max_saturation_run: int = 0
    max_roll_step_deg: float = 0.0


@dataclass(frozen=True)
class AchievedRollScore:
    """Achieved-roll reversals, or the named reasons the evidence is invalid."""

    reversals: int | None = None
    evidence_errors: tuple[str, ...] = ()


@dataclass
class EpisodeMetrics:
    sample_count: int = 0
    snap_distance_m: float | None = None
    median_wall_gap_s: float | None = None
    max_wall_gap_s: float | None = None
    max_source_gap_s: float | None = None
    observed_speedup: float | None = None
    # Commanded-roll reversals: a diagnostic only, not gated.
    significant_reversals: int = 0
    achieved_roll_reversals: int | None = None
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
    """One owned stack as the operator sees it: its scenario and its I/O ports."""

    api: JsonApiPort
    sys_ids: tuple[int, ...]
    backend_tail: TailPort
    #: The operator's clock: every wait and deadline of the eval reads it.
    clock: ClockPort
    scenario: "DemoScenario"
    #: Opens the backend event stream for a WebSocket URL.
    event_stream_factory: Callable[[str], EventStreamPort]
    #: The NavPy log lines of one sysid; the operator reads no log file itself.
    navigation_tail: Callable[[int], TailPort]
    launch_evidence: dict[str, JsonValue] | None = None
    #: The slot's GCS monitor endpoint, for the explicit connect.
    monitor_device: str | None = None
    #: Reads one vehicle's parameter snapshot for the command bounds. None is
    #: wall mode's MAVFTP refresh; lockstep reads the bound parameters by name
    #: (owner ruling 7).
    parameter_reader: Callable[[JsonApiPort, int], JsonValue] | None = None

    def __post_init__(self) -> None:
        self.sys_ids = UavIds.from_values(self.sys_ids, len(self.scenario.slots)).values


@dataclass
class MissionArtifacts:
    sys_ids: tuple[int, ...]
    approvals: list[dict[str, JsonValue]]
    #: The scenario's slot count, which the sysids must match.
    count: int
    plan_path: str | None = None

    def __post_init__(self) -> None:
        self.sys_ids = UavIds.from_values(self.sys_ids, self.count).values


__all__ = [
    "AchievedRollScore",
    "CadenceLimits",
    "EpisodeMetrics",
    "GateLimits",
    "MissionArtifacts",
    "RegressionError",
    "RunReport",
    "RollQualityLimits",
    "StackContext",
    "FinalApproachCommand",
    "FinalApproachEpisode",
    "FinalApproachResponse",
    "FinalApproachScore",
    "FinalApproachTiming",
    "UavIds",
    "VehicleReport",
    "finite_number",
    "positive_int",
]
