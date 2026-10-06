"""Data contracts shared by the navigation evaluator's focused services."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MatrixCase:
    target_alt_m: int
    speedup: float
    navigation_speedup: float
    wind_speed_mps: float
    wind_direction_deg: int


@dataclass(frozen=True)
class MissionItem:
    seq: int
    command: int
    frame: int
    lat_deg: float | None
    lon_deg: float | None
    mission_alt_m: float | None
    param1: float | None = None
    param2: float | None = None
    param3: float | None = None
    param4: float | None = None
    current: int | None = None
    autocontinue: int | None = None
    mission_type: int | None = None


@dataclass(frozen=True)
class TargetLocation:
    lat_deg: float
    lon_deg: float
    rel_alt_m: float
    abs_alt_m: float


@dataclass(frozen=True)
class TargetExpectation:
    target_wp: int
    mission_seq: int
    expected_task_id: int
    expected_obj_id: int
    location: TargetLocation


@dataclass(frozen=True)
class SelectionEvidence:
    catalog_wp: int | None
    catalog_seq: int | None
    task_id: int | None
    obj_id: int | None
    fallback_location_registered: bool
    event_wall_time_s: float | None
    target_lat_deg: float | None
    target_lon_deg: float | None
    target_abs_alt_m: float | None

    def to_record(self) -> dict[str, object]:
        """Keep the established evidence format at its serialization boundary."""
        return {
            "catalog_wp": self.catalog_wp,
            "catalog_seq": self.catalog_seq,
            "task_id": self.task_id,
            "obj_id": self.obj_id,
            "default_ooi_registered": self.fallback_location_registered,
            "event_wall_time_s": self.event_wall_time_s,
            "target_lat_deg": self.target_lat_deg,
            "target_lon_deg": self.target_lon_deg,
            "target_abs_alt_m": self.target_abs_alt_m,
        }


@dataclass(frozen=True)
class EvidenceGateResult:
    passed: bool
    errors: tuple[str, ...]
    coordinate_error_m: float | None
    altitude_error_m: float | None


@dataclass(frozen=True)
class PositionSample:
    lat_deg: float
    lon_deg: float
    abs_alt_m: float
    rel_alt_m: float
    received_wall_time_s: float
    source_time_s: float | None = None


@dataclass(frozen=True)
class PositionStreamAnchor:
    source_time_s: float
    live_edge_wall_time_s: float


@dataclass(frozen=True)
class ClosestApproach:
    dist_3d_m: float
    horizontal_m: float
    vertical_m: float


@dataclass(frozen=True)
class LaunchVerdict:
    chat: int
    requested_speedup: float | None
    measured_rates: dict[str, float]

    @property
    def measured_rate(self) -> float | None:
        if len(self.measured_rates) != 1:
            return None
        return next(iter(self.measured_rates.values()))
