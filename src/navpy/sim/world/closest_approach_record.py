"""Closest approach to the own dock: one display record per approach episode.

Sim only (gcs_sim_closest_approach.md). The contact sensor reports the
episode's closest point to the true dock when the episode ends; the record is
written to stdout for the GCS backend, which starts every wall-clock sim NavPy
and reads its output, and to the NavPy log. It is display evidence only:
nothing here reaches frames, the navigation law or the command path.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, fields
from typing import Optional, TypeVar

NAVPY_SIM_CLOSEST_APPROACH_MARKER = "NAVPY_SIM_CLOSEST_APPROACH_V1"

OUTCOME_DOCKED = "docked"
OUTCOME_MISSED = "missed"
OUTCOMES = frozenset({OUTCOME_DOCKED, OUTCOME_MISSED})


@dataclass(frozen=True)
class ApproachPoint:
    lat: float
    lon: float
    alt_amsl_m: float
    # Height over home, the trail's display height; None without telemetry.
    alt_rel_m: Optional[float] = None


@dataclass(frozen=True)
class MissDistance:
    d3_m: float
    h_m: float
    v_m: float


@dataclass(frozen=True)
class AttitudeDeg:
    """The UAV attitude at that moment, degrees; display only (the ghost)."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class ClosestApproach:
    vehicle: int
    task_id: int
    dock_uid: int
    episode: int
    outcome: str
    aircraft_time_s: float
    point: ApproachPoint
    dock: ApproachPoint
    distance: MissDistance
    attitude: Optional[AttitudeDeg] = None


# The wire record is flat: the GCS map reads these keys.
_INT_KEYS = ("vehicle", "task_id", "dock_uid", "episode")
_DOCK_PREFIX = "dock_"


def to_mapping(record: ClosestApproach) -> dict[str, object]:
    """The flat wire mapping (attitude keys None without one)."""
    values: dict[str, object] = {
        key: getattr(record, key) for key in (*_INT_KEYS, "outcome", "aircraft_time_s")
    }
    values.update(asdict(record.point))
    values.update({_DOCK_PREFIX + k: v for k, v in asdict(record.dock).items()})
    values.update(asdict(record.distance))
    attitude = record.attitude
    for field in fields(AttitudeDeg):
        values[field.name] = None if attitude is None else getattr(attitude, field.name)
    return values


def encode(record: ClosestApproach) -> str:
    """One marker line: the marker, a space, compact sorted JSON."""
    payload = json.dumps(to_mapping(record), sort_keys=True, separators=(",", ":"))
    return f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} {payload}"


def parse(line: str) -> Optional[ClosestApproach]:
    """An exact, well-typed record line, else None (never raises)."""
    prefix = f"{NAVPY_SIM_CLOSEST_APPROACH_MARKER} "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        payload = json.loads(line[len(prefix):])
    except (TypeError, ValueError, RecursionError):
        return None
    if not isinstance(payload, dict):
        return None
    return from_mapping(payload)


def from_mapping(payload: Mapping[str, object]) -> Optional[ClosestApproach]:
    """A record from the flat wire mapping, else None (never raises)."""
    if any(not _integer(payload.get(key)) for key in _INT_KEYS):
        return None
    outcome = payload.get("outcome")
    if not isinstance(outcome, str) or outcome not in OUTCOMES:
        return None
    point = _point(payload, "")
    dock = _point(payload, _DOCK_PREFIX)
    distance = _values(payload, MissDistance)
    if point is None or dock is None or distance is None:
        return None
    if not _finite(payload.get("aircraft_time_s")):
        return None
    attitude_keys = [field.name for field in fields(AttitudeDeg)]
    if all(payload.get(key) is None for key in attitude_keys):
        attitude = None
    else:
        attitude = _values(payload, AttitudeDeg)
        if attitude is None:
            return None
    return ClosestApproach(
        *(payload[key] for key in _INT_KEYS),  # type: ignore[arg-type]
        outcome=outcome,
        aircraft_time_s=payload["aircraft_time_s"],  # type: ignore[arg-type]
        point=point,
        dock=dock,
        distance=distance,
        attitude=attitude,
    )


def _point(payload: Mapping[str, object], prefix: str) -> Optional[ApproachPoint]:
    alt_rel = payload.get(prefix + "alt_rel_m")
    if alt_rel is not None and not _finite(alt_rel):
        return None
    values = [payload.get(prefix + key) for key in ("lat", "lon", "alt_amsl_m")]
    if not all(_finite(value) for value in values):
        return None
    return ApproachPoint(*values, alt_rel_m=alt_rel)  # type: ignore[arg-type]


_Values = TypeVar("_Values", MissDistance, AttitudeDeg)


def _values(payload: Mapping[str, object], record_type: type[_Values]) -> Optional[_Values]:
    values = [payload.get(field.name) for field in fields(record_type)]
    if not all(_finite(value) for value in values):
        return None
    return record_type(*values)


def _integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _finite(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:  # an int too large for a float
        return False


def print_record(record: ClosestApproach) -> None:
    """Wall-clock sim: one flushed stdout line, read by the GCS backend."""
    print(encode(record), flush=True)


ClosestApproachPublisher = Callable[[ClosestApproach], None]

_publish: ClosestApproachPublisher = print_record


def set_publisher(publish: ClosestApproachPublisher) -> None:
    """Publish records through ``publish`` instead of stdout (lockstep, tests)."""
    global _publish
    _publish = publish


def publish(record: ClosestApproach) -> None:
    _publish(record)


__all__ = [
    "NAVPY_SIM_CLOSEST_APPROACH_MARKER",
    "OUTCOME_DOCKED",
    "OUTCOME_MISSED",
    "ApproachPoint",
    "AttitudeDeg",
    "ClosestApproach",
    "MissDistance",
    "ClosestApproachPublisher",
    "encode",
    "from_mapping",
    "parse",
    "print_record",
    "publish",
    "set_publisher",
    "to_mapping",
]
