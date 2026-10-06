"""Captured autopilot command bounds for terminal-evidence validation."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from scripts.eval_gcs_demo_models import TerminalCommand, ThreeUavIds, positive_int
from scripts.eval_gcs_demo_ports import JsonValue


@dataclass(frozen=True)
class VehicleCommandBounds:
    sys_id: int
    pitch_min_deg: float
    pitch_max_deg: float
    roll_limit_deg: float
    throttle: float

    def __post_init__(self) -> None:
        positive_int("command-bounds sys_id", self.sys_id)
        values = (
            self.pitch_min_deg,
            self.pitch_max_deg,
            self.roll_limit_deg,
            self.throttle,
        )
        if any(
            isinstance(value, bool) or not math.isfinite(float(value))
            for value in values
        ):
            raise ValueError("command bounds must contain finite numbers")
        if not -90.0 <= self.pitch_min_deg < self.pitch_max_deg <= 90.0:
            raise ValueError("captured pitch bounds are invalid")
        if not 0.0 < self.roll_limit_deg <= 90.0:
            raise ValueError("captured roll limit is invalid")
        if not 0.0 <= self.throttle <= 1.0:
            raise ValueError("captured throttle is outside [0, 1]")


@dataclass(frozen=True)
class DemoCommandBounds:
    schema_version: int
    vehicles: tuple[VehicleCommandBounds, VehicleCommandBounds, VehicleCommandBounds]

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ValueError("command-bounds schema_version must be 1")
        ThreeUavIds.from_values(item.sys_id for item in self.vehicles)

    def for_vehicle(self, sys_id: int) -> VehicleCommandBounds:
        checked = positive_int("command-bounds sys_id", sys_id)
        return next(item for item in self.vehicles if item.sys_id == checked)


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _finite(name: str, value: JsonValue) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def command_bounds_from_snapshot(
    sys_id: int,
    snapshot: JsonValue,
    *,
    roll_limit_deg: float,
) -> VehicleCommandBounds:
    checked = positive_int("command-bounds sys_id", sys_id)
    body = _object(snapshot, "full parameter snapshot")
    response_sys_id = positive_int(
        "full parameter snapshot sys_id",
        body.get("sys_id"),
    )
    if response_sys_id != checked:
        raise ValueError("full parameter snapshot sys_id mismatch")
    raw_records = body.get("params")
    if not isinstance(raw_records, list):
        raise ValueError("full parameter snapshot params must be a list")
    values: dict[str, JsonValue] = {}
    for raw in raw_records:
        record = _object(raw, "full parameter record")
        name = record.get("name")
        if type(name) is not str or not name:
            raise ValueError("full parameter record name must be a string")
        if name in values:
            raise ValueError(f"duplicate full parameter record {name}")
        values[name] = record.get("value")
    required = (
        "PTCH_LIM_MIN_DEG",
        "PTCH_LIM_MAX_DEG",
        "AAS_DEL_THR",
        "TRIM_THROTTLE",
    )
    missing = [name for name in required if name not in values]
    if missing:
        raise ValueError(f"full parameter snapshot lacks command bounds {missing}")
    configured = _finite("AAS_DEL_THR", values["AAS_DEL_THR"])
    trim = _finite("TRIM_THROTTLE", values["TRIM_THROTTLE"])
    throttle_percent = configured if configured >= 0.0 else trim
    return VehicleCommandBounds(
        checked,
        _finite("PTCH_LIM_MIN_DEG", values["PTCH_LIM_MIN_DEG"]),
        _finite("PTCH_LIM_MAX_DEG", values["PTCH_LIM_MAX_DEG"]),
        _finite("ROLL_LIMIT_DEG", roll_limit_deg),
        throttle_percent / 100.0,
    )


def command_bounds_errors(
    commands: list[TerminalCommand],
    bounds: VehicleCommandBounds,
) -> list[str]:
    errors: list[str] = []
    outside_roll = [
        item.cmd_roll
        for item in commands
        if item.issued
        and item.cmd_roll is not None
        and abs(item.cmd_roll) > bounds.roll_limit_deg
    ]
    outside_pitch = [
        item.cmd_pitch
        for item in commands
        if item.issued
        and item.cmd_pitch is not None
        and not bounds.pitch_min_deg <= item.cmd_pitch <= bounds.pitch_max_deg
    ]
    wrong_throttle = [
        item.cmd_thr
        for item in commands
        if item.issued
        and (
            item.cmd_thr is None
            or not math.isclose(item.cmd_thr, bounds.throttle, abs_tol=1e-6)
        )
    ]
    if outside_roll:
        errors.append(
            "terminal roll command exceeds configured ROLL_LIMIT_DEG: "
            f"{outside_roll}"
        )
    if outside_pitch:
        errors.append(f"terminal pitch command exceeds captured bounds: {outside_pitch}")
    if wrong_throttle:
        errors.append(
            "terminal throttle command differs from captured runtime value: "
            f"{wrong_throttle}"
        )
    return errors


def parse_command_bounds(payload: JsonValue) -> DemoCommandBounds:
    body = _object(payload, "demo command bounds")
    if set(body) != {"schema_version", "vehicles"} or body["schema_version"] != 1:
        raise ValueError("invalid demo command-bounds schema")
    raw_vehicles = body["vehicles"]
    if not isinstance(raw_vehicles, list) or len(raw_vehicles) != 3:
        raise ValueError("demo command bounds require exactly three vehicles")
    keys = {
        "sys_id",
        "pitch_min_deg",
        "pitch_max_deg",
        "roll_limit_deg",
        "throttle",
    }
    vehicles: list[VehicleCommandBounds] = []
    for raw in raw_vehicles:
        item = _object(raw, "vehicle command bounds")
        if set(item) != keys:
            raise ValueError("invalid vehicle command-bounds schema")
        vehicles.append(VehicleCommandBounds(
            positive_int("command-bounds sys_id", item["sys_id"]),
            _finite("pitch_min_deg", item["pitch_min_deg"]),
            _finite("pitch_max_deg", item["pitch_max_deg"]),
            _finite("roll_limit_deg", item["roll_limit_deg"]),
            _finite("throttle", item["throttle"]),
        ))
    return DemoCommandBounds(1, tuple(vehicles))  # type: ignore[arg-type]


def persist_command_bounds(bounds: DemoCommandBounds, path: Path) -> None:
    serialized = json.loads(json.dumps(asdict(bounds)))
    validated = parse_command_bounds(serialized)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(asdict(validated), indent=2), encoding="utf-8")
    temporary.replace(path)


def load_command_bounds(path: Path) -> DemoCommandBounds:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid demo command-bounds artifact {path}: {error}") from error
    return parse_command_bounds(payload)


__all__ = [
    "DemoCommandBounds",
    "VehicleCommandBounds",
    "command_bounds_errors",
    "command_bounds_from_snapshot",
    "load_command_bounds",
    "parse_command_bounds",
    "persist_command_bounds",
]
