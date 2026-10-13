"""Strict parsing of atomic command, approval, and speedup evidence."""

from __future__ import annotations

import csv
import math
import re
from pathlib import Path
from typing import Iterable

from navpy.core.approach.vision_nav.visual_pass import (
    centred_limit_deg,
    is_centred_angles,
)
from scripts.eval_gcs_demo_audit import (
    navigation_speedup_errors,
    load_approval_records,
    validate_approval_records,
)
from scripts.eval_gcs_demo_models import FinalApproachCommand


_FINAL_APPROACH_FIELDS = (
    "source",
    "generation",
    "task",
    "obj",
    "obs_ts",
    "dt_wall_ms",
    "body_bearing_deg",
    "cmd_roll",
    "cmd_pitch",
    "cmd_thr",
    "issued",
    "passed",
)
_FINAL_APPROACH_PASS_FIELDS = (
    "source",
    "generation",
    "task",
    "obj",
    "obs_ts",
    "body_bearing_deg",
    "reason",
    "camera_off_axis_deg",
    "camera_half_fov_deg",
)
_FINAL_APPROACH_PASS_REASONS = frozenset({"centred_loss"})


def seconds_of_day(value: str) -> float:
    match = re.fullmatch(r"(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?", value.strip())
    if match is None:
        raise ValueError(f"invalid log timestamp {value!r}")
    hour, minute, second = (int(match.group(index)) for index in range(1, 4))
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError(f"invalid log timestamp {value!r}")
    fraction = float(f"0.{match.group(4)}") if match.group(4) else 0.0
    return hour * 3600.0 + minute * 60.0 + second + fraction


def positive_deltas(values: Iterable[float]) -> list[float]:
    items = list(values)
    deltas: list[float] = []
    for previous, current in zip(items, items[1:]):
        delta = current - previous
        if delta < -43200.0:
            delta += 86400.0
        deltas.append(delta)
    return deltas


def _payload(
    payload: str,
    fields: tuple[str, ...] = _FINAL_APPROACH_FIELDS,
    event: str = "FINAL_APPROACH_CMD",
) -> dict[str, str]:
    pairs: list[tuple[str, str]] = []
    for token in payload.split(";"):
        if token.count("=") != 1:
            raise ValueError(f"malformed {event} token {token!r}")
        key, value = token.split("=", 1)
        pairs.append((key, value))
    keys = tuple(key for key, _value in pairs)
    if keys != fields:
        raise ValueError(f"{event} fields/order must be {fields}, got {keys}")
    return dict(pairs)


def _integer(name: str, value: str, *, allow_zero: bool) -> int:
    if re.fullmatch(r"0|[1-9]\d*", value) is None:
        raise ValueError(f"{name} must be a canonical integer")
    number = int(value)
    if not allow_zero and number == 0:
        raise ValueError(f"{name} must be positive")
    return number


def _float(name: str, value: str, *, optional: bool = False) -> float | None:
    if optional and value == "":
        return None
    try:
        number = float(value)
    except ValueError as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _boolean(name: str, value: str) -> bool:
    if value == "True":
        return True
    if value == "False":
        return False
    raise ValueError(f"{name} must be exactly True or False")


def _final_approach_command(row: list[str], line_number: int) -> FinalApproachCommand:
    if len(row) < 3:
        raise ValueError(f"FINAL_APPROACH_CMD row {line_number} has fewer than 3 columns")
    payload = _payload(row[2])
    source = payload["source"]
    if not source or source.strip() != source:
        raise ValueError("FINAL_APPROACH_CMD source must be a non-empty exact string")
    issued = _boolean("issued", payload["issued"])
    passed = _boolean("passed", payload["passed"])
    command = FinalApproachCommand(
        wall_s=seconds_of_day(row[0]),
        source=source,
        generation=_integer("generation", payload["generation"], allow_zero=True),
        task=_integer("task", payload["task"], allow_zero=False),
        obj=_integer("obj", payload["obj"], allow_zero=True),
        obs_ts=_float("obs_ts", payload["obs_ts"]),  # type: ignore[arg-type]
        dt_wall_ms=_float("dt_wall_ms", payload["dt_wall_ms"], optional=True),
        body_bearing_deg=_float("body_bearing_deg", payload["body_bearing_deg"]),  # type: ignore[arg-type]
        cmd_roll=_float("cmd_roll", payload["cmd_roll"], optional=True),
        cmd_pitch=_float("cmd_pitch", payload["cmd_pitch"], optional=True),
        cmd_thr=_float("cmd_thr", payload["cmd_thr"], optional=True),
        issued=issued,
        passed=passed,
    )
    if issued and (command.cmd_roll is None or command.cmd_pitch is None):
        raise ValueError("issued FINAL_APPROACH_CMD must contain roll and pitch")
    if issued and passed:
        raise ValueError("FINAL_APPROACH_CMD cannot be both issued and passed")
    if not issued and any(
        value is not None
        for value in (command.cmd_roll, command.cmd_pitch, command.cmd_thr)
    ):
        raise ValueError("unissued FINAL_APPROACH_CMD must not contain command values")
    if command.dt_wall_ms is not None and command.dt_wall_ms < 0.0:
        raise ValueError("FINAL_APPROACH_CMD dt_wall_ms must be non-negative")
    if not -180.0 <= command.body_bearing_deg <= 180.0:
        raise ValueError("FINAL_APPROACH_CMD body_bearing_deg must be in [-180, 180]")
    if command.cmd_roll is not None and not -180.0 <= command.cmd_roll <= 180.0:
        raise ValueError("FINAL_APPROACH_CMD cmd_roll must be in [-180, 180]")
    if command.cmd_pitch is not None and not -90.0 <= command.cmd_pitch <= 90.0:
        raise ValueError("FINAL_APPROACH_CMD cmd_pitch must be in [-90, 90]")
    if command.cmd_thr is not None and not 0.0 <= command.cmd_thr <= 1.0:
        raise ValueError("FINAL_APPROACH_CMD cmd_thr must be in [0, 1]")
    return command


def _final_approach_pass(row: list[str], line_number: int) -> FinalApproachCommand:
    """A pass latched without a new frame, as a trailing pass-suppressed row."""
    if len(row) < 3:
        raise ValueError(f"FINAL_APPROACH_PASS row {line_number} has fewer than 3 columns")
    payload = _payload(row[2], _FINAL_APPROACH_PASS_FIELDS, "FINAL_APPROACH_PASS")
    source = payload["source"]
    if not source or source.strip() != source:
        raise ValueError("FINAL_APPROACH_PASS source must be a non-empty exact string")
    reason = payload["reason"]
    if reason not in _FINAL_APPROACH_PASS_REASONS:
        raise ValueError(f"FINAL_APPROACH_PASS reason {reason!r} is not recognised")
    angles: dict[str, float] = {}
    for name in ("camera_off_axis_deg", "camera_half_fov_deg"):
        angle = _float(name, payload[name])
        if angle is None or not 0.0 <= angle <= 180.0:
            raise ValueError(f"FINAL_APPROACH_PASS {name} must be in [0, 180]")
        angles[name] = angle
    # The same completion predicate the runtime latched on (visual_pass.py).
    limit_deg = centred_limit_deg(angles["camera_half_fov_deg"])
    if not is_centred_angles(
        angles["camera_off_axis_deg"], angles["camera_half_fov_deg"]
    ):
        raise ValueError(
            "FINAL_APPROACH_PASS centred_loss off-axis "
            f"{angles['camera_off_axis_deg']:g} deg exceeds the centred limit "
            f"{limit_deg:g} deg (half-FOV {angles['camera_half_fov_deg']:g} deg)"
        )
    command = FinalApproachCommand(
        wall_s=seconds_of_day(row[0]),
        source=source,
        generation=_integer("generation", payload["generation"], allow_zero=True),
        task=_integer("task", payload["task"], allow_zero=False),
        obj=_integer("obj", payload["obj"], allow_zero=True),
        obs_ts=_float("obs_ts", payload["obs_ts"]),  # type: ignore[arg-type]
        dt_wall_ms=None,
        body_bearing_deg=_float("body_bearing_deg", payload["body_bearing_deg"]),  # type: ignore[arg-type]
        cmd_roll=None,
        cmd_pitch=None,
        cmd_thr=None,
        issued=False,
        passed=True,
        pass_reason=reason,
    )
    if not -180.0 <= command.body_bearing_deg <= 180.0:
        raise ValueError("FINAL_APPROACH_PASS body_bearing_deg must be in [-180, 180]")
    return command


def parse_final_approach_commands(path: Path) -> list[FinalApproachCommand]:
    commands: list[FinalApproachCommand] = []
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            if len(row) >= 2 and row[1] == "EVENT:FINAL_APPROACH_CMD":
                commands.append(_final_approach_command(row, line_number))
    return commands


def parse_final_approach_command_episodes(path: Path) -> list[list[FinalApproachCommand]]:
    """Require one source identity and explicit pass before each debug SNAP.

    The pass is a pass-suppressed FINAL_APPROACH_CMD row or a
    FINAL_APPROACH_PASS row (dock lost centred) with the episode's identity.
    """
    episodes: list[list[FinalApproachCommand]] = []
    current: list[FinalApproachCommand] = []
    source_key: tuple[str, int, int, int] | None = None
    passed = False
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            event = row[1] if len(row) >= 2 else ""
            if event == "EVENT:FINAL_APPROACH_CMD":
                command = _final_approach_command(row, line_number)
                if source_key is None:
                    source_key = command.source_key
                elif command.source_key != source_key:
                    raise ValueError("final-approach episode changed atomic source identity")
                if passed and (command.issued or not command.passed):
                    raise ValueError(
                        "non-pass-suppressed FINAL_APPROACH_CMD appeared after pass "
                        "and before SNAP"
                    )
                current.append(command)
                passed = passed or command.passed
            elif event == "EVENT:FINAL_APPROACH_PASS":
                latched = _final_approach_pass(row, line_number)
                if not current:
                    raise ValueError("FINAL_APPROACH_PASS outside a final-approach episode")
                if latched.source_key != source_key:
                    raise ValueError("FINAL_APPROACH_PASS changed atomic source identity")
                if passed:
                    raise ValueError("FINAL_APPROACH_PASS after the episode already passed")
                if latched.obs_ts < max(command.obs_ts for command in current):
                    raise ValueError(
                        "FINAL_APPROACH_PASS obs_ts precedes earlier command evidence"
                    )
                current.append(latched)
                passed = True
            elif event == "EVENT:SNAP_COMPONENTS" and current:
                if not passed:
                    raise ValueError("final-approach episode reached SNAP without passed=True")
                episodes.append(current)
                current = []
                source_key = None
                passed = False
    if current:
        raise ValueError("final-approach episode did not terminate at SNAP_COMPONENTS")
    return episodes


def mission_navigation_segment(navigation_text: str) -> str:
    snap_at = navigation_text.find("SNAP(VISION-NAV")
    if snap_at < 0:
        return navigation_text
    process_start = navigation_text.rfind("Heartbeat from system", 0, snap_at)
    return navigation_text[process_start:] if process_start >= 0 else navigation_text


__all__ = [
    "navigation_speedup_errors",
    "load_approval_records",
    "mission_navigation_segment",
    "parse_final_approach_command_episodes",
    "parse_final_approach_commands",
    "positive_deltas",
    "seconds_of_day",
    "validate_approval_records",
]
