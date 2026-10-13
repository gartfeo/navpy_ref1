"""Strict parsing of atomic command, approval, and speedup evidence."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from scripts.eval_gcs_demo_audit import (
    navigation_speedup_errors,
    load_approval_records,
    validate_approval_records,
)
from scripts.eval_gcs_demo_docked import (
    DOCKED_AFTER_LAST_FRAME_MAX_S,
    DOCK_NET_RADIUS_M,
    docked_errors as _docked_errors,
    parse_dock_armed as _dock_armed,
    parse_docked as _docked,
)
from scripts.eval_gcs_demo_models import (
    DockArmed,
    DockedEvidence,
    FinalApproachCommand,
)
from scripts.eval_gcs_demo_tokens import (
    canonical_integer as _integer,
    exact_boolean as _boolean,
    finite_number as _float,
    payload_fields,
    seconds_of_day,
)


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
    "missed",
)
def positive_deltas(values: Iterable[float]) -> list[float]:
    items = list(values)
    deltas: list[float] = []
    for previous, current in zip(items, items[1:]):
        delta = current - previous
        if delta < -43200.0:
            delta += 86400.0
        deltas.append(delta)
    return deltas


def _final_approach_command(row: list[str], line_number: int) -> FinalApproachCommand:
    if len(row) < 3:
        raise ValueError(f"FINAL_APPROACH_CMD row {line_number} has fewer than 3 columns")
    payload = payload_fields(row[2], _FINAL_APPROACH_FIELDS, "FINAL_APPROACH_CMD")
    source = payload["source"]
    if not source or source.strip() != source:
        raise ValueError("FINAL_APPROACH_CMD source must be a non-empty exact string")
    issued = _boolean("issued", payload["issued"])
    missed = _boolean("missed", payload["missed"])
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
        missed=missed,
    )
    if issued and (command.cmd_roll is None or command.cmd_pitch is None):
        raise ValueError("issued FINAL_APPROACH_CMD must contain roll and pitch")
    if issued and missed:
        raise ValueError("FINAL_APPROACH_CMD cannot be both issued and missed")
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


def parse_final_approach_commands(path: Path) -> list[FinalApproachCommand]:
    commands: list[FinalApproachCommand] = []
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            if len(row) >= 2 and row[1] == "EVENT:FINAL_APPROACH_CMD":
                commands.append(_final_approach_command(row, line_number))
    return commands


def parse_final_approach_command_episodes(
    path: Path,
    *,
    vehicle_id: int | None = None,
) -> list[list[FinalApproachCommand]]:
    """Require one source identity and one DOCKED row before each debug SNAP.

    Docking counts only from DOCKED (the sim contact sensor), matched to the
    episode's own DOCK_ARMED row. A duplicate, stale or wrong-episode event,
    an event for another vehicle or another dock, an event outside the
    episode's command time span, and any command row after the event are
    rejected.
    """
    episodes: list[list[FinalApproachCommand]] = []
    current: list[FinalApproachCommand] = []
    source_key: tuple[str, int, int, int] | None = None
    armed: DockArmed | None = None
    docked: DockedEvidence | None = None
    last_episode: int | None = None
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            event = row[1] if len(row) >= 2 else ""
            if event == "EVENT:DOCK_ARMED":
                if current:
                    raise ValueError("DOCK_ARMED inside an open final-approach episode")
                armed = _dock_armed(row, line_number)
            elif event == "EVENT:FINAL_APPROACH_CMD":
                command = _final_approach_command(row, line_number)
                if docked is not None:
                    raise ValueError("FINAL_APPROACH_CMD appeared after DOCKED")
                if source_key is None:
                    source_key = command.source_key
                elif command.source_key != source_key:
                    raise ValueError("final-approach episode changed atomic source identity")
                current.append(command)
            elif event == "EVENT:DOCKED":
                latched = _docked(row, line_number)
                if docked is not None:
                    raise ValueError("duplicate DOCKED in one final-approach episode")
                error = _docked_errors(
                    latched, armed, current, vehicle_id, last_episode,
                )
                if error is not None:
                    raise ValueError(error)
                docked = latched
                last_episode = latched.episode
            elif event == "EVENT:SNAP_COMPONENTS":
                if current:
                    if docked is None:
                        raise ValueError(
                            "final-approach episode reached SNAP without DOCKED"
                        )
                    episodes.append(current)
                current = []
                source_key = None
                armed = None
                docked = None
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
    "DOCKED_AFTER_LAST_FRAME_MAX_S",
    "DOCK_NET_RADIUS_M",
    "navigation_speedup_errors",
    "load_approval_records",
    "mission_navigation_segment",
    "parse_final_approach_command_episodes",
    "parse_final_approach_commands",
    "positive_deltas",
    "seconds_of_day",
    "validate_approval_records",
]
