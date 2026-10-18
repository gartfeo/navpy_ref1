"""The delivery profile's assignment evidence and run gates (e6).

Roles come from the backend's one APPLIED (``eval_delivery_roles.py``); each
role's checks are in ``eval_delivery_checks.py``. Each UAV must dock at its
role's world dock: the offerer at the kept dock, the winner at the offered
dock, home at its own zone's home dock. Docked means the DOCKED row: the sim
contact sensor's closest pass to that dock, from telemetry, inside the net
(owner, 2026-10-17; the frame-sampled SNAP line is coarser). A missed
approach ends with a DOCK_FAILED row (missed_dock.md), reported with its
reason. See docs/design/delivery_eval_scenario.md, "e6 as built".
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import cast

from navpy.sim.world.fleet_dock_world import DockSpec, parse_fleet_docks
from scripts.eval_delivery_checks import (
    assignment_errors,
    home_errors,
    offer_errors,
    offerer_errors,
    winner_errors,
    world_id_errors,
)
from scripts.eval_delivery_fixture import DOCK_FILE_NAME, load_fixture, zone_slot
from scripts.eval_delivery_roles import HOME, OFFERER, WINNER, DeliveryRoles, resolve_roles
from scripts.eval_gcs_demo_assignment_lines import AssignmentLines, LogFormat, parse_assignment_lines
from scripts.eval_gcs_demo_constants import backend_log_path
from scripts.eval_gcs_demo_docked import DockFailedEvidence
from scripts.eval_gcs_demo_evidence import mission_navigation_segment, parse_final_approach_outcomes
from scripts.eval_gcs_demo_assignment_audit import AssignmentEvidence
from scripts.eval_gcs_demo_models import FinalApproachEpisode
from scripts.eval_gcs_demo_poi_binding import vehicle_poi_binding_errors
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoMissionPlan, DemoScenario
from scripts.eval_gcs_demo_vehicle import navigation_log


@dataclass(frozen=True)
class DeliveryEvidence:
    sys_ids: tuple[int, ...]  # zone order
    expected: Mapping[str, JsonValue]
    docks: tuple[DockSpec, ...]
    texts: dict[int, str]  # full NavPy navigation logs
    episodes: dict[int, list[FinalApproachEpisode]]
    lines: AssignmentLines
    # The validated DOCK_FAILED rows of each UAV (missed_dock.md).
    failures: dict[int, list[DockFailedEvidence]] = field(default_factory=dict)

    def dock(self, name: str) -> int:
        return [dock.name for dock in self.docks].index(name)


def _backend_lines(log_dir: Path) -> AssignmentLines:
    lines = parse_assignment_lines(
        backend_log_path(log_dir).read_text(encoding="utf-8", errors="strict")
    )
    if lines.log_format is not LogFormat.ACKED:
        raise ValueError("the delivery run needs the acked assignment log format")
    return lines


def delivery_assignment_evidence(log_dir: Path, plan: DemoMissionPlan) -> AssignmentEvidence:
    """{helper: task} of each APPLIED; the run gates check them."""
    del plan
    try:
        lines = _backend_lines(log_dir)
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        return AssignmentEvidence({}, [f"invalid global assignment evidence: {error}"], False)
    return AssignmentEvidence({ack.helper: ack.task for ack in lines.acks}, [], True)


def load_delivery_evidence(
    log_dir: Path, plan: DemoMissionPlan, scenario: DemoScenario,
) -> DeliveryEvidence:
    fixture = load_fixture(scenario.path)
    flown = json.loads((log_dir / DOCK_FILE_NAME).read_text(encoding="utf-8"))
    if flown != fixture["docks"]:
        raise ValueError(f"{DOCK_FILE_NAME} is not the fixture's dock world")
    zones = {zone_slot(index): index for index in range(len(plan.vehicles))}
    sys_ids = tuple(
        vehicle.sys_id for vehicle in sorted(plan.vehicles, key=lambda item: zones[item.slot])
    )
    texts = {
        sys_id: navigation_log(log_dir, sys_id).read_text(encoding="utf-8", errors="replace")
        for sys_id in sys_ids
    }
    outcomes = {
        sys_id: parse_final_approach_outcomes(
            log_dir / f"uav_{sys_id}_navigation_debug.csv", vehicle_id=sys_id,
        )
        for sys_id in sys_ids
    }
    return DeliveryEvidence(
        sys_ids,
        cast(Mapping[str, JsonValue], fixture["expected"]),
        parse_fleet_docks(flown),
        texts,
        {sys_id: episodes for sys_id, (episodes, _) in outcomes.items()},
        _backend_lines(log_dir),
        {sys_id: failures for sys_id, (_, failures) in outcomes.items()},
    )


def _approved_at(approvals: Sequence[Mapping[str, JsonValue]], sys_id: int) -> tuple[int, float] | None:
    """(task, the earliest time the approval could be posted).

    approved_at_unix_s is taken when the POST returns, after the backend has
    forwarded the approval, so NavPy can arm before it. The POST is sent
    only once the operator delay has passed since the request was seen.
    """
    rows = [row for row in approvals if row["sys_id"] == sys_id]
    if len(rows) != 1:
        return None  # approval_counts reports it
    row = rows[0]
    posted = float(row["request_observed_at_unix_s"]) + float(row["requested_delay_s"])  # type: ignore[arg-type]
    return int(row["task_id"]), posted  # type: ignore[arg-type]


def docking_errors(
    sys_id: int, text: str, episodes: list[FinalApproachEpisode],
    docks: Sequence[DockSpec], dock: int, approval: tuple[int, float] | None,
    failures: Sequence[DockFailedEvidence] = (),
) -> list[str]:
    """One episode at *dock*, entered after its approval (the operator's
    earliest POST time, and NavPy's own confirmed -> INIT NAV order), then
    DOCKED there."""
    name = docks[dock].name
    errors = [
        f"UAV {sys_id} dock failed at "
        f"{docks[failed.dock].name if failed.dock < len(docks) else failed.dock}: {failed.reason}"
        for failed in failures
    ]
    if len(episodes) != 1:
        return errors + [f"UAV {sys_id} has {len(episodes)} final-approach episodes, not one at {name}"]
    episode = episodes[0]
    if {command.obj for command in episode.commands} != {dock}:
        errors.append(f"UAV {sys_id} approached dock ids {sorted({c.obj for c in episode.commands})}, not {name}")
    armed = episode.armed
    if armed is None or armed.dock != dock:
        errors.append(f"UAV {sys_id} has no DOCK_ARMED row for {name}")
    elif approval is not None and not (
        armed.wall_s is not None and math.isfinite(armed.wall_s) and armed.wall_s >= approval[1]
    ):
        errors.append(f"UAV {sys_id} entered NAV for {name} before its approval")
    if approval is not None and not (
        0 <= text.find(f"POI {approval[0]} confirmed by ground station.") < text.find("INIT: NAV MODE")
    ):
        errors.append(f"UAV {sys_id} entered NAV before it logged the approval of task {approval[0]}")
    docked = episode.docked_row
    if docked is None or docked.dock != dock:
        errors.append(f"UAV {sys_id} has no DOCKED row at {name}")
    elif armed is None or armed.source_ts is None or not docked.aircraft_ts >= armed.source_ts:
        errors.append(f"UAV {sys_id} DOCKED at {name} is not after its NAV entry")
    errors.extend(vehicle_poi_binding_errors(
        sys_id, text, [list(item.commands) for item in episodes],
        (docks[dock].lat, docks[dock].lon), approval[0] if approval else None,
    ))
    return errors


def delivery_errors(
    evidence: DeliveryEvidence, roles: DeliveryRoles,
    approvals: Sequence[Mapping[str, JsonValue]],
) -> list[str]:
    expected, docks = evidence.expected, evidence.docks
    kept, offered = evidence.dock(str(expected["kept_dock"])), evidence.dock(str(expected["offered_dock"]))
    mission = {sys_id: mission_navigation_segment(text) for sys_id, text in evidence.texts.items()}
    offerer = cast(int, roles.sys_id(OFFERER))
    homes = [evidence.dock(name) for name in roles.home_docks.values()]
    episode_ids = {
        sys_id: [command.obj for episode in items for command in episode.commands]
        + [episode.armed.dock for episode in items if episode.armed is not None]
        for sys_id, items in evidence.episodes.items()
    }
    errors = list(roles.errors)
    errors += world_id_errors(evidence.texts, episode_ids, len(docks))
    errors += offer_errors(evidence.texts, offerer, docks, offered, homes, roles.applied)
    errors += offerer_errors(mission[offerer], kept)
    targets = {offerer: kept}
    if roles.applied is not None:
        winner, home = cast(int, roles.sys_id(WINNER)), roles.sys_id(HOME)
        errors += assignment_errors(evidence.lines, roles.applied, home, docks[offered])
        errors += winner_errors(mission[winner], roles.applied, docks, offered)
        targets[winner] = offered
        if home is not None:
            targets[home] = evidence.dock(roles.home_docks[home])
            errors += home_errors(mission[home], docks, targets[home])
    for sys_id, dock in targets.items():
        errors += docking_errors(
            sys_id, mission[sys_id], evidence.episodes[sys_id], docks, dock,
            _approved_at(approvals, sys_id), evidence.failures.get(sys_id, ()),
        )
    return [f"delivery: {error}" for error in errors]


def delivery_run_gate_errors(
    log_dir: Path, plan: DemoMissionPlan,
    approvals: Sequence[Mapping[str, JsonValue]],
    scenario: DemoScenario | None = None,
) -> list[str]:
    if scenario is None:
        return ["delivery: the run gates need the delivery scenario"]
    try:
        evidence = load_delivery_evidence(log_dir, plan, scenario)
        applied = {(ack.owner, ack.helper, ack.task) for ack in evidence.lines.acks}
        roles = resolve_roles(evidence.sys_ids, evidence.expected, applied)
        return delivery_errors(evidence, roles, approvals)
    except (OSError, UnicodeError, KeyError, TypeError, ValueError) as error:
        return [f"delivery: invalid evidence: {error}"]


__all__ = [
    "DeliveryEvidence",
    "delivery_assignment_evidence",
    "delivery_errors",
    "delivery_run_gate_errors",
    "docking_errors",
    "load_delivery_evidence",
]
