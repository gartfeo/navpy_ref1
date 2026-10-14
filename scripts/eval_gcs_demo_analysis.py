"""Resolved-plan-driven analysis for one explicit demo scenario."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import NamedTuple, Sequence

from scripts.eval_gcs_demo_audit import (
    load_approval_records,
    validate_approval_records,
)
from scripts.eval_gcs_demo_assignment_audit import derive_global_assignment_map
from scripts.eval_gcs_demo_assignment_lines import LogFormat, parse_assignment_lines
from scripts.eval_gcs_demo_command_bounds import load_command_bounds
from scripts.eval_gcs_demo_constants import (
    NAVIGATION_SPEEDUP_OVERRIDE,
    backend_log_path,
)
from scripts.eval_gcs_demo_models import (
    GateLimits,
    RunReport,
    finite_number,
)
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_gate_profiles import (
    CERTIFIED_PROFILE,
    OWNER_ONLY_SLICE_PROFILE,
    PROFILE_SIM_SPEEDUP,
)
from scripts.eval_gcs_demo_inputs import recorded_lockstep, require_recorded_scenario
from scripts.eval_gcs_demo_scenario import DemoMissionPlan, DemoScenario, load_resolved_plan
from scripts.eval_gcs_demo_poi_binding import final_approach_poi_binding_errors
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, slice_gate_errors
from scripts.eval_gcs_demo_vehicle import analyze_vehicle


def mission_assignment_errors(
    log_dir: Path,
    plan: DemoMissionPlan,
    global_assignments: dict[int, int],
    approvals: list[dict[str, JsonValue]],
) -> list[str]:
    approved_tasks: dict[int, int] = {}
    for sys_id in plan.sys_ids:
        rows = [row for row in approvals if row["sys_id"] == sys_id]
        if len(rows) == 1:
            approved_tasks[sys_id] = int(rows[0]["task_id"])
    return final_approach_poi_binding_errors(
        log_dir,
        plan,
        global_assignments,
        approved_tasks,
    )


def approval_counts(
    approvals: list[dict[str, JsonValue]],
    plan: DemoMissionPlan,
) -> tuple[dict[int, int], list[str]]:
    errors: list[str] = []
    ids = plan.sys_ids
    expected = set(ids)
    observed = {int(item["sys_id"]) for item in approvals}
    if observed - expected:
        errors.append(f"approval artifact contains unknown sysids {sorted(observed - expected)}")
    counts = {
        sys_id: sum(item["sys_id"] == sys_id for item in approvals)
        for sys_id in ids
    }
    for sys_id, count in counts.items():
        if count != 1:
            errors.append(
                f"vehicle {sys_id} expected exactly one audited approval, found {count}"
            )
    return counts, errors


class AssignmentEvidence(NamedTuple):
    assignments: dict[int, int]
    errors: list[str]
    # Acked log format: peers fly only after their owner's APPLIED.
    acked: bool


def global_assignment_evidence(
    log_dir: Path,
    plan: DemoMissionPlan,
) -> AssignmentEvidence:
    acked = False
    try:
        backend_log = backend_log_path(log_dir).read_text(
            encoding="utf-8",
            errors="strict",
        )
        acked = parse_assignment_lines(backend_log).log_format is LogFormat.ACKED
        assignments = derive_global_assignment_map(backend_log, plan)
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        return AssignmentEvidence(
            {}, [f"invalid global assignment evidence: {error}"], acked,
        )
    return AssignmentEvidence(assignments, [], acked)


def owner_only_assignment_evidence(
    log_dir: Path,
    plan: DemoMissionPlan,
) -> AssignmentEvidence:
    """The slice's one assignment, {owner: its one POI}, from the scenario.

    The backend log must hold no task-assignment line: with no peer, no
    request, response or APPLIED exists. Its absence alone proves nothing
    (the listener suppresses adverts with one vehicle); the NavPy log gate
    and the journal's message counts (``slice_gate_errors``) carry that proof.
    """
    if plan.peers or len(plan.pois) != 1:
        raise ValueError("an owner-only assignment needs one vehicle and one POI")
    assignments = {plan.owner.sys_id: plan.pois[0].task_id}
    try:
        backend_log = backend_log_path(log_dir).read_text(
            encoding="utf-8",
            errors="strict",
        )
        lines = parse_assignment_lines(backend_log)
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        return AssignmentEvidence(
            assignments, [f"invalid global assignment evidence: {error}"], False,
        )
    count = len(lines.requests) + len(lines.responses) + len(lines.acks)
    errors = (
        [f"owner-only slice backend log holds {count} task-assignment lines"]
        if count
        else []
    )
    return AssignmentEvidence(assignments, errors, False)


def _no_run_gates(
    _log_dir: Path, _plan: DemoMissionPlan, _approvals: list[dict[str, JsonValue]],
) -> list[str]:
    return []


WALL, LOCKSTEP = "wall", "lockstep"
#: (gate profile, execution mode) -> its run-level gates beyond the shared
#: ones: a lockstep slice run adds the journal gates; a wall run (either
#: profile) has none. The certified profile has no lockstep run before step 10.
RunGates = Callable[[Path, DemoMissionPlan, list[dict[str, JsonValue]]], list[str]]
_RUN_GATES: dict[tuple[str, str], RunGates] = {
    (CERTIFIED_PROFILE, WALL): _no_run_gates,
    (OWNER_ONLY_SLICE_PROFILE, WALL): _no_run_gates,
    (OWNER_ONLY_SLICE_PROFILE, LOCKSTEP): slice_gate_errors,
}


def execution_mode(log_dir: Path) -> str:
    """The run's recorded execution mode: lockstep if its input manifest says
    so or it holds a lockstep journal, else wall. A run recorded as lockstep
    without its journal then fails the journal gates."""
    if recorded_lockstep(log_dir) or (log_dir / JOURNAL_FILE).exists():
        return LOCKSTEP
    return WALL


def _run_gates(log_dir: Path, profile: str) -> RunGates:
    mode = execution_mode(log_dir)
    gates = _RUN_GATES.get((profile, mode))
    if gates is None:
        raise ValueError(f"gate profile {profile!r} has no {mode} run gates")
    return gates


#: Gate profile -> how the run's global assignment is established.
_ASSIGNMENT_EVIDENCE: dict[str, Callable[[Path, DemoMissionPlan], AssignmentEvidence]] = {
    CERTIFIED_PROFILE: global_assignment_evidence,
    OWNER_ONLY_SLICE_PROFILE: owner_only_assignment_evidence,
}


def analyze_run(
    log_dir: Path,
    *,
    scenario: DemoScenario,
    sys_ids: Sequence[int] | None = None,
    approvals: list[dict[str, JsonValue]] | None = None,
    limits: GateLimits = GateLimits(),
    navigation_speedup: float = NAVIGATION_SPEEDUP_OVERRIDE,
) -> RunReport:
    if not isinstance(limits, GateLimits):
        raise TypeError("limits must be GateLimits")
    if not isinstance(scenario, DemoScenario):
        raise TypeError("scenario must be a loaded DemoScenario")
    # Every caller (the CLI, the run comparison) gets the same refusal.
    require_recorded_scenario(log_dir, scenario)
    requested_speedup = finite_number("navigation_speedup", navigation_speedup)
    sim_speedup = PROFILE_SIM_SPEEDUP[scenario.gate_profile]
    if requested_speedup > 0.0 and scenario.gate_profile != CERTIFIED_PROFILE:
        # The slice's cadence is lockstep's factor; no override may relax it.
        raise ValueError(
            f"gate profile {scenario.gate_profile!r} has no navigation_speedup override"
        )
    supplied_ids = scenario.uav_ids(sys_ids) if sys_ids is not None else None
    supplied_approvals = (
        validate_approval_records(approvals) if approvals is not None else None
    )
    plan = load_resolved_plan(log_dir / "demo_mission_plan.json", scenario)
    command_bounds = load_command_bounds(log_dir / "demo_command_bounds.json")
    plan_ids = scenario.uav_ids(plan.sys_ids)
    if supplied_ids is not None and supplied_ids.values != plan_ids.values:
        raise ValueError(
            f"supplied sysids {supplied_ids.values} do not match resolved plan "
            f"{plan_ids.values}"
        )
    if tuple(item.sys_id for item in command_bounds.vehicles) != plan_ids.values:
        raise ValueError("command-bounds sysids do not match resolved plan order")
    approval_rows = (
        supplied_approvals
        if supplied_approvals is not None
        else load_approval_records(log_dir / "operator_approvals.json")
    )
    counts, approval_errors = approval_counts(approval_rows, plan)
    evidence = _ASSIGNMENT_EVIDENCE[scenario.gate_profile](log_dir, plan)
    assignments = evidence.assignments
    errors = approval_errors + evidence.errors + mission_assignment_errors(
        log_dir,
        plan,
        assignments,
        approval_rows,
    ) + _run_gates(log_dir, scenario.gate_profile)(log_dir, plan, approval_rows)
    reports = [
        analyze_vehicle(
            log_dir,
            vehicle.sys_id,
            vehicle.role,
            approval_count=counts[vehicle.sys_id],
            limits=limits,
            navigation_speedup=requested_speedup,
            launch_speedup=sim_speedup,
            command_bounds=command_bounds.for_vehicle(vehicle.sys_id),
            assignment_acked=evidence.acked,
            gate_profile=scenario.gate_profile,
        )
        for vehicle in plan.vehicles
    ]
    passed = (
        not errors
        and len(reports) == len(scenario.slots)
        and all(report.passed for report in reports)
    )
    return RunReport(
        passed=passed,
        log_dir=str(log_dir.resolve()),
        sys_ids=list(plan_ids.values),
        sim_speedup=sim_speedup,
        navigation_speedup_override=requested_speedup,
        approvals=approval_rows,
        global_assignments=assignments,
        vehicles=reports,
        errors=errors,
    )


__all__ = [
    "AssignmentEvidence",
    "analyze_run",
    "analyze_vehicle",
    "execution_mode",
    "approval_counts",
    "global_assignment_evidence",
    "mission_assignment_errors",
    "owner_only_assignment_evidence",
]
