"""Resolved-plan-driven analysis for the exact three-UAV demo."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from scripts.eval_gcs_demo_audit import (
    load_approval_records,
    validate_approval_records,
)
from scripts.eval_gcs_demo_assignment_audit import derive_global_assignment_map
from scripts.eval_gcs_demo_command_bounds import load_command_bounds
from scripts.eval_gcs_demo_constants import NAVIGATION_SPEEDUP_OVERRIDE, SIM_SPEEDUP
from scripts.eval_gcs_demo_models import (
    GateLimits,
    RunReport,
    ThreeUavIds,
    finite_number,
)
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoMissionPlan, load_resolved_plan
from scripts.eval_gcs_demo_poi_binding import final_approach_poi_binding_errors
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
    ids = ThreeUavIds.from_values(plan.sys_ids)
    expected = set(ids.values)
    observed = {int(item["sys_id"]) for item in approvals}
    if observed - expected:
        errors.append(f"approval artifact contains unknown sysids {sorted(observed - expected)}")
    counts = {
        sys_id: sum(item["sys_id"] == sys_id for item in approvals)
        for sys_id in ids.values
    }
    for sys_id, count in counts.items():
        if count != 1:
            errors.append(
                f"vehicle {sys_id} expected exactly one audited approval, found {count}"
            )
    return counts, errors


def global_assignment_evidence(
    log_dir: Path,
    plan: DemoMissionPlan,
) -> tuple[dict[int, int], list[str]]:
    try:
        backend_log = (log_dir / "gcs_backend.log").read_text(
            encoding="utf-8",
            errors="strict",
        )
        assignments = derive_global_assignment_map(backend_log, plan)
    except (OSError, UnicodeError, TypeError, ValueError) as error:
        return {}, [f"invalid global assignment evidence: {error}"]
    return assignments, []


def analyze_run(
    log_dir: Path,
    *,
    sys_ids: Sequence[int] | None = None,
    approvals: list[dict[str, JsonValue]] | None = None,
    limits: GateLimits = GateLimits(),
    navigation_speedup: float = NAVIGATION_SPEEDUP_OVERRIDE,
) -> RunReport:
    if not isinstance(limits, GateLimits):
        raise TypeError("limits must be GateLimits")
    requested_speedup = finite_number("navigation_speedup", navigation_speedup)
    supplied_ids = ThreeUavIds.from_values(sys_ids) if sys_ids is not None else None
    supplied_approvals = (
        validate_approval_records(approvals) if approvals is not None else None
    )
    plan = load_resolved_plan(log_dir / "demo_mission_plan.json")
    command_bounds = load_command_bounds(log_dir / "demo_command_bounds.json")
    plan_ids = ThreeUavIds.from_values(plan.sys_ids)
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
    assignments, assignment_errors = global_assignment_evidence(log_dir, plan)
    errors = approval_errors + assignment_errors + mission_assignment_errors(
        log_dir,
        plan,
        assignments,
        approval_rows,
    )
    reports = [
        analyze_vehicle(
            log_dir,
            vehicle.sys_id,
            vehicle.role,
            approval_count=counts[vehicle.sys_id],
            limits=limits,
            navigation_speedup=requested_speedup,
            command_bounds=command_bounds.for_vehicle(vehicle.sys_id),
        )
        for vehicle in plan.vehicles
    ]
    passed = not errors and len(reports) == 3 and all(report.passed for report in reports)
    return RunReport(
        passed=passed,
        log_dir=str(log_dir.resolve()),
        sys_ids=list(plan_ids.values),
        sim_speedup=SIM_SPEEDUP,
        navigation_speedup_override=requested_speedup,
        approvals=approval_rows,
        global_assignments=assignments,
        vehicles=reports,
        errors=errors,
    )


__all__ = [
    "analyze_run",
    "analyze_vehicle",
    "approval_counts",
    "global_assignment_evidence",
    "mission_assignment_errors",
]
