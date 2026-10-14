"""Strict, compact acceptance gates for the ideal three-UAV workflow."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from scripts.eval_gcs_demo_analysis import (
    approval_counts,
    global_assignment_evidence,
    mission_assignment_errors,
)
from scripts.eval_gcs_demo_audit import validate_approval_records
from scripts.eval_gcs_demo_constants import SIM_SPEEDUP
from scripts.eval_gcs_demo_models import RunReport
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario, load_resolved_plan
from scripts.eval_ideal_three_uav_source_time import load_source_time_evidence
from scripts.eval_ideal_three_uav_vehicle import analyze_ideal_vehicle


def analyze_ideal_run(
    log_dir: Path,
    *,
    scenario: DemoScenario,
    sys_ids: Sequence[int] | None = None,
    approvals: list[dict[str, JsonValue]] | None = None,
) -> RunReport:
    plan = load_resolved_plan(log_dir / "demo_mission_plan.json", scenario)
    plan_ids = scenario.uav_ids(plan.sys_ids)
    if sys_ids is not None:
        supplied = scenario.uav_ids(sys_ids)
        if supplied.values != plan_ids.values:
            raise ValueError("supplied sysids do not match the resolved plan")
    approval_rows = validate_approval_records(
        approvals
        if approvals is not None
        else _load_approvals(log_dir / "operator_approvals.json")
    )
    counts, approval_errors = approval_counts(approval_rows, plan)
    evidence = global_assignment_evidence(log_dir, plan)
    assignments = evidence.assignments
    source_time = load_source_time_evidence(log_dir, plan_ids.values, scenario)
    errors = approval_errors + evidence.errors + mission_assignment_errors(
        log_dir,
        plan,
        assignments,
        approval_rows,
    ) + source_time.errors
    reports = [
        analyze_ideal_vehicle(
            log_dir,
            vehicle.sys_id,
            vehicle.role,
            approval_count=counts[vehicle.sys_id],
            source_metrics=source_time.metrics_by_sys_id.get(vehicle.sys_id),
            source_errors=source_time.errors_by_sys_id.get(vehicle.sys_id, ()),
        )
        for vehicle in plan.vehicles
    ]
    return RunReport(
        passed=not errors and all(report.passed for report in reports),
        log_dir=str(log_dir.resolve()),
        sys_ids=list(plan_ids.values),
        sim_speedup=SIM_SPEEDUP,
        navigation_speedup_override=0.0,
        approvals=approval_rows,
        global_assignments=assignments,
        vehicles=reports,
        errors=errors,
    )


def _load_approvals(path: Path) -> list[dict[str, JsonValue]]:
    import json

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not isinstance(payload.get("approvals"), list):
        raise ValueError(f"invalid approval artifact {path}")
    return payload["approvals"]


__all__ = ["analyze_ideal_run", "analyze_ideal_vehicle"]
