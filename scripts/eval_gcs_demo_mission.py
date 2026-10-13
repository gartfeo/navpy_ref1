"""Resolved-plan mission launch and WebSocket confirmation workflow."""

from __future__ import annotations

from pathlib import Path

from scripts.eval_gcs_demo_command_bounds import persist_command_bounds
from scripts.eval_gcs_demo_confirmation import (
    ConfirmationRequest,
    ConfirmationTimeout,
    run_confirmation_workflow,
)
from scripts.eval_gcs_demo_confirmation_evidence import (
    persist_confirmation_failure as _persist_confirmation_failure,
)
from scripts.eval_gcs_demo_models import (
    MissionArtifacts,
    RegressionError,
    StackContext,
    finite_number,
)
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import (
    DemoMissionPlan,
    persist_resolved_plan,
    resolve_demo_plan,
)
from scripts.eval_gcs_demo_stack_ops import (
    configure_vehicle_params,
    start_esp32_simulator,
    verify_aas_profile,
    wait_for_missions,
    wait_for_vehicles,
)
from scripts.eval_gcs_demo_lifecycle import connect_vehicles, start_companions
from scripts.eval_gcs_demo_mission_upload import upload_scenario_missions


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise TypeError(f"{label} response must be an object")
    return value


def _navigation_text(context: StackContext, sys_id: int) -> str:
    """Every NavPy log line of *sysid* so far, through the context's port."""
    tail = context.navigation_tail(sys_id)
    tail.read_new()
    return tail.text


def _has_snap(context: StackContext, sys_id: int) -> bool:
    return "SNAP(VISION-NAV" in _navigation_text(context, sys_id)


def all_have_snap(context: StackContext) -> bool:
    return all(_has_snap(context, sys_id) for sys_id in context.sys_ids)


def final_approach_nav_started(context: StackContext, sys_id: int) -> bool:
    text = _navigation_text(context, sys_id)
    process_start = text.rfind("Heartbeat from system")
    current = text[process_start:] if process_start >= 0 else text
    return "INIT: NAV MODE" in current


def resolve_live_plan(context: StackContext, log_dir: Path) -> DemoMissionPlan:
    scenario = context.scenario
    ids = scenario.uav_ids(context.sys_ids)
    owner = scenario.owner_sys_id(ids)
    mission = context.api.get_json(f"/api/vehicles/{owner}/mission", timeout=90.0)
    plan = resolve_demo_plan(ids, _object(mission, "owner mission"), scenario)
    persist_resolved_plan(plan, log_dir / "demo_mission_plan.json", scenario)
    return plan


def prepare_container_launch(context: StackContext, timeout_s: float) -> None:
    timeout = finite_number("container preparation timeout", timeout_s)
    timer = context.clock
    deadline = timer.monotonic() + timeout
    last_detail: JsonValue = None
    while timer.monotonic() < deadline:
        status, raw = context.api.post_json(
            "/api/control/launch",
            {
                "sys_ids": list(context.sys_ids),
                "force": False,
                "pitot_covered": False,
            },
            expected=(200, 409),
            timeout=90.0,
        )
        body = _object(raw, "container launch")
        if status == 200:
            if body.get("status") != "container_prepared":
                raise RegressionError(f"unexpected container response: {body}")
            return
        last_detail = body
        timer.sleep(2.0)
    raise RegressionError(f"container launch never became ready: {last_detail}")


def trigger_all(context: StackContext) -> None:
    for sys_id in context.sys_ids:
        status, raw = context.api.post_json(
            "/api/control/launch/trigger",
            {"sys_id": sys_id},
        )
        body = _object(raw, "launch trigger")
        if status != 200 or body.get("status") != "triggered":
            raise RegressionError(f"vehicle {sys_id} trigger failed: {body}")


def approve_requests_until_snap(
    context: StackContext,
    plan: DemoMissionPlan,
    log_dir: Path,
    timeout_s: float,
    confirmation_delay_s: float,
) -> list[dict[str, JsonValue]]:
    vehicle_ids = set(context.sys_ids)
    vehicle_count = len(context.scenario.slots)
    if len(vehicle_ids) != vehicle_count:
        raise RegressionError(
            f"confirmation workflow requires {vehicle_count} unique vehicles, "
            "one per scenario slot"
        )

    def has_snap(sys_id: int) -> bool:
        return _has_snap(context, sys_id)

    observed_sys_ids: set[int] = set()
    attempted_sys_ids: set[int] = set()
    accepted_approvals: list[dict[str, JsonValue]] = []

    def on_request_observed(request: ConfirmationRequest) -> None:
        if request.sys_id in observed_sys_ids:
            raise RegressionError(
                "vehicle requested more than one confirmation round "
                f"at ({request.sys_id}, {request.local_task_id})"
            )
        observed_sys_ids.add(request.sys_id)

    def before_approve(request: ConfirmationRequest) -> None:
        if final_approach_nav_started(context, request.sys_id):
            raise RegressionError(
                f"vehicle {request.sys_id} entered NAV before delayed operator "
                f"approval for local task {request.local_task_id}"
            )

    def on_approval_attempted(request: ConfirmationRequest) -> None:
        attempted_sys_ids.add(request.sys_id)

    def on_approval_accepted(record: dict[str, JsonValue]) -> None:
        accepted_approvals.append(record)

    try:
        approvals = run_confirmation_workflow(
            event_stream=context.event_stream_factory(context.api.websocket_url),
            api=context.api,
            expected_sys_ids=vehicle_ids,
            trigger=lambda: trigger_all(context),
            stop_when=lambda: all_have_snap(context),
            requested_delay_s=confirmation_delay_s,
            timeout_s=timeout_s,
            clock=context.clock,
            on_request_observed=on_request_observed,
            before_approve=before_approve,
            on_approval_attempted=on_approval_attempted,
            on_approval_accepted=on_approval_accepted,
        )
    except ConfirmationTimeout as error:
        missing_snap, evidence_errors = _persist_confirmation_failure(
            context,
            log_dir,
            {request.sys_id for request in error.requests},
            attempted_sys_ids,
            error.approvals,
            status="timeout",
            reason=str(error),
            has_snap=has_snap,
        )
        if evidence_errors:
            raise RegressionError(
                f"{error}; missing SNAP sysids {missing_snap}; "
                f"confirmation evidence incomplete: {'; '.join(evidence_errors)}"
            ) from error
        raise ConfirmationTimeout(
            error.expected_sys_ids,
            error.seen,
            error.approvals,
            missing_snap_sysids=missing_snap,
        ) from error
    except Exception as error:
        missing_snap, evidence_errors = _persist_confirmation_failure(
            context,
            log_dir,
            observed_sys_ids,
            attempted_sys_ids,
            accepted_approvals,
            status="workflow_error",
            reason=f"{type(error).__name__}: {error}",
            has_snap=has_snap,
        )
        if evidence_errors:
            raise RegressionError(
                f"{type(error).__name__}: {error}; "
                f"request observed sysids {sorted(observed_sys_ids)}; "
                f"approval attempted sysids {sorted(attempted_sys_ids)}; "
                f"approval accepted sysids {sorted({int(row['sys_id']) for row in accepted_approvals})}; "
                f"missing SNAP sysids {missing_snap}; "
                f"confirmation evidence incomplete: {'; '.join(evidence_errors)}"
            ) from error
        raise
    approved_sysids = {item["sys_id"] for item in approvals}
    if (
        approved_sysids != vehicle_ids
        or len(approvals) != vehicle_count
    ):
        missing_snap, evidence_errors = _persist_confirmation_failure(
            context,
            log_dir,
            observed_sys_ids,
            attempted_sys_ids,
            approvals,
            status="approval_mismatch",
            reason="SNAP approvals are not exactly one local confirmation per vehicle",
            has_snap=has_snap,
        )
        detail = (
            "SNAP approvals are not exactly one local confirmation per vehicle: "
            f"approved sysids {sorted(approved_sysids)}; "
            f"missing confirmation request sysids {sorted(vehicle_ids - observed_sys_ids)}; "
            f"request seen but unapproved sysids {sorted(observed_sys_ids - approved_sysids)}; "
            f"missing SNAP sysids {missing_snap}"
        )
        if evidence_errors:
            detail += f"; confirmation evidence incomplete: {'; '.join(evidence_errors)}"
        raise RegressionError(detail)
    return approvals


def run_mission(
    context: StackContext,
    log_dir: Path,
    timeout_s: float,
    confirmation_delay_s: float,
) -> MissionArtifacts:
    connect_vehicles(context)
    wait_for_vehicles(context, timeout_s)
    command_bounds = configure_vehicle_params(context, timeout_s)
    upload_scenario_missions(context)
    verify_aas_profile(context)
    # Once, after configuration and upload: the companions read both at start.
    start_companions(context, timeout_s)
    start_esp32_simulator(context)
    wait_for_missions(context, timeout_s)
    plan = resolve_live_plan(context, log_dir)
    persist_command_bounds(
        command_bounds,
        log_dir / "demo_command_bounds.json",
    )
    if plan.sys_ids != context.sys_ids:
        raise RegressionError("resolved plan sysids changed before launch")
    prepare_container_launch(context, min(timeout_s, 180.0))
    approvals = approve_requests_until_snap(
        context,
        plan,
        log_dir,
        timeout_s,
        confirmation_delay_s,
    )
    return MissionArtifacts(
        context.sys_ids,
        approvals,
        len(context.scenario.slots),
        str((log_dir / "demo_mission_plan.json").resolve()),
    )


__all__ = [
    "all_have_snap",
    "approve_requests_until_snap",
    "prepare_container_launch",
    "resolve_live_plan",
    "run_mission",
    "final_approach_nav_started",
    "trigger_all",
]
