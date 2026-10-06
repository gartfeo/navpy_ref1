"""One-stack execution of the normal three-UAV ideal-vision workflow."""

from __future__ import annotations

import json
import time
from copy import deepcopy
from pathlib import Path

from navpy.exception_groups import BaseExceptionGroup
from scripts.eval_gcs_demo_command_bounds import persist_command_bounds
from scripts.eval_gcs_demo_companion_ready import (
    capture_post_restart_status,
    restart_configured_companions_ready,
)
from scripts.eval_gcs_demo_config import (
    aas_params_for,
    build_environment,
    build_settings,
)
from scripts.eval_gcs_demo_mission import (
    approve_requests_until_snap,
    prepare_container_launch,
    resolve_live_plan,
)
from scripts.eval_gcs_demo_models import (
    MissionArtifacts,
    RegressionError,
    StackContext,
    finite_number,
)
from scripts.eval_gcs_demo_process import with_owned_stack
from scripts.eval_gcs_demo_stack_ops import (
    configure_vehicle_params,
    start_esp32_simulator,
    verify_aas_profile,
    wait_for_companions,
    wait_for_missions,
    wait_for_vehicles,
)
from scripts.eval_gcs_demo_upload import (
    capture_mission_assignments,
    capture_nav_start_bindings,
    mission_assignment_digest,
    shifted_mission_assignments,
    upload_mission_assignments,
)
from scripts.eval_gcs_demo_runtime import free_tcp_port
from scripts.eval_certificate_source_time import SOURCE_TIME_FLUSH_GRACE_S


IDEAL_PROFILE = "ideal_360"


def build_ideal_settings(esp32_port: int) -> dict:
    settings = deepcopy(build_settings(esp32_port))
    settings["camera"] = {
        "vision_profile": IDEAL_PROFILE,
        "vision_device": IDEAL_PROFILE,
        "vision_zoom": "1",
    }
    return settings


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def run_ideal_mission(
    context: StackContext,
    run_dir: Path,
    timeout_s: float,
) -> MissionArtifacts:
    """Upload a changed plan while companions run, then press normal START."""
    wait_for_vehicles(context, timeout_s)
    wait_for_companions(context, timeout_s)
    original_assignments = capture_mission_assignments(context)
    shifted_assignments = shifted_mission_assignments(original_assignments)
    original_digest = mission_assignment_digest(original_assignments)
    shifted_digest = mission_assignment_digest(shifted_assignments)
    if original_digest == shifted_digest:
        raise RegressionError("shifted mission digest did not change")
    _write_json(
        run_dir / "mission_upload_evidence.json",
        {
            "workflow": [
                "companions_running",
                "mission_upload",
                "parameter_write",
                "restart_ready",
                "start",
            ],
            "required_companion_restart_count": 1,
            "original_sha256": original_digest,
            "uploaded_sha256": shifted_digest,
            "assignments": shifted_assignments,
        },
    )
    mission_error: BaseException | None = None
    artifacts: MissionArtifacts | None = None
    try:
        upload_mission_assignments(context, shifted_assignments)
        owner_sys_id = context.sys_ids[0]
        nav_start_bindings = capture_nav_start_bindings(
            context,
            {
                sys_id: int(
                    aas_params_for(sys_id, owner_sys_id)["nav_last_wp"]
                )
                for sys_id in context.sys_ids
            },
        )
        command_bounds = configure_vehicle_params(context, timeout_s)
        verify_aas_profile(context)
        restart_instances = restart_configured_companions_ready(
            context,
            timeout_s,
            nav_start_bindings,
        )
        post_restart_status = capture_post_restart_status(
            context,
            restart_instances,
        )
        _write_json(
            run_dir / "mission_upload_evidence.json",
            {
                "workflow": [
                    "companions_running",
                    "mission_upload",
                    "parameter_write",
                    "restart_ready",
                    "start",
                ],
                "companion_restart_count": 1,
                "original_sha256": original_digest,
                "uploaded_sha256": shifted_digest,
                "assignments": shifted_assignments,
                "nav_start_bindings": nav_start_bindings,
                "restart_instances": restart_instances,
                "post_restart_status": post_restart_status,
            },
        )
        start_esp32_simulator(context)
        wait_for_missions(context, timeout_s)
        plan = resolve_live_plan(context, run_dir)
        if plan.sys_ids != context.sys_ids:
            raise RegressionError("resolved ideal plan sysids changed before launch")
        persist_command_bounds(
            command_bounds,
            run_dir / "demo_command_bounds.json",
        )
        prepare_container_launch(context, min(timeout_s, 180.0))
        approvals = approve_requests_until_snap(
            context,
            plan,
            run_dir,
            timeout_s,
            0.0,
        )
        _write_json(run_dir / "operator_approvals.json", {"approvals": approvals})
        time.sleep(SOURCE_TIME_FLUSH_GRACE_S)
        context.backend_tail.read_new()
        artifacts = MissionArtifacts(
            context.sys_ids,
            approvals,
            context.backend_tail.text,
            str((run_dir / "demo_mission_plan.json").resolve()),
        )
    except BaseException as error:
        mission_error = error
    restore_error: BaseException | None = None
    try:
        upload_mission_assignments(context, original_assignments)
    except BaseException as error:
        restore_error = error
    failures = [
        error
        for error in (mission_error, restore_error)
        if error is not None
    ]
    if len(failures) == 1:
        raise failures[0]
    if failures:
        raise BaseExceptionGroup(
            "ideal mission failed and original mission restoration failed",
            failures,
        )
    if artifacts is None:
        raise RegressionError("ideal mission returned no artifacts")
    return artifacts


def execute_ideal_live_run(
    run_dir: Path,
    timeout_s: float,
) -> MissionArtifacts:
    """Execute in the fresh evidence directory already claimed by the caller."""
    timeout = finite_number("ideal live-run timeout", timeout_s)
    if timeout <= 0.0:
        raise ValueError("ideal live-run timeout must be positive")
    if not run_dir.is_dir():
        raise RegressionError("ideal live run requires a caller-owned directory")
    settings_path = run_dir / "gcs_settings.json"
    _write_json(settings_path, build_ideal_settings(free_tcp_port()))
    environment = build_environment(settings_path, run_dir)
    context_holder: list[StackContext] = []

    def action(context: StackContext) -> MissionArtifacts:
        context_holder[:] = [context]
        if context.launch_evidence is None:
            raise RegressionError("exact stack has no verified launch evidence")
        _write_json(
            run_dir / "stack_launch_evidence.json",
            context.launch_evidence,
        )
        return run_ideal_mission(context, run_dir, timeout)

    try:
        return with_owned_stack(environment, timeout, action)
    finally:
        if context_holder:
            context = context_holder[-1]
            try:
                context.backend_tail.read_new()
            except OSError:
                pass
            (run_dir / "gcs_backend.log").write_text(
                context.backend_tail.text,
                encoding="utf-8",
            )


__all__ = [
    "build_ideal_settings",
    "execute_ideal_live_run",
    "run_ideal_mission",
]
