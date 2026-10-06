"""Top-level live-run orchestration for the exact three-UAV scenario."""

from __future__ import annotations

import json
import socket
from pathlib import Path

from scripts.eval_gcs_demo_audit import validate_approval_records
from scripts.eval_gcs_demo_config import build_environment, build_settings
from scripts.eval_gcs_demo_models import (
    MissionArtifacts,
    RegressionError,
    StackContext,
    finite_number,
)
from scripts.eval_gcs_demo_mission import run_mission
from scripts.eval_gcs_demo_process import with_owned_stack
from scripts.eval_gcs_demo_scenario import load_scenario_manifest
from scripts.eval_gcs_demo_stack_ops import configure_vehicle_params


def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def execute_live_run(
    run_dir: Path,
    timeout_s: float,
    confirmation_delay_s: float = 15.0,
    *,
    navigation_speedup: float = 0.0,
) -> MissionArtifacts:
    timeout = finite_number("live-run timeout", timeout_s)
    if timeout <= 0.0:
        raise ValueError("live-run timeout must be positive")
    delay = finite_number("confirmation delay", confirmation_delay_s)
    speedup = finite_number("navigation speedup", navigation_speedup)
    if not isinstance(run_dir, Path):
        raise TypeError("run_dir must be a Path")
    load_scenario_manifest()
    run_dir.mkdir(parents=True, exist_ok=True)
    if any(run_dir.glob("uav_*_navigation.log")):
        raise RegressionError(
            f"run directory already contains navigation logs: {run_dir}"
        )
    settings_path = run_dir / "gcs_settings.json"
    settings_path.write_text(
        json.dumps(build_settings(free_tcp_port()), indent=2),
        encoding="utf-8",
    )
    env = build_environment(
        settings_path,
        run_dir,
        navigation_speedup=speedup,
    )
    configured: set[tuple[int, int, int]] = set()
    mission: MissionArtifacts | None = None
    for _attempt in range(3):
        def action(context: StackContext) -> tuple[str, object]:
            if context.sys_ids not in configured:
                configure_vehicle_params(context, timeout)
                configured.add(context.sys_ids)
                return "configured", context.sys_ids
            return "mission", run_mission(context, run_dir, timeout, delay)

        kind, payload = with_owned_stack(env, timeout, action)
        if kind == "mission":
            if not isinstance(payload, MissionArtifacts):
                raise TypeError("mission action returned an invalid artifact")
            mission = payload
            break
    if mission is None:
        raise RegressionError(
            "the auto-allocated GCS sysid set changed after every "
            "configuration launch"
        )
    approvals = validate_approval_records(mission.approvals)
    (run_dir / "gcs_backend.log").write_text(
        mission.backend_log_text,
        encoding="utf-8",
    )
    (run_dir / "operator_approvals.json").write_text(
        json.dumps({"approvals": approvals}, indent=2),
        encoding="utf-8",
    )
    return mission


__all__ = ["execute_live_run", "free_tcp_port"]
