"""Thin public/CLI facade for the exact three-UAV GCS demo evaluator."""

from __future__ import annotations

import sys
from pathlib import Path


_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for _import_root in (_REPOSITORY_ROOT, _REPOSITORY_ROOT / "src"):
    _import_path = str(_import_root)
    if _import_path not in sys.path:
        sys.path.insert(0, _import_path)

from scripts.eval_gcs_demo_analysis import analyze_run, analyze_vehicle
from scripts.eval_gcs_demo_api import FileTail, GcsApi, TelemetryEventStream
from scripts.eval_gcs_demo_cli import main, parse_args, report_dict
from scripts.eval_gcs_demo_config import (
    aas_params_for,
    build_environment,
    build_settings,
    full_param_changes,
    launch_command,
    stop_command,
)
from scripts.eval_gcs_demo_constants import (
    BACKEND_LOG,
    EVALUATOR_LOCK_PATH,
    EXPECTED_UAV_COUNT,
    FORBIDDEN_VISION_PROFILE,
    GCS_LAUNCH,
    GCS_STOP,
    NAVIGATION_SPEEDUP_OVERRIDE,
    ROOT,
    SIM_SPEEDUP,
    TERMINAL_ROLL_LIMIT_DEG,
    VISION_MOUNT_PITCH_DEG,
    VISION_PROFILE,
)
from scripts.eval_gcs_demo_mission import (
    all_have_snap as _all_have_snap,
    approve_requests_until_snap as _approve_real_requests_until_snap,
    run_mission as _run_mission,
)
from scripts.eval_gcs_demo_models import (
    EpisodeMetrics,
    GateLimits,
    MissionArtifacts,
    RegressionError,
    RunReport,
    VehicleReport,
)
from scripts.eval_gcs_demo_process import (
    exclusive_evaluator_lock,
    launch_stack as _launch_stack,
    stop_stack as _stop_stack,
    wait_until as _wait_until,
    with_owned_stack as _with_owned_stack,
)
from scripts.eval_gcs_demo_runtime import execute_live_run
from scripts.eval_gcs_demo_stack_ops import (
    configure_vehicle_params as _configure_vehicle_params,
    restart_companions as _restart_companions,
)


_report_dict = report_dict


__all__ = [
    "BACKEND_LOG",
    "EVALUATOR_LOCK_PATH",
    "EXPECTED_UAV_COUNT",
    "EpisodeMetrics",
    "FORBIDDEN_VISION_PROFILE",
    "FileTail",
    "GCS_LAUNCH",
    "GCS_STOP",
    "NAVIGATION_SPEEDUP_OVERRIDE",
    "GateLimits",
    "GcsApi",
    "MissionArtifacts",
    "ROOT",
    "RegressionError",
    "RunReport",
    "SIM_SPEEDUP",
    "TERMINAL_ROLL_LIMIT_DEG",
    "TelemetryEventStream",
    "VISION_MOUNT_PITCH_DEG",
    "VISION_PROFILE",
    "VehicleReport",
    "aas_params_for",
    "analyze_run",
    "analyze_vehicle",
    "build_environment",
    "build_settings",
    "execute_live_run",
    "exclusive_evaluator_lock",
    "full_param_changes",
    "launch_command",
    "main",
    "parse_args",
    "report_dict",
    "stop_command",
]


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RegressionError as error:
        print(f"REGRESSION SETUP FAILED: {error}", file=sys.stderr)
        raise SystemExit(2)
