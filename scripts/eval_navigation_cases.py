"""Compatibility facade for isolated final approach SITL evaluation."""

from __future__ import annotations

import argparse
import secrets
import subprocess
import sys
import time
from collections import deque
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

from pymavlink import mavutil


WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(WORKTREE / "src"))
sys.path.insert(0, str(WORKTREE))
sys.path.insert(0, str(SCRIPTS))

from gcs.backend import instance_registry as reg  # noqa: E402
from scripts import eval_certificate as cert  # noqa: E402

import eval_navigation_case as case_runner  # noqa: E402
import eval_navigation_cli as cli  # noqa: E402
import eval_navigation_evidence as evidence  # noqa: E402
import eval_navigation_launch as launch  # noqa: E402
import eval_navigation_logs as logs  # noqa: E402
import eval_navigation_matrix as matrix  # noqa: E402
import eval_navigation_mission as mission  # noqa: E402
import eval_navigation_navpy as navpy_child  # noqa: E402
import eval_navigation_runner as evaluator_runner  # noqa: E402
import eval_navigation_scoring as scoring  # noqa: E402
import eval_navigation_telemetry as telemetry  # noqa: E402
import eval_nav_origin as nav_origin  # noqa: E402
import eval_navigation_vehicle_config as vehicle_config  # noqa: E402
from eval_navigation_case_ports import CaseProcessPorts  # noqa: E402
from eval_navigation_models import (  # noqa: E402
    ClosestApproach,
    EvidenceGateResult,
    LaunchVerdict,
    MatrixCase,
    MissionItem,
    PositionSample,
    PositionStreamAnchor,
    SelectionEvidence,
    PoiExpectation,
    PoiLocation,
)


DEFAULT_PYTHON = cli.DEFAULT_PYTHON
DEFAULT_MAX_ATTEMPTS = cli.DEFAULT_MAX_ATTEMPTS
DEFAULT_MAX_DISTANCE_M = cli.DEFAULT_MAX_DISTANCE_M
EARTH_RADIUS_M = scoring.EARTH_RADIUS_M
COORDINATE_SCORE_RATE_HZ = scoring.COORDINATE_SCORE_RATE_HZ
COORDINATE_SCORE_MAX_SOURCE_DT_S = scoring.COORDINATE_SCORE_MAX_SOURCE_DT_S
COORDINATE_SCORE_MAX_SPATIAL_GAP_M = scoring.COORDINATE_SCORE_MAX_SPATIAL_GAP_M
COORDINATE_SCORE_MIN_SAMPLES = scoring.COORDINATE_SCORE_MIN_SAMPLES
COORDINATE_SCORE_REORDER_WINDOW_S = scoring.COORDINATE_SCORE_REORDER_WINDOW_S
SCORING_INTERVAL_SAMPLE_MAX_AGE_S = telemetry.SCORING_INTERVAL_SAMPLE_MAX_AGE_S
POI_SNAP_BIND_TIMEOUT_S = evidence.POI_SNAP_BIND_TIMEOUT_S
NAVPY_LOG_SUBDIR = logs.NAVPY_LOG_SUBDIR
LAUNCH_VERDICT_TIMEOUT_S = launch.LAUNCH_VERDICT_TIMEOUT_S
LAUNCHER_LOG_TAIL_BYTES = launch.LAUNCHER_LOG_TAIL_BYTES
_POI_SNAP_MISSING = evidence._POI_SNAP_MISSING


CoordinateScorer = scoring.CoordinateScorer
parse_float_list = matrix.parse_float_list
parse_int_list = matrix.parse_int_list
parse_poi_alts = matrix.parse_poi_alts
parse_navigation_speedups = matrix.parse_navigation_speedups
matrix_cases = matrix.matrix_cases
horizontal_distance_m = scoring.horizontal_distance_m
_local_vector = scoring._local_vector
_closest_on_segment = scoring._closest_on_segment
_same_position = scoring._same_position
mission_item_from_message = mission.mission_item_from_message
resolve_poi_expectation = mission.resolve_poi_expectation
download_mission = mission.download_mission
resolve_home_abs_alt_m = mission.resolve_home_abs_alt_m
parse_selection_evidence = evidence.parse_selection_evidence
poi_episode_binding_error = evidence.poi_episode_binding_error
await_poi_snap_binding = evidence.await_poi_snap_binding
selected_location_from_evidence = evidence.selected_location_from_evidence
validate_pre_snap_evidence = evidence.validate_pre_snap_evidence
wait_for_text = logs.wait_for_text
wait_for_ready_text = logs.wait_for_ready_text
navpy_log_paths = logs.navpy_log_paths
snap_from_compact = logs.snap_from_compact
parse_snap_summary = logs.parse_snap_summary
parse_snap_components = logs.parse_snap_components
sidecar_paths = logs.sidecar_paths
post_snap_poi_seen = logs.post_snap_poi_seen
_message_from_poi = telemetry.message_from_poi
_recv_poi_message = telemetry.recv_poi_message
command_long = telemetry.command_long
position_sample_from_message = telemetry.position_sample_from_message
_position_stream_is_live = telemetry.position_stream_is_live
wait_for_heartbeat = telemetry.wait_for_heartbeat
request_coordinate_score_stream = telemetry.request_coordinate_score_stream
require_nav_solution = nav_origin.require_nav_solution
require_fleet_nav_solution = nav_origin.require_fleet_nav_solution
_drain_position_messages = telemetry.drain_position_messages
set_param = vehicle_config.set_param
read_param = vehicle_config.read_param
_param_value_row = vehicle_config.param_value_row
download_all_params = vehicle_config.download_all_params
_missing_param_indices = vehicle_config.missing_param_indices
certificate_invalid_reason_for = vehicle_config.certificate_invalid_reason_for
_identity_fingerprint = vehicle_config.identity_fingerprint
launcher_failure_hint = launch.launcher_failure_hint
build_parser = cli.build_parser
parse_args = cli.parse_args
write_csv = evaluator_runner.write_csv
_stat_text = evaluator_runner._stat_text
_terminate_child = navpy_child.terminate_child


def certificate_identity(
    master: Any,
    *,
    mission: Sequence[MissionItem],
    sysid: int,
    enabled: bool,
) -> dict[str, Any] | None:
    return vehicle_config.certificate_identity(
        master,
        mission=mission,
        sysid=sysid,
        enabled=enabled,
        worktree=WORKTREE,
    )


def stop_own_stack(python: Path, *, chat: int | None = None) -> None:
    launch.stop_own_stack(python, worktree=WORKTREE, chat=chat)


def build_swarm_command(
    python: Path,
    speedup: float,
    *,
    launch_token: str | None = None,
) -> list[str]:
    return launch.build_swarm_command(
        python,
        speedup,
        worktree=WORKTREE,
        launch_token=launch_token,
    )


def wait_for_eval_chat(
    process: subprocess.Popen[Any],
    timeout_s: float,
    *,
    speedup: float | None = None,
    stderr_log: Path | None = None,
    launch_token: str | None = None,
) -> LaunchVerdict:
    return launch.wait_for_eval_chat(
        process,
        timeout_s,
        registry=reg,
        worktree=WORKTREE,
        speedup=speedup,
        stderr_log=stderr_log,
        launch_token=launch_token,
        failure_hint=launcher_failure_hint,
    )


def start_swarm(
    python: Path,
    run_case_dir: Path,
    speedup: float,
) -> tuple[subprocess.Popen[Any], LaunchVerdict]:
    return launch.start_swarm(
        python,
        run_case_dir,
        speedup,
        worktree=WORKTREE,
        command_builder=build_swarm_command,
        wait_for_chat=wait_for_eval_chat,
    )


def build_navpy_command(
    python: Path,
    *,
    sysid: int,
    nav_device: str,
    speedup: float,
    navigation_speedup: float,
    poi_wp: int,
    poi_rel_alt_m: float,
    args: argparse.Namespace,
) -> list[str]:
    return navpy_child.build_navpy_command(
        python,
        sysid=sysid,
        nav_device=nav_device,
        speedup=speedup,
        navigation_speedup=navigation_speedup,
        poi_wp=poi_wp,
        poi_rel_alt_m=poi_rel_alt_m,
        args=args,
    )


def launch_navpy(
    python: Path,
    run_case_dir: Path,
    *,
    sysid: int,
    nav_device: str,
    speedup: float,
    navigation_speedup: float,
    poi_wp: int,
    poi_rel_alt_m: float,
    args: argparse.Namespace,
) -> subprocess.Popen[Any]:
    return navpy_child.launch_navpy(
        python,
        run_case_dir,
        worktree=WORKTREE,
        sysid=sysid,
        nav_device=nav_device,
        speedup=speedup,
        navigation_speedup=navigation_speedup,
        poi_wp=poi_wp,
        poi_rel_alt_m=poi_rel_alt_m,
        args=args,
        command_builder=build_navpy_command,
    )


def run_case(
    index: int,
    case: MatrixCase,
    attempt: int,
    run_dir: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    return case_runner.run_case(
        index,
        case,
        attempt,
        run_dir,
        args,
        worktree=WORKTREE,
        process_ports=CaseProcessPorts(
            stop_stack=stop_own_stack,
            start_swarm=start_swarm,
            wait_for_heartbeat=wait_for_heartbeat,
            request_coordinate_stream=request_coordinate_score_stream,
            launch_navpy=launch_navpy,
            terminate_child=_terminate_child,
        ),
    )


def main(argv: Sequence[str] | None = None) -> int:
    return evaluator_runner.main(
        argv,
        run_case_fn=run_case,
        worktree=WORKTREE,
    )


if __name__ == "__main__":
    raise SystemExit(main())
