"""Case directory, SITL, vehicle configuration, and mission startup."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from gcs.backend import instance_ports as ip
from pymavlink import mavutil

from scripts import eval_certificate as cert

from eval_navigation_case_ports import CaseProcessPorts
from eval_navigation_case_state import CaseState
from eval_navigation_logs import navpy_log_paths, wait_for_ready_text
from eval_navigation_mission import (
    download_mission,
    resolve_home_abs_alt_m,
    resolve_poi_expectation,
)
from eval_navigation_models import MatrixCase, MissionItem
from eval_navigation_telemetry import (
    command_long,
)
from eval_nav_origin import require_nav_solution
from eval_navigation_vehicle_config import (
    certificate_identity,
    read_param,
    set_param,
)


def prepare_case(
    state: CaseState,
    case: MatrixCase,
    args: argparse.Namespace,
    *,
    worktree: Path,
    process_ports: CaseProcessPorts,
) -> None:
    """Acquire every resource and start AUTO mission execution."""
    _launch_stack(state, case, args, process_ports=process_ports)
    processes = state.processes
    assert processes.master is not None and processes.sysid is not None
    mission = download_mission(processes.master)
    home_abs_alt_m = resolve_home_abs_alt_m(
        processes.master,
        timeout_s=args.heartbeat_timeout,
    )
    state.evidence.home_abs_alt_m = home_abs_alt_m
    state.evidence.expectation = resolve_poi_expectation(
        mission,
        poi_wp=args.poi_wp,
        poi_rel_alt_m=state.poi_rel_alt_m,
        home_abs_alt_m=home_abs_alt_m,
    )
    required, optional, certificate_readback = _configure_vehicle(
        processes.master,
        case,
        args,
        certificate=state.certificate,
    )
    state.evidence.identity = certificate_identity(
        processes.master,
        mission=mission,
        sysid=processes.sysid,
        enabled=state.certificate,
        worktree=worktree,
    )
    _write_metadata(
        state,
        case,
        args,
        mission=mission,
        required=required,
        optional=optional,
        certificate_readback=certificate_readback,
    )
    _start_navpy_mission(state, case, args, process_ports=process_ports)


def _launch_stack(
    state: CaseState,
    case: MatrixCase,
    args: argparse.Namespace,
    *,
    process_ports: CaseProcessPorts,
) -> None:
    process_ports.stop_stack(args.python)
    process, verdict = process_ports.start_swarm(
        args.python,
        state.paths.case_dir,
        case.speedup,
    )
    state.processes.swarm = process
    state.processes.launch_verdict = verdict
    state.processes.chat = verdict.chat
    state.processes.sysid = ip.sysids_for_chat(verdict.chat)[0]
    sysid = state.processes.sysid
    state.paths.compact, state.paths.navigation = navpy_log_paths(
        state.paths.case_dir, sysid
    )
    control_device = ip.monitor_device(verdict.chat)
    state.processes.master = process_ports.wait_for_heartbeat(
        control_device, args.heartbeat_timeout
    )
    if state.processes.master is None:
        raise RuntimeError(
            f"no heartbeat on isolated eval device {control_device}"
        )
    acknowledged = process_ports.request_coordinate_stream(
        state.processes.master
    )
    state.evidence.coordinate_stream_acknowledged = acknowledged
    if not acknowledged:
        raise RuntimeError(
            "GLOBAL_POSITION_INT 30 Hz stream request was not acknowledged"
        )


def _configure_vehicle(
    master: Any,
    case: MatrixCase,
    args: argparse.Namespace,
    *,
    certificate: bool,
) -> tuple[dict[str, bool], dict[str, bool], dict[str, Any]]:
    required = {
        "ARMING_CHECK": set_param(master, "ARMING_CHECK", 0),
        "SIM_WIND_SPD": set_param(master, "SIM_WIND_SPD", case.wind_speed_mps),
        "SIM_WIND_DIR": set_param(
            master, "SIM_WIND_DIR", case.wind_direction_deg
        ),
    }
    if certificate:
        for parameter, value in cert.CERTIFICATE_WIND_PARAMS:
            required[parameter] = set_param(master, parameter, value)
    optional: dict[str, bool] = {}
    if args.guid_options is not None:
        optional["GUID_OPTIONS"] = set_param(
            master,
            "GUID_OPTIONS",
            args.guid_options,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
    if args.stall_prevention is not None:
        optional["STALL_PREVENTION"] = set_param(
            master,
            "STALL_PREVENTION",
            args.stall_prevention,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
    failed = [name for name, success in required.items() if not success]
    if failed:
        raise RuntimeError(f"parameter echo failed: {','.join(failed)}")
    readback = _certificate_readback(master) if certificate else {}
    return required, optional, readback


def _certificate_readback(master: Any) -> dict[str, Any]:
    readback = {
        parameter: read_param(master, parameter)
        for parameter in cert.CERTIFICATE_READBACK_PARAMS
    }
    expected = dict(cert.CERTIFICATE_WIND_PARAMS)
    expected["SIM_SPEEDUP"] = float(cert.CERTIFICATE_SPEEDUP)
    wrong = [
        f"{parameter}={observed}"
        for parameter, observed in readback.items()
        if observed is None
        or abs(float(observed) - expected[parameter])
        > cert.CERTIFICATE_PARAM_TOLERANCE
    ]
    if wrong:
        raise RuntimeError(
            "certificate simulator parameters did not read back: "
            + ", ".join(sorted(wrong))
        )
    return readback


def _write_metadata(
    state: CaseState,
    case: MatrixCase,
    args: argparse.Namespace,
    *,
    mission: list[MissionItem],
    required: dict[str, bool],
    optional: dict[str, bool],
    certificate_readback: dict[str, Any],
) -> None:
    del mission
    identity = state.evidence.identity
    if identity is not None:
        (state.paths.case_dir / "identity.json").write_text(
            json.dumps(identity, indent=2, default=str), encoding="utf-8"
        )
    master = state.processes.master
    expectation = state.evidence.expectation
    verdict = state.processes.launch_verdict
    assert master is not None and expectation is not None
    observed_names = (
        "ROLL_LIMIT_DEG",
        "LEVEL_ROLL_LIMIT",
        "TKOFF_LVL_ALT",
        "AIRSPEED_CRUISE",
        "AIRSPEED_MIN",
        "ARSPD_FBW_MIN",
        "TRIM_THROTTLE",
        "NAVL1_PERIOD",
        "PTCH_LIM_MIN_DEG",
        "STALL_PREVENTION",
        "TECS_SINK_MAX",
    )
    metadata = {
        "case": asdict(case),
        "chat": state.processes.chat,
        "sysid": state.processes.sysid,
        "vision_profile": args.vision_profile,
        "detector_type": args.detector_type,
        "poi_expectation": asdict(expectation),
        "configured_poi_rel_alt_m": state.poi_rel_alt_m,
        "coordinate_stream_acknowledged": (
            state.evidence.coordinate_stream_acknowledged
        ),
        "required_param_results": required,
        "optional_param_results": optional,
        "observed_params": {
            name: read_param(master, name) for name in observed_names
        },
        "launch_requested_speedup": (
            verdict.requested_speedup if verdict else None
        ),
        "launch_measured_rates": verdict.measured_rates if verdict else {},
        "certificate_mode": state.certificate,
        "certificate_param_readback": certificate_readback,
    }
    (state.paths.case_dir / "case.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )


def _start_navpy_mission(
    state: CaseState,
    case: MatrixCase,
    args: argparse.Namespace,
    *,
    process_ports: CaseProcessPorts,
) -> None:
    processes = state.processes
    assert processes.sysid is not None and processes.chat is not None
    processes.navpy = process_ports.launch_navpy(
        args.python,
        state.paths.case_dir,
        sysid=processes.sysid,
        nav_device=ip.companion_device(processes.sysid),
        speedup=case.speedup,
        navigation_speedup=case.navigation_speedup,
        poi_wp=args.poi_wp,
        poi_rel_alt_m=state.poi_rel_alt_m,
        args=args,
    )
    if not wait_for_ready_text(state.paths.case_dir / "navpy.err.log", 90):
        raise RuntimeError("NavPy readiness text was not observed")
    master = processes.master
    mode_map = master.mode_mapping()
    auto_mode = mode_map.get("AUTO") if mode_map else None
    if auto_mode is None:
        raise RuntimeError("AUTO mode is unavailable")
    require_nav_solution(master)
    master.set_mode(auto_mode)
    time.sleep(1.0)
    command_long(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
    time.sleep(1.0)
    command_long(master, mavutil.mavlink.MAV_CMD_MISSION_START, 0.0, 0.0)
