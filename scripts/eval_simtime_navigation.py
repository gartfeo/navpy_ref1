"""Run one opt-in synchronized flight through the isolated evaluation launcher."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

from eval_simtime_step import ROOT, artifacts, wait_marker, wsl_path
from gcs.backend import instance_ports as ip, instance_registry as registry
import swarm_run
from noise_isolation_profiles import log_operation
from simtime_step_launch import require_base_interpreter, wait_launch_unlock
from simtime_navigation_faults import FAULTS, StrayInjector, validate_failure
from simtime_navigation_identity import record_identity
from lockstep_evaluation_lifecycle import stop_owned_supervisor
from scripts.eval_gcs_demo_execution import exclusive_evaluator_lock


def run_case(args: argparse.Namespace, case: Path) -> dict:
    with exclusive_evaluator_lock(ROOT / '.sitl-runs/lockstep-evaluator.lock'):
        return _run_case(args, case)


def _run_case(args: argparse.Namespace, case: Path) -> dict:
    case.mkdir(parents=True)
    chat = swarm_run._resolve_chat(None, eval_mode=True)
    vehicle = ip.sysids_for_chat(chat)[0]
    supervisor = peer = None
    injector = None
    streams = []
    outcome = {"speedup": args.speedup, "delayed": args.delayed,
               "camera_hz": args.camera_hz, "vehicle": vehicle}
    mode = "navigation-unready" if args.fault == "logger" else "navigation"
    paths = artifacts("prepare", args.firmware_root, vehicle, case, mode)
    try:
        if args.fault == "none":
            log_operation("before", args.firmware_root, vehicle, case)
        record_identity(ROOT, args.firmware_root, case, args.peer_python)
        peer_log = (case / "peer.log").open("wb")
        streams.append(peer_log)
        command = ["wsl.exe", "--exec", args.peer_python,
                   wsl_path(ROOT / "scripts/simtime_navigation_peer.py"),
                   "--defaults", wsl_path(case / "navigation-defaults.parm"),
                   "--directory", paths["directory"], "--result", wsl_path(case / "peer.json"),
                   "--pid-file", wsl_path(case / "peer.pid"), "--speedup", str(args.speedup),
                   "--camera-hz", str(args.camera_hz), "--fault", args.fault]
        if getattr(args, "throttle_percent", None) is not None:
            command.extend(["--throttle-percent", str(args.throttle_percent)])
        if args.delayed:
            command.append("--delayed")
        (case / "peer-command.json").write_text(json.dumps(command))
        peer = subprocess.Popen(command, stdout=peer_log, stderr=subprocess.STDOUT)
        wait_marker(case / "peer.log", peer, ("NAVPY_NAVIGATION_PEER_READY",), 60)
        log = (case / "supervisor.log").open("wb")
        streams.append(log)
        command = [sys.executable, str(ROOT / "scripts/swarm_run.py"), "--eval",
                   "--single-boot", "--instances", "1", "--dist", "0",
                   "--speedup", str(args.speedup), "--sitl-root", args.firmware_root,
                   "--sitl-binary", paths["wrapper"], "--defaults", paths["defaults"]]
        (case / "command.json").write_text(json.dumps(command, indent=2))
        supervisor = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        if args.fault == "stray":
            wait_marker(case / "supervisor.log", supervisor, ("Starting sketch 'ArduPlane'",), 40)
            injector = StrayInjector(ip.monitor_port(chat), vehicle, case)
        marker = wait_marker(case / "supervisor.log", supervisor,
                             ("NAVPY_NAVIGATION_COMPLETE", "NAVPY_NAVIGATION_INVALID"), 180)
        expected = "NAVPY_NAVIGATION_COMPLETE" if args.fault == "none" else "NAVPY_NAVIGATION_INVALID"
        if marker != expected:
            raise RuntimeError(marker)
        peer.wait(timeout=15)
        if peer.returncode and args.fault == "none":
            raise RuntimeError(f"peer exited {peer.returncode}")
        if args.fault == "none":
            wait_marker(case / "supervisor.log", supervisor,
                        ("instances streaming telemetry at SIM_SPEEDUP=",), 30)
    finally:
        errors = []
        if injector is not None:
            try:
                injector.close()
            except Exception as error:
                errors.append(str(error))
        try:
            if supervisor is not None:
                unlock = wait_launch_unlock(supervisor, registry._sitl_launch_lock_path())
                (case / "launch-unlock.json").write_text(json.dumps(unlock))
        except Exception as error:
            errors.append(str(error))
        try:
            stop_owned_supervisor(ROOT, chat, supervisor, case / "teardown.log")
        except Exception as error:
            errors.append(str(error))
        try:
            artifacts("stop-peer", args.firmware_root, vehicle, case, "navigation")
            if peer is not None:
                peer.wait(timeout=10)
        except Exception as error:
            errors.append(str(error))
        for stream in streams:
            stream.close()
        try:
            artifacts("collect", args.firmware_root, vehicle, case, "navigation")
            if args.fault == "none" and (case / "logs-before.json").is_file():
                log_operation("after", args.firmware_root, vehicle, case)
        except Exception as error:
            errors.append(str(error))
        if errors:
            raise RuntimeError("teardown incomplete: " + "; ".join(errors))
    if args.fault != "none":
        outcome |= validate_failure(case, args.fault)
    else:
        outcome["transport_complete"] = True
    (case / "result.json").write_text(json.dumps(outcome, indent=2))
    return outcome


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    parser.add_argument("--speedup", type=float, default=10)
    parser.add_argument("--camera-hz", type=int, choices=(40, 50), default=40)
    parser.add_argument("--throttle-percent", type=float,
                        help="Approach throttle percent; -1 explicitly masks throttle, omitted uses trim fallback")
    parser.add_argument("--delayed", action="store_true")
    parser.add_argument("--fault", choices=FAULTS, default="none")
    args = parser.parse_args()
    require_base_interpreter()
    case = ROOT / ".sitl-runs" / f"navigation-step-{datetime.now():%Y%m%d-%H%M%S}"
    print(f"EVIDENCE {case}", flush=True)
    print(json.dumps(run_case(args, case)), flush=True)


if __name__ == "__main__":
    main()
