"""One owned launcher, one or three independently lockstepped simulator vehicles."""

from __future__ import annotations

import argparse
from datetime import datetime
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import uuid

from eval_simtime_step import ROOT, artifacts, wait_marker, wsl_path
from gcs.backend import instance_ports as ip, instance_registry as registry
from scripts.eval_gcs_demo_execution import exclusive_evaluator_lock
from lockstep_evaluation_lifecycle import stop_owned_supervisor, wait_for_all
from simtime_navigation_identity import record_identity
from simtime_step_launch import require_base_interpreter, wait_launch_unlock
from swarm_run_cli import _home_coords
import swarm_run
from noise_isolation_profiles import PROFILES, prepare_profile, log_operation

HOME = "40.3117414,44.4552111,1294.85,0"


def resolve_home(raw: str | None) -> str:
    """Keep the reference launch exact while validating explicit locations."""
    if raw is None:
        return HOME
    home = _home_coords(raw)
    if float(home.rsplit(",", 1)[1]) != 0:
        raise argparse.ArgumentTypeError("this SITL launcher requires heading 0")
    parts = home.split(",")
    if any(float(f"{float(value):.8f}") != float(value) for value in parts[:2]):
        raise argparse.ArgumentTypeError("this SITL launcher supports at most eight decimal places for latitude and longitude")
    return ",".join((*parts[:3], "0.0"))


def select_launch_location(args: argparse.Namespace) -> tuple[list[float] | None, str]:
    dock = getattr(args, "dock", None)
    if dock is not None and (len(dock) != 3 or not all(math.isfinite(x) for x in dock)
                             or not -90 <= dock[0] <= 90 or not -180 <= dock[1] <= 180):
        raise ValueError("dock requires finite latitude, longitude and absolute altitude")
    raw_home = getattr(args, "home", None)
    if raw_home is not None and dock is None:
        raise ValueError("explicit home requires --dock with a selected point")
    return dock, resolve_home(raw_home)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def driver_identity() -> dict[str, str]:
    paths = sorted((ROOT / 'scripts').glob('*.py'))
    paths += sorted((ROOT / 'src/gcs/backend').rglob('*.py'))
    # Offline validators do not execute in a flight and identify themselves in reports.
    excluded = {'compare_lockstep_fleet.py', 'compare_simtime_navigation.py',
                'compare_noise_matrix.py', 'read_noise_parameters.py', 'analyze_noise_profiles.py',
                'pose_rounding_evidence.py', 'pose_rounding_analysis.py', 'replay_pose_rounding.py',
                'pose_precision_evidence.py', 'pose_precision_analysis.py'}
    return {p.relative_to(ROOT).as_posix(): digest(p) for p in paths if p.name not in excluded}


def experiment_profile(args: argparse.Namespace, manifest: dict) -> tuple[str | None, str]:
    profile = getattr(args, "noise_profile", None)
    if profile is not None:
        if args.instances != 1 or profile not in PROFILES:
            raise ValueError("noise investigation requires one vehicle and a named profile")
        manifest["noise_profile"] = profile
    pose_capture = getattr(args, "pose_capture", None)
    if pose_capture is not None:
        if type(pose_capture) is not bool or profile != "noise-off-1000" or args.instances != 1:
            raise ValueError("pose experiment requires one noise-off-1000 vehicle")
        manifest["pose_capture"] = pose_capture
    artifact_mode = "navigation-pose" if pose_capture else "navigation"
    pose_source = getattr(args, "pose_source", None)
    if pose_source is not None:
        if pose_capture is not True or pose_source not in ("rounded", "precast"):
            raise ValueError("render pose source requires explicit pose capture")
        manifest["pose_source"] = pose_source
        artifact_mode += f"-{pose_source}"
    return profile, artifact_mode


def run(args: argparse.Namespace, directory: Path) -> dict:
    dock, home = select_launch_location(args)
    directory.mkdir(parents=True)
    manifest = dict(version=1, run_id=uuid.uuid4().hex, status="preparing",
                    instances=args.instances, speedup=args.speedup, delayed=args.delayed,
                    camera_hz=40, home=home, firmware_root=args.firmware_root,
                    driver_sha256=driver_identity())
    if dock is not None:
        manifest["dock"] = list(dock)
    manifest["approach_throttle_percent"] = getattr(args, "throttle_percent", None)
    profile, artifact_mode = experiment_profile(args, manifest)
    manifest_path = directory / "fleet.json"
    def save() -> None:
        manifest_path.write_text(json.dumps(manifest, indent=2))
    save()
    with exclusive_evaluator_lock(ROOT / ".sitl-runs/lockstep-evaluator.lock"):
        chat = swarm_run._resolve_chat(None, eval_mode=True)
        manifest.update(chat=chat, vehicles=ip.sysids_for_chat(chat)[:args.instances])
        entry = registry.get(chat)
        if entry and entry.get("sitl") and registry.pid_matches(
                entry.get("sitl_pid"), entry.get("sitl_pid_start")):
            manifest.update(status="rejected", error="existing active simulator")
            save()
            raise RuntimeError(manifest["error"])
        cases, peers, streams, paths = {}, {}, [], {}
        supervisor = None
        errors = []
        failure = None
        try:
            launch_hash = subprocess.run(["wsl.exe", "--exec", "sha256sum",
                "/home/gart/ardupilot/Tools/autotest/run_swarm.sh"], check=True,
                capture_output=True, text=True, timeout=20)
            manifest["launcher_sha256"] = launch_hash.stdout.split()[0]
            checked = subprocess.run(["wsl.exe", "--exec", "python3", "-c",
                "import sys,json;from pathlib import Path;r=Path(sys.argv[1]);"
                "print(json.dumps([str(r/str(i)) for i in (1,2,3) if (r/str(i)).exists()]))",
                args.firmware_root], check=True, capture_output=True, text=True, timeout=20)
            manifest["template_directories"] = json.loads(checked.stdout)
            if manifest["template_directories"]:
                raise RuntimeError("isolated firmware root contains launch templates")
            for vehicle in manifest["vehicles"]:
                case = directory / str(vehicle)
                case.mkdir()
                paths[vehicle] = artifacts("prepare", args.firmware_root, vehicle, case, artifact_mode)
                cases[vehicle] = case
                if profile is not None:
                    prepare_profile(case, args.firmware_root, profile)
                before = log_operation("before", args.firmware_root, vehicle, case)
                if before["directory"] != args.firmware_root + "/" + str(vehicle):
                    raise ValueError("evaluation requires a canonical firmware root")
                record_identity(ROOT, args.firmware_root, case, args.peer_python)
            first = cases[manifest["vehicles"][0]]
            for name in ("launch-probe.sh", "navigation-defaults.parm"):
                hashes = {digest(case / name) for case in cases.values()}
                if len(hashes) != 1:
                    raise RuntimeError(f"fleet differs in shared {name}")
            manifest.update(wrapper_sha256=digest(first / "launch-probe.sh"),
                            defaults_sha256=digest(first / "navigation-defaults.parm"),
                            shared_profile_vehicle=manifest["vehicles"][0])
            for vehicle, case in cases.items():
                stream = (case / "peer.log").open("wb")
                streams.append(stream)
                command = ["wsl.exe", "--exec", args.peer_python,
                           wsl_path(ROOT / "scripts/simtime_navigation_peer.py"),
                           "--defaults", wsl_path(case / "navigation-defaults.parm"),
                           "--directory", paths[vehicle]["directory"],
                           "--result", wsl_path(case / "peer.json"),
                           "--pid-file", wsl_path(case / "peer.pid"),
                           "--speedup", str(args.speedup), "--camera-hz", "40", "--fault", "none"]
                if getattr(args, "throttle_percent", None) is not None:
                    command.extend(["--throttle-percent", str(args.throttle_percent)])
                if dock is not None:
                    command.extend(["--dock", *(format(Decimal(str(value)), "f") for value in dock)])
                if args.delayed:
                    command.append("--delayed")
                (case / "peer-command.json").write_text(json.dumps(command))
                peers[vehicle] = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
                wait_marker(case / "peer.log", peers[vehicle], ("NAVPY_NAVIGATION_PEER_READY",), 60)
            shared = paths[manifest["shared_profile_vehicle"]]
            command = [sys.executable, str(ROOT / "scripts/swarm_run.py"), "--eval", "--single-boot",
                       "--instances", str(args.instances), "--dist", "0", "--home", manifest["home"],
                       "--speedup", str(args.speedup), "--sitl-root", args.firmware_root,
                       "--sitl-binary", shared["wrapper"], "--defaults", shared["defaults"]]
            (directory / "command.json").write_text(json.dumps(command, indent=2))
            stream = (directory / "supervisor.log").open("wb")
            streams.append(stream)
            supervisor = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            manifest.update(status="running", supervisor_pid=supervisor.pid)
            save()
            wait_for_all(directory / "supervisor.log", supervisor, args.instances)
            for peer in peers.values():
                peer.wait(timeout=15)
                if peer.returncode:
                    raise RuntimeError(f"peer exited {peer.returncode}")
            wait_marker(directory / "supervisor.log", supervisor,
                        ("instances streaming telemetry at SIM_SPEEDUP=",), 30)
        except BaseException as error:
            failure = error
            errors.append(f'{type(error).__name__}: {error}')
        finally:
            if supervisor is not None:
                try:
                    unlock = wait_launch_unlock(supervisor, registry._sitl_launch_lock_path())
                    (directory / "launch-unlock.json").write_text(json.dumps(unlock))
                except Exception as error:
                    errors.append(str(error))
                try:
                    stop_owned_supervisor(ROOT, chat, supervisor, directory / "teardown.log")
                except Exception as error:
                    errors.append(str(error))
            for vehicle, case in cases.items():
                try:
                    artifacts("stop-peer", args.firmware_root, vehicle, case, "navigation")
                    if vehicle in peers:
                        peers[vehicle].wait(timeout=10)
                except Exception as error:
                    errors.append(str(error))
                try:
                    collected = artifacts("collect", args.firmware_root, vehicle, case, artifact_mode)
                    if not collected.get('collected'):
                        errors.append(f'vehicle {vehicle}: control trace missing')
                    if not (case / 'peer.json').is_file():
                        errors.append(f'vehicle {vehicle}: peer trace missing')
                    if (case / "logs-before.json").is_file():
                        log_operation("after", args.firmware_root, vehicle, case)
                except Exception as error:
                    errors.append(str(error))
            for stream in streams:
                try:
                    stream.close()
                except Exception as error:
                    errors.append(str(error))
            manifest.update(status="rejected" if errors else "captured", errors=errors)
            manifest['supervisor_alive'] = supervisor is not None and supervisor.poll() is None
            if manifest['supervisor_alive']:
                errors.append(f'owned supervisor still alive: {supervisor.pid}; teardown incomplete')
                manifest['status'] = 'rejected'
            manifest["evidence_sha256"] = {
                path.relative_to(directory).as_posix(): digest(path)
                for path in directory.rglob("*")
                if path.is_file() and path != manifest_path and path.name != "previous-eeprom.bin"}
            save()
        if failure is not None and not isinstance(failure, Exception):
            raise failure
        if errors:
            raise RuntimeError("fleet rejected: " + "; ".join(errors))
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    parser.add_argument("--instances", type=int, choices=(1, 3), default=3)
    parser.add_argument("--speedup", type=float, choices=(1., 10.), default=10.)
    parser.add_argument("--home", type=resolve_home, metavar="LAT,LON,ALT_MSL,0",
                        help="Explicit launch point; requires --dock; heading 0; lat/lon at most 8 decimals")
    parser.add_argument("--dock", type=float, nargs=3, metavar=("LAT", "LON", "ALT_MSL"),
                        help="Explicit simulated dock position; altitude is absolute metres, not AGL")
    parser.add_argument("--throttle-percent", type=float,
                        help="Approach throttle percent; -1 explicitly masks throttle, omitted uses trim fallback")
    parser.add_argument("--delayed", action="store_true")
    parser.add_argument("--noise-profile", choices=tuple(PROFILES))
    pose = parser.add_mutually_exclusive_group()
    pose.add_argument("--pose-capture", dest="pose_capture", action="store_true", default=None)
    pose.add_argument("--pose-control", dest="pose_capture", action="store_false")
    parser.add_argument("--pose-source", choices=("rounded", "precast"))
    args = parser.parse_args()
    require_base_interpreter()
    directory = ROOT / ".sitl-runs" / f"lockstep-fleet-{datetime.now():%Y%m%d-%H%M%S-%f}"
    print(f"EVIDENCE {directory}", flush=True)
    result = run(args, directory)
    print(json.dumps({k: result[k] for k in ("run_id", "status", "vehicles", "errors")}), flush=True)


if __name__ == "__main__":
    main()
