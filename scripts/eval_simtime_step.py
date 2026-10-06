"""Run the identity-only SITL prototype through the owned evaluation launcher."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from gcs.backend import instance_ports as ip, instance_registry as registry  # noqa: E402
import swarm_run  # noqa: E402
from simtime_step_launch import require_base_interpreter, wait_launch_unlock  # noqa: E402
from simtime_step_peer import DELAYS_S, FAULTS  # noqa: E402
from simtime_step_failures import validate_failure  # noqa: E402
from simtime_step_disabled import observe_disabled  # noqa: E402


def wsl_path(path: Path) -> str:
    path = path.resolve()
    if not path.drive or len(path.drive) != 2:
        raise ValueError("experiment expects a Windows drive path")
    return f"/mnt/{path.drive[0].lower()}/" + path.as_posix()[3:]


def artifacts(operation: str, firmware: str, vehicle: int, case: Path,
              mode: str) -> dict:
    command = ["wsl.exe", "--exec", "python3",
               wsl_path(ROOT / "scripts/simtime_step_artifacts.py"), operation,
               "--root", firmware, "--vehicle", str(vehicle),
               "--case", wsl_path(case), "--mode", mode]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stderr or result.stdout)
    return json.loads(result.stdout)


def wait_marker(path: Path, process: subprocess.Popen, markers: tuple[str, ...],
                timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
        for marker in markers:
            if marker in text:
                return marker
        if process.poll() is not None:
            raise RuntimeError(f"process exited {process.returncode}: {text[-3000:]}")
        time.sleep(0.1)  # bounded process supervision, not simulation timing
    raise TimeoutError(f"missing {markers} in {path}")


def validate_evidence(case: Path, mode: str, expected_vehicle: int) -> dict:
    with (case / "navpy-step.csv").open(encoding="utf-8") as stream:
        metadata = next(stream).strip()
        rows = list(csv.DictReader(stream))
    if len(rows) != 1000:
        raise ValueError("incomplete AP window")
    previous = None
    for index, row in enumerate(rows, 1):
        if int(row["step"]) != index or row["before_us"] != row["after_us"]:
            raise ValueError("step sequence or frozen clock violation")
        if int(row["complete_us"]) < int(row["before_us"]):
            raise ValueError("control completed before its input boundary")
        if previous and (int(row["tick"]) != int(previous["tick"]) + 1
                         or int(row["before_us"]) <= int(previous["complete_us"])):
            raise ValueError("AP progression violation")
        if mode == "delayed" and int(row["wait_us"]) < int(
            DELAYS_S[(index - 1) % len(DELAYS_S)] * 1_000_000
        ):
            raise ValueError("declared host delay did not occur")
        previous = row
    fields = dict(item.split("=", 1) for item in metadata.split(",")[1:])
    if fields["handshake"] != str(int(mode != "observe")):
        raise ValueError("wrong AP experiment mode")
    if int(fields["vehicle"]) != expected_vehicle:
        raise ValueError("AP vehicle differs from the reserved instance")
    if mode != "observe":
        peer = json.loads((case / "peer.json").read_text())
        if peer["fault"] != "none" or peer["delayed"] != (mode == "delayed"):
            raise ValueError("peer used a different experimental condition")
        records = peer["records"]
        if len(records) != len(rows):
            raise ValueError("incomplete peer window")
        for row, record in zip(rows, records):
            if fields["boot"] != "".join(f"{part:016x}" for part in record["boot"]):
                raise ValueError("AP/peer boot disagreement")
            if int(fields["vehicle"]) != record["vehicle"]:
                raise ValueError("AP/peer vehicle disagreement")
            if (int(row["step"]), int(row["tick"]), int(row["before_us"])) != (
                record["step"], record["tick"], record["source_us"]
            ):
                raise ValueError("AP/peer identity disagreement")
    return {"rows": len(rows), "metadata": metadata,
            "first_us": int(rows[0]["before_us"]),
            "last_us": int(rows[-1]["before_us"]),
            "achieved_speed": (int(rows[-1]["before_us"]) - int(rows[0]["before_us"]))
                / (int(rows[-1]["wall_us"]) - int(rows[0]["wall_us"])),
            "max_wait_us": max(int(row["wait_us"]) for row in rows)}


def run_case(firmware: str, case: Path, speed: float, mode: str, fault: str) -> dict:
    case.mkdir()
    chat = swarm_run._resolve_chat(None, eval_mode=True)
    vehicle = ip.sysids_for_chat(chat)[0]
    supervisor = peer = None
    outcome = {"speed": speed, "mode": mode, "fault": fault, "vehicle": vehicle}
    streams = []
    try:
        paths = artifacts("prepare", firmware, vehicle, case, mode)
        if mode not in ("observe", "off"):
            peer_log = (case / "peer.log").open("wb")
            streams.append(peer_log)
            command = ["wsl.exe", "--exec", "python3",
                       wsl_path(ROOT / "scripts/simtime_step_peer.py"),
                       "--directory", paths["directory"], "--fault", fault,
                       "--result", wsl_path(case / "peer.json"),
                       "--pid-file", wsl_path(case / "peer.pid")]
            if mode == "delayed":
                command.append("--delayed")
            peer = subprocess.Popen(command, stdout=peer_log, stderr=subprocess.STDOUT)
            wait_marker(case / "peer.log", peer, ("NAVPY_STEP_PEER_READY",), 20)
        log = (case / "supervisor.log").open("wb")
        streams.append(log)
        command = [sys.executable, str(ROOT / "scripts/swarm_run.py"), "--eval",
                   "--single-boot",
                   "--instances", "1", "--dist", "0", "--speedup", str(speed),
                   "--sitl-root", firmware, "--sitl-binary", paths["wrapper"]]
        (case / "command.json").write_text(json.dumps(command, indent=2))
        supervisor = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        started = time.monotonic()
        markers = (("instances streaming telemetry at SIM_SPEEDUP=", "NAVPY_STEP_INVALID")
                   if mode == "off" else ("NAVPY_STEP_COMPLETE", "NAVPY_STEP_INVALID"))
        marker = wait_marker(case / "supervisor.log", supervisor, markers, 150)
        outcome["wall_s"] = time.monotonic() - started
        expected = (markers[0] if fault == "none" else "NAVPY_STEP_INVALID")
        if marker != expected:
            raise RuntimeError(f"expected {expected}, got {marker}")
        if fault == "none":
            wait_marker(case / "supervisor.log", supervisor,
                        ("instances streaming telemetry at SIM_SPEEDUP=",), 30)
        outcome["marker"] = marker
        if mode == "off":
            outcome |= observe_disabled(vehicle)
        if peer is not None:
            peer.wait(timeout=12)
            if peer.returncode:
                raise RuntimeError(f"peer exited {peer.returncode}")
    finally:
        # Use the directory's existing isolated teardown; never broad-kill.
        cleanup_errors = []
        try:
            if supervisor is not None:
                # Let startup unwind even when a peer/validation failure occurs.
                unlock = wait_launch_unlock(supervisor, registry._sitl_launch_lock_path())
                (case / "launch-unlock.json").write_text(json.dumps(unlock, indent=2))
        except Exception as error:
            cleanup_errors.append(str(error))
        try:
            stopped = subprocess.run([sys.executable, str(ROOT / "scripts/gcs_stop.py"),
                                      "--eval"], cwd=ROOT, capture_output=True,
                                     text=True, timeout=45)
            (case / "teardown.log").write_text(stopped.stdout + stopped.stderr)
            stopped.check_returncode()
            if supervisor is not None:
                supervisor.wait(timeout=10)
        except Exception as error:
            cleanup_errors.append(str(error))
        try:
            artifacts("stop-peer", firmware, vehicle, case, mode)
            if peer is not None:
                peer.wait(timeout=10)
        except Exception as error:
            cleanup_errors.append(str(error))
        for stream in streams:
            stream.close()
        try:
            collected = artifacts("collect", firmware, vehicle, case, mode)
            if (fault != "none" or mode == "off") and collected["collected"]:
                raise ValueError("invalid run produced complete evidence")
        except Exception as error:
            cleanup_errors.append(str(error))
        if cleanup_errors:
            raise RuntimeError("teardown incomplete: " + "; ".join(cleanup_errors))
    if fault == "none":
        if mode != "off":
            outcome |= validate_evidence(case, mode, vehicle)
        log_text = (case / "supervisor.log").read_text(encoding="utf-8", errors="replace")
        if mode == "off" and "NAVPY_STEP" in log_text:
            raise ValueError("default-off simulator activated the probe")
        attempts = re.findall(r"instances streaming telemetry.*?\(attempt (\d+)\)", log_text)
        if log_text.count("Starting sketch 'ArduPlane'") != 1 or attempts != ["1"]:
            raise ValueError("positive trial must verify exactly one simulator boot")
        outcome["launch_attempt"] = 1
    else:
        outcome |= validate_failure(case, fault)
    outcome["passed"] = True
    (case / "result.json").write_text(json.dumps(outcome, indent=2))
    return outcome


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--speedups", type=float, nargs="+", default=[1, 10])
    parser.add_argument("--modes", nargs="+", choices=("off", "observe", "immediate", "delayed"),
                        default=["observe", "immediate", "delayed"])
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--fault", choices=FAULTS, default="none")
    args = parser.parse_args()
    try:
        require_base_interpreter()
    except ValueError as error:
        parser.error(str(error))
    if "off" in args.modes and any(speed != 10 for speed in args.speedups):
        parser.error("the default-off smoke check supports only --speedups 10")
    if args.repetitions < 1 or any(not 0 < value <= 100 for value in args.speedups):
        parser.error("positive repetitions and speedups up to 100 are required")
    if args.fault != "none" and any(mode in ("observe", "off") for mode in args.modes):
        parser.error("fault injection requires a handshake mode")
    existing = registry.find_for_owner(registry.owner_for(str(ROOT)), label="sitl-eval")
    if existing and existing.get("sitl_pid"):
        raise RuntimeError("stop this directory's existing evaluation before the prototype")
    output = ROOT / ".sitl-runs" / ("step-handshake-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    output.mkdir(parents=True)
    print(f"RESULTS {output}", flush=True)
    results = []
    for speed in args.speedups:
        for repeat in range(1, args.repetitions + 1):
            for mode in args.modes:
                case = output / f"speed-{speed:g}-{mode}-{repeat}-{args.fault}"
                result = run_case(args.firmware_root, case, speed, mode, args.fault)
                results.append(result)
                print(json.dumps(result), flush=True)
                (output / "summary.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
