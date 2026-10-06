"""Three-UAV live certificate for direct SIYI geo tracking and zoom."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

from gcs.backend import instance_ports as ip
from scripts import eval_direct_pixel_pn as one
from scripts import eval_direct_pixel_pn_three_uav as three
from scripts.eval_navigation_cases import (
    download_mission,
    require_nav_solution,
    resolve_home_abs_alt_m,
    resolve_target_expectation,
    set_param,
    stop_own_stack,
    wait_for_heartbeat,
)
from scripts.eval_sim_parameters import sim_parameters


def _parser() -> argparse.ArgumentParser:
    parser = one._parser(cruise_speedup=False)  # one clock per vehicle
    parser.set_defaults(speedups="10,1", repetitions=1, timeout=180.0)
    return parser


def _launch_child(
    python: Path,
    directory: Path,
    *,
    sys_id: int,
    target: object,
    scoring_start_seq: int,
    timeout_s: float,
) -> subprocess.Popen[bytes]:
    command = [
        str(python),
        str(one.SCRIPTS / "siyi_geo_track_child.py"),
        "--connection", ip.companion_device(sys_id),
        "--sysid", str(sys_id),
        "--target-lat", repr(target.lat_deg),
        "--target-lon", repr(target.lon_deg),
        "--target-alt", repr(target.abs_alt_m),
        "--engage-seq", str(scoring_start_seq),
        "--timeout", repr(timeout_s),
        "--result", str(directory / "result.json"),
        "--ready", str(directory / "ready.marker"),
    ]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(one.WORKTREE / "src")
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    environment["NAVPY_LOG_DIR"] = str(directory / "navpy-logs")
    return subprocess.Popen(
        command,
        cwd=one.WORKTREE,
        env=environment,
        stdout=(directory / "child.out.log").open("wb"),
        stderr=(directory / "child.err.log").open("wb"),
    )


def _collect(
    children: dict[int, subprocess.Popen[bytes]],
    directories: dict[int, Path],
    timeout_s: float,
) -> dict[int, dict[str, object]]:
    results: dict[int, dict[str, object]] = {}
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s and len(results) < len(children):
        for sys_id, child in children.items():
            result_path = directories[sys_id] / "result.json"
            if sys_id not in results and result_path.exists():
                results[sys_id] = json.loads(result_path.read_text(encoding="utf-8"))
            elif sys_id not in results and child.poll() is not None:
                raise RuntimeError(
                    f"SIYI geo child sysid={sys_id} exited code={child.returncode}"
                )
        time.sleep(0.02)
    missing = sorted(set(children) - set(results))
    if missing:
        raise TimeoutError(f"SIYI geo results timed out for sysids {missing}")
    return results


def _wait_ready(
    marker: Path,
    process: subprocess.Popen[bytes],
    timeout_s: float,
) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError(
                f"SIYI geo child exited before ready: code={process.returncode}"
            )
        if marker.exists():
            return
        time.sleep(0.05)
    raise TimeoutError("SIYI geo child did not become ready")


def _case(
    python: Path,
    root: Path,
    *,
    speedup: float,
    repetition: int,
    args: argparse.Namespace,
) -> dict[str, object]:
    directory = root / f"speed-{speedup:g}-run-{repetition}"
    directory.mkdir(parents=True)
    swarm = master = None
    children: dict[int, subprocess.Popen[bytes]] = {}
    chat = None
    try:
        stop_own_stack(python)
        swarm, launch = one._start_swarm(python, directory, speedup, instances=3)
        chat = launch.chat
        sys_ids = ip.sysids_for_chat(chat)
        master = wait_for_heartbeat(ip.monitor_device(chat), 120.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")
        targets: dict[int, object] = {}
        scoring_start_sequences: dict[int, int] = {}
        for sys_id in sys_ids:
            three._select(master, sys_id)
            mission = download_mission(master)
            home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
            targets[sys_id] = resolve_target_expectation(
                mission,
                target_wp=args.target_wp,
                target_rel_alt_m=args.target_alt,
                home_abs_alt_m=home_alt,
            ).location
            scoring_start_sequences[sys_id] = resolve_target_expectation(
                mission,
                target_wp=args.scoring_start_wp,
                target_rel_alt_m=args.target_alt,
                home_abs_alt_m=home_alt,
            ).mission_seq
            # The shared push list, not a stale private copy: this harness
            # accepts --sitl-param through the shared parser, so it must also
            # honour it (and the SIM_RATE_HZ default the scored runs fly).
            for name, value in sim_parameters(args):
                if not set_param(master, name, value):
                    raise RuntimeError(f"sysid={sys_id} parameter echo failed: {name}")
            # Absorb the EKF-origin wait before the timed children exist.
            require_nav_solution(master, sys_id)
        directories = {sys_id: directory / f"uav-{sys_id}" for sys_id in sys_ids}
        for child_dir in directories.values():
            child_dir.mkdir()
        children = {
            sys_id: _launch_child(
                python,
                directories[sys_id],
                sys_id=sys_id,
                target=targets[sys_id],
                scoring_start_seq=scoring_start_sequences[sys_id],
                timeout_s=args.timeout,
            )
            for sys_id in sys_ids
        }
        for sys_id, child in children.items():
            _wait_ready(directories[sys_id] / "ready.marker", child, 90.0)
        three._start_missions(master, sys_ids)
        results = _collect(children, directories, args.timeout)
        errors = [
            f"sysid={sys_id}: {results[sys_id]}"
            for sys_id in sys_ids
            if results[sys_id].get("passed") is not True
        ]
        verdict = {
            "passed": not errors,
            "errors": errors,
            "sitl_params": list(args.sitl_param or []),
            "sitl_params_pushed": [list(pair) for pair in sim_parameters(args)],
            "vehicles": {str(sys_id): results[sys_id] for sys_id in sys_ids},
        }
        (directory / "verdict.json").write_text(
            json.dumps(verdict, indent=2), encoding="utf-8"
        )
        return verdict
    finally:
        if master is not None:
            master.close()
        for child in children.values():
            one._terminate(child)
        try:
            stop_own_stack(python, chat=chat)
        except Exception:
            pass
        one._terminate(swarm)


def main() -> int:
    args = _parser().parse_args()
    root = one.WORKTREE / ".sitl-runs" / (
        f"siyi-geo-track-3uav-{time.strftime('%Y%m%d-%H%M%S')}"
    )
    root.mkdir(parents=True)
    results = []
    for speedup in one._speeds(args.speedups):
        for repetition in range(1, args.repetitions + 1):
            try:
                result = _case(
                    args.python.resolve(),
                    root,
                    speedup=speedup,
                    repetition=repetition,
                    args=args,
                )
            except Exception as error:
                result = {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
            result.update({"speedup": speedup, "repetition": repetition})
            results.append(result)
            print("PASS" if result["passed"] else "FAIL", result, flush=True)
    summary = {"passed": all(row["passed"] for row in results), "results": results}
    (root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"ARTIFACT {root}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
