"""NavPy child command construction and lifecycle."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from pathlib import Path
from typing import Any, Callable

from scripts import eval_certificate as cert

from eval_navigation_logs import NAVPY_LOG_SUBDIR


def build_navpy_command(
    python: Path,
    *,
    sysid: int,
    nav_device: str,
    speedup: float,
    navigation_speedup: float,
    target_wp: int,
    target_rel_alt_m: float,
    args: argparse.Namespace,
) -> list[str]:
    """Build one headless final approach NavPy command."""
    del speedup
    return [
        str(python),
        "-m",
        "navpy.main",
        "-c",
        nav_device,
        "-ss",
        str(sysid),
        "-ll",
        "DEBUG",
        "-nt",
        args.network_type,
        "-lsd",
        "Vehicle",
        "--detector-type",
        args.detector_type,
        "--vision-profile",
        args.vision_profile,
        "-mwp",
        "2",
        "-ma",
        "-5",
        "-tct",
        "40",
        "-twps",
        f"{target_wp},",
        "-talt",
        str(int(round(target_rel_alt_m))),
        "-ut",
        "false",
        "-udt",
        "false",
        "--pitch-controller",
        args.pitch_controller,
        "-pld",
        "-1",
        "-plrd",
        "-1",
        "-pkp",
        "0.35",
        "-nos",
        "-gsu",
        f"{navigation_speedup:g}",
        "-dt",
        str(int(args.del_throttle)),
        "-da",
        str(int(args.del_angle)),
    ]


def launch_navpy(
    python: Path,
    run_case_dir: Path,
    *,
    worktree: Path,
    sysid: int,
    nav_device: str,
    speedup: float,
    navigation_speedup: float,
    target_wp: int,
    target_rel_alt_m: float,
    args: argparse.Namespace,
    command_builder: Callable[..., list[str]] = build_navpy_command,
) -> subprocess.Popen[Any]:
    """Launch NavPy with logs and source-time evidence isolated per case."""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(worktree / "src")
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    environment["NAVPY_LOG_DIR"] = str(run_case_dir / NAVPY_LOG_SUBDIR)
    environment[cert.SOURCE_TIME_ENV] = str(
        run_case_dir / cert.SOURCE_TIME_SUBDIR
    )
    command = command_builder(
        python,
        sysid=sysid,
        nav_device=nav_device,
        speedup=speedup,
        navigation_speedup=navigation_speedup,
        target_wp=target_wp,
        target_rel_alt_m=target_rel_alt_m,
        args=args,
    )
    (run_case_dir / "navpy.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    stdout_handle = (run_case_dir / "navpy.out.log").open("wb")
    stderr_handle = (run_case_dir / "navpy.err.log").open("wb")
    return subprocess.Popen(
        command,
        cwd=str(worktree),
        env=environment,
        stdout=stdout_handle,
        stderr=stderr_handle,
    )


def terminate_child(process: subprocess.Popen[Any] | None) -> None:
    """Terminate and reap one owned child process."""
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
