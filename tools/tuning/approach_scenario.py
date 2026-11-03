"""The approach runner's side of a run's scenario (hitl_final_approach.md section 9, stage 1 step 14).

The runner never imports the scenario service or scoring: it runs each in its own process and reads only the one JSON
line it prints, which never holds a pose. ``public`` gets the seed's public half (scenario_public.py): a dry run before
the run folder exists, then the run's scenario folder, whose scenario.json must match. ``verify`` gets scoring's
verdict once every process has stopped. The private files under the folder's PRIVATE_DIR are only passed on by path
(Unreal's) or left unread.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Optional

from tools.hitl import scenario_public

SCENARIO_DIR = "scenario"
SERVICE_TIMEOUT_S = 60.0        # my value: the service only reads the series and writes a few small files
MODULES = {"prepare": "tools.hitl.scenario_service", "verify": "tools.hitl.scenario_scoring"}


def service(command: str, *options: str) -> dict:
    """The printed line of the service (prepare) or scoring (verify), each in its own process. A refused prepare
    ends the run before it starts; a verify that prints no verdict is a failed one."""
    argv = [sys.executable, "-s", "-m", MODULES[command], *(("prepare",) if command == "prepare" else ()), *options]
    done = subprocess.run(argv, capture_output=True, text=True, timeout=SERVICE_TIMEOUT_S, check=False)
    lines = done.stdout.strip().splitlines()
    if command == "prepare" and done.returncode != 0:
        raise SystemExit(f"error: scenario service {command}: {(lines or [done.stderr.strip()])[-1]}")
    try:
        return json.loads(lines[-1])
    except (IndexError, ValueError):
        return {"reproduced": False, "problems": [f"scenario {command} printed no verdict"]}


def public(series: str, index: int, scenario_dir: Optional[Path] = None) -> dict:
    """The seed's public half: a dry run, or the scenario folder the service prepares, which must match."""
    where = ("--out", str(scenario_dir)) if scenario_dir else ("--dry-run",)
    half = service("prepare", "--series", series, "--index", str(index), *where)
    scenario_public.validate_public(half)
    if scenario_dir is not None and scenario_public.read_public(scenario_dir) != half:
        raise SystemExit("error: the scenario folder's public half is not the one the service printed")
    return half


def verify(scenario_dir: Path, unreal_log: Path) -> dict:
    return service("verify", "--scenario", str(scenario_dir), "--unreal-log", str(unreal_log))


def reproduced(verdict: dict, public: dict) -> bool:
    """Scoring's verdict holds for this run only when it names the run's own public scene."""
    return verdict.get("reproduced") is True and verdict.get("scene_hash") == public["scene_hash"]


def private_folder(run_dir: Path) -> Path:
    """The run's private scenario files: the runner leaves them unread (scoring checks them)."""
    return Path(run_dir) / SCENARIO_DIR / scenario_public.PRIVATE_DIR


def unreal_file(run_dir: Path) -> Path:
    """Unreal's private file, passed to it by path only."""
    return scenario_public.consumer_file(Path(run_dir) / SCENARIO_DIR, "unreal")


__all__ = ["MODULES", "SCENARIO_DIR", "SERVICE_TIMEOUT_S", "private_folder", "public", "reproduced", "service",
           "unreal_file", "verify"]
