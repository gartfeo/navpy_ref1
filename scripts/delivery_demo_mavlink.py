"""The hand demo's MAVLink record: where it goes, how to read it, what it says.

``delivery_demo`` names the file through ``TLOG_ENV``; ``swarm_run`` records
the router mirror there after launch verification (``mavlink_recorder``).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from scripts.mavlink_recorder import TLOG_ENV, status_path

TLOG_NAME = "mavlink.tlog"
#: What the Ready text's read command prints: texts and command answers.
READ_TYPES = "STATUSTEXT,COMMAND_ACK"


def powershell_command(*paths: object) -> str:
    """A PowerShell 5.1 line; literal-quoted paths keep spaces, ``$`` and backticks."""
    return "& " + " ".join("'" + str(path).replace("'", "''") + "'" for path in paths)


def tlog_path(run_dir: Path) -> Path:
    return run_dir / TLOG_NAME


def recorder_environment(run_dir: Path) -> dict[str, str]:
    """The variable that turns the record on, for this run only."""
    return {TLOG_ENV: str(tlog_path(run_dir).resolve())}


def ready_text(run_dir: Path) -> str:
    tlog = tlog_path(run_dir)
    # pymavlink installs mavlogdump.py beside the venv's python.exe.
    read = powershell_command(
        sys.executable, Path(sys.executable).with_name("mavlogdump.py"),
        "--show-source", "--types", READ_TYPES, tlog,
    )
    return (
        f"MAVLink record (router mirror after launch verification, not all ArduPilot "
        f"messages): {tlog}. Whether it started: {status_path(tlog)}.\n"
        f"Read it (a snapshot, also while recording): {read}\n"
        "Mission Planner: no port is exported in this demo (--loopback-only)."
    )


def record_lines(run_dir: Path) -> list[str]:
    """The record's status file, for ``delivery_demo_check``."""
    path = status_path(tlog_path(run_dir))
    if not path.is_file():
        return [f"MAVLink record: no {path.name} (a run from before the record, or it never started)"]
    status = json.loads(path.read_text(encoding="utf-8"))
    stop = status.get("stop_reason") or "none written (a forced stop such as gcs_stop.py, or still recording)"
    lines = [
        f"MAVLink record: started {status.get('started')}, {status.get('bytes')} bytes in "
        f"{status.get('records')} records, last write {status.get('last_write')}, stop: {stop}"
    ]
    if status.get("unframed_bytes"):
        lines.append(f"  {status['unframed_bytes']} received bytes were not whole MAVLink frames")
    if status.get("error"):
        lines.append(f"  error: {status['error']}")
    return lines
