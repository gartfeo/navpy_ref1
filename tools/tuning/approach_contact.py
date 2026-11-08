"""The approach runner's contact wiring (hitl_final_approach.md section 9, steps 26 and 27).

The runner writes the run's contact link (tools/hitl/contact_service.py ``link``: a fresh key and two loopback
endpoints, in the private run folder), starts the contact service as its own process once Unreal has built the scene
(the service reads Unreal's readback of it), sends the service a second copy of the bridge's vehicle_state, and passes
the link to navpy (``--hitl-contact``). Everything stays on WSL's loopback, beside the bridge and navpy. The runner
itself never reads the service's private scenario file or its evidence, and never imports the service: it reaches it
only through its processes (``link`` and ``serve``), as it reaches the scenario service.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from tools.hitl.contact_loop import READY_LINE
from tools.tuning.approach_procs import STOP_SIGINT, ProcSpec

ROLE = "contact"
HOST = "127.0.0.1"
# My values: three free loopback UDP ports beside the vehicle_state port 17113 (none is used elsewhere in the chain).
SERVICE_PORT, NAVPY_PORT, POSE_PORT = 17120, 17121, 17122
LINK_FILE = "contact_link.json"         # the run's key and endpoints: private, in the run folder
EVIDENCE_FILE = "contact_evidence.jsonl"  # the service's own evidence (arms, refusals, contacts, the closest pass)
READY_TIMEOUT_S = 30.0                  # my value: two binds and one scene read
LINK_TIMEOUT_S = 60.0                   # my value: one record read and one small file write


def link(run_dir: Path, record_path: Path) -> Path:
    """The run's link file, written by the service's ``link`` command (refused outside the private folder or when one
    exists: one link per run); SystemExit when it refuses."""
    out = Path(run_dir) / LINK_FILE
    done = subprocess.run([sys.executable, "-s", "-m", "tools.hitl.contact_service", "link", "--run-record",
                           str(record_path), "--service", f"{HOST}:{SERVICE_PORT}", "--navpy", f"{HOST}:{NAVPY_PORT}",
                           "--out", str(out)], capture_output=True, text=True, timeout=LINK_TIMEOUT_S, check=False)
    if done.returncode != 0 or not out.is_file():
        raise SystemExit(f"error: the contact link was refused: {(done.stdout + done.stderr).strip()}")
    return out


def pose_copy() -> str:
    """The bridge's extra ``--vehicle-state-copy`` target: the service's pose socket."""
    return f"{HOST}:{POSE_PORT}"


def service(python: str, repo: Path, run_dir: Path, link_file: Path, scenario_dir: Path) -> ProcSpec:
    """The contact service; ready once both sockets are bound and the scene's dock was read."""
    argv = (python, "-s", "-m", "tools.hitl.contact_service", "serve", "--link", str(link_file),
            "--scenario", str(scenario_dir), "--poses", pose_copy(), "--evidence", str(Path(run_dir) / EVIDENCE_FILE))
    return ProcSpec(ROLE, argv, "^" + READY_LINE, READY_TIMEOUT_S, STOP_SIGINT, stop_wait_s=10.0,
                    ports=(("udp", SERVICE_PORT), ("udp", POSE_PORT)), cwd=str(repo),
                    env={"PYTHONPATH": f"{Path(repo) / 'src'}:{repo}", "PYTHONUNBUFFERED": "1"})


def navpy_args(link_file: Path) -> tuple[str, ...]:
    return ("--hitl-contact", str(link_file))


__all__ = ["EVIDENCE_FILE", "LINK_FILE", "NAVPY_PORT", "POSE_PORT", "ROLE", "SERVICE_PORT", "link", "navpy_args",
           "pose_copy", "service"]
