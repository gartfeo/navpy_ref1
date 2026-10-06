"""Child-process lifecycle for the single-UAV direct-pixel harness.

One concern: the navigation child as an OS process -- launching it with
its recorded command line, waiting for its readiness marker, and
terminating it (or any sibling process) without leaving zombies.  What
the child DOES is direct_pixel_pn_child.py's business.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from eval_navigation_models import TargetLocation  # noqa: E402
from pixel_pn_terminal_speed import TerminalSpeedPlan  # noqa: E402


def launch_child(
    python: Path,
    case_dir: Path,
    *,
    device: str,
    sysid: int,
    target: TargetLocation,
    scoring_start_seq: int,
    timeout_s: float,
    speed_plan: TerminalSpeedPlan | None = None,
    child_script: Path = SCRIPTS / "direct_pixel_pn_child.py",
) -> subprocess.Popen[bytes]:
    command = [
        str(python),
        str(child_script),
        "--connection",
        device,
        "--sysid",
        str(sysid),
        "--target-lat",
        repr(target.lat_deg),
        "--target-lon",
        repr(target.lon_deg),
        "--target-alt",
        repr(target.abs_alt_m),
        "--engage-seq",
        str(scoring_start_seq),
        "--timeout",
        repr(timeout_s),
        "--result",
        str(case_dir / "result.json"),
        "--engaged",
        str(case_dir / "engaged.marker"),
    ]
    command.extend(() if speed_plan is None else speed_plan.child_args())
    (case_dir / "child.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(WORKTREE / "src")
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    environment["NAVPY_LOG_DIR"] = str(case_dir / "navpy-logs")
    return subprocess.Popen(
        command,
        cwd=str(WORKTREE),
        env=environment,
        stdout=(case_dir / "child.out.log").open("wb"),
        stderr=(case_dir / "child.err.log").open("wb"),
    )


def wait_ready(
    path: Path,
    process: subprocess.Popen[bytes],
    timeout_s: float,
    marker: str = "DIRECT_PIXEL_READY",
) -> None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if process.poll() is not None:
            raise RuntimeError(
                f"direct pixel child exited code={process.returncode}"
            )
        if path.exists() and marker in path.read_text(
            encoding="utf-8", errors="replace"
        ):
            return
        time.sleep(0.05)
    raise TimeoutError("direct pixel child did not become ready")


def terminate(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


__all__ = ["launch_child", "terminate", "wait_ready"]
