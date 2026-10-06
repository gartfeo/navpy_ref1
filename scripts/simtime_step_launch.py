"""Wait for the experiment's supervisor to release startup serialization."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import time

from swarm_run_verification_model import VERIFY_TIMEOUT, PARAM_TIMEOUT, BOOT_EVIDENCE_TIMEOUT


def require_base_interpreter() -> None:
    if sys.platform == "win32" and sys.prefix != sys.base_prefix:
        raise ValueError("run this experiment with base Python, not a Windows venv redirector")


def _owner(path: Path) -> str | None:
    try:
        fields = path.read_text().split()
        return fields[0] if fields else ""
    except FileNotFoundError:
        return None


def wait_launch_unlock(process: subprocess.Popen, path: Path) -> dict:
    # Existing verification budgets plus bounded teardown overhead. We await
    # lock release, not process exit: the router can outlive an invalid SITL.
    began = time.monotonic()
    deadline = began + VERIFY_TIMEOUT + PARAM_TIMEOUT + BOOT_EVIDENCE_TIMEOUT + 10
    observed_own_lock = False
    while time.monotonic() < deadline:
        owner = _owner(path)
        if owner == "":
            time.sleep(0.1)  # another supervisor may be creating its lock
            continue
        if owner != str(process.pid):
            return {"supervisor_pid": process.pid, "observed_own_lock": observed_own_lock,
                    "wait_s": time.monotonic() - began, "owner_after": owner}
        observed_own_lock = True
        if process.poll() is not None:
            if _owner(path) == owner:
                raise RuntimeError("experiment supervisor died holding its launch lock")
            continue  # release/exit raced with the first read
        time.sleep(0.1)
    raise TimeoutError("experiment supervisor did not release its launch lock")
