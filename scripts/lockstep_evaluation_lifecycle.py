"""Owned process supervision for deterministic evaluation; never a sim clock."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import time

from gcs.backend import instance_registry as registry
from simtime_navigation_protocol import WINDOW_SIZE


def stop_owned_supervisor(root: Path, chat: int, supervisor: subprocess.Popen | None, log: Path) -> None:
    if supervisor is None:
        return
    entry = registry.get(chat)
    if entry is None:
        if supervisor.poll() is None:
            raise RuntimeError("live supervisor lost registry ownership")
        log.write_text("Supervisor already exited and retracted its owned stack.\n")
        return
    if entry.get("sitl_pid") != supervisor.pid:
        if not entry.get("sitl_pid") and supervisor.poll() is not None:
            log.write_text("Supervisor exited before claiming a simulator stack.\n")
            return
        raise RuntimeError("refusing teardown of another supervisor")
    stopped = subprocess.run([sys.executable, str(root / "scripts/gcs_stop.py"), "--eval"],
                             cwd=root, capture_output=True, text=True, timeout=45)
    log.write_text(stopped.stdout + stopped.stderr)
    stopped.check_returncode()
    supervisor.wait(timeout=10)


def completion_count(text: str, expected: int) -> bool:
    if "NAVPY_NAVIGATION_INVALID" in text:
        raise RuntimeError("simulator invalidated the run")
    count = text.count(f"NAVPY_NAVIGATION_COMPLETE count={WINDOW_SIZE}")
    if count > expected:
        raise RuntimeError("extra simulator completion")
    return count == expected


def wait_for_all(path: Path, supervisor: subprocess.Popen, expected: int, timeout: float = 180.) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        text = path.read_text(encoding="utf-8", errors="replace")
        if completion_count(text, expected):
            return
        if supervisor.poll() is not None:
            raise RuntimeError(f"supervisor exited before all completions: {text[-2500:]}")
        time.sleep(.1)  # process supervision only; no simulated event is scheduled here
    raise TimeoutError("incomplete fleet control windows")
