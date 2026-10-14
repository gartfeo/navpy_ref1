"""``gcs_stop`` for a lockstep slot: stop by ownership, then compare-release.

1. Mark the entry ``stopping`` under the registry lock: the child inventory
   is frozen, so a coordinator still spawning gets a refusal and kills its
   child before ``go``.
2. Stop every frozen child that is still the recorded process (local: pid and
   start time; WSL: pid, start ticks and cwd under the run root).
3. Stop the coordinator if it is still the recorded owner process.
4. Once every frozen child and the coordinator are confirmed gone (a wait on
   the process, or for WSL the helper's result or a fresh probe), release the entry if it still
   carries the owner identity read in step 1; a replacement claim is never
   touched. A child that will not go keeps the entry, so it stays stoppable.

This is also the recovery after an owner death: the entry outlives its owner
while a recorded child lives.
"""

from __future__ import annotations

import os
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from gcs.backend import instance_registry as reg
from gcs.backend import instance_registry_lockstep as lockstep

#: Wall budget for a killed process to be confirmed gone (host supervision).
EXIT_CONFIRM_WALL_BUDGET_S = 10.0
_POSIX_PROBE_S = 0.05
_SYNCHRONIZE = 0x00100000
_QUERY_LIMITED_INFORMATION = 0x1000
_WAIT_OBJECT_0 = 0x0
#: OpenProcess's error for a pid that names no process.
_ERROR_INVALID_PARAMETER = 87


class Win32Process:
    """The Win32 calls :func:`wait_gone` makes (a seam: tests stub it)."""

    def __init__(self) -> None:
        import ctypes

        self._ctypes = ctypes
        self._kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

    def open(self, pid: int) -> tuple[int, int]:
        """``(handle, 0)``, or ``(0, GetLastError())``."""
        handle = self._kernel32.OpenProcess(_SYNCHRONIZE | _QUERY_LIMITED_INFORMATION, False, pid)
        return (handle, 0) if handle else (0, self._ctypes.get_last_error())

    def start_time(self, handle: int) -> int | None:
        """The creation time of the process behind ``handle`` (the registry's stamp)."""
        from ctypes import wintypes

        times = [wintypes.FILETIME() for _ in range(4)]
        if not self._kernel32.GetProcessTimes(handle, *(self._ctypes.byref(t) for t in times)):
            return None
        return (times[0].dwHighDateTime << 32) | times[0].dwLowDateTime

    def wait(self, handle: int, timeout_ms: int) -> int:
        return self._kernel32.WaitForSingleObject(handle, timeout_ms)

    def close(self, handle: int) -> None:
        self._kernel32.CloseHandle(handle)


def wait_gone(pid: object, start: object, budget_s: float = EXIT_CONFIRM_WALL_BUDGET_S,
              *, win32: Win32Process | None = None) -> bool:
    """True only once ``pid`` is proven not to name the recorded process.

    Windows, fail-closed: gone when a valid handle's process has another
    start time (read through that handle: the pid was reused), on
    ``WAIT_OBJECT_0`` from a valid handle, or when ``OpenProcess`` reports no
    such pid and a fresh identity check (pid and start time) agrees. Any
    other failure, a failed start-time read included, is unconfirmed.
    """
    if os.name == "nt" or win32 is not None:
        return _win32_gone(int(pid), start, budget_s, win32 or Win32Process())
    deadline = time.monotonic() + budget_s
    while reg.pid_matches(pid, start):
        if time.monotonic() > deadline:
            return False
        time.sleep(_POSIX_PROBE_S)
    return True


def _win32_gone(pid: int, start: object, budget_s: float, win32: Win32Process) -> bool:
    handle, error = win32.open(pid)
    if not handle:
        return error == _ERROR_INVALID_PARAMETER and not reg.pid_matches(pid, start)
    try:
        started = win32.start_time(handle)
        if started is None:
            return False
        if start and started != start:
            return True  # another process has the pid: the recorded one is gone
        return win32.wait(handle, int(budget_s * 1000)) == _WAIT_OBJECT_0
    finally:
        win32.close(handle)


#: Helper results that prove the recorded WSL process is gone: it exited on
#: TERM, was already gone, or the pid now names another process. A sent KILL
#: ("killed") proves nothing until a fresh probe agrees.
WSL_GONE_RESULTS = frozenset({"stopped", "gone", "not-ours"})


def _wsl_module() -> Any:
    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.insert(0, root)
    from scripts import lockstep_wsl

    return lockstep_wsl


def _stop_wsl(child: dict[str, Any]) -> bool:
    """Stop one recorded WSL child; True only once it is confirmed gone."""
    lockstep_wsl = _wsl_module()
    identity = (int(child["wsl_pid"]), int(child["start_ticks"]), str(child["cwd"]), str(child["root"]))
    try:
        if lockstep_wsl.stop_recorded(*identity) in WSL_GONE_RESULTS:
            return True
        return not lockstep_wsl.recorded_alive(*identity)
    except Exception:  # unreachable WSL, a failed probe, a wsl.exe timeout: not confirmed
        return False


def _stop_local(process: dict[str, Any], kill: Callable[[object], None], budget_s: float) -> bool:
    """Kill a recorded local process if it is still the recorded one, then confirm it gone."""
    if reg.pid_matches(process.get("pid"), process.get("start")):
        kill(process["pid"])
    return wait_gone(process.get("pid"), process.get("start"), budget_s)


def _confirmed(stop: Callable[[dict[str, Any]], bool], child: dict[str, Any]) -> bool:
    """Any failure to stop or probe one child leaves it unconfirmed; teardown goes on."""
    try:
        return bool(stop(child))
    except Exception:
        return False


def stop_lockstep(
    chat: int,
    *,
    kill: Callable[[object], None],
    stop_wsl: Callable[[dict[str, Any]], bool] = _stop_wsl,
    report: Callable[[str], None] = print,
    confirm_budget_s: float = EXIT_CONFIRM_WALL_BUDGET_S,
) -> bool:
    """Stop slot ``chat`` by ownership; True if its entry was released."""
    frozen = lockstep.mark_stopping(chat)
    if frozen is None:
        report(f"chat {chat}: no lockstep entry (already stopped?)")
        return False
    state = frozen["lockstep"]
    gone = []
    for child in state["children"]:
        if child.get("kind") == "wsl":
            gone.append(_confirmed(stop_wsl, child))
            continue
        gone.append(_confirmed(lambda c: _stop_local(c, kill, confirm_budget_s), child))
    owner = state["owner"]
    owner_gone = _confirmed(lambda c: _stop_local(c, kill, confirm_budget_s), owner)
    report(f"chat {chat}: stopped {sum(gone)} of {len(gone)} recorded children; "
           f"coordinator {'gone' if owner_gone else 'still running'}")
    if not (all(gone) and owner_gone):
        report(f"chat {chat}: not confirmed gone; entry kept (run gcs_stop again)")
        return False
    released = lockstep.release_if_owner(chat, owner)
    report(f"chat {chat}: {'released slot' if released else 'slot already released or reclaimed'}")
    return released


__all__ = ["EXIT_CONFIRM_WALL_BUDGET_S", "stop_lockstep", "wait_gone"]
