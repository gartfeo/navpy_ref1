"""Start one NavPy inside its job before it runs (``bounded_cpu_share.md``).

NavPy starts suspended, joins its ``KILL_ON_JOB_CLOSE`` job and only then
resumes: the order Microsoft documents for ``AssignProcessToJobObject``. The
venv launcher therefore cannot start the real interpreter outside the job,
so a job kill and the CPU share's demotion reach every NavPy process.

A share holder needs that job. When it cannot be made, the holder gives up
its share first (demoted, then released) and this NavPy starts as
background; when the demotion fails, NavPy does not start. Elsewhere a
missing job is only logged, and NavPy runs on.

A NavPy beside the shares starts BELOW_NORMAL by its creation flag, so it
never raises itself to ABOVE_NORMAL, even when this backend could not run
as background.
"""

from __future__ import annotations

import logging
import subprocess
import sys
from collections.abc import Callable

from gcs.backend.cpu_share_holder import StackScheduling, stack_scheduling
from gcs.backend.cpu_share_native import BELOW_NORMAL_PRIORITY_CLASS
from gcs.backend.navpy_process_instance import JOBS_LOCK, _assign_job, _close_job, _create_job
from navpy.core.kernel.exception_groups import BaseExceptionGroup

CREATE_SUSPENDED = 0x00000004


def _resume(proc: subprocess.Popen[bytes]) -> None:
    """Resume the suspended process. ``Popen`` has closed its main thread's
    handle, so this resumes the process (``NtResumeProcess``, as psutil's
    ``Process.resume`` does on Windows)."""
    import ctypes
    from ctypes import wintypes

    ntdll = ctypes.WinDLL("ntdll")  # type: ignore[attr-defined]
    ntdll.NtResumeProcess.argtypes = [wintypes.HANDLE]
    ntdll.NtResumeProcess.restype = ctypes.c_long
    status = ntdll.NtResumeProcess(int(proc._handle))  # type: ignore[attr-defined]
    if status < 0:
        raise OSError(f"NtResumeProcess failed: NTSTATUS {status & 0xFFFFFFFF:#010x}")


def _discard(proc: subprocess.Popen[bytes]) -> None:
    """End a child that never ran (still suspended) and close its pipes."""
    proc.kill()
    proc.wait(timeout=5)
    for stream in (proc.stdin, proc.stdout):
        if stream is not None:
            stream.close()


def _creation_flags(kwargs: dict[str, object], scheduling: StackScheduling) -> int:
    flags = int(kwargs.get("creationflags", 0))  # type: ignore[call-overload]
    return flags if scheduling.holding else flags | BELOW_NORMAL_PRIORITY_CLASS


def _start_without_job(
    command: list[str],
    kwargs: dict[str, object],
    popen_factory: Callable[..., subprocess.Popen[bytes]],
    scheduling: StackScheduling,
    reason: str,
) -> subprocess.Popen[bytes]:
    """NavPy outside a job, only once this stack holds no share."""
    scheduling.give_up(reason)
    if scheduling.holding:
        raise OSError(f"NavPy not started: {reason} and the CPU share could not be given up")
    return popen_factory(command, **{**kwargs, "creationflags": _creation_flags(kwargs, scheduling)})


def spawn_in_job(
    command: list[str],
    kwargs: dict[str, object],
    *,
    popen_factory: Callable[..., subprocess.Popen[bytes]],
    sys_id: int,
    logger: logging.Logger,
    scheduling: StackScheduling = stack_scheduling,
) -> tuple[subprocess.Popen[bytes], int | None]:
    """The running NavPy and its job (None: it runs without one)."""
    if sys.platform != "win32":
        return popen_factory(command, **kwargs), None
    with JOBS_LOCK:
        try:
            job: int | None = _create_job()
        except OSError:
            logger.warning("Failed to create job object for sys_id=%d", sys_id)
            return _start_without_job(command, kwargs, popen_factory, scheduling,
                                      "job creation failed"), None
        flags = _creation_flags(kwargs, scheduling) | CREATE_SUSPENDED
        try:
            proc = popen_factory(command, **{**kwargs, "creationflags": flags})
        except BaseException as failure:
            try:
                _close_job(job)  # type: ignore[arg-type]
            except BaseException as cleanup_error:
                raise BaseExceptionGroup(
                    "NavPy spawn and job cleanup failed",
                    (failure, cleanup_error),
                ) from failure
            raise
        try:
            _assign_job(job, proc)  # type: ignore[arg-type]
        except Exception:
            logger.warning("Failed to assign process to job for sys_id=%d", sys_id)
            _close_job(job)  # type: ignore[arg-type]
            job = None
            if scheduling.holding:
                _discard(proc)
                return _start_without_job(command, kwargs, popen_factory, scheduling,
                                          "job assignment failed"), None
        try:
            _resume(proc)
        except BaseException:
            _discard(proc)
            if job is not None:
                _close_job(job)
            raise
        return proc, job


__all__ = ["CREATE_SUSPENDED", "spawn_in_job"]
