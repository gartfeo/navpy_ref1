"""State, output capture, and teardown for one NavPy subprocess."""

from __future__ import annotations

import logging
import subprocess
import sys
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.runtime_ready import parse_runtime_ready


_OUTPUT_BUFFER_SIZE = 100

if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x2000
    _JobObjectExtendedLimitInformation = 9

    class _JOBOBJECT_BASIC_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [
            ("PerProcessUserTimeLimit", wintypes.LARGE_INTEGER),
            ("PerJobUserTimeLimit", wintypes.LARGE_INTEGER),
            ("LimitFlags", wintypes.DWORD),
            ("MinimumWorkingSetSize", ctypes.c_size_t),
            ("MaximumWorkingSetSize", ctypes.c_size_t),
            ("ActiveProcessLimit", wintypes.DWORD),
            ("Affinity", ctypes.c_size_t),
            ("PriorityClass", wintypes.DWORD),
            ("SchedulingClass", wintypes.DWORD),
        ]

    class _IO_COUNTERS(ctypes.Structure):
        _fields_ = [
            ("ReadOperationCount", ctypes.c_uint64),
            ("WriteOperationCount", ctypes.c_uint64),
            ("OtherOperationCount", ctypes.c_uint64),
            ("ReadTransferCount", ctypes.c_uint64),
            ("WriteTransferCount", ctypes.c_uint64),
            ("OtherTransferCount", ctypes.c_uint64),
        ]

    class _JOBOBJECT_EXTENDED_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [
            ("BasicLimitInformation", _JOBOBJECT_BASIC_LIMIT_INFORMATION),
            ("IoInfo", _IO_COUNTERS),
            ("ProcessMemoryLimit", ctypes.c_size_t),
            ("JobMemoryLimit", ctypes.c_size_t),
            ("PeakProcessMemoryUsed", ctypes.c_size_t),
            ("PeakJobMemoryUsed", ctypes.c_size_t),
        ]


def _create_job() -> int:
    """Create a Windows Job Object with KILL_ON_JOB_CLOSE."""
    if sys.platform != "win32":
        raise OSError("Windows Job Objects are unavailable")
    handle = _kernel32.CreateJobObjectW(None, None)
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    info = _JOBOBJECT_EXTENDED_LIMIT_INFORMATION()
    info.BasicLimitInformation.LimitFlags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    if not _kernel32.SetInformationJobObject(
        handle,
        _JobObjectExtendedLimitInformation,
        ctypes.byref(info),
        ctypes.sizeof(info),
    ):
        _kernel32.CloseHandle(handle)
        raise ctypes.WinError(ctypes.get_last_error())
    return handle


def _assign_job(job_handle: int, proc: subprocess.Popen[bytes]) -> None:
    """Assign a subprocess to a Windows Job Object."""
    if sys.platform != "win32":
        raise OSError("Windows Job Objects are unavailable")
    if not _kernel32.AssignProcessToJobObject(job_handle, int(proc._handle)):
        raise ctypes.WinError(ctypes.get_last_error())


def _close_job(job_handle: int) -> None:
    if sys.platform == "win32":
        _kernel32.CloseHandle(job_handle)


def _terminate_job(job_handle: int) -> None:
    """Terminate all processes in a Windows Job Object and close it."""
    if sys.platform != "win32":
        raise OSError("Windows Job Objects are unavailable")
    _kernel32.TerminateJobObject(job_handle, 1)
    _kernel32.CloseHandle(job_handle)


@dataclass
class NavpyInstance:
    """Tracks one NavPy subprocess generation."""

    sys_id: int
    connection: str
    process: subprocess.Popen[bytes]
    started_at: float = field(default_factory=time.time)
    output: deque[str] = field(
        default_factory=lambda: deque(maxlen=_OUTPUT_BUFFER_SIZE)
    )
    ready_payload: dict[str, object] | None = None
    _ready_error: str | None = field(default=None, repr=False)
    _ready_or_exit: threading.Event = field(
        default_factory=threading.Event,
        repr=False,
    )
    _reader_thread: threading.Thread | None = field(default=None, repr=False)
    _job_handle: int | None = field(default=None, repr=False)


def read_navpy_output(
    inst: NavpyInstance,
    readiness_validator: (
        Callable[[NavpyInstance, dict[str, object]], str | None] | None
    ) = None,
) -> None:
    """Read stdout and publish the exact runtime-readiness marker."""
    stdout = inst.process.stdout
    try:
        if stdout is None:
            return
        for raw_line in iter(stdout.readline, b""):
            line = raw_line.decode("utf-8", errors="replace").rstrip("\n\r")
            inst.output.append(line)
            payload = parse_runtime_ready(line)
            if payload is not None:
                validation_error: str | None = None
                if readiness_validator is not None:
                    try:
                        validation_error = readiness_validator(inst, payload)
                    except Exception as error:
                        validation_error = (
                            "readiness validation failed: "
                            f"{type(error).__name__}: {error}"
                        )
                if validation_error is not None:
                    inst._ready_error = validation_error
                    inst.output.append(
                        f"NavPy readiness rejected: {validation_error}"
                    )
                    inst._ready_or_exit.set()
                    break
                inst.ready_payload = payload
                inst._ready_or_exit.set()
    except Exception as error:
        if inst.ready_payload is None and inst._ready_error is None:
            inst._ready_error = (
                f"output reader failed: {type(error).__name__}: {error}"
            )
            inst.output.append(inst._ready_error)
    finally:
        if stdout is not None:
            try:
                stdout.close()
            except Exception:
                pass
        inst._ready_or_exit.set()


def snapshot_navpy_instance(inst: NavpyInstance) -> dict[str, object]:
    running = inst.process.poll() is None
    return {
        "sys_id": inst.sys_id,
        "connection": inst.connection,
        "pid": inst.process.pid,
        "running": running,
        "exit_code": None if running else inst.process.poll(),
        "uptime_s": round(time.time() - inst.started_at, 1) if running else 0,
        "ready": running and inst.ready_payload is not None,
        "runtime": (
            dict(inst.ready_payload) if inst.ready_payload is not None else None
        ),
        "startup_error": inst._ready_error,
        "log": list(inst.output),
    }


def terminate_navpy_instance(
    inst: NavpyInstance,
    logger: logging.Logger,
) -> None:
    proc = inst.process
    job = inst._job_handle
    if job is not None:
        _terminate_job(job)
        inst._job_handle = None
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)
        logger.info("NavPy stopped: sys_id=%d, pid=%d", inst.sys_id, proc.pid)
        return
    if proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)
    logger.info("NavPy stopped: sys_id=%d, pid=%d", inst.sys_id, proc.pid)


__all__ = [
    "NavpyInstance",
    "read_navpy_output",
    "snapshot_navpy_instance",
    "terminate_navpy_instance",
]
