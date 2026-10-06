"""Build and launch one GCS-managed NavPy subprocess."""

from __future__ import annotations

import logging
import os
import re
import subprocess
import sys
import threading
from collections.abc import Callable
from pathlib import Path

from navpy.exception_groups import BaseExceptionGroup
from gcs.backend.navpy_process_instance import (
    NavpyInstance,
    _assign_job,
    _close_job,
    _create_job,
    terminate_navpy_instance,
)


log = logging.getLogger(__name__)
_SRC_DIR = str(Path(__file__).resolve().parents[2])
_UDP_BIND_CONNECTION = re.compile(r"^udp(?:in)?:[^:]+:(\d+)$")


def _foreign_udp_holders(connection: str) -> list[int]:
    """Return foreign PIDs already bound to a local UDP server port."""
    match = _UDP_BIND_CONNECTION.match((connection or "").strip())
    if not match:
        return []
    from gcs.backend.instance_registry import listeners_on_port, udp_port_held

    port = int(match.group(1))
    if not udp_port_held(port):
        return []
    holders = [
        pid for pid in listeners_on_port(port, "udp") if pid != os.getpid()
    ]
    if holders:
        return holders if udp_port_held(port) else []
    # The owner may have released the socket between the fast bind probe and
    # netstat. Recheck the source of truth before reporting an unnamed holder.
    return [-1] if udp_port_held(port) else []


def _ready_udp_owner_error(
    connection: str,
    runtime_pid: object,
) -> str | None:
    """Verify sole UDP ownership after the child reports runtime readiness."""
    match = _UDP_BIND_CONNECTION.match((connection or "").strip())
    if not match:
        return None
    if type(runtime_pid) is not int or runtime_pid <= 0:
        return "runtime readiness did not report a valid process_pid"
    from gcs.backend.instance_registry import listeners_on_port

    port = int(match.group(1))
    holders = sorted(set(listeners_on_port(port, "udp")))
    if runtime_pid not in holders:
        return (
            f"companion UDP port {port} is not owned by ready runtime pid "
            f"{runtime_pid}; observed pid(s) {holders or [-1]}"
        )
    foreign = [pid for pid in holders if pid != runtime_pid]
    if foreign:
        return (
            f"companion UDP port {port} is shared by ready runtime pid "
            f"{runtime_pid} and foreign pid(s) {foreign}"
        )
    return None


def _build_env() -> dict[str, str]:
    """Build the child environment with NavPy sources and UTF-8 output."""
    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    if _SRC_DIR not in existing:
        env["PYTHONPATH"] = f"{_SRC_DIR}{os.pathsep}{existing}" if existing else _SRC_DIR
    env["PYTHONUTF8"] = "1"
    return env


def _normalize_vision_profile(value: str | None) -> str | None:
    if not isinstance(value, str):
        return None
    profile = value.strip()
    return profile or None


def _normalize_log_level(value: str | None) -> str | None:
    if not isinstance(value, str) or not value.strip():
        return None
    from navpy.logger.cache_log_level import CacheLogLevel

    try:
        return CacheLogLevel.from_name(value).name
    except ValueError:
        log.warning("Ignoring invalid NavPy log level %r", value)
        return None


def _build_navpy_command(
    sys_id: int,
    connection: str,
    *,
    detector_debug_show: bool,
    vision_profile: str | None,
    log_level: str | None,
    navigation_speedup: float,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "navpy.main",
        "-c",
        connection,
        "-ss",
        str(sys_id),
        "-nt",
        "mav",
        "--detector-type",
        "sim",
    ]
    if navigation_speedup > 0.0:
        command.extend(["-gsu", str(navigation_speedup)])
    command.extend(["-lsd", "Vehicle", "Network"])
    normalized_profile = _normalize_vision_profile(vision_profile)
    if normalized_profile:
        command.extend(["--vision-profile", normalized_profile])
    normalized_level = _normalize_log_level(log_level)
    if normalized_level:
        command.extend(["--log-level", normalized_level])
    if detector_debug_show:
        command.append("--detector-debug-show")
    return command


def launch_navpy_instance(
    sys_id: int,
    connection: str,
    *,
    detector_debug_show: bool,
    vision_profile: str | None,
    log_level: str | None,
    navigation_speedup: float,
    popen_factory: Callable[..., subprocess.Popen[bytes]],
    output_reader: Callable[[NavpyInstance], None],
    logger: logging.Logger,
) -> NavpyInstance:
    command = _build_navpy_command(
        sys_id,
        connection,
        detector_debug_show=detector_debug_show,
        vision_profile=vision_profile,
        log_level=log_level,
        navigation_speedup=navigation_speedup,
    )
    kwargs: dict[str, object] = {
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "env": _build_env(),
    }
    if sys.platform == "win32":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    job_handle: int | None = None
    if sys.platform == "win32":
        try:
            job_handle = _create_job()
        except OSError:
            logger.warning("Failed to create job object for sys_id=%d", sys_id)
    try:
        proc = popen_factory(command, **kwargs)  # noqa: S603
    except BaseException as failure:
        if job_handle is not None:
            try:
                _close_job(job_handle)
            except BaseException as cleanup_error:
                raise BaseExceptionGroup(
                    "NavPy spawn and job cleanup failed",
                    (failure, cleanup_error),
                ) from failure
        raise
    if job_handle is not None:
        try:
            _assign_job(job_handle, proc)
        except Exception:
            logger.warning("Failed to assign process to job for sys_id=%d", sys_id)
            _close_job(job_handle)
            job_handle = None
    inst = NavpyInstance(
        sys_id=sys_id,
        connection=connection,
        process=proc,
        _job_handle=job_handle,
    )
    try:
        reader = threading.Thread(
            target=output_reader,
            args=(inst,),
            daemon=True,
        )
        inst._reader_thread = reader
        reader.start()
    except BaseException as failure:
        cleanup_errors: list[BaseException] = []
        try:
            terminate_navpy_instance(inst, logger)
        except BaseException as cleanup_error:
            cleanup_errors.append(cleanup_error)
        try:
            if proc.stdout is not None:
                proc.stdout.close()
        except BaseException as cleanup_error:
            cleanup_errors.append(cleanup_error)
        if cleanup_errors:
            raise BaseExceptionGroup(
                "NavPy launch and cleanup failed",
                (failure, *cleanup_errors),
            ) from failure
        raise
    return inst


__all__ = ["launch_navpy_instance"]
