"""Acknowledged clean stop for one NavPy subprocess, with a hard-kill fallback."""

from __future__ import annotations

import logging
import subprocess

from gcs.backend.navpy_process_instance import (
    NavpyInstance,
    _close_job,
    terminate_navpy_instance,
)
from navpy.app.runtime_stop import NAVPY_STOP_REQUEST


#: A clean stop joins every NavPy producer (each join is bounded at a few
#: seconds) and drains the log queue before the ack; past this the stop counts
#: as missed and the job object is terminated.
NAVPY_STOP_ACK_TIMEOUT_S = 20.0
#: After the process exits, its stdout reader only has to read the last lines.
_READER_DRAIN_TIMEOUT_S = 5.0

STOP_ACKED = "acked"
STOP_TERMINATED = "terminated"
STOP_ALREADY_EXITED = "already_exited"
#: The stop itself raised; the generation stays tracked.
STOP_FAILED = "stop_failed"


def _request_stop(inst: NavpyInstance) -> bool:
    stdin = inst.process.stdin
    if stdin is None:
        return False
    if stdin.closed:
        # An earlier attempt already sent it; a retry waits and terminates.
        return True
    try:
        stdin.write(f"{NAVPY_STOP_REQUEST}\n".encode("ascii"))
        stdin.close()
    except OSError:
        return False
    return True


def stop_navpy_instance(
    inst: NavpyInstance,
    logger: logging.Logger,
    ack_timeout_s: float = NAVPY_STOP_ACK_TIMEOUT_S,
) -> str:
    """Ask for a clean stop; terminate only when the ack never comes.

    Returns ``acked``, ``terminated`` (a missed ack: a host fault for an
    evaluator) or ``already_exited`` (the process ended before the request).
    """
    proc = inst.process
    if proc.poll() is not None:
        terminate_navpy_instance(inst, logger)
        return STOP_ALREADY_EXITED
    if _request_stop(inst):
        try:
            proc.wait(timeout=ack_timeout_s)
        except subprocess.TimeoutExpired:
            pass
        else:
            reader = inst._reader_thread
            if reader is not None:
                reader.join(_READER_DRAIN_TIMEOUT_S)
            if inst.stop_acked:
                job = inst._job_handle
                if job is not None:
                    inst._job_handle = None
                    _close_job(job)
                logger.info(
                    "NavPy stopped cleanly: sys_id=%d, pid=%d",
                    inst.sys_id,
                    proc.pid,
                )
                return STOP_ACKED
    logger.warning(
        "NavPy stop not acknowledged: sys_id=%d, pid=%d; terminating (host fault)",
        inst.sys_id,
        proc.pid,
    )
    terminate_navpy_instance(inst, logger)
    return STOP_TERMINATED


__all__ = [
    "NAVPY_STOP_ACK_TIMEOUT_S",
    "STOP_ACKED",
    "STOP_ALREADY_EXITED",
    "STOP_FAILED",
    "STOP_TERMINATED",
    "stop_navpy_instance",
]
