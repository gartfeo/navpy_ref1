"""The runner's end of a HITL approach (design hitl_final_approach.md section 2, "Handoff at the end").

Before any RTL the runner stops navpy (NAVPY_STOP_V1 on its stdin, bounded wait; on timeout it terminates the navpy
process it started) and then closes navpy's router endpoint itself: the barrier navpy passes before its terminal
record, idempotent, so after a crash, a forced stop or a failed navpy barrier no navpy packet still buffered in the
router can follow the RTL. Without the router's ack the router process is terminated, the runner opens the board's
port itself and sends RTL there, and the run has failed. A floor breach or a timeout takes the same path
(HeightFloor's before_recovery, flight_floor.py).
"""
from __future__ import annotations

import subprocess
import time
from typing import Callable, Optional

from pymavlink import mavutil

from navpy.app.router_barrier import BarrierResult, RouterBarrier
from navpy.app.runtime_stop import NAVPY_STOP_REQUEST
from tools.tuning.board import SOURCE_SYSTEM
from tools.tuning.vehicle import MODE

# navpy's clean stop joins its workers and flushes its logs (runtime_resources.py); seconds on a PC. Past this it is
# stuck (no ack is printed on a worker error) and the runner terminates it.
NAVPY_STOP_WAIT_S = 10.0
KILL_WAIT_S = 5.0                # a terminated process is gone in milliseconds; past this it is killed
# The direct RTL after a failed barrier: the board's heartbeat (1 Hz) and its mode change, with margin.
DIRECT_RTL_WAIT_S = 10.0
DIRECT_RTL_RESEND_S = 1.0        # DO_SET_MODE again while the heartbeat does not show RTL (a lost command)
RECOVERY_MODE = "RTL"

Recorder = Callable[[], dict]


def end_process(process: subprocess.Popen, wait_s: float = KILL_WAIT_S) -> dict:
    """Terminate, then kill if it does not go."""
    if process.poll() is not None:
        return {"ended": "already exited", "returncode": process.returncode}
    process.terminate()
    try:
        process.wait(wait_s)
        return {"ended": "terminated", "returncode": process.returncode}
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        return {"ended": "killed", "returncode": process.returncode}


def stop_navpy(process: subprocess.Popen, wait_s: float = NAVPY_STOP_WAIT_S) -> dict:
    """Ask navpy to stop on its stdin (it runs with --stop-on-stdin) and wait for it to exit; else end it."""
    if process.poll() is not None:
        return {"stop": "already exited", "returncode": process.returncode}
    line = f"{NAVPY_STOP_REQUEST}\n"
    try:
        process.stdin.write(line if process.text_mode else line.encode())
        process.stdin.flush()
    except (OSError, ValueError):            # its stdin is already closed: it is on its way out
        pass
    try:
        process.wait(wait_s)
        return {"stop": "requested", "returncode": process.returncode}
    except subprocess.TimeoutExpired:
        return {"stop": "timed out", **end_process(process)}


def direct_rtl(device: str, baud: int, wait_s: float = DIRECT_RTL_WAIT_S) -> dict:
    """RTL on the board's own port (the router is gone): DO_SET_MODE until the heartbeat shows it, bounded."""
    conn = mavutil.mavlink_connection(device, baud=baud, source_system=SOURCE_SYSTEM)
    try:
        deadline = time.monotonic() + wait_s
        if conn.wait_heartbeat(timeout=wait_s) is None:
            return {"rtl": "direct", "confirmed": False, "commands_sent": 0, "error": "no heartbeat"}
        sent = 0
        while (left := deadline - time.monotonic()) > 0:
            conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                       0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE[RECOVERY_MODE],
                                       0, 0, 0, 0, 0)
            sent += 1
            resend_at = time.monotonic() + min(DIRECT_RTL_RESEND_S, left)
            while (wait := resend_at - time.monotonic()) > 0:
                hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=wait)
                if hb is not None and hb.custom_mode == MODE[RECOVERY_MODE]:
                    return {"rtl": "direct", "confirmed": True, "commands_sent": sent}
        return {"rtl": "direct", "confirmed": False, "commands_sent": sent}
    finally:
        conn.close()


class NavpyHandoff:
    """Stops navpy and closes its router endpoint before the runner's RTL; once (a second call returns the record).

    before_rtl() returns the record. record["rtl"] == "direct" means the barrier failed: the router is gone, the RTL
    has already been sent on the board's own port and record["failed"] says why the run has failed. Otherwise the
    caller sends its RTL through the router as usual. An error in a step is recorded; until the router has confirmed
    the close, it takes the no-ack path."""

    def __init__(self, stop: Recorder, barrier: RouterBarrier, stop_router: Recorder, rtl_direct: Recorder) -> None:
        self._stop = stop
        self._barrier = barrier
        self._stop_router = stop_router
        self._rtl_direct = rtl_direct
        self.record: Optional[dict] = None

    def before_rtl(self) -> dict:
        if self.record is not None:
            return self.record
        record = self.record = {}
        try:
            record["navpy"] = self._stop()
        except Exception as exc:     # noqa: BLE001 - the barrier below still decides the path
            record["navpy"] = {"error": f"{type(exc).__name__}: {exc}"}
        try:
            result = self._barrier.close_endpoint()
        except Exception as exc:     # noqa: BLE001 - no confirmed close: the same path as no ack
            result = BarrierResult(None, f"{type(exc).__name__}: {exc}")
        record["barrier"] = {"ok": result.ok, "ack": result.ack, "error": result.error}
        if not result.ok:                # marked first: nothing may go through the router now, whatever follows
            record.update({"rtl": "direct", "confirmed": False,
                           "failed": f"navpy's router endpoint not closed before RTL: {result.error}"})
            try:
                record["router"] = self._stop_router()
            except Exception as exc:     # noqa: BLE001 - recorded; the direct RTL is still sent
                record["router"] = {"error": f"{type(exc).__name__}: {exc}"}
            try:
                record.update(self._rtl_direct())
            except Exception as exc:     # noqa: BLE001 - e.g. the port not free yet: recorded, the run has failed
                record["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[driver] navpy handoff: {record}", flush=True)
        return record
