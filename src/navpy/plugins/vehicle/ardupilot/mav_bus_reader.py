"""Single-reader MAVLink dispatch runtime for a shared bus."""
from __future__ import annotations

import logging
from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging import reader_trace as _tr  # TEMP diag
from navpy.plugins.vehicle.ardupilot.mav_bus_heartbeats import MavBusHeartbeats
from navpy.plugins.vehicle.ardupilot.mav_bus_membership import MavBusMembership
from navpy.plugins.vehicle.ardupilot.mav_bus_reader_budget import (
    DEFAULT_READER_BATCH_BUDGET_S,
)

log = logging.getLogger(__name__)

_PEER_RESET_LOG_INTERVAL_S = 30.0


def _yield_reader_thread() -> None:
    """Let ready control/source threads run after one packet batch."""
    rt.sleep(0)


class MavBusReader:
    def __init__(
        self,
        connection: mavutil.mavfile,
        device: str,
        membership: MavBusMembership,
        heartbeats: MavBusHeartbeats,
        stop_event: rt.Event,
        *,
        cooperative_yield: Callable[[], None] = _yield_reader_thread,
        on_exit: Callable[[], None] = lambda: None,
        batch_budget_s: Callable[[], float] = lambda: DEFAULT_READER_BATCH_BUDGET_S,
        clock: Callable[[], float] = rt.monotonic,
    ) -> None:
        self._connection = connection
        self._device = device
        self._membership = membership
        self._heartbeats = heartbeats
        self._stop = stop_event
        self._cooperative_yield = cooperative_yield
        self._on_exit = on_exit
        self._batch_budget_s = batch_budget_s
        self._clock = clock
        self._is_udp = device.startswith(("udp:", "udpin:"))
        self._last_reset_log_s = 0.0
        self._thread = rt.Thread(target=self._run, daemon=True)
        self._acc = [0.0, 0, 0.0, 0]  # TEMP diag: recvfrom s, n, select s, n
        if _tr.ENABLED:
            self._wrap_io()

    def _wrap_io(self) -> None:  # TEMP diag
        conn, acc, pc = self._connection, self._acc, _tr._pc
        orig_recv, orig_select = conn.recv, conn.select

        def recv(n=None):
            t = pc()
            try:
                return orig_recv(n)
            finally:
                acc[0] += pc() - t
                acc[1] += 1

        def select(timeout):
            t = pc()
            try:
                return orig_select(timeout)
            finally:
                acc[2] += pc() - t
                acc[3] += 1

        conn.recv = recv
        conn.select = select

    def _recv_traced(self, blocking: bool) -> MAVLink_message | None:  # TEMP diag
        if not _tr.ENABLED:
            if blocking:
                return self._connection.recv_match(blocking=True, timeout=0.5)
            return self._connection.recv_match(blocking=False)
        acc = self._acc
        acc[0] = 0.0
        acc[1] = 0
        acc[2] = 0.0
        acc[3] = 0
        t0 = _tr._pc()
        if blocking:
            message = self._connection.recv_match(blocking=True, timeout=0.5)
        else:
            message = self._connection.recv_match(blocking=False)
        t1 = _tr._pc()
        self._rx = (t0, t1 - t0, acc[0], acc[2], acc[1], acc[3], blocking)
        return message

    def _dispatch_traced(self, message: MAVLink_message) -> None:  # TEMP diag
        if not _tr.ENABLED:
            self._dispatch(message)
            return
        t0 = _tr._pc()
        self._dispatch(message)
        t1 = _tr._pc()
        rx = getattr(self, "_rx", None)
        if rx is None:
            return
        self._rx = None
        try:
            mtype = message.get_type()
            src = message.get_srcSystem()
        except Exception:
            mtype, src = "?", 0
        boot = getattr(message, "time_boot_ms", "")
        _tr.rec("pk", rx[0], rx[1], rx[2], rx[3], t1 - t0,
                f"{mtype}|{boot}|{src}|{rx[4]}|{rx[5]}|{int(rx[6])}")

    def start(self) -> None:
        with rt.detached():  # owned by its bus (closed by it), not the opening request
            self._thread.start()

    def join(self, timeout_s: float | None = None) -> bool:
        """Whether the reader has stopped (its own thread counts: it exits next)."""
        if rt.current_thread() is self._thread:
            return True
        self._thread.join(timeout=timeout_s)
        return not self._thread.is_alive()

    def _run(self) -> None:
        try:
            self._read_until_stopped()
        finally:
            self._on_exit()

    def _read_until_stopped(self) -> None:
        while not self._stop.is_set():
            try:
                message = self._recv_traced(True)
                if message is not None:
                    _tb = _tr._pc()
                    try:
                        self._dispatch_batch(message)
                    finally:
                        _ty = _tr._pc()
                        if _tr.ENABLED:
                            _tr.rec("bt", _tb, _ty - _tb, getattr(self, "_bn", 0), getattr(self, "_be", ""))
                        # Accelerated SITL can keep the UDP decoder permanently
                        # runnable.  Yield once per batch so the detector and
                        # AP-rate command worker are not starved by a backlog.
                        # At hardware rates recv_match normally blocks.  This
                        # handoff adds no fixed delay or clock scaling; it only
                        # lets another ready thread run under contention.
                        self._cooperative_yield()
                        if _tr.ENABLED:
                            _tr.rec("yd", _ty, _tr._pc() - _ty)
            except ConnectionResetError:
                self._handle_reset()
            except Exception:
                if not self._stop.is_set():
                    log.exception("MavBus %s reader error", self._device)

    def _dispatch_batch(self, message: MAVLink_message) -> None:
        """Dispatch ``message``, then every packet decodable without blocking.

        The batch ends when the non-blocking receive returns ``None`` (socket
        empty or a partial frame the next blocking call completes), on stop,
        or once it has run for the bus budget (one scheduler wall slot).  The
        bound is checked between packets, so it overshoots by at most one
        packet's receive and dispatch.
        """
        deadline_s = self._clock() + self._batch_budget_s()
        self._bn = 0
        while True:
            self._dispatch_traced(message)
            self._bn += 1
            if self._stop.is_set() or self._clock() >= deadline_s:
                self._be = "budget"
                return
            message = self._recv_traced(False)
            if message is None:
                self._be = "empty"
                return

    def _dispatch(self, message: MAVLink_message) -> None:
        self._heartbeats.observe(message)
        for receiver in self._membership.receivers_for(message.get_srcSystem()):
            receiver.feed_message(message)

    def _handle_reset(self) -> None:
        if self._stop.is_set():
            return
        if not self._is_udp:
            log.exception("MavBus %s reader error", self._device)
            return
        now_s = rt.monotonic()
        if now_s - self._last_reset_log_s > _PEER_RESET_LOG_INTERVAL_S:
            self._last_reset_log_s = now_s
            log.debug(
                "MavBus %s: UDP peer temporarily unreachable (connection reset)",
                self._device,
            )
