"""Single-reader MAVLink dispatch runtime for a shared bus."""
from __future__ import annotations

import logging
from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
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
                message = self._connection.recv_match(blocking=True, timeout=0.5)
                if message is not None:
                    try:
                        self._dispatch_batch(message)
                    finally:
                        # Accelerated SITL can keep the UDP decoder permanently
                        # runnable.  Yield once per batch so the detector and
                        # AP-rate command worker are not starved by a backlog.
                        # At hardware rates recv_match normally blocks.  This
                        # handoff adds no fixed delay or clock scaling; it only
                        # lets another ready thread run under contention.
                        self._cooperative_yield()
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
        while True:
            self._dispatch(message)
            if self._stop.is_set() or self._clock() >= deadline_s:
                return
            message = self._connection.recv_match(blocking=False)
            if message is None:
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
