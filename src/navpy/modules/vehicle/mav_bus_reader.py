"""Single-reader MAVLink dispatch runtime for a shared bus."""
from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.vehicle.mav_bus_heartbeats import MavBusHeartbeats
from navpy.modules.vehicle.mav_bus_membership import MavBusMembership

log = logging.getLogger(__name__)

_PEER_RESET_LOG_INTERVAL_S = 30.0


def _yield_reader_thread() -> None:
    """Let ready control/source threads run after one decoded packet."""
    time.sleep(0)


class MavBusReader:
    def __init__(
        self,
        connection: mavutil.mavfile,
        device: str,
        membership: MavBusMembership,
        heartbeats: MavBusHeartbeats,
        stop_event: threading.Event,
        *,
        cooperative_yield: Callable[[], None] = _yield_reader_thread,
    ) -> None:
        self._connection = connection
        self._device = device
        self._membership = membership
        self._heartbeats = heartbeats
        self._stop = stop_event
        self._cooperative_yield = cooperative_yield
        self._is_udp = device.startswith(("udp:", "udpin:"))
        self._last_reset_log_s = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout_s: float | None = None) -> None:
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=timeout_s)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                message = self._connection.recv_match(blocking=True, timeout=0.5)
                if message is not None:
                    try:
                        self._dispatch(message)
                    finally:
                        # Accelerated SITL can keep the UDP decoder permanently
                        # runnable.  Yield after one packet so the detector and
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
        now_s = time.monotonic()
        if now_s - self._last_reset_log_s > _PEER_RESET_LOG_INTERVAL_S:
            self._last_reset_log_s = now_s
            log.debug(
                "MavBus %s: UDP peer temporarily unreachable (connection reset)",
                self._device,
            )
