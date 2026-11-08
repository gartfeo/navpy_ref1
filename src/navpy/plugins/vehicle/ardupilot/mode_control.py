"""Flight mode read and command owner."""
from __future__ import annotations

from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

from navpy.core.ports.flight_mode import FlightMode
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity


class ModeControl:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        heartbeat_state: HeartbeatState,
        logger_ref: LoggerRef,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._heartbeat = heartbeat_state
        self._logger_ref = logger_ref

    def set(
        self,
        flight_mode: FlightMode | str,
        *,
        fallback_custom_mode: int | None = None,
        lock_timeout_s: float | None = None,
        guard: Callable[[], bool] | None = None,
    ) -> bool:
        """``lock_timeout_s`` bounds the wait for the send lock (not GUIDED's fenced send): when a stuck send holds
        it, nothing is written, now or later, and False is returned. ``guard`` runs under the lock right before the
        write; False cancels it."""
        name = flight_mode.name if hasattr(flight_mode, "name") else str(flight_mode)

        def _send(connection: mavutil.mavfile) -> bool:
            mode_mapping = connection.mode_mapping()
            mode_id = (
                fallback_custom_mode
                if mode_mapping is None
                else mode_mapping.get(name, fallback_custom_mode)
            )
            if mode_id is None:
                self._logger_ref.value.error(f"Unknown mode {flight_mode}")
                return False
            if guard is not None and not guard():
                return False
            connection.mav.set_mode_send(
                self._identity.target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            return True

        if name != FlightMode.GUIDED.value:
            if lock_timeout_s is None:
                return bool(self._transport.call(_send))
            sent = self._transport.try_call(_send, blocking=True, timeout=lock_timeout_s)
            if sent is None:
                self._logger_ref.value.warning(
                    f"{name} request not sent: the link's send lock was busy for {lock_timeout_s:g} s",
                    key="vehicle",
                )
            return bool(sent)
        # NavPy taking GUIDED is a navigation command: the closed command
        # fence drops it (mav_command_fence.py). Every other mode request,
        # the hold's LOITER and AUTO included, is never fenced.
        with self._transport.fenced() as connection:
            if connection is None:
                if self._transport.fence.first_drop_of_generation():
                    self._logger_ref.value.warning(
                        "GUIDED request dropped by the closed command fence "
                        f"(generation {self._transport.fence.generation}).",
                        key="vehicle",
                    )
                return False
            return _send(connection)

    @property
    def heartbeat_count(self) -> int:
        """Heartbeats observed so far; read it before :attr:`current` to pair them."""
        return self._heartbeat.count

    @property
    def current(self) -> FlightMode | None:
        heartbeat = self._heartbeat.message
        if heartbeat is None:
            return None
        try:
            return FlightMode(mavutil.mode_string_v10(heartbeat))
        except ValueError:
            return None
