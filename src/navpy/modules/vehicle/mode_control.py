"""Flight mode read and command owner."""
from __future__ import annotations

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.link_state import HeartbeatState
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


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
    ) -> bool:
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
            connection.mav.set_mode_send(
                self._identity.target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            return True

        return bool(self._transport.call(_send))

    @property
    def current(self) -> FlightMode | None:
        heartbeat = self._heartbeat.message
        if heartbeat is None:
            return None
        try:
            return FlightMode(mavutil.mode_string_v10(heartbeat))
        except ValueError:
            return None
