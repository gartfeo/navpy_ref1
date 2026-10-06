"""Atomic mode-plus-guided navigation command transactions."""
from __future__ import annotations

import math

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_DO_REPOSITION,
    MAV_CMD_NAV_WAYPOINT,
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
)

from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.mode_control import ModeControl
from navpy.modules.vehicle.parameter_client import ParameterClient
from navpy.modules.vehicle.position_service import PositionService
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


class GuidedNavigation:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        mode_control: ModeControl,
        position: PositionService,
        parameters: ParameterClient,
        logger_ref: LoggerRef,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._mode = mode_control
        self._position = position
        self._parameters = parameters
        self._logger_ref = logger_ref

    def goto(self, location: Location) -> None:
        altitude = float(location.alt)
        home = self._position.home_location if location.is_absolute else None
        if home is not None:
            altitude -= float(home.alt)
        with self._transport.transaction() as connection:
            if self._mode.current != FlightMode.GUIDED:
                self._mode.set(FlightMode.GUIDED)
            connection.mav.mission_item_send(
                self._identity.target_system,
                0,
                0,
                MAV_FRAME_GLOBAL_RELATIVE_ALT,
                MAV_CMD_NAV_WAYPOINT,
                2,
                0,
                0,
                0,
                0,
                0,
                float(location.lat),
                float(location.lng),
                altitude,
            )

    def goto_loiter(self, location: Location, radius: float) -> None:
        try:
            command_radius = float(radius)
        except (TypeError, ValueError):
            self._logger_ref.value.warning(
                f"goto_loiter: non-numeric radius {radius!r}; "
                "not commanding loiter.",
                key="vehicle",
            )
            return
        if not math.isfinite(command_radius) or command_radius <= 0.0:
            self._logger_ref.value.warning(
                f"goto_loiter: invalid radius {command_radius}; "
                "not commanding loiter.",
                key="vehicle",
            )
            return
        altitude = float(location.alt)
        home = self._position.home_location if location.is_absolute else None
        if home is not None:
            altitude -= float(home.alt)
        try:
            self._parameters.send_unverified("WP_LOITER_RAD", command_radius)
        except Exception as exc:
            self._logger_ref.value.warning(
                f"goto_loiter: WP_LOITER_RAD send failed ({exc}); "
                f"DO_REPOSITION param3 still carries radius={command_radius:.0f}m.",
                key="vehicle",
            )
        with self._transport.transaction() as connection:
            if self._mode.current != FlightMode.GUIDED:
                self._mode.set(FlightMode.GUIDED)
            connection.mav.command_int_send(
                self._identity.target_system,
                0,
                MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                MAV_CMD_DO_REPOSITION,
                0,
                0,
                -1,
                MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                command_radius,
                0,
                int(round(float(location.lat) * 1e7)),
                int(round(float(location.lng) * 1e7)),
                altitude,
            )
        self._logger_ref.value.info(
            f"GUIDED_LOITER cmd=DO_REPOSITION center={float(location.lat):.6f},"
            f"{float(location.lng):.6f},{altitude:.1f} "
            f"radius={command_radius:.0f}m",
            key="vehicle",
        )
