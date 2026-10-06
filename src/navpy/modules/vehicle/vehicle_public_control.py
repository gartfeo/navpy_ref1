"""Stateless public control facets for :class:`VehicleMav`."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.vehicle_capability_interfaces import CommandValue
from navpy.modules.vehicle.vehicle_public_ports import (
    CommandParts,
    FlightControlParts,
    LifetimeParts,
)


class VehicleFlightControlFacet:
    """Delegate high-level flight control to focused capabilities."""

    _parts: FlightControlParts

    def goto(self, location: Location) -> None:
        return self._parts.navigation.goto(location)

    def goto_loiter(self, location: Location, radius: float) -> None:
        return self._parts.navigation.goto_loiter(location, radius)

    def set_attitude(
        self,
        roll: float | None,
        pitch: float | None,
        yaw: float | None = None,
        thr: float | None = None,
    ) -> None:
        return self._parts.attitude.set(roll, pitch, yaw, thr)

    def set_mode(
        self,
        flight_mode: FlightMode | str,
        *,
        fallback_custom_mode: int | None = None,
    ) -> bool:
        return self._parts.mode.set(
            flight_mode,
            fallback_custom_mode=fallback_custom_mode,
        )


class VehicleCommandFacet:
    """Delegate low-level MAVLink command emission."""

    _parts: CommandParts

    def send_command_long(
        self,
        command: int,
        p1: CommandValue = 0,
        p2: CommandValue = 0,
        p3: CommandValue = 0,
        p4: CommandValue = 0,
        p5: CommandValue = 0,
        p6: CommandValue = 0,
        p7: CommandValue = 0,
    ) -> None:
        return self._parts.commands.send_command_long(
            command,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        )

    def send_manual_control(
        self,
        x: int,
        y: int,
        z: int,
        r: int,
        buttons: int = 0,
    ) -> None:
        return self._parts.commands.send_manual_control(
            x,
            y,
            z,
            r,
            buttons,
        )

    def disarm(self) -> None:
        return self._parts.commands.disarm()


class VehicleLifetimeFacet:
    """Delegate logger replacement and deterministic shutdown."""

    _parts: LifetimeParts

    def set_logger(self, logger: ILogger) -> None:
        return self._parts.lifetime.set_logger(logger)

    def close(self) -> None:
        return self._parts.lifetime.close()
