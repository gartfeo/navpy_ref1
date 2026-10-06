"""Composite compatibility contract built from narrow capabilities."""

from __future__ import annotations

from navpy.modules.vehicle.vehicle_capability_interfaces import CommandValue
from navpy.modules.vehicle.vehicle_capability_interfaces import (
    VehicleFlightControl,
    VehicleIdentityAccess,
    VehicleLifetime,
    VehicleLimits,
    VehicleLogging,
    VehicleMessaging,
    VehicleMissionAccess,
    VehicleMotionTelemetry,
    VehicleParameters,
    VehiclePowerTelemetry,
    VehicleSimulationAccess,
)


class IVehicle(
    VehicleIdentityAccess,
    VehicleMotionTelemetry,
    VehiclePowerTelemetry,
    VehicleLimits,
    VehicleFlightControl,
    VehicleMissionAccess,
    VehicleParameters,
    VehicleSimulationAccess,
    VehicleMessaging,
    VehicleLogging,
    VehicleLifetime,
):
    """Complete legacy contract; new leaves should request one capability."""

    def __init__(self, target_system: int) -> None:
        self._target_system = target_system
        self._source_system = target_system

    @property
    def target_system(self) -> int:
        return self._target_system

    @target_system.setter
    def target_system(self, value: int) -> None:
        self._target_system = value

    @property
    def source_system(self) -> int:
        return self._source_system

    @source_system.setter
    def source_system(self, value: int) -> None:
        self._source_system = value

    @property
    def transport_source_system(self) -> int | None:
        return None

    @property
    def transport_source_component(self) -> int | None:
        return None

    @property
    def battery_voltage(self) -> float | None:
        return None

    @property
    def battery_current(self) -> float | None:
        return None

    def get_parameter_fresh(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None:
        del timeout, retries, quiet
        return self.get_parameter(name)

    def send_parameter_unverified(self, name: str, value: int | float) -> bool:
        return bool(self.set_parameter(name, value))

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
        del command, p1, p2, p3, p4, p5, p6, p7
        raise NotImplementedError("legacy vehicle has no COMMAND_LONG capability")


__all__ = ["IVehicle"]
