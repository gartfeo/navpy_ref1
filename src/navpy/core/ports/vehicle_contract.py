"""Composite compatibility contract built from narrow capabilities."""

from __future__ import annotations

from navpy.core.ports.parameter_exchange import ExchangePolicy, ParameterExchange
from navpy.core.ports.vehicle_control_interfaces import (
    CommandValue,
    PARAM_PROTOCOL_ATTEMPTS,
    ParameterValue,
    VehicleFlightControl,
    VehicleMissionAccess,
    VehicleParameters,
    VehicleSimulationAccess,
)
from navpy.core.ports.vehicle_telemetry_interfaces import (
    VehicleIdentityAccess,
    VehicleLimits,
    VehicleMotionTelemetry,
    VehiclePowerTelemetry,
)
from navpy.core.ports.vehicle_runtime_interfaces import (
    VehicleLifetime,
    VehicleLogging,
    VehicleMessaging,
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
        retries: int = PARAM_PROTOCOL_ATTEMPTS,
        quiet: bool = False,
    ) -> float | None:
        del timeout, retries, quiet
        return self.get_parameter(name)

    def send_parameter_unverified(self, name: str, value: int | float) -> bool:
        return bool(self.set_parameter(name, value))

    def read_parameter(self, name: str, policy: ExchangePolicy, timeout: float = 2.0) -> ParameterExchange:
        del name, policy, timeout
        raise NotImplementedError("legacy vehicle has no bounded parameter exchange")

    def write_parameter(self, name: str, value: ParameterValue, policy: ExchangePolicy,
                        timeout: float = 2.0) -> ParameterExchange:
        del name, value, policy, timeout
        raise NotImplementedError("legacy vehicle has no bounded parameter exchange")

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
