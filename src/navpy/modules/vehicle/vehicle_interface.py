"""Exact compatibility re-exports for vehicle contracts."""

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
from navpy.modules.vehicle.vehicle_contract import IVehicle


__all__ = [
    "IVehicle",
    "VehicleFlightControl",
    "VehicleIdentityAccess",
    "VehicleLifetime",
    "VehicleLimits",
    "VehicleLogging",
    "VehicleMessaging",
    "VehicleMissionAccess",
    "VehicleMotionTelemetry",
    "VehicleParameters",
    "VehiclePowerTelemetry",
    "VehicleSimulationAccess",
]
