"""Compatibility exports for interface-segregated vehicle capabilities."""

from navpy.modules.vehicle.vehicle_control_interfaces import (
    CommandValue,
    ParameterValue,
    VehicleFlightControl,
    VehicleMissionAccess,
    VehicleParameters,
    VehicleSimulationAccess,
)
from navpy.modules.vehicle.vehicle_runtime_interfaces import (
    RcChannelReceiver,
    VehicleLifetime,
    VehicleLogging,
    VehicleMessaging,
)
from navpy.modules.vehicle.vehicle_telemetry_interfaces import (
    VehicleIdentityAccess,
    VehicleLimits,
    VehicleMotionTelemetry,
    VehiclePowerTelemetry,
)

__all__ = [
    "CommandValue",
    "ParameterValue",
    "RcChannelReceiver",
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
