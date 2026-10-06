"""Vehicle module - Vehicle abstraction and MAVLink communication.

This module owns ALL vehicle-related functionality:
- IVehicle: Abstract vehicle interface (ABC with enforcement)
- VehicleMav: MAVLink-based vehicle implementation
- FlightMode: Flight mode enumeration
- MissionItem: Mission item data model

Usage:
    from navpy.modules.vehicle import IVehicle, VehicleMav, FlightMode
"""
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.vehicle_factory import create_vehicle
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.mission_item import MissionItem
from navpy.modules.vehicle.mav_bus import MavBus
from navpy.modules.common.models.wind import Wind

__all__ = [
    "IVehicle",
    "VehicleMav",
    "MavBus",
    "create_vehicle",
    "FlightMode",
    "MissionItem",
    "Wind",
]

