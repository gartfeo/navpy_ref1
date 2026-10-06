"""NavPy Modules - The domain logic layer.

This package contains all domain-specific modules organized by responsibility:

Modules:
    common      - Shared types (Location, Attitude, Wind)
    vehicle     - Vehicle abstraction and MAVLink communication
    vision      - Detection, cameras, gimbals, delivery-reference tracking
    navigation        - Navigation algorithms, geo-reference, terrain utilities
    nav         - Navigation controller and delivery-reference management
    comm        - Network communication (WiFi, serial, MAVLink)
    swarm       - Fleet coordination and delivery-task distribution

Each module exposes:
    - Interfaces (ABC with @abstractmethod for enforcement)
    - Implementations
    - Configuration dataclasses (where applicable)

Usage:
    # Import from specific modules
    from navpy.modules.vehicle import IVehicle, VehicleMav, FlightMode
    from navpy.modules.navigation import Navigation, GeoRefCalc, ZcUtil
    from navpy.modules.vision import DetectorAbc, DetectorSim
    from navpy.modules.swarm import TaskActor, TaskDispatch
    from navpy.modules.common import Location, Attitude, Wind
    
    # Or use the app container for DI
    from navpy.app import AppContainer, AppConfig
    container = AppContainer(AppConfig.default())
    vehicle = container.vehicle
"""
# Note: We intentionally don't import submodules at package level
# to avoid circular imports and keep imports fast.
# Use explicit imports like: from navpy.modules.navigation import Navigation

__all__ = [
    "common",
    "vehicle",
    "vision",
    "navigation",
    "nav",
    "comm",
    "mesh",
    "swarm",
]

