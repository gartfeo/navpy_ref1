"""NavPy Modules - The domain logic layer.

This package contains all domain-specific modules organized by responsibility:

Modules:
    common      - Shared types (Location, Attitude, Wind)
    vehicle     - Vehicle abstraction and MAVLink communication
    vision      - Detection, cameras, gimbals, POI tracking
    navigation        - Navigation algorithms, geo-reference, terrain utilities
    nav         - Navigation controller and POI management
    comm        - Network communication (WiFi, serial, MAVLink)
    swarm       - Swarm coordination and task distribution

Each module exposes:
    - Interfaces (ABC with @abstractmethod for enforcement)
    - Implementations
    - Configuration dataclasses (where applicable)
"""
