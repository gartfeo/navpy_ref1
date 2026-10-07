from __future__ import annotations

from typing import Optional

from navpy.modules.common.models.location import Location
from navpy.modules.nav.mission_metadata import read_mission_metadata
from navpy.modules.vehicle.vehicle_interface import IVehicle


class MissionCatalog:
    """Own the current mission's POI and scan-band metadata."""

    def __init__(self, vehicle: IVehicle) -> None:
        self._vehicle = vehicle
        self.default_delivery_hub: Optional[Location] = None
        self.default_delivery_hub_type: Optional[str] = None
        self.default_delivery_hub_active = False
        self.scan_altitude_rel: Optional[float] = None

    def refresh(self) -> None:
        metadata = read_mission_metadata(self._vehicle)
        self.default_delivery_hub = metadata.default_delivery_hub
        self.default_delivery_hub_type = metadata.default_delivery_hub_type
        self.scan_altitude_rel = metadata.scan_altitude_rel

    def mark_fallback_active(self) -> None:
        self.default_delivery_hub_active = True

    def clear_navigation_task(self) -> None:
        self.default_delivery_hub_active = False
