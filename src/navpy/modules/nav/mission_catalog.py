from __future__ import annotations

from typing import Optional

from navpy.modules.common.models.location import Location
from navpy.modules.nav.mission_metadata import read_mission_metadata
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.vision_class_profile import get_class_detect_size


class MissionCatalog:
    """Own the current mission's target and scan-band metadata."""

    def __init__(self, vehicle: IVehicle) -> None:
        self._vehicle = vehicle
        self.fallback_delivery_location: Optional[Location] = None
        self.fallback_delivery_location_type: Optional[str] = None
        self.fallback_delivery_location_active = False
        self.smallest_class_id = 0
        self.scan_altitude_rel: Optional[float] = None

    def refresh(self) -> None:
        metadata = read_mission_metadata(self._vehicle)
        self.fallback_delivery_location = metadata.fallback_delivery_location
        self.fallback_delivery_location_type = metadata.fallback_delivery_location_type
        self.scan_altitude_rel = metadata.scan_altitude_rel
        self.smallest_class_id = (
            min(metadata.detect_class_ids, key=get_class_detect_size)
            if metadata.detect_class_ids
            else 0
        )

    def mark_fallback_active(self) -> None:
        self.fallback_delivery_location_active = True

    def clear_navigation_task(self) -> None:
        self.fallback_delivery_location_active = False
