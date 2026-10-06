from __future__ import annotations

from navpy.modules.common.models.location import Location


class SimulationObject:
    """Positioned simulator delivery reference, not a verified recipient."""

    def __init__(self, uid: int, loc_global: Location, height: int,
                 location_type: str | None = None) -> None:
        self.uid = uid
        self.g_loc = loc_global
        self.height = height
        self.location_type = location_type
