from __future__ import annotations

from typing import Optional

from navpy.core.kernel.location import Location


class MissionCatalog:
    """Own the current mission's POI metadata."""

    def __init__(self) -> None:
        self.default_delivery_hub: Optional[Location] = None
        self.default_delivery_hub_type: Optional[str] = None
        self.default_delivery_hub_active = False

    def update(self, hub: Optional[Location], hub_type: Optional[str]) -> None:
        self.default_delivery_hub = hub
        self.default_delivery_hub_type = hub_type

    def mark_fallback_active(self) -> None:
        self.default_delivery_hub_active = True

    def clear_navigation_task(self) -> None:
        self.default_delivery_hub_active = False
