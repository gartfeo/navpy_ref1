from __future__ import annotations

from collections.abc import Sequence
from typing import Optional

from navpy.core.kernel.location import Location
from navpy.core.ports.mission_marker import MissionMarker

# The hub's marker code on the wire (the plugin's META_DEFAULT_DELIVERY_HUB in
# mission_encoding.py; products may not import plugins, so a test pins them equal).
HUB_MARKER_CODE = 3


class MissionCatalog:
    """Own the current mission's POI metadata."""

    def __init__(self) -> None:
        self.default_delivery_hub: Optional[Location] = None
        self.default_delivery_hub_type: Optional[str] = None
        self.default_delivery_hub_active = False

    def update(self, markers: Sequence[MissionMarker]) -> None:
        """Keep the mission's last hub marker, or none."""
        hub: Optional[MissionMarker] = None
        for marker in markers:
            if marker.code == HUB_MARKER_CODE:
                hub = marker
        self.default_delivery_hub = None if hub is None else hub.location
        self.default_delivery_hub_type = None if hub is None else hub.location_type

    def mark_fallback_active(self) -> None:
        self.default_delivery_hub_active = True

    def clear_navigation_task(self) -> None:
        self.default_delivery_hub_active = False
