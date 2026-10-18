"""Which detected docks a UAV may take or offer, by its role.

- The home dock (the DDH in its mission) is never offered to the swarm
  (todo 29). A detection is the home dock when it lies within
  ``DOCK_ASSOCIATION_RADIUS_M`` of the mission's DDH position; using that
  position is the DDH exemption of the dock location rule.
- On the way home, a delivery dock comes before the home dock: it is
  taken like a dock found in the UAV's own zone (owner ruling 4,
  2026-10-16).
- A UAV with an assigned peer task goes to that dock only: it offers no
  dock, and it takes only a detection at the task's position (owner
  ruling 5, 2026-10-16).

See docs/design/delivery_eval_scenario.md, "Remaining plan", step e5.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.flow.nav_constants import DOCK_ASSOCIATION_RADIUS_M
from navpy.core.kernel.location import Location
from navpy.core.ports.detect_data import DetectedObject


@dataclass(frozen=True)
class DockRolePorts:
    home_dock: Callable[[], Optional[Location]]
    homing: Callable[[], bool]
    assigned: Callable[[], bool]
    assigned_dock: Callable[[], Optional[Location]]
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]


class DockRoles:
    """Decide, per detection, whether this UAV may take or offer it."""

    def __init__(self, ports: DockRolePorts) -> None:
        self._ports = ports

    def may_offer(self, poi: DetectedObject) -> bool:
        if self._ports.assigned():
            return False
        return not self.is_home_dock(poi)

    def may_take(self, poi: DetectedObject) -> bool:
        assigned_dock = self._ports.assigned_dock()
        if assigned_dock is None:
            return True
        return _within(self.dock_location(poi), assigned_dock)

    def leaves_home_orbit(self, poi: DetectedObject) -> bool:
        """A delivery dock taken on the way home needs its own circle."""
        return (
            self._ports.homing()
            and not self._ports.assigned()
            and not self.is_home_dock(poi)
        )

    def order(self, pois: list[DetectedObject]) -> list[DetectedObject]:
        """On the way home, delivery docks first; otherwise unchanged."""
        if not self._ports.homing():
            return pois
        return sorted(pois, key=self.is_home_dock)

    def is_home_dock(self, poi: DetectedObject) -> bool:
        home = self._ports.home_dock()
        if home is None:
            return False
        return _within(self.dock_location(poi), home)

    def dock_location(self, poi: DetectedObject) -> Optional[Location]:
        """The position an offer would carry (PeerPoiNotifier's order)."""
        if poi.geo.projected_poi_location is not None:
            return poi.geo.projected_poi_location
        if (
            self._ports.detector_is_simulation()
            and poi.geo.truth_poi_location is not None
        ):
            return poi.geo.truth_poi_location
        return self._ports.ground_location(poi, allow_fallback=False)


def _within(location: Optional[Location], dock: Location) -> bool:
    if location is None:
        return False
    return location.distance_to(dock) <= DOCK_ASSOCIATION_RADIUS_M


__all__ = ["DockRolePorts", "DockRoles"]
