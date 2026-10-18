"""Which detected docks a UAV with a peer task may take or offer.

A UAV with an assigned or pending peer task goes to that dock only: it
offers no dock, and it takes only a detection whose position lies within
``DOCK_ASSOCIATION_RADIUS_M`` of the task's position (owner ruling 5,
2026-10-16). This swarm rule is core's; a mission module adds its own offer
and order rules on top of it in ``PoiSelector``.

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
class TaskFocusPorts:
    assigned: Callable[[], bool]
    assigned_dock: Callable[[], Optional[Location]]
    detector_is_simulation: Callable[[], bool]
    ground_location: Callable[..., Optional[Location]]


class TaskFocus:
    """Keep a UAV with a peer task on that task's dock."""

    def __init__(self, ports: TaskFocusPorts) -> None:
        self._ports = ports

    def assigned(self) -> bool:
        return self._ports.assigned()

    def may_offer(self, poi: DetectedObject) -> bool:
        """A UAV with a peer task offers no dock, whichever it sees."""
        return not self._ports.assigned()

    def may_take(self, poi: DetectedObject) -> bool:
        assigned_dock = self._ports.assigned_dock()
        if assigned_dock is None:
            return True
        return same_dock(self.dock_location(poi), assigned_dock)

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


def same_dock(location: Optional[Location], dock: Location) -> bool:
    """True when a detection's position lies at a known dock."""
    if location is None:
        return False
    return location.distance_to(dock) <= DOCK_ASSOCIATION_RADIUS_M


__all__ = ["TaskFocus", "TaskFocusPorts", "same_dock"]
