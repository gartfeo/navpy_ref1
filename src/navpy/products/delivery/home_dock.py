"""Delivery's home-dock rules, on top of core's task focus.

- The home dock (the DDH in its mission) is never offered to the swarm
  (todo 29). A detection is the home dock when it lies within
  ``DOCK_ASSOCIATION_RADIUS_M`` of the mission's DDH position; using that
  position is the DDH exemption of the dock location rule.
- On the way home, a delivery dock comes before the home dock: it is
  taken like a dock found in the UAV's own zone, and it gets its own
  circle (owner ruling 4, 2026-10-16).

See docs/design/delivery_eval_scenario.md, "Remaining plan", step e5.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel.location import Location
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.skills.search.task_focus import TaskFocus, same_dock


class HomeDockRules:
    """Keep the home dock out of offers, and last on the way home."""

    def __init__(
        self,
        focus: TaskFocus,
        home_dock: Callable[[], Optional[Location]],
        homing: Callable[[], bool],
    ) -> None:
        self._focus = focus
        self._home_dock = home_dock
        self._homing = homing

    def offer_allowed(self, poi: DetectedObject) -> bool:
        return not self.is_home_dock(poi)

    def order(self, pois: list[DetectedObject]) -> list[DetectedObject]:
        """On the way home, delivery docks first; otherwise unchanged."""
        if not self._homing():
            return pois
        return sorted(pois, key=self.is_home_dock)

    def needs_own_orbit(self, poi: DetectedObject) -> bool:
        """A delivery dock taken on the way home needs its own circle."""
        return (
            self._homing()
            and not self._focus.assigned()
            and not self.is_home_dock(poi)
        )

    def is_home_dock(self, poi: DetectedObject) -> bool:
        home = self._home_dock()
        if home is None:
            return False
        return same_dock(self._focus.dock_location(poi), home)


__all__ = ["HomeDockRules"]
