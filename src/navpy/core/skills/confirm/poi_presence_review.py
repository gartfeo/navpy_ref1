"""Fresh POI-presence and nonfinal_approach geo-hold review."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from navpy.core.kernel.location import Location
from navpy.core.skills.confirm.confirmation_policy import ConfirmationTimingPolicy
from navpy.core.flow.nav_state import GeoHoldState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.confirm.track_recovery import GeoHoldCoordinator
from navpy.core.ports.detect_data import DetectedObject


@dataclass(frozen=True)
class PoiPresencePorts:
    final_approach_running: Callable[[], bool]
    absolute_location: Callable[[Location | None], Location | None]


class ActivePoiPresenceQuery(Protocol):
    """Fresh active-POI lookup used by presence review."""

    def find_active_poi_detection(self) -> DetectedObject | None: ...


class PoiPresenceReview:
    """Observe freshness without coupling status transitions to source lookup."""

    def __init__(
        self,
        ports: PoiPresencePorts,
        geo_hold: GeoHoldState,
        source: ActivePoiPresenceQuery,
        timing: ConfirmationTimingPolicy,
        geo_coordinator: GeoHoldCoordinator,
    ) -> None:
        self._ports = ports
        self._geo_hold = geo_hold
        self._source = source
        self._timing = timing
        self._geo_coordinator = geo_coordinator

    def update(
        self,
        active: DetectedObject,
        status: ConfirmationStatus | None,
    ) -> bool:
        """True only when a failed geo-hold re-arm tore the navigation task down."""
        fresh = self._source.find_active_poi_detection()
        present = fresh is not None
        self._timing.update_loss_tracking(present)
        if (
            not self._ports.final_approach_running()
            and fresh is not None
            and fresh.geo.projected_poi_location is not None
        ):
            self._geo_hold.last_own_poi_geo = self._ports.absolute_location(
                fresh.geo.projected_poi_location
            )
        return self._geo_coordinator.update(active, status, present)


__all__ = [
    "ActivePoiPresenceQuery",
    "PoiPresencePorts",
    "PoiPresenceReview",
]
