"""Fresh target-presence and nonterminal geo-hold review."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.common.models.location import Location
from navpy.modules.nav.confirmation_policy import ConfirmationTimingPolicy
from navpy.modules.nav.nav_state import GeoHoldState
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.nav.track_recovery import GeoHoldCoordinator
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class TargetPresencePorts:
    terminal_active: Callable[[], bool]
    absolute_location: Callable[[Location | None], Location | None]


class ActiveTargetPresenceQuery(Protocol):
    """Fresh active-target lookup used by presence review."""

    def find_active_target_detection(self) -> DetectedObject | None: ...


class TargetPresenceReview:
    """Observe freshness without coupling status transitions to source lookup."""

    def __init__(
        self,
        ports: TargetPresencePorts,
        geo_hold: GeoHoldState,
        source: ActiveTargetPresenceQuery,
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
        fresh = self._source.find_active_target_detection()
        present = fresh is not None
        self._timing.update_loss_tracking(present)
        if (
            not self._ports.terminal_active()
            and fresh is not None
            and fresh.geo.projected_target_location is not None
        ):
            self._geo_hold.last_own_target_geo = self._ports.absolute_location(
                fresh.geo.projected_target_location
            )
        return self._geo_coordinator.update(active, status, present)


__all__ = [
    "ActiveTargetPresenceQuery",
    "TargetPresencePorts",
    "TargetPresenceReview",
]
