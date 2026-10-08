"""Resolve active-POI review state into focused transition owners."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.nav.confirmed_poi_release import ConfirmedPoiRelease
from navpy.modules.nav.peer_task_priority import PeerTaskPriority
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.nav.poi_presence_review import PoiPresenceReview
from navpy.modules.nav.poi_rejection_exit import PoiRejectionExit
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class PoiSelectionPorts:
    active_poi: Callable[[], DetectedObject | None]
    poi_status: Callable[[DetectedObject], ConfirmationStatus | None]


class ConfirmationStatusDecision:
    """Dispatch POI state while leaves own presence, release, and exit."""

    def __init__(
        self,
        selection: PoiSelectionPorts,
        phase: NavPhaseState,
        status_reporter: NavigationStatusReporter,
        presence: PoiPresenceReview,
        confirmed: ConfirmedPoiRelease,
        rejection: PoiRejectionExit,
        peer_task: PeerTaskPriority,
    ) -> None:
        self._selection = selection
        self._phase = phase
        self._status_reporter = status_reporter
        self._presence = presence
        self._confirmed = confirmed
        self._rejection = rejection
        self._peer_task = peer_task

    def dispatch(self) -> bool:
        self._status_reporter.last_ignore_code = 0
        active = self._selection.active_poi()
        if active is None:
            self._phase.request(NavState.DETECT)
            return False
        if self._phase.previous is not NavState.NAV and self._peer_task.pending():
            # Not yet approached: the own POI yields to the assigned peer
            # task without RESET, which would release that task.
            self._peer_task.hand_off(active)
            self._phase.request(NavState.DETECT)
            return True
        status = self._selection.poi_status(active)
        if self._presence.update(active, status):
            return True
        if status == ConfirmationStatus.CONFIRMED:
            self._confirmed.apply(active)
            return False
        return self._rejection.apply(active, status)


__all__ = ["PoiSelectionPorts", "ConfirmationStatusDecision"]
