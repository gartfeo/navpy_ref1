"""Resolve active-target review state into focused transition owners."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.nav.confirmed_target_release import ConfirmedTargetRelease
from navpy.modules.nav.nav_state import NavPhaseState, NavState
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.nav.target_presence_review import TargetPresenceReview
from navpy.modules.nav.target_rejection_exit import TargetRejectionExit
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class TargetSelectionPorts:
    active_target: Callable[[], DetectedObject | None]
    target_status: Callable[[DetectedObject], ConfirmationStatus | None]


class ConfirmationStatusDecision:
    """Dispatch target state while leaves own presence, release, and exit."""

    def __init__(
        self,
        selection: TargetSelectionPorts,
        phase: NavPhaseState,
        status_reporter: NavigationStatusReporter,
        presence: TargetPresenceReview,
        confirmed: ConfirmedTargetRelease,
        rejection: TargetRejectionExit,
    ) -> None:
        self._selection = selection
        self._phase = phase
        self._status_reporter = status_reporter
        self._presence = presence
        self._confirmed = confirmed
        self._rejection = rejection

    def dispatch(self) -> bool:
        self._status_reporter.last_ignore_code = 0
        active = self._selection.active_target()
        if active is None:
            self._phase.request(NavState.DETECT)
            return False
        status = self._selection.target_status(active)
        if self._presence.update(active, status):
            return True
        if status == ConfirmationStatus.CONFIRMED:
            self._confirmed.apply(active)
            return False
        return self._rejection.apply(active, status)


__all__ = ["TargetSelectionPorts", "ConfirmationStatusDecision"]
