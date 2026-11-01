"""Resolve active-POI review state into focused transition owners."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.confirm.confirmed_poi_release import (
    ConfirmedPoiRelease,
    ReleaseOutcome,
)
from navpy.core.swarm.peer_task_priority import PeerTaskPriority
from navpy.core.swarm.peer_task_reask import PeerTaskReask
from navpy.core.flow.navigation_task_reset import AutoMissionResume
from navpy.core.flow.nav_state import NavPhaseState, NavState
from navpy.core.flow.nav_status import NavigationStatusReporter
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.confirm.poi_presence_review import PoiPresenceReview
from navpy.core.skills.confirm.poi_rejection_exit import PoiRejectionExit
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


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
        peer_reask: PeerTaskReask,
        resume_auto: AutoMissionResume,
        logger: ILogger,
    ) -> None:
        self._selection = selection
        self._phase = phase
        self._status_reporter = status_reporter
        self._presence = presence
        self._confirmed = confirmed
        self._rejection = rejection
        self._peer_task = peer_task
        self._peer_reask = peer_reask
        self._resume_auto = resume_auto
        self._logger = logger

    def dispatch(self) -> None:
        self._status_reporter.last_ignore_code = 0
        active = self._selection.active_poi()
        if active is None:
            self._phase.request(NavState.DETECT)
            return
        if self._phase.previous is not NavState.NAV and self._peer_task.pending():
            # Not yet approached: the own POI yields to the assigned peer
            # task without RESET, which would release that task.
            self._peer_task.hand_off(active)
            self._phase.request(NavState.DETECT)
            return
        status = self._selection.poi_status(active)
        if self._presence.update(active, status):
            self._resume_after_geo_abort(active)
            return
        if status == ConfirmationStatus.CONFIRMED:
            release = self._confirmed.apply(active)
            if release is ReleaseOutcome.READY:
                self._phase.request(NavState.NAV)
            elif release is ReleaseOutcome.ABORTED:
                self._resume_after_geo_abort(active)
            return
        if self._rejection.apply(active, status):
            if (
                status == ConfirmationStatus.TIMEOUT_REJECTED
                and self._peer_reask.keep(active)
            ):
                # No answer is not "no": stay on the assigned dock.
                return
            self._resume_auto.run()
            self._logger.info(
                "REJECTED: resuming AUTO mission",
                key="nav",
                dest=LogStatusDest.DRONE,
            )

    def _resume_after_geo_abort(self, active: DetectedObject) -> None:
        self._logger.info(
            f"GEO-HOLD abort: P{get_poi_task_id(active)} navigation task torn "
            "down to DETECT (no tracking armed), resuming AUTO mission",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        self._resume_auto.run()


__all__ = ["PoiSelectionPorts", "ConfirmationStatusDecision"]
