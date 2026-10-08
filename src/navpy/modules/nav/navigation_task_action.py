"""Entry orchestration for a newly selected navigation_task."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_policy import PoiRetryPolicy
from navpy.modules.nav.navigation_speedup import NavigationSpeedupLease
from navpy.modules.nav.mission_navigation import FallbackMissionNavigation
from navpy.modules.nav.nav_state import NavigationTaskState, FinalApproachNavState
from navpy.modules.nav.peer_navigation import PeerNavigationCoordinator
from navpy.modules.nav.peer_task_priority import PeerTaskPriority
from navpy.modules.nav.self_detected_approach import SelfDetectedApproach
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.detector_ports import TrackingCommandPort
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class NavigationTaskAction:
    """Choose the next work source and begin a local navigation_task."""

    def __init__(
        self,
        navigation_task: NavigationTaskState,
        final_approach: FinalApproachNavState,
        confirmation_manager: ConfirmationManager,
        retry: PoiRetryPolicy,
        tracking: TrackingCommandPort,
        speedup: NavigationSpeedupLease,
        peer_navigation: PeerNavigationCoordinator,
        peer_task: PeerTaskPriority,
        fallback_navigation: FallbackMissionNavigation,
        self_approach: SelfDetectedApproach,
        final_approach_active: Callable[[], bool],
        logger: ILogger,
    ) -> None:
        self._navigation_task = navigation_task
        self._final_approach = final_approach
        self._confirmation_manager = confirmation_manager
        self._retry = retry
        self._tracking = tracking
        self._speedup = speedup
        self._peer_navigation = peer_navigation
        self._peer_task = peer_task
        self._fallback_navigation = fallback_navigation
        self._self_approach = self_approach
        self._final_approach_active = final_approach_active
        self._logger = logger

    def handle_new_poi(self, poi: Optional[DetectedObject]) -> None:
        if self._peer_task.pending():
            # The peer task outranks an own POI and the DDH return.
            if poi is not None:
                self._peer_task.hand_off(poi)
            self._peer_navigation.setup()
            return
        if poi is not None:
            if (
                self._navigation_task.peer_navigation
                and not self._final_approach_active()
                and not self._peer_navigation.near_poi()
            ):
                return
            status = self._confirmation_manager.get_status(poi)
            if status in {
                ConfirmationStatus.CONFIRMED,
                ConfirmationStatus.REJECTED,
                ConfirmationStatus.PEER_NOTIFIED,
            }:
                return
            is_reask = status is ConfirmationStatus.TIMEOUT_REJECTED
            if is_reask:
                if not self._retry.can_reask(poi):
                    return
            if self._retry.is_in_poi_cooldown(poi):
                return
            started = self.start(poi)
            if started and status is ConfirmationStatus.DROPPED:
                # Taken up again by the peer approach; a status would keep
                # the confirmation from being asked.
                self._confirmation_manager.clear_status(poi)
            if started and is_reask:
                self._retry.begin_reask(poi)
        elif (
            self._fallback_navigation.should_nav_to_fallback()
            and not self._navigation_task.peer_navigation
        ):
            self._fallback_navigation.setup()

    def start(self, poi: DetectedObject) -> bool:
        poi_id = get_poi_task_id(poi)
        if not self._speedup.apply():
            self._logger.warning(
                f"POI: P{poi_id} deferred until scheduler-rate "
                "rollback verifies",
                key="nav",
            )
            return False
        self._tracking.start_tracking(poi_id or poi.identity.obj_id)
        self._confirmation_manager.set_active_poi(poi)
        self._final_approach.confirmed_recorded = False
        self._final_approach.deferred_record_started_at = None
        self._self_approach.prepare(poi)
        self._logger.info(
            f"POI: P{poi_id} (tracking obj_id={poi.identity.obj_id})",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True


__all__ = ["NavigationTaskAction"]
