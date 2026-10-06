"""Entry orchestration for a newly selected navigation_task."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_policy import TargetRetryPolicy
from navpy.modules.nav.navigation_speedup import NavigationSpeedupLease
from navpy.modules.nav.mission_navigation import FallbackMissionNavigation
from navpy.modules.nav.nav_state import NavigationTaskState, TerminalNavState
from navpy.modules.nav.peer_navigation import PeerNavigationCoordinator
from navpy.modules.nav.self_detected_approach import SelfDetectedApproach
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vision.detector_ports import TrackingCommandPort
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


class NavigationTaskAction:
    """Choose the next work source and begin a local navigation_task."""

    def __init__(
        self,
        navigation_task: NavigationTaskState,
        terminal: TerminalNavState,
        confirmation_manager: ConfirmationManager,
        retry: TargetRetryPolicy,
        tracking: TrackingCommandPort,
        speedup: NavigationSpeedupLease,
        peer_navigation: PeerNavigationCoordinator,
        fallback_navigation: FallbackMissionNavigation,
        self_approach: SelfDetectedApproach,
        terminal_active: Callable[[], bool],
        logger: ILogger,
    ) -> None:
        self._navigation_task = navigation_task
        self._terminal = terminal
        self._confirmation_manager = confirmation_manager
        self._retry = retry
        self._tracking = tracking
        self._speedup = speedup
        self._peer_navigation = peer_navigation
        self._fallback_navigation = fallback_navigation
        self._self_approach = self_approach
        self._terminal_active = terminal_active
        self._logger = logger

    def handle_new_target(self, target: Optional[DetectedObject]) -> None:
        if target is not None:
            if (
                self._navigation_task.peer_navigation
                and not self._terminal_active()
                and not self._peer_navigation.near_target()
            ):
                return
            status = self._confirmation_manager.get_status(target)
            if status in {
                ConfirmationStatus.CONFIRMED,
                ConfirmationStatus.REJECTED,
                ConfirmationStatus.PEER_NOTIFIED,
            }:
                return
            is_reask = status is ConfirmationStatus.TIMEOUT_REJECTED
            if is_reask:
                if not self._retry.can_reask(target):
                    return
            if self._retry.is_in_target_cooldown(target):
                return
            if self.start(target) and is_reask:
                self._retry.begin_reask(target)
        elif (
            self._peer_navigation.has_assignment()
            and not self._navigation_task.peer_navigation
        ):
            self._peer_navigation.setup()
        elif (
            self._fallback_navigation.should_nav_to_fallback()
            and not self._navigation_task.peer_navigation
        ):
            self._fallback_navigation.setup()

    def start(self, target: DetectedObject) -> bool:
        target_id = get_target_task_id(target)
        if not self._speedup.apply():
            self._logger.warning(
                f"TARGET: T{target_id} deferred until scheduler-rate "
                "rollback verifies",
                key="nav",
            )
            return False
        self._tracking.start_tracking(target_id or target.identity.obj_id)
        self._confirmation_manager.set_active_target(target)
        self._terminal.confirmed_recorded = False
        self._terminal.deferred_record_started_at = None
        self._self_approach.prepare(target)
        self._logger.info(
            f"TARGET: T{target_id} (tracking obj_id={target.identity.obj_id})",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True


__all__ = ["NavigationTaskAction"]
