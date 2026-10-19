"""Delivery's order of work in DETECT: a peer task, its own POI, then the hub."""

from __future__ import annotations

from typing import Optional

from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.skills.acquire.peer_navigation import PeerNavigationCoordinator
from navpy.core.skills.search.navigation_task_action import NavigationTaskAction
from navpy.core.swarm.peer_task_priority import PeerTaskPriority
from navpy.products.delivery.fallback_mission_navigation import (
    FallbackMissionNavigation,
)


class DeliveryWorkSource:
    """Pick the next work when no POI is active."""

    def __init__(
        self,
        navigation_task: NavigationTaskState,
        peer_task: PeerTaskPriority,
        peer_navigation: PeerNavigationCoordinator,
        fallback_navigation: FallbackMissionNavigation,
        tasks: NavigationTaskAction,
    ) -> None:
        self._navigation_task = navigation_task
        self._peer_task = peer_task
        self._peer_navigation = peer_navigation
        self._fallback_navigation = fallback_navigation
        self._tasks = tasks

    def handle_new_poi(self, poi: Optional[DetectedObject]) -> None:
        if self._peer_task.pending():
            # The peer task outranks an own POI and the DDH return.
            if poi is not None:
                self._peer_task.hand_off(poi)
            self._peer_navigation.setup()
            return
        if poi is not None:
            self._tasks.try_start(poi)
        elif (
            self._fallback_navigation.should_nav_to_fallback()
            and not self._navigation_task.peer_navigation
        ):
            self._fallback_navigation.setup()


__all__ = ["DeliveryWorkSource"]
