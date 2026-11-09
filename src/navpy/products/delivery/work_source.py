"""Delivery's order of work in DETECT: a peer task, its own POI, then the hub.

A failed one-shot sortie returning to the DDH (RecoveryHold.sortie_returning; navpy_recovery.md step 10, item 3)
does no other work: no peer task and no delivery dock, only the hub, flown to from wherever the UAV is. It is flown
again after a hold that reopened in NavPy's LOITER (RecoveryHold.take_return_rearm).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
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
        return_ports: Optional[HubReturnPorts] = None,
    ) -> None:
        self._navigation_task = navigation_task
        self._peer_task = peer_task
        self._peer_navigation = peer_navigation
        self._fallback_navigation = fallback_navigation
        self._tasks = tasks
        self._return = return_ports or HubReturnPorts()

    def handle_new_poi(self, poi: Optional[DetectedObject]) -> None:
        if self._return.returning():
            self._return_to_hub(poi)
            return
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

    def _return_to_hub(self, poi: Optional[DetectedObject]) -> None:
        if poi is not None and self._return.is_home_dock(poi):
            self._tasks.try_start(poi)
        elif self._fallback_navigation.can_return() or self._return.rearmed():
            self._fallback_navigation.setup()


@dataclass(frozen=True)
class HubReturnPorts:
    returning: Callable[[], bool] = lambda: False
    is_home_dock: Callable[[DetectedObject], bool] = lambda _poi: False
    #: Once after a hold reopened in NavPy's LOITER (RecoveryHold.take_return_rearm): fly the return again.
    rearmed: Callable[[], bool] = lambda: False


__all__ = ["DeliveryWorkSource", "HubReturnPorts"]
