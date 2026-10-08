"""A peer task assigned to this UAV outranks its own unapproached POI.

One final approach per flight: once the swarm assigns a peer task, an own
POI found but not yet approached is dropped and offered to the other UAVs,
and the DDH return comes after the task (docs/design/
swarm-task-assignment-ack.md, "Priority (nav)").
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData
from navpy.modules.nav.confirmation_manager import (
    ConfirmationManager,
    ConfirmationStatus,
)
from navpy.modules.nav.nav_state import NavigationTaskState
from navpy.modules.nav.peer_poi_notification import PeerPoiNotifier
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id, same_poi_identity


@dataclass(frozen=True)
class PeerTaskPriorityPorts:
    selected_poi: Callable[[], Optional[TaskAssignMsgData]]
    stop_tracking: Callable[[], None]


class PeerTaskPriority:
    """Hand an own POI to the swarm when a peer task is assigned."""

    def __init__(
        self,
        ports: PeerTaskPriorityPorts,
        navigation_task: NavigationTaskState,
        confirmation_manager: ConfirmationManager,
        peer_notifier: PeerPoiNotifier,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._confirmation_manager = confirmation_manager
        self._peer_notifier = peer_notifier
        self._logger = logger

    def pending(self) -> bool:
        """An assigned peer task whose approach has not started yet."""
        return (
            not self._navigation_task.peer_navigation
            and self._ports.selected_poi() is not None
        )

    def hand_off(self, poi: DetectedObject) -> None:
        """Drop an own POI and offer it to the other UAVs."""
        assigned = self._ports.selected_poi()
        active = self._confirmation_manager.active_poi
        if active is not None and same_poi_identity(active, poi):
            self._confirmation_manager.clear_active_poi()
            self._ports.stop_tracking()
        poi_id = get_poi_task_id(poi)
        self._confirmation_manager.clear_status(poi)
        if assigned is not None and poi_id == assigned.task_id:
            # The assigned task's own dock: the peer approach covers it.
            self._confirmation_manager.update_status(
                poi, ConfirmationStatus.PEER_NOTIFIED,
            )
        else:
            self._peer_notifier.notify([poi])
        self._logger.info(
            f"POI: P{poi_id} handed to the swarm; flying peer task "
            f"{assigned.task_id if assigned is not None else '?'} first.",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["PeerTaskPriority", "PeerTaskPriorityPorts"]
