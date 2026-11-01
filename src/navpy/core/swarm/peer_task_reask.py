"""An unanswered confirmation of the assigned dock keeps the peer task.

A confirmation timeout is "no answer", not the operator's "no": while the
dock's re-ask budget lasts, the UAV keeps its assigned peer task, stays in
the peer orbit, and asks again when it sees the dock, as the first round
started (docs/design/swarm-task-assignment-ack.md, "Unanswered confirmation
of the assigned dock").
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.location import Location
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.task_message_data import TaskAssignMsgData
from navpy.core.flow.nav_constants import CONFIRM_REASK_MAX_ATTEMPTS
from navpy.core.flow.nav_state import GeoHoldState, NavigationTaskState
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


@dataclass(frozen=True)
class PeerTaskReaskPorts:
    selected_poi: Callable[[], Optional[TaskAssignMsgData]]
    can_reask: Callable[[DetectedObject], bool]
    clear_detections: Callable[[], None]
    # Geo pointing as the peer approach's setup arms it (PeerGeoTracker).
    point_camera_at: Callable[[Location], bool]
    stop_camera_pointing: Callable[[], None]


class PeerTaskReask:
    """Keep the assigned task after a timeout while re-asks are left.

    The budget is the own-POI one (``CONFIRM_REASK_MAX_ATTEMPTS`` more
    rounds), counted per task as well as per POI id: the dock may come back
    under a new id, which starts with a fresh POI budget.
    """

    def __init__(
        self,
        ports: PeerTaskReaskPorts,
        navigation_task: NavigationTaskState,
        geo_hold: GeoHoldState,
        orbit_approach: bool,
        logger: ILogger,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._geo_hold = geo_hold
        self._orbit_approach = orbit_approach
        self._logger = logger

    def keep(self, active: DetectedObject) -> bool:
        """True when the task is kept; the caller then leaves AUTO alone."""
        task = self._ports.selected_poi()
        if task is None or not self._navigation_task.peer_navigation:
            return False
        poi_id = get_poi_task_id(active)
        self._navigation_task.peer_unanswered_rounds += 1
        if (
            self._navigation_task.peer_unanswered_rounds
            > CONFIRM_REASK_MAX_ATTEMPTS
            or not self._ports.can_reask(active)
        ):
            self._logger.warning(
                f"POI: P{poi_id} got no answer after every re-ask; peer task "
                f"{task.task_id} released locally, no automatic recovery",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
            return False
        self._ports.clear_detections()
        self._geo_hold.active = False
        self._geo_hold.last_own_poi_geo = None
        self._geo_hold.acquisition_log_bucket = None
        dock = self._navigation_task.peer_poi_location
        if self._orbit_approach and dock is not None:
            # Point the camera at the task's dock again, as setup did.
            self._ports.point_camera_at(dock)
        else:
            self._ports.stop_camera_pointing()
        self._logger.info(
            f"POI: P{poi_id} got no answer; asking again for peer task "
            f"{task.task_id} (kept)",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        return True


__all__ = ["PeerTaskReask", "PeerTaskReaskPorts"]
