"""POI advertisement and missing-peer rebroadcast coordination."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import (
    TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import class_to_task_type
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_rebroadcast_state import TaskRebroadcastState
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


REBROADCAST_INTERVAL_S = 0.5


class TaskRebroadcastCoordinator:
    """Advertise new tasks and retry only for peers still missing a bid."""

    def __init__(
        self,
        auction: TaskAuctionState,
        rebroadcast: TaskRebroadcastState,
        sender: TaskMessageSender,
        logger: ILogger,
    ) -> None:
        self._auction = auction
        self._rebroadcast = rebroadcast
        self._sender = sender
        self._logger = logger

    def notify_pois(self, pois: list[DetectedObject]) -> None:
        for poi in pois:
            task = self._task_from_detection(poi)
            if task is None:
                continue
            registered = self._auction.register(task)
            if registered is None:
                continue
            _, generation = registered
            self.notify_task_available([task])
            self._rebroadcast.schedule(
                task.task_id,
                generation,
                self._rebroadcast_task,
                REBROADCAST_INTERVAL_S,
            )

    def notify_task_available(self, tasks: list[TaskMsgData]) -> None:
        self._rebroadcast.send_if_available(tasks, self._sender.available)

    def peer_discovered(self, peer_id: int) -> None:
        self._rebroadcast.discover_peer(peer_id, self._rebroadcast_task)

    def peer_heard(self, peer_id: int) -> bool:
        return self._rebroadcast.peer_heard(peer_id)

    def peer_checked_out(self, peer_id: int) -> bool:
        return self._rebroadcast.peer_checked_out(peer_id)

    def expire_silent_peers(self) -> bool:
        return self._rebroadcast.expire_silent_peers()

    def restart(
        self,
        *,
        expected_generation: int | None = None,
    ) -> None:
        self._rebroadcast.restart_available(
            self._rebroadcast_task,
            expected_generation=expected_generation,
        )

    def known_peers(self) -> set[int]:
        return self._rebroadcast.peer_ids()

    def _rebroadcast_task(self, task_id: int, generation: int) -> None:
        plan = self._rebroadcast.prepare(task_id, generation)
        if plan is None:
            return
        # A failed send keeps the schedule; only a reserved task stops it.
        if self._rebroadcast.send_if_available(
            [plan.task],
            self._sender.available,
        ) is None:
            return
        self._logger.info(
            f"Rebroadcast task {task_id}, waiting for "
            f"{plan.missing_count} peers"
        )
        self._rebroadcast.schedule(
            task_id,
            generation,
            self._rebroadcast_task,
            REBROADCAST_INTERVAL_S,
        )

    @staticmethod
    def _task_from_detection(poi: DetectedObject) -> TaskMsgData | None:
        poi_id = get_poi_task_id(poi)
        if poi_id is None:
            return None
        location = (
            LocationMsgData(
                poi.geo.projected_poi_location.lat,
                poi.geo.projected_poi_location.lng,
                poi.geo.projected_poi_location.alt,
            )
            if poi.geo.projected_poi_location is not None
            else LocationMsgData(0.0, 0.0, 0.0)
        )
        class_id = poi.classification.class_id
        return TaskMsgData(
            task_id=poi_id,
            task_type=class_to_task_type(class_id),
            location=location,
            class_id=class_id,
        )
