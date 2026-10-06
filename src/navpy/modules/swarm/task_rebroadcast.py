"""Target advertisement and missing-peer rebroadcast coordination."""

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
from navpy.modules.vision.target_identity import get_target_task_id


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

    def notify_targets(self, targets: list[DetectedObject]) -> None:
        for target in targets:
            task = self._task_from_detection(target)
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
        self._sender.available(tasks)

    def peer_discovered(self, peer_id: int) -> None:
        self._rebroadcast.discover_peer(peer_id, self._rebroadcast_task)

    def restart(
        self,
        *,
        exclude_task_id: int | None = None,
        expected_generation: int | None = None,
    ) -> None:
        self._rebroadcast.restart_available(
            self._rebroadcast_task,
            exclude_task_id=exclude_task_id,
            expected_generation=expected_generation,
        )

    def known_peers(self) -> set[int]:
        return self._rebroadcast.peer_ids()

    def _rebroadcast_task(self, task_id: int, generation: int) -> None:
        plan = self._rebroadcast.prepare(task_id, generation)
        if plan is None:
            return
        self._sender.available([plan.task])
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
    def _task_from_detection(target: DetectedObject) -> TaskMsgData | None:
        target_id = get_target_task_id(target)
        if target_id is None:
            return None
        location = (
            LocationMsgData(
                target.geo.projected_target_location.lat,
                target.geo.projected_target_location.lng,
                target.geo.projected_target_location.alt,
            )
            if target.geo.projected_target_location is not None
            else LocationMsgData(0.0, 0.0, 0.0)
        )
        class_id = target.classification.class_id
        return TaskMsgData(
            task_id=target_id,
            task_type=class_to_task_type(class_id),
            location=location,
            class_id=class_id,
        )
