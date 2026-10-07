"""Task capability evaluation and inbound participation decisions."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignRequestMsg
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskRequestMsg,
)
from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_actor_slots import SelectedTaskSlot
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_ports import TaskLocationReader


class TaskCapabilityEvaluator:
    """Estimate which advertised tasks the local vehicle can service."""

    def __init__(
        self,
        vehicle: TaskLocationReader,
        estimate_time: Callable[[Location], Optional[float]],
    ) -> None:
        self._vehicle = vehicle
        self._estimate_time = estimate_time

    def has_current_location(self) -> bool:
        return self._vehicle.location(False) is not None

    def evaluate(self, tasks: list[TaskMsgData]) -> list[TaskHandleMsgData]:
        return [
            TaskHandleMsgData(
                task.task_id,
                self._estimate_time(
                    Location(
                        task.location.lat,
                        task.location.lng,
                        task.location.alt,
                        is_absolute=True,
                    )
                ),
            )
            for task in tasks
        ]


class TaskParticipationCoordinator:
    """Respond to task advertisements and assignment requests."""

    def __init__(
        self,
        actor_id: int,
        selection: SelectedTaskSlot,
        evaluator: TaskCapabilityEvaluator,
        sender: TaskMessageSender,
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._selection = selection
        self._evaluator = evaluator
        self._sender = sender
        self._logger = logger

    def on_available_request(self, message: AvailableTaskRequestMsg) -> None:
        if message.sender_id == self._actor_id:
            return
        self._release_if_re_advertised(message)
        if not self._evaluator.has_current_location():
            self._logger.warning(
                f"{self._actor_id}: Location not set. "
                "Cannot send available message."
            )
            return
        if self._selection.selected() is not None:
            self._logger.info("Already handling tasks. Can't handle more tasks.")
            return

        tasks = self._evaluator.evaluate(message.tasks)
        if not tasks:
            self._logger.info("No tasks can handle")
            return

        self._sender.available_response(message.sender_id, tasks)

    def on_assign_request(self, message: TaskAssignRequestMsg) -> None:
        accepted = self._selection.try_accept(message.task, message.sender_id)
        self._sender.assignment_response(
            receiver_id=message.sender_id,
            task_id=message.task.task_id,
            accepted=accepted,
        )

    def _release_if_re_advertised(self, message: AvailableTaskRequestMsg) -> None:
        # An owner advertises only tasks it holds as unassigned, so a held
        # task in its advertisement means the owner released this peer's
        # reservation (e.g. after our accept response was lost).
        released = self._selection.release_if_held(
            message.sender_id,
            {task.task_id for task in message.tasks},
        )
        if released is not None:
            self._logger.warning(
                f"Task {released.task_id} re-advertised by owner "
                f"{message.sender_id}; releasing local assignment."
            )
