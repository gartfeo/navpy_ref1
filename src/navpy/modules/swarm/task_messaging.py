"""Swarm task protocol message construction, sending, and routing."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_assignment_msg import (
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
)
from navpy.modules.comm.messages.task_message_data import (
    TaskAssignMsgData,
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_auction_models import TaskReservation
from navpy.modules.swarm.task_ports import TaskMessageBroadcaster


class TaskMessageSender:
    """Build and send task protocol messages through one narrow network port."""

    def __init__(
        self,
        actor_id: int,
        network: TaskMessageBroadcaster,
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._network = network
        self._logger = logger

    def available(self, tasks: list[TaskMsgData]) -> bool:
        return self._send(
            AvailableTaskRequestMsg(sender_id=self._actor_id, tasks=tasks),
            "Broadcast available tasks.",
            "Failed to broadcast available tasks",
        )

    def assignment_request(self, reservation: TaskReservation) -> bool:
        task = reservation.task
        message = TaskAssignRequestMsg(
            sender_id=self._actor_id,
            receiver_id=reservation.peer_id,
            task=TaskAssignMsgData(
                reservation.task_id,
                task.task_type,
                task.location,
                class_id=getattr(task, "class_id", 0),
            ),
        )
        return self._send(
            message,
            f"[Global] Assign request: task {reservation.task_id} "
            f"-> peer {reservation.peer_id}",
            f"Failed to send task assign request to {reservation.peer_id}",
        )

    def available_response(
        self,
        receiver_id: int,
        tasks: list[TaskHandleMsgData],
    ) -> bool:
        return self._send(
            AvailableTaskResponseMsg(
                sender_id=self._actor_id,
                receiver_id=receiver_id,
                tasks=tasks,
            ),
            f"Sent available task response to {receiver_id}",
            f"Failed to send available task response to {receiver_id}",
        )
    def assignment_response(
        self,
        receiver_id: int,
        task_id: int,
        accepted: bool,
    ) -> bool:
        return self._send(
            TaskAssignResponseMsg(
                sender_id=self._actor_id,
                receiver_id=receiver_id,
                task_id=task_id,
                is_accepted=accepted,
            ),
            f"Sent task assignment response for task {task_id} "
            f"to {receiver_id}",
            f"Failed to send task assignment response to {receiver_id}",
        )

    def heartbeat(self) -> bool:
        return self._send(
            SwarmHeartbeatMsg.create_with_meta(self._actor_id),
            None,
            "Failed to broadcast swarm heartbeat",
        )

    def checkin(self) -> bool:
        return self._send(
            CheckInMsg(self._actor_id),
            "CheckIn",
            "Failed to broadcast check-in message",
        )

    def checkout(self, location: LocationMsgData) -> bool:
        return self._send(
            CheckOutMsg(self._actor_id, location),
            f"CheckOut. Location: {location}",
            "Failed to broadcast checkout message",
        )

    def _send(
        self,
        message: MsgABC,
        success: Optional[str],
        failure: str,
    ) -> bool:
        try:
            self._network.broadcast(message)
        except OSError as exc:
            self._logger.error(f"{failure}: {exc}")
            return False
        if success is not None:
            self._logger.info(success)
        return True


class TaskMessageRouter:
    """Apply lifecycle/address filters before data-driven message dispatch."""

    _PEER_TASK_TYPES = {
        MsgType.AVAILABLE_TASK_REQUEST,
        MsgType.AVAILABLE_TASK_RESPONSE,
        MsgType.TASK_ASSIGN_REQUEST,
        MsgType.TASK_ASSIGN_RESPONSE,
    }
    _IGNORED_TYPES = {
        MsgType.LOG_STATUS,
        MsgType.TASK_CONFIRM_REQUEST,
        MsgType.TASK_CONFIRM_RESPONSE,
    }

    def __init__(
        self,
        actor_id: int,
        is_started: Callable[[], bool],
        peer_is_admitted: Callable[[int], bool],
        handlers: Mapping[MsgType, Callable[[MsgABC], None]],
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._is_started = is_started
        self._peer_is_admitted = peer_is_admitted
        self._handlers = dict(handlers)
        self._logger = logger

    def route(self, message: MsgABC) -> None:
        if not self._is_started():
            return
        if message.receiver_id is not None and message.receiver_id != self._actor_id:
            return

        message_type = message.msg_type()
        if (
            message_type in self._PEER_TASK_TYPES
            and not self._peer_is_admitted(message.sender_id)
        ):
            self._logger.warning(
                f"Ignoring {message_type.name} from unadmitted peer "
                f"{message.sender_id}."
            )
            return
        handler = self._handlers.get(message_type)
        if handler is None:
            if message_type not in self._IGNORED_TYPES:
                self._logger.warning(f"No handler for message type: {message_type}")
            return
        try:
            handler(message)
        except OSError as exc:
            self._logger.error(f"Error handling message {message_type}: {exc}")
