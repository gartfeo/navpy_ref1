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
from navpy.modules.comm.messages.msg_meta import MsgMetaProvider
from navpy.modules.comm.messages.swarm_ack_msg import SwarmAckMsg
from navpy.modules.comm.messages.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from navpy.modules.comm.messages.ttl_defaults import get_ttl_ms
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_ack_timing import ack_ttl_ms
from navpy.modules.swarm.task_auction_models import TaskReservation
from navpy.modules.swarm.task_msg_refs import MsgRef, msg_ref
from navpy.modules.swarm.task_ports import TaskMessageBroadcaster


class TaskMessageSender:
    """Build and send task protocol messages through one narrow network port.

    Each send stamps a fresh UID before broadcasting and returns it (None
    when the transport failed), so callers can match later acks to it.
    """

    def __init__(
        self,
        actor_id: int,
        network: TaskMessageBroadcaster,
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._network = network
        self._logger = logger

    def available(self, tasks: list[TaskMsgData]) -> Optional[MsgRef]:
        return self._send(
            AvailableTaskRequestMsg(sender_id=self._actor_id, tasks=tasks),
            "Broadcast available tasks.",
            "Failed to broadcast available tasks",
        )

    def assignment_request(
        self,
        reservation: TaskReservation,
    ) -> Optional[MsgRef]:
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
    ) -> Optional[MsgRef]:
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
    ) -> Optional[MsgRef]:
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

    def ack(self, message: MsgABC, status: int) -> Optional[MsgRef]:
        """Acknowledge ``message`` by its UID, living as long as it does."""
        acked = msg_ref(message)
        acked_type = message.msg_type()
        if acked is None:
            self._logger.warning(
                f"Cannot ack {acked_type.name} from {message.sender_id}: "
                "it carries no message UID."
            )
            return None
        return self._send(
            SwarmAckMsg(
                sender_id=self._actor_id,
                receiver_id=message.sender_id,
                ref_boot_id=acked.boot_id,
                ref_msg_seq=acked.msg_seq,
                ref_msg_type=acked_type.value,
                status=status,
            ),
            f"Ack {acked_type.name} {acked.boot_id}:{acked.msg_seq} "
            f"to {message.sender_id} (status {status})",
            f"Failed to ack {acked_type.name} to {message.sender_id}",
            ttl_ms=ack_ttl_ms(acked_type),
        )

    def heartbeat_message(self, state: SwarmNodeState) -> SwarmHeartbeatMsg:
        """Build and stamp a state report; send it with send_heartbeat()."""
        message = SwarmHeartbeatMsg(self._actor_id, state=int(state))
        self._stamp(message, None)
        return message

    def send_heartbeat(self, message: SwarmHeartbeatMsg) -> Optional[MsgRef]:
        return self._broadcast(
            message,
            None,
            "Failed to broadcast swarm heartbeat",
        )

    def checkin(self) -> Optional[MsgRef]:
        return self._send(
            CheckInMsg(self._actor_id),
            "CheckIn",
            "Failed to broadcast check-in message",
        )

    def checkout(self, location: LocationMsgData) -> Optional[MsgRef]:
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
        *,
        ttl_ms: Optional[int] = None,
    ) -> Optional[MsgRef]:
        self._stamp(message, ttl_ms)
        return self._broadcast(message, success, failure)

    @staticmethod
    def _stamp(message: MsgABC, ttl_ms: Optional[int]) -> None:
        message.set_meta(MsgMetaProvider.get_instance().create_meta(
            get_ttl_ms(message.msg_type()) if ttl_ms is None else ttl_ms
        ))

    def _broadcast(
        self,
        message: MsgABC,
        success: Optional[str],
        failure: str,
    ) -> Optional[MsgRef]:
        try:
            self._network.broadcast(message)
        except OSError as exc:
            self._logger.error(f"{failure}: {exc}")
            return None
        if success is not None:
            self._logger.info(success)
        return msg_ref(message)


class TaskMessageRouter:
    """Apply lifecycle/address filters before data-driven message dispatch."""

    _PEER_TASK_TYPES = {
        MsgType.AVAILABLE_TASK_REQUEST,
        MsgType.AVAILABLE_TASK_RESPONSE,
        MsgType.TASK_ASSIGN_REQUEST,
        MsgType.TASK_ASSIGN_RESPONSE,
        MsgType.SWARM_ACK,
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
