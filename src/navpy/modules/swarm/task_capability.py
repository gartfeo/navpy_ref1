"""Task capability evaluation and inbound participation decisions."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignRequestMsg
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskRequestMsg,
)
from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.common.models.location import Location
from navpy.modules.swarm.task_ack_routing import acked_ref
from navpy.modules.swarm.task_actor_slots import (
    AckKind,
    RequestDecision,
    RequestKind,
    SelectedTaskSlot,
)
from navpy.modules.swarm.task_assign_reply import AssignReplies
from navpy.modules.swarm.task_auction_models import DECLINED_ETA_MIN
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_msg_refs import msg_ref
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
        """One answer per task: its ETA, or a decline it cannot fly."""
        return [
            TaskHandleMsgData(task.task_id, self._eta(task.location))
            for task in tasks
        ]

    def can_fly(self, location: LocationMsgData) -> bool:
        return self._eta(location) != DECLINED_ETA_MIN

    def _eta(self, location: LocationMsgData) -> float:
        # FlyEstimator answers -1 (or None) when the battery or the wind
        # rules the flight out.
        eta = self._estimate_time(
            Location(location.lat, location.lng, location.alt, is_absolute=True)
        )
        if type(eta) not in {int, float} or not math.isfinite(eta) or eta < 0:
            return DECLINED_ETA_MIN
        return eta


class TaskParticipationCoordinator:
    """Answer task adverts, assignment requests and their acks as a helper."""

    def __init__(
        self,
        actor_id: int,
        lock: threading.RLock,
        selection: SelectedTaskSlot,
        replies: AssignReplies,
        evaluator: TaskCapabilityEvaluator,
        sender: TaskMessageSender,
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._lock = lock
        self._selection = selection
        self._replies = replies
        self._evaluator = evaluator
        self._sender = sender
        self._logger = logger
        self._decisions: dict[
            RequestKind,
            Callable[[int, RequestDecision, int], None],
        ] = {
            RequestKind.WAIT: self._wait,
            RequestKind.REPEAT: lambda _owner, decision, _task: (
                self._replies.answer_again(decision.held)
            ),
            RequestKind.REJECT: self._reject,
        }

    def on_available_request(self, message: AvailableTaskRequestMsg) -> None:
        owner_id = message.sender_id
        if owner_id == self._actor_id:
            return
        # Bids are stamped under the actor lock, in order with step-4 copies.
        with self._lock:
            dropped = self._selection.on_advert(
                msg_ref(message),
                {task.task_id for task in message.tasks},
            )
            if dropped is not None:
                self._replies.stop_accept()
                self._logger.warning(
                    f"Task {dropped.task.task_id} re-advertised by owner "
                    f"{owner_id}; dropping its offer."
                )
            if self._selection.node_state() is not SwarmNodeState.FREE:
                self._logger.info(
                    "Busy (holding a task or flying a final approach); "
                    "not bidding."
                )
                return
            if not self._evaluator.has_current_location():
                self._logger.warning(
                    f"{self._actor_id}: Location not set. "
                    "Cannot send available message."
                )
                return
            tasks = self._evaluator.evaluate(message.tasks)
            if not tasks:
                self._logger.info("No tasks can handle")
                return
            self._sender.available_response(owner_id, tasks)

    def on_assign_request(self, message: TaskAssignRequestMsg) -> None:
        owner_id = message.sender_id
        task = message.task
        request = msg_ref(message)
        if request is None:
            # Mixed versions are unsupported: an offer without a UID cannot
            # be acked, so it gets one reject.
            self._sender.assignment_response(owner_id, task.task_id, False)
            return
        with self._lock:
            # The ack is stamped before any step-4 copy answering it.
            self._sender.ack(message, ACK_STATUS_RECEIVED)
            decision = self._selection.on_request(
                task,
                request,
                self._evaluator.can_fly(task.location),
            )
            self._decisions[decision.kind](owner_id, decision, task.task_id)

    def on_response_ack(self, ack: SwarmAckMsg) -> None:
        """Apply the owner's ack of one of this UAV's step-4 copies."""
        acked = acked_ref(ack)
        sender = msg_ref(ack)
        if sender is None:
            return
        with self._lock:
            if self._replies.on_reject_ack(acked, ack.status):
                return
            kind, held = self._selection.on_ack(acked, ack.status, sender)
            if kind is AckKind.NONE:
                return
            self._replies.stop_accept()
            if kind is AckKind.ASSIGNED:
                self._logger.info(
                    f"Task {held.task.task_id} assigned by owner "
                    f"{held.owner.owner_id}"
                )
                return
            self._logger.warning(
                f"Task {held.task.task_id}: owner {held.owner.owner_id} "
                "no longer offers it; dropping it."
            )

    def set_approaching(self, approaching: bool) -> None:
        """Nav's own final approach drops a WAITING offer; the owner retries."""
        with self._lock:
            dropped = self._selection.set_approaching(approaching)
            if dropped is None:
                return
            self._replies.stop_accept()
            self._replies.start_reject(*dropped.key)
            self._logger.warning(
                f"Own final approach started; rejecting task "
                f"{dropped.task.task_id} from owner {dropped.owner.owner_id}."
            )

    def close(self) -> None:
        """Stop answering for good (actor shutdown)."""
        self._replies.close()

    def _wait(self, owner_id: int, decision: RequestDecision, task_id: int) -> None:
        self._replies.start_accept(decision.held)
        self._logger.info(
            f"Task {task_id} offered by owner {owner_id}; "
            "waiting for its confirmation."
        )

    def _reject(self, owner_id: int, decision: RequestDecision, task_id: int) -> None:
        self._replies.start_reject(owner_id, task_id)
        self._logger.info(
            f"Task {task_id} from owner {owner_id} rejected: {decision.reason}."
        )


__all__ = ["TaskCapabilityEvaluator", "TaskParticipationCoordinator"]
