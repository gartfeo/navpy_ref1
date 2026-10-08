"""Owner-side task bidding and acknowledged assignment coordination."""

from __future__ import annotations

from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.swarm_ack_msg import SwarmAckMsg
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignResponseMsg
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskResponseMsg,
)
from navpy.modules.comm.messages.task_message_data import TaskHandleMsgData
from navpy.modules.swarm.task_ack_timing import ASSIGN_ACK_TIMING
from navpy.modules.swarm.task_auction_confirmation import (
    TaskAssignConfirmation,
)
from navpy.modules.swarm.task_auction_models import (
    AssignConfirmationPorts,
    ResponseVerdict,
    TaskAssignmentPlanner,
    TaskRejectOutcome,
    TaskReservation,
    is_declined_eta,
)
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_msg_refs import MsgRef, msg_ref
from navpy.modules.swarm.task_rebroadcast import TaskRebroadcastCoordinator


PEER_SELECTION_DELAY_S = 2.0

# Step-3 copies, then the release of an unconfirmed reservation.
ASSIGN_REQUEST_SCHEDULE = ASSIGN_ACK_TIMING.request

# (log level, message) per step-4 verdict; a repeat is not logged.
_VERDICT_LOGS = {
    ResponseVerdict.NOT_YOURS: (
        "warning", "Task dispatch {task} not reserved to {peer}.",
    ),
    ResponseVerdict.UNFENCED: (
        "info",
        "Task {task}: response from {peer} precedes its request ack; "
        "asked it to repeat.",
    ),
    ResponseVerdict.CONFIRMED: ("info", "Task {task} accepted by {peer}"),
    ResponseVerdict.REJECTED: ("info", "Task {task} rejected by {peer}"),
    ResponseVerdict.KEPT: (
        "warning",
        "Task {task} stays with {peer}: a confirmed task is never taken back.",
    ),
}


class TaskAuctionCoordinator:
    """Collect bids, reserve globally optimal peers, and process replies."""

    def __init__(
        self,
        state: TaskAuctionState,
        confirmation: TaskAssignConfirmation,
        planner: TaskAssignmentPlanner,
        sender: TaskMessageSender,
        rebroadcast: TaskRebroadcastCoordinator,
        logger: ILogger,
    ) -> None:
        self._state = state
        self._confirmation = confirmation
        self._planner = planner
        self._sender = sender
        self._rebroadcast = rebroadcast
        self._logger = logger
        self._confirmation_ports = AssignConfirmationPorts(
            send_request=sender.assignment_request,
            advertise=sender.available,
            on_due=self._on_confirmation_due,
        )

    def on_available_response(self, message: AvailableTaskResponseMsg) -> None:
        peer_id = message.sender_id
        if not self._state.admits_peer(peer_id):
            self._logger.warning(
                f"Ignoring task offer from unadmitted peer {peer_id}."
            )
            return
        order = msg_ref(message)
        # A bid or a decline reports the peer FREE.
        busy_changed = self._state.observe_peer(
            peer_id, SwarmNodeState.FREE, order,
        )
        answered = False
        for handle in message.tasks:
            answered = self._record_answer(peer_id, handle, order) or answered
        if busy_changed:
            self.replan()
        elif answered:
            self._run_complete_assignment()

    def on_assign_response(self, message: TaskAssignResponseMsg) -> None:
        verdict = self._confirmation.answer_response(message)
        self._sender.ack(message, verdict.ack_status)
        log = _VERDICT_LOGS.get(verdict.verdict)
        if log is not None:
            level, text = log
            getattr(self._logger, level)(
                text.format(task=message.task_id, peer=message.sender_id)
            )
        if verdict.reject is not None:
            self._after_reject(message.task_id, verdict.reject)
        if verdict.busy_changed:
            self.replan()

    def on_request_ack(self, ack: SwarmAckMsg) -> None:
        task_id = self._confirmation.on_request_ack(ack)
        if task_id is not None:
            self._logger.info(
                f"Task {task_id}: request acked by {ack.sender_id}; "
                "awaiting its response."
            )

    def replan(self) -> None:
        """The busy set changed: advertise to free peers and plan again."""
        self._rebroadcast.restart()
        self._run_complete_assignment()

    def _record_answer(
        self,
        peer_id: int,
        handle: TaskHandleMsgData,
        order: Optional[MsgRef],
    ) -> bool:
        if is_declined_eta(handle.time_in_min):
            declined = self._state.record_decline(
                handle.task_id, peer_id, order=order,
            )
            if declined:
                self._logger.info(
                    f"Task {handle.task_id} declined by {peer_id} "
                    "(cannot fly it)"
                )
            return declined
        if not self._state.record_offer(
            handle.task_id,
            peer_id,
            handle,
            self._select_peer_for_task,
            PEER_SELECTION_DELAY_S,
            order=order,
        ):
            self._logger.warning(
                f"Task {handle.task_id}: offer from {peer_id} not recorded "
                "(no available task, invalid ETA, or sent before it was busy)."
            )
            return False
        self._logger.info(
            f"Task {handle.task_id} can be handled by "
            f"{peer_id} in {handle.time_in_min} minutes"
        )
        return True

    def _after_reject(self, task_id: int, outcome: TaskRejectOutcome) -> None:
        if outcome.kind == "retry":
            self._run_retry_assignment(task_id, outcome.generation)
            self._rebroadcast.restart(expected_generation=outcome.generation)
            return
        self._logger.warning(
            f"Task {task_id} assignment failed after "
            f"{outcome.retry_count} retries."
        )
        self._rebroadcast.notify_task_available([outcome.task])
        self._rebroadcast.restart(expected_generation=outcome.generation)
        self._run_complete_assignment(outcome.generation)

    def _select_peer_for_task(self, _task_id: int, generation: int) -> None:
        self._run_complete_assignment(generation)

    def _send_assignment(self, reservation: TaskReservation) -> bool:
        request = self._sender.assignment_request(reservation)
        if request is None:
            return False
        self._confirmation.arm(
            reservation,
            request,
            self._on_confirmation_due,
            ASSIGN_REQUEST_SCHEDULE,
        )
        return True

    def _on_confirmation_due(self, reservation: TaskReservation) -> None:
        outcome = self._confirmation.due(
            reservation,
            self._confirmation_ports,
            ASSIGN_REQUEST_SCHEDULE,
        )
        if outcome.kind == "resent":
            self._logger.warning(
                f"Task {reservation.task_id}: request not acked by "
                f"{reservation.peer_id}; resent it "
                f"({outcome.sends}/{ASSIGN_REQUEST_SCHEDULE.copies})."
            )
            return
        if outcome.kind != "released":
            return
        self._logger.warning(
            f"Task {reservation.task_id}: no confirmed response from "
            f"{reservation.peer_id} after {outcome.sends} requests; "
            f"released and re-advertising (retry {outcome.retry_count})."
        )
        self._rebroadcast.restart(expected_generation=outcome.generation)
        self._run_complete_assignment(outcome.generation)

    def _run_retry_assignment(self, task_id: int, generation: int) -> None:
        sent = self._state.plan_retry_reserve_and_send(
            self._planner,
            generation,
            task_id,
            self._send_assignment,
        )
        if sent is False:
            self._logger.debug(
                f"Task {task_id} retry: no free peer with a remaining bid."
            )

    def _run_complete_assignment(self, generation: int | None = None) -> None:
        if generation is None:
            generation = self._state.current_generation()
        sent = self._state.plan_reserve_and_send_if_complete(
            self._planner,
            generation,
            self._send_assignment,
        )
        if sent is None:
            return
        if not sent:
            self._logger.debug("Global assignment: no free peers.")
            self._rebroadcast.restart()
