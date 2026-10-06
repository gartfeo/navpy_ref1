"""Owner-side task bidding and assignment coordination."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignResponseMsg
from navpy.modules.comm.messages.task_availability_msg import (
    AvailableTaskResponseMsg,
)
from navpy.modules.swarm.task_auction_models import TaskAssignmentPlanner
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_rebroadcast import TaskRebroadcastCoordinator


PEER_SELECTION_DELAY_S = 2.0


class TaskAuctionCoordinator:
    """Collect bids, reserve globally optimal peers, and process replies."""

    def __init__(
        self,
        state: TaskAuctionState,
        planner: TaskAssignmentPlanner,
        sender: TaskMessageSender,
        rebroadcast: TaskRebroadcastCoordinator,
        logger: ILogger,
    ) -> None:
        self._state = state
        self._planner = planner
        self._sender = sender
        self._rebroadcast = rebroadcast
        self._logger = logger

    def on_available_response(self, message: AvailableTaskResponseMsg) -> None:
        if not self._state.admits_peer(message.sender_id):
            self._logger.warning(
                f"Ignoring task offer from unadmitted peer {message.sender_id}."
            )
            return
        offer_recorded = False
        for handle in message.tasks:
            if not self._state.record_offer(
                handle.task_id,
                message.sender_id,
                handle,
                self._select_peer_for_task,
                PEER_SELECTION_DELAY_S,
            ):
                self._logger.warning(
                    f"Task {handle.task_id} not found in dispatch list."
                )
                continue
            offer_recorded = True
            self._logger.info(
                f"Task {handle.task_id} can be handled by "
                f"{message.sender_id} in {handle.time_in_min} minutes"
            )
        if offer_recorded:
            self._run_complete_assignment()

    def on_assign_response(self, message: TaskAssignResponseMsg) -> None:
        if message.is_accepted:
            if not self._state.accept(message.task_id, message.sender_id):
                self._logger.warning(
                    f"Task dispatch {message.task_id} not found."
                )
                return
            self._logger.info(
                f"Task {message.task_id} accepted by {message.sender_id}"
            )
            return

        outcome = self._state.reject(message.task_id, message.sender_id)
        if outcome.kind == "missing":
            self._logger.warning(
                f"Task dispatch {message.task_id} not found."
            )
            return
        self._logger.info(
            f"Task {message.task_id} rejected by {message.sender_id}"
        )
        if outcome.kind == "retry":
            self._run_retry_assignment(message.task_id, outcome.generation)
            self._rebroadcast.restart(expected_generation=outcome.generation)
            return

        self._logger.warning(
            f"Task {message.task_id} assignment failed after "
            f"{outcome.retry_count} retries."
        )
        if outcome.task is not None:
            self._rebroadcast.notify_task_available([outcome.task])
        self._rebroadcast.restart(exclude_task_id=message.task_id)

    def _select_peer_for_task(self, _task_id: int, generation: int) -> None:
        self._run_complete_assignment(generation)

    def _run_retry_assignment(self, task_id: int, generation: int) -> None:
        sent = self._state.plan_retry_reserve_and_send(
            self._planner,
            generation,
            task_id,
            self._sender.assignment_request,
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
            self._sender.assignment_request,
        )
        if sent is None:
            return
        if not sent:
            self._logger.debug("Global assignment: no free peers.")
            self._rebroadcast.restart()
