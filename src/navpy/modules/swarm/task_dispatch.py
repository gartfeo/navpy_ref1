import threading
from dataclasses import dataclass, field
from typing import Callable, Optional, Dict

from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_msg_refs import MsgRef


@dataclass
class AssignAttempt:
    """One reservation of the task to a peer; a newer one fences it off.

    ``sends`` counts the elapsed request-copy slots (a slot after the
    helper's first ack sends nothing) and ``request_refs`` the copies sent.
    ``floor`` is that first ack: only a response the helper sent after it
    can confirm the task.
    """

    number: int = 0
    sends: int = 0
    request_refs: set[MsgRef] = field(default_factory=set)
    floor: Optional[MsgRef] = None


@dataclass
class PeerAnswers:
    """How admitted peers answered the task's advert, for the owner.

    ``bid_orders`` holds the order of each bid in ``task_handle_by_peer``;
    ``released_to`` is a peer whose reservation was released and that has
    not answered since, so it keeps getting the advert.
    """

    bid_orders: dict[int, Optional[MsgRef]] = field(default_factory=dict)
    declined: set[int] = field(default_factory=set)
    released_to: Optional[int] = None

    def heard_from(self, peer_id: int) -> None:
        if self.released_to == peer_id:
            self.released_to = None

    def forget(self, peer_id: int) -> None:
        self.bid_orders.pop(peer_id, None)
        self.declined.discard(peer_id)


class TaskDispatch:
    """
    Manages the dispatching of a single task.
    """

    def __init__(self, task: TaskMsgData):
        self.task: TaskMsgData = task
        self.status: TaskDispatchStatus = TaskDispatchStatus.AVAILABLE
        self.task_handle_by_peer: Dict[int, TaskHandleMsgData] = {}
        self.assigned_peer: Optional[int] = None
        self.retry_count: int = 0
        self.max_retry_attempts: int = 3
        self.attempt = AssignAttempt()
        self.answers = PeerAnswers()
        self._timer_event: Optional[threading.Timer] = None
        self._rebroadcast_timer: Optional[threading.Timer] = None
        self._confirm_timer: Optional[threading.Timer] = None
        self._lock = threading.RLock()  # Changed to RLock

    def on_peer_available(
        self,
        peer_id: int,
        task_handle: TaskHandleMsgData,
        order: Optional[MsgRef] = None,
    ) -> None:
        """
        Records a peer's bid (its availability to handle the task).
        """
        with self._lock:
            self.task_handle_by_peer[peer_id] = task_handle
            self.answers.forget(peer_id)
            self.answers.bid_orders[peer_id] = order
            self.answers.heard_from(peer_id)

    def on_peer_declined(self, peer_id: int) -> None:
        """Record a peer that cannot fly this task (e.g. fuel)."""
        with self._lock:
            self.task_handle_by_peer.pop(peer_id, None)
            self.answers.forget(peer_id)
            self.answers.declined.add(peer_id)
            self.answers.heard_from(peer_id)

    def begin_attempt(self) -> int:
        """Start a reservation attempt; its number fences older ones."""
        with self._lock:
            self.attempt = AssignAttempt(self.attempt.number + 1)
            self.answers.released_to = None
            return self.attempt.number

    def start_peer_select_timer(self, select_peer_func: Callable[[int], None], time: float) -> None:
        """
        Starts a timer to select a peer after a fixed delay.
        """
        with self._lock:
            if self._timer_event is not None and self._timer_event.is_alive():
                return

            timer: Optional[threading.Timer] = None

            def select_and_clear() -> None:
                with self._lock:
                    if self._timer_event is not timer:
                        return
                    self._timer_event = None
                select_peer_func(self.task.task_id)

            timer = threading.Timer(time, select_and_clear)
            self._timer_event = timer
            self._timer_event.start()

    def cancel_peer_select_timer(self) -> None:
        """
        Cancels the peer selection timer if it is running.
        """
        with self._lock:
            if self._timer_event is not None:
                self._timer_event.cancel()
                self._timer_event = None

    def peer_reject(self, responder_id: int) -> int:
        """
        Handles a peer's rejection of a task.
        """
        with self._lock:
            self.task_handle_by_peer.pop(responder_id, None)
            self.answers.forget(responder_id)
            if self.assigned_peer == responder_id:
                self.assigned_peer = None  # <-- free reservation
            remaining_peers = len(self.task_handle_by_peer)
            if remaining_peers == 0:
                self.cancel_peer_select_timer()
            self.cancel_confirm_timer()
            self.status = TaskDispatchStatus.AVAILABLE
            return remaining_peers

    def peer_accept(self, responder_id: int) -> None:
        """
        Handles a peer's acceptance of a task.
        """
        with self._lock:
            self.assigned_peer = responder_id
            self.status = TaskDispatchStatus.CONFIRMED
            self.cancel_peer_select_timer()
            self.cancel_rebroadcast()
            self.cancel_confirm_timer()

    def set_status(self, status: TaskDispatchStatus) -> None:
        """
        Sets the status of the task dispatch.
        """
        with self._lock:
            self.status = status

    def start_rebroadcast(self, func: Callable[[int], None], interval: float) -> None:
        """
        Starts a one-shot timer that calls func(task_id) after interval seconds.
        The callback is responsible for rescheduling if needed.
        """
        with self._lock:
            self.cancel_rebroadcast()
            timer: Optional[threading.Timer] = None

            def rebroadcast_and_clear() -> None:
                with self._lock:
                    if self._rebroadcast_timer is not timer:
                        return
                    self._rebroadcast_timer = None
                func(self.task.task_id)

            timer = threading.Timer(interval, rebroadcast_and_clear)
            self._rebroadcast_timer = timer
            self._rebroadcast_timer.start()

    def has_active_rebroadcast(self) -> bool:
        with self._lock:
            return self._rebroadcast_timer is not None and self._rebroadcast_timer.is_alive()

    def cancel_rebroadcast(self) -> None:
        """
        Cancels the rebroadcast timer if running.
        """
        with self._lock:
            if self._rebroadcast_timer is not None:
                self._rebroadcast_timer.cancel()
                self._rebroadcast_timer = None

    def start_confirm_timer(self, func: Callable[[int], None], interval: float) -> None:
        """
        Starts a one-shot timer that calls func(task_id) while an assign
        request awaits its response. The callback reschedules if needed.
        """
        with self._lock:
            self.cancel_confirm_timer()
            timer: Optional[threading.Timer] = None

            def confirm_and_clear() -> None:
                with self._lock:
                    if self._confirm_timer is not timer:
                        return
                    self._confirm_timer = None
                func(self.task.task_id)

            timer = threading.Timer(interval, confirm_and_clear)
            self._confirm_timer = timer
            self._confirm_timer.start()

    def cancel_confirm_timer(self) -> None:
        """
        Cancels the assign-confirmation timer if running.
        """
        with self._lock:
            if self._confirm_timer is not None:
                self._confirm_timer.cancel()
                self._confirm_timer = None

    def shutdown(self) -> None:
        """
        Cleans up resources.
        """
        self.cancel_peer_select_timer()
        self.cancel_rebroadcast()
        self.cancel_confirm_timer()
