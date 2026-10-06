import threading
from typing import Callable, Optional, Dict

from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.types import TaskDispatchStatus


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
        self._timer_event: Optional[threading.Timer] = None
        self._rebroadcast_timer: Optional[threading.Timer] = None
        self._lock = threading.RLock()  # Changed to RLock

    def on_peer_available(self, peer_id: int, task_handle: TaskHandleMsgData) -> None:
        """
        Records a peer's availability to handle the task.
        """
        with self._lock:
            self.task_handle_by_peer[peer_id] = task_handle

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
            if self.assigned_peer == responder_id:
                self.assigned_peer = None  # <-- free reservation
            remaining_peers = len(self.task_handle_by_peer)
            if remaining_peers == 0:
                self.cancel_peer_select_timer()
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

    def shutdown(self) -> None:
        """
        Cleans up resources.
        """
        self.cancel_peer_select_timer()
        self.cancel_rebroadcast()
