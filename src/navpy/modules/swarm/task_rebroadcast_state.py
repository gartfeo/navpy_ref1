"""Peer-aware rebroadcast scheduling over shared auction state."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.modules.comm.messages.task_message_data import TaskMsgData
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import (
    TaskRebroadcastPlan,
    _TaskAuctionStore,
)
from navpy.modules.swarm.task_auction_queries import busy_peers
from navpy.modules.swarm.task_msg_refs import MsgRef


class TaskRebroadcastState:
    """Own peer-aware rebroadcast scheduling over the shared task store."""

    def __init__(self, store: _TaskAuctionStore) -> None:
        self._store = store

    def discover_peer(
        self,
        peer_id: int,
        callback: Callable[[int, int], None],
    ) -> bool:
        with self._store.lock:
            if self._store.closed:
                return False
            is_new = self._store.peers.add(peer_id)
        if is_new:
            self.restart_available(callback)
        return is_new

    def schedule(
        self,
        task_id: int,
        generation: int,
        callback: Callable[[int, int], None],
        delay_s: float,
    ) -> None:
        with self._store.lock:
            if self._store.closed or generation != self._store.generation:
                return
            if not self._store.peers.snapshot():
                return
            dispatch = self._store.dispatches.get(task_id)
            if dispatch is None or dispatch.status != TaskDispatchStatus.AVAILABLE:
                return
            dispatch.start_rebroadcast(
                lambda current_id: callback(current_id, generation),
                delay_s,
            )

    def prepare(
        self,
        task_id: int,
        generation: int,
    ) -> Optional[TaskRebroadcastPlan]:
        with self._store.lock:
            if self._store.closed or generation != self._store.generation:
                return None
            dispatch = self._store.dispatches.get(task_id)
            if dispatch is None or dispatch.status != TaskDispatchStatus.AVAILABLE:
                return None
            missing = (
                self._store.peers.snapshot()
                - busy_peers(self._store.dispatches)
                - set(dispatch.task_handle_by_peer)
            )
            if not missing:
                dispatch.cancel_rebroadcast()
                return None
            return TaskRebroadcastPlan(
                task_id=task_id,
                task=dispatch.task,
                missing_count=len(missing),
                generation=generation,
            )

    def restart_available(
        self,
        callback: Callable[[int, int], None],
        *,
        exclude_task_id: Optional[int] = None,
        expected_generation: Optional[int] = None,
    ) -> None:
        with self._store.lock:
            if (
                self._store.closed
                or (
                    expected_generation is not None
                    and expected_generation != self._store.generation
                )
            ):
                return
            peers = self._store.peers.snapshot()
            if not peers:
                return
            busy = busy_peers(self._store.dispatches)
            generation = self._store.generation
            for task_id, dispatch in self._store.dispatches.items():
                if task_id == exclude_task_id:
                    continue
                if (
                    dispatch.status != TaskDispatchStatus.AVAILABLE
                    or dispatch.has_active_rebroadcast()
                ):
                    continue
                missing = peers - busy - set(dispatch.task_handle_by_peer)
                if missing:
                    dispatch.start_rebroadcast(
                        lambda current_id, g=generation: callback(current_id, g),
                        0.0,
                    )

    def send_if_available(
        self,
        tasks: list[TaskMsgData],
        send: Callable[[list[TaskMsgData]], Optional[MsgRef]],
    ) -> Optional[bool]:
        """Advertise only still-AVAILABLE tasks, under the store lock.

        Assign requests are also sent under this lock, so an advertisement
        can never follow a reservation's request: a peer reads its held
        task in an owner's advertisement as a release. Returns None when
        nothing is AVAILABLE, else whether the send succeeded.
        """
        with self._store.lock:
            if self._store.closed:
                return None
            available = [
                task
                for task in tasks
                if (dispatch := self._store.dispatches.get(task.task_id))
                is not None
                and dispatch.status is TaskDispatchStatus.AVAILABLE
            ]
            if not available:
                return None
            return send(available) is not None

    def peer_ids(self) -> set[int]:
        return self._store.peers.snapshot()


__all__ = ["TaskRebroadcastState"]
