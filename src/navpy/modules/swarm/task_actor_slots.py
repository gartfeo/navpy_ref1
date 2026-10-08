"""Thread-safe local-selection and peer-roster state."""

from __future__ import annotations

import threading
from collections.abc import Collection
from typing import Optional

from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData


MAX_REMOTE_PEERS = 2

# (boot_id, msg_seq) of a received message; None when it carried no meta.
MessageOrder = Optional[tuple[int, int]]


def _sent_before(message: MessageOrder, reference: MessageOrder) -> bool:
    """True only when both come from one sender boot and message is older."""
    return (
        message is not None
        and reference is not None
        and message[0] == reference[0]
        and message[1] < reference[1]
    )


class SelectedTaskSlot:
    """Own the single peer-assigned task accepted by this UAV."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._selected: Optional[TaskAssignMsgData] = None
        self._owner_id: Optional[int] = None
        self._accepted_order: MessageOrder = None

    def try_accept(
        self,
        task: TaskAssignMsgData,
        owner_id: int,
        order: MessageOrder = None,
    ) -> bool:
        """Accept into an empty slot; re-accept a resent request idempotently."""
        with self._lock:
            if self._selected is None:
                self._selected = task
                self._owner_id = owner_id
                self._accepted_order = order
                return True
            if (
                self._owner_id != owner_id
                or self._selected.task_id != task.task_id
            ):
                return False
            if _sent_before(self._accepted_order, order):
                self._accepted_order = order
            return True

    def release_if_held(
        self,
        owner_id: int,
        task_ids: Collection[int],
        order: MessageOrder = None,
    ) -> Optional[TaskAssignMsgData]:
        """Drop the held task if its owner lists it as available again.

        An advertisement the owner sent before the accepted request (link
        reordering) is ignored.
        """
        with self._lock:
            if (
                self._selected is None
                or self._owner_id != owner_id
                or self._selected.task_id not in task_ids
                or _sent_before(order, self._accepted_order)
            ):
                return None
            released = self._selected
            self.clear()
            return released

    def selected(self) -> Optional[TaskAssignMsgData]:
        with self._lock:
            return self._selected

    def clear(self) -> None:
        with self._lock:
            self._selected = None
            self._owner_id = None
            self._accepted_order = None


class PeerRoster:
    """Own the set of swarm peers discovered from heartbeats."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._peers: set[int] = set()

    def add(self, peer_id: int) -> bool:
        with self._lock:
            if peer_id in self._peers or len(self._peers) >= MAX_REMOTE_PEERS:
                return False
            self._peers.add(peer_id)
            return True

    def contains(self, peer_id: int) -> bool:
        with self._lock:
            return peer_id in self._peers

    def snapshot(self) -> set[int]:
        with self._lock:
            return set(self._peers)

    def is_full(self) -> bool:
        with self._lock:
            return len(self._peers) >= MAX_REMOTE_PEERS

    def clear(self) -> None:
        with self._lock:
            self._peers.clear()


__all__ = [
    "MAX_REMOTE_PEERS",
    "MessageOrder",
    "PeerRoster",
    "SelectedTaskSlot",
]
