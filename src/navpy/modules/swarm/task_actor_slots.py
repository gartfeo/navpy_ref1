"""Thread-safe local-selection and peer-roster state."""

from __future__ import annotations

import threading
from collections.abc import Collection
from typing import Optional

from navpy.modules.comm.messages.task_message_data import TaskAssignMsgData


MAX_REMOTE_PEERS = 2


class SelectedTaskSlot:
    """Own the single peer-assigned task accepted by this UAV."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._selected: Optional[TaskAssignMsgData] = None
        self._owner_id: Optional[int] = None

    def try_accept(self, task: TaskAssignMsgData, owner_id: int) -> bool:
        """Accept into an empty slot; re-accept a resent request idempotently."""
        with self._lock:
            if self._selected is None:
                self._selected = task
                self._owner_id = owner_id
                return True
            return (
                self._owner_id == owner_id
                and self._selected.task_id == task.task_id
            )

    def release_if_held(
        self,
        owner_id: int,
        task_ids: Collection[int],
    ) -> Optional[TaskAssignMsgData]:
        """Drop the held task if its owner lists it as available again."""
        with self._lock:
            if (
                self._selected is None
                or self._owner_id != owner_id
                or self._selected.task_id not in task_ids
            ):
                return None
            released = self._selected
            self._selected = None
            self._owner_id = None
            return released

    def selected(self) -> Optional[TaskAssignMsgData]:
        with self._lock:
            return self._selected

    def clear(self) -> None:
        with self._lock:
            self._selected = None
            self._owner_id = None


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


__all__ = ["MAX_REMOTE_PEERS", "PeerRoster", "SelectedTaskSlot"]
