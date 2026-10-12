"""Bounded handoff from simulator pose callbacks to the renderer."""

from __future__ import annotations

import threading
from collections import deque
from collections.abc import Callable
from typing import Deque, Generic, Optional, TypeVar


PoseT = TypeVar("PoseT")


class PoseInbox(Generic[PoseT]):
    """Own only queued poses and renderer wake-up coordination."""

    def __init__(self, capacity: int) -> None:
        self._capacity = max(1, int(capacity))
        self._condition = threading.Condition()
        self._queue: Deque[PoseT] = deque()

    @property
    def capacity(self) -> int:
        return self._capacity

    def is_full(self) -> bool:
        with self._condition:
            return len(self._queue) >= self._capacity

    def put(self, pose: PoseT) -> None:
        with self._condition:
            self._queue.append(pose)
            self._condition.notify()

    def wait_and_pop_latest(
        self,
        stop_event: threading.Event,
        preserve: Callable[[PoseT], bool],
    ) -> tuple[Optional[PoseT], list[PoseT]]:
        """Preserve reset boundaries; otherwise select the newest pose."""
        with self._condition:
            self._condition.wait_for(
                lambda: stop_event.is_set() or bool(self._queue)
            )
            if stop_event.is_set():
                return None, []
            boundary_index = next(
                (
                    index
                    for index, pose in enumerate(self._queue)
                    if preserve(pose)
                ),
                None,
            )
            if boundary_index is not None:
                superseded = [
                    self._queue.popleft()
                    for _ in range(boundary_index)
                ]
                selected = self._queue.popleft()
            else:
                selected = self._queue.pop()
                superseded = list(self._queue)
                self._queue.clear()
            self._condition.notify_all()
            return selected, superseded

    def clear(self) -> list[PoseT]:
        with self._condition:
            dropped = list(self._queue)
            self._queue.clear()
            self._condition.notify_all()
            return dropped

    def wake(self) -> None:
        with self._condition:
            self._condition.notify_all()


__all__ = ["PoseInbox"]
