"""A FIFO queue over :class:`BatonRuntime` with the :class:`queue.Queue` surface.

``put`` and ``get`` block, time out and raise :class:`queue.Full` and
:class:`queue.Empty` as the standard library does; ``task_done`` and ``join``
count unfinished items. A wait keeps one absolute deadline across contention
and decides from the queue's state after each park (``baton_sync``).
:meth:`BatonQueue.deliver` is the coordinator's input path: an input never
blocks and is never dropped, so ``maxsize`` bounds only ``put``.
"""

from __future__ import annotations

import queue
from collections import deque
from typing import Any

from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread


class BatonQueue:
    def __init__(self, runtime: BatonRuntime, maxsize: int = 0) -> None:
        self._runtime = runtime
        self._maxsize = maxsize
        self._items: deque[Any] = deque()
        self._getters: deque[ManagedThread] = deque()
        self._putters: deque[ManagedThread] = deque()
        self._joiners: deque[ManagedThread] = deque()
        self._unfinished = 0

    def qsize(self) -> int:
        return len(self._items)

    def empty(self) -> bool:
        return not self._items

    def full(self) -> bool:
        return 0 < self._maxsize <= len(self._items)

    def put(self, item: Any, block: bool = True, timeout: float | None = None) -> None:
        me = self._runtime.current()
        deadline = self._deadline(timeout) if block else None
        while self.full():
            remaining = None if deadline is None else deadline - self._runtime.monotonic()
            if not block or (remaining is not None and remaining <= 0):
                raise queue.Full
            self._putters.append(me)
            self._runtime.park("queue", remaining)
            if me in self._putters:  # not woken by a get: the timer fired
                self._putters.remove(me)
        self._append(item)

    def put_nowait(self, item: Any) -> None:
        self.put(item, block=False)

    def deliver(self, item: Any) -> None:
        self._runtime.require_coordinator()
        self._append(item)

    def get(self, block: bool = True, timeout: float | None = None) -> Any:
        me = self._runtime.current()
        deadline = self._deadline(timeout) if block else None
        while not self._items:
            remaining = None if deadline is None else deadline - self._runtime.monotonic()
            if not block or (remaining is not None and remaining <= 0):
                raise queue.Empty
            self._getters.append(me)
            self._runtime.park("queue", remaining)
            if me in self._getters:  # not handed an item: the timer fired
                self._getters.remove(me)
        item = self._items.popleft()
        if self._putters:
            self._runtime.wake(self._putters.popleft())
        return item

    def get_nowait(self) -> Any:
        return self.get(block=False)

    def task_done(self) -> None:
        self._runtime.current()
        if self._unfinished <= 0:
            raise ValueError("task_done() called too many times")
        self._unfinished -= 1
        if not self._unfinished:
            while self._joiners:
                self._runtime.wake(self._joiners.popleft())

    def join(self) -> None:
        me = self._runtime.current()
        while self._unfinished:
            self._joiners.append(me)
            self._runtime.park("queue")
            if me in self._joiners:
                self._joiners.remove(me)

    def _append(self, item: Any) -> None:
        self._items.append(item)
        self._unfinished += 1
        if self._getters:
            self._runtime.wake(self._getters.popleft())

    def _deadline(self, timeout: float | None) -> float | None:
        if timeout is None:
            return None
        if timeout < 0:
            raise ValueError("'timeout' must be a non-negative number")
        return self._runtime.monotonic() + timeout


__all__ = ["BatonQueue"]
