"""The baton runtime's parts: thread records, the thread table, the timer heap."""

from __future__ import annotations

import heapq
import itertools
import threading
from dataclasses import dataclass, field


@dataclass(eq=False)
class ManagedThread:
    seq: int
    name: str
    go: threading.Semaphore = field(default_factory=lambda: threading.Semaphore(0))
    done: bool = False
    park_token: int = 0
    parked_on: str | None = None
    has_timer: bool = False
    joiners: list["ManagedThread"] = field(default_factory=list)
    os_thread: threading.Thread | None = None
    #: Whether a child made without an explicit ``daemon`` is a daemon (threading's rule).
    daemon: bool = False
    #: The product-facing thread object (``rt.current_thread()``), if any.
    handle: object | None = None
    #: The cancellation scope it inherited at spawn (``cancel_scope``), if any.
    scope: object | None = None


class ThreadTable:
    """The managed threads: identity lookup, reaping and the thread audit."""

    def __init__(self) -> None:
        self._seq = itertools.count(1)
        self._by_ident: dict[int, ManagedThread] = {}
        self._threads: list[ManagedThread] = []
        self.errors: list[BaseException] = []
        self.switches = 0

    def new(self, name: str) -> ManagedThread:
        return ManagedThread(seq=next(self._seq), name=name)

    def add(self, thread: ManagedThread) -> None:
        self._threads.append(thread)
        self._by_ident[thread.os_thread.ident] = thread

    def lookup(self, ident: int) -> ManagedThread | None:
        return self._by_ident.get(ident)

    def reap(self, thread: ManagedThread) -> None:
        """Join a finished thread (it only returns now) and retire it."""
        thread.os_thread.join()
        del self._by_ident[thread.os_thread.ident]
        self._threads.remove(thread)

    def alive(self) -> list[ManagedThread]:
        return [thread for thread in self._threads if not thread.done]

    def __len__(self) -> int:
        return len(self._threads)

    def stray(self, baseline: set[threading.Thread]) -> list[str]:
        """Names of live OS threads that are neither pre-existing nor managed."""
        managed = {thread.os_thread for thread in self._threads}
        return [t.name for t in threading.enumerate() if t not in baseline and t not in managed]

    def deadlocked(self) -> list[str]:
        """Threads waiting without a timer on a lock or a join."""
        return [
            f"{thread.name} ({thread.parked_on})" for thread in self.alive()
            if thread.parked_on in ("lock", "join") and not thread.has_timer
        ]


class TimerHeap:
    """Timers in ``(due seconds, park seq)`` order; stale entries are skipped."""

    def __init__(self) -> None:
        self._park_seq = itertools.count(1)
        self._heap: list[tuple[float, int, ManagedThread, int]] = []

    def push(self, due_s: float, thread: ManagedThread) -> None:
        heapq.heappush(self._heap, (due_s, next(self._park_seq), thread, thread.park_token))

    def any_due(self, now_s: float) -> bool:
        return bool(self._heap) and self._heap[0][0] <= now_s

    def pop_due(self, now_s: float) -> list[ManagedThread]:
        due = []
        while self.any_due(now_s):
            _due, _seq, thread, token = heapq.heappop(self._heap)
            if thread.park_token == token and thread.parked_on is not None:
                due.append(thread)
        return due


__all__ = ["ManagedThread", "ThreadTable", "TimerHeap"]
