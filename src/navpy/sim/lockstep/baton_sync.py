"""Synchronization primitives over :class:`BatonRuntime`.

Semantics follow :mod:`threading` and :mod:`queue` where product code relies
on them. Ownership hands off directly on release (FIFO). After a park, each
primitive decides from its own state, never from how the park ended: a
handoff, notify or put can land after a timer fired but before the thread
ran. Product threads use these only while holding the baton; the coordinator
feeds inputs through :meth:`BatonQueue.deliver` only.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread


class BatonLock:
    def __init__(self, runtime: BatonRuntime, *, reentrant: bool = False) -> None:
        self._runtime = runtime
        self._reentrant = reentrant
        self._owner: ManagedThread | None = None
        self._count = 0
        self._waiters: deque[ManagedThread] = deque()

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        me = self._runtime.current()
        if self._owner is None:
            self._owner, self._count = me, 1
            return True
        if self._owner is me:
            if not self._reentrant:
                self._runtime.fail(f"{me.name} re-acquired a plain lock it holds")
            self._count += 1
            return True
        if not blocking or timeout == 0:
            return False
        self._waiters.append(me)
        self._runtime.park("lock", None if timeout < 0 else timeout)
        if self._owner is me:  # handed over by release()
            return True
        self._waiters.remove(me)
        return False

    def release(self) -> None:
        me = self._runtime.current()
        if self._owner is not me:
            if self._runtime.aborted:
                return  # unwinding: an abort interrupted the re-acquire this exit undoes
            self._runtime.fail(f"{me.name} released a lock it does not hold")
        self._count -= 1
        if self._count:
            return
        self._owner = None
        if self._waiters:
            nxt = self._waiters.popleft()
            self._owner, self._count = nxt, 1
            self._runtime.wake(nxt)

    def _release_all(self) -> int:
        count, self._count = self._count, 1
        self.release()
        return count

    def _restore(self, count: int) -> None:
        try:
            self.acquire()
        finally:
            if self._owner is self._runtime.current():
                self._count = count  # even when an abort lands after the handoff

    def locked(self) -> bool:
        return self._owner is not None

    __enter__ = acquire

    def __exit__(self, *_exc: object) -> None:
        self.release()


class BatonCondition:
    def __init__(self, runtime: BatonRuntime, lock: BatonLock | None = None) -> None:
        self._runtime = runtime
        self._lock = lock or BatonLock(runtime, reentrant=True)
        self._waiters: deque[ManagedThread] = deque()

    def __enter__(self) -> bool:
        return self._lock.acquire()

    def __exit__(self, *_exc: object) -> None:
        self._lock.release()

    def _require_lock(self) -> None:
        me = self._runtime.current()
        if self._lock._owner is not me:
            self._runtime.fail(f"{me.name} used a condition without holding its lock")

    def wait(self, timeout: float | None = None) -> bool:
        self._require_lock()
        me = self._runtime.current()
        count = self._lock._release_all()
        self._waiters.append(me)
        try:
            self._runtime.park("condition", timeout)
        finally:
            notified = me not in self._waiters  # notify() took it off the queue
            if not notified:
                self._waiters.remove(me)
            self._lock._restore(count)
        return notified

    def wait_for(self, predicate: Callable[[], Any], timeout: float | None = None) -> Any:
        self._require_lock()
        deadline = None if timeout is None else self._runtime.monotonic() + timeout
        result = predicate()
        while not result:
            remaining = None if deadline is None else deadline - self._runtime.monotonic()
            if remaining is not None and remaining <= 0:
                break
            self.wait(remaining)
            result = predicate()
        return result

    def notify(self, n: int = 1) -> None:
        self._require_lock()
        for _ in range(min(n, len(self._waiters))):
            self._runtime.wake(self._waiters.popleft())

    def notify_all(self) -> None:
        self.notify(len(self._waiters))


class BatonEvent:
    def __init__(self, runtime: BatonRuntime) -> None:
        self._runtime = runtime
        self._flag = False
        self._waiters: deque[ManagedThread] = deque()

    def is_set(self) -> bool:
        return self._flag

    def set(self) -> None:
        self._runtime.current()
        self._set()

    def deliver_set(self) -> None:
        """The coordinator's input path for :meth:`set`."""
        self._runtime.require_coordinator()
        self._set()

    def _set(self) -> None:
        self._flag = True
        while self._waiters:
            self._runtime.wake(self._waiters.popleft())

    def clear(self) -> None:
        self._runtime.current()
        self._flag = False

    def wait(self, timeout: float | None = None) -> bool:
        me = self._runtime.current()
        if self._flag:
            return True
        self._waiters.append(me)
        self._runtime.park("event", timeout)
        if me in self._waiters:  # not released by set(): the timer fired
            self._waiters.remove(me)
            return self._flag
        return True  # set() released it, even if cleared since


class BatonQueue:
    """FIFO queue; :meth:`deliver` is the coordinator's input path."""

    def __init__(self, runtime: BatonRuntime) -> None:
        self._runtime = runtime
        self._items: deque[Any] = deque()
        self._getters: deque[ManagedThread] = deque()

    def put(self, item: Any) -> None:
        self._runtime.current()
        self._append(item)

    def deliver(self, item: Any) -> None:
        self._runtime.require_coordinator()
        self._append(item)

    def _append(self, item: Any) -> None:
        self._items.append(item)
        if self._getters:
            self._runtime.wake(self._getters.popleft())

    def get(self, timeout: float | None = None) -> Any:
        me = self._runtime.current()
        deadline = None if timeout is None else self._runtime.monotonic() + timeout
        while not self._items:
            remaining = None if deadline is None else deadline - self._runtime.monotonic()
            if remaining is not None and remaining <= 0:
                raise TimeoutError("lockstep queue get timed out")
            self._getters.append(me)
            self._runtime.park("queue", remaining)
            if me in self._getters:  # not handed an item: the timer fired
                self._getters.remove(me)
        return self._items.popleft()


def sleep(runtime: BatonRuntime, seconds: float) -> None:
    runtime.park("sleep", max(0.0, seconds))


def join(runtime: BatonRuntime, thread: ManagedThread, timeout: float | None = None) -> bool:
    me = runtime.current()
    if thread is me:
        runtime.fail(f"{me.name} joined itself")
    if not thread.done:
        thread.joiners.append(me)
        runtime.park("join", timeout)
        if me in thread.joiners:
            thread.joiners.remove(me)
    return thread.done


def module_shims(runtime: BatonRuntime) -> tuple[SimpleNamespace, SimpleNamespace]:
    """``time`` and ``threading`` stand-ins for a module under the baton."""
    time_shim = SimpleNamespace(
        monotonic=runtime.monotonic,
        perf_counter=runtime.monotonic,
        sleep=lambda seconds: sleep(runtime, seconds),
    )
    threading_shim = SimpleNamespace(
        Lock=lambda: BatonLock(runtime),
        RLock=lambda: BatonLock(runtime, reentrant=True),
        Condition=lambda lock=None: BatonCondition(runtime, lock),
        Event=lambda: BatonEvent(runtime),
    )
    return time_shim, threading_shim


__all__ = [
    "BatonCondition",
    "BatonEvent",
    "BatonLock",
    "BatonQueue",
    "join",
    "module_shims",
    "sleep",
]
