"""Synchronization primitives over :class:`BatonRuntime`, with the semantics of
:mod:`threading` and :mod:`queue` that product code relies on. Ownership
hands off directly on release (FIFO). After a park, each
primitive decides from its own state, never from how the park ended: a
handoff, notify or put can land after a timer fired but before the thread
ran. Every blocking wait is a cancellation checkpoint, on entry too
(:mod:`navpy.sim.lockstep.cancel_scope`); a waiter that raises undoes its
own state, and passes on a lock or permit already handed to it. Product
threads use these only while holding the baton; the coordinator
feeds inputs through :meth:`BatonEvent.deliver_set` (and the queue's
``deliver``, :mod:`navpy.sim.lockstep.baton_queue`) only.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from typing import Any

from navpy.sim.lockstep import cancel_scope
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
        if blocking:
            cancel_scope.checkpoint()
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
        try:
            self._runtime.park("lock", None if timeout < 0 else timeout)
        except BaseException:
            if self._owner is me:  # handed over, then cancelled: pass it on
                self._count = 1
                self._hand_off()
            elif me in self._waiters:
                self._waiters.remove(me)
            raise
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
        self._hand_off()

    def _hand_off(self) -> None:
        """Free the lock, or hand it straight to the first waiter."""
        self._count = 0
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
            with cancel_scope.shielded():  # a condition's reacquire always completes
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
        except BaseException:
            if me in self._waiters:
                self._waiters.remove(me)
            elif self._waiters:  # notified, then cancelled: the next waiter takes it
                self._runtime.wake(self._waiters.popleft())
            raise
        finally:
            notified = me not in self._waiters  # notify() took it off the queue
            if not notified:
                self._waiters.remove(me)
            self._lock._restore(count)
        return notified

    def wait_for(self, predicate: Callable[[], Any], timeout: float | None = None) -> Any:
        self._require_lock()
        cancel_scope.checkpoint()  # even for a true predicate or an expired timeout
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
        cancel_scope.checkpoint()
        if self._flag:
            return True
        self._waiters.append(me)
        try:
            self._runtime.park("event", timeout)
        except BaseException:
            if me in self._waiters:
                self._waiters.remove(me)
            raise
        if me in self._waiters:  # not released by set(): the timer fired
            self._waiters.remove(me)
            return self._flag
        return True  # set() released it, even if cleared since


class BatonSemaphore:
    """A counting semaphore; ``bounded`` refuses a release past the initial value.

    A release hands its permit straight to the first waiter (FIFO), as a lock
    release does.
    """

    def __init__(self, runtime: BatonRuntime, value: int = 1, *, bounded: bool = False) -> None:
        if value < 0:
            raise ValueError("semaphore initial value must be >= 0")
        self._runtime = runtime
        self._value = value
        self._initial = value if bounded else None
        self._waiters: deque[ManagedThread] = deque()

    def acquire(self, blocking: bool = True, timeout: float | None = None) -> bool:
        me = self._runtime.current()
        if not blocking and timeout is not None:
            raise ValueError("can't specify timeout for non-blocking acquire")
        if blocking:
            cancel_scope.checkpoint()
        if self._value > 0:
            self._value -= 1
            return True
        if not blocking:
            return False
        self._waiters.append(me)
        try:
            self._runtime.park("semaphore", timeout)
        except BaseException:
            if me in self._waiters:
                self._waiters.remove(me)
            else:  # handed a permit, then cancelled: pass it on
                self._give(1)
            raise
        if me in self._waiters:  # not handed a permit: the timer fired
            self._waiters.remove(me)
            return False
        return True

    def release(self, n: int = 1) -> None:
        self._runtime.current()
        if n < 1:
            raise ValueError("n must be one or more")
        if self._initial is not None and self._value + n > self._initial:
            raise ValueError("Semaphore released too many times")
        self._give(n)

    def _give(self, n: int) -> None:
        for _ in range(n):
            if self._waiters:
                self._runtime.wake(self._waiters.popleft())
            else:
                self._value += 1

    __enter__ = acquire

    def __exit__(self, *_exc: object) -> None:
        self.release()


def sleep(runtime: BatonRuntime, seconds: float) -> None:
    runtime.park("sleep", max(0.0, seconds))


def yield_now(runtime: BatonRuntime) -> None:
    """Let every thread already runnable go first, at the same ``t_k``."""
    runtime.park("yield", requeue=True)


def join(runtime: BatonRuntime, thread: ManagedThread, timeout: float | None = None) -> bool:
    me = runtime.current()
    if thread is me:
        runtime.fail(f"{me.name} joined itself")
    cancel_scope.checkpoint()
    if not thread.done:
        thread.joiners.append(me)
        try:
            runtime.park("join", timeout)
        finally:
            if me in thread.joiners:
                thread.joiners.remove(me)
    return thread.done


__all__ = [
    "BatonCondition",
    "BatonEvent",
    "BatonLock",
    "BatonSemaphore",
    "join",
    "sleep",
    "yield_now",
]
