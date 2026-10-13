"""The threads facet of :class:`LockstepRuntime`: primitives and managed threads.

:class:`LockstepThread` has the :class:`threading.Thread` surface the product
uses (target, args, kwargs, name, daemon; ``start``, ``join(timeout)``,
``is_alive``, ``ident``) and runs as a managed thread of the baton.
:class:`LockstepTimer` is the standard library's ``Timer`` on the baton: one
event wait of ``interval``, so it fires at the first tick at or after its float
deadline. ``get_ident`` and ``ident`` are creation sequence numbers, never OS
idents, so they repeat run to run.
"""

from __future__ import annotations

import itertools
import threading
from collections.abc import Callable, Iterable, Mapping
from typing import Any

from navpy.sim.lockstep.baton import BatonRuntime, ManagedThread
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import (
    BatonCondition,
    BatonEvent,
    BatonLock,
    BatonSemaphore,
    join,
)


class LockstepThread:
    def __init__(
        self,
        threads: "LockstepThreads",
        group: None = None,
        target: Callable[..., object] | None = None,
        name: str | None = None,
        args: Iterable[Any] = (),
        kwargs: Mapping[str, Any] | None = None,
        *,
        daemon: bool | None = None,
    ) -> None:
        if group is not None:
            raise ValueError("group argument must be None for now")
        self._threads = threads
        self._target = target
        self._args = tuple(args)
        self._kwargs = dict(kwargs or {})
        self._name = str(name) if name is not None else threads.new_name(target)
        self._daemon = threads.inherited_daemon() if daemon is None else bool(daemon)
        self._managed: ManagedThread | None = None

    @property
    def name(self) -> str:
        return self._name

    @name.setter
    def name(self, value: str) -> None:
        self._name = str(value)

    @property
    def daemon(self) -> bool:
        return self._daemon

    @daemon.setter
    def daemon(self, value: bool) -> None:
        if self._managed is not None:
            raise RuntimeError("cannot set daemon status of active thread")
        self._daemon = bool(value)

    @property
    def ident(self) -> int | None:
        return None if self._managed is None else self._managed.seq

    def start(self) -> None:
        if self._managed is not None:
            raise RuntimeError("threads can only be started once")
        self._managed = self._threads.baton.spawn(self.run, name=self._name)
        self._managed.daemon = self._daemon
        self._managed.handle = self

    def run(self) -> None:
        try:
            if self._target is not None:
                self._target(*self._args, **self._kwargs)
        finally:
            del self._target, self._args, self._kwargs

    def join(self, timeout: float | None = None) -> None:
        if self._managed is None:
            raise RuntimeError("cannot join thread before it is started")
        join(self._threads.baton, self._managed, timeout)

    def is_alive(self) -> bool:
        return self._managed is not None and not self._managed.done

    def adopt(self, managed: ManagedThread) -> None:
        """Become the handle of a thread the runtime spawned directly."""
        self._managed = managed
        managed.handle = self

    def __repr__(self) -> str:
        state = "started" if self.is_alive() else ("stopped" if self._managed else "initial")
        return f"<LockstepThread({self._name}, {state} {self.ident})>"


class LockstepTimer(LockstepThread):
    """Call ``function`` after ``interval`` seconds unless cancelled first."""

    def __init__(
        self,
        threads: "LockstepThreads",
        interval: float,
        function: Callable[..., object],
        args: Iterable[Any] | None = None,
        kwargs: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(threads)
        self.interval = interval
        self.function = function
        self.args = tuple(args) if args is not None else ()
        self.kwargs = dict(kwargs) if kwargs is not None else {}
        self.finished = BatonEvent(threads.baton)

    def cancel(self) -> None:
        self.finished.set()

    def run(self) -> None:
        self.finished.wait(self.interval)
        if not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
        self.finished.set()


class LockstepThreads:
    """``ThreadsPort`` on a baton: every primitive parks a managed thread."""

    def __init__(self, baton: BatonRuntime) -> None:
        self.baton = baton
        self._names = itertools.count(1)

    def Lock(self) -> BatonLock:
        return BatonLock(self.baton)

    def RLock(self) -> BatonLock:
        return BatonLock(self.baton, reentrant=True)

    def Condition(self, lock: Any = None) -> BatonCondition:
        return BatonCondition(self.baton, lock)

    def Event(self) -> BatonEvent:
        return BatonEvent(self.baton)

    def Semaphore(self, value: int = 1) -> BatonSemaphore:
        return BatonSemaphore(self.baton, value)

    def BoundedSemaphore(self, value: int = 1) -> BatonSemaphore:
        return BatonSemaphore(self.baton, value, bounded=True)

    def Thread(self, **kwargs: Any) -> LockstepThread:
        return LockstepThread(self, **kwargs)

    def Timer(self, interval: float, function: Callable[..., object],
              args: Iterable[Any] | None = None,
              kwargs: Mapping[str, Any] | None = None) -> LockstepTimer:
        return LockstepTimer(self, interval, function, args, kwargs)

    def current_thread(self) -> LockstepThread:
        """The running managed thread's handle (off the baton: a fault)."""
        managed = self.baton.current()
        if managed.handle is None:
            LockstepThread(self, name=managed.name, daemon=managed.daemon).adopt(managed)
        return managed.handle

    def get_ident(self) -> int:
        return self.baton.current().seq

    def Queue(self, maxsize: int = 0) -> BatonQueue:
        return BatonQueue(self.baton, maxsize)

    def new_name(self, target: Callable[..., object] | None) -> str:
        """``Thread-N (target)`` as the standard library names it, numbered per runtime."""
        name = f"Thread-{next(self._names)}"
        target_name = getattr(target, "__name__", None)
        return name if target_name is None else f"{name} ({target_name})"

    def inherited_daemon(self) -> bool:
        """A new thread is a daemon if its creating managed thread is one.

        Read from the managed thread itself, so it does not depend on whether
        ``current_thread()`` has made a handle yet.
        """
        managed = self.baton.threads.lookup(threading.get_ident())
        return managed is not None and managed.daemon


__all__ = ["LockstepThread", "LockstepThreads", "LockstepTimer"]
