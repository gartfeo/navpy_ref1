"""RuntimeTime: the one port for clocks, waits, threads, queues, RNG and ids.

Covered product code uses this module (``rt``) instead of ``time``, ``threading``,
``queue``, ``random``, ``uuid``, ``secrets`` and ``datetime.now`` (enforced by
``tests/guards/test_runtime_time_lint.py``). A process binds one runtime before
first use, so lockstep (``deterministic_demo_eval.md`` step 9) runs the same code
on the sim clock; ``wall_runtime()``, the default, calls the standard library.
"""

from __future__ import annotations

import queue as _queue
import random as _random
import secrets as _secrets
import threading as _threading
import time as _time
import uuid as _uuid
from contextlib import nullcontext
from dataclasses import dataclass
from datetime import datetime, tzinfo
from typing import Any, Callable, ContextManager, Iterable, Mapping, Optional, Protocol

Empty = _queue.Empty


class LockPort(Protocol):
    """What callers of a runtime lock may use (annotations only)."""
    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool: ...

    def release(self) -> None: ...

    def __enter__(self) -> bool: ...

    def __exit__(self, *exc: object) -> object: ...


class QueuePort(Protocol):
    """What callers of a runtime queue may use (annotations only)."""
    def put(self, item: Any, block: bool = True, timeout: Optional[float] = None) -> None: ...

    def get(self, block: bool = True, timeout: Optional[float] = None) -> Any: ...

    def task_done(self) -> None: ...

    def join(self) -> None: ...


class ClockPort(Protocol):
    def monotonic(self) -> float: ...

    def time(self) -> float: ...

    def perf_counter(self) -> float: ...

    def now(self, tz: Optional[tzinfo] = None) -> datetime: ...

    def sleep(self, seconds: float) -> None: ...


class ThreadsPort(Protocol):
    """Threads and every primitive that can park one."""

    def Lock(self) -> Any: ...
    def RLock(self) -> Any: ...
    def Condition(self, lock: Any = None) -> Any: ...
    def Event(self) -> Any: ...
    def Semaphore(self, value: int = 1) -> Any: ...
    def BoundedSemaphore(self, value: int = 1) -> Any: ...
    def Thread(self, **kwargs: Any) -> Any: ...
    def Timer(self, interval: float, function: Callable[..., object],
              args: Optional[Iterable[Any]] = None,
              kwargs: Optional[Mapping[str, Any]] = None) -> Any: ...
    def current_thread(self) -> Any: ...
    def get_ident(self) -> int: ...
    def Queue(self, maxsize: int = 0) -> Any: ...
    def shielded(self) -> ContextManager[None]: ...
    def detached(self) -> ContextManager[None]: ...


class EntropyPort(Protocol):
    def rng(self) -> Any: ...

    def uuid4(self) -> _uuid.UUID: ...

    def token_urlsafe(self, nbytes: Optional[int] = None) -> str: ...


@dataclass(frozen=True)
class RuntimeTime:
    """Clocks, primitives and entropy of one process."""

    clock: ClockPort
    threads: ThreadsPort
    entropy: EntropyPort


class WallClock:
    def monotonic(self) -> float:
        return _time.monotonic()

    def time(self) -> float:
        return _time.time()

    def perf_counter(self) -> float:
        return _time.perf_counter()

    def now(self, tz: Optional[tzinfo] = None) -> datetime:
        return datetime.now(tz)

    def sleep(self, seconds: float) -> None:
        _time.sleep(seconds)


class WallThreads:
    def Lock(self) -> Any:
        return _threading.Lock()

    def RLock(self) -> Any:
        return _threading.RLock()

    def Condition(self, lock: Any = None) -> Any:
        return _threading.Condition(lock)

    def Event(self) -> Any:
        return _threading.Event()

    def Semaphore(self, value: int = 1) -> Any:
        return _threading.Semaphore(value)

    def BoundedSemaphore(self, value: int = 1) -> Any:
        return _threading.BoundedSemaphore(value)

    def Thread(self, **kwargs: Any) -> Any:
        return _threading.Thread(**kwargs)

    def Timer(self, interval: float, function: Callable[..., object],
              args: Optional[Iterable[Any]] = None,
              kwargs: Optional[Mapping[str, Any]] = None) -> Any:
        return _threading.Timer(interval, function, args, kwargs)

    def current_thread(self) -> Any:
        return _threading.current_thread()

    def get_ident(self) -> int:
        return _threading.get_ident()

    def Queue(self, maxsize: int = 0) -> Any:
        return _queue.Queue(maxsize)

    def shielded(self) -> ContextManager[None]:
        return nullcontext()  # wall waits are never cancelled

    def detached(self) -> ContextManager[None]:
        return nullcontext()


class WallEntropy:
    def rng(self) -> Any:
        return _random  # the module's shared generator, as before

    def uuid4(self) -> _uuid.UUID:
        return _uuid.uuid4()

    def token_urlsafe(self, nbytes: Optional[int] = None) -> str:
        return _secrets.token_urlsafe(nbytes)


def wall_runtime() -> RuntimeTime:
    """Host clocks, OS threads and OS entropy: today's behavior."""
    return RuntimeTime(WallClock(), WallThreads(), WallEntropy())


class RuntimeBinding:
    """The process's runtime: bound once, before anything has used it."""

    def __init__(self) -> None:
        self._runtime: Optional[RuntimeTime] = None
        self._used = False

    def bind(self, runtime: RuntimeTime) -> None:
        if self._used or self._runtime is not None:
            raise RuntimeError("the runtime must be bound once, before its first use")
        self._runtime = runtime

    def current(self) -> RuntimeTime:
        self._used = True
        if self._runtime is None:
            self._runtime = wall_runtime()
        return self._runtime


_binding = RuntimeBinding()


def bind(runtime: RuntimeTime) -> None:
    """Bind this process's runtime; only before any primitive or clock exists."""
    _binding.bind(runtime)


def monotonic() -> float:
    return _binding.current().clock.monotonic()


def time() -> float:
    return _binding.current().clock.time()


def perf_counter() -> float:
    return _binding.current().clock.perf_counter()


def sleep(seconds: float) -> None:
    _binding.current().clock.sleep(seconds)


def now(tz: Optional[tzinfo] = None) -> datetime:
    """``datetime.now(tz)`` on the runtime's unix clock."""
    return _binding.current().clock.now(tz)


def Lock() -> Any:
    return _binding.current().threads.Lock()


def RLock() -> Any:
    return _binding.current().threads.RLock()


def Condition(lock: Any = None) -> Any:
    return _binding.current().threads.Condition(lock)


def Event() -> Any:
    return _binding.current().threads.Event()


def Semaphore(value: int = 1) -> Any:
    return _binding.current().threads.Semaphore(value)


def BoundedSemaphore(value: int = 1) -> Any:
    return _binding.current().threads.BoundedSemaphore(value)


def Thread(**kwargs: Any) -> Any:
    return _binding.current().threads.Thread(**kwargs)


def Timer(interval: float, function: Callable[..., object],
          args: Optional[Iterable[Any]] = None,
          kwargs: Optional[Mapping[str, Any]] = None) -> Any:
    return _binding.current().threads.Timer(interval, function, args, kwargs)


def current_thread() -> Any:
    return _binding.current().threads.current_thread()


def get_ident() -> int:
    return _binding.current().threads.get_ident()


def Queue(maxsize: int = 0) -> Any:
    return _binding.current().threads.Queue(maxsize)


def shielded() -> ContextManager[None]:
    """Rollback or cleanup: its waits ignore a lockstep cancel (wall: no-op)."""
    return _binding.current().threads.shielded()


def detached() -> ContextManager[None]:
    """Long-lived workers started here are owned by their component (a vehicle,
    a transport, a logger) and stopped by its close, never by the calling
    request's lockstep cancel: they do not inherit its scope (wall: no-op)."""
    return _binding.current().threads.detached()


def rng() -> Any:
    """The runtime's random generator (``uniform``, ``randint``, ...)."""
    return _binding.current().entropy.rng()


def uuid4() -> _uuid.UUID:
    return _binding.current().entropy.uuid4()


def token_urlsafe(nbytes: Optional[int] = None) -> str:
    return _binding.current().entropy.token_urlsafe(nbytes)


__all__ = [
    "BoundedSemaphore", "ClockPort", "Condition", "Empty", "EntropyPort", "Event", "Lock",
    "LockPort", "Queue", "QueuePort", "RLock", "RuntimeBinding", "RuntimeTime", "Semaphore",
    "Thread", "ThreadsPort", "Timer", "WallClock", "WallEntropy", "WallThreads", "bind",
    "current_thread", "detached", "get_ident", "monotonic", "now", "perf_counter", "rng", "shielded",
    "sleep", "time", "token_urlsafe", "uuid4", "wall_runtime",
]
