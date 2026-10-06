"""Deterministic FIFO worker for navigation stream I/O."""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar, cast


_T = TypeVar("_T")
_STOP = object()
NAVIGATION_STREAM_JOIN_TIMEOUT_S = 2.0
# Two wall-seconds at the exact demo's worst-case 50 Hz * 10x cadence, with
# room for both compact and debug writes per command.  Control fences and STOP
# records bypass this bound, so saturation always fails without blocking.
NAVIGATION_STREAM_MAX_PENDING = 2 * 50 * 10 * 2


class NavigationStreamBacklogError(RuntimeError):
    """The nonblocking evidence FIFO exhausted its finite stall budget."""


@dataclass(frozen=True)
class _DrainFence:
    reached: threading.Event


@dataclass(frozen=True)
class _DataItem(Generic[_T]):
    value: _T


class NavigationStreamWorker(Generic[_T]):
    """Keep blocking preformatted stream I/O off the command thread."""

    def __init__(
        self,
        handler: Callable[[_T], None],
        *,
        max_pending: int = NAVIGATION_STREAM_MAX_PENDING,
    ) -> None:
        if type(max_pending) is not int or max_pending <= 0:
            raise ValueError("navigation stream max_pending must be positive")
        self._handler = handler
        self._queue: queue.Queue[object] = queue.Queue()
        self._slots = threading.BoundedSemaphore(max_pending)
        self._max_pending = max_pending
        self._accept_lock = threading.Lock()
        self._error_lock = threading.Lock()
        self._accepting = True
        self._first_error: BaseException | None = None
        self._thread: threading.Thread | None = None

    def submit(self, item: _T) -> bool:
        with self._accept_lock:
            if not self._accepting:
                error = self.first_error
                if error is not None:
                    raise error
                return False
            self._start_locked()
            if not self._slots.acquire(blocking=False):
                error = NavigationStreamBacklogError(
                    "navigation stream backlog exceeded nonblocking capacity "
                    f"{self._max_pending}"
                )
                self._fail_locked(error)
                raise error
            self._queue.put(_DataItem(item))
            return True

    def wait_until_idle(
        self,
        timeout_s: float = NAVIGATION_STREAM_JOIN_TIMEOUT_S,
    ) -> None:
        if threading.current_thread() is self._thread:
            raise TimeoutError("navigation stream worker cannot drain itself")
        with self._accept_lock:
            thread = self._thread
            if thread is None:
                return
            if not self._accepting:
                fence = None
            else:
                fence = _DrainFence(threading.Event())
                self._queue.put(fence)
        if fence is None:
            thread.join(timeout=timeout_s)
            reached = not thread.is_alive()
        else:
            reached = fence.reached.wait(timeout=timeout_s)
        if not reached:
            raise TimeoutError(
                "navigation stream worker did not drain before the deadline"
            )

    @property
    def first_error(self) -> BaseException | None:
        with self._error_lock:
            return self._first_error

    def raise_if_failed(self) -> None:
        error = self.first_error
        if error is not None:
            raise error

    def close(self) -> None:
        with self._accept_lock:
            if self._accepting:
                self._accepting = False
                thread = self._thread
                if thread is not None:
                    self._queue.put(_STOP)
            else:
                thread = self._thread
        if thread is None or not thread.is_alive():
            return
        if thread is threading.current_thread():
            raise TimeoutError("navigation stream worker cannot join itself")
        thread.join(timeout=NAVIGATION_STREAM_JOIN_TIMEOUT_S)
        if thread.is_alive():
            raise TimeoutError(
                "navigation stream worker did not stop; output remains owned"
            )

    def _start_locked(self) -> None:
        if self._thread is not None:
            return
        thread = threading.Thread(
            target=self._run,
            name="NavigationStreamWriter",
            daemon=True,
        )
        self._thread = thread
        try:
            thread.start()
        except BaseException as error:
            self._remember_error(error)
            self._accepting = False
            self._thread = None
            raise

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                if isinstance(item, _DrainFence):
                    item.reached.set()
                    continue
                if isinstance(item, _DataItem):
                    try:
                        self._handler(cast(_T, item.value))
                    except BaseException as error:
                        self._fail(error)
                    finally:
                        self._slots.release()
            finally:
                self._queue.task_done()

    def _fail(self, error: BaseException) -> None:
        with self._accept_lock:
            self._fail_locked(error)

    def _fail_locked(self, error: BaseException) -> None:
        self._remember_error(error)
        if self._accepting:
            self._accepting = False
            self._queue.put(_STOP)

    def _remember_error(self, error: BaseException) -> None:
        with self._error_lock:
            if self._first_error is None:
                self._first_error = error


__all__ = [
    "NAVIGATION_STREAM_MAX_PENDING",
    "NavigationStreamBacklogError",
    "NavigationStreamWorker",
]
