"""Cancel handle returned by a message subscription."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt


class Subscription:
    def __init__(self, cancel: Callable[[], None]) -> None:
        self._cancel_action = cancel
        self._lock = rt.Lock()
        self._cancelled = False

    def cancel(self) -> None:
        with self._lock:
            if self._cancelled:
                return
            self._cancelled = True
        self._cancel_action()

    def __call__(self) -> None:
        self.cancel()
