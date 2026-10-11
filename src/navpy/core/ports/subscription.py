"""Cancel handle returned by a message subscription."""

from __future__ import annotations

import threading
from collections.abc import Callable


class Subscription:
    def __init__(self, cancel: Callable[[], None]) -> None:
        self._cancel_action = cancel
        self._lock = threading.Lock()
        self._cancelled = False

    def cancel(self) -> None:
        with self._lock:
            if self._cancelled:
                return
            self._cancelled = True
        self._cancel_action()

    def __call__(self) -> None:
        self.cancel()
