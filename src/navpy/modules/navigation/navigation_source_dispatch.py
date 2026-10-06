"""Single-owner terminal-source handoff into the command scheduler.

The slot also carries the command-loop OBSERVER, because it belongs to the
same owner as the dispatch callback and must appear and disappear with it:
an observer left bound to a closed source would record iterations against
a ledger nobody drains.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Any, Protocol

from navpy.modules.navigation.navigation_command_worker import (
    CommandLoopObserver,
)


SourceDispatch = Callable[[], bool]


class BindSourceDispatch(Protocol):
    """Publish the dispatch callback, and optionally its observer."""

    def __call__(
        self,
        callback: SourceDispatch,
        observer: CommandLoopObserver | None = None,
    ) -> None:
        ...


class NavigationSourceDispatchSlot:
    """Bind one long-lived source callback for the navigation worker."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._callback: SourceDispatch | None = None
        self._observer: CommandLoopObserver | None = None

    def bind(
        self,
        callback: SourceDispatch,
        observer: CommandLoopObserver | None = None,
    ) -> None:
        """Publish the navigation-owned callback exactly once."""
        with self._lock:
            if self._callback is not None:
                raise RuntimeError("navigation source dispatch is already bound")
            self._callback = callback
            self._observer = observer

    def dispatch_available(self) -> bool:
        """Ask the bound source to admit at most one current observation."""
        with self._lock:
            callback = self._callback
        return False if callback is None else bool(callback())

    def note_iteration(self, iteration: int) -> None:
        """Forward one worker loop pass to the bound observer, if any."""
        with self._lock:
            observer = self._observer
        if observer is not None:
            observer.note_iteration(iteration)

    def note_command(
        self, iteration: int, command: Any, raised: bool = False
    ) -> None:
        """Forward the command that pass issued to the bound observer."""
        with self._lock:
            observer = self._observer
        if observer is not None:
            observer.note_command(iteration, command, raised)


__all__ = [
    "BindSourceDispatch",
    "NavigationSourceDispatchSlot",
    "SourceDispatch",
]
