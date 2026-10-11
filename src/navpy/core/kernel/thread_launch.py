"""Atomic ownership fence for daemon targets with ambiguous starts."""

from __future__ import annotations

import threading


class ThreadLaunchGate:
    """Either commit a target to dependency use or cancel it permanently."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._entered = threading.Event()
        self._cancelled = False
        self._committed = False

    def enter(self) -> bool:
        """Called first by the target, before it touches owned dependencies."""
        with self._lock:
            self._entered.set()
            if self._cancelled:
                return False
            self._committed = True
            return True

    def cancel_before_commit(self) -> bool:
        """Make a target that has not committed safe to abandon."""
        with self._lock:
            if self._committed:
                return False
            self._cancelled = True
            return True

    @property
    def entered(self) -> bool:
        return self._entered.is_set()

    def wait_until_entered(self, timeout_s: float) -> bool:
        return self._entered.wait(max(0.0, float(timeout_s)))

    @property
    def committed(self) -> bool:
        with self._lock:
            return self._committed


__all__ = ["ThreadLaunchGate"]
