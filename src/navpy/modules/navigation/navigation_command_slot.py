"""Thread-safe latest-work slot shared by navigation runtimes and their worker."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional, TypeVar


_T = TypeVar("_T")


@dataclass(frozen=True)
class NavigationCommandLease:
    """One consumed payload tied to the phase that admitted it."""

    payload: Any
    generation: int


class NavigationCommandSlot:
    """Own pending work, phase invalidation, signaling, and worker leases."""

    def __init__(self, lock: threading.RLock, signal: threading.Event) -> None:
        self._lock = lock
        self._signal = signal
        self._pending: Any = None
        self._phase_generation = 0
        self._in_flight_generation: Optional[int] = None

    def replace(self, payload: Any) -> Any:
        with self._lock:
            previous = self._pending
            self._pending = payload
            return previous

    def peek(self) -> Any:
        with self._lock:
            return self._pending

    def take_or_else(
        self,
        fallback: Callable[[], Any],
    ) -> Optional[NavigationCommandLease]:
        """Lease pending work or one fallback atomically at a command tick."""
        with self._lock:
            payload = self._pending
            self._pending = None
            self._signal.clear()
            if payload is None:
                payload = fallback()
            if payload is None:
                return None
            self._in_flight_generation = self._phase_generation
            return NavigationCommandLease(payload, self._phase_generation)

    def finish(self, lease: NavigationCommandLease) -> None:
        with self._lock:
            if self._in_flight_generation == lease.generation:
                self._in_flight_generation = None

    def invalidate(self) -> Any:
        """Start a new phase while preserving the truth of in-flight work."""
        with self._lock:
            self._phase_generation += 1
            pending = self._pending
            self._pending = None
            self._signal.clear()
            return pending

    def execute_if_current(
            self,
            lease: NavigationCommandLease,
            operation: Callable[[], _T],
    ) -> Optional[_T]:
        """Serialize phase validation with one command transaction."""
        with self._lock:
            if lease.generation != self._phase_generation:
                return None
            return operation()

    def has_pending_or_in_flight(self) -> bool:
        with self._lock:
            return self._pending is not None or self._in_flight_generation is not None

    def signal_pending(self) -> None:
        with self._lock:
            if self._pending is not None:
                self._signal.set()

    def clear_signal(self) -> None:
        self._signal.clear()


__all__ = [
    "NavigationCommandLease",
    "NavigationCommandSlot",
]
