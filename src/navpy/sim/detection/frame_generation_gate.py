"""Admission and commit generations for simulator source frames."""

from __future__ import annotations

import threading
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Iterator, Optional


@dataclass(frozen=True)
class FrameGeneration:
    """Identity and cancellation signal for one admitted frame generation."""

    epoch: int
    generation: int
    invalidated: threading.Event = field(
        compare=False,
        repr=False,
        default_factory=threading.Event,
    )


@dataclass(frozen=True)
class ResetTicket:
    serial: int


class FrameGenerationGate:
    """Linearize admission, commits, reset closure, and permanent stop."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._epoch = 0
        self._generation = 0
        self._current = FrameGeneration(0, 0)
        self._admission_open = True
        self._stopped = False
        self._reset_serial = 0

    @property
    def token(self) -> FrameGeneration:
        # Reading the immutable reference without waiting is intentional. A
        # callback arriving during reset must retain the invalid old token and
        # be rejected, not wait and join the freshly opened epoch.
        return self._current

    @contextmanager
    def admission(
        self,
        token: FrameGeneration,
    ) -> Iterator[Optional[FrameGeneration]]:
        with self._lock:
            yield token if self._accepts(token) else None

    @contextmanager
    def commit(
        self,
        token: FrameGeneration,
    ) -> Iterator[bool]:
        # The lock remains held through all post-render side effects and the
        # final publication. Reset therefore happens wholly before or after a
        # committed frame, never in its middle.
        with self._lock:
            yield self._accepts(token)

    def supersede(self) -> FrameGeneration:
        """Invalidate queued/in-flight work and open a newer generation."""
        with self._lock:
            self._current.invalidated.set()
            self._generation += 1
            self._current = FrameGeneration(self._epoch, self._generation)
            return self._current

    def begin_reset(self) -> Optional[ResetTicket]:
        """Close admission before any component state is cleared."""
        with self._lock:
            if self._stopped:
                return None
            self._current.invalidated.set()
            self._admission_open = False
            self._generation += 1
            self._reset_serial += 1
            return ResetTicket(self._reset_serial)

    def finish_reset(self, ticket: ResetTicket) -> bool:
        """Open a fresh epoch only after the caller completed every reset."""
        with self._lock:
            if self._stopped or ticket.serial != self._reset_serial:
                return False
            self._epoch += 1
            self._current = FrameGeneration(self._epoch, self._generation)
            self._admission_open = True
            return True

    def stop(self) -> None:
        with self._lock:
            self._current.invalidated.set()
            self._admission_open = False
            self._stopped = True
            self._generation += 1

    def is_current(self, token: FrameGeneration) -> bool:
        with self._lock:
            return self._accepts(token)

    def _accepts(self, token: FrameGeneration) -> bool:
        return bool(
            not self._stopped
            and self._admission_open
            and token is self._current
            and not token.invalidated.is_set()
        )


__all__ = ["FrameGeneration", "FrameGenerationGate", "ResetTicket"]
