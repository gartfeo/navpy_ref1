"""Cancellation scopes for the backend's background producers (``ProducerScopes``).

Each tracked task (telemetry, auto-connect, launch tasks) and thread (the
mission probe) gets its own :class:`CancelScope`, so the shutdown's cancel
step stops each one's waits without touching another unit's. A task's
unexpected end latches a fault (:mod:`gcs.backend.lockstep.faults`).
"""

from __future__ import annotations

import contextvars
from collections.abc import Iterator
from contextlib import contextmanager

from gcs.backend.lockstep.faults import FaultLatch, Outcome
from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.cancel_scope import CancelScope


class LockstepProducerScopes:
    def __init__(self, faults: FaultLatch) -> None:
        self._scopes: list[CancelScope] = []
        self._faults = faults

    def _new(self, name: str) -> CancelScope:
        self._scopes = [scope for scope in self._scopes if not scope.closed]
        scope = CancelScope(f"producer {name}")
        self._scopes.append(scope)
        return scope

    def context(self, name: str) -> contextvars.Context:
        return cancel_scope.bind(contextvars.copy_context(), self._new(name))

    @contextmanager
    def entered(self, name: str) -> Iterator[CancelScope]:
        scope = self._new(name)
        try:
            with cancel_scope.entered(scope):
                yield scope
        finally:
            scope.close()

    def observe(self, producer: str, task: Outcome) -> None:
        self._faults.observe_task(f"producer {producer}", task)

    def cancel_all(self) -> int:
        """Cancel every producer scope still open; returns how many."""
        return sum(scope.cancel() for scope in list(self._scopes))


__all__ = ["LockstepProducerScopes"]
