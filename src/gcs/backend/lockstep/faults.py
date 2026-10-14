"""Unobserved failures latch a participant fault (``lockstep_slice.md``, "Cancel").

A request task, a producer task, a scheduled broadcast or an executor job
whose awaiter is gone that ends with an
unexpected exception, or with a ``Cancelled`` of a scope nobody cancelled,
is not an outcome: it latches a fault in the tick's ``DONE`` (``faults``), so
the run ends unclean even if some ``except`` swallows the raise.
"""

from __future__ import annotations

import asyncio
from typing import Protocol

from navpy.sim.lockstep import cancel_scope
from navpy.sim.lockstep.baton import is_abort
from navpy.sim.lockstep.participant_hooks import OutboxBuffer


class Outcome(Protocol):
    """A done task or future (asyncio's or ``run_coroutine_threadsafe``'s)."""

    def cancelled(self) -> bool: ...

    def exception(self) -> BaseException | None: ...


def unexpected(error: BaseException | None) -> bool:
    """Neither success, nor a cancellation that ended its unit, nor an abort."""
    if error is None or is_abort(error) or cancel_scope.consumed(error):
        return False
    return not isinstance(error, asyncio.CancelledError)


class FaultLatch:
    def __init__(self, out: OutboxBuffer) -> None:
        self._out = out
        self.latched: list[str] = []

    def latch(self, where: str, error: BaseException | str) -> None:
        kind = error if isinstance(error, str) else type(error).__name__
        fault = f"{kind}:{where}"
        self.latched.append(fault)
        self._out.faults.append(fault)

    def job_failure(self, error: BaseException, observed: bool) -> None:
        """``BatonEventLoop.job_failures.report``: a failure nobody took, or a
        ``Cancelled`` of a scope nobody cancelled, is a fault."""
        stray = isinstance(error, cancel_scope.Cancelled) and not cancel_scope.consumed(error)
        if stray or (not observed and unexpected(error)):
            self.latch("executor job", error)

    def observe_task(self, where: str, task: Outcome) -> None:
        """A done callback: latch a task's or future's unexpected end."""
        if task.cancelled():
            return
        error = task.exception()
        if unexpected(error):
            self.latch(where, error)


__all__ = ["FaultLatch", "Outcome", "unexpected"]
