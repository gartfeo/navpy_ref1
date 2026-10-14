"""Cancellation scopes: one per unit of lockstep work (``lockstep_slice.md``, "Cancel").

A scope is a context variable, not thread state: every coroutine of the
backend shares the loop thread, so a thread cannot say which request it
serves. Each ``asgi_request`` runs its task in its own scope (keyed by its
wire request id) and each background producer owns one. A managed thread
starts in a copy of its creator's context and a ``BatonExecutor`` job in its
submitter's, so a route's executor job and its child work share the route's
scope.

Every unshielded baton wait is a checkpoint (:meth:`BatonRuntime.park`): in a
cancelled scope it raises :class:`Cancelled` before it parks, a cancel wakes
a parked waiter, and the waiter raises once it runs again. So a cancelled
scope never returns from a wait, and no product side effect runs after its
cancel. Each primitive undoes its own waiter state in a ``finally``; a lock
already handed to a cancelled waiter is passed on. :func:`shielded` exempts
rollback and cleanup (and a condition's reacquire) from the checkpoint.

Consumption: a :class:`Cancelled` whose scope was cancelled ends its unit
normally (:func:`consumed`); anything else is still a fault.
"""

from __future__ import annotations

import contextvars
from collections.abc import Iterator
from contextlib import contextmanager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navpy.sim.lockstep.baton import BatonRuntime
    from navpy.sim.lockstep.baton_parts import ManagedThread


class Cancelled(BaseException):
    """Raised by a wait in a cancelled scope. A ``BaseException``: no
    ``except Exception`` retry branch catches it."""

    def __init__(self, scope: "CancelScope") -> None:
        super().__init__(scope.name)
        self.scope = scope


class CancelScope:
    """A cancellation token; :meth:`cancel` wakes every thread parked in it."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.cancelled = False
        #: Its unit ended: a later cancel changes nothing (a response stands).
        self.closed = False
        # Insertion order: the parked threads wake in the order they parked.
        self._parked: dict["ManagedThread", "BatonRuntime"] = {}

    def cancel(self) -> bool:
        """Cancel and wake the parked waiters; False if closed or already cancelled.

        The coordinator calls it in a tick's input phase (a cancel is an
        input), or a baton holder does (shutdown).
        """
        if self.cancelled or self.closed:
            return False
        self.cancelled = True
        for thread, runtime in list(self._parked.items()):
            runtime.wake(thread)
        return True

    def close(self) -> None:
        self.closed = True

    def check(self) -> None:
        if self.cancelled:
            raise Cancelled(self)

    def watch(self, thread: "ManagedThread", runtime: "BatonRuntime") -> None:
        self._parked[thread] = runtime

    def unwatch(self, thread: "ManagedThread") -> None:
        self._parked.pop(thread, None)

    def __repr__(self) -> str:
        state = "cancelled" if self.cancelled else ("closed" if self.closed else "open")
        return f"<CancelScope {self.name} {state}>"


_scope: contextvars.ContextVar[CancelScope | None] = contextvars.ContextVar(
    "lockstep_cancel_scope", default=None,
)
_shielded: contextvars.ContextVar[bool] = contextvars.ContextVar("lockstep_shielded", default=False)


def current() -> CancelScope | None:
    """The scope of the running context, if any."""
    return _scope.get()


def active() -> CancelScope | None:
    """The scope a wait here must observe: none while shielded."""
    return None if _shielded.get() else _scope.get()


def checkpoint() -> None:
    """A wait's entry: in a cancelled, unshielded scope, raise before taking anything.

    A wait that would return at once (a free lock or permit, a set event, a
    non-empty queue, a finished thread) is a checkpoint too.
    """
    scope = active()
    if scope is not None:
        scope.check()


@contextmanager
def entered(scope: CancelScope) -> Iterator[CancelScope]:
    """Run the block in ``scope`` (it replaces any inherited one)."""
    token = _scope.set(scope)
    try:
        yield scope
    finally:
        _scope.reset(token)


def bind(context: contextvars.Context, scope: CancelScope) -> contextvars.Context:
    """``context`` with ``scope`` set: for a task created in its own scope."""
    context.run(_scope.set, scope)
    return context


@contextmanager
def detached() -> Iterator[None]:
    """Threads started in the block carry no scope: a component's own worker
    (stopped by its close), not its creating request's."""
    token = _scope.set(None)
    try:
        yield
    finally:
        _scope.reset(token)


@contextmanager
def shielded() -> Iterator[None]:
    """Waits in the block are not checkpoints: rollback and cleanup complete."""
    token = _shielded.set(True)
    try:
        yield
    finally:
        _shielded.reset(token)


def consumed(error: BaseException) -> bool:
    """A cancellation that ends its unit normally: every leaf is a
    :class:`Cancelled` of a scope that was cancelled."""
    if isinstance(error, Cancelled):
        return error.scope.cancelled
    nested = getattr(error, "exceptions", None)
    return bool(nested) and all(consumed(inner) for inner in nested)


__all__ = [
    "CancelScope", "Cancelled", "active", "bind",
    "checkpoint", "consumed", "current", "detached", "entered", "shielded",
]
