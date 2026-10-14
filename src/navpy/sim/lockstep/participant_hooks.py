"""What a product plugs into the generic participant loop.

The loop (:mod:`navpy.sim.lockstep.participant_loop`) owns the wire, the
tick order and the end; a product supplies a :class:`ParticipantHooks`
built by a factory named in its environment record (``module:attribute``,
called with a :class:`ParticipantContext`). NavPy's hooks (lifecycle stop,
log lines) are step 9f's and the backend's (ASGI requests, launcher port)
step 9e's; the tests use toy hooks.

Threading: ``construct`` runs on the managed bootstrap thread at the
admission tick. ``on_end``, ``on_lifecycle``, ``on_cancel`` and
``on_request`` run on the loop's thread (the baton coordinator) in the tick's
input phase, so they may only deliver (``BatonQueue.deliver``,
``BatonEvent.deliver_set``). ``finished``, ``flushes``, ``drain_outbox`` and
``exit_code`` run on the loop's thread after quiescence.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol

from navpy.sim.lockstep.part_wire import (
    Event,
    Lifecycle,
    Request,
    RequestId,
    Response,
    SpawnRequest,
    StopRequest,
)
from navpy.sim.lockstep.participant import Participant


class PortLink(Protocol):
    """One logical device's participant side (``LockstepLink`` is one)."""

    def deliver(self, data: bytes) -> None: ...

    def drain_egress(self) -> bytes: ...


class RequestIds:
    """A participant's own ids: ``(its ordinal, 1, 2, ...)``."""

    def __init__(self, ordinal: int) -> None:
        self._ordinal = ordinal
        self._sequence = 0

    def next(self) -> RequestId:
        self._sequence += 1
        return self._ordinal, self._sequence


@dataclass(frozen=True)
class Outbox:
    responses: tuple[Response, ...] = ()
    events: tuple[Event, ...] = ()
    spawn_requests: tuple[SpawnRequest, ...] = ()
    stop_requests: tuple[StopRequest, ...] = ()
    log_lines: tuple[str, ...] = ()
    #: Stable fault kinds only (journaled through ``DONE``); details to stderr.
    faults: tuple[str, ...] = ()


@dataclass
class OutboxBuffer:
    """Collects what managed threads produce during a tick (one runs at a time)."""

    responses: list[Response] = field(default_factory=list)
    events: list[Event] = field(default_factory=list)
    spawn_requests: list[SpawnRequest] = field(default_factory=list)
    stop_requests: list[StopRequest] = field(default_factory=list)
    log_lines: list[str] = field(default_factory=list)
    faults: list[str] = field(default_factory=list)

    def take(self) -> Outbox:
        """Everything since the last take; id lists in id order."""
        outbox = Outbox(
            tuple(sorted(self.responses, key=lambda r: r.id)), tuple(self.events),
            tuple(sorted(self.spawn_requests, key=lambda s: s.id)),
            tuple(sorted(self.stop_requests, key=lambda s: s.id)),
            tuple(self.log_lines), tuple(self.faults),
        )
        for stream in (self.responses, self.events, self.spawn_requests, self.stop_requests,
                       self.log_lines, self.faults):
            stream.clear()
        return outbox


@dataclass(frozen=True)
class ParticipantContext:
    participant: Participant
    #: The product's run inputs from the environment record (JSON).
    config: Mapping[str, Any]
    ids: RequestIds


class ParticipantHooks:
    """Defaults for every hook; a product overrides what it uses."""

    #: Logical device name -> its link (``lockstep:serial0``, ...).
    links: Mapping[str, PortLink] = {}

    def construct(self) -> None:
        """Import and build the product (bootstrap thread, admission tick)."""

    def on_end(self) -> None:
        """The first drain ``TICK`` (``end: true``): stop and flush."""

    def on_lifecycle(self, entry: Lifecycle) -> None:
        """A lifecycle record addressed to this participant."""

    def on_cancel(self, request_id: RequestId) -> None:
        """Cancel the scope of a pending request."""

    def on_request(self, request: Request) -> None:
        """Start a request (it may answer in a later ``DONE``)."""

    def finished(self) -> bool:
        """The product's own end is complete (after a stop or a drain end)."""
        return False

    def flushes(self) -> Sequence[Callable[[], object]]:
        """The strict flushes of the end (``Participant.drain_flush``)."""
        return ()

    def drain_outbox(self) -> Outbox:
        return Outbox()

    def exit_code(self, flushed: bool) -> int:
        return 0 if flushed else 1


__all__ = [
    "Outbox", "OutboxBuffer", "ParticipantContext", "ParticipantHooks", "PortLink", "RequestIds",
]
