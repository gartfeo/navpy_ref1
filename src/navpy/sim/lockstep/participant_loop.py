"""The generic participant loop: ``REGISTER``, then one ``DONE`` per ``TICK``.

Bootstrap steps 1-2 (:func:`navpy.sim.lockstep.participant.bootstrap`) come
first; this is step 3 on. Per ``TICK k`` (``lockstep_slice.md``):

1. The first ``TICK`` is the admission tick: the product is built on the
   managed bootstrap thread (:meth:`Participant.admit`); afterwards each
   ``TICK`` must be exactly the next tick.
2. ``BatonRuntime.tick`` sets ``t_k`` and delivers the inputs in tick input
   order: the drain ``end``, lifecycle records, cancels, requests (id order),
   then the port bytes per device; then quiescence, then due timers.
3. Once the product's own end is complete (a lifecycle stop, or ``end``),
   the strict flushes run on a managed thread (:meth:`Participant.drain_flush`).
4. ``DONE k`` carries the links' egress and the hooks' outbox; when the
   flush thread has finished, it carries ``ended{flushed}`` and is the last
   message: the loop returns the exit code.

A product or contract fault becomes ``DONE k`` with a stable fault kind
(the exception class and a logical tag, never its text) and
``ended{flushed: false}``: never a silent exit.
"""

from __future__ import annotations

import sys
import traceback

from navpy.sim.lockstep.baton import ManagedThread
from navpy.sim.lockstep.part_socket import PartSocket, PeerClosed
from navpy.sim.lockstep.part_wire import Done, PartWireError, Register, Tick, check_tick
from navpy.sim.lockstep.participant import Participant
from navpy.sim.lockstep.participant_hooks import ParticipantHooks

#: Exit code when the coordinator closed the link before this participant
#: ended: the coordinator is gone (its fault, never a membership change).
EXIT_COORDINATOR_GONE = 3


class ParticipantLoop:
    def __init__(self, participant: Participant, hooks: ParticipantHooks, link: PartSocket) -> None:
        self._participant = participant
        self._baton = participant.runtime.baton
        self._hooks = hooks
        self._link = link
        self._expected: int | None = None
        self._end_thread: ManagedThread | None = None
        self._flushed = False

    def run(self, register: Register) -> int:
        """Register, then tick until the ending ``DONE``; return the exit code."""
        self._link.send(register)
        while True:
            try:
                message, _raw = self._link.recv()
            except PeerClosed:
                return EXIT_COORDINATOR_GONE
            if not isinstance(message, Tick):
                raise PartWireError(f"expected TICK, got {type(message).__name__}")
            done = self.step(message)
            self._link.send(done)
            if done.ended is not None:
                return self._hooks.exit_code(done.ended)

    def step(self, tick: Tick) -> Done:
        """Run one ``TICK`` to its ``DONE``."""
        if self._expected is None:
            self._participant.admit(self._hooks.construct)
        else:
            check_tick(tick.k, self._expected)
        self._expected = tick.k + 1
        try:
            self._baton.tick(tick.t_us, lambda: self._deliver(tick))
            self._settle_end()
        except Exception as error:  # a fault: reported, then the end
            # The wire carries a stable kind only; the text (paths, OS errors)
            # goes to stderr, which the coordinator's host captures in a log.
            traceback.print_exception(error, file=sys.stderr)
            return self._done(tick.k, fault=f"{type(error).__name__}:tick")
        return self._done(tick.k)

    def _deliver(self, tick: Tick) -> None:
        hooks = self._hooks
        if tick.end:
            hooks.on_end()
        for entry in tick.lifecycle:
            hooks.on_lifecycle(entry)
        for request_id in tick.cancels:
            hooks.on_cancel(request_id)
        for request in tick.requests:
            hooks.on_request(request)
        for device, data in tick.ports:
            link = hooks.links.get(device)
            if link is None:
                raise PartWireError(f"TICK {tick.k} carries bytes for unknown device {device!r}")
            link.deliver(data)

    def _settle_end(self) -> None:
        if self._end_thread is None and self._hooks.finished():
            self._end_thread = self._baton.spawn(self._end, name="participant-end")
            self._baton.run_until_quiescent()
            self._baton.settle_timers()

    def _end(self) -> None:
        self._flushed = self._participant.drain_flush(self._hooks.flushes())

    def _done(self, k: int, fault: str | None = None) -> Done:
        ports = tuple(
            (device, self._hooks.links[device].drain_egress()) for device in sorted(self._hooks.links)
        )
        outbox = self._hooks.drain_outbox()
        faults = outbox.faults + ((fault,) if fault else ())
        if faults:
            ended: bool | None = False
        elif self._end_thread is not None and self._end_thread.done:
            ended = self._flushed
        else:
            ended = None
        return Done(
            k, ports, outbox.responses, outbox.events, outbox.spawn_requests,
            outbox.stop_requests, outbox.log_lines, faults, ended,
        )


__all__ = ["EXIT_COORDINATOR_GONE", "ParticipantLoop"]
