"""Tick step 5: the operator phase, and its requests and cancels queued for ``TICK k+1``.

``OperatorPhase`` runs the operator's tick on the tick's inbox, admits its
items with the app relay's (``coordinator_admission``) and journals the
admission and what the operator's ports saw. ``pace`` runs after each reply
with ``t_k`` (the 1x pacer, for the person watching; host side like the
host delays, it never enters the journal).

A request or cancel goes to its target participant (a live member; one
spawned this tick and admitted at k+1 counts). A target that is not a live
member is a coordinator fault: the operator addressed nobody.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable

from navpy.sim.lockstep.coordinator_admission import Admission
from navpy.sim.lockstep.coordinator_deadline import OPERATOR as OPERATOR_WAIT
from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_tick import TickOutput
from navpy.sim.lockstep.coordinator_types import (
    CoordinatorFault,
    OperatorCancel,
    OperatorHost,
    OperatorInbox,
    OperatorRequest,
)
from navpy.sim.lockstep.part_wire import Request


class OperatorPhase:
    def __init__(self, operator: OperatorHost, roster: Roster, book: LifecycleBook,
                 admission: Admission, pace: Callable[[int], None]) -> None:
        self._operator = operator
        self._roster = roster
        self._book = book
        self._admission = admission
        self.pace = pace

    @property
    def done(self) -> bool:
        return self._operator.done

    def run(self, k: int, t_us: int, output: TickOutput, record: TickRecord) -> None:
        if self._operator.done:
            self._admission.route(output.responses, output.events)
            return
        # The baton's handoffs and timer rounds all draw on what is left of D_k.
        requests = self._roster.deadline.bounded(
            OPERATOR_WAIT, lambda wall: self._operator.run_tick(self._inbox(k, t_us, output, wall)))
        deliveries, admission = self._admission.admit(k, requests, self._ready)
        deliver_operator_requests(self._roster, deliveries, k)
        if admission is not None:
            record.note("admission", admission)
        self._note(record)

    def drain(self, k: int, t_us: int, output: TickOutput, record: TickRecord) -> None:
        self._operator.deliver_drain(self._inbox(k, t_us, output))
        self._note(record)

    def _inbox(self, k: int, t_us: int, output: TickOutput,
               wall_deadline: float | None = None) -> OperatorInbox:
        responses = self._admission.route(output.responses, output.events)  # the relay's go back
        return OperatorInbox(k, t_us, responses, tuple(output.events),
                             tuple(output.log_lines), tuple(self._book.operator), wall_deadline)

    def _ready(self, ordinal: int) -> bool:
        member = self._roster.current(ordinal)
        return member is not None and member.live and member.ready

    def _note(self, record: TickRecord) -> None:
        observed = self._operator.observations()
        if observed:  # what the operator's ports saw this tick: receipts, drops
            record.note("operator", observed)


def deliver_operator_requests(
    roster: Roster, items: Iterable[OperatorRequest | OperatorCancel], k: int,
) -> None:
    for item in items:
        member = roster.current(item.target)
        if member is None or not member.live:
            raise CoordinatorFault("operator_target", f"operator {type(item).__name__} {item.id} "
                                   "to a participant that is not live", tick=k, ordinal=item.target)
        if isinstance(item, OperatorCancel):
            member.inputs.cancels.append(item.id)
        else:
            member.inputs.requests.append(Request(item.id, item.payload))


__all__ = ["OperatorPhase", "deliver_operator_requests"]
