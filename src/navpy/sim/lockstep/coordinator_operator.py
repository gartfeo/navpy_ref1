"""Tick step 5's output: the operator's requests and cancels, queued for ``TICK k+1``.

A request or cancel goes to its target participant (a live member; one
spawned this tick and admitted at k+1 counts). A target that is not a live
member is a coordinator fault: the operator addressed nobody.
"""

from __future__ import annotations

from collections.abc import Iterable

from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, OperatorCancel, OperatorRequest
from navpy.sim.lockstep.part_wire import Request


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


__all__ = ["deliver_operator_requests"]
