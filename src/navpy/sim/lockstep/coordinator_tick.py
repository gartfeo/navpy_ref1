"""Tick steps 2-4: route egress, ``TICK`` every admitted participant, collect ``DONE``.

Shared by mission, finalization and drain. Participants are admitted at the
first exchange after their registration (the boot barrier tick for the
initial ones, k+1 for one spawned from ``DONE k``). ``TICK``s go out in
ordinal order and the participants run concurrently; ``DONE``s are read and
applied in the same order, so the result depends only on the ``TICK``s.
Within one ``DONE``: port egress, responses, events (readiness), log lines,
spawn requests, stop requests, faults, then the end, which retires the
participant now and holds the clock until its process exits.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Member, Roster
from navpy.sim.lockstep.coordinator_types import (
    SERIAL_DEVICES,
    TERMINAL_MARKER,
    CoordinatorFault,
    ParticipantFault,
)
from navpy.sim.lockstep.part_wire import Done, Event, Response

#: The ``DONE`` event that commits readiness (NavPy's ``runtime_ready``;
#: the backend's lifespan startup complete).
READY_EVENT = "runtime_ready"


@dataclass
class TickOutput:
    """What tick k produced for the SITL (step 6) and the operator (step 5)."""

    ingress: dict[str, bytes] = field(default_factory=dict)
    responses: list[tuple[int, Response]] = field(default_factory=list)
    events: list[tuple[int, Event]] = field(default_factory=list)
    log_lines: list[tuple[int, str]] = field(default_factory=list)
    terminal: bool = False
    faults: list[ParticipantFault] = field(default_factory=list)


class TickExchange:
    def __init__(self, roster: Roster, book: LifecycleBook, terminal_roles: tuple[str, ...]) -> None:
        self._roster = roster
        self._book = book
        #: Only these roles' log lines can carry the mission terminal marker (NavPy's SNAP).
        self._terminal_roles = terminal_roles

    def run(self, k: int, t_us: int, record: TickRecord, egress: dict[str, bytes] | None,
            *, draining: bool) -> TickOutput:
        """``egress`` is the SITL frame's per device; ``None`` in drain (no port bytes)."""
        self._book.begin_tick(record)
        for member in self._roster.members:
            if member.admitted_at is None and member.live:
                member.admitted_at = k
        members = self._roster.active()
        owned = {device: m for m in members for device in m.devices}
        for device, data in (egress or {}).items():
            if device not in owned:
                record.discard(device, len(data))
        sent = []
        for member in members:
            ports = () if egress is None else tuple(
                (device, egress.get(device, b"")) for device in sorted(member.devices))
            sent.append(self._roster.send(member, member.take_tick(k, t_us, ports, draining)))
        output = TickOutput()
        for member, raw_tick in zip(members, sent):
            done, raw_done = self._roster.receive(member, k)
            record.member(member.ordinal, raw_tick, raw_done)
            self._apply(member, done, k, t_us, record, output, draining)
        self._book.check_budgets(t_us)
        return output

    def _apply(self, member: Member, done: Done, k: int, t_us: int, record: TickRecord,
               output: TickOutput, draining: bool) -> None:
        for device, data in done.ports:
            if device not in member.devices:
                raise CoordinatorFault("foreign_device", f"wrote to {device!r}, not its link",
                                       tick=k, ordinal=member.ordinal)
            if draining:
                record.drain_egress(member.ordinal, device, data)
            elif device in SERIAL_DEVICES:
                output.ingress[device] = output.ingress.get(device, b"") + data
        output.responses.extend((member.ordinal, response) for response in done.responses)
        for event in done.events:
            output.events.append((member.ordinal, event))
            if event.kind == READY_EVENT:
                self._book.ready(member, event.payload)
        navpy = member.spawn.role in self._terminal_roles
        for line in done.log_lines:
            output.log_lines.append((member.ordinal, line))
            output.terminal = output.terminal or (navpy and TERMINAL_MARKER in line)
        for spawn in done.spawn_requests:
            self._book.spawn(spawn, member.ordinal, draining=draining)
        for stop in done.stop_requests:
            self._book.stop(stop, member.ordinal, t_us, draining=draining)
        output.faults.extend(
            ParticipantFault("participant", fault, tick=k, ordinal=member.ordinal)
            for fault in done.faults)
        if done.ended is not None:
            self._book.retire(member, k, done.ended)
            self._roster.await_exit(member)


__all__ = ["READY_EVENT", "TickExchange", "TickOutput"]
