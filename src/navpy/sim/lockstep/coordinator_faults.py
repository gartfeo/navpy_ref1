"""Scheduled faults at run time: started, healed and journaled tick by tick (step 7).

The plans are ``fault_schedule``'s. :meth:`FaultRunner.tick` runs at the end of
every tick that is not a drain, after every ``DONE k`` is applied, so what it
changes takes effect from tick k+1: a radio cut from frame k+1, a lifecycle record
in ``TICK k+1``, a respawn admitted at k+1 (like one spawned from ``DONE k``).

One fault at a time, each with its recovery: a fault due while another is active
or inside its recovery deadline waits for that deadline, so no fault can stand in
the way of another's recovery. At equal times the plan order decides.

A tick record gets ``faults`` only when something happened, so a fault-free
journal does not change:

- ``["start", id]``;
- ``["heal", id, deadline_us]``, with ``"forced"`` when the fence ended it early;
  the deadline is the heal's time plus the plan's ``recovery_s``; a restart heals
  in the tick it respawned, and has no early end: its stop takes the time it takes;
- ``["skip", id, "fence"]``: the fence came first;
- ``["skip", id, "presence"]``: every UAV had checked in before a restart was due.

Whether the mission recovered is the gates' question
(``scripts/eval_gcs_demo_fleet_faults.py``): none runs during flight.
"""

from __future__ import annotations

from collections.abc import Sequence

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_router import BACKEND_ENDPOINT, router_endpoints
from navpy.sim.lockstep.fault_schedule import (
    FENCE_MARKER,
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PRESENCE_MARKER,
    SKIP_AT_FENCE,
    SKIP_AT_PRESENCE,
    FaultPlan,
)
from navpy.sim.lockstep.part_wire import FREEZE
from navpy.sim.lockstep.radio_cuts import RadioCuts


def _us(seconds: float) -> int:
    return round(seconds * 1e6)


class FaultRunner:
    def __init__(self, plans: Sequence[FaultPlan], roster: Roster, book: LifecycleBook,
                 cuts: RadioCuts, fleet_size: int) -> None:
        self._plans = tuple(plans)
        self._pending = list(plans)
        self._active: FaultPlan | None = None
        self._started_us = 0
        #: No fault starts before this: the last one's recovery deadline.
        self._clear_us = 0
        #: Each anchor marker's first time, whether the fence came, and who has checked in.
        self._anchors: dict[str, int] = {}
        self._fenced = False
        self._checked_in: set[int] = set()
        self._roster = roster
        self._book = book
        self._cuts = cuts
        self._fleet_size = fleet_size

    def grid(self) -> list[dict[str, object]]:
        """What the journal's grid records, before the run."""
        return [plan.to_json() for plan in self._plans]

    def tick(self, t_us: int, record: TickRecord, lines: Sequence[tuple[int, str]]) -> None:
        if not self._plans:
            return
        self._latch(t_us, lines)
        events = [] if self._active is None else self._advance(self._active, t_us)
        events += self._skips()
        if self._active is None and t_us >= self._clear_us:
            plan = next((p for p in self._pending if self._due(p, t_us)), None)
            if plan is not None:
                self._start(plan, t_us)
                events.append(["start", plan.id])
        if events:
            record.note("faults", events)

    def _latch(self, t_us: int, lines: Sequence[tuple[int, str]]) -> None:
        for ordinal, line in lines:
            if ordinal > self._fleet_size:  # the backend's own lines
                continue
            for marker in (LAUNCH_MARKER, PRESENCE_MARKER):
                if marker in line:
                    self._anchors.setdefault(marker, t_us)
            if PRESENCE_MARKER in line:
                self._checked_in.add(ordinal)
            self._fenced = self._fenced or FENCE_MARKER in line

    def _skips(self) -> list[list[object]]:
        """Plans that can no longer start, taken off the pending list, as their events."""
        if self._fenced:
            closed = [(plan, SKIP_AT_FENCE) for plan in self._pending]
        elif len(self._checked_in) == self._fleet_size:
            closed = [(plan, SKIP_AT_PRESENCE) for plan in self._pending if plan.kind == NAVPY_RESTART]
        else:
            closed = []
        for plan, _reason in closed:
            self._pending.remove(plan)
        return [["skip", plan.id, reason] for plan, reason in closed]

    def _due(self, plan: FaultPlan, t_us: int) -> bool:
        anchor = self._anchors.get(plan.anchor)
        return anchor is not None and t_us >= anchor + _us(plan.start_s)

    def _start(self, plan: FaultPlan, t_us: int) -> None:
        self._pending.remove(plan)
        self._active = plan
        self._started_us = t_us
        if plan.kind == LINK_DROP:
            others = [e for e in router_endpoints(self._fleet_size) if e != plan.target]
            self._cuts.cut(plan.id, [plan.target], [(other, plan.target) for other in others])
        elif plan.kind == LATE_HEARTBEAT:
            self._freeze(plan, True)
        elif plan.kind == NAVPY_RESTART:
            member = self._roster.current(plan.ordinal)
            assert member is not None
            self._book.stop_scheduled(member, t_us, plan.id)
        else:
            uav = f"sitl{plan.ordinal}"
            peers = [e for e in router_endpoints(self._fleet_size) if e not in (uav, BACKEND_ENDPOINT)]
            self._cuts.cut(plan.id, [], [*((uav, peer) for peer in peers), *((peer, uav) for peer in peers)])

    def _freeze(self, plan: FaultPlan, frozen: bool) -> None:
        member = self._roster.current(plan.ordinal)
        if member is not None and member.live:  # else the run is ending on its exit
            spawn = member.spawn
            self._book.commit(spawn.ordinal, FREEZE, None, spawn.ordinal, spawn.generation, None,
                              {"frozen": frozen, "fault": plan.id})

    def _advance(self, plan: FaultPlan, t_us: int) -> list[list[object]]:
        """The active fault's heal, once it is due; empty before."""
        forced = False
        if plan.kind == NAVPY_RESTART:
            member = self._roster.current(plan.ordinal)
            if member is None or member.live:  # still stopping; its stop budget bounds this
                return []
            self._book.respawn(member, plan.id)
        else:
            forced = t_us < self._started_us + _us(plan.duration_s)
            if forced and not self._fenced:
                return []
            if plan.kind == LATE_HEARTBEAT:
                self._freeze(plan, False)
            else:
                self._cuts.heal(plan.id)
        self._active = None
        self._clear_us = t_us + _us(plan.recovery_s)
        return [["heal", plan.id, self._clear_us, *(["forced"] if forced else [])]]


__all__ = ["FaultRunner"]
