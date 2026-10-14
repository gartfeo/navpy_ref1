"""The lockstep coordinator's run: boot, mission, finalization, drain, snapshot.

``lockstep_slice.md``, "Run phases" and "Tick k", for N SITLs
(``lockstep_fleet.md``; the slice is N = 1). Per tick k:

1. Read and check frame k from every SITL, then the fleet clock
   (:class:`~navpy.sim.lockstep.coordinator_fleet.SitlFleet`).
2-4. :class:`~navpy.sim.lockstep.coordinator_tick.TickExchange`.
5. The operator's baton runs to quiescence at ``t_k``; its requests travel in
   ``TICK k+1``.
6. Reply k to every SITL, then journal the tick.

Every wall wait of steps 1-6 draws on the tick's one exchange deadline
(``coordinator_deadline``). The initial participants spawn and register
before the SITLs are launched (``launch``), so frame 1's deadline covers
only the launches.

Boot ticks run no participant, up to and including the tick at which the
last SITL latches ``helpers_quiet`` (the boot barrier, within the
manifest's boot budget); the initial participants are admitted at the tick
after it. The mission ends ``ok`` at k_T, the tick at which every expected
companion has latched its first terminal marker (k_T is the latest of
those ticks), or as a journaled timeout at the scenario's sim budget. An
operator whose run returns before k_T (its own sim-time workflow ended
without the terminal marker, e.g. a confirmation timeout) ends the mission
at that tick with the journaled outcome ``operator_returned``, never
``ok``; its companion stop still has to have been acked, so drain and
snapshot run as usual. If the last first SNAP lands in the tick the budget
or the operator's return also hits, completion wins.
Finalization ends at k_F, the tick the operator's run returns; by then every
companion-role participant must have retired through an acked participant
stop. Drain reads every SITL's parked frame k_F+1 (checked like any frame)
and never replies, then runs synthetic ticks with ``end`` until every
participant ended. The snapshot is taken only after every participant ended
flushed and exited; then every SITL is stopped by ownership.

Port bytes take the manifest's path (``coordinator_ports``): the slice's
direct links, or a routed fleet's router, queues and packet records
(``coordinator_router``). The journal opens with the run's tick-grid
parameters and closes with the MAVLink message counts per link and
direction (``MavlinkTally``; tick records carry their NAVLINK counts),
including the parked frames' egress and the participants' drain egress. A fault journals its class, kind
and logical context only; its message goes to the ``*_host`` sidecar. Every
fault stops by ownership. A one-SITL run keeps the slice's journal schema
until step 10c re-baselines it (owner Q1): fleet-only fields (the fleet
header, latch ticks, first SNAPs) are journaled only for a fleet.
"""

from __future__ import annotations

import json
import logging
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from navpy.sim.lockstep.baton import LockstepFault
from navpy.sim.lockstep.coordinator_deadline import OPERATOR as OPERATOR_WAIT
from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
from navpy.sim.lockstep.coordinator_fleet import Sitl, SitlFleet, SitlHost, anchor_now
from navpy.sim.lockstep.coordinator_frames import SitlLink
from navpy.sim.lockstep.coordinator_journal import RunJournal, TickRecord, sha256
from navpy.sim.lockstep.coordinator_latches import SnapLatch
from navpy.sim.lockstep.coordinator_lifecycle import OPERATOR, LifecycleBook
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_operator import deliver_operator_requests
from navpy.sim.lockstep.coordinator_ports import make_ports
from navpy.sim.lockstep.coordinator_types import (
    DRAIN_BUDGET_US,
    FINALIZATION_BUDGET_US,
    CoordinatorFault,
    HostFault,
    Manifest,
    OperatorHost,
    OperatorInbox,
    RunFault,
)
from navpy.sim.lockstep.coordinator_tick import TickExchange, TickOutput
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.step_wire import Frame

FAULT_HOST_FILE = "lockstep_fault_host.json"
_log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RunResult:
    outcome: str
    terminal_tick: int
    final_tick: int
    parked_tick: int


class LockstepCoordinator:
    """``sitls`` are in SITL order, matching ``manifest.sitl_ports``. The
    exchange deadline is the roster's (``Roster.deadline``), so every wait
    of a tick draws on the same one; ``launch`` launches the SITLs after the
    initial participants registered and anchors frame 1's deadline."""

    def __init__(self, manifest: Manifest, sitls: Sequence[Sitl], operator: OperatorHost,
                 roster: Roster, ids: RequestIds, host_dir: Path, *,
                 delay: Callable[[str], None] = lambda point: None,
                 launch: Callable[[ExchangeDeadline], None] = anchor_now) -> None:
        self._manifest = manifest
        self._fleet = SitlFleet(manifest, sitls, roster.deadline, launch=launch)
        self._operator = operator
        self._roster = roster
        self._book = LifecycleBook(roster, ids)
        self._exchange = TickExchange(roster, self._book, manifest.companion_roles,
                                      make_ports(manifest, roster))
        self._snaps = SnapLatch(manifest.expected_fleet)
        self._ids = ids
        self._host_dir = host_dir
        self._delay = delay
        self._k = 0
        self.journal = RunJournal()

    def run(self) -> RunResult:
        self.journal.add({"grid": self._grid()})
        try:
            for role, ordinal in self._manifest.initial:
                self._roster.spawn(role, ordinal, self._ids.next(), OPERATOR)
            self._fleet.launch()
            k_t, k_f, t_f, outcome = self._mission()
            self._check_companions_stopped(k_f)
            parked = self._park(k_f)
            self._drain(k_f, t_f)
            counts = self._exchange.ports.closing()
            self._fleet.snapshot(parked)
            self._delay("before_stop")
            self._fleet.stop()
        except BaseException as fault:  # any end before a clean one stops by ownership
            self._fail(fault)
            raise
        self._roster.close()
        self.journal.add(counts)
        end = {"outcome": outcome, "terminal_tick": k_t, "final_tick": k_f, "parked_tick": parked}
        if self._manifest.is_fleet:
            end["first_snaps"] = self._snaps.records()
        self.journal.add(end)
        return RunResult(outcome, k_t, k_f, parked)

    def _grid(self) -> dict[str, object]:
        plan = self._manifest
        grid: dict[str, object] = {
            "max_frame_gap_us": plan.max_frame_gap_us,
            "drain_period_us": plan.drain_period_us,
            "mission_budget_us": plan.mission_budget_us,
        }
        if plan.is_fleet:
            grid.update(sitl_ports=[list(ports) for ports in plan.sitl_ports],
                        boot_budget_us=plan.boot_budget_us, expected_fleet=list(plan.expected_fleet))
        return grid

    def _fail(self, fault: BaseException) -> None:
        if isinstance(fault, RunFault):
            self.journal.add(fault.journal_record(self._k))
        else:
            self.journal.add({"fault": "unexpected", "kind": type(fault).__name__,
                              "tick": self._k, "ordinal": None})
        # Each stage runs whatever the others do; the host diagnostics go last.
        # A cleanup error is logged, never raised: the original fault stands.
        for stage in (self._roster.stop_all, self._fleet.stop, lambda: self._write_fault(fault)):
            try:
                stage()
            except Exception:
                _log.exception("lockstep fault cleanup stage failed")

    def _write_fault(self, fault: BaseException) -> None:
        self._host_dir.mkdir(parents=True, exist_ok=True)
        (self._host_dir / FAULT_HOST_FILE).write_text(json.dumps(
            {"tick": self._k, "class": type(fault).__name__, "message": str(fault)}, indent=2),
            encoding="utf-8")

    def _mission(self) -> tuple[int, int, int, str]:
        k, phase, started_us, k_t, t_kt, outcome = 0, "boot", 0, 0, 0, "ok"
        fleet, ports = self._fleet, self._exchange.ports
        while True:
            k = self._k = k + 1
            frames, raws = fleet.read(k)
            t_us = frames[0].source_us
            if phase == "boot" and fleet.boot.complete:
                phase, started_us = "mission", t_us
            record = TickRecord(k, phase, t_us)
            record.note("frame", fleet.per_sitl([sha256(raw) for raw in raws]))
            ports.frames(k, record, fleet.egress(frames))
            if phase == "boot":
                latched = fleet.boot.observe(k, t_us, [frame.helpers_quiet for frame in frames])
                if latched and self._manifest.is_fleet:
                    record.note("latched", [index + 1 for index in latched])
                ports.inbound(k, record, {})  # no participant runs: bytes for one are discarded
                self._reply(k, record, frames)
                continue
            output = self._exchange.run(k, t_us, record, draining=False)
            self._run_operator(k, t_us, output, record)
            self._reply(k, record, frames)
            self._raise_faults(output)
            if phase == "mission":
                self._snaps.observe(k, output.snaps)
                if self._snaps.complete:  # checked first: completion wins a shared tick
                    phase, k_t, t_kt = "finalization", self._snaps.terminal_tick, t_us
                elif t_us - started_us >= self._manifest.mission_budget_us:
                    phase, k_t, t_kt, outcome = "finalization", k, t_us, "timeout"
                    self.journal.add({"timeout": "mission sim-time budget", "k": k})
                elif self._operator.done:
                    phase, k_t, t_kt, outcome = "finalization", k, t_us, "operator_returned"
                    self.journal.add({"operator_returned": k})
            if phase == "finalization" and self._operator.done:
                return k_t, k, t_us, outcome
            if phase == "finalization" and t_us - t_kt > FINALIZATION_BUDGET_US:
                raise CoordinatorFault("finalization_budget", "finalization passed its budget", tick=k)

    def _check_companions_stopped(self, k_f: int) -> None:
        """Before drain, each companion has retired through an acked participant stop."""
        for member in self._roster.members:
            spawn = member.spawn
            if spawn.role in self._manifest.companion_roles and not (
                not member.live and self._book.stopped(spawn.ordinal, spawn.generation)
            ):
                raise HostFault("companion_not_stopped",
                                "a companion was not stopped through the participant stop "
                                "before drain", tick=k_f, ordinal=spawn.ordinal)

    def _inbox(self, k: int, t_us: int, output: TickOutput,
               wall_deadline: float | None = None) -> OperatorInbox:
        return OperatorInbox(k, t_us, tuple(output.responses), tuple(output.events),
                             tuple(output.log_lines), tuple(self._book.operator), wall_deadline)

    def _run_operator(self, k: int, t_us: int, output: TickOutput, record: TickRecord) -> None:
        if self._operator.done:
            return
        deadline = self._roster.deadline
        # The baton's handoffs and timer rounds all draw on what is left of D_k.
        left_s = deadline.budget(OPERATOR_WAIT, math.inf)
        inbox = self._inbox(k, t_us, output, None if math.isinf(left_s) else time.monotonic() + left_s)
        try:
            requests = self._operator.run_tick(inbox)
        except LockstepFault as error:  # a handoff past its budget: the deadline's, once spent
            deadline.raise_if_spent(OPERATOR_WAIT, error)
            raise
        deadline.check(OPERATOR_WAIT)
        deliver_operator_requests(self._roster, requests, k)
        self._note_operator(record)

    def _note_operator(self, record: TickRecord) -> None:
        observed = self._operator.observations()
        if observed:  # what the operator's ports saw this tick: receipts, drops
            record.note("operator", observed)

    def _reply(self, k: int, record: TickRecord, frames: list[Frame]) -> None:
        raws = self._fleet.reply(k, self._exchange.ports.reply(k, record, frames))
        record.note("reply", self._fleet.per_sitl([sha256(raw) for raw in raws]))
        self.journal.add(record.data)
        self._delay("after_reply")

    def _raise_faults(self, output: TickOutput) -> None:
        if output.faults:
            raise output.faults[0]
        if self._book.faults:
            raise self._book.faults[0]

    def _park(self, k_f: int) -> int:
        """Read every SITL's parked frame k_F+1 with every check; journal it; never reply."""
        self._k = k_f + 1
        parked, egress = self._fleet.park(k_f + 1)
        self._exchange.ports.park(parked, egress)
        self.journal.add({"parked": parked})
        self._delay("after_parked")
        return parked["tick"]

    def _drain(self, k_f: int, t_f: int) -> None:
        k = k_f
        while any(m.live for m in self._roster.members):
            k = self._k = k + 1
            t_us = t_f + (k - k_f) * self._manifest.drain_period_us
            if t_us - t_f > DRAIN_BUDGET_US:
                raise CoordinatorFault("drain_budget", "drain passed its budget", tick=k)
            record = TickRecord(k, "drain", t_us)
            output = self._exchange.run(k, t_us, record, draining=True)
            self._exchange.ports.drained(record)
            self._operator.deliver_drain(self._inbox(k, t_us, output))
            self._note_operator(record)
            self.journal.add(record.data)
            self._raise_faults(output)
        for member in self._roster.members:
            if not member.flushed:
                raise HostFault("unclean_end", "ended unflushed", tick=member.retired_at,
                                ordinal=member.ordinal)


__all__ = ["FAULT_HOST_FILE", "LockstepCoordinator", "RunResult", "Sitl", "SitlHost", "SitlLink"]
