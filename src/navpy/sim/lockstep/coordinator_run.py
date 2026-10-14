"""The lockstep coordinator's run: boot, mission, finalization, drain, snapshot.

``lockstep_slice.md``, "Run phases" and "Tick k". Per SITL tick k:

1. Read and check frame k (:class:`~navpy.sim.lockstep.coordinator_frames.FrameReader`).
2-4. :class:`~navpy.sim.lockstep.coordinator_tick.TickExchange`.
5. The operator's baton runs to quiescence at ``t_k``; its requests travel in
   ``TICK k+1``.
6. Reply k, then journal the tick.

Boot ticks run no participant, up to and including the first frame that
reports ``helpers_quiet`` (the boot barrier); the initial participants are
admitted at the tick after it. The mission ends at k_T, the first tick whose
log lines carry the terminal marker, or as a journaled timeout at the
scenario's sim budget; the operator returning before k_T is a fault.
Finalization ends at k_F, the tick the operator's run returns; by then every
companion-role participant must have retired through an acked participant
stop. Drain reads the parked frame k_F+1 (checked like any frame) and never
replies, then runs synthetic ticks with ``end`` until every participant
ended. The snapshot is taken only after every participant ended flushed and
exited; then the SITL is stopped by ownership.

A fault journals its class, kind and logical context only; its message goes
to the ``*_host`` sidecar. Every fault stops by ownership.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from navpy.sim.lockstep.coordinator_frames import FrameReader, SitlLink
from navpy.sim.lockstep.coordinator_journal import RunJournal, TickRecord, sha256
from navpy.sim.lockstep.coordinator_lifecycle import OPERATOR, LifecycleBook
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_operator import deliver_operator_requests
from navpy.sim.lockstep.coordinator_types import (
    DRAIN_BUDGET_US,
    FINALIZATION_BUDGET_US,
    SERIAL_DEVICES,
    CoordinatorFault,
    HostFault,
    Manifest,
    OperatorHost,
    OperatorInbox,
    RunFault,
)
from navpy.sim.lockstep.coordinator_tick import TickExchange, TickOutput
from navpy.sim.lockstep.participant_hooks import RequestIds

FAULT_HOST_FILE = "lockstep_fault_host.json"
_log = logging.getLogger(__name__)


class SitlHost(Protocol):
    def snapshot(self, parked_tick: int) -> None:
        """Take the SITL artifacts up to and including the parked frame's row."""

    def stop(self) -> None:
        """Stop the SITL (and relay) by ownership."""


@dataclass(frozen=True)
class RunResult:
    outcome: str
    terminal_tick: int
    final_tick: int
    parked_tick: int


class LockstepCoordinator:
    def __init__(self, manifest: Manifest, link: SitlLink, sitl: SitlHost, operator: OperatorHost,
                 roster: Roster, ids: RequestIds, host_dir: Path, *,
                 delay: Callable[[str], None] = lambda point: None) -> None:
        self._manifest = manifest
        self._frames = FrameReader(link, manifest.max_frame_gap_us)
        self._sitl = sitl
        self._operator = operator
        self._roster = roster
        self._book = LifecycleBook(roster, ids)
        self._exchange = TickExchange(roster, self._book, manifest.companion_roles)
        self._ids = ids
        self._host_dir = host_dir
        self._delay = delay
        self._k = 0
        self.journal = RunJournal()

    def run(self) -> RunResult:
        try:
            for role, ordinal in self._manifest.initial:
                self._roster.spawn(role, ordinal, self._ids.next(), OPERATOR)
            k_t, k_f, t_f, outcome = self._mission()
            self._check_companions_stopped(k_f)
            parked = self._park(k_f)
            self._drain(k_f, t_f)
            self._sitl.snapshot(parked)
            self._delay("before_stop")
            self._sitl.stop()
        except BaseException as fault:  # any end before a clean one stops by ownership
            self._fail(fault)
            raise
        self._roster.close()
        self.journal.add({"outcome": outcome, "terminal_tick": k_t, "final_tick": k_f,
                          "parked_tick": parked})
        return RunResult(outcome, k_t, k_f, parked)

    def _fail(self, fault: BaseException) -> None:
        if isinstance(fault, RunFault):
            self.journal.add(fault.journal_record(self._k))
        else:
            self.journal.add({"fault": "unexpected", "kind": type(fault).__name__,
                              "tick": self._k, "ordinal": None})
        # Each stage runs whatever the others do; the host diagnostics go last.
        # A cleanup error is logged, never raised: the original fault stands.
        for stage in (self._roster.stop_all, self._sitl.stop, lambda: self._write_fault(fault)):
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
        k, phase, barrier, started_us, k_t, t_kt, outcome = 0, "boot", False, 0, 0, 0, "ok"
        while True:
            k = self._k = k + 1
            frame, raw = self._frames.read(k)
            if phase == "boot" and barrier:
                phase, started_us = "mission", frame.source_us
            t_us = frame.source_us
            record = TickRecord(k, phase, t_us)
            record.note("frame", sha256(raw))
            egress = {SERIAL_DEVICES[i]: port.egress for i, port in enumerate(frame.ports)}
            if phase == "boot":
                barrier = barrier or frame.helpers_quiet
                for device, data in egress.items():
                    record.discard(device, len(data))
                self._reply(k, record, TickOutput())
                continue
            output = self._exchange.run(k, t_us, record, egress, draining=False)
            self._run_operator(k, t_us, output, record)
            self._reply(k, record, output)
            self._raise_faults(output)
            if phase == "mission":
                if output.terminal:
                    phase, k_t, t_kt = "finalization", k, t_us
                elif t_us - started_us >= self._manifest.mission_budget_us:
                    phase, k_t, t_kt, outcome = "finalization", k, t_us, "timeout"
                    self.journal.add({"timeout": "mission sim-time budget", "k": k})
                elif self._operator.done:
                    raise CoordinatorFault("operator_early",
                                           "the operator returned before the mission terminal tick", tick=k)
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

    def _inbox(self, k: int, t_us: int, output: TickOutput) -> OperatorInbox:
        return OperatorInbox(k, t_us, tuple(output.responses), tuple(output.events),
                             tuple(output.log_lines), tuple(self._book.operator))

    def _run_operator(self, k: int, t_us: int, output: TickOutput, record: TickRecord) -> None:
        if self._operator.done:
            return
        deliver_operator_requests(self._roster, self._operator.run_tick(self._inbox(k, t_us, output)), k)
        self._note_operator(record)

    def _note_operator(self, record: TickRecord) -> None:
        observed = self._operator.observations()
        if observed:  # what the operator's ports saw this tick: receipts, drops
            record.note("operator", observed)

    def _reply(self, k: int, record: TickRecord, output: TickOutput) -> None:
        raw = self._frames.reply(k, tuple(output.ingress.get(device, b"") for device in SERIAL_DEVICES))
        record.note("reply", sha256(raw))
        self.journal.add(record.data)
        self._delay("after_reply")

    def _raise_faults(self, output: TickOutput) -> None:
        if output.faults:
            raise output.faults[0]
        if self._book.faults:
            raise self._book.faults[0]

    def _park(self, k_f: int) -> int:
        """Read the parked frame k_F+1 with every frame check; journal it; never reply."""
        self._k = k_f + 1
        frame, raw = self._frames.read(k_f + 1)
        self.journal.add({"parked": {"tick": frame.tick, "length": len(raw), "sha256": sha256(raw)}})
        self._delay("after_parked")
        return frame.tick

    def _drain(self, k_f: int, t_f: int) -> None:
        k = k_f
        while any(m.live for m in self._roster.members):
            k = self._k = k + 1
            t_us = t_f + (k - k_f) * self._manifest.drain_period_us
            if t_us - t_f > DRAIN_BUDGET_US:
                raise CoordinatorFault("drain_budget", "drain passed its budget", tick=k)
            record = TickRecord(k, "drain", t_us)
            output = self._exchange.run(k, t_us, record, None, draining=True)
            self._operator.deliver_drain(self._inbox(k, t_us, output))
            self._note_operator(record)
            self.journal.add(record.data)
            self._raise_faults(output)
        for member in self._roster.members:
            if not member.flushed:
                raise HostFault("unclean_end", "ended unflushed", tick=member.retired_at,
                                ordinal=member.ordinal)


__all__ = ["FAULT_HOST_FILE", "LockstepCoordinator", "RunResult", "SitlHost", "SitlLink"]
