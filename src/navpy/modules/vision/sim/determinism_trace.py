"""Bounded, in-memory decision trace for the direct-pixel path.

Landing 1 of the one-clock determinism work: it RECORDS the selection decisions
that currently depend on host timing and changes none of them. It still does
not carry run/source identity, autopilot boot epoch -- ``epoch`` here counts
ACTIVATIONS, a different thing -- L, or raw integer stamps.

Contract this file keeps:

- **Decision-neutral.** Nothing here is read by control, and recording never
  raises into the caller: a fault latches ``failed``, so a broken trace can
  neither change a command nor pass as a clean one. It does cost CPU and does
  lengthen the source's critical section: decision-neutral, NOT free.
- **No file I/O during the scoring window.** The owner drains the rows after the leg,
  and a drain latches the trace incomplete: summarise FIRST, then drain.
- **One read per store.** A post-leg reader reduces a ``TraceCapture``: each
  store read in ONE acquisition of its own lock, so no row count is ever
  paired with a status from another instant.
- **No lock inversion, by OWNERSHIP.** There are three locks: the row lock,
  the command loop's own inside ``CommandLoopLog``, and the ATTITUDE ledger's
  inside ``AdmissionLedger``. Holding two at once is the shape a deadlock
  grows from. This class holds the command loop and the ledger and takes NO
  lock; ``RowJournal`` takes the row lock and is not able to reach either.
  Seven review rounds went into DETECTING that property by parsing, and every
  round found another spelling; it is now a reachability question answered by
  following values, and ``determinism_journal`` states exactly how far that
  answer reaches.
  What a row needs from another store crosses as a VALUE, read here before
  the journal is called: the worker iteration, and a ruling from the ledger.
  An earlier claim here was wrong and stays withdrawn: recording does NOT
  "never call out". ``record_stage`` runs ``source_seconds_of`` and
  ``frame_digest`` under the row lock, and those read attributes off a foreign
  object whose getter could acquire anything. What holds is narrower: the
  helpers themselves are lock-free, and no attribute the production pose path
  supplies is a lock-taking property.

The ATTITUDE watermark advances only on samples whose association succeeded or
was epoch-fenced, so it is a LOWER bound on the newest ATTITUDE source time. A
refusal (69-75 per archived run) leaves it where it was AND records no source
stamp, so no recorded regression is NOT proof source time never regressed: a
reboot seen only by refused samples goes unnoted by the rows. The ATTITUDE
ledger holds the raw ``time_boot_ms`` of every ruling, and every ASSOCIATION
row names its ruling, refusals included (D2, D3). Such a bound only
under-states if a row is attributed to the instant its caller ACTED -- see
``record_output``.

Row layouts are in ``determinism_events``; one module-level boolean gates
construction, off by default.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

from navpy.modules.vision.sim.determinism_admission_ledger import (
    AdmissionLedger,
    LedgerCapture,
)
from navpy.modules.vision.sim.determinism_command_log import (
    CommandCapture,
    CommandLoopLog,
)
from navpy.modules.vision.sim.determinism_journal import RowJournal
from navpy.modules.vision.sim.determinism_row_log import TraceStatus
from navpy.modules.vision.sim.determinism_trace_gate import (
    DEFAULT_TRACE_CAPACITY,
    DETERMINISM_TRACE_CAPACITY_ENV,
    DETERMINISM_TRACE_ENV,
    capacity_from_env,
    enabled_from_env,
)


ENABLED = enabled_from_env()


@dataclass(frozen=True)
class TraceCapture:
    """Every store as one immutable record, for a post-leg reader to reduce.

    Each store is read in ONE acquisition of its own lock -- the rows, the
    commands, then the ledger, one after another and never nested, which keeps
    the ownership rule above. So each part is internally consistent, and the
    parts describe the same instant only once nothing is recording: a producer
    still running between two reads lands in one and not the other, and one
    running after all of them is in none. That is why the owner takes it LAST,
    and why ``complete`` means recording integrity AT THIS BOUNDARY. It does
    not certify that the leg finished.
    """

    period_us: int
    rows: tuple[tuple, ...]
    status: TraceStatus
    commands: CommandCapture
    ledger: LedgerCapture

    @property
    def complete(self) -> bool:
        """The rows' and the commands' ``status.complete``, and a ledger with
        no hole. Whether the ledger holds every ruling it was owed is its
        subscription's evidence, which the owner reports (D8.5, D8.6)."""
        return self.status.complete and not self.ledger.incomplete


class DeterminismTrace:
    """Slot-keyed decision rows, the worker ledger and the ATTITUDE ledger.

    A facade over three stores that share no lock and no reference. It holds
    the command loop and the ledger; ``RowJournal`` holds the row lock. None
    can reach another's lock, which is why nothing here needs to be inspected
    for lock nesting -- see ``determinism_journal`` for what that replaced.
    """

    def __init__(self, period_us: int, *, capacity: int | None = None) -> None:
        rows = capacity_from_env() if capacity is None else int(capacity)
        self._journal = RowJournal(period_us, rows)
        self._commands = CommandLoopLog(rows)
        self._ledger = AdmissionLedger(rows)

    @property
    def period_us(self) -> int:
        return self._journal.period_us

    @property
    def command_log(self) -> CommandLoopLog:
        """The worker's own ledger, keyed by ITERATION and never by slot."""
        return self._commands

    @property
    def journal(self) -> RowJournal:
        """The row ledger. Read on its own by the admission link as it
        subscribes (D8.5), and exposed so a test can instrument its lock."""
        return self._journal

    @property
    def ledger(self) -> AdmissionLedger:
        """The ATTITUDE ledger, whose ``append`` the owner subscribes to the
        router's admission tap before the source starts (D2)."""
        return self._ledger

    def watermark_us(self) -> int | None:
        """The newest ATTITUDE stamp NOW, so a caller can attribute a row to
        the instant it acted rather than the instant it finished."""
        return self._journal.watermark_us()

    def record_association(
        self,
        *,
        epoch: int,
        outcome: str,
        source_s: Any = None,
    ) -> None:
        """One ATTITUDE ruled on: committed, refused, or epoch-fenced.

        The row names the latest ruling the ledger ADMITTED, read here in one
        acquisition of the ledger's lock, released before the journal takes
        the row lock: a VALUE, like the worker iteration (D3).
        """
        _, admitted = self._ledger.cutoff()
        self._journal.record_association(
            epoch=epoch, outcome=outcome, source_s=source_s, ruling=admitted
        )

    def record_truth(self, *, epoch: int, outcome: str, message: Any = None) -> None:
        """One SIM_STATE arrival, readable or not."""
        self._journal.record_truth(epoch=epoch, outcome=outcome, message=message)

    def record_stage(
        self,
        *,
        epoch: int,
        outcome: str,
        sample: Any = None,
        frame: Any = None,
    ) -> None:
        """A projected frame reached the publish slot, or did not."""
        self._journal.record_stage(
            epoch=epoch, outcome=outcome, sample=sample, frame=frame
        )

    def record_discard(self, *, epoch: int, reason: str, victim: Any) -> None:
        """A sample dropped before anything consumed it: the gap, itemised."""
        self._journal.record_discard(
            epoch=epoch, reason=reason, victim=victim
        )

    def record_output(
        self,
        *,
        epoch: int,
        outcome: str,
        taken_at_us: int | None,
        frame: Any = None,
    ) -> None:
        """One dispatch attempt = one logical output slot on this path.

        The ONLY place either ledger is read for the other's row, and it happens
        here, before the journal is called and therefore before any row lock is
        taken. The iteration crosses as a VALUE. If reading it faults, the
        journal is told -- a trace that lost a row must not read as complete.
        """
        try:
            iteration = self._commands.current_iteration()
        except Exception:  # noqa: BLE001 - a recorder fault must not command
            self._journal.fail()
            return
        self._journal.record_output(
            epoch=epoch,
            outcome=outcome,
            taken_at_us=taken_at_us,
            iteration=iteration,
            frame=frame,
        )

    def record_lifecycle(
        self, *, epoch: int, outcome: str, resulting_epoch: int
    ) -> None:
        """One leg boundary, at the ENDING epoch, emptied slots or not."""
        self._journal.record_lifecycle(
            epoch=epoch, outcome=outcome, resulting_epoch=resulting_epoch
        )

    def record_subscription(self, *, epoch: int, outcome: str) -> None:
        """The source's subscriptions opened or closed. The row names the
        latest ruling the ledger OBSERVED, admitted or not, read here as a
        VALUE: the bound of the window the source was subscribed for (D3)."""
        observed, _ = self._ledger.cutoff()
        self._journal.record_subscription(
            epoch=epoch, outcome=outcome, ruling=observed
        )

    def guard_callback(
        self, callback: Callable[[Any], None]
    ) -> Callable[[Any], None]:
        """``callback``, with every exception that leaves it counted (D3).

        The vehicle's callback registry catches what a message callback
        raises, so a source whose callback raised records nothing for that
        message and the rows could not say so. This counts it in the journal
        and raises the SAME object on, so the registry sees what it saw
        before. A count that fails flags the journal instead, and the same
        object is still raised. An interrupt that lands in the count is a
        recording's interrupt: D5 passes it on as itself, with the journal
        failed and the callback's exception as its context. Raising that
        exception instead would swallow the interrupt, which the untraced
        registry, catching Exception alone, never does.
        """
        journal = self._journal

        def guarded(message: Any) -> None:
            try:
                callback(message)
            except BaseException:
                journal.count_callback_fault()
                raise

        return guarded

    def capture(self) -> TraceCapture:
        """Every store, each in one acquisition -- see ``TraceCapture``.

        O(rows + entries): a post-leg read, never an scoring-path one. A
        hole in the command ledger folds into ``status``, one in the ATTITUDE
        ledger into ``TraceCapture.complete``.
        """
        rows, status = self._journal.capture()
        commands = self._commands.capture()
        ledger = self._ledger.capture()
        return TraceCapture(
            period_us=self._journal.period_us,
            rows=rows,
            status=replace(status, commands_incomplete=commands.incomplete),
            commands=commands,
            ledger=ledger,
        )


def build_trace(period_us: int) -> DeterminismTrace | None:
    """A trace when enabled, else None so callers skip the work entirely."""
    if not ENABLED:
        return None
    return DeterminismTrace(period_us)


__all__ = [
    "DEFAULT_TRACE_CAPACITY",
    "DETERMINISM_TRACE_CAPACITY_ENV",
    "DETERMINISM_TRACE_ENV",
    "ENABLED",
    "DeterminismTrace",
    "TraceCapture",
    "TraceStatus",
    "build_trace",
]
