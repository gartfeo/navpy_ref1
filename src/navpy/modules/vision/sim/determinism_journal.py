"""The row journal: everything that happens while the ROW LOCK is held.

Split out of ``determinism_trace`` so that a lock-ordering bug has to be
BUILT rather than merely typed: the command loop is not held here, so reaching
it takes putting it somewhere first. That is weaker than "unconstructible",
which an earlier version of this docstring claimed and a review disproved.

The trace owns three locks -- this one, over the row list, the command loop's
own inside ``CommandLoopLog``, and the ATTITUDE ledger's inside
``AdmissionLedger``. Holding two at once is the shape a deadlock grows from,
so no recording call may. Seven review rounds tried to prove that by
PARSING the source and every round found another spelling.
The way out was to stop
asking which spellings a parser should reject and start asking whether the
command loop is REACHABLE from here, which is answered by following values.

``DeterminismTrace`` keeps the command loop and the ledger and takes no row
lock of its own. What a row needs FROM them -- the worker iteration, a ruling
-- is read there and passed in as a VALUE. ``__slots__`` here and on
``BoundedRowLog`` are part of that rather than a size optimisation:
reachability is TRANSITIVE, and a review parked the command loop one hop out
on the row log, where every name-based check looked straight past it.

What is established, at its real width:

- Five fields here, ten on the row log and three on the seal state, all
  enforced by ``__slots__``, so none of them accepts a new attribute.
- This module imports neither the command loop nor the ledger, and this class
  names neither.
- After every recorder has run, neither is reachable from the journal over the
  live object graph, walked from the journal AND from this module's
  globals. The traversal is ``gc.get_referents``, the interpreter's own, because
  a hand-written attribute walk missed five hiding places in one review.

What is NOT established, and is not claimed. ``gc.get_referents`` is documented
as not necessarily returning every directly reachable object -- a weak reference
is one it returns nothing for -- so the walk handles the cases it knows and is a
strong check, not a proof. It does not descend into modules, classes, or a
function's globals. And a FOREIGN object handed in as a frame or sample could
have a getter that acquires anything; the runtime overlap measurement is kept
for that.

Every recording -- ``fail`` and ``drain`` included -- is ONE critical section
of the row lock: refused once ``seal`` has run, and a fault inside it counted
before the lock is released. ``determinism_seal`` holds that rule for every
ledger. Nothing but an interrupt raises into the caller, because the caller is
a command path, and an interrupt is flagged before it leaves.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import replace
from typing import Any, TypeVar

from navpy.modules.vision.sim.determinism_row_layouts import (
    association_row,
    discard_row,
    lifecycle_row,
    output_row,
    stage_row,
    subscription_row,
    truth_row,
)
from navpy.modules.vision.sim.determinism_row_log import (
    BoundedRowLog,
    TraceStatus,
)
from navpy.modules.vision.sim.determinism_seal import (
    SealState,
    record_under,
    seal_under,
)
from navpy.modules.vision.sim.determinism_watermark import advanced_watermark
from navpy.modules.vision.sim.determinism_truth_sample import capture_truth
from navpy.modules.vision.sim.determinism_slots import (
    source_microseconds,
    source_seconds_of,
)

_T = TypeVar("_T")


def _record(journal: RowJournal, change: Callable[[], _T]) -> _T | None:
    """Run ``change`` as ONE recording under the row lock.

    On an error the journal's flag is set first and the row log latched
    second, both before the lock is released. The log is latched as well, so
    its own status says a row was lost without depending on the fold in
    ``capture``. An interrupt sets the flag alone (``determinism_seal``).
    """
    return record_under(
        journal._lock, journal._seal, change, lambda: journal._log.fail()
    )


class RowJournal:
    """Slot-keyed rows over a bounded log, under one lock, safe from any thread.

    Holds no command loop. The module docstring states what that buys and,
    equally, what it does not. Each row's layout is spelled once, in
    ``determinism_row_layouts``; what goes into it, and when, is decided here.
    """

    __slots__ = ("_period_us", "_lock", "_log", "_watermark_us", "_seal")

    def __init__(self, period_us: int, capacity: int) -> None:
        self._period_us = int(period_us) if period_us and period_us > 0 else 0
        self._lock = threading.Lock()
        self._log = BoundedRowLog(int(capacity))
        self._watermark_us: int | None = None
        # Sealed, refused and faulted, all under the row lock.
        self._seal = SealState()

    @property
    def period_us(self) -> int:
        return self._period_us

    def watermark_us(self) -> int | None:
        """The newest ATTITUDE stamp NOW, so a caller can attribute a row to
        the instant it acted rather than the instant it finished."""
        with self._lock:
            return self._watermark_us

    def record_association(
        self,
        *,
        epoch: int,
        outcome: str,
        source_s: Any = None,
        ruling: int | None = None,
    ) -> None:
        """One ATTITUDE ruled on: committed, refused, or epoch-fenced.

        The only place the ATTITUDE watermark advances. ``ruling`` is the
        latest the ledger admitted, read by the trace as a VALUE.
        """
        def change() -> None:
            micros = source_microseconds(source_s)
            if micros is not None:
                self._watermark_us = advanced_watermark(
                    self._log, self._watermark_us, epoch, micros
                )
            self._log.append(association_row(
                epoch, outcome, micros, self._period_us, ruling
            ))

        _record(self, change)

    def record_truth(self, *, epoch: int, outcome: str, message: Any = None) -> None:
        """One SIM_STATE event and raw observation, in a single bounded row."""
        _record(self, lambda: self._log.append(truth_row(
            epoch, outcome, self._watermark_us, self._period_us, capture_truth(message)
        )))

    def record_stage(
        self,
        *,
        epoch: int,
        outcome: str,
        sample: Any = None,
        frame: Any = None,
    ) -> None:
        """A projected frame reached the publish slot, or did not.

        ``sample`` supplies the input slot even when there is no frame -- a
        failed projection still consumed one. ``frame``, when present, also
        supplies the payload digest.
        """
        _record(self, lambda: self._log.append(stage_row(
            epoch,
            outcome,
            source_microseconds(
                source_seconds_of(frame if frame is not None else sample)
            ),
            self._watermark_us,
            frame,
            self._period_us,
        )))

    def record_discard(self, *, epoch: int, reason: str, victim: Any) -> None:
        """A sample dropped before anything consumed it: the gap, itemised."""
        _record(self, lambda: self._log.append(discard_row(
            epoch,
            reason,
            source_microseconds(source_seconds_of(victim)),
            self._watermark_us,
            self._period_us,
        )))

    def record_output(
        self,
        *,
        epoch: int,
        outcome: str,
        taken_at_us: int | None,
        iteration: int | None,
        frame: Any = None,
    ) -> None:
        """One dispatch attempt = one logical output slot on this path.

        ``taken_at_us`` is the watermark the caller captured when it TOOK the
        frame. Delivery runs unlocked, so reading the live watermark here
        would count an ATTITUDE that arrived DURING delivery as already known
        when the frame was chosen, and overstate the frame's lag.

        ``iteration`` is the worker iteration this dispatch ran in, read from
        the command loop by the caller and passed in for the reason the module
        docstring gives. Not a slot number -- see ``determinism_command_log``.
        """
        _record(self, lambda: self._log.append(output_row(
            epoch,
            outcome,
            source_microseconds(source_seconds_of(frame)),
            taken_at_us,
            iteration,
            self._period_us,
        )))

    def record_lifecycle(
        self, *, epoch: int, outcome: str, resulting_epoch: int
    ) -> None:
        """One leg boundary, at the ENDING epoch, emptied slots or not."""
        _record(self, lambda: self._log.append(lifecycle_row(
            epoch, outcome, resulting_epoch, self._watermark_us,
            self._period_us,
        )))

    def record_subscription(
        self, *, epoch: int, outcome: str, ruling: int | None
    ) -> None:
        """The source's subscriptions opened or closed, with the latest
        ruling the ledger observed, read by the trace as a VALUE."""
        _record(self, lambda: self._log.append(subscription_row(
            epoch, outcome, ruling
        )))

    def count_callback_fault(self) -> None:
        """One exception left the source's message callback (D3)."""
        _record(self, lambda: self._log.note_callback_fault())

    def fail(self) -> None:
        """Latch a fault the CALLER hit before delegating, as one recording.

        ``record_output`` needs a worker iteration and reading it can fail,
        outside this class -- so a trace that lost a row has to be able to say
        so from outside too. One acquisition, refused once sealed like every
        recording; the caller arrives after the command loop released its own
        lock, so there is nothing to nest. The fault is this recording's
        content, so the flag comes first and the log's latch second, as for
        any fault (D5). Cannot raise, like the recorders.
        """
        def change() -> None:
            self._seal.faulted = True
            self._log.fail()

        _record(self, change)

    def seal(self) -> None:
        """Refuse every later recording, ``fail`` and ``drain``, counting each.

        Takes the row lock alone: an owner sealing the other ledger as well
        seals the two one after the other, never nested.
        """
        seal_under(self._lock, self._seal)

    def capture(self) -> tuple[tuple[tuple, ...], TraceStatus]:
        """The rows and this ledger's status, from ONE acquisition.

        Taken under two, a row appended between them is counted by the status
        and missing from the rows -- a pair no instant ever held. The fault
        flag is folded in, because a fault the log could not even latch still
        means a row was lost, and the refusal count is read in the same
        acquisition. The trace folds in the other stores.
        """
        with self._lock:
            rows = tuple(self._log.snapshot())
            status = self._log.status()
            faulted, refused = self._seal.faulted, self._seal.refused
        return rows, replace(
            status, failed=status.failed or faulted, refused=refused
        )

    def drain(self) -> list[tuple]:
        """Hand out the rows. Called after the leg, never during it.

        A mutation like any recording, so once sealed it is refused, counted,
        and hands out nothing.
        """
        rows = _record(self, lambda: self._log.drain())
        return [] if rows is None else rows


__all__ = ["RowJournal"]
