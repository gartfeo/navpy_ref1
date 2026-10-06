"""A bounded row buffer that reports its own incompleteness.

Split from the recorder so the two failure modes that invalidate a run --
running out of budget, and faulting while recording -- are held in one small
object with no knowledge of what a row means.

Deliberately NOT a ring buffer. Overflow keeps the EARLIEST rows, because the
scored leg starts at activation and a trace that quietly discarded its
beginning would answer the wrong question. Overflow instead counts each row
it drops, and ``overflowed``, read from that count, makes the run invalid for
a determinism verdict. Read, never latched beside the count: an interrupt
landing between two writes would part them.

Not internally locked: the owner serialises access, so the log stays a plain
buffer and there is exactly one lock in the trace.

The first violation is latched TWICE: once trace-wide, and once per epoch.
The per-epoch latch is what lets a leg-scoped verdict survive a ``drain``.
Scanning the rows for it cannot: draining is the intended end-of-leg step,
and a summary taken after one would scan an empty list and call a violated
leg clean.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from navpy.modules.vision.sim.determinism_events import (
    EVENT_STAGE,
    STAGE_PAYLOAD_DIGEST,
)
from navpy.modules.vision.sim.determinism_slots import PAYLOAD_UNREADABLE


def is_unreadable_stage(row: tuple) -> bool:
    """A staging row whose payload was present and could not be read.

    The EVENT is checked first, and that is not defensive tidiness. Index
    ``STAGE_PAYLOAD_DIGEST`` is 7, and index 7 of an OUTPUT row is a worker
    iteration count -- an entirely different quantity that happens to live in
    the same column. Reading the column without checking the event was safe only
    by luck: an int never equals ``PAYLOAD_UNREADABLE``. Luck is not a contract,
    and the next row type to end at index 7 need not be so accommodating.

    ``PAYLOAD_UNREADABLE`` is a HOLE: two runs whose frames were both
    undigestable have not been shown to agree. A plain ``None`` means there was
    nothing to digest, which two runs CAN agree on, so it latches nothing. And
    both are separate from ``fail``: a recorder bug and an undigestable input
    are different facts about a run.
    """
    return (
        len(row) > STAGE_PAYLOAD_DIGEST
        and row[0] == EVENT_STAGE
        and row[STAGE_PAYLOAD_DIGEST] == PAYLOAD_UNREADABLE
    )


@dataclass(frozen=True)
class TraceStatus:
    """Completeness of one trace. Anything false here invalidates a verdict."""

    rows: int
    capacity: int
    dropped: int
    overflowed: bool
    failed: bool
    first_violation: tuple | None
    violations_by_epoch: Mapping[Any, tuple]
    drained: bool = False
    commands_incomplete: bool = False
    payload_unreadable: bool = False
    # Exceptions that left the source's message callback, counted by the
    # trace's guard (D3). Each is a message that can be missing its rows.
    callback_faults: int = 0
    # Recordings that reached a sealed journal and changed nothing:
    # counted, never folded into ``complete``. See determinism_seal.
    refused: int = 0

    @property
    def complete(self) -> bool:
        """Whether this trace can still substantiate a verdict.

        ``drained`` counts: draining is the intended end-of-leg step, but a
        drained log no longer holds the rows its own counts describe, so a
        summary taken after one is not a verdict. Summarise, then drain.

        ``commands_incomplete`` counts because the worker's ledger is part
        of the same evidence: a command sequence with a hole in it cannot
        show two runs issued the same commands.

        ``payload_unreadable`` counts for the same reason on the input
        side: two runs whose frames could not be digested have not been
        shown to agree. Kept separate from ``failed`` because a recorder
        bug and an undigestable input are different facts.

        ``callback_faults`` counts because a callback that raised stopped
        part-way through the decision it was recording, and nothing else in
        the rows says a decision is missing.

        ``refused`` does NOT count. A refused recording changed
        nothing, so the rows are what they were at the seal. Whether
        a case with refusals may be used is a separate rule, and zero
        refusals is not quiescence.
        """
        return not (
            self.overflowed
            or self.failed
            or self.drained
            or self.commands_incomplete
            or self.payload_unreadable
            or self.callback_faults
        )

    def first_violation_in(self, epoch: Any = None) -> tuple | None:
        """The latched violation for one leg, or the trace-wide one
        unscoped. Never derived from the rows, so a drain cannot erase it."""
        if epoch is None:
            return self.first_violation
        return self.violations_by_epoch.get(epoch)


def status_contradiction(
    status: TraceStatus,
    rows: tuple[tuple, ...],
    shared: tuple[tuple[int, tuple[tuple, ...]], ...] = (),
) -> str | None:
    """What of ``status`` the rows it describes, the records it shares its
    capacity with, or its own counts, show false: None when nothing does.
    No ``BoundedRowLog`` reports any of it, so a reader holds a manifest's
    journal to this.

    The rows are counted, and never more than the capacity. Nor are the
    records of each store in ``shared``, given as its drop count and the
    records its capacity bounds (``bounded``): ``DeterminismTrace`` gives its
    three stores one capacity, and none holds more than it. An overflow is
    reported when, and only when, a row was dropped, since the log reads it
    from that count. A store that dropped a record is full: each drops one
    only when full, and none shrinks but the journal, by a drain, which an
    interrupt can leave half done and failed. The reader checks latches against
    retained rows separately, in determinism_capture_validation.
    """
    if status.rows != len(rows) or status.rows > status.capacity:
        return f"{status.rows} rows, {len(rows)} held, capacity {status.capacity}"
    held = max((len(records) for _, bound in shared for records in bound), default=0)
    if held > status.capacity:
        return f"a store holds {held} records, past the capacity {status.capacity}"
    if status.overflowed != (status.dropped > 0):
        return f"overflowed is {status.overflowed}, {status.dropped} dropped"
    shrinkable = status.drained or status.failed
    for dropped, bound in ((0 if shrinkable else status.dropped, (rows,)), *shared):
        if dropped and status.capacity not in map(len, bound):
            return f"{dropped} dropped by a store below the capacity {status.capacity}"
    return None


class BoundedRowLog:
    """Append-only rows with a hard budget and a latched validity flag.

    ``__slots__`` is load-bearing, not a size optimisation. ``RowJournal``
    holds one of these, and the lock-ordering argument is that a command loop
    is not REACHABLE from the journal. Reachability is transitive, so an
    object the journal holds must not accept an arbitrary attribute either:
    without this, ``journal._log.worker = command_log`` parks the command loop
    one hop out and every name-based check looks straight past it. A review
    built exactly that and it passed.
    """

    __slots__ = (
        "_capacity",
        "_rows",
        "_dropped",
        "_failed",
        "_payload_unreadable",
        "_first_violation",
        "_by_epoch",
        "_drained",
        "_callback_faults",
    )

    def __init__(self, capacity: int) -> None:
        # As the other stores: a budget below zero holds nothing, as zero does.
        self._capacity = max(0, int(capacity))
        self._rows: list[tuple] = []
        self._dropped = 0
        self._failed = False
        self._payload_unreadable = False
        self._first_violation: tuple | None = None
        self._by_epoch: dict[Any, tuple] = {}
        self._drained = False
        self._callback_faults = 0

    @property
    def capacity(self) -> int:
        return self._capacity

    def append(self, row: tuple) -> None:
        """Record one row, latching what about it must outlive the rows.

        ONE DOOR, for callers that use the API. There were briefly two -- this,
        and an ``append_staged`` that latched an unreadable payload -- and a
        caller could take the wrong one and record a hole as a clean row. A guard
        on the second door would have left the first one open, so the second door
        is gone: whatever must be remembered about a row is remembered by the
        method every row goes through.

        Not a guarantee against code reaching ``_rows`` directly, which Python
        cannot prevent and a review confirmed by doing it. What IS checked, and
        checked by CALLING every public member rather than by reading the source
        for it, is that nothing this class exposes can put a row in front of a
        caller without the latch having fired on it. Two source-reading versions
        of that check were defeated by spelling -- ``extend``, then a local
        alias, then an addition inside ``drain`` where a row count cannot see
        it -- because they kept answering a question about syntax.

        The latch is applied BEFORE the capacity check, deliberately and for the
        same reason ``note_violation`` latches before appending: an overflow that
        drops the row must not also erase the fact that the row was holed. So
        this does not promise the latch describes a row still HELD -- it promises
        the latch describes a row that was actually recorded HERE.
        """
        if is_unreadable_stage(row):
            self._payload_unreadable = True
        if len(self._rows) >= self._capacity:
            self._dropped += 1
            return
        self._rows.append(row)

    def note_violation(self, row: tuple) -> None:
        """Latch the FIRST violation, trace-wide and for this row's epoch.

        Latched rather than re-derived from the rows, so it outlives both an
        overflow that dropped the row and the drain that ends the leg.
        """
        if self._first_violation is None:
            self._first_violation = row
        if len(row) > 1:
            self._by_epoch.setdefault(row[1], row)
        self.append(row)

    def fail(self) -> None:
        """Mark the log unusable. Never raises: its caller is a command path."""
        self._failed = True

    def note_callback_fault(self) -> None:
        """Count one exception that left the source's message callback."""
        self._callback_faults += 1

    def status(self) -> TraceStatus:
        return TraceStatus(
            rows=len(self._rows),
            capacity=self._capacity,
            dropped=self._dropped,
            overflowed=self._dropped > 0,
            failed=self._failed,
            first_violation=self._first_violation,
            violations_by_epoch=MappingProxyType(dict(self._by_epoch)),
            drained=self._drained,
            payload_unreadable=self._payload_unreadable,
            callback_faults=self._callback_faults,
        )

    def drain(self) -> list[tuple]:
        """Hand the rows out, and latch that this log no longer holds them."""
        rows = self._rows
        self._rows = []
        self._drained = True
        return rows

    def snapshot(self) -> list[tuple]:
        return list(self._rows)


__all__ = ["BoundedRowLog", "TraceStatus", "is_unreadable_stage", "status_contradiction"]
