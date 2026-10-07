"""Post-leg reduction of a determinism trace. Never call during a scoring window.

Kept out of the recorder because it is a READER: it walks rows and does
arithmetic, none of which may ever run on the message path.

What the series ARE:

- ``stage_lag_slots`` -- the input watermark slot when a frame was staged, minus
  the frame's own input slot. How far the input stream had advanced by the time
  the frame became publishable.
- ``dispatch_lag_slots`` -- the same difference at the instant a dispatch TOOK
  the frame, from the watermark its caller captured with the take.
- ``ready_iterations`` -- the ``NavigationCommandWorker`` loop passes at which
  dispatched frames were first ready. **Iteration counts, not slot indices.**

W9.2's r(k) is still NOT measured, and the two units above are why. r(k) is the
first logical COMMAND slot at which input k was completely ready; the worker
counts its own passes and skips a missed deadline outright
(``_next_fixed_deadline``: "Never replay a missed slot"), so pass j and
autopilot slot j diverge the first time the host is late. Subtracting an input
slot from an iteration number would produce a figure in no unit at all. What
this landing gives W9.2's calibration campaign is the pair -- input slot and the
iteration that first had it -- plus the digest of the command that iteration
issued. Mapping iterations onto the autopilot grid is Landing 2, and the frozen
L is predeclared from that mapping, not from these numbers.

Both slot series are lower bounds, because the watermark is (see
``determinism_trace``).

Everything is reduced from ONE ``DeterminismTrace.capture``: the rows and the
status beside them come from a single acquisition of the row lock, and the
command evidence from a single acquisition of its own. Read separately, a row
appended between the two reads was counted by the status and missing from the
rows -- a review produced ``rows=1, rows_recorded=2, complete=True`` that way,
a combination no instant ever held. ``summarise_capture`` reduces a capture
it is handed, for a caller that has taken one already.

``summarise`` takes an ``epoch`` because a row from the CRUISE leg must not
decide the SCORED leg's verdict -- most of all a violation, which latches
trace-wide. ``activate()`` bumps the epoch, so scoping by it scopes to one leg.
The caller must KNOW that epoch, and ``close()`` returns it. It is not read off
the rows: a dispatch that takes its frame after ``close()`` records at the
NEXT epoch (one that took it before settles at the ending epoch), and the
boundary's own LIFECYCLE row can be lost to an overflow or a recorder fault.
``epoch=None`` means every leg, never "unknown" -- a caller that does not know
which leg ended must not summarise one.
The scoped violation comes from the recorder's per-epoch LATCH, never from the
rows: draining is the intended end-of-leg step, and a summary that re-derived
the verdict from drained rows would call a violated leg clean.

Command entries are not epoch-scoped, and cannot be: the worker thread has no
activation epoch of its own. They are reported whole, and a hole in them --
dropped or faulted -- makes ``complete`` false, because a command sequence
with a gap cannot show that two runs issued the same commands. So does a
``drain``: summarise FIRST, then drain.

The ATTITUDE ledger is reported whole too, since a ruling has no epoch
either: its figures (D2), and a hole in it makes ``complete`` false.
``callback_faults`` counts the exceptions that left the source's message
callback, each a message that can be missing its rows.
"""

from __future__ import annotations

from typing import Any

from navpy.modules.vision.sim.determinism_admission_ledger import (
    LedgerCapture,
)
from navpy.modules.vision.sim.determinism_command_log import (
    COMMAND_RAISED,
    COMMAND_UNREADABLE,
    CommandCapture,
)
from navpy.modules.vision.sim.determinism_events import (
    COUNTED_EVENTS,
    EVENT_OUTPUT,
    EVENT_STAGE,
    OUTPUT_WORKER_ITERATION,
    OUTPUT_DISPATCHED,
    STAGE_STAGED,
)
from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    TraceCapture,
)


_INPUT_SLOT = 4
_WATERMARK_SLOT = 6


def _series(values: list[int]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min": None, "max": None}
    return {"count": len(values), "min": min(values), "max": max(values)}


def _lag_of(row: tuple) -> int | None:
    if len(row) <= _WATERMARK_SLOT:
        return None
    input_slot, watermark_slot = row[_INPUT_SLOT], row[_WATERMARK_SLOT]
    if input_slot is None or watermark_slot is None:
        return None
    return watermark_slot - input_slot


def _iteration_of(row: tuple) -> int | None:
    """Only ever called on an OUTPUT row -- see the caller.

    A STAGE row's column 7 is a payload DIGEST, and ``int()`` on those
    bytes would raise. The event check at the call site is what keeps them
    apart, so the index is named for the row type it belongs to.
    """
    if len(row) <= OUTPUT_WORKER_ITERATION:
        return None
    value = row[OUTPUT_WORKER_ITERATION]
    return None if value is None else int(value)


def _command_summary(commands: CommandCapture) -> dict[str, Any]:
    """What the worker issued, keyed by its own iteration count.

    The three non-byte outcomes are counted SEPARATELY, never folded into
    ``digested``. Both markers are non-None, so a "digest is not None"
    test called an unreadable command digested and reported bytes that
    were never obtained. ``digested`` now means exactly one thing: this
    many entries carry the real bytes of a command.
    """
    digests = [digest for _, digest in commands.entries]
    return {
        "iterations": commands.iterations,
        "entries": len(digests),
        "digested": sum(
            1
            for digest in digests
            if digest is not None
            and digest not in (COMMAND_RAISED, COMMAND_UNREADABLE)
        ),
        "no_command": sum(1 for digest in digests if digest is None),
        "raised": sum(1 for digest in digests if digest == COMMAND_RAISED),
        "unreadable": commands.unreadable,
        "passes": len(commands.passes),
        "dropped": commands.dropped,
        "failed": commands.failed,
    }


def _ledger_summary(ledger: LedgerCapture) -> dict[str, Any]:
    """Every ATTITUDE ruling the ledger holds, reduced (D2).

    ``first_ruling`` and ``last_ruling`` are the ends of what it HOLDS;
    whether those are the rulings it was owed is the subscription's
    evidence, which the teardown reports beside this summary.
    """
    entries = ledger.entries
    return {
        "entries": len(entries),
        "first_ruling": entries[0][0] if entries else None,
        "last_ruling": entries[-1][0] if entries else None,
        "observed": ledger.observed,
        "admitted": ledger.admitted,
        "rejections": ledger.rejections,
        "stampless": ledger.stampless,
        "gapped": ledger.gapped,
        "discontinuous": ledger.discontinuous,
        "dropped": ledger.dropped,
        "failed": ledger.failed,
        "refused": ledger.refused,
    }


def summarise(
    trace: DeterminismTrace,
    *,
    epoch: int | None = None,
) -> dict[str, Any]:
    """Counts per outcome plus the lag series, over one leg or all of them.

    Reduces ONE capture of the trace -- see the module docstring for why that
    matters, and for why ``epoch`` has to come from the caller.
    """
    return summarise_capture(trace.capture(), epoch=epoch)


def summarise_capture(
    capture: TraceCapture,
    *,
    epoch: int | None = None,
) -> dict[str, Any]:
    """``summarise`` of the capture it is handed, rather than of one it takes.

    For the evidence writer, whose three files share ONE sealed capture: a
    summary that took its own would describe another instant.
    """
    rows = capture.rows
    if epoch is not None:
        rows = tuple(row for row in rows if row[1] == epoch)
    status = capture.status
    counts: dict[str, dict[str, int]] = {
        event: {} for event in COUNTED_EVENTS
    }
    stage_lags: list[int] = []
    dispatch_lags: list[int] = []
    ready_iterations: list[int] = []
    for row in rows:
        bucket = counts.get(row[0])
        if bucket is None:
            continue
        outcome = str(row[2])
        bucket[outcome] = bucket.get(outcome, 0) + 1
        dispatched = row[0] == EVENT_OUTPUT and outcome == OUTPUT_DISPATCHED
        if dispatched:
            iteration = _iteration_of(row)
            if iteration is not None:
                ready_iterations.append(iteration)
        lag = _lag_of(row)
        if lag is None:
            continue
        if row[0] == EVENT_STAGE and outcome == STAGE_STAGED:
            stage_lags.append(lag)
        elif dispatched:
            dispatch_lags.append(lag)
    return {
        "period_us": capture.period_us,
        "epoch": epoch,
        "rows": len(rows),
        "rows_recorded": status.rows,
        "capacity": status.capacity,
        "dropped": status.dropped,
        "overflowed": status.overflowed,
        "failed": status.failed,
        "drained": status.drained,
        "payload_unreadable": status.payload_unreadable,
        "callback_faults": status.callback_faults,
        "complete": capture.complete,
        "first_violation": status.first_violation_in(epoch),
        "counts": counts,
        "stage_lag_slots": _series(stage_lags),
        "dispatch_lag_slots": _series(dispatch_lags),
        "ready_iterations": _series(ready_iterations),
        "commands": _command_summary(capture.commands),
        "ledger": _ledger_summary(capture.ledger),
    }


__all__ = ["summarise", "summarise_capture"]
