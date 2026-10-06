"""Where each field of a determinism-trace row sits: the layouts, as builders.

``determinism_events`` names what a row can say and documents its layout; this
module is that layout as code. Split out of ``RowJournal`` so the journal keeps
only what needs its lock -- the watermark, the bounded log and the fault latch
-- and so each layout is spelled once, beside the vocabulary, rather than inline
in a recorder.

Pure: no lock, no state, no I/O. The caller holds the row lock while it calls
one, because a watermark it passes in is only true under that lock. Each
builder evaluates its fields left to right, in the order the recorders always
did, so a field that raises raises at the same point as before and the frame's
payload digest is still taken last. A ruling is passed in as a value, read
from the ATTITUDE ledger before the row lock was taken.
"""

from __future__ import annotations

from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
)
from navpy.modules.vision.sim.determinism_slots import frame_digest, slot_index
from navpy.modules.vision.sim.determinism_truth_sample import UNAVAILABLE


def association_row(
    epoch: int,
    outcome: str,
    micros: int | None,
    period_us: int,
    ruling: int | None,
) -> tuple[Any, ...]:
    """(ASSOCIATION, epoch, outcome, source_us, slot, ruling)"""
    return (
        EVENT_ASSOCIATION,
        int(epoch),
        str(outcome),
        micros,
        slot_index(micros, period_us),
        None if ruling is None else int(ruling),
    )


def truth_row(
    epoch: int,
    outcome: str,
    watermark_us: int | None,
    period_us: int,
    raw_sample: tuple = UNAVAILABLE,
) -> tuple[Any, ...]:
    """V2: one atomic event and its original raw sample (v1 omitted the tail)."""
    return (
        EVENT_TRUTH,
        int(epoch),
        str(outcome),
        watermark_us,
        slot_index(watermark_us, period_us),
        raw_sample,
    )


def stage_row(
    epoch: int,
    outcome: str,
    micros: int | None,
    watermark_us: int | None,
    frame: Any,
    period_us: int,
) -> tuple[Any, ...]:
    """(STAGE, epoch, outcome, source_us, slot, watermark_us, watermark_slot,
    digest)"""
    return (
        EVENT_STAGE,
        int(epoch),
        str(outcome),
        micros,
        slot_index(micros, period_us),
        watermark_us,
        slot_index(watermark_us, period_us),
        frame_digest(frame),
    )


def discard_row(
    epoch: int,
    reason: str,
    micros: int | None,
    watermark_us: int | None,
    period_us: int,
) -> tuple[Any, ...]:
    """(DECIMATE, epoch, reason, source_us, slot, watermark_us, watermark_slot)"""
    return (
        EVENT_DECIMATE,
        int(epoch),
        str(reason),
        micros,
        slot_index(micros, period_us),
        watermark_us,
        slot_index(watermark_us, period_us),
    )


def output_row(
    epoch: int,
    outcome: str,
    micros: int | None,
    taken_at_us: int | None,
    iteration: int | None,
    period_us: int,
) -> tuple[Any, ...]:
    """(OUTPUT, epoch, outcome, source_us, slot, taken_at_us, taken_at_slot,
    worker_iteration)"""
    return (
        EVENT_OUTPUT,
        int(epoch),
        str(outcome),
        micros,
        slot_index(micros, period_us),
        taken_at_us,
        slot_index(taken_at_us, period_us),
        iteration,
    )


def lifecycle_row(
    epoch: int,
    outcome: str,
    resulting_epoch: int,
    watermark_us: int | None,
    period_us: int,
) -> tuple[Any, ...]:
    """(LIFECYCLE, epoch, outcome, resulting_epoch, watermark_us,
    watermark_slot)"""
    return (
        EVENT_LIFECYCLE,
        int(epoch),
        str(outcome),
        int(resulting_epoch),
        watermark_us,
        slot_index(watermark_us, period_us),
    )


def subscription_row(
    epoch: int,
    outcome: str,
    ruling: int | None,
) -> tuple[Any, ...]:
    """(SUBSCRIPTION, epoch, outcome, ruling)"""
    return (
        EVENT_SUBSCRIPTION,
        int(epoch),
        str(outcome),
        None if ruling is None else int(ruling),
    )


__all__ = [
    "association_row",
    "discard_row",
    "lifecycle_row",
    "output_row",
    "stage_row",
    "subscription_row",
    "truth_row",
]
