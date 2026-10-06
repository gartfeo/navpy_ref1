"""D8's labels: where each record of a case sits on the autopilot's slot grid.

The table is D8's, in the LANDING2 step-1 plan.
Every row and every pass gets its labels as (name, value) pairs, a value
being a slot, UNKNOWN when there is nothing to place it by, or NOT_APPLICABLE
for the frame of an OUTPUT that took none. A committed or fenced association
is placed by its own stamp, and a refused one, which carries none, by the
ledger's stamp for the ruling it names: the ATTITUDE it acted on. A TRUTH, a
LIFECYCLE, and the second slot of a STAGE, a DECIMATE or an OUTPUT are placed
by the watermark, a lower bound on the ATTITUDE time known then. A pass is
placed by its admitted cutoff's stamp, a lower bound too, and an OUTPUT's
pass through its worker iteration. A VIOLATION is labelled by its own detail;
a SUBSCRIPTION has no label, since it bounds coverage.

Labels are meant for an ELIGIBLE case, where every ruling a row names is in
the ledger with its stamp. In a case with a discontinuity or a stampless
admission the ledger's stamps cannot place anything, so there every clock
label is UNKNOWN. NOT_APPLICABLE stays, since it places nothing, and so does
a VIOLATION's detail, which is its own and no clock label. An
INSPECTABLE case is labelled too, and its labels are only as good as its
evidence.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_REFUSED,
    ASSOCIATION_RULING,
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    EVENT_VIOLATION,
    OUTPUT_EMPTY,
    OUTPUT_WORKER_ITERATION,
    SLOT_PAIRS,
    VIOLATION_DETAIL,
)
from navpy.modules.vision.sim.determinism_slots import slot_index
from navpy.modules.vision.sim.determinism_trace import TraceCapture

UNKNOWN = "unknown"
NOT_APPLICABLE = "n.a."
# A VIOLATION's one label: its own detail, never a slot.
_DETAIL = "detail"
# A ledger stamp is whole milliseconds of boot time; a slot, microseconds.
_MICROS_PER_MS = 1_000
# The name of each slot a layout carries, in ``SLOT_PAIRS`` order.
_SLOT_NAMES: Mapping[str, tuple[str, ...]] = MappingProxyType({
    EVENT_ASSOCIATION: ("attitude_slot",),
    EVENT_TRUTH: ("watermark_slot",),
    EVENT_STAGE: ("frame_slot", "watermark_slot"),
    EVENT_DECIMATE: ("victim_slot", "watermark_slot"),
    EVENT_OUTPUT: ("frame_slot", "taken_slot"),
    EVENT_LIFECYCLE: ("watermark_slot",),
    EVENT_SUBSCRIPTION: (),
    EVENT_VIOLATION: (),
})

Labels = tuple[tuple[str, Any], ...]


@dataclass(frozen=True)
class CaseLabels:
    """Each row's labels and each pass's, in capture order."""

    rows: tuple[Labels, ...]
    passes: tuple[Labels, ...]


def case_labels(capture: TraceCapture) -> CaseLabels:
    """Every record's labels, by D8's table (module docstring)."""
    stamps = {
        ruling: stamp
        for ruling, stamp, accepted, _ in capture.ledger.entries
        if accepted
    }

    def ruling_slot(ruling: Any) -> Any:
        stamp = stamps.get(ruling)
        if stamp is None:
            return UNKNOWN
        return _known(slot_index(stamp * _MICROS_PER_MS, capture.period_us))

    passes = tuple(
        (("admitted_slot", ruling_slot(admitted)),)
        for _, _, admitted in capture.commands.passes
    )
    pass_slots = {
        sample[0]: labels[0][1]
        for sample, labels in zip(capture.commands.passes, passes)
    }
    rows = tuple(_row_labels(row, ruling_slot, pass_slots) for row in capture.rows)
    if capture.ledger.discontinuous or capture.ledger.stampless:
        return CaseLabels(_unknown(rows), _unknown(passes))
    return CaseLabels(rows, passes)


def _known(slot: Any) -> Any:
    return UNKNOWN if slot is None else slot


def _row_labels(
    row: tuple,
    ruling_slot: Callable[[Any], Any],
    pass_slots: Mapping[int, Any],
) -> Labels:
    event = row[0]
    labels = [
        (name, _known(row[slot]))
        for name, (_, slot) in zip(_SLOT_NAMES[event], SLOT_PAIRS[event])
    ]
    if event == EVENT_ASSOCIATION and row[2] == ASSOCIATION_REFUSED:
        labels = [("attitude_slot", ruling_slot(row[ASSOCIATION_RULING]))]
    elif event == EVENT_OUTPUT:
        if row[2] == OUTPUT_EMPTY:
            labels[0] = ("frame_slot", NOT_APPLICABLE)
        labels.append((
            "pass_slot", pass_slots.get(row[OUTPUT_WORKER_ITERATION], UNKNOWN)
        ))
    elif event == EVENT_VIOLATION:
        labels = [(_DETAIL, row[VIOLATION_DETAIL])]
    return tuple(labels)


def _unknown(records: tuple[Labels, ...]) -> tuple[Labels, ...]:
    """Every clock label UNKNOWN. NOT_APPLICABLE places nothing, and a
    VIOLATION's detail is no clock label: both stay."""
    return tuple(
        tuple(
            (name, value if value == NOT_APPLICABLE or name == _DETAIL else UNKNOWN)
            for name, value in record
        )
        for record in records
    )


__all__ = ["NOT_APPLICABLE", "UNKNOWN", "CaseLabels", "case_labels"]
