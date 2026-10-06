"""Offline checks of immutable capture figures against retained evidence.

Shared capacity checks remain in status_contradiction. Latch and provable pass
count checks live here, at the reader boundary. Neither validation nor decoding
runs in recording callbacks; this is not authentication or flight certification.
"""

from navpy.modules.vision.sim.determinism_evidence import KIND_ROW, trace_line
from navpy.modules.vision.sim.determinism_events import EVENT_VIOLATION
from navpy.modules.vision.sim.determinism_row_log import (
    TraceStatus,
    is_unreadable_stage,
)
from navpy.modules.vision.sim.determinism_trace import TraceCapture
from navpy.modules.vision.sim.determinism_trace_decode import (
    EvidenceRefused,
    decode_trace,
)


def _violation(row: tuple) -> None:
    """Use the trace codec's exact types/layout, then require a violation."""
    try:
        decoded = decode_trace(trace_line(KIND_ROW, 0, row).encode("ascii"))
    except EvidenceRefused as error:
        raise EvidenceRefused(f"a violation latch is malformed: {error}") from None
    if decoded.rows[0][0] != EVENT_VIOLATION:
        raise EvidenceRefused("a violation latch names another event type")


def _latches(status: TraceStatus, rows: tuple[tuple, ...]) -> None:
    first, epochs = status.first_violation, status.violations_by_epoch
    if first is not None:
        _violation(first)
    for epoch, row in epochs.items():
        _violation(row)
        if type(epoch) is not int or epoch != row[1]:
            raise EvidenceRefused("a violation latch has an invalid epoch key")
    # A failed note_violation may have set the global latch before an interrupt
    # prevented setdefault. Draining/overflow alone cannot split these writes.
    if not status.failed and first != next(iter(epochs.values()), None):
        raise EvidenceRefused("the global and first epoch violation disagree")

    shown = any(map(is_unreadable_stage, rows))
    if shown and not status.payload_unreadable:
        raise EvidenceRefused("a held row's payload was unreadable, and unlatched")
    firsts: dict[int, tuple] = {}
    for row in rows:
        if row[0] != EVENT_VIOLATION:
            continue
        if first is None or row[1] not in epochs:
            raise EvidenceRefused("a held violation was never latched")
        firsts.setdefault(row[1], row)

    if status.drained or status.failed:
        return
    # Overflow preserves the earliest rows. Epochs represented by those rows
    # precede any epoch whose first violation was dropped after the buffer filled.
    if list(epochs)[:len(firsts)] != list(firsts):
        raise EvidenceRefused("epoch latch order differs from retained violations")
    if any(epochs[epoch] != row for epoch, row in firsts.items()):
        raise EvidenceRefused("a retained first violation differs from its latch")
    if not status.dropped and (
        status.payload_unreadable != shown
        or list(epochs.items()) != list(firsts.items())
    ):
        raise EvidenceRefused("the latches are not the retained rows' latches")


def validate_capture(capture: TraceCapture) -> None:
    """Raise EvidenceRefused when retained records disprove capture figures."""
    _latches(capture.status, capture.rows)
    commands = capture.commands
    # The current writer gives every store the journal's capacity. Entries do
    # not shrink, so below that shared capacity, all command-log drops were
    # pass samples counted before dropping. At capacity (including zero), the
    # drop kind is unknown. This checks integrity even for incomplete captures.
    if len(commands.entries) < capture.status.capacity:
        minimum = len(commands.passes) + commands.dropped
        if commands.iterations < minimum:
            raise EvidenceRefused(
                "the iteration count omits held or provably dropped pass samples"
            )
