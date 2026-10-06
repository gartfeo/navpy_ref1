"""Names the direct-pixel source's decision points for the determinism trace.

``DeterminismTrace`` knows how to key and store a row. It does not know that
this source holds a clock pending a truth bracket, or that its publish slot is
newest-wins. That vocabulary lives here, so the source reads as one line per
decision instead of a block of row plumbing, and so the ORDER two rows must be
written in (association before the discard it caused, so the watermark has
already advanced) is decided once rather than at every call site.

When tracing is off the source holds a no-op recorder rather than ``None``.
That keeps the message path free of ``if trace is not None`` at every site;
the real cost decision is made once, at construction, by the module gate.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_command_log import (
    NULL_COMMAND_LOOP,
    NullCommandLoop,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    ASSOCIATION_REFUSED,
    DISCARD_PENDING_LEG_ENDED,
    DISCARD_PENDING_OVERWRITTEN,
    DISCARD_PUBLISH_LEG_ENDED,
    DISCARD_PUBLISH_OVERWRITTEN,
    DISCARD_UNBRACKETABLE,
    OUTPUT_DELIVERED,
    OUTPUT_EMPTY,
    OUTPUT_EXCEPTION,
    OUTPUT_REJECTED,
    OUTPUT_STALE_EPOCH,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
    TRUTH_RECORDED,
    TRUTH_UNREADABLE,
)
from navpy.modules.vision.sim.determinism_slots import (
    scheduler_slot_period_us,
)
from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    build_trace,
)
from navpy.modules.vision.sim.direct_delivery_metrics import LegBoundary


class DisabledPixelTrace:
    """The off switch, as an object: every decision costs one no-op call."""

    __slots__ = ()

    @property
    def trace(self) -> DeterminismTrace | None:
        return None

    def watermark_us(self) -> int | None:
        return None

    def leg_ended(self, boundary: LegBoundary) -> None: ...

    def truth(self, epoch: int, recorded: bool, message: Any = None) -> None: ...

    def unbracketable(self, epoch: int, pending: Any) -> None: ...

    def association_refused(self, epoch: int) -> None: ...

    def association_fenced(self, epoch: int, associated: Any) -> None: ...

    def association_committed(
        self, epoch: int, associated: Any, displaced: Any
    ) -> None: ...

    def stage(
        self,
        epoch: int,
        outcome: str,
        associated: Any,
        frame: Any,
        displaced: Any,
    ) -> None: ...

    def output(
        self,
        epoch: int,
        outcome: str,
        frame: Any,
        taken_at_us: int | None,
    ) -> None: ...

    def subscription_opened(self, epoch: int) -> None: ...

    def subscription_closed(self, epoch: int) -> None: ...

    def guard_callback(
        self, callback: Callable[[Any], None]
    ) -> Callable[[Any], None]:
        """The callback itself: registered exactly as with no trace at all."""
        return callback


class DirectPixelTrace:
    """One method per place this source decides what a slot will see."""

    __slots__ = ("_trace",)

    def __init__(self, trace: DeterminismTrace) -> None:
        self._trace = trace

    @property
    def trace(self) -> DeterminismTrace | None:
        return self._trace

    def watermark_us(self) -> int | None:
        """The input watermark NOW, for a caller about to act unlocked."""
        return self._trace.watermark_us()

    def leg_ended(self, boundary: LegBoundary) -> None:
        """The boundary, then what it threw away, all at the ENDING epoch.

        W5: the engagement boundary has to be visible in the row stream, not
        inferred from an epoch that changed between two rows. So its row is
        written even when both slots were empty, and FIRST, so a reader meets
        the boundary before what it cost.
        """
        epoch = boundary.ending_epoch
        self._trace.record_lifecycle(
            epoch=epoch,
            outcome=boundary.outcome,
            resulting_epoch=boundary.resulting_epoch,
        )
        for victim, reason in (
            (boundary.pending, DISCARD_PENDING_LEG_ENDED),
            (boundary.published, DISCARD_PUBLISH_LEG_ENDED),
        ):
            if victim is not None:
                self._trace.record_discard(
                    epoch=epoch, reason=reason, victim=victim
                )

    def truth(self, epoch: int, recorded: bool, message: Any = None) -> None:
        self._trace.record_truth(
            epoch=epoch,
            outcome=TRUTH_RECORDED if recorded else TRUTH_UNREADABLE,
            message=message,
        )

    def unbracketable(self, epoch: int, pending: Any) -> None:
        """A held clock the truth history has moved past: dropped, not lost to
        timing. Kept distinct from the two overwrites for exactly that reason."""
        self._trace.record_discard(
            epoch=epoch, reason=DISCARD_UNBRACKETABLE, victim=pending
        )

    def association_refused(self, epoch: int) -> None:
        self._trace.record_association(
            epoch=epoch, outcome=ASSOCIATION_REFUSED
        )

    def association_fenced(self, epoch: int, associated: Any) -> None:
        self._trace.record_association(
            epoch=epoch,
            outcome=ASSOCIATION_FENCED,
            source_s=associated.attitude_timestamp_s,
        )

    def association_committed(
        self, epoch: int, associated: Any, displaced: Any
    ) -> None:
        """Association FIRST, so the watermark has already advanced when the
        discard is attributed: the older clock was displaced BY this newer
        one, which is the whole newest-wins defect."""
        self._trace.record_association(
            epoch=epoch,
            outcome=ASSOCIATION_COMMITTED,
            source_s=associated.attitude_timestamp_s,
        )
        if displaced is not None:
            self._trace.record_discard(
                epoch=epoch,
                reason=DISCARD_PENDING_OVERWRITTEN,
                victim=displaced,
            )

    def stage(
        self,
        epoch: int,
        outcome: str,
        associated: Any,
        frame: Any,
        displaced: Any,
    ) -> None:
        """One projected frame ruled on -- staged, fenced, failed or inactive.

        A frame it DISPLACED is the measured gap: projected, never delivered,
        no rejection recorded. That row goes first, so the frame that caused
        the drop has already advanced the watermark it is measured against.
        """
        if displaced is not None:
            self._trace.record_discard(
                epoch=epoch,
                reason=DISCARD_PUBLISH_OVERWRITTEN,
                victim=displaced,
            )
        self._trace.record_stage(
            epoch=epoch,
            outcome=outcome,
            sample=associated,
            frame=frame,
        )

    def output(
        self,
        epoch: int,
        outcome: str,
        frame: Any,
        taken_at_us: int | None,
    ) -> None:
        self._trace.record_output(
            epoch=epoch,
            outcome=outcome,
            frame=frame,
            taken_at_us=taken_at_us,
        )

    def subscription_opened(self, epoch: int) -> None:
        """Open: every ATTITUDE admitted after this row's ruling is
        dispatched to the callback (D3)."""
        self._trace.record_subscription(
            epoch=epoch, outcome=SUBSCRIPTION_OPENED
        )

    def subscription_closed(self, epoch: int) -> None:
        """About to close: every ATTITUDE admitted before this row's ruling
        was dispatched to the callback, on the bus's one reader thread."""
        self._trace.record_subscription(
            epoch=epoch, outcome=SUBSCRIPTION_CLOSED
        )

    def guard_callback(
        self, callback: Callable[[Any], None]
    ) -> Callable[[Any], None]:
        """The source's message callback, its faults counted by the trace."""
        return self._trace.guard_callback(callback)


def command_loop_observer(
    trace: DeterminismTrace | None,
) -> PassObserver | NullCommandLoop:
    """The worker-side recorder for this trace, or the no-op when off.

    Not a forwarder through the trace, whose rows are keyed by slot: each
    pass takes the ATTITUDE ledger's lock for its cutoff, releases it, then
    takes the command log's, and a command takes the command log's alone
    (``PassObserver``, D3).
    """
    if trace is None:
        return NULL_COMMAND_LOOP
    return PassObserver(trace.ledger, trace.command_log)


def delivery_outcome(delivered: bool) -> str:
    """A refusing consumer and an empty slot are different failures."""
    return OUTPUT_DELIVERED if delivered else OUTPUT_REJECTED


def build_direct_pixel_trace(
    scheduler_rate_hz: float,
) -> DirectPixelTrace | DisabledPixelTrace:
    """A recorder on the given scheduler grid, or the off switch.

    Takes the rate rather than the vehicle: reading SCHED_LOOP_RATE a second
    time costs a live-link round trip even with tracing OFF, and a second
    answer would put the slot grid and the association window on grids that
    disagree.
    """
    trace = build_trace(scheduler_slot_period_us(scheduler_rate_hz))
    return DisabledPixelTrace() if trace is None else DirectPixelTrace(trace)


__all__ = [
    "OUTPUT_EMPTY",
    "OUTPUT_EXCEPTION",
    "OUTPUT_STALE_EPOCH",
    "DirectPixelTrace",
    "command_loop_observer",
    "DisabledPixelTrace",
    "build_direct_pixel_trace",
    "delivery_outcome",
]
