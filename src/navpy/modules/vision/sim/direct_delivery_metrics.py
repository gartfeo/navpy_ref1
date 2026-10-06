"""Publish slot and frame/delivery tallies for the direct-pixel source.

Both live here because both are the source's MUTABLE state, and the source
owns exactly one lock for all of it: every field below is read and written
under ``DirectPoiPixelSource._lock``. Keeping them in one holder is what
lets the source stay a message handler instead of also being a state bag.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from navpy.modules.vision.sim.determinism_events import (
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    OUTPUT_DELIVERED,
    OUTPUT_EXCEPTION,
    OUTPUT_REJECTED,
    STAGE_FENCED,
    STAGE_INACTIVE,
    STAGE_PROJECTION_FAILED,
    STAGE_STAGED,
)


@dataclass(frozen=True)
class DirectPixelSourceMetrics:
    projected_frames: int
    projection_failures: int
    delivered_frames: int
    delivery_rejections: int
    delivery_exceptions: int
    advanced_truth_frames: int
    association_refusals: int
    pose_skew_ms_mean: float
    pose_skew_ms_max: float
    pose_skew_delta_ms_std: float


class PoseSkewStatistics:
    """Running attitude-vs-truth receipt skew: mean, peak, and delta spread.

    Its own object because it is a STATISTIC, not a tally -- six fields that
    only ever move together and are read only by the snapshot. Beside the
    frame counts they made one class own two different kinds of number.
    """

    def __init__(self) -> None:
        self.count = 0
        self.sum_ms = 0.0
        self.max_ms = 0.0
        self.previous_ms: float | None = None
        self.delta_sum_ms = 0.0
        self.delta_sumsq_ms = 0.0

    def record(self, skew_ms: float) -> None:
        self.count += 1
        self.sum_ms += skew_ms
        self.max_ms = max(self.max_ms, skew_ms)
        if self.previous_ms is not None:
            delta = skew_ms - self.previous_ms
            self.delta_sum_ms += delta
            self.delta_sumsq_ms += delta * delta
        self.previous_ms = skew_ms

    @property
    def mean_ms(self) -> float:
        return self.sum_ms / self.count if self.count else 0.0

    @property
    def delta_std_ms(self) -> float:
        count = self.count - 1
        if count < 1:
            return 0.0
        mean = self.delta_sum_ms / count
        variance = self.delta_sumsq_ms / count - mean * mean
        return math.sqrt(max(variance, 0.0))


class DirectDeliveryCounters:
    """Mutable frame/delivery tallies, always touched under the source lock."""

    def __init__(self) -> None:
        self.projected_frames = 0
        self.projection_failures = 0
        self.delivered_frames = 0
        self.delivery_rejections = 0
        # The consumer RAISED. Kept apart from a refusal because the worker
        # CATCHES it (navigation_command_worker.py:136-140): a crashing consumer
        # would otherwise be indistinguishable from a quiet one.
        self.delivery_exceptions = 0
        self.advanced_truth_frames = 0
        # An ATTITUDE that could not close a pair. A few are normal (the pair
        # needs one event of each kind); a run where this climbs while
        # projected_frames stays at zero is a STARVED source, not a quiet one,
        # and without the tally the two look identical from the verdict.
        self.association_refusals = 0
        self.skew = PoseSkewStatistics()

    def record_outcome(self, outcome: str) -> None:
        """Tally one SETTLED dispatch, named by the row's own outcome.

        One vocabulary rather than two, so a counter and the row that explains
        it can never disagree about what happened to a frame. Outcomes that
        consumed no frame (empty slot, stale epoch) tally nothing.
        """
        if outcome == OUTPUT_DELIVERED:
            self.delivered_frames += 1
        elif outcome == OUTPUT_REJECTED:
            self.delivery_rejections += 1
        elif outcome == OUTPUT_EXCEPTION:
            self.delivery_exceptions += 1

    def record_skew(self, skew_ms: float) -> None:
        # Every published frame is interpolated inside a bracketing truth
        # pair -- an unbracketed clock is dropped, never rendered -- so this
        # now tracks projected_frames. It stays recorded because the two
        # diverging would mean the drop rule had been broken.
        self.advanced_truth_frames += 1
        self.skew.record(skew_ms)

    def snapshot(self) -> DirectPixelSourceMetrics:
        return DirectPixelSourceMetrics(
            self.projected_frames,
            self.projection_failures,
            self.delivered_frames,
            self.delivery_rejections,
            self.delivery_exceptions,
            self.advanced_truth_frames,
            self.association_refusals,
            self.skew.mean_ms,
            self.skew.max_ms,
            self.skew.delta_std_ms,
        )


@dataclass(frozen=True)
class LegBoundary:
    """One activate() or close(): both epochs, read inside the transition,
    and the (pending, published) samples it threw away."""

    outcome: str
    ending_epoch: int
    resulting_epoch: int
    pending: Any | None
    published: Any | None


class DirectPublishState:
    """Newest-only publish slot, its pending clock, and the activation epoch.

    ``epoch`` exists because rendering runs OUTSIDE the source lock: a frame
    captured before ``activate()`` must not publish after it, or a cruise-leg
    frame becomes the scored leg's first frame. Every mutator here assumes the
    owner already holds the lock.
    """

    def __init__(self) -> None:
        self.active = False
        self.latest: Any | None = None
        self.pending: Any | None = None
        self.source_now_s: float | None = None
        self.epoch = 0
        self.counters = DirectDeliveryCounters()

    def activate(self) -> LegBoundary:
        """Start a scored leg: publish nothing captured before this instant.

        Returns the boundary, with the (pending, published) samples it threw
        away, so the caller can account for them; without that they would
        simply vanish.

        The tallies restart with the leg. They are read to judge ONE scored
        leg, and the advance diagnostics beside them already reset here, so
        carrying frame counts and a cross-leg skew delta over would make the
        two halves of the same verdict describe different spans.
        """
        ending, discarded = self.epoch, self._empty_slots()
        self.active = True
        self.source_now_s = None
        self.counters = DirectDeliveryCounters()
        self.epoch += 1
        return LegBoundary(LIFECYCLE_ACTIVATED, ending, self.epoch, *discarded)

    def close(self) -> LegBoundary:
        """Stop the leg, and fence anything already in flight out of it."""
        ending, discarded = self.epoch, self._empty_slots()
        self.active = False
        self.epoch += 1
        return LegBoundary(LIFECYCLE_CLOSED, ending, self.epoch, *discarded)

    def _empty_slots(self) -> tuple[Any | None, Any | None]:
        pending, self.pending = self.pending, None
        latest, self.latest = self.latest, None
        return pending, latest

    def hold(self, association: Any) -> Any | None:
        """Hold a clock awaiting its truth bracket; return what it DISPLACED.

        Newest-wins: a clock still waiting is dropped when a newer one lands.
        Returning the casualty rather than dropping it silently is what lets
        the caller account for a frame that never existed.
        """
        displaced, self.pending = self.pending, association
        return displaced

    def publish(self, frame: Any) -> Any | None:
        """Fill the publish slot; return the frame this one DISPLACED.

        A displaced frame was projected and never delivered, and no delivery
        rejection is recorded for it -- which is exactly the archived
        projected-minus-delivered gap.
        """
        displaced, self.latest = self.latest, frame
        return displaced

    def release(self, rendered: Any) -> None:
        """Empty the pending slot only if it still holds ``rendered``.

        Rendering runs UNLOCKED, so a newer ATTITUDE can store its own clock
        meanwhile; clearing blindly would delete a clock nothing has used.
        """
        if self.pending is rendered:
            self.pending = None

    def stage(
        self,
        associated: Any,
        projected: Any,
        epoch: int,
        skew_ms: float,
    ) -> tuple[str, Any | None]:
        """Rule on one projected frame; return (outcome, displaced frame).

        The caller renders UNLOCKED and holds the lock across this call. Every
        other slot transition already lives here, and this was the one still
        written out in the source -- the one place the recorded ledger could
        drift from the slot it claims to describe.
        """
        if not self.is_current(epoch):
            # activate()/close() ran mid-render: the frame belongs to the leg
            # that ended, and must not seed this one.
            return STAGE_FENCED, None
        self.release(associated)
        if projected is None:
            if self.active:
                self.counters.projection_failures += 1
            return STAGE_PROJECTION_FAILED, None
        self.source_now_s = associated.attitude_timestamp_s
        if not self.active:
            return STAGE_INACTIVE, None
        displaced = self.publish(projected)
        self.counters.projected_frames += 1
        self.counters.record_skew(skew_ms)
        return STAGE_STAGED, displaced

    def settle(self, epoch: int, outcome: str) -> None:
        """Tally a dispatch that has finished, if the leg still owns it."""
        if self.is_current(epoch):
            self.counters.record_outcome(outcome)

    def take_latest(self) -> Any | None:
        """Hand out the newest frame, leaving the slot empty."""
        frame = self.latest
        self.latest = None
        return frame

    def is_current(self, epoch: int) -> bool:
        """False once activate() has run since ``epoch`` was captured."""
        return self.epoch == epoch


__all__ = [
    "DirectDeliveryCounters",
    "PoseSkewStatistics",
    "DirectPixelSourceMetrics",
    "DirectPublishState",
    "LegBoundary",
]
