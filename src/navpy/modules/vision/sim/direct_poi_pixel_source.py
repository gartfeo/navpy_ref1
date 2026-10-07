"""Simulator-only known-POI source for isolating pure-vision navigation.

WHY THE POSE COMES FROM SIM_STATE AND NOT FROM ATTITUDE
------------------------------------------------------
Synthesizing the POI pixel means rotating the POI vector into the body
frame, and that rotation needs a yaw. Taking it from the ATTITUDE message is
wrong here: ATTITUDE carries the vehicle's compass-derived ESTIMATE, and this
project forbids compass yaw in the final-approach command path precisely because its
bias is indistinguishable from a navigation error. A biased yaw rotates the
synthesized line of sight by that same bias, so the contamination survives into
the ray even though the law never reads yaw itself -- a 15 degree yaw shift
moves the pixel by hundreds of pixels (measured in ``scripts/sitl_truth_pose.py``).

SIM_STATE is the simulator's own truth: the attitude the aircraft HAS. Stock
SIM_STATE carries no timestamp, so ATTITUDE supplies the clock and
``PoseAssociator`` bounds the pairing on host receipt stamps. The navlink
fork adds a ``time_us`` source stamp (the FDM state time, on the same
autopilot clock as ATTITUDE ``time_boot_ms``); ``AAS_TRUTH_POSE_TIME_AXIS``
selects which clock interpolates and queries. ``source`` is the default: two
interleaved A/Bs (pre-merge and 2026-08-27 on merged dev, p=0.001) confirmed
it removes ~1.15 mdeg of receipt-jitter bearing noise. Stock firmware without
``time_us`` still degrades sticky to receipt stamps.

Two selection points here are newest-wins and therefore host-dependent: the
held clock (``hold``) and the publish slot (``publish``). Both are recorded by
``direct_pixel_trace`` and neither is changed by it; removing them is Landing 2
of the one-clock work.

Every trace row is appended while THIS class holds ``_lock`` -- the same lock
that serialises the slot transitions the rows describe -- so a row can never
be ordered against a slot transition it did not see. That is weaker than
"row order is decision order", and deliberately so: an OUTPUT row is a
SETTLEMENT row. Dispatch runs unlocked, so newer STAGE rows can legitimately
precede the OUTPUT row of a frame that was taken before them. The frame's
own take instant is preserved instead, in the watermark column
``dispatch_available`` captures with the take.

The worker's iteration number comes from the same lock's other side: the
command worker publishes it to the trace's ``CommandLoopLog`` before this
source is asked to dispatch, so every OUTPUT row names the pass it ran in.
An iteration is not an autopilot slot -- see ``determinism_command_log``.

SUBSCRIPTION rows bracket the subscriptions, one once ``start()`` opened them
and one before ``close()`` cancels them, both under ``_lock`` (D3).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from navpy.modules.vision.sim.direct_dispatch_metrics import (
    DirectPixelSourceMetrics,
    DirectPublishState,
)
from navpy.modules.vision.sim.direct_pixel_pipeline import (
    DIRECT_PIXEL_SOURCE_NAME,
    advance_for_pending,
    build_direct_pixel_pipeline,
)
from navpy.modules.vision.sim.direct_pixel_trace import (
    OUTPUT_EMPTY,
    OUTPUT_EXCEPTION,
    OUTPUT_STALE_EPOCH,
    dispatch_outcome,
)
from navpy.modules.vision.sim.pose_associator import AssociatedPose
from navpy.modules.vision.sim.sim_camera_ports import FrameSize


# GLOBAL_POSITION_INT is deliberately absent: PoseAssociator treats it as a
# truth event interchangeably with SIM_STATE, so subscribing it would let the
# pair complete on an estimated position with no truth sample involved.
DIRECT_PIXEL_POSE_MESSAGE_TYPES = ("ATTITUDE", "SIM_STATE")
# Stock SIM_STATE has no clock of its own, so a truth sample's time can only
# be BOUNDED by the interval it landed in, and only ATTITUDE closes an
# interval. (With the navlink ``time_us`` stamp the source axis positions
# samples exactly; the association flow itself is unchanged.)
DIRECT_PIXEL_CLOCK_MESSAGE_TYPE = "ATTITUDE"
# Each SIM_STATE also feeds the bracketing ``TruthPoseHistory``.
DIRECT_PIXEL_TRUTH_MESSAGE_TYPE = "SIM_STATE"


class DirectPoiPixelSource:
    """Convert one known geo POI into newest-only ideal camera pixels."""

    def __init__(
        self,
        vehicle: IVehicle,
        poi: Location,
        cadence: SchedulerCadence,
        *,
        aircraft_sequence: str,
        aircraft_degrees: bool,
        dispatch: Callable[[DetectedObject], bool],
        frame_size: FrameSize = FrameSize(2560, 1440),
        wall_now_s: Callable[[], float] = time.time,
    ) -> None:
        self._dispatch = dispatch
        self._lock = threading.Lock()
        self._state = DirectPublishState()
        pipeline = build_direct_pixel_pipeline(
            vehicle,
            poi,
            cadence,
            aircraft_sequence=aircraft_sequence,
            aircraft_degrees=aircraft_degrees,
            frame_size=frame_size,
            source_now_s=self.source_now,
            wall_now_s=wall_now_s,
        )
        self._stream = pipeline.stream
        self._associator = pipeline.associator
        self._truth_history = pipeline.truth_history
        self._renderer = pipeline.renderer
        self._trace = pipeline.trace

    def start(self) -> None:
        self._stream.start(
            self._on_message,
            DIRECT_PIXEL_POSE_MESSAGE_TYPES,
            request_truth=True,
        )
        with self._lock:
            self._trace.subscription_opened(self._state.epoch)

    def activate(self) -> None:
        with self._lock:
            self._trace.leg_ended(self._state.activate())
            # No cruise-leg pair or half-done association crosses the change.
            self._associator.reset()
            self._truth_history.reset_pair()
            self._truth_history.reset_diagnostics()

    def dispatch_available(self) -> bool:
        trace = self._trace
        with self._lock:
            poi = self._state.take_latest()
            epoch = self._state.epoch
            taken_at_us = trace.watermark_us()
            if poi is None:
                trace.output(epoch, OUTPUT_EMPTY, None, taken_at_us)
                return False
        with self._lock:
            # A SECOND acquisition on purpose: the lock was released, so a leg
            # boundary can have run since the take -- and it emptied a slot
            # that no longer held this frame. It belongs to the leg that ended.
            if not (self._state.active and self._state.is_current(epoch)):
                trace.output(epoch, OUTPUT_STALE_EPOCH, poi, taken_at_us)
                return False
        dispatched, outcome = False, OUTPUT_EXCEPTION
        try:
            dispatched = bool(self._dispatch(poi))
            outcome = dispatch_outcome(dispatched)
        finally:
            # A raising consumer still EMPTIED the slot, and the worker
            # CATCHES the exception (navigation_command_worker.py:136-140), so
            # without this the frame leaves no row and no tally at all.
            with self._lock:
                self._state.settle(epoch, outcome)
                trace.output(epoch, outcome, poi, taken_at_us)
        return dispatched

    @property
    def metrics(self) -> DirectPixelSourceMetrics:
        with self._lock:
            return self._state.counters.snapshot()

    @property
    def determinism_trace(self) -> DeterminismTrace | None:
        """The decision trace for the owner to drain AFTER the scored leg.

        None when tracing is off; nothing in the command path reads it.
        """
        return self._trace.trace

    @property
    def advance_diagnostics(self) -> dict[str, float]:
        with self._lock:
            return self._truth_history.diagnostics

    def source_now(self) -> float:
        with self._lock:
            return self._state.source_now_s or 0.0

    def close(self) -> int:
        """End the leg and return its epoch, whatever the trace kept."""
        with self._lock:
            boundary = self._state.close()
            self._trace.leg_ended(boundary)
            self._trace.subscription_closed(boundary.ending_epoch)
        self._stream.close()
        return boundary.ending_epoch

    def _on_message(self, message: object) -> None:
        try:
            message_type = str(message.get_type())
        except Exception:
            return
        if message_type == DIRECT_PIXEL_TRUTH_MESSAGE_TYPE:
            self._on_truth(message)
            return
        if message_type != DIRECT_PIXEL_CLOCK_MESSAGE_TYPE:
            # HELD, not paired: pairing on arrival would stamp the sample with
            # the EARLIER ATTITUDE. The FIRST ATTITUDE after it closes it.
            return
        self._on_clock()

    def _on_truth(self, message: object) -> None:
        trace = self._trace
        with self._lock:
            # Locked with the record itself: an event noted on either side
            # of activate()'s reset would pair across the cadence change.
            self._associator.note_event(DIRECT_PIXEL_TRUTH_MESSAGE_TYPE)
            recorded = self._truth_history.note_current()
            pending = self._state.pending
            epoch = self._state.epoch
            trace.truth(epoch, recorded, message)
            if not recorded:
                # No readable pose: the pair still ENDS before the pending
                # clock. Keep it; the next real sample closes the bracket.
                return
            # Queried under the lock the axis fallback flips in.
            advanced = advance_for_pending(self._truth_history, pending)
            if pending is not None and advanced is None:
                # Unbracketable, and the history only moves away from this
                # clock. Drop it: one raw stale bearing between interpolated
                # neighbours differences into a rate impulse.
                trace.unbracketable(epoch, pending)
                self._state.release(pending)
                return
        if pending is not None and advanced is not None:
            self._render(pending, advanced, epoch)

    def _on_clock(self) -> None:
        trace = self._trace
        with self._lock:
            # Only the event NOTE is locked: pairing reads vehicle state, and
            # a slow read would stall every truth sample behind the lock.
            self._associator.note_event(DIRECT_PIXEL_CLOCK_MESSAGE_TYPE)
            epoch = self._state.epoch
        associated = self._associator.associate()
        if associated is None:
            with self._lock:
                if self._state.active:
                    self._state.counters.association_refusals += 1
                trace.association_refused(epoch)
            return
        self._associator.commit(associated)
        # HOLD it: rendering waits for the next truth sample, so this clock is
        # BRACKETED and its pose interpolates instead of being guessed forward
        # (cost: one truth period of near-constant latency).
        with self._lock:
            if not self._state.is_current(epoch):
                # activate() ran after this pair was captured: it must not
                # seed the scored leg's slot.
                trace.association_fenced(epoch, associated)
                return
            displaced = self._state.hold(associated)
            trace.association_committed(epoch, associated, displaced)

    def _render(
        self,
        associated: AssociatedPose,
        pose: tuple[Location, Attitude],
        epoch: int,
    ) -> None:
        """Project outside the lock, then publish under it if still ours."""
        projected = self._renderer.render(associated, pose)
        skew_ms = 1000.0 * (
            associated.attitude_receipt_s - associated.truth_receipt_s
        )
        with self._lock:
            outcome, displaced = self._state.stage(
                associated, projected, epoch, skew_ms
            )
            self._trace.stage(
                epoch, outcome, associated, projected, displaced
            )


__all__ = [
    "DIRECT_PIXEL_CLOCK_MESSAGE_TYPE",
    "DIRECT_PIXEL_POSE_MESSAGE_TYPES",
    "DIRECT_PIXEL_SOURCE_NAME",
    "DIRECT_PIXEL_TRUTH_MESSAGE_TYPE",
    "DirectPixelSourceMetrics",
    "DirectPoiPixelSource",
]
