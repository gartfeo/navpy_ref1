"""Wire the direct-pixel source's collaborators and their shared bounds.

Split from ``direct_poi_pixel_source`` for the same reason
``direct_pixel_render`` was: that module owns the message plumbing, the lock
and the publish slot, and it should not also own how the association gate, the
truth history, the renderer and the decision trace are configured.

The bounds are shared, which is why they are derived in ONE place: the pose
stream rate sets a two-period association skew, that skew is the wall bound
the associator gates receipts with, and the same scheduler period is the
source-axis span the truth pair may not exceed. The trace's slot grid comes
from the scheduler rate the stream resolved, the one read all of these share.
Deriving any of them separately would let them disagree. So the trace is
built after the stream, and the stream reaches it late, at ``start()``, to
count what the source's callback raises.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.vehicle.pose_streams import (
    pose_frame_association_max_skew_s,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.sim.direct_pixel_render import DirectPixelRenderer
from navpy.modules.vision.sim.direct_pixel_trace import (
    DirectPixelTrace,
    DisabledPixelTrace,
    build_direct_pixel_trace,
)
from navpy.modules.vision.sim.pose_associator import PoseAssociator
from navpy.modules.vision.sim.pose_stream_link import PoseStreamLink
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.sim.truth_pose_history import TruthPoseHistory
from navpy.modules.vision.sim.truth_pose_time_axis import (
    pair_span_s,
    render_clock_s,
    resolve_truth_pose_time_axis,
)


DIRECT_PIXEL_SOURCE_NAME = "direct_poi_pixel"


@dataclass(frozen=True)
class DirectPixelPipeline:
    """The collaborators one direct-pixel source drives, already bounded."""

    stream: PoseStreamLink
    associator: PoseAssociator
    truth_history: TruthPoseHistory
    renderer: DirectPixelRenderer
    trace: DirectPixelTrace | DisabledPixelTrace


def build_direct_pixel_pipeline(
    vehicle: IVehicle,
    poi: Location,
    cadence: SchedulerCadence,
    *,
    aircraft_sequence: str,
    aircraft_degrees: bool,
    frame_size: FrameSize,
    source_now_s: Callable[[], float],
    wall_now_s: Callable[[], float] = time.time,
) -> DirectPixelPipeline:
    """Build every collaborator the source drives, as one set."""
    # ``trace`` is bound below; start() runs only after this returns.
    stream = PoseStreamLink(
        vehicle, guard=lambda on_message: trace.guard_callback(on_message)
    )
    scheduler_period_s = pose_frame_association_max_skew_s(stream.rate_hz)

    def max_receipt_skew_s() -> float:
        return cadence.wall_period_for_scheduler_period(scheduler_period_s)

    def max_pair_span_s() -> float:
        # The gap gate compares stamps on the axis in use, which the history
        # can change (sticky fallback), so it is read at call time.
        return pair_span_s(
            truth_history.axis_in_use,
            scheduler_period_s,
            max_receipt_skew_s,
        )

    truth_history = TruthPoseHistory(
        truth_pose=lambda: vehicle.simulator_truth_pose,
        max_span_s=max_pair_span_s,
        time_axis=resolve_truth_pose_time_axis(),
    )
    associator = PoseAssociator(
        attitude_sample=lambda: vehicle.attitude_sample,
        truth_pose=lambda: vehicle.simulator_truth_pose,
        air_speed=lambda: vehicle.air_speed,
        maximum_receipt_skew_s=max_receipt_skew_s,
        require_event_pair=True,
    )
    renderer = DirectPixelRenderer(
        poi,
        source_name=DIRECT_PIXEL_SOURCE_NAME,
        aircraft_sequence=aircraft_sequence,
        aircraft_degrees=aircraft_degrees,
        frame_size=frame_size,
        source_now_s=source_now_s,
        wall_now_s=wall_now_s,
    )
    # The rate the stream ALREADY resolved: the trace's slot grid has to be
    # the grid the association window was sized on, and a second parameter
    # read can answer differently.
    trace = build_direct_pixel_trace(stream.scheduler_rate_hz)
    return DirectPixelPipeline(
        stream=stream,
        associator=associator,
        truth_history=truth_history,
        renderer=renderer,
        trace=trace,
    )


def advance_for_pending(
    truth_history: TruthPoseHistory,
    pending: object | None,
) -> tuple[Location, Attitude] | None:
    """The interpolated truth pose for a held clock, or None if unbracketable.

    The query clock depends on the axis in use, and the history can change
    that axis (sticky fallback), so both are read together at call time.
    """
    if pending is None:
        return None
    return truth_history.pose_at(
        render_clock_s(truth_history.axis_in_use, pending)
    )


__all__ = [
    "DIRECT_PIXEL_SOURCE_NAME",
    "DirectPixelPipeline",
    "advance_for_pending",
    "build_direct_pixel_pipeline",
]
