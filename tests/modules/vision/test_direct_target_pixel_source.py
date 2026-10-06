from __future__ import annotations

import ast
import builtins
import contextlib
import gc
import inspect
import itertools
import textwrap
import weakref
import threading
from types import (
    BuiltinFunctionType,
    FunctionType,
    MethodType,
    ModuleType,
    SimpleNamespace,
)
from unittest.mock import Mock

import numpy as np
import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_SIM_STATE,
)

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.navigation_command_worker import (
    NavigationCommandWorker,
    NavigationCommandWorkerPorts,
)
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    TerminalFrameProjector,
    TerminalProjectionConfig,
)
from navpy.modules.vision.sim import determinism_command_log
from navpy.modules.vision.sim import determinism_events
from navpy.modules.vision.sim import determinism_journal
from navpy.modules.vision.sim import determinism_trace
from navpy.modules.vision.sim import direct_target_pixel_source as source_module
from navpy.modules.vision.sim import pose_stream_link
from navpy.modules.vision.sim.determinism_slots import (
    PAYLOAD_UNREADABLE,
    command_digest,
    frame_digest,
)
from navpy.modules.vision.sim.determinism_trace_summary import summarise
from navpy.modules.vision.sim.direct_delivery_metrics import (
    DirectPublishState,
    LegBoundary,
)
from navpy.modules.vision.sim.direct_pixel_trace import command_loop_observer
from navpy.modules.vision.sim.direct_target_pixel_source import (
    DIRECT_PIXEL_POSE_MESSAGE_TYPES,
    DIRECT_PIXEL_SOURCE_NAME,
    DirectTargetPixelSource,
)


CURRENT = Location(40.0, 44.0, 1000.0, is_absolute=True)
TARGET = Location(40.001, 44.002, 900.0, is_absolute=True)
# 50 Hz SCHED_LOOP_RATE -> 40 Hz pose stream -> two periods of pairing skew.
SKEW_BOUND_S = 0.05
POSE_MESSAGE_ID_BY_NAME = {
    "ATTITUDE": MAVLINK_MSG_ID_ATTITUDE,
    "GLOBAL_POSITION_INT": MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    "SIM_STATE": MAVLINK_MSG_ID_SIM_STATE,
}


def _vehicle(
    *,
    truth_attitude: Attitude,
    telemetry_attitude: Attitude,
    truth_location: Location = CURRENT,
    attitude_receipt_s: float = 100.0,
    truth_receipt_s: float = 100.0,
    time_boot_s: float = 12.5,
) -> Mock:
    vehicle = Mock()
    vehicle.get_param_or_default.return_value = 50.0
    vehicle.air_speed = 40.0
    vehicle.attitude_sample = SimpleNamespace(
        attitude=telemetry_attitude,
        time_boot_s=time_boot_s,
        receipt_time_s=attitude_receipt_s,
        body_rates_rad_s=(0.1, 0.2, 0.3),
    )
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=truth_location,
        attitude=truth_attitude,
        receipt_time_s=truth_receipt_s,
    )
    # Populated so a regression to the telemetry pose reader would still run
    # rather than fail on a missing double.
    vehicle.location.return_value = truth_location
    return vehicle


def _message(name: str) -> SimpleNamespace:
    return SimpleNamespace(get_type=lambda: name)


def _source(vehicle: Mock, delivered: list) -> DirectTargetPixelSource:
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: delivered.append(detection) or True,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    return source


def _flush_truth(vehicle: Mock, source: DirectTargetPixelSource) -> None:
    """Deliver the NEXT truth sample so a pending attitude clock renders.

    It lands one pose period after the ATTITUDE, which is what closes the
    bracket: the clock has to sit BETWEEN two truth samples, and a sample
    that arrived before it would leave the source extrapolating.
    """
    pose = vehicle.simulator_truth_pose
    closing_s = max(
        float(pose.receipt_time_s),
        float(vehicle.attitude_sample.receipt_time_s),
    ) + 0.025
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=pose.location,
        attitude=pose.attitude,
        receipt_time_s=closing_s,
    )
    source._on_message(_message("SIM_STATE"))


def _pixel_from(vehicle: Mock, message_types: tuple[str, ...]) -> tuple:
    delivered: list = []
    source = _source(vehicle, delivered)
    for name in message_types:
        source._on_message(_message(name))
    _flush_truth(vehicle, source)
    assert source.dispatch_available()
    return (delivered[-1].pixel.u_px, delivered[-1].pixel.v_px)


def test_known_geo_target_becomes_pixel_then_geo_free_terminal_frame() -> None:
    attitude = Attitude(-4.0, 123.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)

    assert source.dispatch_available()
    assert len(delivered) == 1
    detection = delivered[0]
    assert detection.pixel.source_name == DIRECT_PIXEL_SOURCE_NAME
    frame = TerminalFrameProjector(
        TerminalProjectionConfig("ZYX", True)
    ).project(detection.visual_detection(), 0)
    assert frame is not None
    assert frame.source_timestamp_s == 12.5
    assert not hasattr(frame, "target_location")
    assert not hasattr(frame, "camera_location")
    assert np.linalg.norm(frame.body_ray) == pytest.approx(1.0)
    assert source.metrics.projected_frames == 1
    assert source.metrics.projection_failures == 0
    assert source.metrics.delivered_frames == 1
    assert source.metrics.delivery_rejections == 0


def test_newest_pixel_replaces_older_without_dispatching_twice() -> None:
    attitude = Attitude(0.0, 0.0, 0.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        attitude_receipt_s=10.0,
        truth_receipt_s=10.0,
        time_boot_s=1.0,
    )
    timestamps: list[float] = []
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: timestamps.append(
            detection.pixel.source_timestamp_s
        ) or True,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    # Receipts advance one 25 ms pose period at a time so every clock is
    # bracketed; an unbracketable clock now drops instead of rendering.
    for timestamp in (1.0, 2.0):
        vehicle.simulator_truth_pose.receipt_time_s = 10.0 + 0.025 * timestamp
        source._on_message(_message("SIM_STATE"))
        vehicle.attitude_sample.time_boot_s = timestamp
        vehicle.attitude_sample.receipt_time_s = 10.0 + 0.025 * timestamp
        source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose.receipt_time_s = 10.075
    source._on_message(_message("SIM_STATE"))

    assert source.dispatch_available()
    assert not source.dispatch_available()
    assert timestamps == [2.0]
    assert source.metrics.projected_frames == 2
    assert source.metrics.delivered_frames == 1


def test_pixel_is_rendered_from_truth_yaw_not_compass_yaw() -> None:
    """A compass yaw bias must not move the synthesized pixel.

    ATTITUDE yaw is the vehicle's EKF/compass estimate and can hold a large
    bias. Rendering the target ray with it rotates the line of sight by that
    same bias, which is indistinguishable from a navigation error in the miss
    the law is scored on -- so the pixel must come from simulator truth.
    """
    truth_attitude = Attitude(-4.0, 100.0, 3.0)
    biased = _pixel_from(
        _vehicle(
            truth_attitude=truth_attitude,
            telemetry_attitude=Attitude(-4.0, 123.0, 3.0),
        ),
        ("SIM_STATE", "ATTITUDE"),
    )
    unbiased = _pixel_from(
        _vehicle(
            truth_attitude=truth_attitude,
            telemetry_attitude=truth_attitude,
        ),
        ("SIM_STATE", "ATTITUDE"),
    )

    assert biased == unbiased


def test_global_position_int_alone_cannot_complete_the_event_pair() -> None:
    """An estimated position must not stand in for a truth sample.

    ``PoseAssociator`` treats GLOBAL_POSITION_INT and SIM_STATE as the same
    kind of truth event, so subscribing to the estimate would let the pair
    complete with no truth sample involved at all.
    """
    assert "GLOBAL_POSITION_INT" not in DIRECT_PIXEL_POSE_MESSAGE_TYPES
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("ATTITUDE"))
    source._on_message(_message("GLOBAL_POSITION_INT"))

    assert not source.dispatch_available()
    assert delivered == []
    assert source.metrics.projected_frames == 0


def test_truth_sample_is_held_until_an_attitude_closes_its_interval() -> None:
    """SIM_STATE carries no clock, so it cannot pair on arrival.

    Associating it the moment it lands would stamp it with the EARLIER
    ATTITUDE and report a pairing skew of zero, leaving the skew gate unable
    to reject anything.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("ATTITUDE"))
    assert source.metrics.projected_frames == 0

    source._on_message(_message("SIM_STATE"))
    assert source.metrics.projected_frames == 0, "paired on arrival"

    vehicle.attitude_sample.time_boot_s = 12.6
    vehicle.attitude_sample.receipt_time_s = 100.02
    source._on_message(_message("ATTITUDE"))
    assert source.metrics.projected_frames == 0, "rendered before the bracket"

    _flush_truth(vehicle, source)

    assert source.metrics.projected_frames == 1
    assert source.dispatch_available()
    assert delivered[0].pixel.source_timestamp_s == 12.6


def test_only_the_first_attitude_after_a_truth_sample_closes_it() -> None:
    """A consumed truth sample must not be re-paired with a later ATTITUDE.

    The streams are not guaranteed to alternate. Leaving the sample available
    would let every later ATTITUDE measure itself against the same stale
    SIM_STATE and manufacture frames that no truth sample backs.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        truth_receipt_s=100.0,
        attitude_receipt_s=100.0125,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=attitude,
        receipt_time_s=100.025,
    )
    source._on_message(_message("SIM_STATE"))
    assert source.metrics.projected_frames == 1

    # First ATTITUDE after the 100.025 truth sample closes it...
    vehicle.attitude_sample.time_boot_s = 12.6
    vehicle.attitude_sample.receipt_time_s = 100.04
    source._on_message(_message("ATTITUDE"))
    # ...so a second ATTITUDE with no fresh truth in between must not
    # re-pair the consumed sample and hijack the pending clock.
    vehicle.attitude_sample.time_boot_s = 12.7
    vehicle.attitude_sample.receipt_time_s = 100.06
    source._on_message(_message("ATTITUDE"))

    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=attitude,
        receipt_time_s=100.05,
    )
    source._on_message(_message("SIM_STATE"))

    assert source.metrics.projected_frames == 2
    assert source.dispatch_available()
    assert delivered[-1].pixel.source_timestamp_s == 12.6


def test_last_truth_sample_before_an_attitude_wins() -> None:
    """When several SIM_STATE arrive first, the newest is nearest the clock."""
    telemetry = Attitude(-4.0, 123.0, 3.0)
    second_truth = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=Attitude(-4.0, 60.0, 3.0),
        telemetry_attitude=telemetry,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=second_truth,
        receipt_time_s=100.0,
    )
    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)

    assert source.dispatch_available()
    only_second = _pixel_from(
        _vehicle(
            truth_attitude=second_truth,
            telemetry_attitude=telemetry,
        ),
        ("SIM_STATE", "ATTITUDE"),
    )
    assert (
        delivered[-1].pixel.u_px,
        delivered[-1].pixel.v_px,
    ) == only_second


def test_attitude_stream_without_any_truth_sample_produces_no_frames() -> None:
    """A SIM_STATE stream that never starts must starve, not fall back."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    vehicle.simulator_truth_pose = None
    delivered: list = []
    source = _source(vehicle, delivered)

    for index in range(5):
        vehicle.attitude_sample.time_boot_s = 12.5 + index * 0.025
        vehicle.attitude_sample.receipt_time_s = 100.0 + index * 0.025
        source._on_message(_message("ATTITUDE"))

    assert not source.dispatch_available()
    assert delivered == []
    assert source.metrics.projected_frames == 0


@pytest.mark.parametrize(
    ("truth_receipt_s", "expected_frames"),
    [(100.0 - SKEW_BOUND_S, 1), (100.0 - SKEW_BOUND_S - 0.001, 0)],
)
def test_pairing_skew_is_rejected_past_the_interval_bound(
    truth_receipt_s: float,
    expected_frames: int,
) -> None:
    """The interval between the two streams is the bound, and it is enforced.

    At 40 Hz an unbounded pairing lets the aircraft rotate measurably between
    the truth sample and the clock that stamps it, which lands as aim error in
    the rendered ray.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        truth_receipt_s=truth_receipt_s,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    # The closing sample lands ON the attitude clock: with the truth sample a
    # full bound old, that is the only spacing that still keeps the pair
    # inside the association gate AND the clock inside the pair. Anything
    # later would be refused as an over-wide gap, which would hide whether
    # the SKEW gate -- the thing under test -- did its job.
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=attitude,
        receipt_time_s=100.0,
    )
    source._on_message(_message("SIM_STATE"))

    assert source.metrics.projected_frames == expected_frames


def test_start_requests_the_simulator_truth_stream() -> None:
    """SIM_STATE has no default Plane stream, so it must be asked for."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])

    source.start()

    requested = {
        call.kwargs.get("p1")
        for call in vehicle.send_command_long.call_args_list
    }
    subscribed = [call.args[0] for call in vehicle.on_message.call_args_list]
    assert MAVLINK_MSG_ID_SIM_STATE in requested
    assert subscribed == list(DIRECT_PIXEL_POSE_MESSAGE_TYPES)
    # Subscribed-implies-requested. A message this source waits on but never
    # asks for would leave the event pair permanently unsatisfied.
    assert {POSE_MESSAGE_ID_BY_NAME[name] for name in subscribed} <= requested


def test_pixel_renders_from_truth_interpolated_to_the_pairing_clock() -> None:
    """The attitude clock is BRACKETED by truth samples and interpolated.

    Rendering used to fire on the closing ATTITUDE and EXTRAPOLATE the held
    pair past its newest sample; under angular acceleration the residual
    landed as bearing noise once the law differentiated (measured
    2026-08-24). The render now waits for the next truth sample, so the
    clock sits INSIDE the pair: yaw 101 at 100.025 and yaw 102 at 100.05
    interpolate to 101.5 at the 100.0375 attitude receipt.
    """
    telemetry = Attitude(-4.0, 123.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=Attitude(-4.0, 100.0, 3.0),
        telemetry_attitude=telemetry,
        truth_receipt_s=100.0,
        attitude_receipt_s=100.0375,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 101.0, 3.0),
        receipt_time_s=100.025,
    )
    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 102.0, 3.0),
        receipt_time_s=100.05,
    )
    source._on_message(_message("SIM_STATE"))

    assert source.dispatch_available()
    zero_skew = _pixel_from(
        _vehicle(
            truth_attitude=Attitude(-4.0, 101.5, 3.0),
            telemetry_attitude=telemetry,
            truth_receipt_s=100.0375,
            attitude_receipt_s=100.0375,
        ),
        ("SIM_STATE", "ATTITUDE"),
    )
    assert (
        delivered[-1].pixel.u_px,
        delivered[-1].pixel.v_px,
    ) == pytest.approx(zero_skew)
    assert source.metrics.advanced_truth_frames == 1


def test_pose_skew_metrics_flow_from_associations() -> None:
    """The pairing skew is measured per frame and surfaced in the metrics."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        truth_receipt_s=100.0,
        attitude_receipt_s=100.04,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))

    vehicle.simulator_truth_pose.receipt_time_s = 100.05
    source._on_message(_message("SIM_STATE"))

    vehicle.attitude_sample.time_boot_s = 12.6
    vehicle.attitude_sample.receipt_time_s = 100.07
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose.receipt_time_s = 100.08
    source._on_message(_message("SIM_STATE"))

    metrics = source.metrics
    assert metrics.projected_frames == 2
    # Association skews are 40 ms then 20 ms; one delta, degenerate std 0.
    assert metrics.pose_skew_ms_mean == pytest.approx(30.0)
    assert metrics.pose_skew_ms_max == pytest.approx(40.0)
    assert metrics.pose_skew_delta_ms_std == pytest.approx(0.0)
    # Both renders had a bracketing pair, so both advanced.
    assert metrics.advanced_truth_frames == 2


def test_activation_scopes_advance_diagnostics_to_the_scored_leg() -> None:
    """Frames before activate() fly the cruise leg at another speedup.

    Their tallies must not pollute the story of the SCORED leg's advance
    shortfall -- the exact question the diagnostics exist to answer.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: True,
        wall_now_s=lambda: 100.0,
    )
    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    # Same-receipt flush drops the pair: the clock is refused as unpaired
    # and the frame is dropped rather than rendered from the raw pose.
    source._on_message(_message("SIM_STATE"))
    assert source.advance_diagnostics["unpaired"] == 1.0

    source.activate()

    assert source.advance_diagnostics["unpaired"] == 0.0


def test_activation_drops_the_cruise_leg_truth_pair() -> None:
    """A truth pair built before activate() must not seed a scored rate.

    The cruise leg runs at another speedup, so a rate spanning activation
    would read the cadence change as pose motion. After activate() the
    history starts empty: the first clock cannot be bracketed and is
    DROPPED -- publishing the raw held pose instead would hand the law one
    stale bearing between interpolated neighbors, and the law differences
    every consecutive frame, so the age error would return as an
    entry/exit rate impulse (review finding). The first published frame is
    the first BRACKETED one.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        truth_receipt_s=100.0,
        attitude_receipt_s=100.0625,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 101.0, 3.0),
        receipt_time_s=100.025,
    )
    source._on_message(_message("SIM_STATE"))

    source.activate()

    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 102.0, 3.0),
        receipt_time_s=100.05,
    )
    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    # Same-receipt flush: one post-activation sample cannot bracket the
    # clock, so the frame must drop -- never borrow the cruise-leg pair
    # and never render the raw held pose.
    source._on_message(_message("SIM_STATE"))

    assert not source.dispatch_available()
    assert source.advance_diagnostics["unpaired"] == 1.0
    assert source.metrics.projected_frames == 0

    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 102.5, 3.0),
        receipt_time_s=100.075,
    )
    source._on_message(_message("SIM_STATE"))
    vehicle.attitude_sample.receipt_time_s = 100.0875
    vehicle.attitude_sample.time_boot_s = 12.525
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 103.0, 3.0),
        receipt_time_s=100.1,
    )
    source._on_message(_message("SIM_STATE"))

    assert source.dispatch_available()
    assert source.metrics.advanced_truth_frames == 1


def test_wide_gap_clock_is_dropped_not_rendered_raw() -> None:
    """A clock whose truth pair is over-wide must drop, not render raw.

    pose_at refuses an over-wide gap; rendering the raw held pose instead
    would inject one variable-age bearing into the consecutive-frame
    derivative (review finding). The stream resumes on the next clock the
    history CAN bracket.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        truth_receipt_s=100.0,
        attitude_receipt_s=100.065,
    )
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 101.0, 3.0),
        receipt_time_s=100.06,
    )
    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 102.0, 3.0),
        receipt_time_s=100.13,
    )
    source._on_message(_message("SIM_STATE"))

    assert not source.dispatch_available()
    assert source.advance_diagnostics["gap_wide"] == 1.0
    assert source.metrics.projected_frames == 0

    vehicle.attitude_sample.receipt_time_s = 100.145
    vehicle.attitude_sample.time_boot_s = 12.6
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = SimpleNamespace(
        location=CURRENT,
        attitude=Attitude(-4.0, 103.0, 3.0),
        receipt_time_s=100.155,
    )
    source._on_message(_message("SIM_STATE"))

    assert source.dispatch_available()
    assert source.metrics.advanced_truth_frames == 1


def test_activation_drops_the_pending_attitude_clock() -> None:
    """A clock associated on the cruise leg must not render a scored frame."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))

    source.activate()
    _flush_truth(vehicle, source)

    assert not source.dispatch_available()
    assert source.metrics.projected_frames == 0


def test_activation_during_render_drops_the_cruise_frame() -> None:
    """activate() between capture and publish must void the in-flight frame.

    The pending clock is captured under the lock, but projection runs
    outside it. If activate() lands in that window, `_active` is already
    True when the render publishes, and a cruise-leg frame would become the
    first scored frame (found in review). The activation epoch fences it.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))

    original_render = source._renderer.render

    def activating_render(*args, **kwargs):
        source.activate()
        return original_render(*args, **kwargs)

    source._renderer.render = activating_render
    _flush_truth(vehicle, source)

    assert not source.dispatch_available()
    assert source.metrics.projected_frames == 0
    assert delivered == []


def test_activation_during_association_drops_the_stale_pending() -> None:
    """activate() between commit() and the pending store must drop the pair.

    The association is built and committed outside the lock; if activate()
    lands before the store, the freshly cleared pending slot is refilled
    with a cruise-leg association, and the NEXT truth sample would publish
    it into the scored leg carrying a post-activation epoch, so the render
    fence cannot catch it (found in review).
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    original_commit = source._associator.commit

    def activating_commit(associated) -> None:
        original_commit(associated)
        source.activate()

    source._associator.commit = activating_commit
    source._on_message(_message("ATTITUDE"))

    assert source._state.pending is None
    _flush_truth(vehicle, source)
    assert not source.dispatch_available()
    assert source.metrics.projected_frames == 0
    assert delivered == []


def test_a_render_does_not_delete_a_newer_pending_clock() -> None:
    """Rendering runs outside the lock, so a newer clock can arrive mid-flight.

    Clearing the pending slot blindly when the render finishes would delete
    that newer clock and cost the frame it was about to become (found in
    review). The slot is only emptied if it still holds what was rendered.
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))

    original_render = source._renderer.render

    def render_then_pair_again(*args, **kwargs):
        # A second ATTITUDE closes a fresh truth sample while this render is
        # still projecting.
        vehicle.simulator_truth_pose = SimpleNamespace(
            location=CURRENT,
            attitude=attitude,
            receipt_time_s=100.05,
        )
        source._on_message(_message("SIM_STATE"))
        vehicle.attitude_sample.receipt_time_s = 100.06
        vehicle.attitude_sample.time_boot_s = 12.6
        source._on_message(_message("ATTITUDE"))
        return original_render(*args, **kwargs)

    source._renderer.render = render_then_pair_again
    _flush_truth(vehicle, source)

    assert source._state.pending is not None
    assert source._state.pending.attitude_timestamp_s == 12.6


def test_activation_resets_the_half_finished_association() -> None:
    """A cruise-leg event must not complete a pair after activate().

    The associator holds one pending event of each kind, so a SIM_STATE
    noted before activation could pair with the first scored ATTITUDE and
    stamp a scored frame with a cruise-leg truth sample (found in review).
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])

    source._on_message(_message("SIM_STATE"))

    source.activate()

    vehicle.attitude_sample.receipt_time_s = 100.01
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)

    assert source.metrics.projected_frames == 0
    assert source.metrics.association_refusals == 1


def test_a_frame_taken_before_close_is_not_delivered_after_it() -> None:
    """Taking and delivering cannot be one atomic step.

    `dispatch_available` removes the frame under the lock and then calls a
    foreign callback, so the leg can end in between. A frame belonging to a
    finished leg must not reach the consumer (found in review).
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)

    source.close()

    assert not source.dispatch_available()
    assert delivered == []


def test_activation_restarts_the_frame_tallies() -> None:
    """Metrics describe ONE scored leg, like the advance diagnostics beside
    them: a second activate() must not report a leg's frames plus the last
    one's, nor a skew delta measured across the gap (found in review)."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)
    assert source.dispatch_available()
    assert source.metrics.projected_frames == 1
    assert source.metrics.delivered_frames == 1

    source.activate()

    assert source.metrics.projected_frames == 0
    assert source.metrics.delivered_frames == 0
    assert source.metrics.pose_skew_ms_mean == 0.0


def test_truth_samples_are_recorded_under_the_source_lock() -> None:
    """A SIM_STATE landing while activate() clears the pair is a race.

    note_current() reads the newest sample and writes the pair back; if it
    interleaves with reset_pair() it can restore a pre-activation sample
    and let a rate span the cadence change. The source must serialize the
    record with its own lock (found in review).
    """
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    locked_reads: list[bool] = []
    pose = vehicle.simulator_truth_pose

    def observed_pose() -> SimpleNamespace:
        locked_reads.append(source._lock.locked())
        return pose

    source._truth_history._truth_pose = observed_pose

    source._on_message(_message("SIM_STATE"))

    assert locked_reads == [True]


# --------------------------------------------------------------------------
# source time axis (AAS_TRUTH_POSE_TIME_AXIS=source)
# --------------------------------------------------------------------------


def _truth(
    receipt_s: float, source_s: float | None, attitude: Attitude
) -> SimpleNamespace:
    return SimpleNamespace(
        location=CURRENT,
        attitude=attitude,
        receipt_time_s=receipt_s,
        source_time_s=source_s,
    )


def test_source_axis_renders_on_the_autopilot_clock(monkeypatch) -> None:
    """With ``time_us`` stamps the render clock is the ATTITUDE boot stamp.

    The receipts are arranged so the RECEIPT axis would refuse: the closing
    truth receipt (99.995) sits before the attitude receipt (100.0), which
    on that axis is a forward query. The source stamps bracket the attitude
    BOOT stamp (12.49 <= 12.5 <= 12.515), so a render proves the query ran
    on the autopilot clock, not the host receipt clock.
    """
    monkeypatch.setenv("AAS_TRUTH_POSE_TIME_AXIS", "source")
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(
        truth_attitude=attitude,
        telemetry_attitude=attitude,
        attitude_receipt_s=100.0,
        time_boot_s=12.5,
    )
    vehicle.simulator_truth_pose = _truth(99.98, 12.49, attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    vehicle.simulator_truth_pose = _truth(99.995, 12.515, attitude)
    source._on_message(_message("SIM_STATE"))

    assert source.dispatch_available()
    assert len(delivered) == 1
    diagnostics = source.advance_diagnostics
    assert diagnostics["accepted"] == 1.0
    assert diagnostics["source_axis_fallback"] == 0.0


def test_source_axis_without_stamps_falls_back_and_still_renders(
    monkeypatch,
) -> None:
    """Stock firmware (no ``time_us``): the run degrades to receipt stamps."""
    monkeypatch.setenv("AAS_TRUTH_POSE_TIME_AXIS", "source")
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)

    source._on_message(_message("SIM_STATE"))
    source._on_message(_message("ATTITUDE"))
    _flush_truth(vehicle, source)

    assert source.dispatch_available()
    assert len(delivered) == 1
    assert source.advance_diagnostics["source_axis_fallback"] >= 1.0


def test_unknown_time_axis_env_fails_loud(monkeypatch) -> None:
    """A mistyped A/B arm must die at construction, not fly the wrong axis."""
    monkeypatch.setenv("AAS_TRUTH_POSE_TIME_AXIS", "wall")
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)

    with pytest.raises(ValueError):
        _source(vehicle, [])
# --------------------------------------------------------------------------
# Determinism trace (Landing 1). The trace RECORDS host-dependent selection;
# it must not change any of it.
# --------------------------------------------------------------------------


class _Schedule:
    """Drive the source through a scripted message order, host clock free.

    Every stamp is derived from the step index, never from a wall clock, so
    the same script replays identically under any host timing -- which is what
    "an identical injected schedule" has to mean for a decision-neutrality
    claim to say anything.

    Truth samples advance one pose period on BOTH the source and the receipt
    axis. Each ATTITUDE's boot stamp lands just after the newest truth sample,
    so the next truth sample brackets it: that is the interpolation the source
    holds a pending clock for.
    """

    TRUTH_PERIOD_S = 0.025
    ATTITUDE_INTO_PERIOD_S = 0.010
    ATTITUDE_RECEIPT_LAG_S = 0.005
    FIRST_TRUTH_SOURCE_S = 12.475
    FIRST_TRUTH_RECEIPT_S = 100.0

    def __init__(self, vehicle: Mock, source: DirectTargetPixelSource,
                 attitude: Attitude) -> None:
        self._vehicle = vehicle
        self._source = source
        self._attitude = attitude
        self._index = 0
        self._truth_source_s = self.FIRST_TRUTH_SOURCE_S
        self._truth_receipt_s = self.FIRST_TRUTH_RECEIPT_S
        self.dispatches: list[bool] = []

    def refresh(self, *, readable: bool = True) -> None:
        """Advance the vehicle's cached truth pose WITHOUT notifying the source.

        The vehicle caches a decoded SIM_STATE independently of our
        subscription callback, so the cache can be a sample ahead of what the
        source has processed. Splitting the two is what makes the pending-clock
        overwrite reachable from a schedule instead of by poking state.
        """
        self._truth_source_s = (
            self.FIRST_TRUTH_SOURCE_S + self._index * self.TRUTH_PERIOD_S
        )
        self._truth_receipt_s = (
            self.FIRST_TRUTH_RECEIPT_S + self._index * self.TRUTH_PERIOD_S
        )
        self._index += 1
        self._vehicle.simulator_truth_pose = (
            _truth(self._truth_receipt_s, self._truth_source_s, self._attitude)
            if readable
            else None
        )

    def truth(self, *, readable: bool = True) -> None:
        self.refresh(readable=readable)
        self._source._on_message(_message("SIM_STATE"))

    def attitude(self) -> None:
        self._vehicle.attitude_sample = SimpleNamespace(
            attitude=self._attitude,
            time_boot_s=self._truth_source_s + self.ATTITUDE_INTO_PERIOD_S,
            receipt_time_s=(
                self._truth_receipt_s + self.ATTITUDE_RECEIPT_LAG_S
            ),
            body_rates_rad_s=(0.1, 0.2, 0.3),
        )
        self._source._on_message(_message("ATTITUDE"))

    def dispatch(self) -> None:
        self.dispatches.append(self._source.dispatch_available())

    def run(self, script: str) -> None:
        for step in script.split():
            if step == "T":
                self.truth()
            elif step == "t":
                self.truth(readable=False)
            elif step == "r":
                self.refresh()
            elif step == "A":
                self.attitude()
            elif step == "D":
                self.dispatch()
            else:  # pragma: no cover - a typo in a test script
                raise AssertionError(f"unknown schedule step {step!r}")


# One clean frame; a refused association; two renders with only one dispatch
# in between (the publish-slot overwrite that produced the archived
# projected-minus-delivered gap); then a dispatch with nothing to take.
NEUTRALITY_SCHEDULE = "T T A T D A A T D A T A T D D"


def _replay(script: str, *, tracing: bool, monkeypatch) -> dict:
    """Run ``script`` with tracing on or off and return only what control sees."""
    monkeypatch.setattr(determinism_trace, "ENABLED", tracing)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)
    assert (source.determinism_trace is not None) is tracing
    schedule = _Schedule(vehicle, source, attitude)
    schedule.run(script)
    return {
        "dispatches": tuple(schedule.dispatches),
        "digests": tuple(frame_digest(frame) for frame in delivered),
        "source_now_s": source.source_now(),
        "metrics": source.metrics,
        "advance": dict(source.advance_diagnostics),
        "trace": source.determinism_trace,
        "source": source,
    }


def test_tracing_is_off_by_default() -> None:
    """The gate is a module boolean, so an unset environment must trace not."""
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    assert source.determinism_trace is None


def test_tracing_does_not_change_any_selection(monkeypatch) -> None:
    """Decision-neutral: same schedule, same commands, traced or not.

    Not physically neutral -- digesting costs CPU -- so the schedule is
    injected rather than timed.
    """
    off = _replay(NEUTRALITY_SCHEDULE, tracing=False, monkeypatch=monkeypatch)
    on = _replay(NEUTRALITY_SCHEDULE, tracing=True, monkeypatch=monkeypatch)

    assert off["trace"] is None
    assert on["trace"] is not None
    # The frames the navigation law received, bit for bit, in order.
    assert on["digests"] == off["digests"]
    # Real command-input bytes, not merely non-None. PAYLOAD_UNREADABLE
    # is non-None, so `is not None` alone passed a run in which EVERY
    # digest had failed to read -- which is the opposite of what this
    # test exists to establish.
    assert all(
        digest is not None and digest != PAYLOAD_UNREADABLE
        for digest in on["digests"]
    )
    # The per-slot outcome sequence, including the empty one.
    assert on["dispatches"] == off["dispatches"]
    assert on["source_now_s"] == off["source_now_s"]
    assert on["metrics"] == off["metrics"]
    assert on["advance"] == off["advance"]


def test_the_traced_schedule_actually_exercises_the_coalescing(
    monkeypatch,
) -> None:
    """A neutrality proof over a schedule with no discards proves nothing."""
    result = _replay(NEUTRALITY_SCHEDULE, tracing=True, monkeypatch=monkeypatch)
    summary = summarise(result["trace"])
    assert summary["complete"] is True
    assert summary["first_violation"] is None
    assert summary["period_us"] == 20_000
    assert summary["counts"] == {
        determinism_events.EVENT_ASSOCIATION: {
            determinism_events.ASSOCIATION_COMMITTED: 4,
            determinism_events.ASSOCIATION_REFUSED: 1,
        },
        determinism_events.EVENT_TRUTH: {determinism_events.TRUTH_RECORDED: 6},
        determinism_events.EVENT_STAGE: {determinism_events.STAGE_STAGED: 4},
        determinism_events.EVENT_DECIMATE: {
            determinism_events.DISCARD_PUBLISH_OVERWRITTEN: 1
        },
        determinism_events.EVENT_OUTPUT: {
            determinism_events.OUTPUT_DELIVERED: 3,
            determinism_events.OUTPUT_EMPTY: 1,
        },
        # The one boundary this replay crosses: _source() activates it.
        determinism_events.EVENT_LIFECYCLE: {
            determinism_events.LIFECYCLE_ACTIVATED: 1
        },
        # The replay drives the message handler directly and never calls
        # start() or close(), so no subscription opened or closed.
        determinism_events.EVENT_SUBSCRIPTION: {},
        determinism_events.EVENT_VIOLATION: {},
    }
    # This schedule dispatches straight after staging with no ATTITUDE in
    # between, so both lags are zero BY CONSTRUCTION. The distribution that
    # matters comes from a flight.
    assert summary["dispatch_lag_slots"] == {"count": 3, "min": 0, "max": 0}
    assert summary["stage_lag_slots"] == {"count": 4, "min": 0, "max": 0}
    # Nothing drove a worker here, so no iteration was ever published and
    # the OUTPUT rows carry no iteration to report.
    assert summary["ready_iterations"] == {
        "count": 0, "min": None, "max": None
    }


def _publish_slot_ledger(trace, still_staged_epoch: int | None) -> dict:
    """Every frame that ENTERED the publish slot, and how it left, per epoch.

    A frame reaches the slot only through ``publish``, and it leaves exactly
    once: displaced by a newer frame, emptied by a leg boundary, or taken by a
    dispatch. Because ``activate()`` empties the slot, a frame always leaves in
    the epoch it entered, so the books balance epoch by epoch.
    """
    ledger: dict[int, dict[str, int]] = {}

    def bucket(epoch: int) -> dict[str, int]:
        return ledger.setdefault(
            epoch, {"in": 0, "overwritten": 0, "leg_ended": 0, "taken": 0,
                    "still_staged": 0}
        )

    for row in list(trace.capture().rows):
        event, epoch, outcome = row[0], row[1], row[2]
        if event == determinism_events.EVENT_STAGE:
            if outcome == determinism_events.STAGE_STAGED:
                bucket(epoch)["in"] += 1
        elif event == determinism_events.EVENT_DECIMATE:
            if outcome == determinism_events.DISCARD_PUBLISH_OVERWRITTEN:
                bucket(epoch)["overwritten"] += 1
            elif outcome == determinism_events.DISCARD_PUBLISH_LEG_ENDED:
                bucket(epoch)["leg_ended"] += 1
        elif event == determinism_events.EVENT_OUTPUT:
            # An output row with a source stamp took a frame out of the slot;
            # OUTPUT_EMPTY found nothing there.
            if row[3] is not None:
                bucket(epoch)["taken"] += 1
    if still_staged_epoch is not None:
        bucket(still_staged_epoch)["still_staged"] += 1
    return ledger


def _assert_publish_slot_balances(source) -> dict:
    trace = source.determinism_trace
    assert trace is not None
    still = source._state.epoch if source._state.latest is not None else None
    ledger = _publish_slot_ledger(trace, still)
    for epoch, counts in ledger.items():
        assert counts["in"] == (
            counts["overwritten"] + counts["leg_ended"] + counts["taken"]
            + counts["still_staged"]
        ), f"publish slot does not balance in epoch {epoch}: {counts}"
    return ledger


def test_the_trace_accounts_for_the_projected_minus_delivered_gap(
    monkeypatch,
) -> None:
    """The aggregate counters SHOW the gap; the trace has to EXPLAIN it.

    Every frame that entered the publish slot left it exactly once. If the
    books do not balance, a record site is missing and no verdict built on the
    trace can be trusted.
    """
    result = _replay(NEUTRALITY_SCHEDULE, tracing=True, monkeypatch=monkeypatch)
    metrics = result["metrics"]
    ledger = _assert_publish_slot_balances(result["source"])
    assert metrics.projected_frames == 4
    assert metrics.delivered_frames == 3
    assert ledger[1] == {
        "in": 4,
        "overwritten": 1,
        "leg_ended": 0,
        "taken": 3,
        "still_staged": 0,
    }
    # The counters and the trace agree on how many frames were published.
    assert metrics.projected_frames == ledger[1]["in"]


def test_the_publish_slot_balances_across_a_leg_boundary(monkeypatch) -> None:
    """activate() empties both slots; without a row those samples vanish.

    W5: the engagement boundary must be visible in the ledger, because
    activate() lands at an arbitrary host instant and decides which frame does
    NOT seed the scored leg.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    # A frame is staged and a clock is held, then the leg restarts.
    schedule.run("T T A T A")
    first_leg = source._state.epoch
    source.activate()
    schedule.run("T T A T D")

    ledger = _assert_publish_slot_balances(source)
    assert ledger[first_leg]["leg_ended"] == 1
    reasons = [
        row[2]
        for row in list(source.determinism_trace.capture().rows)
        if row[0] == determinism_events.EVENT_DECIMATE
        and row[1] == first_leg
    ]
    # The held clock is recorded too, attributed to the leg that ended.
    assert determinism_events.DISCARD_PENDING_LEG_ENDED in reasons
    assert determinism_events.DISCARD_PUBLISH_LEG_ENDED in reasons


def test_close_records_what_it_fenced_out_of_the_leg(monkeypatch) -> None:
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("T T A T A")
    leg = source._state.epoch
    source.close()

    _assert_publish_slot_balances(source)
    assert sorted(
        row[2]
        for row in list(source.determinism_trace.capture().rows)
        if row[0] == determinism_events.EVENT_DECIMATE and row[1] == leg
    ) == [
        determinism_events.DISCARD_PENDING_LEG_ENDED,
        determinism_events.DISCARD_PUBLISH_LEG_ENDED,
    ]


def test_close_returns_the_epoch_its_leg_ended_rows_carry(monkeypatch) -> None:
    """The owner needs the ended leg's epoch, and the rows cannot vouch for it.

    A dispatch that takes its frame after close() records at the NEXT epoch,
    and a boundary with both slots empty recorded nothing (review round 1, F1
    and F3); its own LIFECYCLE row can still be lost to an overflow or a
    fault. So close() returns the epoch it attributed the leg's end to.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    _Schedule(vehicle, source, attitude).run("T T A T A")

    ending = source.close()

    leg_ended = {
        row[1]
        for row in list(source.determinism_trace.capture().rows)
        if row[0] == determinism_events.EVENT_DECIMATE
        and row[2] in (
            determinism_events.DISCARD_PENDING_LEG_ENDED,
            determinism_events.DISCARD_PUBLISH_LEG_ENDED,
        )
    }
    assert leg_ended == {ending} == {1}


def test_a_quiet_leg_still_names_its_epoch_on_close(monkeypatch) -> None:
    """F3: both slots empty, and close() still names the leg it ended.

    Each boundary also leaves its own row now, at the epoch it ended and with
    the epoch it made current (Landing 2, D4). A quiet boundary used to leave
    no row at all, and this asserted an empty trace. close() also says the
    subscriptions are closing now, after the boundary, whether or not start()
    opened them (D3), and there is no ruling for it to name yet.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])

    assert source.close() == 1
    assert list(source.determinism_trace.capture().rows) == [
        (determinism_events.EVENT_LIFECYCLE, 0,
         determinism_events.LIFECYCLE_ACTIVATED, 1, None, None),
        (determinism_events.EVENT_LIFECYCLE, 1,
         determinism_events.LIFECYCLE_CLOSED, 2, None, None),
        (determinism_events.EVENT_SUBSCRIPTION, 1,
         determinism_events.SUBSCRIPTION_CLOSED, None),
    ]


def test_every_boundary_leaves_one_row_first_at_the_ending_epoch(
    monkeypatch,
) -> None:
    """D4: each activate() and close() records one LIFECYCLE row at the epoch
    it ENDED, with the epoch it made current and the watermark it saw, before
    the discards it caused and whether or not either slot held anything."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    _Schedule(vehicle, source, attitude).run("T T A T A")
    source.activate()
    ending = source.close()

    trace = source.determinism_trace
    rows = list(trace.capture().rows)
    at = [
        index for index, row in enumerate(rows)
        if row[0] == determinism_events.EVENT_LIFECYCLE
    ]
    assert [rows[index][1:4] for index in at] == [
        (0, determinism_events.LIFECYCLE_ACTIVATED, 1),
        (1, determinism_events.LIFECYCLE_ACTIVATED, 2),
        (2, determinism_events.LIFECYCLE_CLOSED, 3),
    ]
    assert ending == 2 and source._state.epoch == 3
    # The second activate() emptied both slots: its row comes first, and the
    # two discards follow it, at the same, ending epoch.
    assert [row[:3] for row in rows[at[1] + 1:at[1] + 3]] == [
        (determinism_events.EVENT_DECIMATE, 1,
         determinism_events.DISCARD_PENDING_LEG_ENDED),
        (determinism_events.EVENT_DECIMATE, 1,
         determinism_events.DISCARD_PUBLISH_LEG_ENDED),
    ]
    # The first boundary came before any ATTITUDE; the later two carry the
    # watermark the journal held, and its slot.
    watermark = trace.watermark_us()
    assert watermark is not None
    assert [rows[index][4:] for index in at] == [
        (None, None),
        (watermark, watermark // trace.period_us),
        (watermark, watermark // trace.period_us),
    ]


def test_a_boundary_reads_both_epochs_inside_its_own_transition() -> None:
    """The resulting epoch is not worked out by the caller: the state returns
    it from the same transition that ended the leg, beside what it emptied."""
    state = DirectPublishState()
    state.pending, state.latest = "held", "shown"

    assert state.activate() == LegBoundary(
        determinism_events.LIFECYCLE_ACTIVATED, 0, 1, "held", "shown"
    )
    assert (state.epoch, state.pending, state.latest) == (1, None, None)
    assert state.close() == LegBoundary(
        determinism_events.LIFECYCLE_CLOSED, 1, 2, None, None
    )
    assert state.epoch == 2


def test_a_leg_never_activated_closes_as_epoch_zero(monkeypatch) -> None:
    """The cruise leg, never engaged, is visible as epoch 0 rather than scored."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: True,
        wall_now_s=lambda: 100.0,
    )

    assert source.close() == 0


def test_a_rejected_frame_still_leaves_the_publish_slot(monkeypatch) -> None:
    """A refusing consumer takes the frame; it is not left behind."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: False,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    _Schedule(vehicle, source, attitude).run("T T A T D")

    ledger = _assert_publish_slot_balances(source)
    assert ledger[source._state.epoch]["taken"] == 1
    assert source.metrics.delivery_rejections == 1


def test_publish_slot_overwrite_names_the_frame_no_dispatch_took(
    monkeypatch,
) -> None:
    """The archived 8.9-10.6% gap, itemised: which frame, from which slot."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)
    schedule = _Schedule(vehicle, source, attitude)

    # Two renders, one dispatch: the first staged frame is displaced.
    schedule.run("T T A T A T D")

    assert len(delivered) == 1
    trace = source.determinism_trace
    assert trace is not None
    rows = list(trace.capture().rows)
    staged = [
        row
        for row in rows
        if row[0] == determinism_events.EVENT_STAGE
        and row[2] == determinism_events.STAGE_STAGED
    ]
    discarded = [
        row
        for row in rows
        if row[0] == determinism_events.EVENT_DECIMATE
        and row[2] == determinism_events.DISCARD_PUBLISH_OVERWRITTEN
    ]
    assert len(staged) == 2
    assert len(discarded) == 1
    # The victim is the OLDER staged frame, and the survivor is what shipped.
    assert discarded[0][3] == staged[0][3]
    assert delivered[0].pixel.source_timestamp_s * 1e6 == staged[1][3]
    # Slot keys are real integers on the autopilot clock, never floats.
    assert type(discarded[0][4]) is int
    assert discarded[0][4] < staged[1][4]


def test_pending_clock_overwrite_is_recorded_when_truth_is_unreadable(
    monkeypatch,
) -> None:
    """The other coalescer: a held clock displaced before it ever rendered.

    Reachable without poking state: a SIM_STATE arriving while the vehicle's
    truth cache is momentarily unreadable notes a truth event but records no
    sample, so the held clock survives it. The cache then refreshes, and the
    next ATTITUDE -- which now has newer receipts on both sides -- commits and
    displaces the clock that never rendered.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("T T A t r A")

    trace = source.determinism_trace
    assert trace is not None
    rows = list(trace.capture().rows)
    assert [
        row[2]
        for row in rows
        if row[0] == determinism_events.EVENT_TRUTH
    ] == [
        determinism_events.TRUTH_RECORDED,
        determinism_events.TRUTH_RECORDED,
        determinism_events.TRUTH_UNREADABLE,
    ]
    displaced = [
        row
        for row in rows
        if row[0] == determinism_events.EVENT_DECIMATE
        and row[2] == determinism_events.DISCARD_PENDING_OVERWRITTEN
    ]
    assert len(displaced) == 1
    # Attributed against the watermark of the sample that displaced it.
    assert displaced[0][3] < displaced[0][5]


def test_unbracketable_drop_is_not_counted_as_an_overwrite(
    monkeypatch,
) -> None:
    """A deliberate drop and a host-timing overwrite must stay distinguishable."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("T T A")
    # Jump the truth history far past the held clock: it can no longer be
    # bracketed, and the history only moves further away.
    vehicle.simulator_truth_pose = _truth(101.0, 13.5, attitude)
    source._on_message(_message("SIM_STATE"))

    trace = source.determinism_trace
    assert trace is not None
    reasons = [
        row[2]
        for row in list(trace.capture().rows)
        if row[0] == determinism_events.EVENT_DECIMATE
    ]
    assert reasons == [determinism_events.DISCARD_UNBRACKETABLE]


def test_every_dispatch_attempt_records_exactly_one_output_slot(
    monkeypatch,
) -> None:
    """One row per logical output slot, whether or not a frame was there."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    delivered: list = []
    source = _source(vehicle, delivered)
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("D T T A T D D")

    trace = source.determinism_trace
    assert trace is not None
    outputs = [
        row[2]
        for row in list(trace.capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
    ]
    assert outputs == [
        determinism_events.OUTPUT_EMPTY,
        determinism_events.OUTPUT_DELIVERED,
        determinism_events.OUTPUT_EMPTY,
    ]
    assert len(outputs) == len(schedule.dispatches)


def test_a_rejected_delivery_is_recorded_as_its_own_outcome(
    monkeypatch,
) -> None:
    """A refusing consumer and an empty slot are different failures."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: False,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("T T A T D")

    assert schedule.dispatches == [False]
    trace = source.determinism_trace
    assert trace is not None
    assert [
        row[2]
        for row in list(trace.capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
    ] == [determinism_events.OUTPUT_REJECTED]
    assert source.metrics.delivery_rejections == 1


def test_a_frame_from_the_previous_leg_is_fenced_not_staged(
    monkeypatch,
) -> None:
    """W5: activate() must not let a cruise-leg frame seed the scored leg."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    schedule.run("T T A")
    epoch_before = source._state.epoch
    source.activate()
    # The truth sample that would have rendered the held clock now arrives
    # after the leg changed.
    schedule.run("T")

    trace = source.determinism_trace
    assert trace is not None
    rows = list(trace.capture().rows)
    assert not [
        row
        for row in rows
        if row[0] == determinism_events.EVENT_STAGE
        and row[2] == determinism_events.STAGE_STAGED
    ]
    # Every row carries its epoch, which is what makes the boundary visible
    # rather than assumed. Each boundary also leaves its own row at the epoch
    # it ENDED, so the activate() inside _source() adds the epoch before these.
    assert {row[1] for row in rows} == {
        epoch_before - 1, epoch_before, epoch_before + 1
    }


def test_the_trace_holds_its_rows_in_memory_until_drained(
    monkeypatch,
) -> None:
    """No file I/O during engagement: the rows are only in the process."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)

    opened: list = []
    real_open = builtins.open

    def _record_open(*args, **kwargs):
        opened.append(args[0] if args else None)
        return real_open(*args, **kwargs)

    monkeypatch.setattr(builtins, "open", _record_open)
    schedule.run(NEUTRALITY_SCHEDULE)

    assert opened == []
    trace = source.determinism_trace
    assert trace is not None
    assert list(trace.capture().rows)
    assert trace.journal.drain()
    assert list(trace.capture().rows) == []


def test_the_grid_period_comes_from_the_autopilot_scheduler_rate(
    monkeypatch,
) -> None:
    """W1's grid is the SCHEDULER period, not the pose period.

    Pose runs at 0.8x the scheduler rate, so ~1 in 5 slots is legitimately
    empty and holds. Keying on the pose period would hide exactly that.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    vehicle.get_param_or_default.return_value = 400.0
    source = _source(vehicle, [])
    trace = source.determinism_trace
    assert trace is not None
    assert trace.period_us == 2_500


def test_an_unreadable_scheduler_rate_still_records_without_slots(
    monkeypatch,
) -> None:
    """A missing rate must not invent a period; rows stay, keys go None.

    Every slot column is checked, chosen by the row's event. A LIFECYCLE row
    keeps its watermark stamp at index 4 and its slot at 5, and a review found
    this test reading index 4 of every row: it passed only because the one
    boundary row came before any watermark, and a lifecycle builder inventing
    slot 0 passed every test. So the leg is closed after an ATTITUDE too.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    # Patched where the ONE read happens. An unusable rate leaves the
    # association window on its own fallback and the grid with no period,
    # which is the whole point of not resolving it twice.
    monkeypatch.setattr(
        pose_stream_link,
        "resolve_ardupilot_scheduler_rate_hz",
        lambda vehicle: 0.0,
    )
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)
    schedule.run("T T A T D")
    source.close()

    trace = source.determinism_trace
    assert trace is not None
    assert trace.period_us == 0
    rows = list(trace.capture().rows)
    assert rows
    # (stamp, slot) column pairs per layout, as determinism_events documents
    # them; a VIOLATION row carries no slot.
    columns = {
        determinism_events.EVENT_ASSOCIATION: ((3, 4),),
        determinism_events.EVENT_TRUTH: ((3, 4),),
        determinism_events.EVENT_STAGE: ((3, 4), (5, 6)),
        determinism_events.EVENT_DECIMATE: ((3, 4), (5, 6)),
        determinism_events.EVENT_OUTPUT: ((3, 4), (5, 6)),
        determinism_events.EVENT_LIFECYCLE: ((4, 5),),
        # A ruling at index 3, and no stamp or slot at all.
        determinism_events.EVENT_SUBSCRIPTION: (),
        determinism_events.EVENT_VIOLATION: (),
    }
    assert {row[0] for row in rows} <= set(columns), (
        "a row's event has no slot columns listed, so none were checked"
    )
    pairs = [
        (row[0], row[stamp], row[slot])
        for row in rows
        for stamp, slot in columns[row[0]]
    ]
    assert [pair for pair in pairs if pair[2] is not None] == []
    # The controls: stamps are still recorded, so a None slot is the missing
    # period and not missing data, and that holds for the boundary rows too.
    assert any(stamp is not None for _, stamp, _ in pairs)
    boundaries = [
        (stamp, slot)
        for event, stamp, slot in pairs
        if event == determinism_events.EVENT_LIFECYCLE
    ]
    assert boundaries[0] == (None, None), "activated before any ATTITUDE"
    assert boundaries[-1][0] is not None, "closed after one, with a stamp"

@pytest.mark.parametrize("tracing", [False, True])
def test_the_scheduler_rate_is_read_once_whatever_the_trace_does(
    tracing, monkeypatch
) -> None:
    """One SCHED_LOOP_RATE read, tracing on or off.

    Two reasons, and both are load-bearing. A DISABLED trace is the production
    path and must cost the vehicle nothing. An ENABLED one must key its grid on
    the SAME answer the association skew window was sized on -- SCHED_LOOP_RATE
    is a live-link round trip and a second read can answer differently.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", tracing)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    reads: list[str] = []

    def counting_param(name: str, default: float) -> float:
        reads.append(name)
        # A CHANGING answer exposes a second read that a constant would hide.
        return 50.0 if len(reads) == 1 else 400.0

    vehicle.get_param_or_default = counting_param
    source = _source(vehicle, [])

    assert reads == ["SCHED_LOOP_RATE"]
    assert source._stream.rate_hz == 40.0
    assert source._stream.scheduler_rate_hz == 50.0
    if tracing:
        assert source.determinism_trace.period_us == 20_000


def test_a_raising_consumer_still_leaves_a_row_and_a_tally(monkeypatch) -> None:
    """A frame the consumer crashed on has LEFT the publish slot.

    NavigationCommandWorker CATCHES the exception and logs it
    (navigation_command_worker.py:136-140), so nothing downstream would show
    that a frame had been taken and lost. The exception itself must reach the
    worker unchanged.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)

    def boom(detection) -> bool:
        raise RuntimeError("consumer blew up")

    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=boom,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    _Schedule(vehicle, source, attitude).run("T T A T")

    with pytest.raises(RuntimeError, match="consumer blew up"):
        source.dispatch_available()

    metrics = source.metrics
    assert metrics.projected_frames == 1
    assert metrics.delivered_frames == 0
    assert metrics.delivery_rejections == 0
    assert metrics.delivery_exceptions == 1
    outputs = [
        row
        for row in list(source.determinism_trace.capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
    ]
    assert [row[2] for row in outputs] == [determinism_events.OUTPUT_EXCEPTION]
    # The row names WHICH frame was lost, not just that one was.
    assert outputs[0][3] is not None
    ledger = _assert_publish_slot_balances(source)
    assert ledger[source._state.epoch]["taken"] == 1


def test_the_dispatch_lag_is_stamped_when_the_frame_was_taken(
    monkeypatch,
) -> None:
    """Delivery runs UNLOCKED, so the input stream keeps advancing during it.

    Those samples were not known when the dispatch chose the frame. Read live
    at the end of delivery, the watermark would make the frame look staler
    than it was and OVERSTATE the lag a frozen lookahead has to cover.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    driver: dict = {}

    def deliver_while_the_stream_advances(detection) -> bool:
        driver["schedule"].run("A T A T")
        return True

    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=deliver_while_the_stream_advances,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    driver["schedule"] = _Schedule(vehicle, source, attitude)
    driver["schedule"].run("T T A T")
    assert source.dispatch_available()

    trace = source.determinism_trace
    output = [
        row
        for row in list(trace.capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
    ][-1]
    # The stream DID advance mid-delivery ...
    assert trace.watermark_us() > output[5]
    # ... and the row is still stamped at the instant of the take.
    assert output[6] == output[4]
    summary = summarise(trace, epoch=source._state.epoch)
    assert summary["dispatch_lag_slots"] == {"count": 1, "min": 0, "max": 0}
    _assert_publish_slot_balances(source)


class _ProxyLaw:
    """A consumer with MEMORY, so which frame came BEFORE is visible in what
    it emits.

    It differences consecutive frames the way the real terminal law forms a
    rate. On this bench the aircraft pose is held fixed, so the pixel never
    moves and only the source clock carries information -- the span back to
    the previous frame is what a different coalescing choice would alter, so
    that is what the command records.
    """

    def __init__(self, queue: list) -> None:
        self.commands: list[tuple[float, float, float]] = []
        self._queue = queue
        self._previous: tuple[float, float] | None = None

    def __call__(self, detection) -> bool:
        pixel = detection.pixel
        current = (float(pixel.source_timestamp_s), float(pixel.u_px))
        previous = self._previous
        span = 0.0 if previous is None else current[0] - previous[0]
        formed = (current[0], span, current[1])
        self.commands.append(formed)
        # QUEUED, not issued: the worker takes work at the top of its NEXT
        # pass, which is the real relationship this bench has to reproduce.
        self._queue.append(formed)
        self._previous = current
        return True


class _ExecutingRuntime:
    """A runtime that really executes queued work and returns a command.

    ``executed`` is the independent sink: it is filled identically whether
    tracing is on or off, so it is the control the recorder is compared
    against rather than a second reading of the recorder itself.
    """

    def __init__(self, queue: list, executed: list) -> None:
        self._queue = queue
        self.executed = executed

    def has_command_pending_or_in_flight(self) -> bool:
        return bool(self._queue)

    def take_work(self):
        return self._queue.pop(0) if self._queue else None

    def execute_work(self, work):
        source_s, span_s, u_px = work
        command = CalcData(u_px, span_s, source_s, None, 0.5)
        self.executed.append(command)
        return command

    def finish_work(self, work) -> None:
        return None

    def postprocess_job(self, work):
        return None


def _run_through_the_worker(script: str, *, tracing: bool, monkeypatch) -> dict:
    """Drive the REAL NavigationCommandWorker over ``script``.

    The worker owns the once-per-slot dispatch and the exception guard around
    it, so a neutrality claim about commands has to go through it rather than
    calling ``dispatch_available`` directly. Its clock is driven, never slept:
    ``monotonic_s`` is always past the deadline, so ``sleep_s`` must not run.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", tracing)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    queue: list = []
    executed: list = []
    law = _ProxyLaw(queue)
    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=law,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    schedule = _Schedule(vehicle, source, attitude)
    # One worker iteration per D: the messages before it, then its dispatch.
    groups = script.split("D")[:-1]
    stop_event = threading.Event()
    pending = iter(groups)
    taken: list[bool] = []
    runtime = _ExecutingRuntime(queue, executed)

    def source_dispatch() -> bool:
        group = next(pending, None)
        if group is not None:
            schedule.run(group)
        result = source.dispatch_available()
        taken.append(result)
        # One pass PAST the last group, so the work the final dispatch
        # queued is executed instead of being left on the queue.
        if len(taken) > len(groups):
            stop_event.set()
        return result

    @contextlib.contextmanager
    def runtime_session():
        yield runtime

    def never_sleep(seconds: float) -> None:
        raise AssertionError("the driven clock is always past the deadline")

    clock = itertools.count(0.0, 1.0)
    NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=Mock(),
            source_dispatch=source_dispatch,
            command_loop=command_loop_observer(source.determinism_trace),
            monotonic_s=lambda: next(clock),
            sleep_s=never_sleep,
        )
    ).run()
    return {
        "commands": tuple(law.commands),
        "executed": tuple(command_digest(c) for c in executed),
        "taken": tuple(taken),
        "metrics": source.metrics,
        "trace": source.determinism_trace,
    }


def test_the_commands_the_worker_issues_are_the_same_traced_or_not(
    monkeypatch,
) -> None:
    """Decision-neutral at the COMMAND BYTES, not merely at the frame.

    The work really reaches ``execute_work``, and what it returned is compared
    bit for bit between a traced and an untraced run. The sink it is compared
    from (``_ExecutingRuntime.executed``) is filled identically in both modes,
    so the control is not a second reading of the recorder.

    What it still does not do: instantiate the real parking navigation law. The
    law here forms a rate from consecutive frames, which is the property a
    different coalescing victim would change; it is not the real law's
    arithmetic.
    """
    off = _run_through_the_worker(
        NEUTRALITY_SCHEDULE, tracing=False, monkeypatch=monkeypatch
    )
    on = _run_through_the_worker(
        NEUTRALITY_SCHEDULE, tracing=True, monkeypatch=monkeypatch
    )

    assert off["trace"] is None
    assert on["trace"] is not None
    assert on["commands"] == off["commands"]
    # The law's memory is engaged, or this compares three independent frames.
    # Every command after the first is formed against the frame that preceded
    # it, so a different coalescing victim would change it.
    assert len(on["commands"]) == 3
    assert on["commands"][0][1] == 0.0
    assert all(command[1] > 0.0 for command in on["commands"][1:])
    assert on["taken"] == off["taken"] == (True, True, True, False, False)
    assert on["metrics"] == off["metrics"]

    # The commands the worker EXECUTED, bitwise, and there really were some.
    assert len(on["executed"]) == 3
    assert all(digest is not None for digest in on["executed"])
    assert on["executed"] == off["executed"]


def test_the_ledger_holds_the_commands_the_worker_actually_executed(
    monkeypatch,
) -> None:
    """The recorded digests are the executed ones, on the right iterations.

    Also the first honest measurement of the input-to-command relationship on
    this path: a frame first ready on pass j reaches ``execute_work`` on pass
    j+1, because the worker takes work at the TOP of a pass and admits the
    newest source sample at the BOTTOM. Measured here, not assumed -- and
    still an ITERATION offset, not an autopilot-slot lag.
    """
    on = _run_through_the_worker(
        NEUTRALITY_SCHEDULE, tracing=True, monkeypatch=monkeypatch
    )
    entries = on["trace"].command_log.entries()

    # One entry per executed command, and the same bytes in the same order.
    assert tuple(digest for _, digest in entries) == on["executed"]
    issued_on = [iteration for iteration, _ in entries]
    assert issued_on == [2, 3, 4]

    summary = summarise(on["trace"])
    # Five passes ran: four scripted, plus the one that drains the last work.
    assert summary["commands"]["iterations"] == 5
    assert summary["commands"]["entries"] == 3
    assert summary["commands"]["digested"] == 3
    assert summary["commands"]["dropped"] == 0
    assert summary["commands"]["failed"] is False

    # NO FALSE ALARM on real staged frames. PAYLOAD_UNREADABLE was introduced so
    # that an undigestable payload could not read as a clean run; the danger it
    # brought with it is the opposite mistake -- classifying something the
    # pipeline legitimately produces as a hole, which would throw away real
    # flight data. Every frame here came through the actual renderer.
    assert summary["payload_unreadable"] is False
    assert summary["complete"] is True
    staged = [
        row
        for row in list(on["trace"].capture().rows)
        if row[0] == determinism_events.EVENT_STAGE
        and row[2] == determinism_events.STAGE_STAGED
    ]
    assert staged, "no frame was staged, so this proves nothing"
    assert all(
        row[determinism_events.STAGE_PAYLOAD_DIGEST] not in
        (None, PAYLOAD_UNREADABLE)
        for row in staged
    )

    # The frames were ready one pass before the command that carried them.
    assert summary["ready_iterations"] == {"count": 3, "min": 1, "max": 3}
    ready_on = [
        row[7]
        for row in list(on["trace"].capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
        and row[2] == determinism_events.OUTPUT_DELIVERED
    ]
    assert ready_on == [1, 2, 3]
    assert issued_on == [pass_number + 1 for pass_number in ready_on]


def test_a_crashing_command_is_recorded_as_a_crash_not_as_no_command(
    monkeypatch,
) -> None:
    """A raise and a None return are different outcomes, so different bytes.

    Two runs that differ only in which one crashed are not the same run, and a
    digest of None for both would say they were.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    trace = determinism_trace.DeterminismTrace(20_000, capacity=8)
    log = trace.command_log
    logger = Mock()

    class _Raises:
        def has_command_pending_or_in_flight(self) -> bool:
            return True

        def take_work(self):
            return "work"

        def execute_work(self, work):
            raise RuntimeError("law blew up")

        def finish_work(self, work) -> None:
            return None

        def postprocess_job(self, work):
            return None

    stop_event = threading.Event()

    @contextlib.contextmanager
    def runtime_session():
        yield _Raises()

    clock = itertools.count(0.0, 1.0)
    NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=logger,
            source_dispatch=lambda: stop_event.set() or False,
            command_loop=command_loop_observer(trace),
            monotonic_s=lambda: next(clock),
            sleep_s=lambda seconds: None,
        )
    ).run()

    assert logger.error.called
    assert log.entries() == [(1, determinism_command_log.COMMAND_RAISED)]
    assert log.failed is False


def test_a_command_that_escapes_as_a_base_exception_is_still_a_crash() -> None:
    """``except Exception`` does not catch everything the loop can meet.

    A KeyboardInterrupt or SystemExit out of ``execute_work`` propagates by
    design, but the ledger still has to say a crash happened: recorded as "ran
    work, produced nothing" it would be indistinguishable from a law that
    legitimately returned None, and two runs differing only in a crash would
    compare equal.
    """
    trace = determinism_trace.DeterminismTrace(20_000, capacity=8)
    log = trace.command_log
    stop_event = threading.Event()

    class _Interrupts:
        def has_command_pending_or_in_flight(self) -> bool:
            return True

        def take_work(self):
            return "work"

        def execute_work(self, work):
            raise KeyboardInterrupt("not an Exception")

        def finish_work(self, work) -> None:
            return None

        def postprocess_job(self, work):
            return None

    @contextlib.contextmanager
    def runtime_session():
        yield _Interrupts()

    clock = itertools.count(0.0, 1.0)
    worker = NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=Mock(),
            source_dispatch=lambda: stop_event.set() or False,
            command_loop=command_loop_observer(trace),
            monotonic_s=lambda: next(clock),
            sleep_s=lambda seconds: None,
        )
    )
    # It propagates: the worker must not swallow a BaseException.
    with pytest.raises(KeyboardInterrupt):
        worker.run()

    assert log.entries() == [(1, determinism_command_log.COMMAND_RAISED)]


# Both names the command log answers to on a trace: the private field and the
# PUBLIC property. Recognising only the private one let a nesting spelled
# ``self.command_log.dropped`` pass -- found by a review that wrote it that way.
# The row lock lives on ``RowJournal``; the command loop lives on
# ``DeterminismTrace``. Neither holds the other. These are the names that would
# have to appear for that to stop being true.
COMMAND_LOG_NAMES = frozenset({"CommandLoopLog", "command_log", "_commands"})
COMMAND_LOG_MODULE = "navpy.modules.vision.sim.determinism_command_log"


def _module_tree(module) -> ast.Module:
    return ast.parse(textwrap.dedent(inspect.getsource(module)))


def _class_tree(cls) -> ast.ClassDef:
    return ast.parse(textwrap.dedent(inspect.getsource(cls))).body[0]


def _lock_acquisitions(tree: ast.AST) -> list[str]:
    """Every ``with <something>:`` and ``.acquire()`` under ``tree``, as text.

    Both spellings, because a bare ``acquire()`` evaded an earlier version of
    this check. It does not matter here WHICH object is being locked: the point
    of this test is now that one class does all the locking and the other does
    none, so any acquisition in the wrong class is a finding whatever it names.
    """
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.With):
            for item in node.items:
                found.append(ast.unparse(item.context_expr))
        elif isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr in (
                "acquire", "release"
            ):
                found.append(ast.unparse(node))
    return found


def test_the_two_locks_cannot_be_held_together_because_neither_side_can_reach_the_other() -> None:
    """The lock-ordering guarantee, as a REACHABILITY fact, not a search.

    Five review rounds went into detecting "no method holds both locks at once"
    by reading source. It was defeated by a nested function, by the public
    ``command_log`` property, by the wrong lock, by a read after an early
    return, by a bare ``acquire()``, and by eight spellings of an alias; three
    rewrites of the alias analysis each traded false alarms for blind spots, and
    the reviewer produced a fresh evasion every round.

    The analysis was the wrong tool. Every row-lock critical section needs the
    row log, the watermark and the slot period, and NOT ONE of them needs the
    command loop -- so the command loop was moved out of reach instead, and the
    question became reachability rather than spelling.

    This test covers the NAME-level half of that, which is depth one. A review
    showed why that is not the whole answer: parking the command loop on an
    object the journal already holds satisfies every check below. The value-level
    half, at any depth, is
    ``test_no_command_loop_is_reachable_from_the_journal_at_any_depth``; neither
    is sufficient alone.

    What is asserted here:

    1. ``RowJournal`` does all the row locking, and ``DeterminismTrace`` does
       none, so no call of the trace's can be holding the row lock when it
       reads the command loop.
    2. The command loop is not reachable from ``RowJournal``: its module does
       not import it, its class never names it, and its instance attributes are
       the five it sets in ``__init__``.

    What this does NOT cover: anything reached through a VALUE rather than a
    name (the walk test above), and a FOREIGN object handed in as a frame or
    sample whose getter could acquire something -- which is why the runtime
    measurement in tests/modules/vision/test_determinism_trace.py is kept.
    """
    journal_tree = _class_tree(determinism_journal.RowJournal)
    trace_tree = _class_tree(determinism_trace.DeterminismTrace)

    # 1. All the locking on one side, none on the other.
    assert _lock_acquisitions(journal_tree), (
        "RowJournal stopped taking the row lock at all"
    )
    assert _lock_acquisitions(trace_tree) == [], (
        "DeterminismTrace acquired a lock, so it can now be holding one while "
        f"it reads the command loop: {_lock_acquisitions(trace_tree)}"
    )

    # 2a. The journal's module cannot even name the command loop's class.
    imported: set[str] = set()
    for node in ast.walk(_module_tree(determinism_journal)):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
            imported.update(alias.name for alias in node.names)
    assert COMMAND_LOG_MODULE not in imported, (
        "determinism_journal imports the command log module, so an instance is "
        "constructible inside the row lock"
    )
    assert not COMMAND_LOG_NAMES & imported, (
        f"determinism_journal imports {sorted(COMMAND_LOG_NAMES & imported)}"
    )

    # 2b. And the class body never names it, by attribute or by bare name.
    named = {
        node.attr for node in ast.walk(journal_tree)
        if isinstance(node, ast.Attribute)
    } | {
        node.id for node in ast.walk(journal_tree) if isinstance(node, ast.Name)
    }
    assert not COMMAND_LOG_NAMES & named, (
        f"RowJournal names the command loop: {sorted(COMMAND_LOG_NAMES & named)}"
    )

    # 2c. Its state is exactly these five, so there is no other attribute an
    # alias could hide behind. Scanned over the WHOLE class, not just
    # ``__init__``: a method that stores what it is handed
    # (``def stash(self, thing): self._hidden = thing``) puts a field on the
    # instance that a constructor scan cannot see and that a fresh instance
    # does not carry, and a mutation doing exactly that went uncaught.
    owned = set()
    for node in ast.walk(journal_tree):
        # Both spellings: an annotated assignment is not an ast.Assign, and
        # ``self._watermark_us: int | None = None`` is one, so scanning only
        # Assign missed a field -- which this test caught on its first run.
        targets = (
            list(node.targets)
            if isinstance(node, ast.Assign)
            else [node.target] if isinstance(node, ast.AnnAssign) else []
        )
        for target in targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                owned.add(target.attr)
    assert owned == {"_period_us", "_lock", "_log", "_watermark_us", "_seal"}, (
        f"RowJournal owns unexpected state: {sorted(owned)}"
    )
    # And the language enforces the same five, so a field added down a path no
    # test drives raises instead of quietly existing. Having no ``__dict__`` at
    # all is the part that matters: it is what makes "these five and nothing
    # else" a property of the object rather than of this assertion.
    assert set(determinism_journal.RowJournal.__slots__) == {"_period_us", "_lock", "_log", "_watermark_us", "_seal"}
    journal = determinism_journal.RowJournal(5_000, 4)
    assert not hasattr(journal, "__dict__"), (
        "a RowJournal carries a __dict__, so it can hold state off its slots"
    )
    try:
        journal._commands = object()
    except AttributeError:
        pass
    else:  # pragma: no cover - only reached if __slots__ stops working
        raise AssertionError("a RowJournal accepted an attribute off its slots")


def _reachable_from(root: object, budget: int = 200_000) -> list[object]:
    """Every object reachable from ``root``, using the INTERPRETER's traversal.

    ``gc.get_referents`` is what the garbage collector itself uses to find what
    an object refers to. That matters because the previous version of this
    helper enumerated attributes and containers by hand, and a review found five
    hiding places it did not know about: a ``list`` subclass carrying an
    instance attribute (the container branch returned before looking at
    ``__dict__``), a closure cell, a name-mangled private slot, a string-form
    ``__slots__``, and a mappingproxy's contents. A hand-written walk carries a
    list of places a reference can live, and that list is never finished.

    It is NOT a complete answer, and the docs say so: get_referents "may not
    return all objects directly reachable". A weak reference is exactly that
    case -- it returns nothing for one, though the referent is alive and
    dereferencing gets it -- so weak references are followed explicitly here.
    That is a case handled, not a class of case eliminated; this is a strong
    check, not a proof.

    Three types are expanded narrowly rather than by referents, because their
    referents include ``__globals__``, and expanding that reaches the entire
    interpreter from any object and proves nothing:

    - a bound method: its ``__self__`` only;
    - a function: its closure cells, defaults, and KEYWORD-ONLY defaults -- the
      last because a review hid a command loop in a ``*, command=log`` parameter
      and this walk, which followed only ``__defaults__``, reported clean;
    - a class, module, or builtin: not expanded at all.

    That boundary is why the module-globals check below is a separate assertion
    rather than being folded in here.
    """
    seen: set[int] = set()
    found: list[object] = []
    stack = [root]
    while stack:
        obj = stack.pop()
        if id(obj) in seen:
            continue
        seen.add(id(obj))
        found.append(obj)
        assert len(found) <= budget, "object graph exceeded the walk budget"

        if isinstance(obj, MethodType):
            stack.append(obj.__self__)
            continue
        if isinstance(obj, FunctionType):
            for cell in obj.__closure__ or ():
                try:
                    stack.append(cell.cell_contents)
                except ValueError:  # an empty cell holds nothing yet
                    pass
            stack.extend(obj.__defaults__ or ())
            stack.extend((obj.__kwdefaults__ or {}).values())
            continue
        if isinstance(obj, weakref.ReferenceType):
            # get_referents returns nothing for one of these, so the referent
            # is fetched directly. It may be dead, which is a None to skip.
            referent = obj()
            if referent is not None:
                stack.append(referent)
            continue
        if isinstance(obj, (type, ModuleType, BuiltinFunctionType)):
            continue
        stack.extend(gc.get_referents(obj))
    return found


def test_no_command_loop_is_reachable_from_the_journal_at_any_depth() -> None:
    """The reachability claim, followed through VALUES rather than names.

    The structural test above is name-level, which is depth one. A review showed
    that is not the whole answer: parking the command loop on an object the
    journal already holds satisfies every check there. So this follows what the
    objects actually HOLD, using the interpreter's own traversal -- see
    ``_reachable_from`` for why it is not a hand-written attribute walk any more.

    Three separate assertions, because they cover three different things and any
    one of them alone would leave a hole a review has already walked through:

    1. a positive control, that the walk finds the command loop the FACADE holds
       -- without it, a walk that silently visited nothing would report the same
       clean answer, which is the failure mode that has cost this workstream
       seven rounds;
    2. the journal's own reachable graph, which must contain no command loop;
    3. this module's globals, which neither the walk from the journal (it stops
       at module objects) nor the import check (it reads import syntax) can see.
       An earlier docstring claimed the import check covered these. It does not:
       another module can write ``determinism_journal.stash = commands`` without
       adding an import anywhere. Each global is WALKED, not just type-checked,
       because ``stash = [commands]`` passed a check that only looked at the
       immediate value.

    Driven against a real trace after every recorder has run, because an attach
    does not have to happen in a constructor.
    """
    trace = determinism_trace.DeterminismTrace(5_000, capacity=16)

    trace.record_association(epoch=1, outcome="committed", source_s=1.0)
    trace.record_truth(epoch=1, outcome="recorded")
    trace.record_stage(epoch=1, outcome="staged")
    trace.record_discard(epoch=1, reason="unbracketable", victim=None)
    trace.record_lifecycle(epoch=1, outcome="closed", resulting_epoch=2)
    trace.command_log.note_iteration(1)
    trace.record_output(
        epoch=1,
        outcome="delivered",
        taken_at_us=trace.watermark_us(),
    )
    assert list(trace.capture().rows), "no rows recorded, so nothing was driven"

    def command_loops(objects):
        return [
            obj for obj in objects
            if isinstance(obj, determinism_command_log.CommandLoopLog)
        ]

    # 1. The control.
    assert command_loops(_reachable_from(trace)), (
        "the walk did not even find the command loop the facade holds"
    )

    # 2. The journal's graph.
    leaked = command_loops(_reachable_from(trace.journal))
    assert not leaked, (
        "a command loop is reachable from the row journal, so a row-lock "
        f"critical section can read it: {leaked}"
    )

    # 3. The module's globals, each walked to the same depth as the journal.
    stashed = [
        name for name, value in vars(determinism_journal).items()
        if command_loops(_reachable_from(value))
    ]
    assert not stashed, (
        "a command loop is reachable from the journal's module globals, where "
        f"any method can read it under the row lock: {stashed}"
    )


# The names the ATTITUDE ledger answers to on a trace, and its module.
LEDGER_NAMES = frozenset({"AdmissionLedger", "ledger", "_ledger"})
LEDGER_MODULE = "navpy.modules.vision.sim.determinism_admission_ledger"


def _imported_names(module) -> set[str]:
    imported: set[str] = set()
    for node in ast.walk(_module_tree(module)):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
            imported.update(alias.name for alias in node.names)
    return imported


def test_the_attitude_ledger_and_the_other_stores_cannot_name_each_other() -> (
    None
):
    """D2's lock rule at the NAME level: the ledger's lock is a leaf, and no
    recording can hold it with the row lock or the command log's, because
    neither side can reach the other. What a row needs from the ledger, a
    ruling, is read by the trace and crosses as a value, like the worker
    iteration. The value-level half is the walk test below.

    1. The journal's module imports no ledger, and ``RowJournal`` names none.
    2. The ledger's module imports neither the journal, the command log nor
       the trace: ``PassObserver`` reaches the command log only through what
       it is handed, typed by a Protocol.
    3. The ledger and the observer hold exactly their slots, so nothing can
       be parked on either.
    """
    from navpy.modules.vision.sim import (
        determinism_admission_ledger as ledger_module,
    )

    imported = _imported_names(determinism_journal)
    assert LEDGER_MODULE not in imported, (
        "determinism_journal imports the ATTITUDE ledger's module"
    )
    assert not LEDGER_NAMES & imported, (
        f"determinism_journal imports {sorted(LEDGER_NAMES & imported)}"
    )
    journal_tree = _class_tree(determinism_journal.RowJournal)
    named = {
        node.attr for node in ast.walk(journal_tree)
        if isinstance(node, ast.Attribute)
    } | {
        node.id for node in ast.walk(journal_tree) if isinstance(node, ast.Name)
    }
    assert not LEDGER_NAMES & named, (
        f"RowJournal names the ATTITUDE ledger: {sorted(LEDGER_NAMES & named)}"
    )

    reached = _imported_names(ledger_module)
    for forbidden in (
        COMMAND_LOG_MODULE,
        "navpy.modules.vision.sim.determinism_journal",
        "navpy.modules.vision.sim.determinism_trace",
    ):
        assert forbidden not in reached, (
            f"the ATTITUDE ledger's module imports {forbidden}"
        )
    stores = COMMAND_LOG_NAMES | {"RowJournal", "DeterminismTrace"}
    assert not stores & reached, (
        f"the ATTITUDE ledger's module imports {sorted(stores & reached)}"
    )

    assert set(ledger_module.AdmissionLedger.__slots__) == {
        "_capacity", "_lock", "_entries", "_dropped", "_observed",
        "_admitted", "_seal",
    }
    assert set(ledger_module.PassObserver.__slots__) == {
        "_ledger", "_commands",
    }
    ledger = ledger_module.AdmissionLedger(4)
    observer = ledger_module.PassObserver(
        ledger, determinism_command_log.CommandLoopLog(4)
    )
    for held in (ledger, observer):
        assert not hasattr(held, "__dict__"), (
            f"a {type(held).__name__} carries a __dict__"
        )
        try:
            held._journal = object()
        except AttributeError:
            pass
        else:  # pragma: no cover - only reached if __slots__ stops working
            raise AssertionError(
                f"a {type(held).__name__} accepted an attribute off its slots"
            )


def test_no_store_reaches_the_attitude_ledger_or_is_reached_from_it() -> None:
    """The ledger's half of the reachability claim, followed through VALUES
    after every recorder, a ruling through a real tap, a sampled pass and a
    guarded callback have run. No journal or command loop is reachable from
    the ledger or its module's globals, and no ledger from the journal, the
    command loop or the journal's globals. The control: the walk finds the
    ledger, and both stores, from the facade that holds them."""
    from navpy.modules.vehicle.admission_tap import AdmissionTap
    from navpy.modules.vehicle.inbound_router import (
        AUTOPILOT_TELEMETRY_TYPES,
        REJECT_STALE_BOOT,
        message_boot_time_ms,
    )
    from navpy.modules.vision.sim import (
        determinism_admission_ledger as ledger_module,
    )

    trace = determinism_trace.DeterminismTrace(5_000, capacity=16)
    tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)
    tap.subscribe("ATTITUDE", trace.ledger.append)
    tap.rule("ATTITUDE", SimpleNamespace(time_boot_ms=1_000), True, None)
    tap.rule(
        "ATTITUDE", SimpleNamespace(time_boot_ms=900), False, REJECT_STALE_BOOT
    )
    observer = ledger_module.PassObserver(trace.ledger, trace.command_log)
    observer.note_iteration(1)
    observer.note_command(1, None)
    trace.record_subscription(epoch=1, outcome="opened")
    trace.record_association(epoch=1, outcome="committed", source_s=1.0)
    trace.record_truth(epoch=1, outcome="recorded")
    trace.record_stage(epoch=1, outcome="staged")
    trace.record_discard(epoch=1, reason="unbracketable", victim=None)
    trace.record_output(epoch=1, outcome="delivered", taken_at_us=None)
    trace.record_lifecycle(epoch=1, outcome="closed", resulting_epoch=2)
    trace.guard_callback(lambda message: None)("message")
    trace.record_subscription(epoch=1, outcome="closed")
    assert trace.ledger.capture().entries, "no ruling reached the ledger"
    assert list(trace.capture().rows), "no rows recorded, so nothing was driven"

    ledgers = ledger_module.AdmissionLedger
    stores = (
        determinism_journal.RowJournal,
        determinism_command_log.CommandLoopLog,
    )

    def of(kinds, objects):
        return [obj for obj in objects if isinstance(obj, kinds)]

    # 1. The controls.
    from_facade = _reachable_from(trace)
    assert of(ledgers, from_facade), "the walk did not find the ledger"
    assert len(of(stores, from_facade)) == 2, "the walk missed a store"

    # 2. Nothing from the ledger, or from its module's globals.
    leaked = of(stores, _reachable_from(trace.ledger))
    assert not leaked, f"a store is reachable from the ledger: {leaked}"
    stashed = [
        name for name, value in vars(ledger_module).items()
        if of(stores, _reachable_from(value))
    ]
    assert not stashed, (
        f"a store is reachable from the ledger's module globals: {stashed}"
    )

    # 3. No ledger from the journal, the command loop, or the journal's
    # globals: the ruling and the pass sample cross as values.
    for name, store in (
        ("journal", trace.journal),
        ("command loop", trace.command_log),
    ):
        leaked = of(ledgers, _reachable_from(store))
        assert not leaked, f"the ledger is reachable from the {name}"
    stashed = [
        name for name, value in vars(determinism_journal).items()
        if of(ledgers, _reachable_from(value))
    ]
    assert not stashed, (
        f"the ledger is reachable from the journal's module globals: {stashed}"
    )


def test_the_only_thing_that_crosses_between_the_ledgers_is_a_value() -> None:
    """The worker iteration crosses as an int, and it is read before any lock.

    This is the one place a row needs something the other ledger knows. If it
    crossed as the LOG rather than as the number, the reachability the test
    above depends on would be gone, so the shape is asserted rather than
    assumed: the journal takes ``iteration`` as a parameter, and the trace reads
    it outside the journal call.
    """
    signature = inspect.signature(
        determinism_journal.RowJournal.record_output
    )
    assert "iteration" in signature.parameters
    assert signature.parameters["iteration"].annotation == "int | None"

    trace = determinism_trace.DeterminismTrace(5_000, capacity=8)
    trace.command_log.note_iteration(7)
    trace.record_output(epoch=1, outcome="delivered", taken_at_us=1_000)
    row = [
        row for row in list(trace.capture().rows)
        if row[0] == determinism_events.EVENT_OUTPUT
    ][-1]
    assert row[determinism_events.OUTPUT_WORKER_ITERATION] == 7

    # And a fault reading it latches the trace incomplete rather than raising
    # into a command path or quietly losing the row.
    class Hostile:
        def current_iteration(self):
            raise RuntimeError("the ledger is broken")

    faulted = determinism_trace.DeterminismTrace(5_000, capacity=8)
    faulted._commands = Hostile()
    faulted.record_output(epoch=1, outcome="delivered", taken_at_us=1_000)
    assert faulted.journal.capture()[1].failed is True


def test_a_recorder_that_throws_cannot_stop_the_command_loop() -> None:
    """The observer is record-only, so its faults stay on its own side."""
    executed: list = []
    logger = Mock()

    class _Hostile:
        def note_iteration(self, iteration: int) -> None:
            raise RuntimeError("observer blew up on the iteration")

        def note_command(self, iteration, command, raised=False) -> None:
            raise RuntimeError("observer blew up on the command")

    class _Works:
        def has_command_pending_or_in_flight(self) -> bool:
            return True

        def take_work(self):
            return "work"

        def execute_work(self, work):
            command = CalcData(1.0, 2.0, 3.0, None, 0.5)
            executed.append(command)
            return command

        def finish_work(self, work) -> None:
            return None

        def postprocess_job(self, work):
            return None

    stop_event = threading.Event()

    @contextlib.contextmanager
    def runtime_session():
        yield _Works()

    clock = itertools.count(0.0, 1.0)
    NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=logger,
            source_dispatch=lambda: stop_event.set() or False,
            command_loop=_Hostile(),
            monotonic_s=lambda: next(clock),
            sleep_s=lambda seconds: None,
        )
    ).run()

    # The command was formed and issued anyway, and both faults were logged.
    assert len(executed) == 1
    messages = [str(call.args[0]) for call in logger.error.call_args_list]
    assert any("command iteration" in message for message in messages)
    assert any("issued command" in message for message in messages)


def test_the_worker_swallows_a_crash_but_the_trace_does_not(
    monkeypatch,
) -> None:
    """The worker's exception guard is exactly why the row has to exist.

    It logs and carries on, so a crashing consumer is invisible from the
    outside; the ledger is the only place the lost frame shows up.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)

    def boom(detection) -> bool:
        raise RuntimeError("consumer blew up")

    source = DirectTargetPixelSource(
        vehicle,
        TARGET,
        Mock(wall_period_for_scheduler_period=lambda value: value),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=boom,
        wall_now_s=lambda: 100.0,
    )
    source.activate()
    schedule = _Schedule(vehicle, source, attitude)
    stop_event = threading.Event()
    logger = Mock()

    def source_dispatch() -> bool:
        schedule.run("T T A T")
        stop_event.set()
        return source.dispatch_available()

    @contextlib.contextmanager
    def runtime_session():
        yield SimpleNamespace(
            has_command_pending_or_in_flight=lambda: False,
            take_work=lambda: None,
        )

    clock = itertools.count(0.0, 1.0)
    NavigationCommandWorker(
        NavigationCommandWorkerPorts(
            stop_event=stop_event,
            command_event=threading.Event(),
            wall_period_s=lambda period_s: period_s,
            runtime_session=runtime_session,
            logger=logger,
            source_dispatch=source_dispatch,
            monotonic_s=lambda: next(clock),
            sleep_s=lambda seconds: None,
        )
    ).run()

    # The loop survived, and said so only in the log.
    assert logger.error.called
    assert source.metrics.delivery_exceptions == 1
    _assert_publish_slot_balances(source)


def _holds_the_source_lock(node: ast.AST) -> bool:
    return isinstance(node, ast.With) and any(
        isinstance(item.context_expr, ast.Attribute)
        and item.context_expr.attr == "_lock"
        and isinstance(item.context_expr.value, ast.Name)
        and item.context_expr.value.id == "self"
        for item in node.items
    )


def _trace_calls(node: ast.AST) -> set[tuple[int, str]]:
    """Every call on the trace: ``self._trace.x(...)`` or its local alias."""
    found = set()
    for child in ast.walk(node):
        if not isinstance(child, ast.Call) or not isinstance(
            child.func, ast.Attribute
        ):
            continue
        owner = child.func.value
        if (isinstance(owner, ast.Name) and owner.id == "trace") or (
            isinstance(owner, ast.Attribute)
            and owner.attr == "_trace"
            and isinstance(owner.value, ast.Name)
            and owner.value.id == "self"
        ):
            found.add((child.lineno, child.func.attr))
    return found


def test_every_trace_row_is_written_under_the_source_lock() -> None:
    """Row order has to be DECISION order, and one lock is what makes it so.

    The message thread and the navigation worker both record. A row appended
    outside ``_lock`` would be right in content and misplaced in sequence,
    which for a ledger is the same as wrong. A race cannot be proven by
    running it, so the structure is checked instead.
    """
    tree = ast.parse(inspect.getsource(source_module))
    every = _trace_calls(tree)
    locked: set[tuple[int, str]] = set()
    for node in ast.walk(tree):
        if _holds_the_source_lock(node):
            locked |= _trace_calls(node)

    assert every, "the invariant is vacuous if no trace call was found"
    assert every - locked == set(), (
        "trace calls outside self._lock: " f"{sorted(every - locked)}"
    )


def test_the_subscription_rows_bracket_the_subscriptions_under_the_lock(
    monkeypatch,
) -> None:
    """D3: the OPENED row once start() has subscribed every type, the CLOSED
    row before close() cancels any, both under the source lock, so the rows
    bound exactly the rulings the source was subscribed for. Checked at the
    moment each row is recorded: a wrapper around the source's trace notes
    the lock then, and the vehicle notes each subscription and cancel."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    events: list[tuple] = []

    def on_message(name, callback):
        events.append(("subscribed", name))
        return SimpleNamespace(
            cancel=lambda: events.append(("cancelled", name))
        )

    vehicle.on_message = on_message
    source = _source(vehicle, [])
    real = source._trace

    class _Bracketed:
        """The source's trace, noting the source lock at each SUBSCRIPTION
        row and forwarding everything."""

        def __getattr__(self, name):
            return getattr(real, name)

        def subscription_opened(self, epoch):
            events.append(("opened", epoch, source._lock.locked()))
            real.subscription_opened(epoch)

        def subscription_closed(self, epoch):
            events.append(("closed", epoch, source._lock.locked()))
            real.subscription_closed(epoch)

    source._trace = _Bracketed()
    source.start()
    source.close()

    assert events == [
        ("subscribed", "ATTITUDE"),
        ("subscribed", "SIM_STATE"),
        ("opened", 1, True),
        ("closed", 1, True),
        ("cancelled", "ATTITUDE"),
        ("cancelled", "SIM_STATE"),
    ]
    assert [
        row for row in source.determinism_trace.capture().rows
        if row[0] == determinism_events.EVENT_SUBSCRIPTION
    ] == [
        (determinism_events.EVENT_SUBSCRIPTION, 1,
         determinism_events.SUBSCRIPTION_OPENED, None),
        (determinism_events.EVENT_SUBSCRIPTION, 1,
         determinism_events.SUBSCRIPTION_CLOSED, None),
    ]


def test_a_frame_fenced_mid_render_names_itself_in_the_ledger(
    monkeypatch,
) -> None:
    """Rendering is the one step deliberately OUTSIDE the lock, so a leg
    boundary can land in the middle of it.

    The frame must not seed the new leg, and the row has to say WHICH frame was
    fenced: a projected frame that reached no slot is otherwise the one kind of
    loss the ledger cannot name.
    """
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    attitude = Attitude(-4.0, 100.0, 3.0)
    vehicle = _vehicle(truth_attitude=attitude, telemetry_attitude=attitude)
    source = _source(vehicle, [])
    schedule = _Schedule(vehicle, source, attitude)
    real_render = source._renderer.render
    rendered: list = []

    def render_then_restart_the_leg(associated, pose):
        frame = real_render(associated, pose)
        if frame is not None and not rendered:
            rendered.append(frame)
            # The boundary lands while this frame is still in flight.
            source.activate()
        return frame

    monkeypatch.setattr(source._renderer, "render", render_then_restart_the_leg)
    schedule.run("T T A T")

    assert rendered, "the schedule never produced a frame to fence"
    stages = [
        row
        for row in list(source.determinism_trace.capture().rows)
        if row[0] == determinism_events.EVENT_STAGE
    ]
    assert [row[2] for row in stages] == [determinism_events.STAGE_FENCED]
    assert stages[0][7] == frame_digest(rendered[0])
    # Nothing reached the publish slot, so nothing can be dispatched from it.
    assert source._state.latest is None
    assert source.metrics.projected_frames == 0
    assert source.dispatch_available() is False
    _assert_publish_slot_balances(source)
