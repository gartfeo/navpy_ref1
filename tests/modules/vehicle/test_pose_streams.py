from types import SimpleNamespace
from unittest.mock import Mock

from navpy.modules.vehicle.pose_streams import (
    ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_SIM_STATE,
    POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S,
    POSE_FRAME_ASSOCIATION_MAX_PERIODS,
    POSE_FRAME_ASSOCIATION_MAX_SKEW_S,
    POSE_STREAM_RATE_HZ,
    SCHED_LOOP_RATE_FALLBACK_HZ,
    pose_frame_association_max_skew_s,
    request_pose_streams,
    request_simulator_truth_pose_stream,
    resolve_ardupilot_scheduler_rate_hz,
    resolve_pose_stream_rate_hz,
)


def test_resolves_pose_rate_from_ardupilot_scheduler():
    vehicle = SimpleNamespace(
        get_param_or_default=Mock(return_value=80.0),
    )

    assert resolve_pose_stream_rate_hz(vehicle) == 64.0
    vehicle.get_param_or_default.assert_called_once_with(
        "SCHED_LOOP_RATE", SCHED_LOOP_RATE_FALLBACK_HZ
    )


def test_resolves_raw_ardupilot_scheduler_rate_for_navigation_loop():
    vehicle = SimpleNamespace(
        get_param_or_default=Mock(return_value=80.0),
    )

    assert resolve_ardupilot_scheduler_rate_hz(vehicle) == 80.0


def test_scheduler_rate_resolution_falls_back_for_invalid_param():
    assert (
        resolve_ardupilot_scheduler_rate_hz(SimpleNamespace())
        == SCHED_LOOP_RATE_FALLBACK_HZ
    )
    for bad in (None, True, 0.0, -1.0, float("nan"), float("inf"), "bad"):
        vehicle = SimpleNamespace(
            get_param_or_default=Mock(return_value=bad),
        )
        assert (
            resolve_ardupilot_scheduler_rate_hz(vehicle)
            == SCHED_LOOP_RATE_FALLBACK_HZ
        )


def test_pose_rate_resolution_falls_back_for_missing_or_invalid_param():
    assert resolve_pose_stream_rate_hz(SimpleNamespace()) == POSE_STREAM_RATE_HZ
    for bad in (None, 0.0, -1.0, float("nan"), float("inf"), "bad"):
        vehicle = SimpleNamespace(
            get_param_or_default=Mock(return_value=bad),
        )
        assert resolve_pose_stream_rate_hz(vehicle) == POSE_STREAM_RATE_HZ


def test_frame_pose_skew_bound_is_derived_from_pose_stream_period():
    assert POSE_FRAME_ASSOCIATION_MAX_PERIODS == 2.0
    assert POSE_FRAME_ASSOCIATION_MAX_SKEW_S == (
        POSE_FRAME_ASSOCIATION_MAX_PERIODS / POSE_STREAM_RATE_HZ
    )


def test_dynamic_frame_pose_skew_bound_tracks_scheduler_rate_and_caps_age():
    assert pose_frame_association_max_skew_s(25.0) == 0.08
    assert pose_frame_association_max_skew_s(100.0) == 0.02
    assert pose_frame_association_max_skew_s(1.0) == (
        POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S
    )
    assert pose_frame_association_max_skew_s("bad") == (
        POSE_FRAME_ASSOCIATION_MAX_SKEW_S
    )


def test_requests_attitude_and_position_at_the_pose_rate():
    vehicle = SimpleNamespace(send_command_long=Mock())

    request_pose_streams(vehicle)

    interval_us = int(round(1_000_000.0 / POSE_STREAM_RATE_HZ))
    vehicle.send_command_long.assert_any_call(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        p1=MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        p2=interval_us,
    )
    vehicle.send_command_long.assert_any_call(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        p1=MAVLINK_MSG_ID_ATTITUDE,
        p2=interval_us,
    )
    assert vehicle.send_command_long.call_count == 2


def test_requests_simulator_truth_only_when_explicitly_requested():
    vehicle = SimpleNamespace(send_command_long=Mock())

    request_simulator_truth_pose_stream(vehicle)

    interval_us = int(round(1_000_000.0 / POSE_STREAM_RATE_HZ))
    vehicle.send_command_long.assert_called_once_with(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        p1=MAVLINK_MSG_ID_SIM_STATE,
        p2=interval_us,
    )


def test_requests_simulator_truth_at_explicit_pose_rate():
    vehicle = SimpleNamespace(send_command_long=Mock())

    request_simulator_truth_pose_stream(vehicle, rate_hz=80.0)

    vehicle.send_command_long.assert_called_once_with(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        p1=MAVLINK_MSG_ID_SIM_STATE,
        p2=12_500,
    )


def test_ardupilot_46_pose_request_stays_within_scheduler_rate_limit():
    vehicle = SimpleNamespace(
        get_param_or_default=Mock(return_value=80.0),
        send_command_long=Mock(),
    )

    pose_rate_hz = resolve_pose_stream_rate_hz(vehicle)
    request_pose_streams(vehicle, rate_hz=pose_rate_hz)
    request_simulator_truth_pose_stream(vehicle, rate_hz=pose_rate_hz)

    assert ARDUPILOT_MESSAGE_RATE_SCHEDULER_FRACTION == 0.8
    assert pose_rate_hz == 64.0
    intervals = [
        call.kwargs["p2"]
        for call in vehicle.send_command_long.call_args_list
    ]
    assert intervals == [15_625, 15_625, 15_625]
    # Emulate ArduPilot 4.6's integer-ms conversion and scheduler-cap check.
    scheduler_loop_period_us = 1_000_000.0 / 80.0
    assert all(
        int(scheduler_loop_period_us / 800.0)
        <= interval_us // 1000
        for interval_us in intervals
    )


def test_noop_when_vehicle_cannot_send_command_long():
    # A vehicle/test double without send_command_long must not raise.
    request_pose_streams(SimpleNamespace())
    request_pose_streams(None)


def test_non_positive_or_non_finite_rate_is_ignored_without_raising():
    # A bad rate_hz must not raise inside Detector.start(); it is a no-op.
    for bad in (0.0, -5.0, float("nan"), float("inf")):
        vehicle = SimpleNamespace(send_command_long=Mock())
        request_pose_streams(vehicle, rate_hz=bad)
        vehicle.send_command_long.assert_not_called()


def test_per_message_failure_is_swallowed():
    vehicle = SimpleNamespace(send_command_long=Mock(side_effect=RuntimeError("link down")))
    logger = Mock()

    # Best-effort: an exception per message is logged, not raised.
    request_pose_streams(vehicle, logger)

    assert logger.warning.call_count == 2
