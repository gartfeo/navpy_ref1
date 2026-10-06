"""Safety regressions for gimbal session command transactions."""

from __future__ import annotations

from threading import Event, Thread
from unittest.mock import MagicMock, Mock

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTrackerConfig,
    TrackingState,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from tests.detection_factory import make_detected_target


def _navigation() -> tuple[GimbalNavigation, MagicMock, MagicMock]:
    gimbal = MagicMock()
    mount = MagicMock()
    mount.name = "transaction_mount"
    mount.gimbal = gimbal
    mount.get_gimbal_data.return_value = GimbalData(
        att=Attitude(-20.0, 75.0, 0.0)
    )
    navigation = GimbalNavigation(
        mount,
        Mock(),
        tracking=GimbalTrackingSetup(GimbalRateTrackerConfig()),
        neutral_pitch_deg=-20.0,
    )
    return navigation, mount, gimbal


def _target(
    source_time: float | None = None,
    source_now: float = 10.0,
    x_error: float = 1060.0,
) -> DetectedObject:
    return make_detected_target(
        obj_id=7,
        x_error=x_error,
        y_error=540.0,
        reference_height_m=2.0,
        k=np.array(
            [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
        ),
        g_data=GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        uas_att=Attitude(0.0, 0.0, 0.0),
        timestamp=source_time,
        timestamp_now_s=lambda: source_now,
    )


def test_stop_start_waits_for_inflight_rate_actuation() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    rate_entered = Event()
    release_rate = Event()
    transition_started = Event()
    transition_done = Event()
    commands: list[tuple[float, float]] = []
    failures: list[BaseException] = []

    def blocking_rate(yaw_rate: float, pitch_rate: float) -> None:
        commands.append((yaw_rate, pitch_rate))
        if len(commands) == 1:
            rate_entered.set()
            if not release_rate.wait(2.0):
                raise TimeoutError("test did not release rate command")

    def run_update() -> None:
        try:
            navigation.update(_target(source_time=10.0))
        except BaseException as exc:  # pragma: no cover - reported below
            failures.append(exc)

    def replace_session() -> None:
        transition_started.set()
        try:
            navigation.stop_tracking(to_neutral=False)
            navigation.start_tracking(99)
        except BaseException as exc:  # pragma: no cover - reported below
            failures.append(exc)
        finally:
            transition_done.set()

    gimbal.set_rate.side_effect = blocking_rate
    update_thread = Thread(target=run_update)
    transition_thread = Thread(target=replace_session)
    update_thread.start()
    assert rate_entered.wait(2.0)
    transition_thread.start()
    assert transition_started.wait(2.0)
    assert not transition_done.wait(0.05)
    release_rate.set()
    update_thread.join(2.0)
    transition_thread.join(2.0)

    assert not failures
    assert not update_thread.is_alive()
    assert not transition_thread.is_alive()
    assert navigation.tracking_obj_id == 99
    assert commands[-1] == (0.0, 0.0)


def test_first_missing_observation_stops_rate_without_attitude_command() -> None:
    navigation, mount, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0))
    mount.reset_mock()
    gimbal.reset_mock()

    navigation.update(None, now=10.1)

    gimbal.set_rate.assert_called_once_with(0.0, 0.0)
    gimbal.set_motion_mode.assert_not_called()
    gimbal.set_att.assert_not_called()
    mount.get_gimbal_data.assert_not_called()
    assert navigation.status.detection.holding


def test_duplicate_source_sample_does_not_advance_loss() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0, source_now=10.0))
    gimbal.reset_mock()

    navigation.update(_target(source_time=10.0, source_now=13.0))

    gimbal.set_rate.assert_not_called()
    gimbal.set_motion_mode.assert_not_called()
    gimbal.set_att.assert_not_called()
    assert navigation.status.detection.last_track_time == 10.0
    assert not navigation.status.detection.recentered


def test_missing_source_timestamp_is_loss_not_fabricated_measurement() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0, source_now=10.0))
    gimbal.reset_mock()

    navigation.update(_target(source_time=float("nan"), source_now=10.1))

    gimbal.set_rate.assert_called_once_with(0.0, 0.0)
    assert navigation.status.detection.last_track_time == 10.0


def test_recentre_failure_is_retried_next_loss_tick() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0, source_now=10.0))
    gimbal.set_att.side_effect = RuntimeError("link down")

    navigation.update(None, now=12.5)

    assert not navigation.status.detection.recentered
    gimbal.set_att.side_effect = None
    navigation.update(None, now=12.6)
    assert navigation.status.detection.recentered


def test_failed_stop_keeps_session_retryable_and_resets_tracker() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0))
    gimbal.set_rate.side_effect = RuntimeError("link down")

    with pytest.raises(RuntimeError, match="link down"):
        navigation.stop_tracking(to_neutral=False)

    assert navigation.tracking_obj_id == 7
    assert navigation.rate_result.state is TrackingState.IDLE
    gimbal.set_rate.side_effect = None
    navigation.stop_tracking(to_neutral=False)
    assert navigation.tracking_obj_id is None


def test_failed_rearm_preserves_prior_session_and_tracker_state() -> None:
    navigation, _, gimbal = _navigation()
    navigation.start_tracking(7)
    navigation.update(_target(source_time=10.0))
    prior_result = navigation.rate_result
    prior_generation = navigation.status.generation
    gimbal.set_motion_mode.side_effect = RuntimeError("link down")

    with pytest.raises(RuntimeError, match="link down"):
        navigation.start_tracking(99)

    assert navigation.tracking_obj_id == 7
    assert navigation.status.generation == prior_generation
    assert navigation.rate_result == prior_result
