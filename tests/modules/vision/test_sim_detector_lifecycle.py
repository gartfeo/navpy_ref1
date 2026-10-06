"""Lifecycle, subscription, and owned-resource tests for DetectorSim."""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from navpy.exception_groups import ExceptionGroup
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vehicle.message_subscriptions import Subscription
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from navpy.modules.vision.sim.frame_generation_gate import FrameGenerationGate
from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSource
from navpy.modules.vision.sim.pose_inbox import PoseInbox
from navpy.modules.vision.sim.sim_detector_lifecycle import SimDetectorLifecycle
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker
from navpy.modules.vision.sim.sim_detector_reset import SimDetectorReset
from navpy.modules.vision.sim.sim_source_activation import SimSourceActivation
from navpy.modules.vision.sim.scheduler_cadence_lease import (
    SchedulerCadenceLease,
)
from navpy.modules.vision.sim.source_frame_coordinator import SourceFrameCoordinator


def _coordinator():
    return SourceFrameCoordinator(
        gate=FrameGenerationGate(),
        inbox=PoseInbox(4),
        publications=DetectionPublicationStore(
            source_driven=True,
            capacity=4,
        ),
        record_outcome=lambda *_args: None,
        overload_warning=lambda _capacity: None,
    )


def test_source_activation_is_side_effect_free_until_first_start():
    request_pose = Mock()
    request_truth = Mock()
    activation = SimSourceActivation(
        request_pose=request_pose,
        request_truth=request_truth,
    )

    request_pose.assert_not_called()
    request_truth.assert_not_called()
    activation.activate()
    activation.activate()

    request_pose.assert_called_once_with()
    request_truth.assert_called_once_with()
    assert activation.source_driven is True


def test_ideal_pose_source_cancels_every_subscription_after_one_raises():
    cancelled = []

    def cancel(index):
        cancelled.append(index)
        if index == 1:
            raise RuntimeError("cancel failed")

    subscriptions = [
        Subscription(lambda index=index: cancel(index))
        for index in range(4)
    ]
    source = IdealPoseSource(
        subscribe=Mock(side_effect=subscriptions),
        is_armed=lambda: True,
        error=Mock(),
        coordinator=_coordinator(),
        clock=Mock(),
        associator=Mock(),
    )
    source.start()

    with pytest.raises(ExceptionGroup, match="cancellation failed"):
        source.detach()

    assert cancelled == [0, 1, 2, 3]
    assert source.subscriptions == ()


def test_ideal_pose_source_fails_start_without_public_subscriptions():
    source = IdealPoseSource(
        subscribe=Mock(side_effect=AttributeError("on_message")),
        is_armed=lambda: True,
        error=Mock(),
        coordinator=_coordinator(),
        clock=Mock(),
        associator=Mock(),
    )

    with pytest.raises(RuntimeError, match="public subscriptions"):
        source.start()

    assert source.subscriptions == ()


def test_start_failure_rolls_back_subscriptions_worker_and_timebase():
    coordinator = _coordinator()
    activation = SimpleNamespace(source_driven=True, activate=Mock())
    pose_source = Mock()
    worker = Mock()
    worker.start.side_effect = RuntimeError("worker start failed")
    resetter = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=activation,
        pose_source=pose_source,
        worker=worker,
        resetter=resetter,
        info=Mock(),
    )

    with pytest.raises(RuntimeError, match="worker start failed"):
        lifecycle.start()

    resetter.prepare.assert_called_once_with()
    pose_source.start.assert_called_once_with()
    activation.activate.assert_called_once_with()
    pose_source.detach.assert_called_once_with()
    worker.join.assert_called_once_with(2.0)
    worker.close.assert_called_once_with()
    assert coordinator.stop_event.is_set()


def test_stop_joins_and_closes_worker_even_when_detach_raises():
    coordinator = _coordinator()
    pose_source = Mock()
    pose_source.detach.side_effect = RuntimeError("detach failed")
    worker = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=pose_source,
        worker=worker,
        resetter=Mock(),
        info=Mock(),
    )

    with pytest.raises(RuntimeError, match="detach failed"):
        lifecycle.stop()

    worker.join.assert_called_once_with(2.0)
    worker.close.assert_called_once_with()
    assert coordinator.stop_event.is_set()


def test_lifecycle_start_and_stop_are_sequentially_idempotent():
    coordinator = _coordinator()
    activation = SimpleNamespace(source_driven=False, activate=Mock())
    pose_source = Mock()
    worker = Mock()
    resetter = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=activation,
        pose_source=pose_source,
        worker=worker,
        resetter=resetter,
        info=Mock(),
    )

    lifecycle.start()
    lifecycle.start()
    lifecycle.stop()
    lifecycle.stop()

    resetter.prepare.assert_called_once_with()
    activation.activate.assert_called_once_with()
    worker.start.assert_called_once_with()
    pose_source.detach.assert_called_once_with()
    worker.join.assert_called_once_with(2.0)
    worker.close.assert_called_once_with()


def test_concurrent_lifecycle_start_prepares_and_starts_exactly_once():
    coordinator = _coordinator()
    entered = threading.Event()
    release = threading.Event()
    resetter = Mock()

    def prepare():
        entered.set()
        assert release.wait(2.0)

    resetter.prepare.side_effect = prepare
    worker = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=Mock(),
        worker=worker,
        resetter=resetter,
        info=Mock(),
    )
    errors = []

    def start():
        try:
            lifecycle.start()
        except Exception as error:
            errors.append(error)

    callers = [threading.Thread(target=start) for _ in range(2)]
    callers[0].start()
    assert entered.wait(1.0)
    callers[1].start()
    release.set()
    for caller in callers:
        caller.join(2.0)

    assert errors == []
    resetter.prepare.assert_called_once_with()
    worker.start.assert_called_once_with()
    lifecycle.stop()


def test_cleanup_retries_only_failed_steps_in_dependency_order():
    errors = [RuntimeError(f"cleanup {index}") for index in range(4)]
    coordinator = Mock()
    coordinator.stop.side_effect = [errors[0], None]
    pose_source = Mock()
    pose_source.detach.side_effect = [errors[1], None]
    worker = Mock()
    worker.join.side_effect = [errors[2], True]
    worker.close.side_effect = [errors[3], None]
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=pose_source,
        worker=worker,
        resetter=Mock(),
        info=Mock(),
    )

    with pytest.raises(ExceptionGroup) as first:
        lifecycle.stop()
    assert first.value.exceptions == tuple(errors[:3])
    worker.close.assert_not_called()

    with pytest.raises(RuntimeError) as second:
        lifecycle.stop()
    assert second.value is errors[3]

    assert lifecycle.stop() is True
    assert coordinator.stop.call_count == 2
    assert pose_source.detach.call_count == 2
    assert worker.join.call_count == 2
    assert worker.close.call_count == 2


def test_stop_timeout_defers_cadence_close_and_retries_worker_only() -> None:
    coordinator = Mock()
    pose_source = Mock()
    worker = Mock()
    worker.join.side_effect = [False, True]
    resetter = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=pose_source,
        worker=worker,
        resetter=resetter,
        info=Mock(),
    )

    assert lifecycle.stop() is False
    worker.close.assert_not_called()
    lifecycle.refresh()
    resetter.refresh.assert_not_called()

    assert lifecycle.stop() is True
    coordinator.stop.assert_called_once_with()
    pose_source.detach.assert_called_once_with()
    assert worker.join.call_count == 2
    worker.close.assert_called_once_with()


def test_keyboard_interrupt_during_start_settles_and_rolls_back() -> None:
    interrupt = KeyboardInterrupt()
    worker = Mock()
    worker.start.side_effect = interrupt
    worker.join.return_value = True
    lifecycle = SimDetectorLifecycle(
        coordinator=_coordinator(),
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=Mock(),
        worker=worker,
        resetter=Mock(),
        info=Mock(),
    )

    with pytest.raises(KeyboardInterrupt) as raised:
        lifecycle.start()

    assert raised.value is interrupt
    assert lifecycle.stop() is True


def test_polling_wait_is_interrupted_by_stop_even_at_slow_wall_cadence() -> None:
    coordinator = _coordinator()
    cadence = Mock()
    cadence.wall_period_for_scheduler_period.return_value = 1000.0
    sample = SimpleNamespace(
        location=Mock(),
        attitude=Mock(),
        time_boot_s=1.0,
        body_rates_rad_s=None,
        receipt_time_s=2.0,
        air_speed_mps=30.0,
    )

    def detect(*_args, **_kwargs):
        coordinator.stop_event.set()

    worker = SimDetectorWorker(
        coordinator=coordinator,
        polling_pose_reader=Mock(read=Mock(return_value=sample)),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(cadence, owned=False),
        scheduler_period_s=0.02,
        ideal_360=False,
        detect_pois=detect,
        record_outcome=Mock(),
    )

    started_s = time.perf_counter()
    worker.run()
    assert time.perf_counter() - started_s < 0.2


def test_info_failure_does_not_change_successful_start():
    lifecycle = SimDetectorLifecycle(
        coordinator=_coordinator(),
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=Mock(),
        worker=Mock(),
        resetter=Mock(),
        info=Mock(side_effect=RuntimeError("log failed")),
    )

    lifecycle.start()
    lifecycle.stop()


def test_reset_owner_attempts_every_clear_before_raising():
    pose_source = Mock()
    pose_source.reset_state.side_effect = RuntimeError("pose reset failed")
    poi_catalog = Mock()
    capture = Mock()
    gap = Mock()
    tracking = Mock()
    ideal_camera = Mock()
    mount = Mock()
    resetter = SimDetectorReset(
        pose_source=pose_source,
        poi_catalog=poi_catalog,
        capture_state=capture,
        gap_state=gap,
        tracking=tracking,
        camera_refresh=mount.refresh,
        ideal_camera=ideal_camera,
    )

    with pytest.raises(ExceptionGroup, match="reset failed"):
        resetter.refresh()

    poi_catalog.refresh.assert_called_once_with()
    capture.reset.assert_called_once_with()
    gap.reset.assert_called_once_with()
    tracking.reset.assert_called_once_with()
    ideal_camera.reset.assert_called_once_with()
    mount.refresh.assert_called_once_with()
    ideal_camera.prepare.assert_called_once_with()


def test_timebase_lease_closes_only_owned_fallback():
    shared = Mock()
    SchedulerCadenceLease(shared, owned=False).close()
    shared.close.assert_not_called()

    owned = Mock()
    lease = SchedulerCadenceLease(owned, owned=True)
    lease.close()
    lease.close()
    owned.close.assert_called_once_with()


def test_concurrent_worker_start_creates_exactly_one_worker_thread():
    coordinator = _coordinator()
    shared_timebase = Mock()
    worker = SimDetectorWorker(
        coordinator=coordinator,
        polling_pose_reader=Mock(),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(shared_timebase, owned=False),
        scheduler_period_s=0.02,
        ideal_360=True,
        detect_pois=Mock(),
        record_outcome=Mock(),
    )
    run_started = threading.Event()
    release = threading.Event()
    runs = []
    runs_lock = threading.Lock()

    def run_once():
        with runs_lock:
            runs.append(threading.current_thread())
        run_started.set()
        assert release.wait(2.0)

    worker.run = run_once
    callers = [threading.Thread(target=worker.start) for _ in range(2)]
    for caller in callers:
        caller.start()
    for caller in callers:
        caller.join(2.0)

    assert run_started.wait(2.0)
    assert len(runs) == 1
    release.set()
    assert worker.join(2.0)
    worker.close()
    shared_timebase.close.assert_not_called()


def test_sim_worker_failure_is_persistent_health_failure() -> None:
    coordinator = _coordinator()
    failure = RuntimeError("sim render failed")
    attempted = threading.Event()
    worker = SimDetectorWorker(
        coordinator=coordinator,
        polling_pose_reader=Mock(),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(Mock(), owned=False),
        scheduler_period_s=0.02,
        ideal_360=True,
        detect_pois=Mock(),
        record_outcome=Mock(),
    )

    def fail():
        attempted.set()
        raise failure

    worker.run = fail
    worker.start()
    assert attempted.wait(timeout=1.0)
    assert worker.join(1.0)

    for _ in range(2):
        with pytest.raises(RuntimeError) as raised:
            worker.raise_if_failed()
        assert raised.value is failure


def test_delayed_worker_cannot_run_after_failed_coordinator_stop() -> None:
    coordinator = _coordinator()
    coordinator_stop = Mock(side_effect=[RuntimeError("coordinator stop failed"), None])
    coordinator.stop = coordinator_stop
    cadence = Mock()
    worker = SimDetectorWorker(
        coordinator=coordinator,
        polling_pose_reader=Mock(),
        error=Mock(),
        cadence_lease=SchedulerCadenceLease(cadence, owned=True),
        scheduler_period_s=0.02,
        ideal_360=True,
        detect_pois=Mock(),
        record_outcome=Mock(),
    )
    worker.run = Mock()
    lifecycle = SimDetectorLifecycle(
        coordinator=coordinator,
        activation=SimpleNamespace(source_driven=False, activate=Mock()),
        pose_source=Mock(),
        worker=worker,
        resetter=Mock(),
        info=Mock(),
    )
    release = threading.Event()
    original_start = threading.Thread.start
    delayed_thread = None
    launcher = None

    def delayed_detector_start(thread):
        nonlocal delayed_thread, launcher
        if thread.name != "DetectorSimWorker":
            return original_start(thread)
        delayed_thread = thread

        def launch_later():
            assert release.wait(timeout=1.0)
            original_start(thread)

        launcher = threading.Thread(target=launch_later, daemon=True)
        original_start(launcher)
        return None

    try:
        with patch.object(threading.Thread, "start", delayed_detector_start):
            lifecycle.start()
            with pytest.raises(RuntimeError, match="coordinator stop failed"):
                lifecycle.stop()
        cadence.close.assert_called_once_with()
    finally:
        release.set()
        launcher.join(timeout=1.0)
        delayed_thread.join(timeout=1.0)

    worker.run.assert_not_called()
    assert lifecycle.stop() is True


def test_detector_construction_defers_stream_and_mount_side_effects():
    from navpy.modules.vision.sim.detector_sim import DetectorSim
    from tests.conftest import create_mock_args

    vehicle = Mock()
    vehicle.target_system = 1
    vehicle.home_location = None
    vehicle.mission_items_count = 0
    vehicle.is_armed = True
    vehicle.get_param_or_default.side_effect = lambda _name, default: default
    vehicle.on_message.side_effect = [
        Subscription(lambda: None) for _ in range(4)
    ]
    mount = Mock()
    mount.name = "ideal"
    mount.image_width = 2560
    mount.image_height = 1440
    mount.get_gimbal_data.return_value = GimbalData(
        att=Attitude(12.0, 34.0, 5.0),
        name="ideal",
    )
    shared_timebase = Mock()
    shared_timebase.wall_period_for_scheduler_period.return_value = 0.02
    logger = Mock()

    with (
        patch(
            "navpy.modules.vision.sim.sim_source_composition.request_pose_streams"
        ) as request_pose,
        patch(
            "navpy.modules.vision.sim.sim_source_composition."
            "request_simulator_truth_pose_stream"
        ) as request_truth,
    ):
        detector = DetectorSim(
            vehicle,
            mount,
            Mock(),
            logger,
            create_mock_args(),
            scheduler_cadence=shared_timebase,
            ideal_360=True,
        )
        request_pose.assert_not_called()
        request_truth.assert_not_called()
        mount.refresh.assert_not_called()
        mount.get_gimbal_data.assert_not_called()

        detector.start()
        request_pose.assert_called_once_with(
            vehicle,
            logger,
            rate_hz=40.0,
        )
        request_truth.assert_called_once_with(
            vehicle,
            logger,
            rate_hz=40.0,
        )
        mount.refresh.assert_called_once_with()
        mount.get_gimbal_data.assert_called_once_with()
        detector.stop()

    shared_timebase.close.assert_not_called()


def test_ideal_source_rate_follows_ardupilot_scheduler_capacity():
    from navpy.modules.vision.sim.sim_source_composition import (
        source_stream_rate_hz,
    )

    fast_scheduler = SimpleNamespace(
        get_param_or_default=Mock(return_value=400.0)
    )
    slow_scheduler = SimpleNamespace(
        get_param_or_default=Mock(return_value=20.0)
    )

    assert source_stream_rate_hz(fast_scheduler) == 320.0
    assert source_stream_rate_hz(slow_scheduler) == 16.0


def test_ideal_receipt_tolerance_tracks_wall_cadence_but_reorder_stays_raw():
    from navpy.modules.vision.sim.sim_detector_config import (
        SimDetectorDependencies,
        SimDetectorOptions,
    )
    from navpy.modules.vision.sim.sim_source_composition import build_source_graph

    vehicle = SimpleNamespace(
        target_system=1,
        get_param_or_default=Mock(return_value=50.0),
        attitude_sample=None,
        simulator_truth_pose=None,
        air_speed=None,
        is_armed=True,
        on_message=Mock(),
    )
    speedup = {"value": 10.0}
    cadence = Mock()
    cadence.wall_period_for_scheduler_period.side_effect = (
        lambda source_period_s: source_period_s / speedup["value"]
    )
    dependencies = SimDetectorDependencies(
        vehicle=vehicle,
        mount=Mock(),
        geo_ref=Mock(),
        logger=Mock(),
        args=SimpleNamespace(),
        zc_util=None,
        scheduler_cadence=cadence,
    )
    options = SimDetectorOptions(
        sim_assets_path=None,
        tracking_config=None,
        zoom_config=None,
        ideal_360=True,
        frame_generator_factory=Mock(),
    )

    with (
        patch(
            "navpy.modules.vision.sim.sim_source_composition.PoseFrameClock"
        ) as clock_type,
        patch(
            "navpy.modules.vision.sim.sim_source_composition.PoseAssociator"
        ) as associator_type,
    ):
        build_source_graph(dependencies, options)

    source_skew_s = 2.0 / 40.0
    assert clock_type.call_args.kwargs["reorder_tolerance_s"] == pytest.approx(
        source_skew_s
    )
    receipt_tolerance = associator_type.call_args.kwargs[
        "maximum_receipt_skew_s"
    ]
    assert callable(receipt_tolerance)
    assert receipt_tolerance() == pytest.approx(source_skew_s / 10.0)
    speedup["value"] = 0.5
    assert receipt_tolerance() == pytest.approx(source_skew_s / 0.5)
    assert cadence.wall_period_for_scheduler_period.call_count == 2
    cadence.wall_period_for_scheduler_period.assert_called_with(source_skew_s)
