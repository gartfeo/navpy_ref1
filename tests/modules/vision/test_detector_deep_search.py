import inspect
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.exception_groups import ExceptionGroup
from navpy.modules.vision.detector import Detector, TrackedObject
from navpy.modules.vision.detector_ports import (
    GimbalSourceIdentity,
    IdentitySchedulerCadence,
    NonSimulationControls,
    PollingDetectionEvents,
)
from navpy.modules.vision.frame_provider import FrameSnapshot
from navpy.modules.vision.real_detector_composition import (
    DetectorDependencies,
    DetectorModelConfig,
    RealDetectorConfig,
    build_real_detector,
)
from navpy.modules.vision.real_detector_controls import DetectorResetController
from navpy.modules.vision.real_detector_lifecycle import (
    DetectorLifecycle,
    DetectorWorkers,
)
from navpy.modules.vision.real_detector_models import build_models
from navpy.modules.vision.real_detector_resources import DetectorResources
from navpy.modules.vision.real_detector_state import (
    ConfirmationFrameStore,
    DeepSearchInbox,
    DetectionBatch,
    DetectionBatchInbox,
    DetectionResultStore,
    DetectorRunState,
    FreshnessPolicy,
    InferenceGeneration,
    LoopTiming,
    OverlayStore,
    PipelineMutationGate,
    RuntimeMetrics,
    next_loop_deadline,
)
from navpy.modules.vision.real_inference import (
    DeepSearchChannel,
    DeepSearchLoop,
    DetectionLoop,
)
from navpy.modules.vision.real_tracking import (
    TrackingBatchProcessor,
    TrackingLoop,
    TrackingModels,
    TrackingPublications,
)
from navpy.modules.vision.real_tracking_batch import (
    TrackingAssociationProcessor,
    TrackingResultPublisher,
)
from navpy.modules.vision.yolo_detector import Detection


def _track(obj_id, missed, timestamp=100.0):
    return TrackedObject(
        id=obj_id,
        cx=100.0,
        cy=100.0,
        w=40.0,
        h=20.0,
        confidence=0.8,
        class_id=2,
        age=10,
        hits=10,
        missed=missed,
        is_confirmed=True,
        timestamp=float(timestamp),
        vx=0.0,
        vy=0.0,
    )


def _deep_search_channel(now_s=100.0):
    clock = {"now": now_s}
    target_lock = SimpleNamespace(locked_id=7)
    overlays = OverlayStore()
    generation = InferenceGeneration(DetectionBatchInbox(), DeepSearchInbox())
    channel = DeepSearchChannel(
        True,
        SimpleNamespace(stale_seconds=0.5),
        target_lock,
        overlays,
        FreshnessPolicy(1.0 / 60.0),
        generation,
        LoopTiming(monotonic=lambda: clock["now"]),
    )
    return channel, target_lock, overlays, clock, generation


def _tracking_processor(
    models,
    publications,
    mapper,
    recovery,
    navigation,
    deep_search,
    metrics,
    mutation_gate,
    generation,
    use_target_lock,
):
    return TrackingBatchProcessor(
        mutation_gate,
        TrackingAssociationProcessor(
            models,
            recovery,
            deep_search,
            metrics,
            generation,
            use_target_lock,
        ),
        TrackingResultPublisher(mapper, publications, navigation),
    )


class TestDetectorDeepSearch(unittest.TestCase):
    def test_deep_search_does_not_run_without_selected_lock(self):
        channel, target_lock, _, _, _ = _deep_search_channel()
        target_lock.locked_id = None

        self.assertFalse(channel.should_run())

    def test_deep_search_does_not_run_for_fresh_selected_track(self):
        channel, _, overlays, _, _ = _deep_search_channel()
        overlays.publish([], _track(7, missed=0))

        with patch(
                "navpy.modules.vision.real_detector_state.time.time",
                return_value=100.1,
        ):
            self.assertFalse(channel.should_run())

    def test_deep_search_runs_when_selected_track_is_coasting(self):
        channel, _, overlays, _, _ = _deep_search_channel()
        overlays.publish([], _track(7, missed=3))

        with patch(
                "navpy.modules.vision.real_detector_state.time.time",
                return_value=100.1,
        ):
            self.assertTrue(channel.should_run())

    def test_deep_search_runs_when_selected_track_is_stale(self):
        channel, _, overlays, _, _ = _deep_search_channel()
        overlays.publish([], _track(7, missed=0, timestamp=99.0))

        with patch(
                "navpy.modules.vision.real_detector_state.time.time",
                return_value=100.0,
        ):
            self.assertTrue(channel.should_run())

    def test_deep_search_runs_when_selected_track_is_wrong_id(self):
        channel, _, overlays, _, _ = _deep_search_channel()
        overlays.publish([], _track(8, missed=0))

        self.assertTrue(channel.should_run())

    def test_deep_search_runs_when_selected_lock_has_no_current_track(self):
        channel, _, _, _, _ = _deep_search_channel()

        self.assertTrue(channel.should_run())

    def test_deep_search_batch_is_consumed_once(self):
        channel, _, _, _, _ = _deep_search_channel()
        det = Detection(100, 100, 20, 10, 0.2, 2)

        channel.publish(channel.reserve(), [det], frame_sequence=None)
        self.assertEqual(channel.take_for_tracking(None), [det])
        self.assertEqual(channel.take_for_tracking(None), [])

    def test_stale_deep_search_batch_is_discarded(self):
        channel, _, _, clock, _ = _deep_search_channel()
        det = Detection(100, 100, 20, 10, 0.2, 2)

        channel.publish(channel.reserve(), [det], frame_sequence=None)
        clock["now"] = 101.0
        self.assertEqual(channel.take_for_tracking(None), [])

    def test_deep_search_pixels_cannot_cross_frame_sequences(self):
        channel, _, _, clock, _ = _deep_search_channel()
        det = Detection(100, 100, 20, 10, 0.2, 2)

        channel.publish(channel.reserve(), [det], frame_sequence=41)
        clock["now"] = 100.1
        self.assertEqual(channel.take_for_tracking(42), [])
        self.assertEqual(channel.take_for_tracking(41), [det])

    def test_next_loop_deadline_recovers_after_overrun(self):
        self.assertAlmostEqual(next_loop_deadline(10.0, 0.1, 10.35), 10.4)


class TestDetectionBatchSequencing(unittest.TestCase):
    """A fresh inference differs from a coast tick even when it is empty."""

    def test_new_inference_is_a_new_batch_then_stale(self):
        inbox = DetectionBatchInbox()
        det = Detection(100, 100, 20, 10, 0.5, 2)
        association = object()

        inbox.publish([det], association, generation=0)
        batch = inbox.take_next()
        self.assertTrue(batch.is_new)
        self.assertEqual(batch.detections, (det,))
        self.assertIs(batch.association, association)

        batch = inbox.take_next()
        self.assertFalse(batch.is_new)
        self.assertEqual(batch.detections, ())
        self.assertIsNone(batch.association)

    def test_empty_inference_is_still_a_new_batch(self):
        inbox = DetectionBatchInbox()
        association = object()

        inbox.publish([], association, generation=0)
        batch = inbox.take_next()

        self.assertTrue(batch.is_new)
        self.assertEqual(batch.detections, ())
        self.assertIs(batch.association, association)

    def test_tracker_uses_detection_association_frame_not_live_frame(self):
        frame = np.full((48, 64, 3), 17, dtype=np.uint8)
        association = SimpleNamespace(
            frame=frame,
            frame_width=64,
            frame_height=48,
            frame_sequence=41,
        )
        track = _track(7, missed=0, timestamp=100.0)
        tracker = Mock()
        tracker.update.return_value = [track]
        identity = Mock()
        identity.update.return_value = [track]
        identity.stable_of.return_value = 7
        target_lock = SimpleNamespace(
            select=Mock(return_value=track),
            locked_id=7,
        )
        results = DetectionResultStore()
        overlays = OverlayStore()
        mapper = Mock()
        mapper.convert.return_value = []
        recovery = Mock()
        recovery.compute_embeddings.return_value = {}
        deep_search = Mock()
        deep_search.take_for_tracking.return_value = []
        generation = InferenceGeneration(
            DetectionBatchInbox(),
            DeepSearchInbox(),
        )
        processor = _tracking_processor(
            TrackingModels(tracker, identity, target_lock),
            TrackingPublications(results, overlays),
            mapper,
            recovery,
            Mock(),
            deep_search,
            RuntimeMetrics(),
            PipelineMutationGate(),
            generation,
            True,
        )

        processor.process(
            DetectionBatch(
                True,
                (),
                association,
                generation.reserve().generation,
            ),
            now_s=1.0,
        )

        tracker.update.assert_called_once_with([], 64, 48, frame=frame)
        mapper.convert.assert_called_once_with([track], track, association)

    def test_tracking_loop_honors_injected_batch_source(self):
        run_state = DetectorRunState()
        run_state.begin()
        inbox = Mock()

        def take_once():
            run_state.request_stop()
            return DetectionBatch(False, (), None, None)

        inbox.take_next.side_effect = take_once
        navigation = Mock()
        loop = TrackingLoop(
            run_state,
            1.0 / 60.0,
            inbox,
            DetectionResultStore(),
            Mock(),
            navigation,
            RuntimeMetrics(),
            Mock(),
            LoopTiming(monotonic=lambda: 1.0, sleep=Mock()),
        )

        loop.run()

        navigation.update.assert_called_once_with([])

    def test_facade_keeps_bounded_constructor_and_read_only_mount(self):
        self.assertEqual(
            list(inspect.signature(Detector.__init__).parameters),
            ["self", "dependencies", "config"],
        )
        detector = object.__new__(Detector)
        with self.assertRaises(AttributeError):
            detector.mount = object()

    def test_yolo_frame_is_the_frame_bound_to_pending_association(self):
        source_frame = np.full((48, 64, 3), 17, dtype=np.uint8)
        snapshot = FrameSnapshot(
            frame=source_frame,
            width=64,
            height=48,
            sequence=41,
            published_at_s=100.0,
        )
        run_state = DetectorRunState()
        run_state.begin()
        provider = Mock()
        provider.get_frame_snapshot.return_value = snapshot
        association = object()
        association_builder = Mock()
        association_builder.capture.return_value = association
        yolo = Mock()

        def detect_once(_frame):
            run_state.request_stop()
            return []

        yolo.detect.side_effect = detect_once
        inbox = DetectionBatchInbox()
        generation = InferenceGeneration(inbox, DeepSearchInbox())
        loop = DetectionLoop(
            run_state,
            1.0 / 20.0,
            provider,
            association_builder,
            yolo,
            generation,
            RuntimeMetrics(),
            Mock(),
            LoopTiming(monotonic=lambda: 1.0, sleep=Mock()),
        )

        loop.run()

        yolo_frame = yolo.detect.call_args.args[0]
        self.assertIsNot(yolo_frame, source_frame)
        np.testing.assert_array_equal(yolo_frame, source_frame)
        association_builder.capture.assert_called_once_with(snapshot, yolo_frame)
        self.assertIs(inbox.take_next().association, association)

    def test_refresh_cannot_reset_pipeline_mid_update(self):
        gate = PipelineMutationGate()
        update_entered = threading.Event()
        release_update = threading.Event()
        reset_called = threading.Event()
        tracker = Mock()

        def update(*_args, **_kwargs):
            update_entered.set()
            self.assertTrue(release_update.wait(timeout=1.0))
            return []

        tracker.update.side_effect = update
        tracker.reset.side_effect = reset_called.set
        identity = Mock()
        identity.update.return_value = []
        target_lock = Mock()
        target_lock.select.return_value = None
        target_lock.locked_id = None
        results = DetectionResultStore()
        overlays = OverlayStore()
        mapper = Mock()
        mapper.convert.return_value = []
        recovery = Mock()
        recovery.compute_embeddings.return_value = None
        deep_search = Mock()
        deep_search.take_for_tracking.return_value = []
        detection_inbox = DetectionBatchInbox()
        deep_inbox = DeepSearchInbox()
        generation = InferenceGeneration(detection_inbox, deep_inbox)
        processor = _tracking_processor(
            TrackingModels(tracker, identity, target_lock),
            TrackingPublications(results, overlays),
            mapper,
            recovery,
            Mock(),
            deep_search,
            RuntimeMetrics(),
            gate,
            generation,
            True,
        )
        run_state = DetectorRunState()
        reset = DetectorResetController(
            run_state,
            results,
            generation,
            ConfirmationFrameStore(),
            tracker,
            identity,
            target_lock,
            Mock(),
            gate,
        )
        association = SimpleNamespace(
            frame=np.zeros((12, 16, 3), dtype=np.uint8),
            frame_width=16,
            frame_height=12,
            frame_sequence=1,
        )
        process_thread = threading.Thread(
            target=processor.process,
            args=(
                DetectionBatch(
                    True,
                    (),
                    association,
                    generation.reserve().generation,
                ),
                1.0,
            ),
        )
        refresh_started = threading.Event()

        def refresh():
            refresh_started.set()
            reset.refresh()

        refresh_thread = threading.Thread(target=refresh)
        process_thread.start()
        self.assertTrue(update_entered.wait(timeout=1.0))
        refresh_thread.start()
        self.assertTrue(refresh_started.wait(timeout=1.0))
        self.assertFalse(reset_called.wait(timeout=0.05))

        release_update.set()
        process_thread.join(timeout=1.0)
        refresh_thread.join(timeout=1.0)

        self.assertFalse(process_thread.is_alive())
        self.assertFalse(refresh_thread.is_alive())
        self.assertTrue(reset_called.is_set())


class TestInferenceGenerationBarrier(unittest.TestCase):
    @staticmethod
    def _reset(generation, gate, tracker=None, target_lock=None):
        return DetectorResetController(
            DetectorRunState(),
            DetectionResultStore(),
            generation,
            ConfirmationFrameStore(),
            tracker or Mock(),
            Mock(),
            target_lock or Mock(),
            Mock(),
            gate,
        )

    def test_detection_in_flight_during_refresh_cannot_publish(self):
        run_state = DetectorRunState()
        run_state.begin()
        entered = threading.Event()
        release = threading.Event()
        snapshot = FrameSnapshot(
            frame=np.zeros((12, 16, 3), dtype=np.uint8),
            width=16,
            height=12,
            sequence=3,
            published_at_s=1.0,
        )
        provider = Mock()
        provider.get_frame_snapshot.return_value = snapshot
        detector = Mock()

        def detect(_frame):
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))
            run_state.request_stop()
            return [Detection(4, 5, 2, 3, 0.8, 1)]

        detector.detect.side_effect = detect
        inbox = DetectionBatchInbox()
        inbox.take_next()
        generation = InferenceGeneration(inbox, DeepSearchInbox())
        metrics = RuntimeMetrics()
        loop = DetectionLoop(
            run_state,
            0.05,
            provider,
            Mock(capture=Mock(return_value=object())),
            detector,
            generation,
            metrics,
            Mock(),
            LoopTiming(monotonic=lambda: 1.0, sleep=Mock()),
        )
        thread = threading.Thread(target=loop.run)
        thread.start()
        self.assertTrue(entered.wait(timeout=1.0))

        self._reset(generation, PipelineMutationGate()).refresh()
        release.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertFalse(inbox.take_next().is_new)
        self.assertEqual(metrics.snapshot()["detect_batches"], 0)

    def test_batch_taken_before_refresh_cannot_mutate_after_refresh(self):
        inbox = DetectionBatchInbox()
        generation = InferenceGeneration(inbox, DeepSearchInbox())
        association = SimpleNamespace(
            frame=np.zeros((12, 16, 3), dtype=np.uint8),
            frame_width=16,
            frame_height=12,
            frame_sequence=3,
        )
        reservation = generation.reserve()
        self.assertTrue(
            generation.publish_detection(reservation, [], association)
        )
        batch = inbox.take_next()
        gate = PipelineMutationGate()
        tracker = Mock()
        target_lock = Mock()
        target_lock.locked_id = None
        processor = _tracking_processor(
            TrackingModels(tracker, Mock(), target_lock),
            TrackingPublications(DetectionResultStore(), OverlayStore()),
            Mock(),
            Mock(),
            Mock(),
            Mock(),
            RuntimeMetrics(),
            gate,
            generation,
            True,
        )

        self._reset(generation, gate, tracker, target_lock).refresh()
        processor.process(batch, now_s=1.0)

        tracker.update.assert_not_called()

    def test_deep_search_in_flight_during_refresh_cannot_publish(self):
        run_state = DetectorRunState()
        run_state.begin()
        entered = threading.Event()
        release = threading.Event()
        provider = Mock()
        provider.get_frame_snapshot.return_value = FrameSnapshot(
            frame=np.zeros((12, 16, 3), dtype=np.uint8),
            width=16,
            height=12,
            sequence=9,
            published_at_s=1.0,
        )
        detector = Mock()

        def detect(_frame):
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))
            run_state.request_stop()
            return [Detection(4, 5, 2, 3, 0.8, 1)]

        detector.detect.side_effect = detect
        target_lock = Mock()
        target_lock.locked_id = 7
        generation = InferenceGeneration(
            DetectionBatchInbox(),
            DeepSearchInbox(),
        )
        config = SimpleNamespace(period=0.05, stale_seconds=0.5)
        channel = DeepSearchChannel(
            True,
            config,
            target_lock,
            OverlayStore(),
            FreshnessPolicy(1.0 / 60.0),
            generation,
            LoopTiming(monotonic=lambda: 1.0),
        )
        metrics = RuntimeMetrics()
        logger = Mock()
        loop = DeepSearchLoop(
            run_state,
            provider,
            detector,
            config,
            channel,
            metrics,
            logger,
            LoopTiming(monotonic=lambda: 1.0, sleep=Mock()),
        )
        thread = threading.Thread(target=loop.run)
        thread.start()
        self.assertTrue(entered.wait(timeout=1.0))

        self._reset(
            generation,
            PipelineMutationGate(),
            target_lock=target_lock,
        ).refresh()
        release.set()
        thread.join(timeout=1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(channel.take_for_tracking(9), [])
        self.assertEqual(metrics.snapshot()["deep_search_runs"], 0)
        logger.info.assert_not_called()


class TestRealDetectorFleetCapabilities(unittest.TestCase):
    def test_explicit_real_capabilities_are_forwarded_without_simulation(self):
        mount = Mock()
        mount.name = "device-name"
        mount.get_gimbal_data.return_value = SimpleNamespace(name="gimbal_0")
        ports = SimpleNamespace(
            mount=mount,
            identity=GimbalSourceIdentity(mount.get_gimbal_data),
            events=PollingDetectionEvents(),
            simulation=NonSimulationControls(),
            cadence=IdentitySchedulerCadence(),
        )
        with patch(
            "navpy.modules.vision.detector.build_real_detector",
            return_value=ports,
        ):
            detector = Detector(Mock(), Mock())

        mount.get_gimbal_data.assert_not_called()
        self.assertEqual(detector.source_name, "gimbal_0")
        self.assertFalse(detector.is_simulation)
        self.assertFalse(detector.has_source_driven_detection_events)
        self.assertFalse(detector.target_uses_source_driven_events(Mock()))
        self.assertEqual(detector.drain_detection_events(Mock()), [])
        self.assertEqual(detector.wall_period_for_scheduler_period(0.125), 0.125)
        self.assertIsNone(detector.set_sim_target(3, Mock(), location_type="vehicle"))


class TestDetectorLifecycle(unittest.TestCase):
    def _lifecycle(
            self,
            pose_streams=None,
            frame_provider=None,
            workers=None,
            resources=None,
            logger=None,
    ):
        run_state = DetectorRunState()
        provider = frame_provider or Mock()
        resources = resources or Mock()
        diagnostics = Mock()
        lifecycle = DetectorLifecycle(
            run_state,
            provider,
            workers or DetectorWorkers(Mock(), Mock(), None),
            resources,
            pose_streams or Mock(),
            diagnostics,
            logger or Mock(),
        )
        return lifecycle, run_state, provider, resources, diagnostics

    def test_pose_request_failure_rolls_back_running_state(self):
        pose_streams = Mock()
        pose_streams.request.side_effect = RuntimeError("request failed")
        lifecycle, run_state, provider, resources, diagnostics = (
            self._lifecycle(pose_streams=pose_streams)
        )

        with self.assertRaisesRegex(RuntimeError, "request failed"):
            lifecycle.start()

        self.assertFalse(run_state.is_running)
        provider.start.assert_not_called()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_baseexception_during_start_settles_phase_before_reraise(self):
        interrupt = KeyboardInterrupt()
        pose_streams = Mock()
        pose_streams.request.side_effect = interrupt
        lifecycle, run_state, provider, resources, diagnostics = (
            self._lifecycle(pose_streams=pose_streams)
        )

        with self.assertRaises(KeyboardInterrupt) as raised:
            lifecycle.start()
        self.assertIs(raised.exception, interrupt)
        self.assertNotEqual(lifecycle._phase, "starting")
        self.assertFalse(run_state.is_running)

        lifecycle.stop()
        provider.stop.assert_not_called()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_frame_start_failure_rolls_back_and_stops_partial_provider(self):
        provider = Mock()
        provider.start.side_effect = RuntimeError("capture failed")
        lifecycle, run_state, provider, resources, _ = self._lifecycle(
            frame_provider=provider,
        )

        with self.assertRaisesRegex(RuntimeError, "capture failed"):
            lifecycle.start()

        self.assertFalse(run_state.is_running)
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()

    def test_stop_closes_resources_once_and_is_idempotent(self):
        lifecycle, run_state, provider, resources, diagnostics = self._lifecycle()
        lifecycle.start()

        lifecycle.stop()
        lifecycle.stop()

        self.assertFalse(run_state.is_running)
        self.assertTrue(run_state.is_stopped)
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once()
        diagnostics.close_ui.assert_called_once_with()

    def test_stop_retries_incomplete_frame_provider_before_resources(self):
        provider = Mock()
        provider.stop.side_effect = [False, True]
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
        )
        lifecycle.start()

        with self.assertRaisesRegex(TimeoutError, "frame provider did not stop"):
            lifecycle.stop()
        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()
        self.assertFalse(run_state.is_stopped)

        lifecycle.stop()
        self.assertEqual(provider.stop.call_count, 2)
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()
        self.assertTrue(run_state.is_stopped)

    def test_start_rollback_retains_incomplete_frame_provider_for_stop(self):
        start_error = RuntimeError("capture start failed")
        provider = Mock()
        provider.start.side_effect = start_error
        provider.stop.side_effect = [False, True]
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
        )

        with self.assertRaises(ExceptionGroup) as raised:
            lifecycle.start()
        self.assertIs(raised.exception.exceptions[0], start_error)
        self.assertIsInstance(raised.exception.exceptions[1], TimeoutError)
        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()
        with self.assertRaisesRegex(RuntimeError, "construct a new Detector"):
            lifecycle.start()

        lifecycle.stop()
        self.assertEqual(provider.stop.call_count, 2)
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()
        self.assertTrue(run_state.is_stopped)

    def test_stop_times_out_and_retains_dependencies_for_worker_retry(self):
        entered = threading.Event()
        release = threading.Event()
        detection = Mock()

        def block():
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))

        detection.run.side_effect = block
        logger = Mock()
        lifecycle, run_state, provider, resources, diagnostics = self._lifecycle(
            workers=DetectorWorkers(detection, Mock(), None),
            logger=logger,
        )
        lifecycle.start()
        self.assertTrue(entered.wait(timeout=1.0))

        with patch(
                "navpy.modules.vision.real_detector_lifecycle."
                "WORKER_JOIN_WARNING_S",
                0.01,
        ):
            release_timer = threading.Timer(0.2, release.set)
            release_timer.start()
            started_s = time.perf_counter()
            with self.assertRaisesRegex(TimeoutError, "did not stop"):
                lifecycle.stop()
            elapsed_s = time.perf_counter() - started_s
            self.assertLess(elapsed_s, 0.15)
            resources.close.assert_not_called()
            diagnostics.close_ui.assert_not_called()
            provider.stop.assert_not_called()
            self.assertFalse(run_state.is_stopped)
            release_timer.join(timeout=1.0)

        lifecycle.stop()
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once()
        diagnostics.close_ui.assert_called_once_with()

    def test_stop_retains_worker_when_liveness_probe_fails(self):
        status_error = RuntimeError("thread status unavailable")

        class FlakyThreadStatus:
            def __init__(self):
                self.status_calls = 0
                self.join_timeouts = []

            def join(self, timeout=None):
                self.join_timeouts.append(timeout)

            def is_alive(self):
                self.status_calls += 1
                if self.status_calls == 1:
                    raise status_error
                return False

        retained = FlakyThreadStatus()
        lifecycle, run_state, provider, resources, diagnostics = self._lifecycle()
        run_state.begin()
        lifecycle._phase = "running"
        launch = Mock()
        launch.cancel_before_commit.return_value = False
        lifecycle._threads = (
            SimpleNamespace(name="detect", thread=retained, launch=launch),
        )  # type: ignore[assignment]

        with self.assertRaises(RuntimeError) as raised:
            lifecycle.stop()
        self.assertIs(raised.exception, status_error)
        provider.stop.assert_not_called()
        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()
        self.assertIs(lifecycle._threads[0].thread, retained)
        self.assertEqual(retained.join_timeouts, [])

        lifecycle.stop()
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_concurrent_start_waits_until_first_start_is_running(self):
        provider_entered = threading.Event()
        release_provider = threading.Event()
        second_returned = threading.Event()
        provider = Mock()
        start_errors = []

        def start_provider():
            provider_entered.set()
            if not release_provider.wait(timeout=1.0):
                raise RuntimeError("provider release timed out")

        provider.start.side_effect = start_provider
        lifecycle, _, _, _, _ = self._lifecycle(frame_provider=provider)

        def first_start():
            try:
                lifecycle.start()
            except Exception as error:
                start_errors.append(error)

        def second_start():
            try:
                lifecycle.start()
            except Exception as error:
                start_errors.append(error)
            finally:
                second_returned.set()

        first_thread = threading.Thread(target=first_start)
        second_thread = threading.Thread(target=second_start)
        first_thread.start()
        self.assertTrue(provider_entered.wait(timeout=1.0))
        second_thread.start()

        self.assertFalse(second_returned.wait(timeout=0.05))
        provider.start.assert_called_once_with()
        release_provider.set()
        first_thread.join(timeout=1.0)
        second_thread.join(timeout=1.0)

        self.assertFalse(first_thread.is_alive())
        self.assertFalse(second_thread.is_alive())
        self.assertTrue(second_returned.is_set())
        self.assertEqual(start_errors, [])
        provider.start.assert_called_once_with()
        lifecycle.stop()

    def test_partial_thread_start_timeout_retains_dependencies_for_stop_retry(self):
        entered = threading.Event()
        release = threading.Event()
        pose_streams = Mock()
        provider = Mock()
        logger = Mock()
        detection = Mock()

        def block():
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))

        detection.run.side_effect = block
        lifecycle, run_state, _, resources, _ = self._lifecycle(
            pose_streams=pose_streams,
            frame_provider=provider,
            workers=DetectorWorkers(detection, Mock(), None),
            logger=logger,
        )
        original_start = threading.Thread.start
        start_count = 0

        def fail_second_start(thread):
            nonlocal start_count
            start_count += 1
            if start_count == 2:
                raise RuntimeError("second worker failed to start")
            return original_start(thread)

        release_timer = threading.Timer(0.2, release.set)
        release_timer.start()
        with patch(
                "navpy.modules.vision.real_detector_lifecycle."
                "WORKER_JOIN_WARNING_S",
                0.01,
        ), patch.object(threading.Thread, "start", fail_second_start):
            started_s = time.perf_counter()
            with self.assertRaises(ExceptionGroup) as raised:
                lifecycle.start()
            self.assertLess(time.perf_counter() - started_s, 0.15)
            self.assertRegex(
                str(raised.exception.exceptions[0]),
                "second worker failed to start",
            )
            self.assertIsInstance(raised.exception.exceptions[1], TimeoutError)
            provider.stop.assert_not_called()
            resources.close.assert_not_called()
            release_timer.join(timeout=1.0)

        self.assertTrue(entered.is_set())
        self.assertFalse(run_state.is_running)
        with self.assertRaisesRegex(RuntimeError, "construct a new Detector"):
            lifecycle.start()

        lifecycle.stop()

        self.assertEqual(pose_streams.request.call_count, 1)
        self.assertEqual(provider.start.call_count, 1)
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once()

    def test_start_failure_after_native_launch_retains_worker_for_stop_retry(self):
        entered = threading.Event()
        release = threading.Event()
        provider = Mock()
        detection = Mock()

        def block():
            entered.set()
            self.assertTrue(release.wait(timeout=1.0))

        detection.run.side_effect = block
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
            workers=DetectorWorkers(detection, Mock(), None),
        )
        original_start = threading.Thread.start

        def launch_then_fail(thread):
            original_start(thread)
            raise RuntimeError("thread launch acknowledgement failed")

        release_timer = threading.Timer(0.2, release.set)
        release_timer.start()
        with patch(
                "navpy.modules.vision.real_detector_lifecycle."
                "WORKER_JOIN_WARNING_S",
                0.01,
        ), patch.object(threading.Thread, "start", launch_then_fail):
            with self.assertRaises(ExceptionGroup) as raised:
                lifecycle.start()

        self.assertIsInstance(raised.exception.exceptions[0], RuntimeError)
        self.assertIsInstance(raised.exception.exceptions[1], TimeoutError)
        self.assertTrue(entered.is_set())
        self.assertFalse(run_state.is_running)
        provider.stop.assert_not_called()
        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()
        release_timer.join(timeout=1.0)

        lifecycle.stop()

        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_delayed_worker_cannot_run_after_failed_stop_signal_cleanup(self):
        release = threading.Event()
        provider = Mock()
        detection = Mock()
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
            workers=DetectorWorkers(detection, Mock(), None),
        )
        stop_error = RuntimeError("stop signal failed")
        run_state.request_stop = Mock(side_effect=[stop_error, None])
        original_start = threading.Thread.start
        delayed_thread = None
        launcher = None
        worker_start_count = 0

        def delay_first_worker(thread):
            nonlocal delayed_thread, launcher, worker_start_count
            worker_start_count += 1
            if worker_start_count != 1:
                return original_start(thread)
            delayed_thread = thread

            def launch_later():
                self.assertTrue(release.wait(timeout=1.0))
                original_start(thread)

            launcher = threading.Thread(target=launch_later, daemon=True)
            original_start(launcher)
            return None

        try:
            with patch.object(threading.Thread, "start", delay_first_worker):
                lifecycle.start()
                with self.assertRaises(RuntimeError) as raised:
                    lifecycle.stop()
                self.assertIs(raised.exception, stop_error)
            resources.close.assert_called_once_with()
            diagnostics.close_ui.assert_called_once_with()
        finally:
            release.set()
            launcher.join(timeout=1.0)
            delayed_thread.join(timeout=1.0)

        detection.run.assert_not_called()
        self.assertTrue(lifecycle.stop())

    def test_logger_failures_do_not_change_lifecycle_outcome(self):
        logger = Mock()
        logger.info.side_effect = RuntimeError("info failed")
        logger.warning.side_effect = RuntimeError("warning failed")
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            logger=logger,
        )

        lifecycle.start()
        lifecycle.stop()

        self.assertTrue(run_state.is_stopped)
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_worker_failure_is_persistent_health_failure(self):
        attempted = threading.Event()
        failure = RuntimeError("detection worker failed")
        detection = Mock()

        def fail():
            attempted.set()
            raise failure

        detection.run.side_effect = fail
        lifecycle, _, _, _, _ = self._lifecycle(
            workers=DetectorWorkers(detection, Mock(), None),
        )
        lifecycle.start()
        self.assertTrue(attempted.wait(timeout=1.0))

        for _ in range(2):
            with self.assertRaises(RuntimeError) as raised:
                lifecycle.raise_if_failed()
            self.assertIs(raised.exception, failure)
        lifecycle.stop()

    def test_stop_retries_provider_before_terminal_cleanup_failures(self):
        provider_error = RuntimeError("provider stop failed")
        resource_error = RuntimeError("resource close failed")
        diagnostics_error = RuntimeError("diagnostics close failed")
        provider = Mock()
        provider.stop.side_effect = [provider_error, True]
        resources = Mock()
        resources.close.side_effect = [resource_error, None]
        lifecycle, run_state, _, _, diagnostics = self._lifecycle(
            frame_provider=provider,
            resources=resources,
        )
        diagnostics.close_ui.side_effect = [diagnostics_error, None]

        with self.assertRaises(RuntimeError) as first:
            lifecycle.stop()
        self.assertIs(first.exception, provider_error)
        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()

        with self.assertRaises(ExceptionGroup) as raised:
            lifecycle.stop()
        self.assertEqual(
            raised.exception.exceptions,
            (resource_error, diagnostics_error),
        )
        lifecycle.stop()

        self.assertEqual(resources.close.call_count, 2)
        self.assertEqual(diagnostics.close_ui.call_count, 2)
        self.assertTrue(run_state.is_stopped)
        self.assertEqual(provider.stop.call_count, 2)

    def test_stop_retries_transient_stop_signal_without_reclosing_resources(self):
        stop_error = RuntimeError("stop signal failed")
        lifecycle, run_state, provider, resources, diagnostics = self._lifecycle()
        run_state.request_stop = Mock(side_effect=[stop_error, None])

        with self.assertRaises(RuntimeError) as first:
            lifecycle.stop()
        self.assertIs(first.exception, stop_error)
        self.assertTrue(lifecycle.is_quiescent)

        self.assertTrue(lifecycle.stop())
        self.assertEqual(run_state.request_stop.call_count, 2)
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_failed_start_rollback_releases_waiters_and_forbids_retry(self):
        start_error = RuntimeError("provider start failed")
        rollback_error = RuntimeError("provider rollback failed")
        provider = Mock()
        provider.start.side_effect = start_error
        provider.stop.side_effect = [rollback_error, True]
        lifecycle, run_state, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider
        )

        with self.assertRaises(ExceptionGroup) as raised:
            lifecycle.start()
        self.assertEqual(
            raised.exception.exceptions,
            (start_error, rollback_error),
        )
        with self.assertRaisesRegex(RuntimeError, "construct a new Detector"):
            lifecycle.start()

        resources.close.assert_not_called()
        diagnostics.close_ui.assert_not_called()
        lifecycle.stop()

        self.assertEqual(provider.stop.call_count, 2)
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()
        self.assertTrue(run_state.is_stopped)

    def test_complete_start_rollback_does_not_replay_reported_errors(self):
        start_error = RuntimeError("capture start failed")
        cleanup_error = RuntimeError("worker join was interrupted")
        provider = Mock()
        provider.start.side_effect = start_error
        lifecycle, _, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
        )
        completed_workers = SimpleNamespace(
            is_complete=True,
            alive=(),
            errors=(cleanup_error,),
        )

        with patch(
            "navpy.modules.vision.real_detector_stop_transaction."
            "wait_for_worker_quiescence",
            return_value=completed_workers,
        ):
            with self.assertRaises(ExceptionGroup) as raised:
                lifecycle.start()

        self.assertEqual(
            raised.exception.exceptions,
            (start_error, cleanup_error),
        )
        self.assertTrue(lifecycle.stop())
        self.assertTrue(lifecycle.stop())
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()

    def test_concurrent_stop_runs_cleanup_exactly_once(self):
        entered = threading.Event()
        release = threading.Event()
        provider = Mock()

        def stop_provider():
            entered.set()
            self.assertTrue(release.wait(2.0))

        provider.stop.side_effect = stop_provider
        lifecycle, _, _, resources, diagnostics = self._lifecycle(
            frame_provider=provider,
        )
        lifecycle.start()
        errors = []

        def stop():
            try:
                lifecycle.stop()
            except Exception as error:
                errors.append(error)

        callers = [threading.Thread(target=stop) for _ in range(2)]
        callers[0].start()
        self.assertTrue(entered.wait(1.0))
        callers[1].start()
        release.set()
        for caller in callers:
            caller.join(2.0)

        self.assertEqual(errors, [])
        provider.stop.assert_called_once_with()
        resources.close.assert_called_once_with()
        diagnostics.close_ui.assert_called_once_with()


class TestDetectorCompositionTransaction(unittest.TestCase):
    def test_resource_cleanup_retries_only_failed_model_action(self):
        close_error = RuntimeError("tracker close failed")
        tracker = Mock()
        tracker.close.side_effect = [close_error, None]
        appearance = Mock()
        yolo = Mock()
        deep_search = Mock()
        confirmation_frames = Mock()
        resources = DetectorResources(
            tracker,
            appearance,
            yolo,
            deep_search,
            confirmation_frames,
        )

        with self.assertRaises(RuntimeError) as first:
            resources.close()
        self.assertIs(first.exception, close_error)
        resources.close()

        self.assertEqual(tracker.close.call_count, 2)
        appearance.close.assert_called_once_with()
        yolo.close.assert_called_once_with()
        deep_search.close.assert_called_once_with()
        confirmation_frames.clear.assert_called_once_with()

    def test_appearance_start_failure_closes_raw_embedder_once(self):
        failure = RuntimeError("appearance worker start failed")
        yolo = Mock(device="cpu")
        tracker = Mock()
        embedder = Mock()
        config = RealDetectorConfig(
            model=DetectorModelConfig(
                "model.pt",
                appearance={"enabled": True},
            ),
        )

        with patch(
                "navpy.modules.vision.real_detector_models.YoloDetector",
                return_value=yolo,
        ), patch(
                "navpy.modules.vision.real_detector_models.create_tracker_backend",
                return_value=tracker,
        ), patch(
                "navpy.modules.vision.real_detector_models.create_appearance_embedder",
                return_value=embedder,
        ), patch(
                "navpy.modules.vision.real_detector_models.deep_search_config_from_settings",
                return_value=None,
        ), patch(
                "navpy.modules.vision.appearance_worker.threading.Thread.start",
                side_effect=failure,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "appearance worker start failed",
            ):
                build_models(Mock(), config, RuntimeMetrics())

        embedder.close.assert_called_once_with()
        tracker.close.assert_called_once_with()
        yolo.close.assert_called_once_with()

    def test_model_build_failure_closes_partial_models_and_keeps_error(self):
        failure = RuntimeError("deep-search construction failed")
        cleanup_failure = RuntimeError("tracker close failed")
        yolo = Mock(device="cpu")
        tracker = Mock()
        tracker.close.side_effect = cleanup_failure
        appearance = Mock()
        config = RealDetectorConfig(
            model=DetectorModelConfig(
                "model.pt",
                appearance={"enabled": True},
                deep_search={"enabled": True},
            ),
        )
        dependencies = DetectorDependencies(Mock(), Mock(), Mock())

        with patch(
                "navpy.modules.vision.real_detector_models.YoloDetector",
                return_value=yolo,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "create_tracker_backend",
                return_value=tracker,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "create_appearance_embedder",
                return_value=object(),
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "AsyncAppearanceEmbedder",
                return_value=appearance,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "deep_search_config_from_settings",
                return_value=SimpleNamespace(),
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "DeepSearchDetector",
                side_effect=failure,
        ):
            with self.assertRaises(ExceptionGroup) as raised:
                build_real_detector(dependencies, config)

        self.assertEqual(
            raised.exception.exceptions,
            (failure, cleanup_failure),
        )
        tracker.close.assert_called_once_with()
        appearance.close.assert_called_once_with()
        yolo.close.assert_called_once_with()

    def test_downstream_failure_closes_all_built_models_once(self):
        failure = RuntimeError("frame provider composition failed")
        cleanup_failure = RuntimeError("tracker close failed")
        yolo = Mock(device="cpu")
        tracker = Mock()
        tracker.close.side_effect = cleanup_failure
        appearance = Mock()
        deep_search = Mock()
        config = RealDetectorConfig(
            model=DetectorModelConfig(
                "model.pt",
                appearance={"enabled": True},
                deep_search={"enabled": True},
            ),
        )
        dependencies = DetectorDependencies(Mock(), Mock(), Mock())

        with patch(
                "navpy.modules.vision.real_detector_models.YoloDetector",
                return_value=yolo,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "create_tracker_backend",
                return_value=tracker,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "create_appearance_embedder",
                return_value=object(),
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "AsyncAppearanceEmbedder",
                return_value=appearance,
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "deep_search_config_from_settings",
                return_value=SimpleNamespace(),
        ), patch(
                "navpy.modules.vision.real_detector_models."
                "DeepSearchDetector",
                return_value=deep_search,
        ), patch(
                "navpy.modules.vision.real_detector_runtime_composition.FrameProvider",
                side_effect=failure,
        ):
            with self.assertRaises(ExceptionGroup) as raised:
                build_real_detector(dependencies, config)

        self.assertEqual(
            raised.exception.exceptions,
            (failure, cleanup_failure),
        )
        tracker.close.assert_called_once_with()
        appearance.close.assert_called_once_with()
        yolo.close.assert_called_once_with()
        deep_search.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
