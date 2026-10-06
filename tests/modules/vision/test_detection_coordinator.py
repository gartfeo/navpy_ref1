"""Tests for DetectionCoordinator."""
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

from navpy.exception_groups import ExceptionGroup
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.detector_lifecycle_fleet import DetectorLifecycleFleet
from navpy.modules.vision.detector_abc import DetectorAbc
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.poi_priority import select_most_centered_poi
from navpy.modules.vision.tracking_command_router import TrackingCommandRouter
from navpy.modules.vision.zoom_control_router import ZoomControlRouter
from tests.detection_factory import make_detected_poi


_DETECTOR_CAPABILITIES = (
    "mounts",
    "source_name",
    "is_simulation",
    "is_detection_armed",
    "loss_hold_sec",
    "is_geo_armed",
    "is_zoom_stable",
    "is_quiescent",
    "has_source_driven_detection_events",
    "get_zoom_result",
    "get_detect_data",
    "drain_detection_events",
    "open_detection_event_lease",
    "poi_uses_source_driven_events",
    "wall_period_for_scheduler_period",
    "start",
    "stop",
    "refresh",
    "set_sim_poi",
    "start_tracking",
    "stop_tracking",
    "start_geo_tracking",
    "update_geo",
    "prepare_geo_acquisition",
    "stop_geo_tracking",
    "set_zoom_size_demand",
    "freeze_final_approach_zoom_at_min",
)


def _detector_double(
    pois=None,
    *,
    is_simulation=True,
    primary_poi=None,
    source_name="",
):
    """Build a test double that explicitly supplies every fleet capability."""
    detector = Mock(spec_set=_DETECTOR_CAPABILITIES)
    detector.mounts = []
    detector.source_name = source_name
    detector.is_simulation = is_simulation
    detector.is_detection_armed = False
    detector.loss_hold_sec = None
    detector.is_geo_armed = False
    detector.is_zoom_stable = True
    detector.is_quiescent = False
    detector.has_source_driven_detection_events = False
    detector.get_zoom_result.return_value = None
    detector.get_detect_data.return_value = DetectResponse(
        pois or [], primary_poi=primary_poi,
    )
    detector.drain_detection_events.return_value = []
    detector.open_detection_event_lease.return_value = None
    detector.poi_uses_source_driven_events.return_value = False
    detector.wall_period_for_scheduler_period.side_effect = lambda period: period
    detector.stop.return_value = True
    return detector


class TestDetectionCoordinator(unittest.TestCase):
    """Tests for DetectionCoordinator class."""

    def setUp(self):
        self.logger = Mock()

    def _create_mock_detector(self, pois=None, is_sim=True, primary_poi=None):
        """Create a mock detector with given POIs."""
        return _detector_double(
            pois,
            is_simulation=is_sim,
            primary_poi=primary_poi,
        )

    def _create_mock_poi(
        self,
        obj_id=1,
        x_error=100,
        y_error=100,
        source_name="gimbal_0",
        *,
        timestamp=None,
        supports_confirmation_frame=True,
    ):
        """Create a real grouped POI at the requested source pixel."""
        return make_detected_poi(
            obj_id=obj_id,
            x_error=x_error,
            y_error=y_error,
            k=[
                [1000.0, 0.0, 1000.0],
                [0.0, 1000.0, 500.0],
                [0.0, 0.0, 1.0],
            ],
            g_data=GimbalData(
                att=Attitude(0.0, 0.0, 0.0),
                name=source_name,
            ),
            timestamp=timestamp,
            supports_confirmation_frame=supports_confirmation_frame,
        )

    def test_empty_detectors_list(self):
        """Coordinator works with empty detector list."""
        coordinator = DetectionCoordinator([], self.logger)

        self.assertEqual(coordinator.detectors, [])
        self.assertEqual(coordinator.mounts, [])
        self.assertFalse(coordinator.is_simulation)

    def test_single_detector(self):
        """Coordinator works with single detector."""
        poi = self._create_mock_poi(obj_id=1)
        detector = self._create_mock_detector([poi], primary_poi=poi)

        coordinator = DetectionCoordinator([detector], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())
        self.assertEqual(len(resp.detected_pois), 1)
        self.assertEqual(resp.detected_pois[0].identity.obj_id, 1)
        self.assertIs(resp.primary_poi, poi)

    def test_wall_period_for_scheduler_period_uses_fastest_child_period(self):
        """Coordinator forwards sim loop timing through child detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()
        detector1.wall_period_for_scheduler_period = Mock(return_value=0.012)
        detector2.wall_period_for_scheduler_period = Mock(return_value=0.008)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        self.assertEqual(
            coordinator.wall_period_for_scheduler_period(0.04), 0.008,
        )
        detector1.wall_period_for_scheduler_period.assert_called_once_with(0.04)
        detector2.wall_period_for_scheduler_period.assert_called_once_with(0.04)

    def test_wall_period_for_scheduler_period_falls_back_without_valid_children(self):
        """Invalid child timing never changes the requested loop period."""
        detector = self._create_mock_detector()
        detector.wall_period_for_scheduler_period = Mock(return_value=-1.0)

        coordinator = DetectionCoordinator([detector], self.logger)

        self.assertEqual(
            coordinator.wall_period_for_scheduler_period(0.04), 0.04,
        )

    def test_source_driven_capability_forwards_from_child(self):
        polling = self._create_mock_detector()
        source_driven = self._create_mock_detector()
        polling.has_source_driven_detection_events = False
        source_driven.has_source_driven_detection_events = True

        coordinator = DetectionCoordinator([polling, source_driven], self.logger)

        self.assertTrue(coordinator.has_source_driven_detection_events)

    def test_drain_detection_events_preserves_events_and_assigns_task_ids(self):
        later = self._create_mock_poi(
            obj_id=1, source_name="gimbal_0", timestamp=10.02,
        )
        earlier = self._create_mock_poi(
            obj_id=1, source_name="gimbal_1", timestamp=10.00,
        )
        detector_later = self._create_mock_detector()
        detector_earlier = self._create_mock_detector()
        detector_later.has_source_driven_detection_events = True
        detector_earlier.has_source_driven_detection_events = True
        detector_later.drain_detection_events.return_value = [
            DetectionPublication(
                (later,),
                source_timestamp_s=10.02,
                source_receipt_timestamp_s=None,
                source_name="gimbal_0",
                source_discontinuity=False,
            ),
        ]
        detector_earlier.drain_detection_events.return_value = [
            DetectionPublication(
                (earlier,),
                source_timestamp_s=10.00,
                source_receipt_timestamp_s=None,
                source_name="gimbal_1",
                source_discontinuity=True,
            ),
        ]
        coordinator = DetectionCoordinator(
            [detector_later, detector_earlier], self.logger,
        )

        responses = coordinator.drain_detection_events(DetectRequest())

        self.assertEqual(
            [response.primary_poi for response in responses],
            [earlier, later],
        )
        self.assertEqual(earlier.identity.task_id, 1)
        self.assertEqual(later.identity.task_id, 2)
        self.assertEqual(
            [response.source_timestamp_s for response in responses],
            [10.00, 10.02],
        )
        self.assertEqual(
            [response.source_name for response in responses],
            ["gimbal_1", "gimbal_0"],
        )
        self.assertTrue(responses[0].source_discontinuity)
        detector_later.drain_detection_events.assert_called_once()
        detector_earlier.drain_detection_events.assert_called_once()

    def test_mixed_source_drain_does_not_resample_polling_child(self):
        source_poi = self._create_mock_poi(
            obj_id=1, source_name="gimbal_source",
        )
        polling_poi = self._create_mock_poi(
            obj_id=1, source_name="gimbal_polling",
        )
        source_poi.replace_timing(
            replace(source_poi.timing, detection_timestamp_s=10.0)
        )
        polling_poi.replace_timing(
            replace(polling_poi.timing, detection_timestamp_s=10.0)
        )
        source = self._create_mock_detector()
        polling = self._create_mock_detector(
            [polling_poi], primary_poi=polling_poi,
        )
        source.has_source_driven_detection_events = True
        polling.has_source_driven_detection_events = False
        source.drain_detection_events.return_value = [
            DetectionPublication(
                (source_poi,),
                source_timestamp_s=10.0,
                source_receipt_timestamp_s=None,
                source_name="gimbal_source",
                source_discontinuity=False,
            ),
        ]
        coordinator = DetectionCoordinator([source, polling], self.logger)

        responses = coordinator.drain_detection_events(DetectRequest())

        self.assertEqual(len(responses), 1)
        self.assertIs(responses[0].primary_poi, source_poi)
        polling.get_detect_data.assert_not_called()

        snapshot = coordinator.get_detect_data(DetectRequest())

        self.assertIn(polling_poi, snapshot.detected_pois)
        self.assertNotEqual(
            source_poi.identity.task_id,
            polling_poi.identity.task_id,
        )
        polling.get_detect_data.assert_called_once()
        self.assertTrue(
            coordinator.poi_uses_source_driven_events(source_poi)
        )
        self.assertFalse(
            coordinator.poi_uses_source_driven_events(polling_poi)
        )

    def test_multiple_detectors_aggregation(self):
        """Coordinator aggregates detections from multiple detectors."""
        poi1 = self._create_mock_poi(obj_id=1)
        poi2 = self._create_mock_poi(obj_id=2)
        poi3 = self._create_mock_poi(obj_id=3)

        detector1 = self._create_mock_detector([poi1], primary_poi=poi1)
        detector2 = self._create_mock_detector([poi2, poi3], primary_poi=poi2)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())
        self.assertEqual(len(resp.detected_pois), 3)

        obj_ids = [t.identity.obj_id for t in resp.detected_pois]
        self.assertIn(1, obj_ids)
        self.assertIn(2, obj_ids)
        self.assertIn(3, obj_ids)

    def test_aggregation_preserves_confirmation_frame_capability(self):
        """Per-detection frame capability survives coordinator aggregation.

        Mixed fleet: a frame-less ideal child and a frame-capable child.
        The flag is per-detection so it must ride each POI through
        aggregation and primary selection unchanged.
        """
        ideal_poi = self._create_mock_poi(
            obj_id=1,
            x_error=1000,
            y_error=500,
            supports_confirmation_frame=False,
        )
        camera_poi = self._create_mock_poi(obj_id=2, x_error=1400, y_error=700,
                                                 source_name="gimbal_1",
                                                 supports_confirmation_frame=True)

        detector1 = self._create_mock_detector([ideal_poi], primary_poi=ideal_poi)
        detector2 = self._create_mock_detector([camera_poi], primary_poi=camera_poi)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())
        by_obj_id = {t.identity.obj_id: t for t in resp.detected_pois}
        self.assertFalse(by_obj_id[1].confirmation.supports_frame)
        self.assertTrue(by_obj_id[2].confirmation.supports_frame)
        self.assertFalse(resp.primary_poi.confirmation.supports_frame)

    def test_primary_poi_comes_from_best_detector_primary(self):
        """Coordinator ranks detector primaries, not arbitrary peer detections."""
        off_center_primary = self._create_mock_poi(obj_id=1, x_error=1400, y_error=700)
        best_primary = self._create_mock_poi(obj_id=2, x_error=1010, y_error=505)
        centered_peer = self._create_mock_poi(obj_id=3, x_error=1000, y_error=500)

        detector1 = self._create_mock_detector([off_center_primary], primary_poi=off_center_primary)
        detector2 = self._create_mock_detector([centered_peer, best_primary], primary_poi=best_primary)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())

        self.assertIs(resp.primary_poi, best_primary)
        self.assertEqual(resp.detected_pois[0].identity.obj_id, 2)

    def test_primary_poi_falls_back_to_best_detection_when_missing(self):
        """Coordinator derives a primary POI when detectors do not provide one."""
        far_poi = self._create_mock_poi(obj_id=1, x_error=1500, y_error=800)
        centered_poi = self._create_mock_poi(obj_id=2, x_error=1005, y_error=495)

        detector1 = self._create_mock_detector([far_poi])
        detector2 = self._create_mock_detector([centered_poi])

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())

        self.assertIs(resp.primary_poi, centered_poi)
        self.assertEqual(resp.detected_pois[0].identity.obj_id, 2)

    def test_assigns_unique_task_ids_for_same_local_obj_id_across_detectors(self):
        """Coordinator namespaces detector-local IDs into unique task IDs."""
        left = self._create_mock_poi(obj_id=1, source_name="gimbal_0")
        right = self._create_mock_poi(obj_id=1, source_name="gimbal_1")

        detector1 = self._create_mock_detector([left], primary_poi=left)
        detector2 = self._create_mock_detector([right], primary_poi=right)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        resp = coordinator.get_detect_data(DetectRequest())

        self.assertEqual(len(resp.detected_pois), 2)
        self.assertEqual(resp.detected_pois[0].identity.task_id, 1)
        self.assertEqual(resp.detected_pois[1].identity.task_id, 2)
        self.assertNotEqual(left.identity.task_id, right.identity.task_id)

    def _create_mock_detector_with_gimbal_name(self, gimbal_name: str, pois=None):
        """Mock detector with an explicit source identity capability."""
        detector = self._create_mock_detector(pois=pois,
                                              primary_poi=pois[0] if pois else None)
        detector.source_name = gimbal_name
        return detector

    def test_start_tracking_routes_by_explicit_source_identity(self):
        """A task ID routes only to the detector exposing its source name."""
        left = self._create_mock_poi(obj_id=1, source_name="gimbal_0")
        right = self._create_mock_poi(obj_id=1, source_name="gimbal_1")
        d0 = self._create_mock_detector_with_gimbal_name("gimbal_0", [left])
        d1 = self._create_mock_detector_with_gimbal_name("gimbal_1", [right])
        coord = DetectionCoordinator([d0, d1], self.logger)

        # Populate the allocator: task_id 1 → (gimbal_0, 1); task_id 2 → (gimbal_1, 1).
        coord.get_detect_data(DetectRequest())

        coord.start_tracking(1)
        d0.start_tracking.assert_called_once_with(1)
        d1.start_tracking.assert_not_called()

        d0.start_tracking.reset_mock()
        coord.start_tracking(2)
        d0.start_tracking.assert_not_called()
        d1.start_tracking.assert_called_once_with(1)

    def test_start_tracking_raises_when_resolved_source_has_no_matching_detector(self):
        """If the allocator resolves a task_id but no detector's gimbal
        matches, raise — a silent no-op would let navigation_task_action.start think
        tracking was armed and advance to CONFIRM with no gimbal moving."""
        tgt = self._create_mock_poi(obj_id=1, source_name="gimbal_missing")
        d = self._create_mock_detector_with_gimbal_name("gimbal_present", [tgt])
        coord = DetectionCoordinator([d], self.logger)
        coord.get_detect_data(DetectRequest())  # allocates task_id 1

        with self.assertRaises(RuntimeError):
            coord.start_tracking(1)
        d.start_tracking.assert_not_called()

    def test_start_tracking_falls_back_to_broadcast_when_unresolvable(self):
        """If the task id never went through the allocator (single-detector
        legacy setup), fall back to broadcasting with a warning."""
        d = self._create_mock_detector()
        coord = DetectionCoordinator([d], self.logger)

        coord.start_tracking(999)  # never allocated
        d.start_tracking.assert_called_once_with(999)

    def test_allocator_rebind_updates_both_maps_and_poi(self):
        """rebind evicts stale mappings both directions and adopts the
        ORIGINAL task_id onto the new identity: a FRESH detection of the
        new identity then resolves to the ORIGINAL task_id via assign(),
        get_identity(task_id) resolves to the new identity, and no stale
        reverse mapping remains for the evicted task_id."""
        from navpy.modules.vision.poi_identity import PoiTaskIdAllocator

        allocator = PoiTaskIdAllocator()

        old_poi = self._create_mock_poi(obj_id=1, source_name="gimbal_0")
        task_id = allocator.assign(old_poi)
        self.assertEqual(task_id, 1)

        new_poi = self._create_mock_poi(obj_id=2, source_name="gimbal_0")
        new_task_id = allocator.assign(new_poi)
        self.assertEqual(new_task_id, 2)

        self.assertTrue(allocator.rebind(task_id, new_poi))
        self.assertEqual(new_poi.identity.task_id, task_id)

        # A FRESH detection of the new identity resolves to the ORIGINAL task_id.
        fresh_new_poi = self._create_mock_poi(obj_id=2, source_name="gimbal_0")
        self.assertEqual(allocator.assign(fresh_new_poi), task_id)

        # get_identity(task_id) now resolves to the NEW identity.
        identity = allocator.get_identity(task_id)
        self.assertEqual(identity.source_name, "gimbal_0")
        self.assertEqual(identity.local_obj_id, 2)

        # No stale reverse mapping remains for the evicted task_id (2).
        self.assertIsNone(allocator.get_identity(new_task_id))

    def test_coordinator_rebind_task_id_routes_start_tracking_to_new_track(self):
        """After rebind_task_id, start_tracking(original_task_id) reaches
        the detector owning the NEW local id — the allocator's
        identity->task_id mapping fully adopts the new track under the
        ORIGINAL task id."""
        left = self._create_mock_poi(obj_id=1, source_name="gimbal_0")
        right = self._create_mock_poi(obj_id=1, source_name="gimbal_1")
        d0 = self._create_mock_detector_with_gimbal_name("gimbal_0", [left])
        d1 = self._create_mock_detector_with_gimbal_name("gimbal_1", [right])
        coord = DetectionCoordinator([d0, d1], self.logger)

        # Populate the allocator: task_id 1 -> (gimbal_0, 1); task_id 2 -> (gimbal_1, 1).
        coord.get_detect_data(DetectRequest())
        self.assertEqual(left.identity.task_id, 1)
        self.assertEqual(right.identity.task_id, 2)

        # gimbal_0's local track is lost; gimbal_1's local track #1 (right)
        # is the SAME physical POI re-detected under a different local id.
        self.assertTrue(coord.rebind_task_id(1, right))
        self.assertEqual(right.identity.task_id, 1)

        coord.start_tracking(1)
        d1.start_tracking.assert_called_once_with(1)
        d0.start_tracking.assert_not_called()

    def test_get_zoom_result_routes_to_active_poi_detector(self):
        """Zoom status must come from the detector that produced the active task id."""
        left = self._create_mock_poi(obj_id=7, source_name="gimbal_0")
        right = self._create_mock_poi(obj_id=7, source_name="gimbal_1")
        d0 = self._create_mock_detector_with_gimbal_name("gimbal_0", [left])
        d1 = self._create_mock_detector_with_gimbal_name("gimbal_1", [right])
        left_result = object()
        right_result = object()
        d0.get_zoom_result.return_value = left_result
        d1.get_zoom_result.return_value = right_result
        coord = DetectionCoordinator([d0, d1], self.logger)
        resp = coord.get_detect_data(DetectRequest())

        result = coord.get_zoom_result(resp.detected_pois[0].identity.task_id)

        self.assertIs(result, left_result)
        d0.get_zoom_result.assert_called_once_with(7)
        d1.get_zoom_result.assert_not_called()

    def test_get_zoom_result_returns_none_when_source_detector_missing(self):
        poi = self._create_mock_poi(obj_id=3, source_name="gimbal_missing")
        detector = self._create_mock_detector_with_gimbal_name("gimbal_present", [poi])
        coord = DetectionCoordinator([detector], self.logger)
        resp = coord.get_detect_data(DetectRequest())

        self.assertIsNone(
            coord.get_zoom_result(resp.detected_pois[0].identity.task_id)
        )
        detector.get_zoom_result.assert_not_called()

    def test_get_zoom_result_returns_none_when_source_identity_does_not_match(self):
        poi = self._create_mock_poi(obj_id=3, source_name="gimbal_0")
        detector = self._create_mock_detector([poi], primary_poi=poi)
        coord = DetectionCoordinator([detector], self.logger)
        resp = coord.get_detect_data(DetectRequest())

        self.assertIsNone(
            coord.get_zoom_result(resp.detected_pois[0].identity.task_id)
        )
        detector.get_zoom_result.assert_not_called()

    def test_get_zoom_result_single_detector_falls_back_to_local_id(self):
        detector = self._create_mock_detector()
        expected = object()
        detector.get_zoom_result.return_value = expected
        coord = DetectionCoordinator([detector], self.logger)

        self.assertIs(coord.get_zoom_result(999), expected)
        detector.get_zoom_result.assert_called_once_with(999)

    def test_get_zoom_result_uses_single_armed_detector_when_unidentified(self):
        d0 = self._create_mock_detector()
        d1 = self._create_mock_detector()
        d1.is_detection_armed = True
        expected = object()
        d1.get_zoom_result.return_value = expected
        coord = DetectionCoordinator([d0, d1], self.logger)

        self.assertIs(coord.get_zoom_result(), expected)
        d0.get_zoom_result.assert_not_called()
        d1.get_zoom_result.assert_called_once_with(None)

    def test_get_zoom_result_returns_none_for_ambiguous_armed_detectors(self):
        d0 = self._create_mock_detector()
        d1 = self._create_mock_detector()
        d0.is_detection_armed = True
        d1.is_detection_armed = True
        coord = DetectionCoordinator([d0, d1], self.logger)

        self.assertIsNone(coord.get_zoom_result())
        d0.get_zoom_result.assert_not_called()
        d1.get_zoom_result.assert_not_called()

    def test_start_calls_all_detectors(self):
        """Start calls start on all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        coordinator.start()

        detector1.start.assert_called_once()
        detector2.start.assert_called_once()

    def test_fleet_snapshots_caller_members(self):
        first = self._create_mock_detector()
        later = self._create_mock_detector()
        members = [first]
        coordinator = DetectionCoordinator(members, self.logger)
        members.append(later)

        coordinator.start()
        coordinator.refresh()
        coordinator.stop()

        first.start.assert_called_once_with()
        first.refresh.assert_called_once_with()
        first.stop.assert_called_once_with()
        later.start.assert_not_called()
        later.refresh.assert_not_called()
        later.stop.assert_not_called()

    def test_start_rolls_back_failing_and_started_detectors_in_reverse(self):
        order = []
        detectors = [self._create_mock_detector() for _ in range(3)]
        detectors[0].start.side_effect = lambda: order.append("first.start")

        def fail_second():
            order.append("second.start")
            raise RuntimeError("second failed")

        detectors[1].start.side_effect = fail_second
        detectors[0].stop.side_effect = lambda: order.append("first.stop") or True
        detectors[1].stop.side_effect = lambda: order.append("second.stop") or True
        coordinator = DetectionCoordinator(detectors, self.logger)

        with self.assertRaisesRegex(RuntimeError, "second failed"):
            coordinator.start()

        self.assertEqual(
            order,
            ["first.start", "second.start", "second.stop", "first.stop"],
        )
        detectors[2].start.assert_not_called()

    def test_start_preserves_detector_rollback_failure(self):
        start_error = RuntimeError("start failed")
        rollback_error = RuntimeError("rollback failed")
        detector = self._create_mock_detector()
        detector.start.side_effect = start_error
        detector.stop.side_effect = rollback_error
        coordinator = DetectionCoordinator([detector], self.logger)

        with self.assertRaises(ExceptionGroup) as raised:
            coordinator.start()

        self.assertEqual(
            raised.exception.exceptions,
            (start_error, rollback_error),
        )

    def test_baseexception_start_rolls_back_attempted_detectors(self):
        interrupt = KeyboardInterrupt()
        detectors = [self._create_mock_detector() for _ in range(2)]
        detectors[1].start.side_effect = interrupt
        coordinator = DetectionCoordinator(detectors, self.logger)

        with self.assertRaises(KeyboardInterrupt) as raised:
            coordinator.start()

        self.assertIs(raised.exception, interrupt)
        detectors[1].stop.assert_called_once_with()
        detectors[0].stop.assert_called_once_with()

    def test_stop_calls_all_detectors(self):
        """Stop calls stop on all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        coordinator.stop()

        detector1.stop.assert_called_once()
        detector2.stop.assert_called_once()

    def test_stop_attempts_all_three_detectors_and_preserves_failures(self):
        """One broken detector cannot strand later detector resources."""
        order = []
        first_error = RuntimeError("first stop failed")
        third_error = RuntimeError("third stop failed")
        detectors = [self._create_mock_detector() for _ in range(3)]

        def fail_first():
            order.append("first")
            raise first_error

        def fail_third():
            order.append("third")
            raise third_error

        detectors[0].stop.side_effect = fail_first
        detectors[1].stop.side_effect = lambda: order.append("second") or True
        detectors[2].stop.side_effect = fail_third
        coordinator = DetectionCoordinator(detectors, self.logger)

        with self.assertRaises(ExceptionGroup) as raised:
            coordinator.stop()

        self.assertEqual(order, ["first", "second", "third"])
        self.assertEqual(
            raised.exception.exceptions,
            (first_error, third_error),
        )

    def test_stop_attempts_later_detectors_after_baseexception(self):
        interrupt = KeyboardInterrupt()
        detectors = [self._create_mock_detector() for _ in range(2)]
        detectors[0].stop.side_effect = interrupt
        coordinator = DetectionCoordinator(detectors, self.logger)

        with self.assertRaises(KeyboardInterrupt) as raised:
            coordinator.stop()

        self.assertIs(raised.exception, interrupt)
        detectors[1].stop.assert_called_once_with()

    def test_refresh_calls_all_detectors(self):
        """Refresh calls refresh on all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        coordinator.refresh()

        detector1.refresh.assert_called_once()
        detector2.refresh.assert_called_once()

    def test_is_simulation_all_sim(self):
        """is_simulation returns True when all detectors are simulation."""
        detector1 = self._create_mock_detector(is_sim=True)
        detector2 = self._create_mock_detector(is_sim=True)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        self.assertTrue(coordinator.is_simulation)

    def test_is_simulation_mixed(self):
        """is_simulation returns False when any detector is not simulation."""
        detector1 = self._create_mock_detector(is_sim=True)
        detector2 = self._create_mock_detector(is_sim=False)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        self.assertFalse(coordinator.is_simulation)

    def test_is_simulation_all_real(self):
        """is_simulation returns False when all detectors are real."""
        detector1 = self._create_mock_detector(is_sim=False)
        detector2 = self._create_mock_detector(is_sim=False)

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        self.assertFalse(coordinator.is_simulation)

    def test_set_sim_poi_propagates_to_all(self):
        """set_sim_poi calls set_sim_poi on all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        location = Location(40.0, 44.0, 1000)
        coordinator.set_sim_poi(5, location)

        detector1.set_sim_poi.assert_called_once_with(5, location, location_type=None)
        detector2.set_sim_poi.assert_called_once_with(5, location, location_type=None)

    def test_is_zoom_stable_true_when_all_detectors_stable(self):
        d1 = self._create_mock_detector()
        d2 = self._create_mock_detector()
        d1.is_zoom_stable = True
        d2.is_zoom_stable = True
        coordinator = DetectionCoordinator([d1, d2], self.logger)
        self.assertTrue(coordinator.is_zoom_stable)

    def test_is_zoom_stable_false_when_any_detector_unstable(self):
        """Mixed fleet: zoom-capable mount mid-converge blocks the gate
        even if the other (fixed) mount reports stable trivially."""
        d1 = self._create_mock_detector()
        d2 = self._create_mock_detector()
        d1.is_zoom_stable = True   # fixed mount — always stable
        d2.is_zoom_stable = False  # zoom-capable, still converging
        coordinator = DetectionCoordinator([d1, d2], self.logger)
        self.assertFalse(coordinator.is_zoom_stable)

    def test_mounts_aggregates_from_all_detectors(self):
        """mounts property returns mounts from all detectors."""
        mount1 = Mock()
        mount2 = Mock()
        mount3 = Mock()

        detector1 = self._create_mock_detector()
        detector1.mounts = [mount1]

        detector2 = self._create_mock_detector()
        detector2.mounts = [mount2, mount3]

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)

        mounts = coordinator.mounts
        self.assertEqual(len(mounts), 3)
        self.assertIn(mount1, mounts)
        self.assertIn(mount2, mounts)
        self.assertIn(mount3, mounts)

    def test_select_best_detection_empty_list(self):
        """POI priority returns None for an empty list."""
        result = select_most_centered_poi([])
        self.assertIsNone(result)

    def test_select_best_detection_single_poi(self):
        """POI priority returns the single POI."""
        poi = self._create_mock_poi()
        result = select_most_centered_poi([poi])
        self.assertEqual(result, poi)


    def test_start_tracking_forwards_to_all_detectors(self):
        """start_tracking forwards obj_id to all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        coordinator.start_tracking(42)

        detector1.start_tracking.assert_called_once_with(42)
        detector2.start_tracking.assert_called_once_with(42)

    def test_start_tracking_rolls_back_failing_and_started_members(self):
        order = []
        first = self._create_mock_detector()
        second = self._create_mock_detector()
        first.start_tracking.side_effect = lambda _obj_id: order.append("first.start")

        def fail_second(_obj_id):
            order.append("second.start")
            raise RuntimeError("arm failed")

        second.start_tracking.side_effect = fail_second
        first.stop_tracking.side_effect = (
            lambda *, to_neutral: order.append(("first.stop", to_neutral))
        )
        second.stop_tracking.side_effect = (
            lambda *, to_neutral: order.append(("second.stop", to_neutral))
        )
        coordinator = DetectionCoordinator([first, second], self.logger)

        with self.assertRaisesRegex(RuntimeError, "arm failed"):
            coordinator.start_tracking(42)

        self.assertEqual(
            order,
            [
                "first.start",
                "second.start",
                ("second.stop", True),
                ("first.stop", True),
            ],
        )

    def test_tracking_router_snapshots_members_and_isolates_logger_failure(self):
        first = self._create_mock_detector()
        later = self._create_mock_detector()
        members = [first]
        logger = Mock()
        logger.warning.side_effect = RuntimeError("logger failed")
        resolver = Mock()
        resolver.identity_for_task.return_value = None
        router = TrackingCommandRouter(members, resolver, logger)
        members.append(later)

        router.start_tracking(7)

        first.start_tracking.assert_called_once_with(7)
        later.start_tracking.assert_not_called()

    def test_tracking_router_rejects_duplicate_resolved_sources(self):
        first = self._create_mock_detector()
        second = self._create_mock_detector()
        first.source_name = "gimbal_0"
        second.source_name = "gimbal_0"
        resolver = Mock()
        resolver.identity_for_task.return_value = SimpleNamespace(
            source_name="gimbal_0",
            local_obj_id=3,
        )
        router = TrackingCommandRouter([first, second], resolver, Mock())

        with self.assertRaisesRegex(RuntimeError, "ambiguous"):
            router.start_tracking(9)

        first.start_tracking.assert_not_called()
        second.start_tracking.assert_not_called()

    def test_resolved_tracking_failure_compensates_the_failing_member(self):
        detector = self._create_mock_detector()
        detector.source_name = "gimbal_0"
        detector.start_tracking.side_effect = RuntimeError("partial arm")
        resolver = Mock()
        resolver.identity_for_task.return_value = SimpleNamespace(
            source_name="gimbal_0",
            local_obj_id=4,
        )
        router = TrackingCommandRouter([detector], resolver, Mock())

        with self.assertRaisesRegex(RuntimeError, "partial arm"):
            router.start_tracking(10)

        detector.stop_tracking.assert_called_once_with(to_neutral=True)

    def test_zoom_router_rejects_duplicate_resolved_sources(self):
        first = self._create_mock_detector()
        second = self._create_mock_detector()
        first.source_name = "gimbal_0"
        second.source_name = "gimbal_0"
        resolver = Mock()
        resolver.identity_for_task.return_value = SimpleNamespace(
            source_name="gimbal_0",
            local_obj_id=3,
        )
        router = ZoomControlRouter([first, second], resolver)

        with self.assertRaisesRegex(RuntimeError, "ambiguous"):
            router.get_zoom_result(9)

    def test_stop_tracking_forwards_to_all_detectors(self):
        """stop_tracking forwards to all detectors."""
        detector1 = self._create_mock_detector()
        detector2 = self._create_mock_detector()

        coordinator = DetectionCoordinator([detector1, detector2], self.logger)
        coordinator.stop_tracking()

        detector1.stop_tracking.assert_called_once_with(to_neutral=True)
        detector2.stop_tracking.assert_called_once_with(to_neutral=True)

    def test_stop_tracking_attempts_all_members_and_preserves_failures(self):
        order = []
        first_error = RuntimeError("first tracking stop failed")
        third_error = RuntimeError("third tracking stop failed")
        detectors = [self._create_mock_detector() for _ in range(3)]

        def fail_first(*, to_neutral):
            order.append(("first", to_neutral))
            raise first_error

        def stop_second(*, to_neutral):
            order.append(("second", to_neutral))

        def fail_third(*, to_neutral):
            order.append(("third", to_neutral))
            raise third_error

        detectors[0].stop_tracking.side_effect = fail_first
        detectors[1].stop_tracking.side_effect = stop_second
        detectors[2].stop_tracking.side_effect = fail_third
        coordinator = DetectionCoordinator(detectors, self.logger)

        with self.assertRaises(ExceptionGroup) as raised:
            coordinator.stop_tracking(to_neutral=False)

        self.assertEqual(
            order,
            [("first", False), ("second", False), ("third", False)],
        )
        self.assertEqual(
            raised.exception.exceptions,
            (first_error, third_error),
        )

    def test_detector_abc_does_not_own_tracking_defaults(self):
        """The compatibility marker must not hide missing capabilities."""
        self.assertTrue({
            "start_tracking", "stop_tracking", "get_zoom_result",
        }.isdisjoint(DetectorAbc.__dict__))


class TestDetectionCoordinatorZoomSizeDemand(unittest.TestCase):
    def setUp(self):
        self.logger = Mock()

    def _create_mock_detector(self):
        return _detector_double()

    def test_set_zoom_size_demand_broadcasts(self):
        detectors = [self._create_mock_detector() for _ in range(3)]
        coordinator = DetectionCoordinator(detectors, self.logger)

        coordinator.set_zoom_size_demand(False)

        for detector in detectors:
            detector.set_zoom_size_demand.assert_called_once_with(False)

    def test_final_approach_zoom_freeze_broadcasts(self):
        detectors = [self._create_mock_detector() for _ in range(3)]
        coordinator = DetectionCoordinator(detectors, self.logger)

        coordinator.freeze_final_approach_zoom_at_min()

        for detector in detectors:
            detector.freeze_final_approach_zoom_at_min.assert_called_once_with()

    def test_detector_abc_does_not_own_zoom_defaults(self):
        self.assertTrue({
            "set_zoom_size_demand", "freeze_final_approach_zoom_at_min",
        }.isdisjoint(DetectorAbc.__dict__))


class TestDetectionCoordinatorGeo(unittest.TestCase):
    """Geo-tracking forwarding + rollback semantics on the coordinator."""

    def setUp(self):
        self.logger = Mock()
        self.poi_loc = Location(40.3, 44.4, 1500.0)
        self.uav_loc = Location(40.31, 44.41, 1700.0)
        self.uav_att = Mock()
        self.geo_ref = Mock()

    def _detector(self):
        return _detector_double()

    def test_start_geo_tracking_broadcasts_to_all_children(self):
        d1, d2 = self._detector(), self._detector()
        coord = DetectionCoordinator([d1, d2], self.logger)

        coord.start_geo_tracking(self.poi_loc, self.geo_ref)

        d1.start_geo_tracking.assert_called_once_with(self.poi_loc, self.geo_ref)
        d2.start_geo_tracking.assert_called_once_with(self.poi_loc, self.geo_ref)

    def test_update_geo_broadcasts_to_all_children(self):
        d1, d2 = self._detector(), self._detector()
        coord = DetectionCoordinator([d1, d2], self.logger)

        coord.update_geo(self.uav_loc, self.uav_att)

        d1.update_geo.assert_called_once_with(self.uav_loc, self.uav_att)
        d2.update_geo.assert_called_once_with(self.uav_loc, self.uav_att)

    def test_prepare_geo_acquisition_broadcasts_and_returns_any_success(self):
        d1, d2 = self._detector(), self._detector()
        d1.prepare_geo_acquisition.return_value = False
        d2.prepare_geo_acquisition.return_value = True
        coord = DetectionCoordinator([d1, d2], self.logger)

        prepared = coord.prepare_geo_acquisition(
            self.uav_loc, self.uav_att, class_id=2, min_pixels=15.0,
        )

        self.assertTrue(prepared)
        d1.prepare_geo_acquisition.assert_called_once_with(
            self.uav_loc, self.uav_att, 2, 15.0,
        )
        d2.prepare_geo_acquisition.assert_called_once_with(
            self.uav_loc, self.uav_att, 2, 15.0,
        )

    def test_stop_geo_tracking_broadcasts_to_all_children(self):
        d1, d2 = self._detector(), self._detector()
        coord = DetectionCoordinator([d1, d2], self.logger)

        coord.stop_geo_tracking()

        d1.stop_geo_tracking.assert_called_once()
        d2.stop_geo_tracking.assert_called_once()

    def test_start_geo_tracking_rolls_back_armed_children_on_failure(self):
        d1, d2, d3 = self._detector(), self._detector(), self._detector()
        d2.start_geo_tracking.side_effect = RuntimeError("locked")
        coord = DetectionCoordinator([d1, d2, d3], self.logger)

        with self.assertRaises(RuntimeError):
            coord.start_geo_tracking(self.poi_loc, self.geo_ref)

        # d1 armed and was rolled back; d2 raised; d3 never armed.
        d1.start_geo_tracking.assert_called_once()
        d1.stop_geo_tracking.assert_called_once()
        d3.start_geo_tracking.assert_not_called()

    def test_start_geo_tracking_first_child_raises_no_others_armed(self):
        d1, d2 = self._detector(), self._detector()
        d1.start_geo_tracking.side_effect = RuntimeError("locked")
        coord = DetectionCoordinator([d1, d2], self.logger)

        with self.assertRaises(RuntimeError):
            coord.start_geo_tracking(self.poi_loc, self.geo_ref)

        d2.start_geo_tracking.assert_not_called()
        # No rollback needed — nobody armed.
        d1.stop_geo_tracking.assert_not_called()

    def test_rollback_failure_logs_but_does_not_suppress_original_raise(self):
        d1, d2 = self._detector(), self._detector()
        d2.start_geo_tracking.side_effect = RuntimeError("locked")
        d1.stop_geo_tracking.side_effect = RuntimeError("rollback failed")
        coord = DetectionCoordinator([d1, d2], self.logger)

        with self.assertRaises(RuntimeError) as ctx:
            coord.start_geo_tracking(self.poi_loc, self.geo_ref)

        # Original "locked" must propagate, not the rollback's own exception.
        self.assertEqual(str(ctx.exception), "locked")
        self.logger.warning.assert_called()

    def test_is_geo_armed_any_true(self):
        d1, d2 = self._detector(), self._detector()
        coord = DetectionCoordinator([d1, d2], self.logger)
        self.assertFalse(coord.is_geo_armed)

        d2.is_geo_armed = True
        self.assertTrue(coord.is_geo_armed)

    def test_is_detection_armed_any_true(self):
        d1, d2 = self._detector(), self._detector()
        coord = DetectionCoordinator([d1, d2], self.logger)
        self.assertFalse(coord.is_detection_armed)

        d1.is_detection_armed = True
        self.assertTrue(coord.is_detection_armed)


class TestDetectorAbcCapabilities(unittest.TestCase):
    """The legacy marker owns no optional detector behavior."""

    def test_detector_abc_does_not_own_geo_defaults(self):
        self.assertTrue({
            "start_geo_tracking",
            "update_geo",
            "prepare_geo_acquisition",
            "stop_geo_tracking",
        }.isdisjoint(DetectorAbc.__dict__))


class TestDetectorLifecycleFleet(unittest.TestCase):
    def test_incomplete_member_stop_propagates_without_skipping_siblings(self):
        first = Mock()
        first.stop.side_effect = [False, True]
        second = Mock()
        second.stop.return_value = True
        fleet = DetectorLifecycleFleet([first, second], Mock())

        self.assertFalse(fleet.stop())
        self.assertTrue(fleet.stop())

        self.assertEqual(first.stop.call_count, 2)
        self.assertEqual(second.stop.call_count, 1)

    def test_none_member_stop_is_not_retired_as_quiescent(self):
        incomplete = Mock()
        incomplete.stop.side_effect = [None, True]
        incomplete.is_quiescent = False
        sibling = Mock()
        sibling.stop.return_value = True
        fleet = DetectorLifecycleFleet([incomplete, sibling], Mock())

        with self.assertRaisesRegex(TypeError, "stop.*return bool"):
            fleet.stop()

        self.assertFalse(fleet.is_quiescent)
        sibling.stop.assert_called_once_with()
        self.assertTrue(fleet.stop())
        self.assertEqual(incomplete.stop.call_count, 2)
        sibling.stop.assert_called_once_with()

    def test_health_contract_checks_every_member(self):
        first = Mock()
        second = Mock()
        fleet = DetectorLifecycleFleet([first, second], Mock())

        fleet.raise_if_failed()

        first.raise_if_failed.assert_called_once_with()
        second.raise_if_failed.assert_called_once_with()

    def test_health_contract_propagates_exact_failure(self):
        failure = RuntimeError("detector worker failed")
        member = Mock()
        member.raise_if_failed.side_effect = failure
        fleet = DetectorLifecycleFleet([member], Mock())

        with self.assertRaises(RuntimeError) as raised:
            fleet.raise_if_failed()

        self.assertIs(raised.exception, failure)

    def test_stop_error_records_explicit_member_quiescence(self):
        failure = RuntimeError("resource cleanup failed")
        member = Mock()
        member.stop.side_effect = failure
        member.is_quiescent = True
        fleet = DetectorLifecycleFleet([member], Mock())

        with self.assertRaises(RuntimeError) as raised:
            fleet.stop()

        self.assertIs(raised.exception, failure)
        self.assertTrue(fleet.is_quiescent)

    def test_detector_abc_does_not_own_status_defaults(self):
        self.assertTrue({
            "is_geo_armed", "is_detection_armed", "loss_hold_sec",
        }.isdisjoint(DetectorAbc.__dict__))


if __name__ == '__main__':
    unittest.main()
