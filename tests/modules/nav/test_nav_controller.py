"""Comprehensive tests for NavController.

Covers:
- State machine transitions (ONHOLD, DETECT, CONFIRM, NAV, RESET, RECOVERY)
- Sense-Decide-Act loop
- POI management (selection, confirmation, rejection)
- Peer navigation
- Mode switch handling
- Edge cases and error conditions
"""
import math
import threading
import unittest
import time
from dataclasses import replace
import numpy as np
from pytest import approx as pytest_approx
from unittest.mock import ANY, Mock, MagicMock, call, patch, PropertyMock

from navpy.exception_groups import ExceptionGroup
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.peer_offset import (
    MIN_APPROACH_STANDOFF_M,
    OFFSET_LOITER_RADIUS_M,
)
from navpy.modules.nav.nav_controller import NavController, NavState, ALT_HYST
from navpy.modules.nav.confirmation_manager import ConfirmationStatus
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.poi_zoom_tracker import ZoomTrackResult, ZoomTrackingState
from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    TaskAssignMsgData,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_actor_slots import SlotState
from tests.modules.nav.nav_test_rig import (
    as_detection_coordination,
    create_nav_test_rig,
)
from tests.detection_factory import make_detected_poi


# =============================================================================
# Test Fixtures and Helpers
# =============================================================================


class _BlockingDetectionEventLease:
    """Protocol-correct idle source lease for navigation unit tests."""

    def __init__(self, reset_handler=None):
        self._reset_handler = reset_handler
        self._closed = threading.Event()

    @property
    def closed(self):
        return self._closed.is_set()

    def wait_and_dispatch(self, _handler):
        self._closed.wait()
        return None

    def close(self):
        self._closed.set()

def _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=200.0, is_armed=True):
    """Create mock vehicle with configurable attributes."""
    vehicle = Mock()
    vehicle.attitude = Attitude(0, 0, 0)
    vehicle.home_location = Location(40.0, -74.0, 100.0)
    vehicle.mission_items_count = 10
    vehicle.mission_items_next = next_wp
    vehicle.source_system = 1
    vehicle.get_mode = mode
    vehicle.is_armed = is_armed
    vehicle.location = Mock(return_value=Location(40.0, -74.0, alt))
    vehicle.get_param_or_default = Mock(side_effect=lambda name, default: default)
    vehicle.get_mission_item_location = Mock(return_value=None)
    vehicle.set_mode = Mock()
    vehicle.restart_mission = Mock()
    vehicle.max_pitch = 25.0
    vehicle.set_attitude = Mock()
    vehicle.get_parameter = Mock(return_value=1.0)
    vehicle.set_parameter = Mock(return_value=True)
    # Default test context is SITL (one-shot disarm is a sim-only reset);
    # real-autopilot tests override this to False.
    vehicle.is_simulated_autopilot = Mock(return_value=True)
    return vehicle


def _create_mock_detector(
        detections=None, primary_poi=None, is_zoom_stable=True,
        zoom_result=None):
    """Create mock detector with configurable detections."""
    detector = Mock()
    detector.get_detect_data = Mock(return_value=DetectResponse(detections or [], primary_poi=primary_poi))
    detector.drain_detection_events = Mock(return_value=[])
    detector.open_detection_event_lease = Mock(
        side_effect=lambda _request, reset_handler=None: (
            _BlockingDetectionEventLease(reset_handler)
        )
    )
    detector.has_source_driven_detection_events = False
    detector.is_simulation = True
    detector.is_zoom_stable = is_zoom_stable
    detector.get_zoom_result = Mock(return_value=zoom_result)
    # Real booleans, not bare Mock attributes — Mock attributes are truthy
    # and would silently arm gates in NavController code paths that rely
    # on is_detection_armed / is_geo_armed for control flow.
    detector.is_detection_armed = False
    detector.is_geo_armed = False
    detector.refresh = Mock()
    detector.set_sim_poi = Mock()
    detector.freeze_final_approach_zoom_at_min = Mock()

    # Wire is_geo_armed to the geo lifecycle the way the real chain behaves
    # (GimbalNavigation._geo_poi drives is_geo_armed): a successful
    # start_geo_tracking arms, stop_geo_tracking and the detection handoff
    # in start_tracking disarm. NavController's geo-hold entry gates
    # _geo_hold_active on is_geo_armed (fail-closed when arming silently
    # declines), so the default mock must actually arm.
    def _arm_geo(*_args, **_kwargs):
        detector.is_geo_armed = True

    def _disarm_geo(*_args, **_kwargs):
        detector.is_geo_armed = False

    detector.start_geo_tracking = Mock(side_effect=_arm_geo)
    detector.stop_geo_tracking = Mock(side_effect=_disarm_geo)
    detector.start_tracking = Mock(side_effect=_disarm_geo)
    detector.update_geo = Mock()
    detector.prepare_geo_acquisition = Mock(return_value=False)
    detector.mounts = []
    # Real float, not a bare Mock attribute — the geo-hold entry check
    # (_update_geo_hold) compares elapsed loss time against this value;
    # a Mock instance would raise TypeError on the comparison. Matches
    # GimbalRateTrackerConfig's default loss_hold_sec.
    detector.loss_hold_sec = 0.5
    return detector


def _zoom_result(
        *,
        state=ZoomTrackingState.HOLDING,
        has_poi=True,
        size_px=150.0,
        target_pixels=150.0,
        at_max_zoom=False):
    return ZoomTrackResult(
        state=state,
        has_poi=has_poi,
        size_px=size_px,
        target_pixels=target_pixels,
        at_max_zoom=at_max_zoom,
    )


def _create_mock_navigation():
    """Create mock navigation with default returns."""
    navigation = Mock()
    navigation.reset = Mock(return_value=Mock(status=Mock(return_value="OK")))
    navigation.algorithm_info = ("l1", 1.0)
    navigation.init = Mock()
    navigation.nav = Mock(return_value=True)
    navigation.pause_final_approach = Mock()
    navigation.final_approach = Mock()
    navigation.final_approach.is_active = False
    navigation.final_approach.clear_source_discontinuity = Mock()
    navigation.final_approach.invalidate_pending_source_work = Mock()
    navigation.final_approach.can_confirm_detection = Mock(return_value=True)
    navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
    navigation.final_approach.nav_without_detection = Mock(return_value=True)
    navigation.final_approach.poi_passed_override = Mock(return_value=None)
    navigation.final_approach.last_measured_lateral_bearing_deg = Mock(return_value=None)
    navigation.legacy_pois = Mock()
    navigation.legacy_pois.locked_distance = Mock(return_value=1000.0)
    navigation.legacy_pois.ground_location = Mock(
        return_value=Location(40.001, -74.001, 0.0)
    )
    navigation.vehicle_commands = Mock()
    navigation.vehicle_commands.peer_poi = Mock()
    navigation.vehicle_commands.peer_poi_loiter = Mock()
    # Sentinel for the GeoRefCalc accessor — NavController hands this
    # to detector.start_geo_tracking; tests assert identity by ``is``.
    navigation.legacy_pois.geo_ref = Mock(name="geo_ref_sentinel")
    return navigation


def _create_mock_args(min_wp=3, min_alt=50.0, confirm_wait=2.0,
                      auto_confirm=False, confirm_on_fail=True, nav_sim_speedup=0.0,
                      is_oneshot=False, confirm_gate_timeout=15.0):
    """Create mock NavArgs with configurable values."""
    args = Mock()
    args.min_wp = min_wp
    args.min_alt = min_alt
    args.confirm_wait_time_sec = confirm_wait
    args.confirm_gate_timeout_sec = confirm_gate_timeout
    args.is_auto_confirm = auto_confirm
    args.is_confirm_on_fail = confirm_on_fail
    args.nav_sim_speedup = nav_sim_speedup
    args.is_oneshot = is_oneshot
    args.refresh = Mock()
    return args


def _create_detected_poi(obj_id=0, x_error=100.0, y_error=100.0,
                            detection_frame=None, bbox=None, confidence=0.95,
                            class_id=0, p_t_g_l=None, task_id=None,
                            tracking_bbox=None,
                            supports_confirmation_frame=True):
    """Create a DetectedObject with configurable attributes."""
    return make_detected_poi(
        obj_id=obj_id,
        task_id=obj_id if task_id is None else task_id,
        x_error=x_error,
        y_error=y_error,
        detection_frame=detection_frame,
        supports_confirmation_frame=bool(supports_confirmation_frame),
        bbox_cxcywh=bbox,
        tracking_bbox_cxcywh=(
            tracking_bbox if tracking_bbox is not None else bbox
        ),
        confidence=confidence,
        class_id=class_id,
        p_t_g_l=p_t_g_l,
        t_g_loc_debug=Location(40.001, -74.001, 0.0),
    )


def _set_poi_source(poi, source_name: str):
    """Update explicit source identity on grouped observation/provenance."""
    poi.replace_pixel(replace(poi.pixel, source_name=source_name))
    poi.replace_pose(replace(
        poi.pose,
        gimbal_data=GimbalData(
            att=Attitude(0.0, 0.0, 0.0),
            name=source_name,
        ),
    ))
    return poi


def _stamp_fresh_source_poi(
        poi, *, frame_timestamp_s=20.0, receipt_timestamp_s=1000.0,
):
    """Give a source-event POI matching raw and receipt clock domains."""
    poi.replace_pixel(replace(
        poi.pixel,
        source_timestamp_s=frame_timestamp_s,
    ))
    poi.replace_timing(replace(
        poi.timing,
        detection_timestamp_s=frame_timestamp_s,
        camera_frame_timestamp_s=frame_timestamp_s,
        detection_now_s=lambda: frame_timestamp_s,
        source_receipt_timestamp_s=receipt_timestamp_s,
        source_receipt_now_s=lambda: receipt_timestamp_s,
    ))
    return poi


def _set_poi_timestamp(poi, timestamp_s):
    poi.replace_pixel(replace(
        poi.pixel,
        source_timestamp_s=timestamp_s,
    ))
    poi.replace_timing(replace(
        poi.timing,
        detection_timestamp_s=timestamp_s,
        camera_frame_timestamp_s=timestamp_s,
    ))
    return poi


def _set_poi_truth_location(poi, location):
    poi.replace_geo(replace(
        poi.geo,
        truth_poi_location=location,
    ))
    return poi


def _source_event(
        detected_pois,
        primary_poi=None,
        *,
        source_timestamp_s=None,
        source_receipt_timestamp_s=None,
        source_name=None,
        source_discontinuity=False,
):
    """Build one immutable source publication in canonical priority order."""
    ordered = list(detected_pois)
    if primary_poi is not None:
        ordered = [primary_poi] + [
            poi for poi in ordered if poi is not primary_poi
        ]
    return DetectionPublication(
        tuple(ordered),
        source_timestamp_s,
        source_receipt_timestamp_s,
        source_name,
        source_discontinuity,
    )


def _create_controller(vehicle=None, detector=None, navigation=None, args=None, logger=None,
                       vision_profile=None, approach_kind=ApproachKind.OFFSET,
                       scheduler_cadence=None):
    """Create an explicit test view of the composed navigation graph."""
    return create_nav_test_rig(
        vehicle or _create_mock_vehicle(),
        detector or _create_mock_detector(),
        navigation or _create_mock_navigation(),
        args or _create_mock_args(),
        logger or Mock(),
        approach_kind,
        vision_profile,
        scheduler_cadence,
    )


def _decide_and_act(controller):
    """Call _decide() then _act() — mimics one main loop iteration (minus sense)."""
    controller.decision.decide()
    controller.actions.act()


# =============================================================================
# NavState Enum Tests
# =============================================================================

class TestNavState(unittest.TestCase):
    """Tests for NavState enum values and semantics."""

    def test_state_values(self):
        """NavState has expected values."""
        self.assertEqual(NavState.ONHOLD.value, 0)
        self.assertEqual(NavState.DETECT.value, 1)
        self.assertEqual(NavState.CONFIRM.value, 2)
        self.assertEqual(NavState.NAV.value, 3)
        self.assertEqual(NavState.RESET.value, 4)
        self.assertEqual(NavState.RECOVERY.value, 5)

    def test_all_states_defined(self):
        """All expected states are defined."""
        states = list(NavState)
        self.assertEqual(len(states), 6)
        self.assertIn(NavState.ONHOLD, states)
        self.assertIn(NavState.DETECT, states)
        self.assertIn(NavState.CONFIRM, states)
        self.assertIn(NavState.NAV, states)
        self.assertIn(NavState.RESET, states)
        self.assertIn(NavState.RECOVERY, states)


# =============================================================================
# Initialization Tests
# =============================================================================

class TestNavControllerInit(unittest.TestCase):
    """Tests for NavController initialization."""

    def test_init_stores_dependencies(self):
        """NavController stores provided dependencies."""
        vehicle = _create_mock_vehicle()
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        args = _create_mock_args()
        logger = Mock()

        controller = NavController(
            vehicle,
            as_detection_coordination(detector),
            navigation,
            args,
            logger,
        )

        self.assertEqual(set(vars(controller)), {"_application"})
        navigation.bind_final_approach_source_dispatch.assert_called_once()

    def test_init_state_is_onhold(self):
        """NavController starts in ONHOLD state."""
        controller = _create_controller()
        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_init_no_active_poi(self):
        """NavController starts with no active POI."""
        controller = _create_controller()
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_init_logs_initial_status(self):
        """NavController logs initial status."""
        logger = Mock()
        _create_controller(logger=logger)
        logger.info.assert_called()

    def test_init_creates_confirmation_manager(self):
        """ConfirmationManager keys both its network identity and its artifact
        filenames on the single vehicle sysid."""
        vehicle = _create_mock_vehicle()
        vehicle.source_system = 42
        controller = _create_controller(vehicle=vehicle)
        self.assertIsNotNone(controller.confirmation_manager)
        self.assertEqual(controller.confirmation_manager._sys_id, 42)
        self.assertFalse(hasattr(controller.confirmation_manager, "_log_sys_id"))

    def test_init_no_task_actor(self):
        """NavController starts without TaskActor (no network)."""
        controller = _create_controller()
        self.assertIsNone(controller.network.task_actor)

    def test_init_peer_navigation_false(self):
        """NavController starts with peer_navigation disabled."""
        controller = _create_controller()
        self.assertFalse(controller.navigation_task.peer_navigation)


# =============================================================================
# State Machine Transition Tests
# =============================================================================

class TestNavControllerStateTransitions(unittest.TestCase):
    """Tests for NavController state machine transitions."""

    def test_decide_onhold_manual_mode(self):
        """_decide stays ONHOLD in MANUAL mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.MANUAL)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.navigation_task.navigation_poi_location = poi.geo.truth_poi_location

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)

    def test_decide_onhold_when_disarmed(self):
        """_decide forces ONHOLD when vehicle is disarmed, even in an active mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, is_armed=False)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.DETECT
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.navigation_task.navigation_poi_location = poi.geo.truth_poi_location

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)

    def test_decide_onhold_stabilize_mode(self):
        """_decide stays ONHOLD in STABILIZE mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.STABILIZE)
        controller = _create_controller(vehicle=vehicle)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_decide_onhold_before_min_wp(self):
        """_decide stays ONHOLD before min waypoint in AUTO mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=2)
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_decide_detect_after_min_wp_auto(self):
        """_decide transitions to DETECT after min waypoint in AUTO mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_reset_homes_gimbal(self):
        """RESET (navigation task ended: passed POI / navigation failure) must stop
        tracking so the gimbal returns to its initial neutral position before
        the next DETECT — instead of lingering pointed at the old POI until
        the ~2s loss-recentre, or chaining to the next POI without homing."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, alt=200.0)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.RESET

        controller.decision.decide()

        controller.detector.stop_tracking.assert_called()
        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_detect_in_guided_mode(self):
        """_decide transitions to DETECT in GUIDED mode (no WP check)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=0)
        controller = _create_controller(vehicle=vehicle)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_detect_in_loiter_mode(self):
        """_decide transitions to DETECT in LOITER mode (no WP check)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.LOITER, next_wp=0)
        controller = _create_controller(vehicle=vehicle)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_recovery_below_min_alt(self):
        """_decide transitions to RECOVERY below min altitude."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=30.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)

    def test_decide_detect_at_min_alt(self):
        """_decide stays in DETECT at exactly min altitude."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=50.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_nav_with_confirmed_poi(self):
        """_decide transitions to NAV with confirmed POI."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.NAV)

    def test_decide_confirmed_final_approach_poi_waits_for_fresh_detection(self):
        """A delayed operator approval must not launch NAV on stale vision.

        This is the mechanism from GCS run 213125: the POI disappeared
        during the manual review, but the CONFIRMED status previously sent the
        vehicle straight into NAV and produced an infinite SNAP.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=True)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.CONFIRM
        controller.detections.detected_pois = []

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        navigation.final_approach.can_confirm_detection.assert_not_called()
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        vehicle.set_parameter.assert_not_called()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()

    def test_decide_confirmed_final_approach_poi_rejects_source_cache_without_event(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        poi = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(
            detections=[poi],
            primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = []
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(
            poi, ConfirmationStatus.CONFIRMED,
        )
        controller.phase.current = NavState.CONFIRM

        controller.sensor.sense()
        controller.decision.decide()
        controller.actions.act()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        navigation.nav.assert_not_called()
        vehicle.set_mode.assert_not_called()

    def test_decide_confirmed_final_approach_poi_accepts_fresh_tracked_orbit_frame(self):
        """A fresh visual releases an approved orbit even if preview is limited.

        During a human review the aircraft continues around the POI, so a
        tangent-orbit preview may be roll-limited while the gimbal still has a
        valid fresh lock. Bearing acquisition owns that NAV entry.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        poi = _stamp_fresh_source_poi(
            _create_detected_poi(obj_id=1)
        )
        detector = _create_mock_detector(detections=[poi])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([poi], primary_poi=poi)
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=False)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.CONFIRM
        controller.sensor.sense()

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.NAV)
        navigation.final_approach.can_confirm_detection.assert_not_called()
        navigation.final_approach.record_confirmed_detection.assert_not_called()

    def test_decide_confirmed_final_approach_poi_rejects_stale_source_event(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        poi = _stamp_fresh_source_poi(
            _create_detected_poi(obj_id=1, task_id=11)
        )
        poi.replace_timing(replace(
            poi.timing,
            detection_now_s=lambda: 21.1,
            source_receipt_now_s=lambda: 1001.1,
        ))
        detector = _create_mock_detector(
            detections=[poi],
            primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([poi], primary_poi=poi)
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(
            poi, ConfirmationStatus.CONFIRMED,
        )
        controller.phase.current = NavState.CONFIRM

        controller.sensor.sense()
        controller.decision.decide()
        controller.actions.act()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        navigation.nav.assert_not_called()
        vehicle.set_mode.assert_not_called()

    def test_manual_source_decide_then_act_records_newest_event_once(self):
        first = _stamp_fresh_source_poi(
            _create_detected_poi(obj_id=1, task_id=11),
            frame_timestamp_s=20.00,
            receipt_timestamp_s=1000.00,
        )
        latest = _stamp_fresh_source_poi(
            _create_detected_poi(obj_id=1, task_id=11),
            frame_timestamp_s=20.02,
            receipt_timestamp_s=1000.02,
        )
        detector = _create_mock_detector(
            detections=[latest],
            primary_poi=latest,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [first],
                primary_poi=first,
                source_timestamp_s=20.00,
            ),
            _source_event(
                [latest],
                primary_poi=latest,
                source_timestamp_s=20.02,
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(latest)
        controller.confirmation_manager.update_status(
            latest, ConfirmationStatus.CONFIRMED,
        )
        controller.phase.current = NavState.CONFIRM
        controller.phase.previous = NavState.CONFIRM

        controller.sensor.sense()
        controller.decision.decide()
        controller.actions.act()

        self.assertEqual(controller.phase.current, NavState.NAV)
        navigation.final_approach.record_confirmed_detection.assert_called_once_with(
            latest
        )
        navigation.nav.assert_called_once_with(latest)

    def test_coordinate_pass_does_not_reset_while_confirmation_is_pending(self):
        """Pass/reset state starts only after final-approach NAV is active."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        navigation = _create_mock_navigation()
        navigation.final_approach.poi_passed_override = Mock(return_value=None)
        navigation.legacy_pois.locked_distance = Mock(side_effect=[30.0, 40.0])
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()
        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)

    def test_decide_confirm_with_confirming_poi(self):
        """_decide transitions to CONFIRM while waiting for confirmation."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)

    def test_decide_detect_after_rejection(self):
        """_decide clears POI and returns to DETECT after rejection."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.navigation_task.navigation_poi_location = poi.geo.truth_poi_location

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)

    def test_decide_detect_with_no_active_poi(self):
        """_decide stays in DETECT with no active POI."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_aborts_confirm_to_detect_on_reacquire_timeout(self):
        """A pre-request CONFIRM POI continuously absent past
        CONFIRM_REACQUIRE_ABORT_SEC aborts to DETECT (mirrors REJECTED: clear
        active POI + stop_tracking) and registers a re-acquire cooldown."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        # POI absent this tick (no detections) and the continuous-absence
        # run began long enough ago; status is still None (pre-request).
        controller.detections.detected_pois = []
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC + 0.5)
        )

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called()
        # Cooldown registered so DETECT does not instantly re-lock it.
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))

    def test_decide_stays_in_confirm_before_reacquire_timeout(self):
        """A pre-request CONFIRM POI absent only briefly stays in CONFIRM
        (continuous-absence run below the bound)."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC - 0.5)
        )

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_confirm_poi_seen_this_tick_is_never_aborted(self):
        """Refresh-before-check: a POI detected THIS tick clears the absence
        accumulator in _decide, so a stale accumulator can't abort it."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        # Present this tick despite a very stale (pre-set) absence start.
        controller.detections.detected_pois = [poi]
        controller.confirm.loss_started_at = controller.clock.decision_s() - 100.0

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        # Accumulator was reset by the this-tick presence refresh.
        self.assertIsNone(controller.confirm.loss_started_at)

    def test_confirming_poi_not_aborted_by_reacquire_path(self):
        """A CONFIRMING (under-review) POI is exempt from the 2.5s
        pre-request abort even when continuously absent well past the bound."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)
        controller.detections.detected_pois = []
        controller.confirm.loss_started_at = controller.clock.decision_s() - 100.0
        # Review only just started — nowhere near the absolute cap. The cap is
        # measured on the wall clock, so stamp it there.
        controller.confirm.review_started_at = controller.clock.wall_s()

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_confirming_poi_aborted_by_max_dwell_cap(self):
        """A CONFIRMING POI that overruns CONFIRM_MAX_DWELL_SEC (wall clock)
        is torn down (backstop) and registers a re-acquire cooldown."""
        from navpy.modules.nav.nav_controller import CONFIRM_MAX_DWELL_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)
        controller.confirm.review_started_at = (
            controller.clock.wall_s() - (CONFIRM_MAX_DWELL_SEC + 1.0)
        )

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called()
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))

    def test_confirm_loss_accumulator_flicker_does_not_abort(self):
        """A POI that reappears each tick never accumulates a continuous
        absence run, so the re-acquire abort never fires despite a long total
        elapsed time."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        controller = _create_controller()
        fake = [1000.0]
        controller.clock.decision_s = lambda: fake[0]

        # present, absent, present, absent, present ... each < bound apart
        for present in (True, False, True, False, True, False):
            controller.timing.update_loss_tracking(present)
            fake[0] += CONFIRM_REACQUIRE_ABORT_SEC - 0.5
            self.assertFalse(controller.timing.reacquire_timed_out())

    def test_confirm_loss_accumulator_sustained_absence_aborts(self):
        """A sustained absence run (never returning) accumulates to the bound
        and trips the re-acquire timeout."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        controller = _create_controller()
        fake = [1000.0]
        controller.clock.decision_s = lambda: fake[0]

        controller.timing.update_loss_tracking(True)   # present
        controller.timing.update_loss_tracking(False)  # present->absent edge
        self.assertFalse(controller.timing.reacquire_timed_out())
        fake[0] += CONFIRM_REACQUIRE_ABORT_SEC + 0.1
        controller.timing.update_loss_tracking(False)  # still absent
        self.assertTrue(controller.timing.reacquire_timed_out())

    def test_confirm_reacquire_state_reset_on_entry_and_exit(self):
        """CONFIRM entry starts the absence accumulator + review clock fresh
        (None, armed lazily per-tick); exit clears them."""
        controller = _create_controller()
        controller.confirm.loss_started_at = 123.0
        controller.confirm.review_started_at = 123.0
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        self.assertIsNone(controller.confirm.loss_started_at)
        self.assertIsNone(controller.confirm.review_started_at)
        controller.confirm.loss_started_at = 456.0
        controller.confirm.review_started_at = 456.0
        controller.transitions.on_change(NavState.CONFIRM, NavState.DETECT)
        self.assertIsNone(controller.confirm.loss_started_at)
        self.assertIsNone(controller.confirm.review_started_at)

    def test_confirm_reacquire_timed_out_false_when_present(self):
        controller = _create_controller()
        controller.confirm.loss_started_at = None
        self.assertFalse(controller.timing.reacquire_timed_out())

    def test_aborted_identity_in_cooldown_skipped_by_select_poi(self):
        """After an abort, _select_poi skips the cooled-down identity so
        DETECT keeps scanning; the entry expires after the cooldown window."""
        from navpy.modules.nav.nav_controller import POI_REACQUIRE_COOLDOWN_SEC
        controller = _create_controller()
        # The cooldown is a WALL-clock bound, so drive the wall accessor.
        fake = [1000.0]
        controller.clock.wall_s = lambda: fake[0]

        poi = _create_detected_poi(obj_id=7)
        controller.retry_policy.register_poi_cooldown(poi)

        # Same identity re-detected: skipped (not a self-POI candidate).
        fresh = _create_detected_poi(obj_id=7)
        self_poi, _ = controller.selector.select([fresh])
        self.assertIsNone(self_poi)

        # After the cooldown window it is selectable again.
        fake[0] += POI_REACQUIRE_COOLDOWN_SEC + 0.1
        self_poi, _ = controller.selector.select([fresh])
        self.assertIs(self_poi, fresh)

    def test_aborted_identity_in_cooldown_skipped_by_handle_new_poi(self):
        """_handle_new_poi does not reselect a cooled-down identity."""
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=7)
        controller.retry_policy.register_poi_cooldown(poi)

        fresh = _create_detected_poi(obj_id=7)
        controller.navigation_task_action.handle_new_poi(fresh)

        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_confirm_dwell_cap_is_at_least_auto_confirm_timeout(self):
        """The absolute CONFIRM cap must never pre-empt the operator/auto
        confirm response, and the pre-request abort must be well below it."""
        from navpy.modules.nav.nav_controller import (
            CONFIRM_MAX_DWELL_SEC,
            CONFIRM_REACQUIRE_ABORT_SEC,
        )
        # AAS_NAV_CWT default (auto-confirm response timeout) is 30s; demo runs
        # use up to 40s. The cap must be >= that operator timeout.
        self.assertGreaterEqual(CONFIRM_MAX_DWELL_SEC, 40.0)
        self.assertLess(CONFIRM_REACQUIRE_ABORT_SEC, CONFIRM_MAX_DWELL_SEC)

    def test_effective_max_dwell_rises_with_confirm_wait_time(self):
        """The effective cap must outlast AAS_NAV_CWT even when the GS raises it
        above the 45s floor (the UI allows up to 60s), so a legitimate review is
        never torn down; and it must fall back to the floor for small/None CWT."""
        from navpy.modules.nav.nav_controller import (
            CONFIRM_MAX_DWELL_SEC,
            CONFIRM_DWELL_MARGIN_SEC,
        )
        controller = _create_controller()
        # Small CWT -> floor applies.
        controller.args.confirm_wait_time_sec = 30.0
        self.assertEqual(controller.timing.max_dwell_sec(), CONFIRM_MAX_DWELL_SEC)
        # None (pre-refresh) -> floor applies, no crash.
        controller.args.confirm_wait_time_sec = None
        self.assertEqual(controller.timing.max_dwell_sec(), CONFIRM_MAX_DWELL_SEC)
        # CWT above the floor -> cap rises to outlast it by the margin.
        controller.args.confirm_wait_time_sec = 60.0
        self.assertEqual(
            controller.timing.max_dwell_sec(), 60.0 + CONFIRM_DWELL_MARGIN_SEC
        )
        self.assertGreater(controller.timing.max_dwell_sec(), 60.0)

    def test_wall_s_is_independent_of_scheduler_cadence(self):
        """Decision clocks stay wall-based; speedup changes cadence only."""
        controller = _create_controller()
        controller.scheduler_cadence = Mock(now=Mock(return_value=9.0e9))
        self.assertLess(controller.clock.decision_s(), 1.0e9)
        self.assertLess(controller.clock.wall_s(), 1.0e9)
        self.assertAlmostEqual(controller.clock.decision_s(), time.monotonic(), delta=2.0)
        self.assertAlmostEqual(controller.clock.wall_s(), time.monotonic(), delta=2.0)
        controller.scheduler_cadence.now.assert_not_called()

    def test_max_dwell_review_stamp_is_wall_clock_under_speedup(self):
        """The CONFIRM_MAX_DWELL_SEC review clock is stamped on the wall clock,
        so a fast sim clock (speedup) cannot collapse the cap. If it used the
        sim clock the stamp would be ~9e9 (and the cap would fire in wall ms)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.scheduler_cadence = Mock(now=Mock(return_value=9.0e9))

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)

        controller.decision.decide()

        # Under review, well inside the cap -> stays CONFIRM.
        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        # Stamped on the wall clock (near monotonic), NOT the 9e9 sim clock.
        self.assertIsNotNone(controller.confirm.review_started_at)
        self.assertLess(controller.confirm.review_started_at, 1.0e9)
        self.assertAlmostEqual(
            controller.confirm.review_started_at, time.monotonic(), delta=2.0)

    def test_cooldown_expiry_is_wall_clock_under_speedup(self):
        """The re-acquire cooldown expiry is stamped on the wall clock, so a
        fast sim clock cannot make it expire immediately."""
        from navpy.modules.nav.nav_controller import POI_REACQUIRE_COOLDOWN_SEC
        controller = _create_controller()
        controller.scheduler_cadence = Mock(now=Mock(return_value=9.0e9))

        poi = _create_detected_poi(obj_id=9)
        controller.retry_policy.register_poi_cooldown(poi)

        key = controller.retry_policy.poi_cooldown_key(poi)
        expiry = controller.retry_state.cooldowns[key]
        # Wall-clock stamp: ~monotonic + window, NOT 9e9 + window.
        self.assertLess(expiry, 1.0e9)
        self.assertAlmostEqual(
            expiry, time.monotonic() + POI_REACQUIRE_COOLDOWN_SEC, delta=2.0)
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))

    def test_class0_recognition_gate_relaxed_to_36(self):
        """The dock (class-0) recognition gate resolves to 36px."""
        from navpy.modules.vision.vision_profiles import (
            get_min_pixels_for_class,
            resolve_profile,
        )
        _, profile, _ = resolve_profile("siyi_zr10", Mock())
        self.assertEqual(get_min_pixels_for_class(profile, 0), 36.0)


# =============================================================================
# Loss Geo-Hold Tests (TRACK-01 / TRACK-02)
# =============================================================================

class TestNavControllerGeoHold(unittest.TestCase):
    """Loss geo-hold orchestration: entry, suspension, timeout boundary,
    clock domain, no-geo carve-out, and the vision-nav gate."""

    def test_loss_with_known_geo_enters_geo_hold_without_recenter(self):
        """Absence past loss_hold_sec with a held geo hands the gimbal from
        detection-tracking to the geo-follow surface with NO FOLLOW/neutral
        recentre (D-01/D-02)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.navigation_task.navigation_poi_location = held_geo
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (detector.loss_hold_sec + 0.1)
        )

        controller.decision.decide()

        self.assertTrue(controller.geo_hold.active)
        self.assertEqual(controller.geo_hold.poi_location, held_geo)
        detector.stop_tracking.assert_called_once_with(to_neutral=False)
        detector.start_geo_tracking.assert_called_once_with(
            held_geo, navigation.legacy_pois.geo_ref,
        )
        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_geo_hold_suspends_confirm_reacquire_abort(self):
        """While the hold is active, the 2.5s CONFIRM_REACQUIRE_ABORT_SEC
        abort does NOT fire even though its bound has been exceeded (D-09):
        the hold's own timeout owns the loss window instead."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        controller = _create_controller()
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC + 0.5)
        )

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        self.assertTrue(controller.geo_hold.active)

    def test_geo_hold_timeout_boundary_fires_same_abort_teardown(self):
        """One decide tick before TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC the hold
        persists; at/after expiry the SAME abort teardown fires (mirrors
        test_decide_aborts_confirm_to_detect_on_reacquire_timeout)."""
        from navpy.modules.nav.nav_controller import TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []

        # One tick before the bound: hold persists, no teardown.
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC - 0.5)
        )
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        self.assertTrue(controller.geo_hold.active)
        detector.stop_tracking.assert_not_called()

        # At/after the bound: same abort teardown as the legacy path.
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC + 0.5)
        )
        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called()
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))
        self.assertFalse(controller.geo_hold.active)

    def test_geo_hold_timeout_runs_on_navigation_task_clock(self):
        """The geo-hold timeout is driven by the navigation task clock (clock.decision_s),
        not the wall clock — D-08."""
        from navpy.modules.nav.nav_controller import TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC
        controller = _create_controller()
        controller.geo_hold.active = True
        fake = [1000.0]
        controller.clock.decision_s = lambda: fake[0]
        controller.confirm.loss_started_at = fake[0]

        fake[0] += TRACK_LOSS_GEO_HOLD_TIMEOUT_SEC - 1.0
        self.assertFalse(controller.timing.reacquire_timed_out())

        fake[0] += 2.0  # crosses the bound on the navigation task clock
        self.assertTrue(controller.timing.reacquire_timed_out())

    def test_loss_without_geo_keeps_legacy_ladder_and_abort(self):
        """With no known geo anywhere, the hold never enters and the
        original 2.5s pre-request abort fires exactly as today (D-01
        carve-out)."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        self.assertIsNone(controller.geo_hold.last_own_poi_geo)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC + 0.5)
        )

        controller.decision.decide()

        self.assertFalse(controller.geo_hold.active)
        detector.start_geo_tracking.assert_not_called()
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called_once_with()

    def test_geo_hold_never_activates_under_vision_nav(self):
        """A geo is present but uses_vision_nav=True: the
        hold never enters (D-03 data gate + explicit defense-in-depth
        mode gate)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        controller.navigation_task.navigation_poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (detector.loss_hold_sec + 0.1)
        )

        controller.decision.decide()

        self.assertFalse(controller.geo_hold.active)
        detector.start_geo_tracking.assert_not_called()
        detector.stop_tracking.assert_not_called()

    def test_held_geo_prefers_last_own_detection_over_assigned_coord(self):
        """D-04: a refreshed own-detection geo wins over the assigned
        task/auction coordinate; falls back to the assigned coordinate when
        no own geo was ever captured."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        assigned = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.navigation_task.navigation_poi_location = assigned

        # No own-detection geo captured yet -> falls back to the assigned
        # coordinate.
        self.assertIs(
            controller.track_recovery.held_poi_geo(
                controller.navigation_task.navigation_poi_location
            ),
            assigned,
        )

        # A present detection carrying its own p_t_g_l is captured as the
        # last-good own-detection geo (D-04) and takes priority thereafter.
        own_geo = Location(40.6, -74.6, 950.0, is_absolute=True)
        poi = _create_detected_poi(obj_id=1, p_t_g_l=own_geo)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = [poi]

        controller.decision.decide()

        self.assertEqual(controller.geo_hold.last_own_poi_geo, own_geo)
        self.assertIs(
            controller.track_recovery.held_poi_geo(
                controller.navigation_task.navigation_poi_location
            ),
            controller.geo_hold.last_own_poi_geo,
        )

    def test_geo_hold_reacquire_rearms_detection_without_recenter(self):
        """D-11 gap-path reacquire: a present detection during the hold
        re-arms detection via start_tracking (no recentre); stop_geo_tracking
        must NOT be called on this path (that would recentre)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1, task_id=7)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = [poi]

        controller.decision.decide()

        detector.start_tracking.assert_called_once_with(7)
        detector.stop_geo_tracking.assert_not_called()
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

    def test_geo_hold_exit_on_confirmed_rearms_before_nav(self):
        """CONFIRMED while the hold is active (legacy PN only): detection is
        re-armed BEFORE releasing into NAV, so NAV never starts with a
        live geo session pointing the gimbal."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = False
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1, task_id=9)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.detections.detected_pois = []  # still absent when confirmation arrives

        controller.decision.decide()

        detector.start_tracking.assert_called_once_with(9)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertEqual(controller.phase.current, NavState.NAV)

    def test_geo_hold_cleared_on_rejected_teardown(self):
        """REJECTED while the hold is active: _resume_auto_mission's
        existing stop_geo_tracking() recenters to search pose and Task 1's
        stale-state-clear covers the hold fields; REJECTED does NOT
        register a re-acquire cooldown (unchanged from today)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.detections.detected_pois = []

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_geo_tracking.assert_called()
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertFalse(controller.retry_policy.is_in_poi_cooldown(poi))

    def test_geo_hold_tick_issues_update_geo_and_acquisition_zoom(self):
        """The hold tick in _act_confirm calls update_geo with the vehicle
        pose, and prepare_geo_acquisition when the held geo is in frame and
        under the class's min-pixel gate (D-12)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        uav_loc_absolute = Location(40.31, -74.41, 1700.0, is_absolute=True)
        vehicle.location = Mock(side_effect=lambda is_relative: (
            None if is_relative else uav_loc_absolute
        ))
        detector = _create_mock_detector()
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = True
        detector.mounts = [mount]
        navigation = _create_mock_navigation()
        navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        poi = _create_detected_poi(obj_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(poi)

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            controller.confirmation_action.act()

        detector.update_geo.assert_called_once_with(uav_loc_absolute, vehicle.attitude)
        detector.prepare_geo_acquisition.assert_called_once()
        args = detector.prepare_geo_acquisition.call_args.args
        self.assertIs(args[0], uav_loc_absolute)
        self.assertIs(args[1], vehicle.attitude)
        self.assertEqual(args[2], 0)

    def test_identity_expired_reacquire_binds_within_gate_radius(self):
        """D-11 second branch: after the tracker's own gap-path identity has
        expired (the OLD task id is absent this tick), a same-class
        candidate whose geo-ref is just INSIDE TRACK_REACQUIRE_GATE_RADIUS_M
        re-binds the ORIGINAL navigation task via the allocator rebind and
        resumes tracking on the ORIGINAL task id — not a fresh one."""
        from navpy.modules.nav.nav_controller import TRACK_REACQUIRE_GATE_RADIUS_M
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(return_value=True)
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.poi_location = held_geo

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirm.loss_started_at = controller.clock.decision_s() - 5.0

        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=0,
            p_t_g_l=Location(40.5001, -74.5001, 900.0, is_absolute=True),
        )
        controller.detections.detected_pois = [candidate]

        with patch(
            "navpy.modules.nav.track_recovery.GeoRefCalc.calculate_distance",
            return_value=TRACK_REACQUIRE_GATE_RADIUS_M - 1.0,
        ):
            controller.decision.decide()

        detector.rebind_task_id.assert_called_once_with(1, candidate)
        detector.start_tracking.assert_called_once_with(1)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_identity_expired_reacquire_rejects_outside_gate_radius(self):
        """Boundary pair with the INSIDE case: a same-class candidate just
        OUTSIDE TRACK_REACQUIRE_GATE_RADIUS_M never rebinds; the hold
        persists exactly as before."""
        from navpy.modules.nav.nav_controller import TRACK_REACQUIRE_GATE_RADIUS_M
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(return_value=True)
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.poi_location = held_geo

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirm.loss_started_at = controller.clock.decision_s() - 5.0

        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=0,
            p_t_g_l=Location(40.501, -74.501, 900.0, is_absolute=True),
        )
        controller.detections.detected_pois = [candidate]

        with patch(
            "navpy.modules.nav.track_recovery.GeoRefCalc.calculate_distance",
            return_value=TRACK_REACQUIRE_GATE_RADIUS_M + 1.0,
        ):
            controller.decision.decide()

        detector.rebind_task_id.assert_not_called()
        detector.start_tracking.assert_not_called()
        self.assertTrue(controller.geo_hold.active)
        self.assertEqual(controller.geo_hold.poi_location, held_geo)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

    def test_identity_expired_reacquire_rejects_wrong_class(self):
        """A near, identity-expired candidate of a DIFFERENT class never
        rebinds regardless of geo distance (wrong-identity guard)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(return_value=True)
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.poi_location = held_geo

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirm.loss_started_at = controller.clock.decision_s() - 5.0

        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=1,  # different class
            p_t_g_l=Location(40.5001, -74.5001, 900.0, is_absolute=True),
        )
        controller.detections.detected_pois = [candidate]

        controller.decision.decide()

        detector.rebind_task_id.assert_not_called()
        self.assertTrue(controller.geo_hold.active)
        self.assertEqual(controller.geo_hold.poi_location, held_geo)

    def test_identity_expired_reacquire_respects_cooldown(self):
        """A near, same-class candidate whose frame-local identity is under
        its own re-lock cooldown never rebinds (wrong-identity guard)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(return_value=True)
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.poi_location = held_geo

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirm.loss_started_at = controller.clock.decision_s() - 5.0

        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=0,
            p_t_g_l=Location(40.5001, -74.5001, 900.0, is_absolute=True),
        )
        controller.retry_policy.register_poi_cooldown(candidate)
        controller.detections.detected_pois = [candidate]

        controller.decision.decide()

        detector.rebind_task_id.assert_not_called()
        self.assertTrue(controller.geo_hold.active)
        self.assertEqual(controller.geo_hold.poi_location, held_geo)

    def test_rebind_preserves_status_continuity(self):
        """D-11 continuity: a status set for the ORIGINAL task id before the
        loss still governs the re-bound detection after identity-expiry
        reacquire (ConfirmationManager keys status purely by task_id, which the
        allocator rebind adopts onto the new track)."""
        from navpy.modules.nav.nav_controller import TRACK_REACQUIRE_GATE_RADIUS_M

        def _fake_rebind(task_id, poi):
            poi.reidentify(task_id=task_id)
            return True

        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(side_effect=_fake_rebind)
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.poi_location = held_geo

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirmation_manager.update_status(active, ConfirmationStatus.CONFIRMING)
        controller.confirm.loss_started_at = controller.clock.decision_s() - 5.0

        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=0,
            p_t_g_l=Location(40.5001, -74.5001, 900.0, is_absolute=True),
        )
        controller.detections.detected_pois = [candidate]

        with patch(
            "navpy.modules.nav.track_recovery.GeoRefCalc.calculate_distance",
            return_value=TRACK_REACQUIRE_GATE_RADIUS_M - 1.0,
        ):
            controller.decision.decide()

        self.assertEqual(candidate.identity.task_id, 1)
        self.assertEqual(
            controller.confirmation_manager.get_status(candidate), ConfirmationStatus.CONFIRMING,
        )
        self.assertFalse(controller.geo_hold.active)

    def test_geo_hold_full_cycle_loss_expiry_rebind_confirm(self):
        """Task 2(c) end-to-end D-11 walk across three real ``_decide()``
        ticks: loss -> hold entry -> identity-expiry -> geo-gate re-bind ->
        confirmation flow proceeds (the ORIGINAL task id resolves to a
        live, present detection again)."""
        from navpy.modules.nav.nav_controller import TRACK_REACQUIRE_GATE_RADIUS_M

        def _fake_rebind(task_id, poi):
            poi.reidentify(task_id=task_id)
            return True

        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.rebind_task_id = Mock(side_effect=_fake_rebind)
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        active = _create_detected_poi(obj_id=1, task_id=1, class_id=0)
        controller.confirmation_manager.set_active_poi(active)
        controller.confirmation_manager.update_status(active, ConfirmationStatus.CONFIRMING)
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.navigation_task.navigation_poi_location = held_geo
        controller.detections.detected_pois = []

        # Tick 1: loss past loss_hold_sec with a known geo enters the hold.
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (detector.loss_hold_sec + 0.1)
        )
        controller.decision.decide()
        self.assertTrue(controller.geo_hold.active)
        detector.start_geo_tracking.assert_called_once_with(held_geo, navigation.legacy_pois.geo_ref)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

        # Tick 2: the OLD local identity has expired; a same-class candidate
        # within the gate radius re-enters as a NEW local id/task id.
        candidate = _create_detected_poi(
            obj_id=2, task_id=2, class_id=0,
            p_t_g_l=Location(40.5001, -74.5001, 900.0, is_absolute=True),
        )
        controller.detections.detected_pois = [candidate]
        with patch(
            "navpy.modules.nav.track_recovery.GeoRefCalc.calculate_distance",
            return_value=TRACK_REACQUIRE_GATE_RADIUS_M - 1.0,
        ):
            controller.decision.decide()

        self.assertFalse(controller.geo_hold.active)
        detector.start_tracking.assert_called_once_with(1)
        self.assertEqual(candidate.identity.task_id, 1)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

        # Tick 3: confirmation flow proceeds -- the ORIGINAL task id
        # resolves to a live, present detection again (status request
        # reachable on the SAME navigation task, no duplicate task created).
        controller.detections.detected_pois = [candidate]
        fresh = controller.source.find_active_poi_detection()
        self.assertIs(fresh, candidate)
        self.assertEqual(
            controller.confirmation_manager.get_status(active), ConfirmationStatus.CONFIRMING,
        )
        self.assertEqual(
            controller.confirmation_manager.get_status(candidate), ConfirmationStatus.CONFIRMING,
        )

    def test_peer_preacq_and_geo_hold_are_mutually_exclusive(self):
        """Peer pre-acquisition (no active POI, DETECT) and the loss
        geo-hold (active POI, CONFIRM) are dispatched by different,
        mutually exclusive NavState branches in _act() — even if both
        mechanisms' state happened to be set at once, only ONE tick block
        fires per _act() call (proven here by a single update_geo call)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        uav_loc_absolute = Location(40.31, -74.41, 1700.0, is_absolute=True)
        vehicle.location = Mock(side_effect=lambda is_relative: (
            None if is_relative else uav_loc_absolute
        ))
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        # Artificially set BOTH mechanisms' state at once — a regression
        # that let both tick blocks fire in the same _act() call would
        # double the update_geo count below.
        controller.navigation_task.peer_navigation = True
        controller.geo_hold.poi_location = Location(41.0, -75.0, 500.0, is_absolute=True)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.phase.current = NavState.CONFIRM
        controller.phase.previous = NavState.CONFIRM

        controller.actions.act()

        detector.update_geo.assert_called_once_with(uav_loc_absolute, vehicle.attitude)


class TestNavControllerGeoHoldFailClosed(unittest.TestCase):
    """Geo-hold is an enhancement gated on capable hardware: when the
    capability is absent or arming/rearm fails, behavior must FALL BACK to
    the pre-existing 2.5 s CONFIRM_REACQUIRE_ABORT_SEC teardown — never a
    dead 180 s window, never a stuck state (PR #221 review findings)."""

    def test_loss_hold_sec_none_keeps_legacy_abort_without_error(self):
        """A base/direct Detector with no rate-tracking GimbalNavigation
        reports loss_hold_sec None. _decide() must not raise (a TypeError
        here escapes _decide, gets swallowed by _nav_loop, and strands the
        controller in CONFIRM forever); no geo-hold enters and the legacy
        2.5 s abort teardown fires exactly as pre-geo-hold."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.loss_hold_sec = None  # no geo-hold-capable navigation
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        # A held geo IS available — capability absence alone must gate.
        controller.navigation_task.navigation_poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC + 0.5)
        )

        controller.decision.decide()  # must not raise

        self.assertFalse(controller.geo_hold.active)
        detector.start_geo_tracking.assert_not_called()
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called_once_with()

    def test_silent_arm_decline_restores_detection_and_keeps_legacy_abort(self):
        """start_geo_tracking can decline without raising (GimbalNavigation
        logs 'ignored — no rate tracker' and returns; is_geo_armed stays
        False) — reachable via a DetectionCoordinator whose fallback
        loss_hold_sec is non-None while no child can geo-arm. The hold must
        NOT be declared, and detection tracking (torn down by the entry's
        stop_tracking(to_neutral=False) BEFORE the arm attempt) must be
        RESTORED — otherwise a POI returning before the 2.5 s abort
        leaves a live navigation task with nothing tracking. The legacy abort
        stays in charge."""
        from navpy.modules.nav.nav_controller import CONFIRM_REACQUIRE_ABORT_SEC
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        # Silent no-op arm: call succeeds but is_geo_armed never flips True.
        detector.start_geo_tracking = Mock()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        controller.navigation_task.navigation_poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (detector.loss_hold_sec + 0.1)
        )

        controller.decision.decide()

        detector.start_geo_tracking.assert_called_once()
        detector.stop_tracking.assert_called_once_with(to_neutral=False)
        # Rearm-or-abort: detection tracking restored for the active obj.
        detector.start_tracking.assert_called_once_with(1)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

        # Loss run reaches the LEGACY bound: the 2.5 s abort fires (not the
        # 180 s geo-hold window).
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (CONFIRM_REACQUIRE_ABORT_SEC + 0.5)
        )
        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertFalse(controller.geo_hold.active)

    def test_silent_arm_decline_with_failed_restore_aborts_navigation_task(self):
        """Silent geo-arm decline AND the detection restore also failing:
        never return leaving a live navigation task in CONFIRM with nothing
        tracking — fire the SAME proven CONFIRM-reacquire abort teardown
        immediately (abort to DETECT, mission resume, cooldown)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.start_geo_tracking = Mock()  # silent decline
        detector.start_tracking = Mock(
            side_effect=RuntimeError("gimbal LOCK failed"),
        )
        controller = _create_controller(vehicle=vehicle, detector=detector)
        resume_spy = Mock(side_effect=controller.auto_resume.run)
        controller.auto_resume.run = resume_spy

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = []
        controller.navigation_task.navigation_poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )
        controller.confirm.loss_started_at = (
            controller.clock.decision_s() - (detector.loss_hold_sec + 0.1)
        )

        controller.decision.decide()  # must not raise

        # Single restore attempt — no retries/polling.
        detector.start_tracking.assert_called_once_with(1)
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertIsNone(controller.confirm.loss_started_at)
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))
        resume_spy.assert_called_once()

    def test_reacquire_rearm_failure_aborts_navigation_task(self):
        """start_tracking raising during the gap-path reacquire must not
        escape _decide and must not linger in CONFIRM with neither geo nor
        detection tracking armed: detections keep flowing (POI present),
        so no loss timeout would ever re-arm. Rearm-or-abort: a second
        start_tracking attempt is pointless (it just raised) — go straight
        to the SAME proven abort teardown."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.start_tracking = Mock(
            side_effect=RuntimeError("gimbal LOCK failed"),
        )
        logger = Mock()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, logger=logger,
        )
        resume_spy = Mock(side_effect=controller.auto_resume.run)
        controller.auto_resume.run = resume_spy
        held_geo = Location(40.5, -74.5, 900.0, is_absolute=True)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = held_geo
        controller.navigation_task.navigation_poi_location = held_geo

        poi = _create_detected_poi(obj_id=1, task_id=7)
        controller.confirmation_manager.set_active_poi(poi)
        controller.detections.detected_pois = [poi]

        controller.decision.decide()  # must not raise

        # Single attempt — no pointless retry of the call that just raised.
        detector.start_tracking.assert_called_once_with(7)
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.poi_location)
        self.assertIsNone(controller.confirm.loss_started_at)
        self.assertTrue(controller.retry_policy.is_in_poi_cooldown(poi))
        resume_spy.assert_called_once()
        logger.warning.assert_called()

    def test_confirmed_reacquire_rearm_failure_never_advances_to_nav(self):
        """CONFIRMED while the hold is active and the reacquire rearm
        raises: never release into NAV with nothing tracking — the abort
        teardown fires instead (second _reacquire_geo_hold call site)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        detector.start_tracking = Mock(
            side_effect=RuntimeError("gimbal LOCK failed"),
        )
        controller = _create_controller(vehicle=vehicle, detector=detector)
        controller.geo_hold.active = True
        controller.geo_hold.poi_location = Location(
            40.5, -74.5, 900.0, is_absolute=True,
        )

        poi = _create_detected_poi(obj_id=1, task_id=9)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.detections.detected_pois = []  # still absent when confirmation arrives

        controller.decision.decide()  # must not raise

        detector.start_tracking.assert_called_once_with(9)
        self.assertNotEqual(controller.phase.current, NavState.NAV)
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertFalse(controller.geo_hold.active)


# =============================================================================
# State Transition Handler Tests
# =============================================================================

class TestNavControllerStateTransitionHandlers(unittest.TestCase):
    """Tests for _on_state_change behavior (called from _act)."""

    def test_entering_nav_inits_navigation(self):
        """Entering NAV state initializes navigation."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.phase.current = NavState.DETECT  # Previous state
        _decide_and_act(controller)

        navigation.init.assert_called_once()

    def test_entering_nav_sets_guided_mode(self):
        """Entering NAV state sets vehicle to GUIDED mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.phase.current = NavState.DETECT
        _decide_and_act(controller)

        vehicle.set_mode.assert_called_with(FlightMode.GUIDED)

    def test_staying_in_nav_does_not_reinit(self):
        """Staying in NAV state does not reinitialize navigation."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.phase.current = NavState.NAV  # Already in NAV
        controller.phase.previous = NavState.NAV  # Sync prev_state
        _decide_and_act(controller)

        navigation.init.assert_not_called()

    def test_mode_switch_from_guided_resets_pois(self):
        """Switching away from GUIDED mode does full _clear_state cleanup."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV
        # Reaching NAV implies dispatch happened and GUIDED was observed
        # at least once; the latch is required for the abort guard to fire.
        controller.navigation_task.nav_mode_observed = True

        # Now mode switches to AUTO
        controller.decision.decide()

        # POI manager should be reset
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIsNone(controller.confirmation_manager.get_status(poi))
        # Full cleanup should have occurred
        self.assertFalse(controller.navigation_task.peer_navigation)
        self.assertFalse(controller.mission.default_delivery_hub_active)
        self.assertEqual(controller.detections.detected_pois, [])

    def test_mode_switch_from_guided_resets_peer_navigation(self):
        """Switching away from GUIDED mode resets peer_navigation flag."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        controller.navigation_task.peer_navigation = True
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = True

        controller.decision.decide()

        self.assertFalse(controller.navigation_task.peer_navigation)

    def test_mode_switch_resets_task_actor(self):
        """Mode switch from GUIDED calls task_actor.reset()."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.network.task_actor = Mock()

        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = True
        controller.decision.decide()

        controller.network.task_actor.reset.assert_called_once()

    def test_nav_waits_and_recommands_guided_within_timeout(self):
        """Before GUIDED is observed, NAV re-commands GUIDED and holds (the
        regression the PR fixed) — it must NOT abort on the first tick."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = False
        controller.navigation_task.guided_request_started_at = None

        with patch.object(controller.vehicle_navigation, "request_guided") as set_guided:
            controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.NAV)
        set_guided.assert_called_once()
        self.assertIsNotNone(controller.navigation_task.guided_request_started_at)

    def test_nav_retries_unacknowledged_request_at_bounded_cadence(self):
        """A missing GUIDED heartbeat is retried without flooding SET_MODE."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = False
        controller.navigation_task.guided_request_started_at = None
        controller.navigation_task.guided_last_attempt_at = None

        with patch.object(
            controller.clock,
            "decision_s",
            side_effect=(10.0, 10.5, 11.0),
        ), patch.object(
            controller.vehicle_navigation,
            "request_guided",
            return_value=True,
        ) as set_guided:
            controller.decision.decide()
            controller.decision.decide()
            controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.NAV)
        self.assertEqual(set_guided.call_count, 2)
        self.assertEqual(controller.navigation_task.guided_request_started_at, 10.0)
        self.assertEqual(controller.navigation_task.guided_last_attempt_at, 11.0)

    def test_nav_aborts_when_guided_not_accepted_within_timeout(self):
        """If the autopilot never accepts GUIDED, the wait is bounded: abort to
        DETECT instead of re-commanding forever with no end."""
        from navpy.modules.nav.nav_controller import GUIDED_ACCEPT_TIMEOUT_S
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = False
        controller.navigation_task.guided_request_started_at = 0.0

        with patch.object(controller.clock, "decision_s",
                          return_value=GUIDED_ACCEPT_TIMEOUT_S + 1.0), \
                patch.object(controller.vehicle_navigation, "request_guided") as set_guided:
            controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        set_guided.assert_not_called()
        self.assertIsNone(controller.navigation_task.guided_request_started_at)

    def test_nav_clears_wait_timer_once_guided_observed(self):
        """Observing GUIDED clears the wait timer so a later transient switch
        away is treated as an abort, not a fresh wait."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = False
        controller.navigation_task.guided_request_started_at = 123.0

        controller.decision.decide()

        self.assertTrue(controller.navigation_task.nav_mode_observed)
        self.assertIsNone(controller.navigation_task.guided_request_started_at)

    def test_mode_switch_clears_state(self):
        """Mode switch from GUIDED does full cleanup (pass detector, detections, etc.)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = True
        controller.navigation_task.peer_navigation = True
        controller.mission.default_delivery_hub_active = True
        controller.detections.detected_pois = [_create_detected_poi(obj_id=1)]
        controller.pass_tracker.approach_started = True
        controller.pass_tracker.previous_distance_m = 100.0

        controller.decision.decide()

        self.assertFalse(controller.navigation_task.peer_navigation)
        self.assertFalse(controller.mission.default_delivery_hub_active)
        self.assertEqual(controller.detections.detected_pois, [])
        self.assertFalse(controller.pass_tracker.approach_started)
        self.assertIsNone(controller.pass_tracker.previous_distance_m)
        detector.refresh.assert_called()


# =============================================================================
# Sense Phase Tests
# =============================================================================

class TestNavControllerSense(unittest.TestCase):
    """Tests for _sense phase."""

    def test_sense_fetches_detections(self):
        """_sense fetches detections from detector."""
        detector = _create_mock_detector()
        controller = _create_controller(detector=detector)

        controller.sensor.sense()

        detector.get_detect_data.assert_called_once()

    def test_sense_does_not_force_lock_a_poi(self):
        """Normal sensing must preserve the detector-selected primary POI."""
        detector = _create_mock_detector()
        controller = _create_controller(detector=detector)

        controller.sensor.sense()

        request = detector.get_detect_data.call_args.args[0]
        self.assertIsNone(request.force_lock_id)
        self.assertIsNone(request.force_lock_bbox_cxcywh)

    def test_sense_stores_detections(self):
        """_sense stores detections in _last_detections."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        controller.sensor.sense()

        self.assertEqual(len(controller.detections.detected_pois), 1)
        self.assertEqual(
            controller.detections.detected_pois[0].identity.obj_id,
            1,
        )

    def test_sense_stores_primary_poi(self):
        """_sense stores detector-selected primary POI."""
        poi1 = _create_detected_poi(obj_id=1)
        poi2 = _create_detected_poi(obj_id=2)
        detector = _create_mock_detector(detections=[poi1, poi2], primary_poi=poi2)
        controller = _create_controller(detector=detector)

        controller.sensor.sense()

        self.assertIs(controller.detections.primary_poi, poi2)


# =============================================================================
# Act Phase Tests - DETECT State
# =============================================================================

class TestNavControllerActDetect(unittest.TestCase):
    """Tests for _act_detect behavior."""

    def test_act_detect_sets_active_poi(self):
        """_act_detect sets active POI when self POI found."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 1)

    def test_act_detect_skips_confirmed_poi(self):
        """_act_detect skips already confirmed POIs."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        # Mark POI as already confirmed
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should not set as active POI
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_act_detect_skips_rejected_poi(self):
        """_act_detect skips already rejected POIs."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        # Mark POI as already rejected
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should not set as active POI
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_act_detect_selects_first_poi(self):
        """_act_detect selects first detection as self POI."""
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        detector = _create_mock_detector(detections=[t1, t2])
        controller = _create_controller(detector=detector)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 1)

    def test_act_detect_does_nothing_when_poi_already_active(self):
        """_act_detect does nothing if active POI already set."""
        existing_poi = _create_detected_poi(obj_id=99)
        new_poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[new_poi])
        controller = _create_controller(detector=detector)

        controller.confirmation_manager.set_active_poi(existing_poi)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should keep existing POI
        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 99)


# =============================================================================
# Act Phase Tests - CONFIRM State
# =============================================================================

class TestNavControllerActConfirm(unittest.TestCase):
    """Tests for _act_confirm behavior."""

    def test_final_approach_local_gate_still_requests_operator_confirmation(self):
        from navpy.modules.navigation.approach_strategy import ApproachKind

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 100, 80),
        )
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=True)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = create_nav_test_rig(
            _create_mock_vehicle(), detector, navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM

        def mark_confirming(pois):
            controller.confirmation_manager.update_status(
                pois[0], ConfirmationStatus.CONFIRMING,
            )

        controller.confirmation_manager.review = Mock(side_effect=mark_confirming)

        controller.confirmation_action.act()

        navigation.final_approach.can_confirm_detection.assert_called_once_with(poi)
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        controller.confirmation_manager.review.assert_called_once_with([poi])
        detector.freeze_final_approach_zoom_at_min.assert_called_once_with()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()
        self.assertEqual(
            controller.confirmation_manager.get_status(poi),
            ConfirmationStatus.CONFIRMING,
        )

    def test_final_approach_operator_review_waits_for_recognition_image_while_approaching(self):
        """Final-approach review must not send the 3 px image seen in run 213125.

        While recognition zoom is converging, the fixed-wing vehicle must keep
        flying its existing approach. Replacing a POI-centred orbit with a
        current-coordinate hold can strand it or break tracking (live runs
        223744 and 225516), so no navigation rewrite occurs here.
        """
        from navpy.modules.navigation.approach_strategy import ApproachKind
        from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M

        frame = np.zeros((1440, 2560, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(1235.0, 444.0, 5.0, 3.0),
            tracking_bbox=(1235.0, 444.0, 5.0, 3.0),
        )
        detector = _create_mock_detector(
            detections=[poi],
            is_zoom_stable=False,
            zoom_result=_zoom_result(
                state=ZoomTrackingState.ZOOMING_IN,
                size_px=6.0,
                target_pixels=48.0,
            ),
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=90.0)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=True)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        vision_profile = {
            "detector": {
                "dock_presets": {"dock": {"min_pixel_size": 48.0}},
            },
        }
        controller = create_nav_test_rig(
            vehicle,
            detector,
            navigation,
            _create_mock_args(),
            Mock(),
            approach_kind=ApproachKind.ORBIT,
            vision_profile=vision_profile,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM

        controller.confirmation_action.act()

        navigation.final_approach.can_confirm_detection.assert_called_once_with(poi)
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        controller.confirmation_manager.review.assert_not_called()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()

    def test_act_confirm_waits_for_good_picture(self):
        """_act_confirm waits until detection_frame is available."""
        poi = _create_detected_poi(obj_id=1, detection_frame=None)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Should not call peer_poi (not ready to confirm)
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_act_confirm_proceeds_without_frame_when_source_renders_none(self):
        """A frame-less-by-design source (sim ideal_360) must not wait.

        The ideal sensor renders no frames at all, so ``detection_frame is
        None`` is permanent. Confirmation is still requested; ConfirmationManager
        then applies the same downstream policy as for a framed source.
        """
        poi = _create_detected_poi(
            obj_id=1, detection_frame=None, supports_confirmation_frame=False,
        )
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation,
            args=_create_mock_args(auto_confirm=True),
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        controller.confirmation_manager.review.assert_called_once_with([poi])

    def test_act_confirm_frameless_source_still_reviews_in_manual(self):
        """Manual confirm stays reachable for a frame-less source.

        The operator popup arrives without a thumbnail (ConfirmationManager skips
        the image), but the confirmation request itself must still be sent so
        ConfirmationManager's own auto/manual policy decides, not the frame gate.
        """
        poi = _create_detected_poi(
            obj_id=1, detection_frame=None, supports_confirmation_frame=False,
        )
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation,
            args=_create_mock_args(auto_confirm=False),
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        controller.confirmation_manager.review.assert_called_once_with([poi])

    def test_act_confirm_frame_capable_source_still_blocks_without_frame(self):
        """Fail-closed: a camera whose frame has not arrived yet still waits.

        Same auto-confirm setting as the waived case above — only the source
        capability differs, so the waiver cannot leak into real detectors.
        """
        poi = _create_detected_poi(obj_id=1, detection_frame=None)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation,
            args=_create_mock_args(auto_confirm=True),
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        controller.confirmation_manager.review.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()

    def test_final_approach_manual_frameless_source_reviews_without_final_approach_record(self):
        """Final-approach MANUAL keeps its deferred record when the frame is waived.

        The frame waiver only removes the image wait. The final-approach observation
        and wind freeze stay deferred to the dive-commit at NAV entry, so
        record_final_approach_confirmed_detection must not fire here.
        """
        poi = _create_detected_poi(
            obj_id=1, detection_frame=None, supports_confirmation_frame=False,
        )
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=True)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = _create_controller(
            detector=detector, navigation=navigation,
            args=_create_mock_args(auto_confirm=False),
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        controller.confirmation_manager.review.assert_called_once_with([poi])
        navigation.final_approach.record_confirmed_detection.assert_not_called()

    def test_act_confirm_requests_confirmation_with_frame(self):
        """_act_confirm requests confirmation when frame available."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80))
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # OFFSET self-detect CONFIRM stops with explicit 80m loiter radius
        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_act_confirm_skips_if_status_already_set(self):
        """_act_confirm does nothing if confirmation already requested."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80))
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Should not call peer_poi again
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_act_confirm_waits_for_zoom_stable(self):
        """_act_confirm defers confirmation until zoom tracker reports stable.

        Without this gate, the operator gets a blurry mid-zoom thumbnail
        because the confirmation frame is captured before the camera has
        settled on the recognition-sized bbox.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80))
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=False)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Zoom not stable → must not stop and must not request review
        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

    def test_act_confirm_proceeds_when_zoom_stable(self):
        """Once zoom is stable, _act_confirm stops and requests review."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80))
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()

    def test_act_confirm_waits_on_in_band_unstable_zoom_result(self):
        """A real in-band HOLD result (bbox inside the hysteresis band,
        below target_pixels) reports unstable: confirmation keeps waiting
        even though the zoom controller will not issue further commands.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        in_band = ZoomTrackResult(
            state=ZoomTrackingState.HOLDING, has_poi=True,
            size_px=140.0, target_pixels=150.0, reason="in-band",
        )
        self.assertFalse(in_band.is_stable)
        detector = _create_mock_detector(
            detections=[poi], is_zoom_stable=False, zoom_result=in_band,
        )
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

    def test_act_confirm_proceeds_on_settling_hold_above_target(self):
        """A settling HOLD at/above target_pixels stays stable: readback
        gaps must not stall a confirmation whose zoom goal is already met.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        settling = ZoomTrackResult(
            state=ZoomTrackingState.HOLDING, has_poi=True,
            size_px=160.0, target_pixels=150.0, reason="settling",
        )
        self.assertTrue(settling.is_stable)
        detector = _create_mock_detector(
            detections=[poi], is_zoom_stable=False, zoom_result=settling,
        )
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()

    def test_act_confirm_blocks_below_min_confirm_pixels(self):
        """bbox height below MIN_CONFIRM_PIXELS → confirmation deferred."""
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        small_h = MIN_CONFIRM_PIXELS - 1
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 12, 12),
            tracking_bbox=(320.0, 240.0, 12.0, float(small_h)),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Pixel gate must block before we stop or request review.
        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

    def test_act_confirm_passes_at_min_confirm_pixels(self):
        """bbox height at/above MIN_CONFIRM_PIXELS → proceeds to review."""
        from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 100, MIN_CONFIRM_PIXELS),
            tracking_bbox=(320.0, 240.0, 100.0, float(MIN_CONFIRM_PIXELS)),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()

    def test_act_confirm_uses_profile_class_pixel_threshold(self):
        """Class-specific profile threshold blocks until recognition pixels."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        vision_profile = {
            "detector": {
                "dock_presets": {
                    "dock": {"min_pixel_size": 150},
                },
            },
        }

        # Gate now measures the bbox DIAGONAL sqrt(w^2+h^2) vs min_pixels=150.
        # 90x100 -> 134 px (< 150) blocks; the wider 120x100 -> 156 px passes,
        # exercising that width now counts toward recognition size.
        small = _create_detected_poi(
            obj_id=1,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 90, 100),
            tracking_bbox=(320.0, 240.0, 90.0, 100.0),
        )
        small_detector = _create_mock_detector(detections=[small], is_zoom_stable=True)
        small_navigation = _create_mock_navigation()
        small_controller = _create_controller(
            detector=small_detector,
            navigation=small_navigation,
            vision_profile=vision_profile,
        )
        small_controller.confirmation_manager.set_active_poi(small)
        small_controller.sensor.sense()
        small_controller.phase.current = NavState.CONFIRM
        small_controller.confirmation_action.act()

        small_navigation.vehicle_commands.peer_poi_loiter.assert_not_called()
        self.assertIsNone(small_controller.confirmation_manager.get_status(small))

        ready = _create_detected_poi(
            obj_id=2,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 120, 100),
            tracking_bbox=(320.0, 240.0, 120.0, 100.0),
        )
        ready_detector = _create_mock_detector(detections=[ready], is_zoom_stable=True)
        ready_navigation = _create_mock_navigation()
        ready_controller = _create_controller(
            detector=ready_detector,
            navigation=ready_navigation,
            vision_profile=vision_profile,
        )
        ready_controller.confirmation_manager.set_active_poi(ready)
        ready_controller.sensor.sense()
        ready_controller.phase.current = NavState.CONFIRM
        ready_controller.confirmation_action.act()

        ready_navigation.vehicle_commands.peer_poi_loiter.assert_called_once()

    def test_act_confirm_blocks_when_live_bbox_ready_but_source_bbox_small(self):
        """Gate must judge the stored source bbox, not transient live tracking bbox."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        vision_profile = {
            "detector": {
                "dock_presets": {"dock": {"min_pixel_size": 150}},
            },
        }
        # Source bbox diagonal 90x100 -> 134 px (< 150) blocks, even though
        # the live tracking bbox 100x152 -> 182 px is already recognition-sized.
        poi = _create_detected_poi(
            obj_id=1,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 90, 100),
            tracking_bbox=(320.0, 240.0, 100.0, 152.0),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation, vision_profile=vision_profile,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

    def test_act_confirm_passes_when_source_bbox_ready_even_if_live_bbox_small(self):
        """A ready stored source frame should pass even if live tracking shrinks later."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        vision_profile = {
            "detector": {
                "dock_presets": {"dock": {"min_pixel_size": 150}},
            },
        }
        poi = _create_detected_poi(
            obj_id=1,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 100, 150),
            tracking_bbox=(320.0, 240.0, 100.0, 20.0),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation, vision_profile=vision_profile,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertFalse(poi.confirmation.degraded)

    def test_act_confirm_best_available_at_max_zoom_does_not_mark_degraded(self):
        """Below-threshold source can proceed only when active zoom is at max."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        vision_profile = {
            "detector": {
                "dock_presets": {"dock": {"min_pixel_size": 150}},
            },
        }
        poi = _create_detected_poi(
            obj_id=1,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 100, 98),
            tracking_bbox=(320.0, 240.0, 100.0, 98.0),
        )
        detector = _create_mock_detector(
            detections=[poi],
            is_zoom_stable=False,
            zoom_result=_zoom_result(
                size_px=98.0,
                target_pixels=150.0,
                at_max_zoom=True,
            ),
        )
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation, vision_profile=vision_profile,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertFalse(poi.confirmation.degraded)

    def test_act_confirm_passes_when_bbox_unknown(self):
        """Missing bbox must not block confirmation (safe no-op).

        Some real detectors may deliver a detection without a tracking
        bbox — in that case the pixel gate has nothing to measure and
        must defer to the other gates rather than blocking indefinitely.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
            tracking_bbox=None,
        )
        poi.capture_confirmation(replace(
            poi.confirmation,
            bbox_cxcywh=None,
        ))
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Pixel gate returns None → does not block; proceeds to review.
        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()

    def test_confirm_gate_timeout_blocks_below_source_pixels_without_max_zoom(self):
        """Timeout must not send a below-threshold source image while zoom can improve."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 10, 5),
            tracking_bbox=(320.0, 240.0, 10.0, 5.0),  # diagonal ~11 px, below MIN_CONFIRM_PIXELS
        )
        poi.set_confirmation_degraded(False)
        # Small timeout so the test runs fast.
        args = _create_mock_args(confirm_gate_timeout=0.05)
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(args=args, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        # Arm the timer as the state machine would on CONFIRM entry,
        # dated far enough in the past to have timed out already.
        controller.confirm.entered_at = time.monotonic() - 1.0

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        self.assertFalse(poi.confirmation.degraded)
        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

    def test_confirm_gate_timeout_sends_recognition_sized_source_without_degraded_flag(self):
        """Timeout can bypass zoom only after the stored source image is large enough."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        vision_profile = {
            "detector": {
                "dock_presets": {"dock": {"min_pixel_size": 150}},
            },
        }
        poi = _create_detected_poi(
            obj_id=1,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 100, 150),
            tracking_bbox=(320.0, 240.0, 100.0, 150.0),
        )
        detector = _create_mock_detector(
            detections=[poi],
            is_zoom_stable=False,
            zoom_result=_zoom_result(state=ZoomTrackingState.ZOOMING_IN),
        )
        navigation = _create_mock_navigation()
        controller = _create_controller(
            args=_create_mock_args(confirm_gate_timeout=0.0),
            detector=detector,
            navigation=navigation,
            vision_profile=vision_profile,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        controller.phase.current = NavState.CONFIRM
        controller.sensor.sense()
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertFalse(poi.confirmation.degraded)

    def test_offset_self_detect_loiter_radius_pinned_to_80m(self):
        """Regression: a stale autopilot WP_LOITER_RAD (e.g. 683m left
        over from a prior gimbal/ORBIT session) must NOT leak into the
        OFFSET self-detect CONFIRM-loiter — the radius is supplied
        explicitly via peer_poi_loiter and the original is captured
        for restore.
        """
        from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=683.0)  # stale gimbal-era radius
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Save fired and captured the stale 683m as the original-to-restore.
        vehicle.get_parameter.assert_any_call("WP_LOITER_RAD")
        self.assertEqual(controller.loiter_radius.original, 683.0)
        # Loiter command carried the explicit 80m radius — bare peer_poi
        # (which would have inherited the stale autopilot value) was NOT used.
        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        radius = navigation.vehicle_commands.peer_poi_loiter.call_args.args[1]
        self.assertEqual(radius, OFFSET_LOITER_RADIUS_M)
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_stabilize_mode_during_pinned_loiter_restores_radius(self):
        """Inactive-mode guard must restore on STABILIZE just like on
        disarm — both are early-return paths in `_decide` that bypass
        the main abort branch.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=683.0)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()
        self.assertEqual(controller.loiter_radius.original, 683.0)

        # Pilot drops out to STABILIZE.
        vehicle.get_mode = FlightMode.STABILIZE
        controller.decision.decide()

        vehicle.set_parameter.assert_any_call("WP_LOITER_RAD", 683.0)
        self.assertIsNone(controller.loiter_radius.original)
        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_disarm_during_pinned_loiter_restores_radius(self):
        """If the pilot disarms mid-navigation task, the disarmed early-return
        in _decide must still run cleanup so the 80m pin doesn't strand
        on the autopilot. Without the cleanup branch in the guard, the
        next mission would inherit our 80m loiter radius.
        """
        from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=683.0)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()
        self.assertEqual(controller.loiter_radius.original, 683.0)

        # Pilot disarms.
        vehicle.is_armed = False
        controller.decision.decide()

        # Restore fired and the saved value cleared.
        vehicle.set_parameter.assert_any_call("WP_LOITER_RAD", 683.0)
        self.assertIsNone(controller.loiter_radius.original)
        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_offset_confirm_skips_pin_when_save_fails(self):
        """If `get_parameter("WP_LOITER_RAD")` returns None (MAVLink
        round-trip exhausted), pinning would leave the autopilot at 80m
        with no recorded original to restore — exactly the failure
        class we're guarding against. The fix must skip the pin and
        fall back to bare peer_poi instead.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        vehicle = _create_mock_vehicle()
        # Param fetch fails — _save_loiter_rad must not record None.
        vehicle.get_parameter = Mock(return_value=None)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # Pin must NOT have happened — fall back to bare peer_poi.
        self.assertIsNone(controller.loiter_radius.original)
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()
        navigation.vehicle_commands.peer_poi.assert_called_once()

    def test_rejected_poi_resume_auto_mission_restores_loiter_rad(self):
        """REJECTED path also restores. _resume_auto_mission has its own
        copy of the cleanup logic (independent of _clear_state) and must
        not strand the 80m pin when the operator rejects the POI.
        """
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        vehicle.get_parameter = Mock(return_value=683.0)
        controller = _create_controller(vehicle=vehicle)

        # Simulate post-CONFIRM state: pinned.
        controller.loiter_radius.original = 683.0
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        controller.decision.decide()

        vehicle.set_parameter.assert_any_call("WP_LOITER_RAD", 683.0)
        self.assertIsNone(controller.loiter_radius.original)

    def test_nav_natural_completion_restores_loiter_rad(self):
        """NAV → RESET (natural completion via passed-POI) must
        flow through _enter_reset → _clear_state → _restore_loiter_rad.
        Locks down the full lifecycle: pin → nav → reset → restore.
        """
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=683.0)
        controller = _create_controller(vehicle=vehicle)

        # Mid-navigation task: pinned, in NAV.
        controller.loiter_radius.original = 683.0
        controller.phase.current = NavState.NAV

        # State machine: NAV → RESET via passed-POI detector.
        controller.transitions.on_change(NavState.NAV, NavState.RESET)

        vehicle.set_parameter.assert_any_call("WP_LOITER_RAD", 683.0)
        self.assertIsNone(controller.loiter_radius.original)

    def test_follow_up_navigation_task_saves_independently(self):
        """After a clean navigation task-end and restore, a second navigation task
        must save again (capturing the restored 683m as the new original)
        and pin 80m — i.e. _save_loiter_rad's idempotency guard does not
        latch across separate navigation runs.
        """
        from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=683.0)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
        )

        # First navigation_task.
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()
        self.assertEqual(controller.loiter_radius.original, 683.0)
        controller.reset.clear()
        self.assertIsNone(controller.loiter_radius.original)

        # Second navigation task must save afresh, not skip because of stale flag.
        poi2 = _create_detected_poi(
            obj_id=2, detection_frame=frame, bbox=(320, 240, 100, 80),
        )
        detector.get_detect_data.return_value = DetectResponse(
            [poi2], primary_poi=None,
        )
        controller.confirmation_manager.set_active_poi(poi2)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()
        self.assertEqual(controller.loiter_radius.original, 683.0)
        self.assertEqual(navigation.vehicle_commands.peer_poi_loiter.call_count, 2)
        self.assertEqual(
            navigation.vehicle_commands.peer_poi_loiter.call_args.args[1], OFFSET_LOITER_RADIUS_M,
        )

    def test_confirm_gate_timer_armed_on_state_entry(self):
        """Transitioning into CONFIRM arms the timeout timer."""
        controller = _create_controller()
        controller.confirm.entered_at = None
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirm.entered_at)

    def test_confirm_gate_timer_cleared_on_state_exit(self):
        """Leaving CONFIRM clears the timeout timer."""
        controller = _create_controller()
        controller.confirm.entered_at = time.monotonic()
        controller.transitions.on_change(NavState.CONFIRM, NavState.DETECT)
        self.assertIsNone(controller.confirm.entered_at)

    def test_confirm_gate_not_timed_out_before_threshold(self):
        """Before timeout, gate still blocks — no degraded flag set."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 100, 80),
            tracking_bbox=(320.0, 240.0, 100.0, 5.0),  # below MIN_CONFIRM_PIXELS
        )
        poi.set_confirmation_degraded(False)
        args = _create_mock_args(confirm_gate_timeout=10.0)  # long timeout
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(args=args, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirm.entered_at = time.monotonic()  # just entered
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        self.assertFalse(poi.confirmation.degraded)
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_extract_confirmation_size_uses_source_bbox_only(self):
        from navpy.modules.nav.nav_controller import _extract_confirmation_size

        poi = _create_detected_poi(
            tracking_bbox=(100.0, 100.0, 50.0, 200.0),
            bbox=(100.0, 100.0, 50.0, 80.0),
        )
        # Diagonal of the SOURCE bbox (w=50, h=80), not the tracking bbox.
        self.assertAlmostEqual(
            _extract_confirmation_size(poi), math.sqrt(50.0**2 + 80.0**2))

    def test_extract_confirmation_size_missing_returns_none(self):
        from navpy.modules.nav.nav_controller import _extract_confirmation_size

        poi = _create_detected_poi(
            tracking_bbox=(100.0, 100.0, 50.0, 200.0),
            bbox=None,
        )
        self.assertIsNone(_extract_confirmation_size(poi))

    def test_confirm_gate_reset_cap_blocks_starvation(self):
        """After MAX_CONFIRM_GATE_RESETS re-acquires, the timer stops being
        extended — FOV-edge oscillation can't starve the timeout."""
        from navpy.modules.nav.nav_controller import MAX_CONFIRM_GATE_RESETS

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 10, 5),
            tracking_bbox=(320.0, 240.0, 10.0, 5.0),  # diagonal ~11 px, stays below threshold
        )
        args = _create_mock_args(confirm_gate_timeout=10.0)
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=False)
        detector.get_zoom_result.return_value = _zoom_result(has_poi=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(args=args, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        controller.phase.current = NavState.CONFIRM
        controller.sensor.sense()

        # Initial acquisition — no reset, counter stays 0.
        controller.confirmation_action.act()
        self.assertEqual(controller.confirm.resets_used, 0)

        # Oscillate has_poi True/False/True/False/... — each False→True
        # after initial should reset the timer until the cap is reached.
        for i in range(MAX_CONFIRM_GATE_RESETS + 3):
            detector.get_zoom_result.return_value = _zoom_result(has_poi=False)
            controller.confirmation_action.act()
            detector.get_zoom_result.return_value = _zoom_result(has_poi=True)
            controller.confirmation_action.act()

        # Counter must stop at the cap, not grow unbounded.
        self.assertEqual(controller.confirm.resets_used, MAX_CONFIRM_GATE_RESETS)

    def test_confirm_gate_timer_resets_on_zoom_reacquire(self):
        """POI drops out then reappears → gate timer resets so a fresh
        zoom convergence gets a fresh budget."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 10, 5),
            tracking_bbox=(320.0, 240.0, 10.0, 5.0),  # diagonal ~11 px, below MIN_CONFIRM_PIXELS
        )
        args = _create_mock_args(confirm_gate_timeout=10.0)
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=False)
        # Rig a zoom_tracker mock that flips has_poi False→True→False→True.
        states = [
            _zoom_result(has_poi=True),   # initial acquisition
            _zoom_result(has_poi=False),  # dropout
            _zoom_result(has_poi=True),   # re-acquire
        ]
        detector.get_zoom_result.return_value = states[0]
        navigation = _create_mock_navigation()
        controller = _create_controller(args=args, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        original_ts = controller.confirm.entered_at

        # Tick 1: initial acquisition, must NOT reset the timer.
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()
        self.assertEqual(controller.confirm.entered_at, original_ts)

        # Tick 2: dropout — flag flips; timer untouched.
        detector.get_zoom_result.return_value = states[1]
        controller.confirmation_action.act()
        self.assertEqual(controller.confirm.entered_at, original_ts)

        # Tick 3: re-acquire — timer must be reset to a later time.
        # 50 ms covers Windows default clock resolution (~15.6 ms).
        time.sleep(0.05)
        detector.get_zoom_result.return_value = states[2]
        controller.confirmation_action.act()
        self.assertGreater(controller.confirm.entered_at, original_ts)

    def test_act_confirm_uses_dock_threshold_from_profile(self):
        """Pixel gate reads the threshold from the profile's dock preset,
        so the dock class must wait for a bbox of that size."""
        profile = {
            "detector": {
                "dock_presets": {
                    "dock": {"min_pixel_size": 30},
                },
            },
        }
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Dock (class 0) POI with 20 px bbox — above the global MIN_CONFIRM_PIXELS,
        # but below the dock-preset threshold of 30.
        poi = _create_detected_poi(
            obj_id=1,
            class_id=0,  # Detection class 0
            detection_frame=frame,
            bbox=(320, 240, 100, 80),
            tracking_bbox=(320.0, 240.0, 100.0, 20.0),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(
            detector=detector, navigation=navigation, vision_profile=profile,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        # 20 < 30 → gate must block; no peer_poi call.
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_decide_to_confirm_arms_gate_timer_via_state_transition(self):
        """Integration: a real DETECT→CONFIRM transition driven through the
        _decide/_act cycle must arm _confirm_gate_entered_at. Without this
        coverage, a regression in _on_state_change dispatch would leave
        the timer None and the degraded-fallback permanently dead."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1,
            detection_frame=frame,
            bbox=(320, 240, 100, 80),
            tracking_bbox=(320.0, 240.0, 100.0, 50.0),
        )
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        # Put the active POI into CONFIRMING so the decide step lands on CONFIRM.
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMING)
        controller.sensor.sense()
        self.assertIsNone(controller.confirm.entered_at)

        _decide_and_act(controller)

        self.assertEqual(controller.phase.current, NavState.CONFIRM)
        self.assertIsNotNone(controller.confirm.entered_at,
                             "_on_state_change must arm the timer on DETECT→CONFIRM")

    def test_confirm_gate_pipeline_end_to_end(self):
        """End-to-end: POI enters CONFIRM with small bbox → pixel gate
        defers → bbox grows past threshold + zoom stabilises → gate opens
        → peer_poi fires and review is requested. This is the single
        scenario that proves all three gates are wired together correctly."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=99,
            class_id=0,
            detection_frame=frame,
            bbox=(320, 240, 100, 5),
            # Start undersized — below MIN_CONFIRM_PIXELS.
            tracking_bbox=(320.0, 240.0, 100.0, 5.0),
        )
        poi.set_confirmation_degraded(False)
        detector = _create_mock_detector(detections=[poi], is_zoom_stable=False)
        navigation = _create_mock_navigation()
        # Ample timeout so the gate timeout doesn't fire before we
        # simulate a normal convergence.
        args = _create_mock_args(confirm_gate_timeout=30.0)
        controller = _create_controller(args=args, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        # Arm CONFIRM entry via the state change handler so the timer is
        # set as it would be in the real _nav_loop.
        controller.transitions.on_change(NavState.DETECT, NavState.CONFIRM)
        controller.phase.current = NavState.CONFIRM

        # Tick 1: pixel gate blocks.
        controller.sensor.sense()
        controller.confirmation_action.act()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))
        self.assertFalse(poi.confirmation.degraded)

        # Tick 2: bbox grows but zoom still converging → zoom gate blocks.
        poi.capture_confirmation(replace(
            poi.confirmation,
            bbox_cxcywh=(320.0, 240.0, 100.0, 40.0),
        ))
        poi.replace_tracking(replace(
            poi.tracking,
            bbox_cxcywh=(320.0, 240.0, 100.0, 40.0),
        ))  # above threshold
        detector.is_zoom_stable = False
        controller.confirmation_action.act()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.get_status(poi))

        # Tick 3: zoom stabilises with a good-sized bbox → both gates pass,
        # confirmation proceeds.
        detector.is_zoom_stable = True
        controller.confirmation_action.act()
        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        # confirmation_degraded must NOT be set on the normal success path.
        self.assertFalse(poi.confirmation.degraded)


# =============================================================================
# Act Phase Tests - NAV State
# =============================================================================

class TestNavControllerActNav(unittest.TestCase):
    """Tests for _act_nav behavior."""

    def test_act_nav_calls_navigation(self):
        """_act_nav calls navigation.nav with fresh detection."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once()

    def test_act_nav_coalesces_source_events_to_newest_per_tick(self):
        first = _create_detected_poi(obj_id=1, task_id=11)
        second = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(detections=[second])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([first], primary_poi=first),
            _source_event([second], primary_poi=second),
        ]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(second)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(second)

    def test_act_nav_retains_source_events_while_guided_mode_is_pending(self):
        first = _create_detected_poi(obj_id=1, task_id=11)
        second = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(detections=[second])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.side_effect = [
            [_source_event([first], primary_poi=first)],
            [_source_event([second], primary_poi=second)],
        ]
        navigation = _create_mock_navigation()
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(second)
        controller.final_approach.confirmed_recorded = True
        controller.phase.current = NavState.NAV

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()
        self.assertEqual(len(controller.detections.pending_events), 1)

        vehicle.get_mode = FlightMode.GUIDED
        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(second)
        self.assertEqual(controller.detections.pending_events, [])

    def test_manual_source_commit_clears_discontinuity_then_uses_newest_event(self):
        first = _create_detected_poi(obj_id=1, task_id=11)
        second = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(detections=[second])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [first],
                primary_poi=first,
                source_timestamp_s=20.00,
                source_name="gimbal_0",
                source_discontinuity=True,
            ),
            _source_event(
                [second],
                primary_poi=second,
                source_timestamp_s=20.02,
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        call_order = []
        navigation.final_approach.clear_source_discontinuity = Mock(
            side_effect=lambda sources: call_order.append(
                f"clear:{','.join(sources)}"
            )
        )
        navigation.final_approach.record_confirmed_detection = Mock(
            side_effect=lambda poi: call_order.append(
                "record_first" if poi is first else "record_second"
            ) or True
        )
        navigation.nav = Mock(
            side_effect=lambda poi: call_order.append(
                "nav_first" if poi is first else "nav_second"
            ) or True
        )
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(second)
        controller.final_approach.confirmed_recorded = False

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertEqual(
            call_order,
            ["clear:gimbal_0", "record_second", "nav_second"],
        )
        self.assertTrue(controller.final_approach.confirmed_recorded)
        self.assertEqual(controller.detections.pending_events, [])
        self.assertFalse(controller.navigation_failures.failed)
        controller.final_approach_nav.close_source_admission()

    def test_manual_source_commit_ignores_superseded_seed_event(self):
        old = _create_detected_poi(obj_id=1, task_id=11)
        fresh = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(
            detections=[fresh], primary_poi=fresh,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([old], primary_poi=old),
            _source_event([fresh], primary_poi=fresh),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.record_confirmed_detection = Mock(
            side_effect=lambda candidate: candidate is fresh
        )
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(fresh)

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertEqual(
            navigation.final_approach.record_confirmed_detection.call_args_list,
            [call(fresh)],
        )
        navigation.nav.assert_called_once_with(fresh)
        self.assertTrue(controller.final_approach.confirmed_recorded)
        self.assertEqual(controller.detections.pending_events, [])
        self.assertFalse(controller.navigation_failures.failed)
        controller.final_approach_nav.close_source_admission()

    def test_act_nav_coalesces_coordinator_child_events_to_newest(self):
        first = _create_detected_poi(obj_id=7)
        second = _create_detected_poi(obj_id=7)
        for timestamp, poi in ((20.00, first), (20.02, second)):
            _set_poi_timestamp(poi, timestamp)
            _set_poi_source(poi, "gimbal_0")
        child = _create_mock_detector(
            detections=[second], primary_poi=second,
        )
        child.has_source_driven_detection_events = True
        child.drain_detection_events.return_value = [
            _source_event([first], primary_poi=first),
            _source_event([second], primary_poi=second),
        ]
        coordinator = DetectionCoordinator([child], Mock())
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=coordinator,
            navigation=navigation,
        )

        controller.sensor.sense()
        controller.confirmation_manager.set_active_poi(
            controller.detections.primary_poi,
        )
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(second)
        self.assertEqual(first.identity.task_id, second.identity.task_id)

    def test_act_nav_coalesces_detection_before_trailing_source_gap(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(detections=[])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [poi],
                primary_poi=poi,
                source_timestamp_s=20.00,
                source_name="gimbal_0",
            ),
            _source_event(
                [],
                source_timestamp_s=20.02,
                source_name="gimbal_0",
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.nav_without_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()
        self.assertTrue(controller.navigation_failures.failed)

    def test_act_nav_coalesces_leading_gap_before_detection(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [],
                source_timestamp_s=20.00,
                source_name="gimbal_0",
            ),
            _source_event(
                [poi],
                primary_poi=poi,
                source_timestamp_s=20.02,
                source_name="gimbal_0",
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.nav_without_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(poi)
        navigation.final_approach.nav_without_detection.assert_not_called()

    def test_act_nav_preserves_skipped_discontinuity_for_newest_event(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        skipped = _source_event(
            [],
            source_timestamp_s=20.00,
            source_name="gimbal_0",
            source_discontinuity=True,
        )
        newest = _source_event(
            [poi],
            primary_poi=poi,
            source_timestamp_s=20.02,
            source_name="gimbal_0",
        )
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [skipped, newest]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.final_approach.clear_source_discontinuity.assert_called_once_with(
            ("gimbal_0",)
        )
        navigation.nav.assert_called_once_with(poi)
        navigation.final_approach.nav_without_detection.assert_not_called()
        self.assertIs(navigation.nav.call_args.args[0], poi)
        self.assertEqual(newest.source_timestamp_s, 20.02)

    def test_manual_source_commit_does_not_fall_back_from_newest_gap(self):
        old = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(old, "gimbal_0")
        detector = _create_mock_detector(detections=[])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [old],
                primary_poi=old,
                source_timestamp_s=20.00,
                source_name="gimbal_0",
            ),
            _source_event(
                [],
                source_timestamp_s=20.02,
                source_name="gimbal_0",
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(old)

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.final_approach.record_confirmed_detection.assert_not_called()
        navigation.nav.assert_not_called()
        navigation.final_approach.nav_without_detection.assert_not_called()
        self.assertFalse(controller.final_approach.confirmed_recorded)
        self.assertEqual(controller.detections.pending_events, [])

    def test_act_nav_resets_visual_history_after_event_overload(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [poi],
                primary_poi=poi,
                source_timestamp_s=20.02,
                source_name="gimbal_0",
                source_discontinuity=True,
            ),
        ]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.final_approach.clear_source_discontinuity.assert_called_once_with(
            ("gimbal_0",)
        )
        navigation.nav.assert_called_once_with(poi)

    def test_act_nav_clears_all_unique_discontinuity_sources_before_latest(self):
        a_first = _create_detected_poi(obj_id=1, task_id=11)
        a_duplicate = _create_detected_poi(obj_id=1, task_id=11)
        b_first = _create_detected_poi(obj_id=1, task_id=11)
        newest = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(newest, "gimbal_b")
        events = [
            _source_event(
                [a_first],
                primary_poi=a_first,
                source_timestamp_s=100.0,
                source_name="gimbal_a",
                source_discontinuity=True,
            ),
            _source_event(
                [a_duplicate],
                primary_poi=a_duplicate,
                source_timestamp_s=0.5,
                source_name="gimbal_a",
                source_discontinuity=True,
            ),
            _source_event(
                [b_first],
                primary_poi=b_first,
                source_timestamp_s=100.0,
                source_name="gimbal_b",
                source_discontinuity=True,
            ),
            _source_event(
                [newest],
                primary_poi=newest,
                source_timestamp_s=1.0,
                source_name="gimbal_b",
            ),
        ]
        detector = _create_mock_detector(detections=[newest], primary_poi=newest)
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = events
        navigation = _create_mock_navigation()
        order = []
        navigation.nav.side_effect = lambda poi: order.append(
            ("nav", poi)
        ) or True
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(newest)
        controller.final_approach.confirmed_recorded = True
        navigation.final_approach.clear_source_discontinuity.side_effect = (
            lambda names: order.append((
                "clear",
                names,
                tuple(controller.detections.pending_events),
            ))
        )

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertEqual(
            order,
            [
                ("clear", ("gimbal_a", "gimbal_b"), tuple(events)),
                ("nav", newest),
            ],
        )
        self.assertEqual(controller.detections.pending_events, [])

    def test_discontinuity_clear_failure_retains_markers_and_skips_dispatch(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        event = _source_event(
            [poi],
            primary_poi=poi,
            source_timestamp_s=1.0,
            source_name="gimbal_0",
            source_discontinuity=True,
        )
        detector = _create_mock_detector(detections=[poi], primary_poi=poi)
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [event]
        navigation = _create_mock_navigation()
        navigation.final_approach.clear_source_discontinuity.side_effect = RuntimeError(
            "epoch clear failed"
        )
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        with self.assertRaisesRegex(RuntimeError, "epoch clear failed"):
            controller.final_approach_nav.act_nav()

        self.assertEqual(controller.detections.pending_events, [event])
        navigation.nav.assert_not_called()

    def test_source_event_ack_preserves_event_appended_during_discontinuity_clear(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        next_poi = _create_detected_poi(obj_id=1, task_id=11)
        event = _source_event(
            [poi],
            primary_poi=poi,
            source_timestamp_s=1.0,
            source_name="gimbal_0",
            source_discontinuity=True,
        )
        next_event = _source_event(
            [next_poi],
            primary_poi=next_poi,
            source_timestamp_s=1.1,
            source_name="gimbal_0",
        )
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [event]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True
        navigation.final_approach.clear_source_discontinuity.side_effect = (
            lambda _names: controller.detections.replace(
                [next_poi],
                next_poi,
                [next_event],
                append_events=True,
            )
        )

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(poi)
        self.assertEqual(controller.detections.pending_events, [next_event])

    def test_replaced_event_batch_is_not_dispatched_after_stale_ack(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        replacement = _create_detected_poi(obj_id=1, task_id=11)
        event = _source_event(
            [poi],
            primary_poi=poi,
            source_timestamp_s=1.0,
            source_name="gimbal_0",
            source_discontinuity=True,
        )
        replacement_event = _source_event(
            [replacement],
            primary_poi=replacement,
            source_timestamp_s=1.1,
            source_name="gimbal_0",
        )
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [event]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True
        navigation.final_approach.clear_source_discontinuity.side_effect = (
            lambda _names: controller.detections.replace(
                [replacement],
                replacement,
                [replacement_event],
                append_events=False,
            )
        )

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()
        self.assertEqual(
            controller.detections.pending_events,
            [replacement_event],
        )

    def test_unnamed_discontinuity_is_explicitly_dropped_and_fails_nav(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        event = _source_event(
            [poi],
            primary_poi=poi,
            source_timestamp_s=1.0,
            source_name=None,
            source_discontinuity=True,
        )
        detector = _create_mock_detector(detections=[poi], primary_poi=poi)
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [event]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_failures.failed)
        self.assertEqual(controller.detections.pending_events, [])
        navigation.final_approach.clear_source_discontinuity.assert_not_called()
        navigation.nav.assert_not_called()

    def test_act_nav_mixed_coordinator_keeps_polling_active_poi(self):
        source_poi = _create_detected_poi(obj_id=1)
        polling_poi = _create_detected_poi(obj_id=1)
        for name, poi in (
                ("gimbal_source", source_poi),
                ("gimbal_polling", polling_poi),
        ):
            _set_poi_source(poi, name)
            _set_poi_timestamp(poi, 20.0)
        source = _create_mock_detector()
        source.has_source_driven_detection_events = True
        source.drain_detection_events.return_value = [
            _source_event(
                [source_poi],
                primary_poi=source_poi,
                source_timestamp_s=20.0,
                source_name="gimbal_source",
            ),
        ]
        polling = _create_mock_detector(
            detections=[polling_poi], primary_poi=polling_poi,
        )
        polling.has_source_driven_detection_events = False
        coordinator = DetectionCoordinator([source, polling], Mock())
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=coordinator,
            navigation=navigation,
        )

        controller.sensor.sense()
        active = next(
            poi for poi in controller.detections.detected_pois
            if poi.pixel.source_name == "gimbal_polling"
        )
        controller.confirmation_manager.set_active_poi(active)
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_called_once_with(polling_poi)
        source.get_detect_data.assert_called_once()
        polling.get_detect_data.assert_called_once()

    def test_sense_samples_mixed_coordinator_polling_child_once_per_tick(self):
        polling_poi = _create_detected_poi(obj_id=1, task_id=11)
        source = _create_mock_detector()
        source.has_source_driven_detection_events = True
        source.drain_detection_events.return_value = []
        polling = _create_mock_detector()
        polling.has_source_driven_detection_events = False
        polling.get_detect_data.side_effect = [
            DetectResponse(
                [polling_poi], primary_poi=polling_poi,
            ),
            DetectResponse([]),
        ]
        coordinator = DetectionCoordinator([source, polling], Mock())
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=coordinator,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(polling_poi)
        controller.phase.current = NavState.NAV

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        polling.get_detect_data.assert_called_once()
        self.assertEqual(controller.detections.detected_pois, [polling_poi])
        navigation.nav.assert_called_once_with(polling_poi)

    def test_mixed_coordinator_polling_loss_uses_one_current_sample(self):
        polling_poi = _create_detected_poi(obj_id=1, task_id=11)
        source = _create_mock_detector()
        source.has_source_driven_detection_events = True
        source.drain_detection_events.return_value = []
        polling = _create_mock_detector()
        polling.has_source_driven_detection_events = False
        polling.get_detect_data.side_effect = [
            DetectResponse(
                [polling_poi], primary_poi=polling_poi,
            ),
            DetectResponse([]),
        ]
        coordinator = DetectionCoordinator([source, polling], Mock())
        navigation = _create_mock_navigation()
        navigation.final_approach.nav_without_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=coordinator,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(polling_poi)
        controller.final_approach.confirmed_recorded = True
        controller.phase.current = NavState.NAV
        controller.clock.source_fallback_s = Mock(return_value=42.0)

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()
        navigation.nav.reset_mock()
        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        self.assertEqual(polling.get_detect_data.call_count, 2)
        navigation.nav.assert_not_called()
        self.assertTrue(controller.navigation_failures.failed)

    def test_mixed_coordinator_discards_other_source_events_each_polling_tick(self):
        source_poi = _create_detected_poi(obj_id=7)
        _set_poi_source(source_poi, "gimbal_source")
        polling_poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(polling_poi, "gimbal_polling")
        source = _create_mock_detector()
        source.has_source_driven_detection_events = True
        source.drain_detection_events.side_effect = lambda _request: [
            _source_event(
                [source_poi],
                primary_poi=source_poi,
                source_timestamp_s=20.0,
                source_name="gimbal_source",
            ),
        ]
        polling = _create_mock_detector(
            detections=[polling_poi], primary_poi=polling_poi,
        )
        polling.has_source_driven_detection_events = False
        coordinator = DetectionCoordinator([source, polling], Mock())
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=coordinator,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(polling_poi)
        controller.final_approach.confirmed_recorded = True
        controller.phase.current = NavState.NAV

        for _ in range(5):
            controller.sensor.sense()
            controller.final_approach_nav.act_nav()
            self.assertEqual(controller.detections.pending_events, [])

        self.assertEqual(navigation.nav.call_count, 5)
        for args in navigation.nav.call_args_list:
            self.assertIs(args.args[0], polling_poi)

    def test_act_nav_fails_when_raw_source_clock_stalls(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        receipt_now_s = [1000.5]
        poi.replace_pixel(replace(poi.pixel, source_timestamp_s=20.0))
        poi.replace_timing(replace(
            poi.timing,
            detection_timestamp_s=20.0,
            camera_frame_timestamp_s=20.0,
            detection_now_s=lambda: 20.0,
            source_receipt_timestamp_s=1000.0,
            source_receipt_now_s=lambda: receipt_now_s[0],
        ))
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True
        controller.sensor.sense()
        controller.phase.current = NavState.NAV

        controller.final_approach_nav.act_nav()
        self.assertFalse(controller.navigation_failures.failed)

        receipt_now_s[0] = 1001.25
        controller.final_approach_nav.act_nav()

        self.assertFalse(controller.navigation_failures.failed)
        navigation.nav.assert_not_called()

    def test_act_nav_fails_when_source_stalls_after_empty_event(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(detections=[])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event(
                [],
                source_timestamp_s=20.0,
                source_receipt_timestamp_s=1000.0,
                source_name="gimbal_0",
            ),
        ]
        navigation = _create_mock_navigation()
        navigation.final_approach.nav_without_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True
        controller.phase.current = NavState.NAV
        receipt_now_s = [1000.5]
        controller.clock.source_fallback_s = lambda: receipt_now_s[0]

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_failures.failed)

        detector.drain_detection_events.return_value = []
        receipt_now_s[0] = 1001.25
        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_failures.failed)

    def test_confirm_freshness_requires_source_receipt_liveness(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        poi.replace_timing(replace(
            poi.timing,
            camera_frame_timestamp_s=20.0,
            detection_now_s=lambda: 20.0,
            source_receipt_timestamp_s=1000.0,
            source_receipt_now_s=lambda: 1001.01,
        ))
        controller = _create_controller()
        controller.detections.detected_pois = [poi]

        self.assertFalse(controller.freshness.is_poi_fresh_for_confirm(11))

        poi.replace_timing(replace(
            poi.timing,
            source_receipt_now_s=lambda: 1000.5,
        ))
        self.assertTrue(controller.freshness.is_poi_fresh_for_confirm(11))

        poi.replace_timing(replace(
            poi.timing,
            detection_now_s=lambda: 19.9,
        ))
        self.assertFalse(controller.freshness.is_poi_fresh_for_confirm(11))

    def test_confirm_receipt_liveness_expands_at_half_speed(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        poi.replace_timing(replace(
            poi.timing,
            camera_frame_timestamp_s=20.0,
            detection_now_s=lambda: 20.75,
            source_receipt_timestamp_s=1000.0,
            source_receipt_now_s=lambda: 1001.5,
        ))
        cadence = Mock()
        cadence.wall_period_for_scheduler_period.side_effect = lambda period: (
            period / 0.5
        )
        controller = _create_controller(scheduler_cadence=cadence)
        controller.detections.detected_pois = [poi]

        self.assertTrue(controller.freshness.is_poi_fresh_for_confirm(11))

    def test_confirm_receipt_liveness_contracts_at_ten_times_speed(self):
        receipt_now_s = [1000.09]
        poi = _create_detected_poi(obj_id=1, task_id=11)
        poi.replace_timing(replace(
            poi.timing,
            camera_frame_timestamp_s=20.0,
            detection_now_s=lambda: 20.75,
            source_receipt_timestamp_s=1000.0,
            source_receipt_now_s=lambda: receipt_now_s[0],
        ))
        cadence = Mock()
        cadence.wall_period_for_scheduler_period.side_effect = lambda period: (
            period / 10.0
        )
        controller = _create_controller(scheduler_cadence=cadence)
        controller.detections.detected_pois = [poi]

        self.assertTrue(controller.freshness.is_poi_fresh_for_confirm(11))
        receipt_now_s[0] = 1000.11
        self.assertFalse(controller.freshness.is_poi_fresh_for_confirm(11))

    def test_act_nav_does_not_replay_source_driven_latest_cache(self):
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        detector.has_source_driven_detection_events = True
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.record_confirmed_detection = Mock(return_value=True)
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()
        navigation.final_approach.record_confirmed_detection.assert_not_called()

    def test_act_nav_does_not_synthesize_gap_without_source_event(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(detections=[])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = []
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()
        navigation.final_approach.nav_without_detection.assert_not_called()

    def test_act_nav_fails_closed_on_untimestamped_source_gap(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        _set_poi_source(poi, "gimbal_0")
        detector = _create_mock_detector(detections=[])
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([], source_name="gimbal_0"),
        ]
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True
        controller.clock.source_fallback_s = Mock(
            side_effect=AssertionError("must not substitute controller time"),
        )

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_failures.failed)
        navigation.nav.assert_not_called()
        navigation.final_approach.nav_without_detection.assert_not_called()
        controller.clock.source_fallback_s.assert_not_called()
        self.assertEqual(controller.detections.pending_events, [])

    def test_act_nav_consumes_source_event_before_callback_exception(self):
        poi = _create_detected_poi(obj_id=1, task_id=11)
        detector = _create_mock_detector(
            detections=[poi], primary_poi=poi,
        )
        detector.has_source_driven_detection_events = True
        detector.drain_detection_events.return_value = [
            _source_event([poi], primary_poi=poi),
        ]
        navigation = _create_mock_navigation()
        navigation.nav.side_effect = RuntimeError("callback failed")
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
            detector=detector,
            navigation=navigation,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = True

        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        with self.assertRaisesRegex(RuntimeError, "callback failed"):
            controller.final_approach_nav.act_nav()

        self.assertEqual(controller.detections.pending_events, [])
        navigation.nav.side_effect = None
        controller.final_approach_nav.act_nav()
        navigation.nav.assert_called_once_with(poi)

    def test_act_nav_sets_failure_flag_on_failure(self):
        """_act_nav sets _navigation_failed flag when navigation fails."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        navigation.nav.return_value = False
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_failures.failed)

    def test_act_nav_holds_until_guided_mode_observed(self):
        """_act_nav does not send navigation commands before GUIDED is active."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        vehicle.set_mode.assert_called_with(FlightMode.GUIDED)
        navigation.nav.assert_not_called()
        self.assertFalse(controller.navigation_task.nav_mode_observed)

    def test_act_nav_marks_navigation_task_started_when_guided_observed(self):
        """_act_nav can issue the first final-approach command (already GUIDED)
        before the next _decide tick; it must mark the navigation task as running so
        a subsequent abort still logs the SNAP + performs the one-shot disarm."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)  # already GUIDED
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation)
        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        self.assertFalse(controller.navigation_task.final_approach_navigation_active)

        controller.final_approach_nav.act_nav()

        self.assertTrue(controller.navigation_task.nav_mode_observed)
        self.assertTrue(controller.navigation_task.final_approach_navigation_active)

    def test_act_nav_waits_when_no_detection(self):
        """_act_nav waits when POI not in current detections."""
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[])  # No detections
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        # Should not call nav
        navigation.nav.assert_not_called()
        # Should keep active POI
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_act_nav_no_active_poi(self):
        """_act_nav does nothing with no active POI."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        navigation.nav.assert_not_called()


# =============================================================================
# Act Phase Tests - RESET State
# =============================================================================

class TestNavControllerActReset(unittest.TestCase):
    """Tests for _act in RESET state."""

    def test_act_reset_pauses_navigation(self):
        """RESET state pauses navigation (transient, no per-tick action)."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        # Force into RESET with prev_state synced (no transition callback)
        controller.phase.current = NavState.RESET
        controller.phase.previous = NavState.RESET
        controller.actions.act()

        navigation.pause_final_approach.assert_called()

    def test_act_reset_enter_clears_state(self):
        """Entering RESET calls _enter_reset (clear state + logger refresh)."""
        detector = _create_mock_detector()
        args = _create_mock_args()
        logger = Mock()
        controller = _create_controller(detector=detector, args=args, logger=logger)

        # Transition from DETECT → RESET
        controller.phase.previous = NavState.DETECT
        controller.phase.current = NavState.RESET
        controller.actions.act()

        # _enter_reset clears state and refreshes logger
        detector.refresh.assert_called()
        args.refresh.assert_called()
        logger.refresh.assert_called()


# =============================================================================
# Act Phase Tests - ONHOLD State
# =============================================================================

class TestNavControllerActOnhold(unittest.TestCase):
    """Tests for _act in ONHOLD state."""

    def test_act_onhold_pauses_navigation(self):
        """ONHOLD state pauses navigation."""
        vehicle = _create_mock_vehicle(mode=FlightMode.MANUAL)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        controller.phase.current = NavState.ONHOLD
        controller.actions.act()

        navigation.pause_final_approach.assert_called()


# =============================================================================
# POI Selection Tests
# =============================================================================

class TestNavControllerPoiSelection(unittest.TestCase):
    """Tests for POI selection logic."""

    def test_select_poi_empty_list(self):
        """_select_poi returns None for empty list."""
        controller = _create_controller()
        self_poi, peer_pois = controller.selector.select([])

        self.assertIsNone(self_poi)
        self.assertEqual(peer_pois, [])

    def test_select_poi_single_poi(self):
        """_select_poi returns the only POI as self POI."""
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)

        self_poi, peer_pois = controller.selector.select([poi])

        self.assertEqual(self_poi.identity.obj_id, 1)
        self.assertEqual(peer_pois, [])

    def test_select_poi_no_peers_without_task_actor(self):
        """_select_poi returns no peers when no task_actor."""
        controller = _create_controller()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)

        self_poi, peer_pois = controller.selector.select([t1, t2])

        self.assertEqual(self_poi.identity.obj_id, 1)
        self.assertEqual(peer_pois, [])

    def test_select_poi_peers_with_task_actor(self):
        """_select_poi returns remaining as peers when task_actor exists."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        self_poi, peer_pois = controller.selector.select([t1, t2, t3])

        self.assertEqual(self_poi.identity.obj_id, 1)
        self.assertEqual(len(peer_pois), 2)

    def test_select_poi_uses_primary_poi_when_no_active(self):
        """_select_poi prefers detector primary POI when no active POI exists."""
        controller = _create_controller()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        controller.detections.primary_poi = t2

        self_poi, _ = controller.selector.select(
            [t1, t2], controller.detections.primary_poi,
        )

        self.assertEqual(self_poi.identity.obj_id, 2)

    def test_select_poi_skips_processed_primary_poi(self):
        """Processed primary POI is skipped and next valid POI is chosen."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        controller.detections.primary_poi = t1
        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)

        self_poi, peer_pois = controller.selector.select(
            [t1, t2], controller.detections.primary_poi,
        )

        self.assertEqual(self_poi.identity.obj_id, 2)
        self.assertIn(t1, peer_pois)

    def test_select_poi_active_poi_stable_across_reorder(self):
        """Active POI (obj_id=1) is returned as self even when detections reorder."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        # Set active POI to P1
        controller.confirmation_manager.set_active_poi(t1)

        # Detections arrive in reordered: [P2, P1, P3]
        self_poi, peer_pois = controller.selector.select([t2, t1, t3])

        self.assertEqual(self_poi.identity.obj_id, 1)
        peer_ids = [p.identity.obj_id for p in peer_pois]
        self.assertEqual(peer_ids, [2, 3])

    def test_select_poi_active_poi_not_in_peers(self):
        """When active POI exists, it never appears in peer_pois."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        controller.confirmation_manager.set_active_poi(t1)

        # Test multiple orderings
        for ordering in [[t1, t2, t3], [t2, t1, t3], [t3, t2, t1]]:
            _, peer_pois = controller.selector.select(ordering)
            peer_ids = [p.identity.obj_id for p in peer_pois]
            self.assertNotIn(
                1,
                peer_ids,
                f"Active POI leaked into peers for ordering "
                f"{[t.identity.obj_id for t in ordering]}",
            )

    def test_select_poi_active_lost_returns_none(self):
        """When active POI not in detections, self_poi is None."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        # Active POI is P1 but it's not in detections
        controller.confirmation_manager.set_active_poi(t1)

        self_poi, peer_pois = controller.selector.select([t2, t3])

        self.assertIsNone(self_poi)
        peer_ids = [p.identity.obj_id for p in peer_pois]
        self.assertEqual(peer_ids, [2, 3])


# =============================================================================
# Peer POI Notification Tests
# =============================================================================

class TestNavControllerPeerNotify(unittest.TestCase):
    """Tests for _notify_peer_pois behavior."""

    def test_notify_peer_pois_excludes_active(self):
        """Active POI is never sent to TaskActor even if in peer_pois list."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)

        # Set P1 as active POI
        controller.confirmation_manager.set_active_poi(t1)

        # Call _notify_peer_pois with P1 in the list (simulating the bug)
        controller.peer_notifier.notify([t1, t2])

        # Only P2 should be notified
        controller.network.task_actor.notify_pois.assert_called_once()
        notified = controller.network.task_actor.notify_pois.call_args[0][0]
        notified_ids = [t.identity.obj_id for t in notified]
        self.assertNotIn(1, notified_ids)
        self.assertIn(2, notified_ids)

    def test_notify_peer_pois_keeps_same_local_obj_id_with_different_task_id(self):
        """Peer POIs are filtered by task identity, not by detector-local obj_id."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)
        controller.network.task_actor = Mock()

        active = _create_detected_poi(obj_id=1, task_id=101)
        peer_same_local = _create_detected_poi(obj_id=1, task_id=202)
        controller.confirmation_manager.set_active_poi(active)

        controller.peer_notifier.notify([peer_same_local])

        controller.network.task_actor.notify_pois.assert_called_once()
        notified = controller.network.task_actor.notify_pois.call_args[0][0]
        self.assertEqual(notified, [peer_same_local])

    def test_notify_peer_pois_skips_already_notified(self):
        """POIs with existing status are not re-appended to notify list."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)

        # Mark P1 as already notified
        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)

        controller.peer_notifier.notify([t1, t2])

        # Only P2 should be notified
        controller.network.task_actor.notify_pois.assert_called_once()
        notified = controller.network.task_actor.notify_pois.call_args[0][0]
        notified_ids = [t.identity.obj_id for t in notified]
        self.assertEqual(notified_ids, [2])

    def test_notify_peer_pois_all_already_notified(self):
        """No notification when all POIs already have status."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)

        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)
        controller.confirmation_manager.update_status(t2, ConfirmationStatus.PEER_NOTIFIED)

        controller.peer_notifier.notify([t1, t2])

        controller.network.task_actor.notify_pois.assert_not_called()

    def test_notify_peer_pois_uses_debug_loc_in_sim(self):
        """In simulation, use t_g_loc_debug instead of calc_t_g_loc."""
        navigation = _create_mock_navigation()
        detector = _create_mock_detector()
        detector.is_simulation = True
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.network.task_actor = Mock()

        debug_loc = Location(40.123, -74.456, 100.0)
        t1 = _create_detected_poi(obj_id=1)
        _set_poi_truth_location(t1, debug_loc)

        controller.peer_notifier.notify([t1])

        # calc_t_g_loc should NOT be called
        navigation.legacy_pois.ground_location.assert_not_called()
        # p_t_g_l should be set to the debug location
        self.assertEqual(t1.geo.projected_poi_location, debug_loc)

    def test_notify_peer_pois_falls_back_to_georef_in_real(self):
        """In real mode, calc_t_g_loc is used even if t_g_loc_debug is set."""
        navigation = _create_mock_navigation()
        detector = _create_mock_detector()
        detector.is_simulation = False
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        _set_poi_truth_location(
            t1,
            Location(40.123, -74.456, 100.0),
        )

        controller.peer_notifier.notify([t1])

        # calc_t_g_loc SHOULD be called, with allow_fallback=False so a near-flat
        # ray cannot notify a phantom max-distance POI (run 174154).
        navigation.legacy_pois.ground_location.assert_called_once_with(t1, allow_fallback=False)

    def test_notify_peer_pois_falls_back_when_debug_loc_none(self):
        """In simulation, fall back to calc_t_g_loc when t_g_loc_debug is None."""
        navigation = _create_mock_navigation()
        detector = _create_mock_detector()
        detector.is_simulation = True
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        _set_poi_truth_location(t1, None)

        controller.peer_notifier.notify([t1])

        # calc_t_g_loc SHOULD be called as fallback, with allow_fallback=False
        # (no phantom max-distance POI — run 174154).
        navigation.legacy_pois.ground_location.assert_called_once_with(t1, allow_fallback=False)


# =============================================================================
# Find Active POI Detection Tests
# =============================================================================

class TestNavControllerFindActivePoi(unittest.TestCase):
    """Tests for _find_active_poi_detection."""

    def test_find_active_poi_no_active(self):
        """_find_active_poi_detection returns None with no active POI."""
        controller = _create_controller()

        result = controller.source.find_active_poi_detection()

        self.assertIsNone(result)

    def test_find_active_poi_found(self):
        """_find_active_poi_detection finds matching detection."""
        poi = _create_detected_poi(obj_id=5)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        controller.confirmation_manager.set_active_poi(poi)
        controller.sensor.sense()

        result = controller.source.find_active_poi_detection()

        self.assertIsNotNone(result)
        self.assertEqual(result.identity.obj_id, 5)

    def test_find_active_poi_not_in_detections(self):
        """_find_active_poi_detection returns None if not in detections."""
        active = _create_detected_poi(obj_id=5)
        other = _create_detected_poi(obj_id=10)
        detector = _create_mock_detector(detections=[other])
        controller = _create_controller(detector=detector)

        controller.confirmation_manager.set_active_poi(active)
        controller.sensor.sense()

        result = controller.source.find_active_poi_detection()

        self.assertIsNone(result)

    def test_find_active_poi_uses_task_id_not_local_obj_id(self):
        """_find_active_poi_detection matches by task identity when local IDs collide."""
        active = _create_detected_poi(obj_id=5, task_id=101)
        wrong = _create_detected_poi(obj_id=5, task_id=202)
        match = _create_detected_poi(obj_id=5, task_id=101)
        controller = _create_controller()
        controller.confirmation_manager.set_active_poi(active)
        controller.detections.detected_pois = [wrong, match]

        result = controller.source.find_active_poi_detection()

        self.assertIs(result, match)


# =============================================================================
# Pass Detection Tests
# =============================================================================

class TestNavControllerPassDetection(unittest.TestCase):
    """Tests for pass/approach detection."""

    def test_passed_detection_wp_before_min(self):
        """_passed_detection_wp returns False before min waypoint."""
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(args=args)

        result = controller.mission_pass.passed_detection_waypoint(2)

        self.assertFalse(result)

    def test_passed_detection_wp_at_min(self):
        """_passed_detection_wp returns False at exactly min waypoint."""
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(args=args)

        result = controller.mission_pass.passed_detection_waypoint(3)

        self.assertFalse(result)

    def test_passed_detection_wp_after_min(self):
        """_passed_detection_wp returns True after min waypoint."""
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(args=args)

        result = controller.mission_pass.passed_detection_waypoint(5)

        self.assertTrue(result)

    def test_passed_poi_wp_no_distance(self):
        """_passed_poi_wp returns False when distance is None."""
        navigation = _create_mock_navigation()
        navigation.legacy_pois.locked_distance.return_value = None
        controller = _create_controller(navigation=navigation)

        result = controller.mission_pass.passed_poi()

        self.assertFalse(result)

    def test_passed_poi_wp_far_away(self):
        """_passed_poi_wp returns False when far from POI."""
        navigation = _create_mock_navigation()
        navigation.legacy_pois.locked_distance.return_value = 1000.0
        controller = _create_controller(navigation=navigation)

        result = controller.mission_pass.passed_poi()

        self.assertFalse(result)

    def test_passed_poi_wp_distance_increasing(self):
        """_passed_poi_wp returns True when distance increases."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        # Simulate approaching then passing
        navigation.legacy_pois.locked_distance.return_value = 30.0  # Close
        controller.mission_pass.passed_poi()  # Start approach

        navigation.legacy_pois.locked_distance.return_value = 40.0  # Increasing
        result = controller.mission_pass.passed_poi()

        self.assertTrue(result)

    def test_passed_poi_wp_sparse_sample_can_jump_beyond_close_radius(self):
        """At 10x, 20 m -> 70 m is a pass, not a new far approach."""
        navigation = _create_mock_navigation()
        navigation.legacy_pois.locked_distance.side_effect = [20.0, 70.0]
        controller = _create_controller(navigation=navigation)

        self.assertFalse(controller.mission_pass.passed_poi())
        self.assertTrue(controller.mission_pass.passed_poi())

    def test_passed_poi_wp_prefers_nav_state_coordinate(self):
        """Exact assigned/orbit GPS owns pass/reset, not final-approach visual geo."""
        navigation = _create_mock_navigation()
        navigation.legacy_pois.locked_distance.return_value = None
        vehicle = _create_mock_vehicle()
        current = Location(40.0, 44.0, 100.0, is_absolute=True)
        vehicle.location.return_value = current
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        assigned = Location(40.001, 44.001, 0.0, is_absolute=True)
        controller.navigation_task.navigation_poi_location = assigned

        with patch(
                "navpy.modules.nav.mission_navigation.GeoRefCalc.calculate_distance",
                side_effect=[30.0, 40.0],
        ) as distance:
            self.assertFalse(controller.mission_pass.passed_poi())
            self.assertTrue(controller.mission_pass.passed_poi())

        navigation.legacy_pois.locked_distance.assert_not_called()
        self.assertEqual(distance.call_count, 2)
        self.assertIs(distance.call_args.args[0], current)
        self.assertIs(distance.call_args.args[1], assigned)

    def test_vision_nav_pass_override_blocks_all_coordinate_fallbacks(self):
        """Pure-vision completion is owned only by the visual runtime latch."""
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.poi_passed_override = Mock(side_effect=[False, True])
        navigation.legacy_pois.locked_distance = Mock(
            side_effect=AssertionError("locked range entered pure-vision pass path")
        )
        vehicle = _create_mock_vehicle()
        vehicle.location = Mock(
            side_effect=AssertionError("GPS entered pure-vision pass path")
        )
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.navigation_task.navigation_poi_location = Location(
            40.001, 44.001, 100.0, is_absolute=True,
        )

        with patch(
                "navpy.modules.nav.mission_navigation.GeoRefCalc.calculate_distance",
                side_effect=AssertionError("geo distance entered pure-vision pass path"),
        ):
            self.assertFalse(controller.mission_pass.passed_poi())
            self.assertTrue(controller.mission_pass.passed_poi())

        vehicle.location.assert_not_called()
        navigation.legacy_pois.locked_distance.assert_not_called()


# =============================================================================
# Peer Navigation Tests
# =============================================================================

class TestNavControllerPeerNavigation(unittest.TestCase):
    """Tests for peer navigation setup and flow."""

    def test_setup_peer_navigation_sets_flag(self):
        """_setup_peer_navigation sets peer_navigation flag."""
        vehicle = _create_mock_vehicle()
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        # Setup task actor with selected POI
        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.001
        location_msg.lng = -74.001
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        controller.peer_navigation.setup()

        self.assertTrue(controller.navigation_task.peer_navigation)

    def test_final_approach_gimbal_orbit_uses_demo_standoff_for_dock(self):
        """SIYI final-approach owner/peers use the same verified 300 m orbit."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 25.0
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 90.0
        poi = Location(40.001, 44.001, 100.0, is_absolute=True)
        camera_sized = ApproachPlan(
            kind=ApproachKind.ORBIT,
            approach_location=poi,
            offset_distance=0.0,
            orbit_radius=666.0,
        )

        with patch(
                "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
                return_value=camera_sized,
        ):
            plan, _ = controller.fallback_navigation.plan_orbit_approach(
                poi,
                class_id=0,
                drone_location=Location(40.0, 44.0, 135.0, is_absolute=True),
            )

        self.assertEqual(plan.orbit_radius, MIN_APPROACH_STANDOFF_M)

    def test_final_approach_gimbal_orbit_uses_relative_height_for_air_pois(self):
        """Co-altitude absolute POIs must not turn AMSL into AGL."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        vehicle = _create_mock_vehicle()
        vehicle.home_location = Location(
            40.0, 44.0, 1000.0, is_absolute=True,
        )
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 25.0
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0

        for height_above_poi_m in (0.0, 9.0, 10.0, 11.0):
            poi = Location(
                40.001,
                44.001,
                1150.0 - height_above_poi_m,
                is_absolute=True,
            )
            camera_sized = ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=poi,
                offset_distance=0.0,
                orbit_radius=666.0,
            )
            with self.subTest(height_above_poi_m=height_above_poi_m), patch(
                    "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
                    return_value=camera_sized,
            ):
                plan, _ = controller.fallback_navigation.plan_orbit_approach(
                    poi,
                    class_id=0,
                    drone_location=Location(
                        40.0, 44.0, 1150.0, is_absolute=True,
                    ),
                )
                self.assertEqual(plan.orbit_radius, MIN_APPROACH_STANDOFF_M)

    def test_setup_peer_navigation_sets_sim_poi(self):
        """_setup_peer_navigation sets sim POI for simulation."""
        vehicle = _create_mock_vehicle()
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, detector=detector, navigation=navigation)

        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.001
        location_msg.lng = -74.001
        location_msg.alt = 584.0  # AMSL altitude — should NOT be passed to sim POI
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        controller.peer_navigation.setup()

        detector.set_sim_poi.assert_called_once_with(8, ANY)  # mission_items_count - 2
        sim_poi_loc = detector.set_sim_poi.call_args[0][1]
        assert sim_poi_loc.alt == 0  # must be home-relative zero, not AMSL

    def test_setup_peer_navigation_calls_peer_poi(self):
        """_setup_peer_navigation calls navigation.vehicle_commands.peer_poi."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.001
        location_msg.lng = -74.001
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        controller.peer_navigation.setup()

        navigation.vehicle_commands.peer_poi.assert_called_once()

    def test_peer_nav_proceeds_with_warning_when_save_fails(self):
        """Peer-nav still dispatches when WP_LOITER_RAD save fails — the
        orbit needs the radius written to function. The cost (an
        unrestorable pin) is a known soft leak; we log a warning and
        proceed.
        """
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=None)  # save fails
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.OFFSET,
                approach_location=Location(40.001, -74.001, 100.0),
                offset_distance=200.0,
                orbit_radius=80.0,
            ),
        ):
            controller.network.task_actor = Mock()
            location_msg = Mock(lat=40.001, lng=-74.001, alt=0.0)
            controller.network.task_actor.selected_poi.return_value = Mock(
                location=location_msg,
            )
            controller.peer_navigation.setup()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertIsNone(controller.loiter_radius.original)

    def test_default_delivery_hub_proceeds_with_warning_when_save_fails(self):
        """Default delivery hub nav: same warning-and-proceed contract as peer-nav."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=None)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.mission.default_delivery_hub = Location(40.001, -74.001, 100.0)

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 100.0),
                offset_distance=0.0,
                orbit_radius=300.0,
            ),
        ):
            controller.fallback_navigation.setup()

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertTrue(controller.mission.default_delivery_hub_active)
        self.assertIsNone(controller.loiter_radius.original)

    def test_default_delivery_hub_pass_coordinate_is_normalized_to_absolute_altitude(self):
        """Mission DDH altitude is home-relative; pass distance uses AMSL."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.home_location = Location(
            40.0, 44.0, 1295.0, is_absolute=True,
        )
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.mission.default_delivery_hub = Location(40.001, 44.001, 5.0)
        plan = ApproachPlan(
            kind=ApproachKind.ORBIT,
            approach_location=controller.mission.default_delivery_hub,
            offset_distance=0.0,
            orbit_radius=300.0,
        )

        with patch.object(
                controller.fallback_navigation, "plan_orbit_approach",
                return_value=(plan, 115.0),
        ):
            controller.fallback_navigation.setup()

        self.assertEqual(
            controller.navigation_task.navigation_poi_location,
            Location(40.001, 44.001, 1300.0, is_absolute=True),
        )

    def test_orbit_self_detect_proceeds_with_warning_when_save_fails(self):
        """ORBIT self-detect branch in navigation_task_action.start: same contract."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=None)
        navigation = _create_mock_navigation()
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 100.0),
                offset_distance=0.0,
                orbit_radius=300.0,
            ),
        ):
            controller.navigation_task_action.start(poi)

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        self.assertEqual(controller.navigation_task.orbit_radius_m, 300.0)
        self.assertIsNone(controller.loiter_radius.original)

    def test_final_approach_self_detect_does_not_command_poi_geo_orbit(self):
        """Pure-vision self detection starts from pixels, never POI geo."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        poi_loc = Location(
            poi.geo.truth_poi_location.lat,
            poi.geo.truth_poi_location.lng,
            100.0,
            is_absolute=True,
        )
        _set_poi_truth_location(poi, poi_loc)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )
        controller.fallback_navigation.plan_orbit_approach = Mock(return_value=(
            ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=poi_loc,
                offset_distance=300.0,
                orbit_radius=300.0,
            ),
            115.0,
        ))

        controller.navigation_task_action.start(poi)

        controller.fallback_navigation.plan_orbit_approach.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()
        self.assertEqual(controller.navigation_task.orbit_radius_m, 0.0)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)

    def test_offset_self_detect_does_not_dispatch_for_vision_nav(self):
        """Pure-vision navigation task cannot derive an offset from POI geo."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.OFFSET,
        )
        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(40.001, -74.001, 100.0),
            offset_distance=200.0,
            orbit_radius=80.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.navigation_task_action.start(poi)

        calc.assert_not_called()
        dispatch.assert_not_called()

    def test_ideal_sim_truth_is_renderer_only_not_approach_or_pass_state(self):
        """Ideal POI GPS remains renderer input, never navigation task state."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        debug_loc = Location(40.001, -74.001, 1296.0, is_absolute=True)
        _set_poi_truth_location(poi, debug_loc)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        # Terrain off: geo-ref localization is unavailable.
        navigation.legacy_pois.ground_location = Mock(return_value=None)
        # Pass ownership stays with NavController (vision-nav law).
        navigation.final_approach.poi_passed_override = Mock(return_value=False)
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.OFFSET,
        )
        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(40.002, -74.002, 100.0),
            offset_distance=200.0,
            orbit_radius=0.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.navigation_task_action.start(poi)

        # Pure-vision self-detect must not seed or calculate POI geo.
        navigation.legacy_pois.ground_location.assert_not_called()
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        calc.assert_not_called()
        dispatch.assert_not_called()

        # With no optical completion latch, the pass gate remains false and
        # must not consult the legacy coordinate-distance detector.
        with patch(
            "navpy.modules.nav.mission_navigation.GeoRefCalc.calculate_distance",
        ) as distance:
            self.assertFalse(controller.mission_pass.passed_poi())
        distance.assert_not_called()
        # The locked-distance fallback (navigation layer) is never consulted.
        navigation.legacy_pois.locked_distance.assert_not_called()

    def test_offset_self_detect_real_detector_still_avoids_poi_geo(self):
        """The pure-vision firewall applies equally to real detections."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        _set_poi_truth_location(
            poi,
            Location(40.009, -74.009, 999.0, is_absolute=True),
        )
        geo_loc = Location(40.002, -74.002, 95.0, is_absolute=True)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        detector = _create_mock_detector()
        detector.is_simulation = False
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.legacy_pois.ground_location = Mock(return_value=geo_loc)
        controller = create_nav_test_rig(
            vehicle, detector, navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.OFFSET,
        )
        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(40.003, -74.003, 100.0),
            offset_distance=200.0,
            orbit_radius=0.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.navigation_task_action.start(poi)

        # allow_fallback=False: a near-flat ray must yield no-fix, never the
        # max-distance phantom (parity with the ORBIT branch — run 174154).
        navigation.legacy_pois.ground_location.assert_not_called()
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        calc.assert_not_called()
        dispatch.assert_not_called()

    def test_offset_self_detect_real_detector_no_geo_needs_no_localization(self):
        """Pure-vision path skips localization, independent of fix quality."""
        from navpy.modules.navigation.approach_strategy import ApproachKind
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector()
        detector.is_simulation = False
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.legacy_pois.ground_location = Mock(return_value=None)
        logger = Mock()
        controller = create_nav_test_rig(
            _create_mock_vehicle(), detector, navigation,
            _create_mock_args(), logger, approach_kind=ApproachKind.OFFSET,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.navigation_task_action.start(poi)

        navigation.legacy_pois.ground_location.assert_not_called()
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        calc.assert_not_called()
        dispatch.assert_not_called()
        skip_warnings = [
            c for c in logger.warning.call_args_list
            if "skipping offset approach" in c.args[0]
        ]
        self.assertEqual(len(skip_warnings), 0)

    def test_offset_self_detect_real_flat_ray_cannot_enter_pure_vision_path(self):
        """Even a callable phantom localizer stays outside pure navigation_task."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        phantom = Location(40.045, -74.045, 250.0, is_absolute=True)

        def localize(detect_data, poi_ned=None, allow_fallback=True):
            return None if not allow_fallback else phantom

        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        detector = _create_mock_detector()
        detector.is_simulation = False
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.legacy_pois.ground_location = Mock(side_effect=localize)
        logger = Mock()
        controller = create_nav_test_rig(
            vehicle, detector, navigation,
            _create_mock_args(), logger, approach_kind=ApproachKind.OFFSET,
        )
        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(40.002, -74.002, 100.0),
            offset_distance=200.0,
            orbit_radius=80.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ) as calc, patch.object(
            controller.vehicle_navigation, "dispatch_approach",
        ) as dispatch, patch.object(
            controller.vehicle_navigation, "request_guided",
        ) as guided, patch.object(
            controller.vehicle_navigation, "save_loiter_radius",
        ) as save_rad:
            controller.navigation_task_action.start(poi)

        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        navigation.legacy_pois.ground_location.assert_not_called()
        calc.assert_not_called()
        dispatch.assert_not_called()
        guided.assert_not_called()
        save_rad.assert_not_called()
        skip_warnings = [
            c for c in logger.warning.call_args_list
            if "skipping offset approach" in c.args[0]
        ]
        self.assertEqual(len(skip_warnings), 0)

    def test_offset_self_detect_skipped_for_legacy_law(self):
        """OFFSET self-detect dispatch does NOT fire for a legacy (non
        vision-nav) fixed-camera config: classify_approach assigns OFFSET
        to every fixed camera, and legacy PN must keep its dev behaviour (no
        lock-time GUIDED switch / offset dispatch)."""
        from navpy.modules.navigation.approach_strategy import ApproachKind
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=80.0)
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = False  # legacy PN/PID
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.OFFSET,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.navigation_task_action.start(poi)

        calc.assert_not_called()
        dispatch.assert_not_called()
        self.assertEqual(controller.navigation_task.orbit_radius_m, 0.0)

    def test_self_detect_early_start_navigation_task_loiters_at_scan_alt(self):
        """Early self-detect (starting navigation mid-climb) sizes the orbit and
        commands the loiter at the mission scan/navigation task altitude, not
        the climbing altitude — the regression the SITL re-run caught."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        poi = _create_detected_poi(obj_id=1)
        # Self-detect orbit now prefers the known sim location (the geo-ref
        # back-projection can hit the far max-distance fallback mid-climb —
        # run 174154); set it to match calc_t_g_loc so planning is identical
        # whichever path runs.
        _set_poi_truth_location(
            poi,
            Location(40.001, -74.001, 1296.0, is_absolute=True),
        )
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        # Climbing: location(True) returns a low relative altitude.
        vehicle.location = Mock(return_value=Location(40.0, -74.0, 35.0))
        navigation = _create_mock_navigation()
        navigation.legacy_pois.ground_location = Mock(
            return_value=Location(40.001, -74.001, 1296.0, is_absolute=True))
        controller = create_nav_test_rig(
            vehicle, _create_mock_detector(), navigation,
            _create_mock_args(), Mock(), approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0  # mission scan/navigation task alt

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 1296.0),
                offset_distance=0.0, orbit_radius=300.0),
        ) as calc:
            controller.navigation_task_action.start(poi)

        # The absolute (terrain-MSL) self-detect POI makes the planning
        # pose ABSOLUTE at home MSL + navigation task alt (100 + 150), not the
        # climbing 35 m — so r_nav_min's height-above-POI is correct
        # for any home elevation, not just home_MSL ~= 0.
        drone_arg = calc.call_args.args[1]
        self.assertTrue(drone_arg.is_absolute)
        self.assertAlmostEqual(drone_arg.alt, 100.0 + 150.0)
        # Loiter still commanded at the relative navigation task altitude.
        self.assertEqual(
            navigation.vehicle_commands.peer_poi_loiter.call_args.args[2], 150.0)
        self.assertEqual(controller.navigation_task.orbit_approach_alt_rel_m, 150.0)

    def test_dispatch_approach_with_radius_uses_peer_poi_loiter(self):
        """ApproachPlan with a positive loiter radius must route through
        peer_poi_loiter, so the radius travels in the DO_REPOSITION command
        (param3) instead of inheriting a stale autopilot WP_LOITER_RAD value.
        """
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )

        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(40.001, -74.001, 100.0),
            offset_distance=200.0,
            orbit_radius=80.0,
        )
        controller.vehicle_navigation.dispatch_approach(plan)

        navigation.vehicle_commands.peer_poi_loiter.assert_called_once()
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_peer_navigation_not_started_twice(self):
        """Peer navigation is not started twice."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.001
        location_msg.lng = -74.001
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        # Already in peer navigation
        controller.navigation_task.peer_navigation = True

        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should not call peer_poi again
        navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_peer_navigation_suppresses_distant_poi(self):
        """During peer navigation, self-detected POIs are suppressed when far from peer POI."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        # Vehicle is at (40.0, -74.0) — far from the peer POI
        vehicle.location = Mock(return_value=Location(40.0, -74.0, 200.0))
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        # Populate detections directly (normally done by _sense)
        controller.detections.detected_pois = [_create_detected_poi(obj_id=99)]

        # Set up peer navigation to a distant location (~11 km away)
        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.1
        location_msg.lng = -74.1
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi
        controller.navigation_task.peer_navigation = True

        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should NOT select the detected POI — UAV is far from peer POI
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_vision_nav_peer_detection_does_not_read_geo_admission_gate(self):
        """Published pixels admit pure navigation task even during peer routing."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        vehicle.location = Mock(
            side_effect=AssertionError("peer geo entered pure navigation task gate")
        )
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        poi = _create_detected_poi(obj_id=42)
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.navigation_task.peer_navigation = True
        controller.network.task_actor = Mock()
        controller.network.task_actor.selected_poi.return_value = Mock()

        controller.navigation_task_action.handle_new_poi(poi)

        self.assertIs(controller.confirmation_manager.active_poi, poi)
        vehicle.location.assert_not_called()

    def test_peer_navigation_allows_nearby_poi(self):
        """During peer navigation, POIs are selected when close to peer POI."""
        # Peer POI at (40.001, -74.001) — vehicle at (40.001, -74.0012) ~15m away
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        vehicle.location = Mock(return_value=Location(40.001, -74.0012, 200.0))
        navigation = _create_mock_navigation()
        poi = _create_detected_poi(obj_id=42)
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        # Populate detections directly (normally done by _sense)
        controller.detections.detected_pois = [poi]

        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.001
        location_msg.lng = -74.001
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi
        controller.navigation_task.peer_navigation = True

        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        # Should select — UAV is close to peer POI
        self.assertEqual(controller.confirmation_manager.active_poi, poi)

    def test_near_peer_poi_no_task_actor(self):
        """_near_peer_poi returns False when no task actor."""
        controller = _create_controller()
        controller.network.task_actor = None
        self.assertFalse(controller.peer_navigation.near_poi())

    def test_near_peer_poi_no_selected_pois(self):
        """_near_peer_poi returns False when no selected POIs."""
        controller = _create_controller()
        controller.network.task_actor = Mock()
        controller.network.task_actor.selected_poi.return_value = None
        self.assertFalse(controller.peer_navigation.near_poi())

    def test_orbit_peer_start_navigation_task_uses_radius_plus_margin(self):
        """ORBIT navigation task arms within orbit_radius + PEER_APPROACH_MARGIN_M.
        A LOITER flies just outside its commanded radius, so the bare radius
        (no margin) would never arm; the margin lets the peer begin its approach."""
        from navpy.modules.navigation.approach_strategy import ApproachKind

        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        # ~153 m from the POI at (40.0, -74.0).
        vehicle.location = Mock(
            return_value=Location(40.0, -74.0018, 200.0, is_absolute=True)
        )
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        # Bare radius 100 m < 153 m would never arm; +200 m margin = 300 m does.
        controller.navigation_task.orbit_radius_m = 100.0
        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.0
        location_msg.lng = -74.0
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        self.assertTrue(controller.peer_navigation.near_poi())

    def test_orbit_start_navigation_task_threshold_follows_radius(self):
        """The ORBIT approach threshold tracks orbit_radius + PEER_APPROACH_MARGIN_M:
        a wider orbit permits approach from farther out, a tighter one requires closing
        in."""
        from navpy.modules.navigation.approach_strategy import ApproachKind

        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        # ~450 m from the POI at (40.0, -74.0).
        vehicle.location = Mock(
            return_value=Location(40.0, -74.00528, 200.0, is_absolute=True)
        )
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.network.task_actor = Mock()
        location_msg = Mock()
        location_msg.lat = 40.0
        location_msg.lng = -74.0
        location_msg.alt = 0.0
        selected_poi = Mock()
        selected_poi.location = location_msg
        controller.network.task_actor.selected_poi.return_value = selected_poi

        # ~450 m away: inside a 300 m orbit (+200 = 500), outside a 100 m one
        # (+200 = 300).
        controller.navigation_task.orbit_radius_m = 300.0
        self.assertTrue(controller.peer_navigation.near_poi())
        controller.navigation_task.orbit_radius_m = 100.0
        self.assertFalse(controller.peer_navigation.near_poi())

    def test_orbit_limits_built_from_vehicle(self):
        from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits

        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        controller = _create_controller(vehicle=vehicle)

        limits = controller.vehicle_navigation.orbit_limits()
        self.assertEqual(
            limits,
            OrbitNavigationLimits(
                airspeed_mps=22.0, roll_limit_deg=45.0, min_pitch_deg=-40.0),
        )

    def test_orbit_limits_airspeed_falls_back_when_missing(self):
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = None
        controller = _create_controller(vehicle=vehicle)

        limits = controller.vehicle_navigation.orbit_limits()
        self.assertEqual(limits.airspeed_mps, 25.0)

    def test_orbit_limits_none_when_pitch_degenerate(self):
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = 0.0  # degenerate -> cannot size the orbit
        vehicle.air_speed = 22.0
        controller = _create_controller(vehicle=vehicle)

        self.assertIsNone(controller.vehicle_navigation.orbit_limits())

    def test_default_delivery_hub_nav_passes_orbit_limits_to_planner(self):
        """The orbit-sizing envelope is forwarded to calc_peer_approach_offset
        so the planner can size the navigation-feasible orbit."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits

        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0
        controller.mission.default_delivery_hub = Location(40.001, -74.001, 0.0)

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 0.0),
                offset_distance=0.0,
                orbit_radius=300.0,
            ),
        ) as calc:
            controller.fallback_navigation.setup()

        passed = calc.call_args.kwargs["orbit_limits"]
        self.assertEqual(
            passed,
            OrbitNavigationLimits(
                airspeed_mps=22.0, roll_limit_deg=45.0, min_pitch_deg=-40.0),
        )

    def test_default_delivery_hub_nav_passes_recognition_px_to_planner(self):
        """The per-class operator-ID demand is computed in NavController and
        forwarded as recognition_px (the max-zoom recognition bound for the
        furthest-standoff ORBIT sizing) — peer_offset stays geometry-only."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        from navpy.modules.vision.vision_profiles import (
            get_min_pixels_for_class,
        )

        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0
        controller.mission.default_delivery_hub = Location(40.001, -74.001, 0.0)

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 0.0),
                offset_distance=0.0,
                orbit_radius=300.0,
            ),
        ) as calc:
            controller.fallback_navigation.setup()

        expected_px = get_min_pixels_for_class(controller.vision_profile, 0)
        self.assertEqual(calc.call_args.kwargs["recognition_px"], expected_px)

    def test_default_delivery_hub_orbit_sizes_for_scan_alt_and_gates(self):
        """Default delivery hub ORBIT sizes for the scan altitude (not the
        instantaneous one), records the navigation task-altitude gate, and
        commands the loiter at that altitude."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        # Off-band: instantaneous relative alt 40 must NOT drive sizing.
        vehicle.location = Mock(side_effect=lambda is_relative: Location(
            40.0, -74.0, 40.0, is_absolute=not is_relative))
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0
        controller.mission.default_delivery_hub = Location(40.001, -74.001, 0.0)

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.001, -74.001, 0.0),
                offset_distance=0.0, orbit_radius=300.0),
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.fallback_navigation.setup()

        # Relative POI -> relative navigation task alt 150, not instant 40.
        drone_arg = calc.call_args.args[1]
        self.assertFalse(drone_arg.is_absolute)
        self.assertAlmostEqual(drone_arg.alt, 150.0)
        self.assertEqual(controller.navigation_task.orbit_approach_alt_rel_m, 150.0)
        self.assertEqual(dispatch.call_args.kwargs["loiter_alt_rel"], 150.0)

    def test_default_delivery_hub_offset_preserves_instantaneous_sizing(self):
        """Default delivery hub OFFSET keeps legacy behavior: no scan-altitude
        substitution, no navigation task-altitude gate, loiter at current alt."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        vehicle.location = Mock(side_effect=lambda is_relative: Location(
            40.0, -74.0, 40.0, is_absolute=not is_relative))
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.OFFSET,
        )
        controller.mission.scan_altitude_rel = 150.0
        controller.mission.default_delivery_hub = Location(40.001, -74.001, 0.0)

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.OFFSET,
                approach_location=Location(40.002, -74.002, 0.0),
                offset_distance=120.0, orbit_radius=OFFSET_LOITER_RADIUS_M),
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.fallback_navigation.setup()

        # OFFSET: drone pose passed unchanged (instantaneous 40), no scan
        # substitution; orbit_limits None; loiter_alt_rel None (no gate).
        drone_arg = calc.call_args.args[1]
        self.assertAlmostEqual(drone_arg.alt, 40.0)
        self.assertIsNone(calc.call_args.kwargs["orbit_limits"])
        self.assertIsNone(controller.navigation_task.orbit_approach_alt_rel_m)
        self.assertIsNone(dispatch.call_args.kwargs["loiter_alt_rel"])

    def test_navigation_task_alt_rel_uses_scan_altitude(self):
        # The mission's scan/zone band (not the higher corridor band) is
        # the navigation task altitude the orbit is sized for.
        controller = _create_controller()
        controller.mission.scan_altitude_rel = 150.0
        self.assertEqual(controller.mission.scan_altitude_rel, 150.0)

    def test_navigation_task_alt_rel_none_without_mission_source(self):
        # No mission scan altitude -> None (callers fall back to legacy
        # camera-range sizing, never the climb-time or corridor altitude).
        controller = _create_controller()
        controller.mission.scan_altitude_rel = None
        self.assertIsNone(controller.mission.scan_altitude_rel)

    def test_setup_peer_navigation_sizes_orbit_for_scan_alt(self):
        """Peer-nav that begins mid-climb must size the ORBIT for the
        scan/navigation task altitude (mission waypoint), not the climbing
        altitude, and command the loiter at that altitude."""
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.home_location = Location(40.0, -74.0, 1295.0)  # home MSL
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        # Drone mid-climb: relative alt 40, absolute 1335.
        vehicle.location = Mock(side_effect=lambda is_relative: Location(
            40.0, -74.0, 40.0 if is_relative else 1335.0,
            is_absolute=not is_relative,
        ))
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0  # scan/navigation task alt (rel-to-home)
        task_loc = Mock(lat=40.5, lng=-74.5, alt=1300.0)  # POI MSL
        controller.network.task_actor = Mock()
        controller.network.task_actor.selected_poi.return_value = Mock(
            location=task_loc,
            class_id=0,
        )

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.5, -74.5, 1300.0, is_absolute=True),
                offset_distance=0.0, orbit_radius=305.0,
            ),
        ) as calc, patch.object(controller.vehicle_navigation, "dispatch_approach") as dispatch:
            controller.peer_navigation.setup()

        # Planning pose = home MSL + scan/navigation task alt (1295 + 150) — the altitude
        # the loiter command will hold — NOT the climbing 1335 or rel 40.
        # This matches the loiter command so r_nav_min's height above
        # POI is the pose the drone actually dives from.
        drone_arg = calc.call_args.args[1]
        self.assertAlmostEqual(drone_arg.alt, 1295.0 + 150.0)
        self.assertTrue(drone_arg.is_absolute)
        # Loiter commanded at the scan/navigation task altitude (home-relative).
        self.assertEqual(dispatch.call_args.kwargs["loiter_alt_rel"], 150.0)
        # Approach-altitude gate is stored for _near_peer_poi.
        self.assertEqual(controller.navigation_task.orbit_approach_alt_rel_m, 150.0)

    def test_near_peer_poi_waits_for_navigation_task_altitude(self):
        """ORBIT approach must not activate while the drone is still climbing,
        even once it is horizontally within the orbit radius."""
        from navpy.modules.navigation.approach_strategy import ApproachKind

        vehicle = _create_mock_vehicle()
        # ~153 m horizontally from the POI; climbing.
        rel_alt = {"v": 100.0}
        vehicle.location = Mock(side_effect=lambda is_relative: Location(
            40.0, -74.0018, rel_alt["v"] if is_relative else 200.0,
            is_absolute=not is_relative,
        ))
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.navigation_task.orbit_radius_m = 300.0
        # The navigation-sized orbit recorded its navigation task altitude.
        controller.navigation_task.orbit_approach_alt_rel_m = 150.0
        controller.network.task_actor = Mock()
        controller.network.task_actor.selected_poi.return_value = Mock(
            location=Mock(lat=40.0, lng=-74.0, alt=0.0),
        )

        # Within 300 m but only at 100 m alt (< 150 - ALT_HYST): not yet.
        self.assertFalse(controller.peer_navigation.near_poi())
        # Reached scan/navigation task altitude: begin approach.
        rel_alt["v"] = 150.0
        self.assertTrue(controller.peer_navigation.near_poi())

    def test_plan_orbit_approach_absolute_target_uses_navigation_task_msl(self):
        # Self-detect / peer: an ABSOLUTE (terrain-MSL) POI makes the
        # planning pose ABSOLUTE at home.alt + navigation task alt, so the
        # height-above-POI subtraction stays in one frame regardless of
        # the frame of the drone_loc passed in (here a relative climbing
        # pose, which must NOT leak into sizing).
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()  # home_location alt = 100
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0
        drone_rel = Location(40.0, -74.0, 30.0, is_absolute=False)
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.5, -74.5, 0.0),
                offset_distance=0.0, orbit_radius=300.0),
        ) as calc:
            _, loiter_alt = controller.fallback_navigation.plan_orbit_approach(
                Location(40.5, -74.5, 1296.0, is_absolute=True), 0, drone_rel)
        drone_arg = calc.call_args.args[1]
        self.assertTrue(drone_arg.is_absolute)
        self.assertAlmostEqual(drone_arg.alt, 100.0 + 150.0)  # home MSL + approach altitude
        self.assertEqual(loiter_alt, 150.0)
        self.assertIsNotNone(calc.call_args.kwargs["orbit_limits"])

    def test_plan_orbit_approach_relative_poi_uses_navigation_task_rel(self):
        # Default delivery hub: a RELATIVE POI (alt 0) makes the planning pose
        # RELATIVE at the navigation task altitude (no home needed). Frame
        # follows the POI, not the drone_loc.
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.lim_roll = 45.0
        vehicle.min_pitch = -40.0
        vehicle.air_speed = 22.0
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = 150.0
        drone_rel = Location(40.0, -74.0, 30.0, is_absolute=False)
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.5, -74.5, 0.0),
                offset_distance=0.0, orbit_radius=300.0),
        ) as calc:
            _, loiter_alt = controller.fallback_navigation.plan_orbit_approach(
                Location(40.5, -74.5, 0.0, is_absolute=False), 0, drone_rel)
        drone_arg = calc.call_args.args[1]
        self.assertFalse(drone_arg.is_absolute)
        self.assertAlmostEqual(drone_arg.alt, 150.0)
        self.assertEqual(loiter_alt, 150.0)
        self.assertIsNotNone(calc.call_args.kwargs["orbit_limits"])

    def test_plan_orbit_approach_legacy_when_no_mission_alt(self):
        # No mission scan altitude -> legacy camera-range sizing: pose
        # unchanged, orbit_limits None, loiter altitude None.
        from navpy.modules.navigation.approach_strategy import (
            ApproachKind, ApproachPlan,
        )
        vehicle = _create_mock_vehicle()
        vehicle.mission_items_next = None
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.mission.scan_altitude_rel = None
        drone = Location(40.0, -74.0, 30.0, is_absolute=False)
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=ApproachPlan(
                kind=ApproachKind.ORBIT,
                approach_location=Location(40.5, -74.5, 0.0),
                offset_distance=0.0, orbit_radius=80.0),
        ) as calc:
            _, loiter_alt = controller.fallback_navigation.plan_orbit_approach(
                Location(40.5, -74.5, 1296.0, is_absolute=True), 0, drone)
        self.assertIs(calc.call_args.args[1], drone)
        self.assertIsNone(calc.call_args.kwargs["orbit_limits"])
        self.assertIsNone(loiter_alt)

    def test_near_peer_poi_no_altitude_gate_for_legacy_sizing(self):
        # When the orbit was legacy-sized (no navigation task altitude recorded),
        # the altitude gate is disabled — horizontal distance alone permits approach.
        from navpy.modules.navigation.approach_strategy import ApproachKind

        vehicle = _create_mock_vehicle()
        vehicle.location = Mock(side_effect=lambda is_relative: Location(
            40.0, -74.0018, 30.0 if is_relative else 200.0,
            is_absolute=not is_relative,
        ))
        controller = _create_controller(
            vehicle=vehicle, approach_kind=ApproachKind.ORBIT,
        )
        controller.navigation_task.orbit_radius_m = 300.0
        controller.navigation_task.orbit_approach_alt_rel_m = None  # legacy sizing
        controller.network.task_actor = Mock()
        controller.network.task_actor.selected_poi.return_value = Mock(
            location=Mock(lat=40.0, lng=-74.0, alt=0.0),
        )
        self.assertTrue(controller.peer_navigation.near_poi())


# =============================================================================
# Lifecycle Tests
# =============================================================================

class TestNavControllerLifecycle(unittest.TestCase):
    """Tests for NavController start/stop lifecycle."""

    def test_raise_if_failed_delegates_to_application(self):
        application = Mock()
        with patch(
            "navpy.modules.nav.nav_controller.create_nav_application",
            return_value=application,
        ):
            controller = NavController(
                Mock(),
                Mock(),
                Mock(),
                Mock(),
                Mock(),
            )

        controller.raise_if_failed()

        application.raise_if_failed.assert_called_once_with()

    def test_shutdown_source_failure_skips_dependent_state_teardown(self):
        from navpy.modules.nav.nav_application import NavShutdown

        order = []
        errors = []

        def failing_step(name):
            error = RuntimeError(f"{name} failed")

            def run(*_args, **_kwargs):
                order.append(name)
                errors.append(error)
                raise error

            return run

        navigation_task = Mock()
        navigation_task.clear.side_effect = failing_step("navigation_task")
        network = Mock()
        network.stop.side_effect = failing_step("listener")
        logger = Mock()
        logger.info.side_effect = failing_step("status")
        shutdown = NavShutdown(
            failing_step("source"),
            failing_step("navigation"),
            navigation_task,
            network,
            logger,
        )

        with self.assertRaises(ExceptionGroup) as raised:
            shutdown.run(lambda: order.append("loop"))

        self.assertEqual(
            order,
            [
                "loop",
                "source",
                "listener",
                "status",
            ],
        )
        self.assertEqual(list(raised.exception.exceptions), errors)

    def test_shutdown_does_not_teardown_live_dependencies_when_loop_wont_stop(self):
        from navpy.modules.nav.nav_application import NavShutdown

        navigation_reset = Mock()
        navigation_task = Mock()
        network = Mock()
        logger = Mock()
        close_source = Mock()
        shutdown = NavShutdown(
            close_source,
            navigation_reset,
            navigation_task,
            network,
            logger,
        )
        quiescence_error = RuntimeError("navigation loop failed to terminate")

        with self.assertRaisesRegex(RuntimeError, "failed to terminate"):
            shutdown.run(Mock(side_effect=quiescence_error))

        close_source.assert_not_called()
        navigation_reset.assert_not_called()
        navigation_task.clear.assert_not_called()
        network.stop.assert_not_called()
        logger.info.assert_not_called()

    def test_start_creates_thread(self):
        """start creates and starts navigation thread."""
        controller = _create_controller()

        controller.application.start(loop_rate_hz=10.0)

        self.assertTrue(controller.loop.thread.is_alive())

        # Cleanup
        controller.application.stop()

    def test_start_sets_loop_rate(self):
        """start sets navigation loop rate."""
        controller = _create_controller()

        controller.application.start(loop_rate_hz=20.0)

        self.assertAlmostEqual(controller.loop.period_s, 1.0 / 20.0)

        # Cleanup
        controller.application.stop()

    def test_start_zero_loop_rate_uses_default(self):
        """start with zero loop_rate uses default."""
        controller = _create_controller()
        default_rate = controller.loop.period_s

        controller.application.start(loop_rate_hz=0.0)

        self.assertEqual(controller.loop.period_s, default_rate)

        # Cleanup
        controller.application.stop()

    def test_stop_stops_thread(self):
        """stop stops the navigation thread."""
        controller = _create_controller()
        controller.application.start(loop_rate_hz=10.0)

        controller.application.stop()

        self.assertFalse(controller.loop.thread.is_alive())

    def test_stop_resets_navigation(self):
        """stop calls navigation.reset()."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.application.stop()

        navigation.reset.assert_called()

    def test_stop_resets_confirmation_manager(self):
        """stop resets POI manager."""
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)

        controller.application.stop()

        self.assertIsNone(controller.confirmation_manager.active_poi)


# =============================================================================
# Network Integration Tests
# =============================================================================

class TestNavControllerNetwork(unittest.TestCase):
    """Tests for NavController network integration."""

    def test_set_network_none(self):
        """set_network handles None network."""
        controller = _create_controller()

        controller.application.set_network(None)

        self.assertIsNone(controller.network.task_actor)

    def test_set_network_creates_task_actor(self):
        """set_network creates TaskActor."""
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()

        controller.application.set_network(network)

        self.assertIsNotNone(controller.network.task_actor)

    def test_set_network_accepts_falsey_valid_transport(self):
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = MagicMock()
        network.__bool__.return_value = False

        controller.application.set_network(network)

        self.assertIsNotNone(controller.network.task_actor)
        self.assertIs(controller.confirmation_manager._network_slot.get(), network)
        self.assertEqual(network.set_listener.call_count, 3)

    def test_set_network_registers_listeners(self):
        """set_network registers listeners on network."""
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()

        controller.application.set_network(network)

        # Should register task_actor, confirmation_manager, and (CONF-03, D-18)
        # the nav-owned ConfirmOverrideListener for SWARM_REQUEST(FORCE_CONFIRM).
        self.assertEqual(network.set_listener.call_count, 3)

    def test_set_network_initializes_task_actor(self):
        """set_network initializes task_actor properly."""
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()

        controller.application.set_network(network)

        # TaskActor should be created and have required methods
        self.assertIsNotNone(controller.network.task_actor)
        self.assertTrue(hasattr(controller.network.task_actor, 'checkin'))

    def test_set_network_sets_confirmation_manager_network(self):
        """set_network sets network on POI manager."""
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()

        controller.application.set_network(network)

        self.assertIs(controller.confirmation_manager._network_slot.get(), network)

    def test_replacing_network_tears_down_and_detaches_only_old_session(self):
        controller = _create_controller()
        first = Mock()
        second = Mock()
        controller.application.set_network(first)
        old_actor = controller.network.task_actor
        old_worker = controller.network.peer_dispatch
        old_actor.shutdown = Mock()
        old_worker.stop = Mock(wraps=old_worker.stop)

        controller.application.set_network(second)

        old_worker.stop.assert_called_once_with()
        old_actor.shutdown.assert_called_once_with()
        detached = [call.args[0] for call in first.remove_listener.call_args_list]
        self.assertIn(old_actor, detached)
        self.assertIn(controller.confirmation_manager, detached)
        self.assertIs(controller.confirmation_manager._network_slot.get(), second)
        active_actor = controller.network.task_actor
        active_actor.shutdown = Mock()

        controller.application.stop()

        old_actor.shutdown.assert_called_once_with()
        active_actor.shutdown.assert_called_once_with()

    def test_setting_network_none_disconnects_active_session(self):
        controller = _create_controller()
        network = Mock()
        controller.application.set_network(network)
        actor = controller.network.task_actor
        worker = controller.network.peer_dispatch
        # worker.stop is replaced with a Mock below, so the real daemon is never
        # joined by the disconnect path; join it via the class at cleanup.
        self.addCleanup(lambda w=worker: type(w).stop(w))
        order = []
        original_set_network = controller.confirmation_manager.set_network
        actor.shutdown = Mock(side_effect=lambda: order.append("shutdown"))
        worker.stop = Mock(side_effect=lambda: order.append("stop"))
        network.remove_listener.side_effect = lambda _listener: order.append("detach")
        controller.confirmation_manager.set_network = Mock(
            side_effect=lambda value: (
                order.append("poi-none"),
                original_set_network(value),
            )[-1]
        )

        controller.application.set_network(None)

        worker.stop.assert_called_once_with()
        actor.shutdown.assert_called_once_with()
        self.assertIsNone(controller.network.task_actor)
        self.assertIsNone(controller.network.peer_dispatch)
        self.assertIsNone(controller.confirmation_manager._network_slot.get())
        self.assertEqual(network.remove_listener.call_count, 3)
        self.assertEqual(
            order,
            ["stop", "detach", "detach", "detach", "poi-none", "shutdown"],
        )

    def test_disconnect_attempts_every_cleanup_and_retains_failed_session(self):
        controller = _create_controller()
        network = Mock()
        controller.application.set_network(network)
        actor = controller.network.task_actor
        worker = controller.network.peer_dispatch
        override_listener = controller.network._override_listener
        # worker.stop is replaced with a Mock below and the failed disconnect
        # retains the worker, so join the real daemon via the class at cleanup.
        self.addCleanup(lambda w=worker: type(w).stop(w))
        order = []
        worker.stop = Mock(side_effect=lambda: order.append("stop"))

        def detach(listener):
            if listener is actor:
                order.append("detach-actor")
                raise OSError("actor detach failed")
            if listener is controller.confirmation_manager:
                order.append("detach-poi-manager")
                raise TypeError("listener contract failed")
            self.assertIs(listener, override_listener)
            order.append("detach-override")

        network.remove_listener.side_effect = detach
        original_set_network = controller.confirmation_manager.set_network
        controller.confirmation_manager.set_network = Mock(
            side_effect=lambda value: (
                order.append("poi-none"),
                original_set_network(value),
            )[-1],
        )
        actor.shutdown = Mock(
            side_effect=lambda: (
                order.append("shutdown"),
                (_ for _ in ()).throw(RuntimeError("checkout failed")),
            )[-1],
        )

        with self.assertRaises(ExceptionGroup) as raised:
            controller.application.set_network(None)

        self.assertEqual(
            order,
            [
                "stop",
                "detach-actor",
                "detach-poi-manager",
                "detach-override",
                "poi-none",
                "shutdown",
            ],
        )
        self.assertEqual(len(raised.exception.exceptions), 3)
        self.assertIs(controller.network.task_actor, actor)
        self.assertIs(controller.network.peer_dispatch, worker)
        self.assertIs(controller.network._network, network)
        self.assertIs(controller.network._override_listener, override_listener)
        self.assertIsNone(controller.confirmation_manager._network_slot.get())


# =============================================================================
# Reset Tests
# =============================================================================

class TestNavControllerReset(unittest.TestCase):
    """Tests for NavController reset behavior (via _enter_reset / _on_state_change)."""

    def test_enter_reset_clears_active_poi(self):
        """_enter_reset clears active POI."""
        controller = _create_controller()
        controller.confirmation_manager.set_active_poi(Mock())

        controller.transitions.enter_reset()

        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_reset_to_detect_sets_auto_mode(self):
        """RESET→DETECT transition sets vehicle to AUTO mode."""
        vehicle = _create_mock_vehicle()
        controller = _create_controller(vehicle=vehicle)

        controller.transitions.on_change(NavState.RESET, NavState.DETECT)

        vehicle.set_mode.assert_called_with(FlightMode.AUTO)

    def test_reset_to_detect_restarts_mission(self):
        """RESET→DETECT transition restarts mission from waypoint 1."""
        vehicle = _create_mock_vehicle()
        controller = _create_controller(vehicle=vehicle)

        controller.transitions.on_change(NavState.RESET, NavState.DETECT)

        vehicle.restart_mission.assert_called_with(1)

    def test_enter_reset_refreshes_detector(self):
        """_enter_reset refreshes detector (via _clear_state)."""
        detector = _create_mock_detector()
        controller = _create_controller(detector=detector)

        controller.transitions.enter_reset()

        detector.refresh.assert_called()

    def test_enter_reset_refreshes_args(self):
        """_enter_reset refreshes args (via _clear_state)."""
        args = _create_mock_args()
        controller = _create_controller(args=args)

        controller.transitions.enter_reset()

        args.refresh.assert_called()

    def test_enter_reset_resets_pass_detector(self):
        """_enter_reset resets pass detection state."""
        controller = _create_controller()
        controller.pass_tracker.previous_distance_m = 100.0
        controller.pass_tracker.approach_started = True
        controller.pass_tracker.increase_count = 5

        controller.transitions.enter_reset()

        self.assertIsNone(controller.pass_tracker.previous_distance_m)
        self.assertFalse(controller.pass_tracker.approach_started)
        self.assertEqual(controller.pass_tracker.increase_count, 0)

    def test_enter_reset_resets_peer_navigation(self):
        """_enter_reset resets peer_navigation flag."""
        controller = _create_controller()
        controller.navigation_task.peer_navigation = True

        controller.transitions.enter_reset()

        self.assertFalse(controller.navigation_task.peer_navigation)

    def test_reset_transient_to_detect_when_alt_ok(self):
        """RESET is transient — goes to DETECT when alt >= min_alt."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=100.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RESET

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_reset_transient_to_recovery_when_alt_low(self):
        """RESET is transient — goes to RECOVERY when alt < min_alt."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=30.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RESET

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)

    def test_vision_nav_reset_returns_to_detect_without_reading_altitude(self):
        """AUTO owns post-pass recovery; pure-vision state cannot read GPS altitude."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        vehicle.location = Mock(
            side_effect=AssertionError("altitude entered pure-vision reset path")
        )
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.phase.current = NavState.RESET

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        vehicle.location.assert_not_called()

    def test_enter_reset_resets_task_actor(self):
        """_enter_reset resets task actor if present (via _clear_state)."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        controller.transitions.enter_reset()

        controller.network.task_actor.reset.assert_called()

    def test_resource_reset_attempts_every_safe_step_and_state_after_each_failure(self):
        from navpy.modules.nav.navigation_task_reset import (
            NavigationTaskResetTransaction,
            NavigationTaskResourcePorts,
            NavigationTaskResourceReset,
        )

        resource_names = [
            "tracking",
            "geo",
            "loiter",
            "speedup",
            "peer",
            "poi",
            "detector",
            "actor",
            "args",
            "mission",
        ]
        for failing_name in resource_names:
            with self.subTest(failing_name=failing_name):
                order = []
                failure = RuntimeError(f"{failing_name} failed")

                def action(name, result=None):
                    def run(*_args, **_kwargs):
                        order.append(name)
                        if name == failing_name:
                            raise failure
                        return result

                    return Mock(side_effect=run)

                tracking = Mock(stop_tracking=action("tracking"))
                geo = Mock(stop_geo_tracking=action("geo"))
                loiter = Mock(restore=action("loiter"))
                speedup = Mock(restore=action("speedup", True))
                confirmation_manager = Mock(reset=action("poi"))
                detector = Mock(refresh=action("detector"))
                args = Mock(refresh=action("args"))
                ports = NavigationTaskResourcePorts(
                    reset_peer_dispatch=action("peer"),
                    reset_task_actor=action("actor"),
                    refresh_mission=action("mission"),
                )
                resources = NavigationTaskResourceReset(
                    ports,
                    tracking,
                    geo,
                    detector,
                    confirmation_manager,
                    loiter,
                    speedup,
                    args,
                )
                state = Mock()
                state.clear.side_effect = lambda: order.append("state")

                with self.assertRaises(ExceptionGroup) as raised:
                    NavigationTaskResetTransaction(
                        state,
                        resources,
                        action("source"),
                    ).clear()

                expected_resources = resource_names
                if failing_name == "peer":
                    expected_resources = [
                        name for name in resource_names
                        if name not in {"poi", "actor"}
                    ]
                self.assertEqual(order, ["source", *expected_resources, "state"])
                resource_group = raised.exception.exceptions[0]
                self.assertIsInstance(resource_group, ExceptionGroup)
                self.assertEqual(resource_group.exceptions, (failure,))

    def test_resource_reset_source_fence_failure_skips_resources_and_state(self):
        from navpy.modules.nav.navigation_task_reset import (
            NavigationTaskResetTransaction,
        )

        source_error = RuntimeError("source failed")
        resources = Mock()
        state = Mock()
        reset = NavigationTaskResetTransaction(
            state,
            resources,
            Mock(side_effect=source_error),
        )

        with self.assertRaises(ExceptionGroup) as raised:
            reset.clear()

        self.assertEqual(raised.exception.exceptions, (source_error,))
        resources.clear.assert_not_called()
        state.clear.assert_not_called()

    def test_resource_reset_reports_pending_speed_restore_and_keeps_going(self):
        controller = _create_controller()
        controller.resource_reset._speedup.restore = Mock(return_value=False)
        controller.navigation_task.peer_navigation = True

        with self.assertRaises(ExceptionGroup) as raised:
            controller.reset.clear()

        self.assertIn(
            "SIM_SPEEDUP rollback remains unverified",
            repr(raised.exception),
        )
        controller.detector.refresh.assert_called_once()
        controller.args.refresh.assert_called_once()
        self.assertFalse(controller.navigation_task.peer_navigation)

    def test_state_reset_attempts_every_owned_reset_after_each_failure(self):
        reset_names = ["mission", "detections", "retry", "pass"]
        for failing_name in reset_names:
            with self.subTest(failing_name=failing_name):
                controller = _create_controller()
                order = []

                def action(name):
                    def run():
                        order.append(name)
                        if name == failing_name:
                            raise RuntimeError(f"{name} failed")

                    return Mock(side_effect=run)

                controller.state_reset._mission.clear_navigation_task = action("mission")
                controller.state_reset._detections.clear = action("detections")
                controller.state_reset._retry.full_reset = action("retry")
                controller.state_reset._pass_tracker.reset = action("pass")

                with self.assertRaises(ExceptionGroup) as raised:
                    controller.state_reset.clear()

                self.assertEqual(order, reset_names)
                self.assertEqual(
                    raised.exception.exceptions[0].args,
                    (f"{failing_name} failed",),
                )


# =============================================================================
# GUIDED Mode Tests
# =============================================================================

class TestNavControllerGuidedMode(unittest.TestCase):
    """Tests for GUIDED mode handling."""

    def test_set_guided_mode_changes_mode(self):
        """_set_guided_mode changes to GUIDED mode."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        controller = _create_controller(vehicle=vehicle)

        result = controller.vehicle_navigation.request_guided()

        self.assertTrue(result)
        vehicle.set_mode.assert_called_with(FlightMode.GUIDED)

    def test_set_guided_mode_already_guided(self):
        """_set_guided_mode returns False if already GUIDED."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        controller = _create_controller(vehicle=vehicle)

        result = controller.vehicle_navigation.request_guided()

        self.assertFalse(result)
        vehicle.set_mode.assert_not_called()


# =============================================================================
# Logging Tests
# =============================================================================

class TestNavControllerLogging(unittest.TestCase):
    """Tests for NavController logging behavior."""

    def test_log_ignore_reason_logs_on_change(self):
        """_log_ignore_reason logs when state changes."""
        logger = Mock()
        controller = _create_controller(logger=logger)

        controller.status.ignore(1, "Test message")

        # Should log (ignoring init log)
        self.assertTrue(any("Test message" in str(call) for call in logger.info.call_args_list))

    def test_log_ignore_reason_does_not_repeat(self):
        """_log_ignore_reason does not repeat same state."""
        logger = Mock()
        controller = _create_controller(logger=logger)

        controller.status.ignore(1, "First")
        initial_count = logger.info.call_count

        controller.status.ignore(1, "Second")

        # Should not log again
        self.assertEqual(logger.info.call_count, initial_count)

    def test_log_ignore_reason_logs_new_state(self):
        """_log_ignore_reason logs when state changes to new value."""
        logger = Mock()
        controller = _create_controller(logger=logger)

        controller.status.ignore(1, "First")
        initial_count = logger.info.call_count

        controller.status.ignore(2, "Second")

        # Should log new state
        self.assertEqual(logger.info.call_count, initial_count + 1)


# =============================================================================
# Integration Tests - Full Flow
# =============================================================================

class TestNavControllerIntegration(unittest.TestCase):
    """Integration tests for full navigation flows."""

    def test_full_flow_detect_to_confirm_to_nav(self):
        """Full flow: DETECT -> CONFIRM -> NAV."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        poi = _create_detected_poi(
            obj_id=1, detection_frame=frame, bbox=(320, 240, 100, 80)
        )
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        args = _create_mock_args(auto_confirm=True)
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation, args=args
        )

        # Phase 1: DETECT - should find POI
        controller.sensor.sense()
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)

        controller.detect_action.act()
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

        # Phase 2: CONFIRM - should request confirmation
        controller.sensor.sense()
        controller.decision.decide()
        # With no status yet, should be CONFIRM
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

        controller.confirmation_action.act()
        # With auto_confirm, should be confirmed now. The mock vehicle
        # mode is still AUTO here because set_mode() is async over
        # MAVLink and our mock does not auto-flip; the abort guard
        # latches off observed_guided_post_dispatch, so an AUTO read on
        # the next decide tick must NOT abort the navigation_task.

        # Phase 3: NAV - should navigate
        controller.sensor.sense()
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.NAV)

        vehicle.get_mode = FlightMode.GUIDED
        controller.final_approach_nav.act_nav()
        navigation.nav.assert_called()

    def test_full_flow_reject_and_restart_navigation_task(self):
        """Full flow: POI rejected, new POI selected."""
        poi1 = _create_detected_poi(obj_id=1)
        poi2 = _create_detected_poi(obj_id=2)
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector(detections=[poi1, poi2])
        controller = _create_controller(vehicle=vehicle, detector=detector)

        # Select first POI
        controller.sensor.sense()
        controller.decision.decide()
        controller.detect_action.act()
        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 1)

        # Reject first POI
        controller.confirmation_manager.update_status(poi1, ConfirmationStatus.REJECTED)

        # Should clear and be ready for new POI
        controller.sensor.sense()
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)

        # Simulate poi1 left field of view, only poi2 detected now
        detector.get_detect_data.return_value = DetectResponse([poi2])
        controller.sensor.sense()

        # Should select second POI
        controller.detect_action.act()
        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 2)

    def test_full_flow_mode_switch_aborts(self):
        """Full flow: mode switch during NAV aborts navigation_task."""
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV
        controller.navigation_task.nav_mode_observed = True

        # Mode switches to AUTO
        vehicle.get_mode = FlightMode.AUTO

        controller.decision.decide()

        # Should reset and go to DETECT
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_full_flow_recovery_on_low_alt(self):
        """Full flow: RECOVERY triggered by low altitude."""
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=200.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV

        # Altitude drops
        vehicle.location.return_value = Location(40.0, -74.0, 30.0)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)


# =============================================================================
# Edge Case Tests
# =============================================================================

class TestNavControllerEdgeCases(unittest.TestCase):
    """Tests for edge cases and error conditions."""

    def test_empty_detections_after_having_poi(self):
        """Handles empty detections while having active POI."""
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5)
        detector = _create_mock_detector(detections=[])
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        # Should wait, not crash
        navigation.nav.assert_not_called()
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_poi_id_zero(self):
        """Handles POI with ID 0."""
        poi = _create_detected_poi(obj_id=0)
        detector = _create_mock_detector(detections=[poi])
        controller = _create_controller(detector=detector)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 0)

    def test_multiple_self_pois_first_selected(self):
        """First self POI is selected when multiple available."""
        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)
        detector = _create_mock_detector(detections=[t1, t2, t3])
        controller = _create_controller(detector=detector)

        controller.sensor.sense()
        controller.phase.current = NavState.DETECT
        controller.detect_action.act()

        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 1)

    def test_detect_phase_does_not_refresh_args(self):
        """In DETECT phase, args are not refreshed (only on reset)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        args = _create_mock_args(min_wp=3)
        controller = _create_controller(vehicle=vehicle, args=args)

        # Go to DETECT state
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)

        # Args refresh should not be called in normal DETECT
        args.refresh.assert_not_called()

    def test_navigation_failure_sets_flag_and_decide_resets(self):
        """Navigation failure sets flag; next _decide clears POI and goes to RESET."""
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        navigation.nav.return_value = False
        controller = _create_controller(vehicle=vehicle, detector=detector, navigation=navigation)

        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV

        controller.sensor.sense()
        controller.final_approach_nav.act_nav()

        # Flag set, not state
        self.assertTrue(controller.navigation_failures.failed)

        # Next decide consumes the flag
        controller.decision.decide()
        self.assertFalse(controller.navigation_failures.failed)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertEqual(controller.phase.current, NavState.RESET)

    def _basin_gap_failure_controller(self, navigation, logger):
        poi = _create_detected_poi(obj_id=1)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        detector = _create_mock_detector(detections=[poi])
        navigation.nav.return_value = False
        # No coordinate distance: _passed_poi_wp must bail out without
        # touching the pass-detector state the tests arrange below.
        navigation.legacy_pois.locked_distance.return_value = None
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation,
            logger=logger,
        )
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV
        controller.pass_tracker.close_observed = True
        controller.sensor.sense()
        controller.final_approach_nav.act_nav()
        self.assertTrue(controller.navigation_failures.failed)
        return controller

    def test_navigation_gap_failure_inside_basin_with_behind_bearing_is_a_pass(self):
        """Basin + last measured bearing behind the wing line = pass.

        At 10x the 1.25-sim-s reacquire window (~125 wall ms) expires before
        a receding GPS pair can trip the distance counter, so the fixed
        forward camera losing the POI behind the aircraft must classify
        as the pass it is (run 0140: basin entered ~48 m, POI behind_cam
        at 3-7 m, all three vehicles reset without the pass marker).
        """
        navigation = _create_mock_navigation()
        navigation.final_approach.last_measured_lateral_bearing_deg.return_value = 167.0
        logger = Mock()
        controller = self._basin_gap_failure_controller(navigation, logger)

        controller.decision.decide()

        self.assertFalse(controller.navigation_failures.failed)
        self.assertEqual(controller.phase.current, NavState.RESET)
        # Pass path: the active POI is retained for RESET bookkeeping,
        # exactly like the coordinate pass branch (the failure path clears it).
        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        logged = [
            call.args[0] for call in logger.info.call_args_list
            if call.args and isinstance(call.args[0], str)
        ]
        self.assertIn("RESET: PASSED POI", logged)

    def test_navigation_gap_failure_inside_basin_with_receding_gps_is_a_pass(self):
        """Basin + at least one receding GPS pair = pass even without bearing."""
        navigation = _create_mock_navigation()
        navigation.final_approach.last_measured_lateral_bearing_deg.return_value = None
        logger = Mock()
        controller = self._basin_gap_failure_controller(navigation, logger)
        controller.pass_tracker.increase_count = 1

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RESET)
        self.assertIsNotNone(controller.confirmation_manager.active_poi)

    def test_inbound_dropout_inside_basin_is_not_a_pass(self):
        """A dropout while still approaching (POI ahead, no receding GPS)
        must stay a failure reset — review finding F2: it must not end a
        one-shot mission as a successful pass."""
        navigation = _create_mock_navigation()
        navigation.final_approach.last_measured_lateral_bearing_deg.return_value = 4.0
        logger = Mock()
        controller = self._basin_gap_failure_controller(navigation, logger)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RESET)
        # Failure path: active POI cleared, no pass marker.
        self.assertIsNone(controller.confirmation_manager.active_poi)
        logged = [
            call.args[0] for call in logger.info.call_args_list
            if call.args and isinstance(call.args[0], str)
        ]
        self.assertNotIn("RESET: PASSED POI", logged)


# =============================================================================
# One-Shot Mode Tests
# =============================================================================

class TestNavControllerOneShot(unittest.TestCase):
    """Tests for one-shot mode behavior via _exit_nav."""

    def test_exit_nav_oneshot_disarms_vehicle(self):
        """In one-shot mode on SITL, _exit_nav disarms after navigation became active."""
        vehicle = _create_mock_vehicle()  # is_simulated_autopilot() -> True
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.navigation_task.final_approach_navigation_active = True  # navigation actually ran
        controller.navigation_task.final_approach_nav_completed = True

        controller.nav_transition.exit()

        vehicle.disarm.assert_called_once()

    def test_exit_nav_oneshot_visual_loss_is_not_successful_completion(self):
        """Issued commands alone must not disarm/latch a failed one-shot pass."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RESET
        controller.navigation_task.final_approach_navigation_active = True
        controller.navigation_task.final_approach_nav_completed = False

        controller.nav_transition.exit()

        vehicle.disarm.assert_not_called()
        self.assertFalse(controller.phase.oneshot_completed)
        self.assertEqual(controller.phase.current, NavState.RESET)

    def test_exit_nav_oneshot_no_disarm_on_real_autopilot(self):
        """SAFETY: on a REAL autopilot (no SIM_SPEEDUP), one-shot must NOT
        force-disarm the airframe in flight — even though the camera may be
        simulated. It still stops navigation (ONHOLD) and logs a warning."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        vehicle.is_simulated_autopilot = Mock(return_value=False)  # real HW
        args = _create_mock_args(is_oneshot=True)
        logger = Mock()
        controller = _create_controller(
            vehicle=vehicle, args=args, logger=logger)
        controller.navigation_task.final_approach_navigation_active = True  # navigation actually ran
        controller.navigation_task.final_approach_nav_completed = True

        outcome = controller.nav_transition.exit()

        vehicle.disarm.assert_not_called()
        self.assertEqual(outcome.redirect, NavState.ONHOLD)
        self.assertTrue(outcome.oneshot_completed)
        self.assertFalse(controller.phase.oneshot_completed)
        warned = any(
            "disarm SUPPRESSED" in str(c.args[0])
            for c in logger.warning.call_args_list if c.args
        )
        self.assertTrue(warned)

    def test_oneshot_latch_keeps_onhold_on_real_after_start_navigation_task(self):
        """After a one-shot navigation task on a real autopilot (still armed, disarm
        suppressed), the durable latch keeps NavPy ONHOLD on later ticks — it
        must NOT select a fresh POI."""
        vehicle = _create_mock_vehicle(
            mode=FlightMode.AUTO, next_wp=5, is_armed=True)
        vehicle.is_simulated_autopilot = Mock(return_value=False)
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.oneshot_completed = True  # a one-shot just completed
        # A fresh confirmed POI is present — it must still be ignored.
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_oneshot_latch_still_allows_altitude_recovery(self):
        """SAFETY: the one-shot latch must NOT block low-altitude recovery — a
        real one-shot that dove low and stayed armed must still climb, not sit
        ONHOLD at low altitude."""
        vehicle = _create_mock_vehicle(
            mode=FlightMode.AUTO, next_wp=5, alt=20.0, is_armed=True)  # below min_alt (50)
        vehicle.is_simulated_autopilot = Mock(return_value=False)
        args = _create_mock_args(is_oneshot=True, min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.oneshot_completed = True

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)  # climbs, not ONHOLD

    def test_oneshot_latch_honors_recovery_hysteresis(self):
        """SAFETY: a latched one-shot in RECOVERY must keep climbing to the full
        min_alt+ALT_HYST hysteresis threshold — the latch must not stop the
        climb early in the [min_alt, min_alt+ALT_HYST) band."""
        vehicle = _create_mock_vehicle(
            mode=FlightMode.AUTO, next_wp=5, alt=51.0, is_armed=True)  # in [50,53)
        vehicle.is_simulated_autopilot = Mock(return_value=False)
        args = _create_mock_args(is_oneshot=True, min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.oneshot_completed = True
        controller.phase.current = NavState.RECOVERY  # still recovering

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)  # keeps climbing

    def test_oneshot_latch_holds_onhold_after_recovery(self):
        """Once back at safe altitude, the latched one-shot holds ONHOLD and
        does NOT re-enter DETECT."""
        vehicle = _create_mock_vehicle(
            mode=FlightMode.AUTO, next_wp=5, alt=200.0, is_armed=True)
        vehicle.is_simulated_autopilot = Mock(return_value=False)
        args = _create_mock_args(is_oneshot=True, min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.oneshot_completed = True
        controller.phase.current = NavState.RECOVERY  # was recovering, now high enough

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_oneshot_latch_clears_on_disarm(self):
        """Disarm is the operator's explicit reset — it clears the one-shot
        latch so a fresh arm cycle can start navigation again."""
        vehicle = _create_mock_vehicle(is_armed=False)
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.oneshot_completed = True

        controller.decision.decide()

        self.assertFalse(controller.phase.oneshot_completed)
        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_exit_nav_non_oneshot_never_disarms(self):
        """Non one-shot never disarms, on SITL or real."""
        for sim in (True, False):
            vehicle = _create_mock_vehicle()
            vehicle.disarm = Mock()
            vehicle.is_simulated_autopilot = Mock(return_value=sim)
            args = _create_mock_args(is_oneshot=False)
            controller = _create_controller(vehicle=vehicle, args=args)

            controller.nav_transition.exit()

            vehicle.disarm.assert_not_called()

    def test_exit_nav_oneshot_goes_to_onhold(self):
        """In one-shot mode, _exit_nav transitions to ONHOLD after navigation was active."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.NAV
        controller.navigation_task.final_approach_navigation_active = True
        controller.navigation_task.final_approach_nav_completed = True

        outcome = controller.nav_transition.exit()

        self.assertEqual(outcome.redirect, NavState.ONHOLD)
        self.assertTrue(outcome.oneshot_completed)
        self.assertEqual(controller.phase.current, NavState.NAV)

    def test_dispatcher_oneshot_redirect_skips_requested_reset_entry(self):
        navigation = _create_mock_navigation()
        controller = _create_controller(
            navigation=navigation,
            args=_create_mock_args(is_oneshot=True),
        )
        controller.network.task_actor = Mock()
        controller.state_reset.clear = Mock(
            wraps=controller.state_reset.clear,
        )
        controller.resource_reset.clear = Mock(
            wraps=controller.resource_reset.clear,
        )
        controller.phase.previous = NavState.NAV
        controller.phase.current = NavState.RESET
        controller.navigation_task.final_approach_navigation_active = True
        controller.navigation_task.final_approach_nav_completed = True

        controller.actions.act()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)
        self.assertEqual(controller.phase.previous, NavState.ONHOLD)
        self.assertTrue(controller.phase.oneshot_completed)
        controller.state_reset.clear.assert_called_once_with()
        controller.resource_reset.clear.assert_called_once_with()
        controller.network.task_actor.start.assert_not_called()
        navigation.pause_final_approach.assert_called_once_with()

    def test_oneshot_redirect_is_not_committed_when_nav_exit_fails(self):
        vehicle = _create_mock_vehicle()
        vehicle.set_parameter.side_effect = [False, False, False, True]
        controller = _create_controller(
            vehicle=vehicle,
            args=_create_mock_args(
                nav_sim_speedup=10.0,
                is_oneshot=True,
            ),
        )
        self.assertFalse(controller.speedup.apply())
        controller.phase.previous = NavState.NAV
        controller.phase.current = NavState.RESET
        controller.navigation_task.final_approach_navigation_active = True
        controller.navigation_task.final_approach_nav_completed = True

        with self.assertRaises(ExceptionGroup):
            controller.actions.act()

        self.assertEqual(controller.phase.previous, NavState.NAV)
        self.assertEqual(controller.phase.current, NavState.RESET)
        self.assertFalse(controller.phase.oneshot_completed)

        controller.actions.act()

        self.assertEqual(controller.phase.previous, NavState.ONHOLD)
        self.assertEqual(controller.phase.current, NavState.ONHOLD)
        self.assertTrue(controller.phase.oneshot_completed)

    def test_exit_nav_oneshot_no_disarm_when_navigation_task_never_started(self):
        """A GUIDED-never-accepted timeout exits NAV without activating navigation; the
        one-shot disarm must NOT fire (re-acquire, don't drop the vehicle)."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.DETECT
        controller.navigation_task.final_approach_navigation_active = False  # never reached GUIDED

        controller.nav_transition.exit()

        vehicle.disarm.assert_not_called()
        self.assertEqual(controller.phase.current, NavState.DETECT)  # not forced ONHOLD

    def test_exit_nav_oneshot_logs_snap(self):
        """In one-shot mode, SNAP is logged before disarm (after navigation was active)."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        navigation = _create_mock_navigation()
        logger = Mock()
        controller = _create_controller(
            vehicle=vehicle, args=args, navigation=navigation, logger=logger
        )
        controller.navigation_task.final_approach_navigation_active = True

        controller.nav_transition.exit()

        navigation.reset.assert_called_once()
        snap_logged = any("SNAP" in str(call) for call in logger.info.call_args_list)
        self.assertTrue(snap_logged)

    def test_exit_nav_no_snap_when_navigation_task_never_started(self):
        """No (meaningless) SNAP log when navigation never became active; navigation
        state is still reset for cleanup."""
        vehicle = _create_mock_vehicle()
        navigation = _create_mock_navigation()
        logger = Mock()
        controller = _create_controller(
            vehicle=vehicle, navigation=navigation, logger=logger)
        controller.navigation_task.final_approach_navigation_active = False

        controller.nav_transition.exit()

        navigation.reset.assert_called_once()  # cleanup still runs
        snap_logged = any("SNAP" in str(c) for c in logger.info.call_args_list)
        self.assertFalse(snap_logged)

    def test_exit_nav_default_does_not_disarm(self):
        """In default mode, _exit_nav does NOT disarm."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=False)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.nav_transition.exit()

        vehicle.disarm.assert_not_called()

    def test_exit_nav_oneshot_clears_state(self):
        """In one-shot mode, _exit_nav clears POI manager and peer nav."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.navigation_task.final_approach_navigation_active = True
        controller.navigation_task.final_approach_nav_completed = True
        controller.navigation_task.peer_navigation = True
        controller.mission.default_delivery_hub_active = True

        controller.nav_transition.exit()

        self.assertFalse(controller.navigation_task.peer_navigation)
        self.assertFalse(controller.mission.default_delivery_hub_active)

    def test_exit_nav_oneshot_does_not_disarm_on_stop(self):
        """_exit_nav does not disarm when stop event is set."""
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.navigation_task.final_approach_navigation_active = True
        controller.loop.stop_event.set()

        controller.nav_transition.exit()

        vehicle.disarm.assert_not_called()

    def test_stop_oneshot_does_not_disarm(self):
        """stop() must NOT disarm in one-shot mode.

        stop() now calls navigation.reset() + _clear_state() directly,
        not _exit_nav(), so disarm never happens.
        """
        vehicle = _create_mock_vehicle()
        vehicle.disarm = Mock()
        args = _create_mock_args(is_oneshot=True)
        controller = _create_controller(vehicle=vehicle, args=args)

        controller.application.stop()

        vehicle.disarm.assert_not_called()

    def test_exit_detector_failure_still_fences_flushes_and_restores_speed(self):
        order = []
        detector_error = RuntimeError("detector stop failed")
        logger_error = RuntimeError("status flush failed")
        vehicle = _create_mock_vehicle()
        vehicle.set_parameter.side_effect = (
            lambda _name, value: order.append(f"speed:{value}") or True
        )
        detector = _create_mock_detector()
        detector.stop_tracking.side_effect = (
            lambda *_args, **_kwargs: order.append("detector")
            or (_ for _ in ()).throw(detector_error)
        )
        navigation = _create_mock_navigation()
        snap = navigation.reset.return_value
        navigation.reset.side_effect = lambda: order.append("fence") or snap
        logger = Mock()
        def fail_status_flush(*_args, **_kwargs):
            order.append("flush")
            raise logger_error

        logger.defer_status_texts.side_effect = fail_status_flush
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
            logger=logger,
            args=_create_mock_args(nav_sim_speedup=10.0),
        )
        self.assertTrue(controller.speedup.apply())
        order.clear()

        with self.assertRaises(ExceptionGroup) as raised:
            controller.nav_transition.exit()

        self.assertEqual(order[:4], ["fence", "detector", "flush", "speed:1.0"])
        self.assertEqual(
            raised.exception.exceptions,
            (detector_error, logger_error),
        )
        logger.defer_status_texts.assert_called_once_with(False, flush=True)

    def test_exit_fences_stale_command_before_blocked_detector_stop(self):
        import threading

        from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot

        lock = threading.RLock()
        slot = NavigationCommandSlot(lock, threading.Event())
        slot.replace("stale-command")
        lease = slot.take_or_else(lambda: None)
        self.assertIsNotNone(lease)
        detector_entered = threading.Event()
        release_detector = threading.Event()

        def block_detector(*_args, **_kwargs):
            detector_entered.set()
            if not release_detector.wait(2.0):
                raise TimeoutError("test did not release detector stop")

        detector = _create_mock_detector()
        detector.stop_tracking.side_effect = block_detector
        navigation = _create_mock_navigation()
        snap = navigation.reset.return_value
        navigation.reset.side_effect = lambda: (slot.invalidate(), snap)[1]
        controller = _create_controller(detector=detector, navigation=navigation)
        exit_thread = threading.Thread(target=controller.nav_transition.exit)
        exit_thread.start()
        try:
            self.assertTrue(detector_entered.wait(1.0))
            issued = []
            self.assertIsNone(
                slot.execute_if_current(lease, lambda: issued.append("actuated"))
            )
            self.assertEqual(issued, [])
        finally:
            release_detector.set()
            exit_thread.join(timeout=2.0)
        self.assertFalse(exit_thread.is_alive())


# =============================================================================
# Default Delivery Hub Navigation Tests
# =============================================================================

class TestNavControllerDefaultDeliveryHub(unittest.TestCase):
    """Tests for default delivery hub (DDH) navigation trigger."""

    def _make_controller_with_fallback(self, next_wp=8, total=10):
        """Create controller with a default delivery hub and configurable mission state."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=next_wp)
        vehicle.mission_items_count = total
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation
        )
        controller.mission.default_delivery_hub = Location(40.5, 44.5, 0.0)
        return controller

    def test_should_nav_at_last_wp(self):
        """Trigger when next_wp == total - 1 (past search, heading to DDH WP)."""
        controller = self._make_controller_with_fallback(next_wp=9, total=10)
        self.assertTrue(controller.fallback_navigation.should_nav_to_fallback())

    def test_should_nav_past_last_wp(self):
        """Trigger when next_wp >= total."""
        controller = self._make_controller_with_fallback(next_wp=10, total=10)
        self.assertTrue(controller.fallback_navigation.should_nav_to_fallback())

    def test_should_not_nav_before_last_wp(self):
        """Do not trigger when still on search waypoints."""
        controller = self._make_controller_with_fallback(next_wp=8, total=10)
        self.assertFalse(controller.fallback_navigation.should_nav_to_fallback())

    def test_should_not_nav_without_default(self):
        """Do not trigger when no default delivery hub is set."""
        controller = self._make_controller_with_fallback(next_wp=9, total=10)
        controller.mission.default_delivery_hub = None
        self.assertFalse(controller.fallback_navigation.should_nav_to_fallback())

    def test_should_not_nav_when_navigation_task_already_started(self):
        """Do not trigger when default delivery hub is already active."""
        controller = self._make_controller_with_fallback(next_wp=9, total=10)
        controller.mission.default_delivery_hub_active = True
        self.assertFalse(controller.fallback_navigation.should_nav_to_fallback())

    def test_setup_fallback_navigation_sets_guided(self):
        """_setup_default_delivery_hub_navigation switches to GUIDED mode."""
        controller = self._make_controller_with_fallback(next_wp=9, total=10)
        controller.fallback_navigation.setup()
        controller.vehicle.set_mode.assert_called_with(FlightMode.GUIDED)
        self.assertTrue(controller.mission.default_delivery_hub_active)

    def test_setup_fallback_navigation_sets_sim_poi(self):
        """_setup_default_delivery_hub_navigation places sim POI at last WP."""
        controller = self._make_controller_with_fallback(next_wp=9, total=10)
        controller.fallback_navigation.setup()
        controller.detector.set_sim_poi.assert_called_once_with(
            9, controller.mission.default_delivery_hub,  # mission_items_count - 1
            location_type=controller.mission.default_delivery_hub_type,
        )


# =============================================================================
# Peer Task Priority Tests (docs/design/swarm-task-assignment-ack.md)
# =============================================================================

def _assigned_peer_task(controller, task_id=77):
    """A task actor whose slot holds an ASSIGNED peer task."""
    task_actor = Mock()
    task_actor.selected_poi.return_value = TaskAssignMsgData(
        task_id=task_id,
        task_type=TaskTypeMsgData.DOCK,
        location=LocationMsgData(40.001, -74.001, 0.0),
    )
    controller.network.task_actor = task_actor
    return task_actor


def _commit(controller, state):
    controller.phase.current = state
    controller.phase.previous = state


class TestNavControllerPeerTaskPriority(unittest.TestCase):
    """An assigned peer task outranks an own POI not yet approached and the
    DDH return; one final approach per flight."""

    # Owner ruling 2026-10-08: the own POI is dropped, never offered. It may
    # be the assigned dock itself (task ids are numbered per UAV), and
    # offering it could send a second UAV there.

    def test_confirm_drops_the_own_poi_without_reset(self):
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7),
        )
        task_actor = _assigned_peer_task(controller)
        own = _create_detected_poi(obj_id=5)
        own.set_p_t_g_loc(Location(40.002, -74.002, 0.0))
        controller.confirmation_manager.set_active_poi(own)
        controller.confirmation_manager.update_status(
            own, ConfirmationStatus.CONFIRMING,
        )
        _commit(controller, NavState.CONFIRM)

        controller.decision.decide()

        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIs(
            controller.confirmation_manager.get_status(own),
            ConfirmationStatus.DROPPED,
        )
        self.assertIs(controller.phase.current, NavState.DETECT)
        # No RESET: that would release the assigned task.
        task_actor.clear_selected_poi.assert_not_called()
        task_actor.reset.assert_not_called()

        controller.actions.act()

        self.assertTrue(controller.navigation_task.peer_navigation)
        controller.navigation.vehicle_commands.peer_poi.assert_called_once()
        task_actor.notify_pois.assert_not_called()

    def test_detect_drops_a_found_own_poi_and_flies_the_peer_task(self):
        controller = _create_controller()
        task_actor = _assigned_peer_task(controller)
        own = _create_detected_poi(obj_id=6)
        own.set_p_t_g_loc(Location(40.002, -74.002, 0.0))
        controller.detections.detected_pois = [own]
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIs(
            controller.confirmation_manager.get_status(own),
            ConfirmationStatus.DROPPED,
        )
        task_actor.notify_pois.assert_not_called()
        self.assertTrue(controller.navigation_task.peer_navigation)

    def test_a_dropped_poi_is_not_offered_beside_another_dock(self):
        controller = _create_controller()
        task_actor = _assigned_peer_task(controller)
        own = _create_detected_poi(obj_id=6)
        own.set_p_t_g_loc(Location(40.002, -74.002, 0.0))
        controller.detections.detected_pois = [own]
        _commit(controller, NavState.DETECT)
        controller.detect_action.act()
        other = _create_detected_poi(obj_id=8)
        other.set_p_t_g_loc(Location(40.003, -74.003, 0.0))

        # The other dock is selected first, so the dropped one is a peer
        # candidate; it is still never offered.
        controller.detections.detected_pois = [other, own]
        controller.detect_action.act()

        offered = [
            poi
            for call in task_actor.notify_pois.call_args_list
            for poi in call.args[0]
        ]
        self.assertNotIn(own, offered)

    def test_the_peer_approach_takes_up_a_dropped_poi_at_its_dock(self):
        # The dropped POI may be the assigned dock: near it, the peer
        # approach starts it, and its confirmation can be asked.
        controller = _create_controller()
        _assigned_peer_task(controller)
        own = _create_detected_poi(obj_id=6)
        own.set_p_t_g_loc(Location(40.002, -74.002, 0.0))
        controller.detections.detected_pois = [own]
        _commit(controller, NavState.DETECT)
        controller.detect_action.act()
        controller.peer_navigation.near_poi = Mock(return_value=True)

        controller.detect_action.act()

        self.assertIs(controller.confirmation_manager.active_poi, own)
        self.assertIsNone(controller.confirmation_manager.get_status(own))

    def test_ddh_return_yields_to_an_assigned_peer_task(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=9)
        vehicle.mission_items_count = 10
        controller = _create_controller(vehicle=vehicle)
        controller.mission.default_delivery_hub = Location(40.5, 44.5, 0.0)
        controller.fallback_navigation.setup()
        self.assertTrue(controller.mission.default_delivery_hub_active)
        _assigned_peer_task(controller)
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        self.assertTrue(controller.navigation_task.peer_navigation)
        self.assertEqual(
            controller.navigation_task.navigation_poi_location.lat,
            pytest_approx(40.001),
        )

    def test_peer_task_does_not_preempt_an_own_final_approach(self):
        controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
        )
        task_actor = _assigned_peer_task(controller)
        own = _create_detected_poi(obj_id=7)
        controller.confirmation_manager.set_active_poi(own)
        controller.confirmation_manager.update_status(
            own, ConfirmationStatus.CONFIRMED,
        )
        _commit(controller, NavState.NAV)

        controller.poi_status.dispatch()

        self.assertIs(controller.confirmation_manager.active_poi, own)
        task_actor.notify_pois.assert_not_called()


class TestNavControllerApproachAvailability(unittest.TestCase):
    """A committed NAV phase reports BUSY and makes the swarm reject offers."""

    def setUp(self):
        patcher = patch(
            "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            threading.TIMEOUT_MAX,
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.GUIDED),
        )
        helper = Mock()
        helper.source_system = 2
        helper.location.return_value = Location(40.0, -74.0, 200.0)
        helper.ground_speed = 20.0
        helper.wind.speed = 0.0
        helper.wind.direction = 0.0
        helper.battery_level = 100.0
        self.swarm_network = Mock(spec=NetworkAbc)
        self.actor = TaskActor(helper, self.swarm_network, Mock())
        self.actor.start()
        self.addCleanup(self.actor.reset)
        self.actor.on_message(SwarmHeartbeatMsg(sender_id=1))
        self.controller.network.task_actor = self.actor
        self.nav_action = Mock()
        self.controller.actions._actions[NavState.NAV] = self.nav_action

    def _heartbeat_state(self):
        self.swarm_network.reset_mock()
        self.actor._presence.heartbeat()
        (beat,) = [c.args[0] for c in self.swarm_network.broadcast.call_args_list]
        return SwarmNodeState(beat.state)

    def test_nav_phase_is_busy_and_rejects_a_step_three(self):
        _commit(self.controller, NavState.NAV)

        self.controller.actions.act()
        self.nav_action.assert_called_once_with()
        self.assertIs(self._heartbeat_state(), SwarmNodeState.BUSY)

        with patch("threading.Timer") as timer:
            self.actor.on_message(TaskAssignRequestMsg(
                sender_id=1,
                receiver_id=2,
                task=TaskAssignMsgData(
                    task_id=9,
                    task_type=TaskTypeMsgData.DOCK,
                    location=LocationMsgData(40.001, -74.001, 0.0),
                ),
                meta=MsgMeta(boot_id=7, msg_seq=50, time_ms=0, ttl_ms=5000),
            ))
            (deferred_reject,) = timer.call_args_list
            deferred_reject.args[1]()

        answers = [
            message
            for c in self.swarm_network.broadcast.call_args_list
            if isinstance((message := c.args[0]), TaskAssignResponseMsg)
        ]
        self.assertEqual(
            [(m.receiver_id, m.task_id, m.is_accepted) for m in answers],
            [(1, 9, False)],
        )
        self.assertIsNone(self.actor.selected_poi())

    def test_leaving_nav_reports_free_again(self):
        _commit(self.controller, NavState.NAV)
        self.controller.actions.act()

        _commit(self.controller, NavState.DETECT)
        self.controller.actions._actions[NavState.DETECT] = Mock()
        self.controller.actions.act()

        self.assertIs(self._heartbeat_state(), SwarmNodeState.FREE)


class TestNavControllerPeerTaskHandshake(unittest.TestCase):
    """Nav flies a peer task only once its owner applied the answer."""

    OWNER_BOOT = 7

    def setUp(self):
        for target, value in (
            ("threading.Timer", MagicMock()),
            (
                "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
                threading.TIMEOUT_MAX,
            ),
        ):
            patcher = patch(target, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.navigation = _create_mock_navigation()
        self.controller = _create_controller(
            vehicle=_create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5),
            navigation=self.navigation,
        )
        helper = Mock()
        helper.source_system = 2
        helper.location.return_value = Location(40.0, -74.0, 200.0)
        helper.ground_speed = 20.0
        helper.wind.speed = 0.0
        helper.wind.direction = 0.0
        helper.battery_level = 100.0
        self.swarm_network = Mock(spec=NetworkAbc)
        self.actor = TaskActor(helper, self.swarm_network, Mock())
        self.actor.start()
        self.addCleanup(self.actor.shutdown)
        self._owner_beat()
        self.controller.network.task_actor = self.actor
        _commit(self.controller, NavState.DETECT)

    def _owner_beat(self):
        self.actor.on_message(SwarmHeartbeatMsg(sender_id=1))

    def _offer(self):
        self.actor.on_message(TaskAssignRequestMsg(
            sender_id=1,
            receiver_id=2,
            task=TaskAssignMsgData(
                task_id=77,
                task_type=TaskTypeMsgData.DOCK,
                location=LocationMsgData(40.001, -74.001, 0.0),
            ),
            meta=MsgMeta(
                boot_id=self.OWNER_BOOT, msg_seq=50, time_ms=0, ttl_ms=5000,
            ),
        ))

    def _apply(self):
        answer = [
            message
            for c in self.swarm_network.broadcast.call_args_list
            if isinstance((message := c.args[0]), TaskAssignResponseMsg)
        ][-1]
        self.actor.on_message(SwarmAckMsg(
            sender_id=1,
            receiver_id=2,
            ref_boot_id=answer.meta.boot_id,
            ref_msg_seq=answer.meta.msg_seq,
            ref_msg_type=MsgType.TASK_ASSIGN_RESPONSE.value,
            status=ACK_STATUS_APPLIED,
            meta=MsgMeta(
                boot_id=self.OWNER_BOOT, msg_seq=60, time_ms=0, ttl_ms=5000,
            ),
        ))

    def test_no_peer_approach_while_waiting(self):
        self._offer()

        self.controller.detect_action.act()

        self.assertIs(self.actor._selection.state(), SlotState.WAITING)
        self.assertFalse(self.controller.navigation_task.peer_navigation)
        self.navigation.vehicle_commands.peer_poi.assert_not_called()

    def test_peer_approach_starts_after_the_owner_applies(self):
        self._offer()
        self.controller.detect_action.act()

        self._apply()
        self.controller.detect_action.act()

        self.assertTrue(self.controller.navigation_task.peer_navigation)
        self.navigation.vehicle_commands.peer_poi.assert_called_once()

    def test_nav_reset_keeps_a_waiting_task_and_releases_an_assigned_one(self):
        self._offer()

        self.controller.transitions.enter_reset()
        self.assertIs(self.actor._selection.state(), SlotState.WAITING)

        self.actor.start()  # nav restarts the actor on its next state
        self._owner_beat()
        self._apply()
        self.assertIs(self.actor._selection.state(), SlotState.ASSIGNED)
        self.controller.transitions.enter_reset()

        self.assertIs(self.actor._selection.state(), SlotState.EMPTY)
        self.assertIsNone(self.controller.network.selected_poi())


# =============================================================================
# Swarm Dispatch Safety Tests
# =============================================================================

class TestNavControllerSwarmDispatchSafety(unittest.TestCase):
    """Tests for swarm dispatch fixes: PEER_NOTIFIED skip, NAV dispatch, reset preservation."""

    def test_select_poi_skips_peer_notified_no_active(self):
        """_select_poi skips PEER_NOTIFIED POIs when no active POI, picks next valid."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        # P1 was dispatched to a peer
        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)

        self_poi, peer_pois = controller.selector.select([t1, t2, t3])

        # P1 should be skipped; P2 becomes self POI
        self.assertEqual(self_poi.identity.obj_id, 2)
        peer_ids = [p.identity.obj_id for p in peer_pois]
        self.assertIn(1, peer_ids)  # P1 goes to peers for re-broadcast
        self.assertIn(3, peer_ids)

    def test_select_poi_all_peer_notified_returns_none(self):
        """_select_poi returns self_poi=None when ALL POIs are PEER_NOTIFIED."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)

        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)
        controller.confirmation_manager.update_status(t2, ConfirmationStatus.PEER_NOTIFIED)

        self_poi, peer_pois = controller.selector.select([t1, t2])

        self.assertIsNone(self_poi)
        self.assertEqual(len(peer_pois), 2)

    def test_handle_new_poi_skips_peer_notified(self):
        """_handle_new_poi does not select a PEER_NOTIFIED POI."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        t1 = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)

        controller.navigation_task_action.handle_new_poi(t1)

        # Should NOT set active POI
        self.assertIsNone(controller.confirmation_manager.active_poi)

    def test_act_nav_dispatches_peer_pois(self):
        """NAV enqueues peer POIs without doing network work inline."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        controller = _create_controller(vehicle=vehicle)
        controller.network.task_actor = Mock()
        controller.network.peer_dispatch = Mock()

        # Active POI P1 being navigated to
        t1 = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(t1)
        controller.confirmation_manager.update_status(t1, ConfirmationStatus.CONFIRMED)

        # New POI P2 enters range during NAV
        t2 = _create_detected_poi(obj_id=2)
        controller.detections.detected_pois = [t1, t2]

        controller.phase.current = NavState.NAV
        controller.final_approach_nav.act_nav()

        # P2 should be submitted to the off-loop peer worker. The NAV call
        # itself must never invoke TaskActor/network I/O.
        controller.network.peer_dispatch.submit.assert_called_once()
        submitted = controller.network.peer_dispatch.submit.call_args[0][0]
        notified_ids = [t.identity.obj_id for t in submitted]
        self.assertIn(2, notified_ids)
        self.assertNotIn(1, notified_ids)
        controller.network.task_actor.notify_pois.assert_not_called()

    def test_reset_clears_all_statuses(self):
        """reset() clears all entries including PEER_NOTIFIED."""
        from navpy.modules.nav.confirmation_manager import ConfirmationManager
        tm = ConfirmationManager(1, _create_mock_args(), Mock())

        t1 = _create_detected_poi(obj_id=1)
        t2 = _create_detected_poi(obj_id=2)
        t3 = _create_detected_poi(obj_id=3)

        tm.update_status(t1, ConfirmationStatus.PEER_NOTIFIED)
        tm.update_status(t2, ConfirmationStatus.CONFIRMED)
        tm.update_status(t3, ConfirmationStatus.REJECTED)

        tm.reset()

        self.assertIsNone(tm.get_status(t1))
        self.assertIsNone(tm.get_status(t2))
        self.assertIsNone(tm.get_status(t3))
        self.assertIsNone(tm.active_poi)

    def test_enter_reset_calls_confirmation_manager_reset(self):
        """NavController._enter_reset() calls confirmation_manager.reset()."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        with patch.object(controller.confirmation_manager, 'reset') as mock_reset:
            controller.transitions.enter_reset()

        mock_reset.assert_called_once_with()

    def test_task_actor_ignores_own_available_request(self):
        """TaskActor routing ignores self-sent availability messages."""
        from navpy.modules.swarm.task_actor import TaskActor

        vehicle = Mock()
        vehicle.source_system = 1
        vehicle.location.return_value = Location(40.0, -74.0, 200.0)
        network = Mock()
        logger = Mock()

        actor = TaskActor(vehicle, network, logger)
        actor.start()
        network.reset_mock()

        msg = AvailableTaskRequestMsg(sender_id=1, tasks=[])
        actor.on_message(msg)

        # Should return early — no broadcast
        network.broadcast.assert_not_called()
        actor.reset()


# =============================================================================
# Resume AUTO Mission After Rejection Tests
# =============================================================================

class TestResumeAutoMissionAfterRejection(unittest.TestCase):
    """Tests for the CONFIRM -> DETECT (rejection) transition.

    When an operator denies a POI confirmation while the vehicle is in
    GUIDED mode, the vehicle must switch back to AUTO and continue the
    mission from the current waypoint (not restart from WP 1).
    """

    def test_rejection_in_guided_mode_resumes_auto(self):
        """Rejection while in GUIDED mode sets vehicle back to AUTO."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        # Simulate: vehicle was in CONFIRM state with an active POI
        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)

    def test_rejection_does_not_restart_mission(self):
        """Rejection should NOT call restart_mission -- vehicle continues from current WP."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        vehicle.restart_mission.assert_not_called()
        vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)

    def test_rejection_clears_peer_navigation(self):
        """Rejection clears peer navigation state."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)
        controller.navigation_task.peer_navigation = True

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertFalse(controller.navigation_task.peer_navigation)

    def test_rejection_clears_selected_pois(self):
        """Rejection clears task actor selected_pois."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        # Set up task actor with selected POIs. Its approach is under way:
        # a peer task not yet started would take priority over this review.
        task_actor = Mock()
        task_actor.selected_poi.return_value = Mock()
        controller.network.task_actor = task_actor
        controller.navigation_task.peer_navigation = True

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        task_actor.clear_selected_poi.assert_called_once_with()

    def test_rejection_pauses_the_final_approach_loop(self):
        """Rejection pauses the navigation final-approach loop."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        navigation.pause_final_approach.assert_called()

    def test_rejection_resets_pass_detector(self):
        """Rejection resets the pass detector state."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        # Simulate some pass detector state
        controller.pass_tracker.previous_distance_m = 100.0
        controller.pass_tracker.approach_started = True
        controller.pass_tracker.increase_count = 3

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertIsNone(controller.pass_tracker.previous_distance_m)
        self.assertFalse(controller.pass_tracker.approach_started)
        self.assertEqual(controller.pass_tracker.increase_count, 0)

    def test_rejection_in_auto_mode_does_not_call_resume(self):
        """If vehicle is already in AUTO mode during rejection, no mode switch needed."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        # set_mode should not be called since mode is already AUTO
        vehicle.set_mode.assert_not_called()

    def test_rejection_clears_default_delivery_hub_navigation_task_started(self):
        """Rejection resets _default_delivery_hub_active flag."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)
        controller.mission.default_delivery_hub_active = True

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertFalse(controller.mission.default_delivery_hub_active)

    def test_rejection_clears_last_detections(self):
        """Rejection clears cached detections so stale data is not re-processed."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)
        controller.detections.detected_pois = [_create_detected_poi(obj_id=99)]

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertEqual(controller.detections.detected_pois, [])

    def test_full_flow_confirm_reject_resume(self):
        """Full flow: DETECT -> CONFIRM -> rejection -> resume AUTO -> DETECT."""
        poi = _create_detected_poi(obj_id=10, detection_frame=np.zeros((100, 100, 3)),
                                          bbox=(50, 50, 20, 20))
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector(detections=[poi])
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, detector=detector, navigation=navigation)

        # Step 1: SENSE + DECIDE -> DETECT (no active POI)
        controller.sensor.sense()
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)

        # Step 2: ACT_DETECT -> sets active POI
        controller.detect_action.act()
        self.assertIsNotNone(controller.confirmation_manager.active_poi)
        self.assertEqual(controller.confirmation_manager.active_poi.identity.obj_id, 10)

        # Step 3: DECIDE -> CONFIRM (active POI, no status yet)
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.CONFIRM)

        # Step 4: Simulate operator rejection while in GUIDED mode
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        vehicle.get_mode = FlightMode.GUIDED  # vehicle was put in GUIDED during CONFIRM

        controller.decision.decide()

        # Should transition to DETECT and resume AUTO
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        vehicle.set_mode.assert_called_with(FlightMode.AUTO)
        vehicle.restart_mission.assert_not_called()

    def test_cancel_after_approval_in_nav_breaks_off_and_resumes(self):
        """Cancel-after-confirmation: an approved POI recalled mid-dive.

        The vehicle is in NAV (mode GUIDED, diving on a CONFIRMED POI).
        The operator cancels, so the status flips CONFIRMED -> REJECTED. The
        next decide tick must break off (clear active POI, stop tracking,
        DETECT) and resume AUTO -- WITHOUT going through the NAV mode-switch
        abort path (mode is still GUIDED). This is the vehicle-side behavior
        the GCS 'Cancel delivery' button relies on.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV

        # Operator cancels mid-dive -> CONFIRMED -> REJECTED.
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called()
        vehicle.set_mode.assert_called_with(FlightMode.AUTO)
        vehicle.restart_mission.assert_not_called()

    def test_resume_auto_mission_without_task_actor(self):
        """_resume_auto_mission works correctly when no task actor is set."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)
        controller.network.task_actor = None

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        # Should not raise even without task actor
        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)

    def test_rejection_sets_current_to_previous_waypoint(self):
        """Rejection resumes from the previous WP to re-fly the diverted leg."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        # next_wp=7, so resume from WP 6 (previous leg)
        vehicle.set_current.assert_called_once_with(6)

    def test_rejection_clamps_resume_wp_to_zero(self):
        """When next_wp is 0, resume WP is clamped to 0 (not negative)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=0)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        vehicle.set_current.assert_called_once_with(0)

    def test_rejection_does_not_refresh_detector(self):
        """Rejection must NOT refresh the detector — resetting the tracker
        causes obj_id reuse, colliding with REJECTED entries in the status map
        and blocking DDH navigation_task."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        detector.refresh.assert_not_called()

    def test_rejection_stops_gimbal_tracking(self):
        """Rejection must stop gimbal tracking so the detector can find new POIs.

        Without this, the gimbal stays locked on the rejected obj_id, eventually
        recenters to FOLLOW (forward-looking), and during orbit around the next
        DDH the camera never sees the POI at the orbit center.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        detector.stop_tracking.assert_called_once()

    def test_rejection_handles_none_mission_items_next(self):
        """When mission_items_next is None, set_current is skipped."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=None)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)
        vehicle.set_current.assert_not_called()


class TestSelectPoiSkipsProcessed(unittest.TestCase):
    """_select_poi must skip REJECTED/CONFIRMED POIs so they don't
    block the elif chain in _handle_new_poi (peer nav, DDH)."""

    def test_rejected_poi_not_returned_as_self(self):
        """A rejected POI should be skipped, not returned as self_poi."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        self_t, peers = controller.selector.select([poi])
        self.assertIsNone(self_t)

    def test_confirmed_poi_not_returned_as_self(self):
        """A confirmed POI should be skipped, not returned as self_poi."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)

        self_t, peers = controller.selector.select([poi])
        self.assertIsNone(self_t)

    def test_new_poi_returned_when_rejected_also_present(self):
        """With a rejected and a new POI, only the new one is self_poi."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)

        rejected = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(rejected, ConfirmationStatus.REJECTED)
        fresh = _create_detected_poi(obj_id=20)

        self_t, peers = controller.selector.select([rejected, fresh])
        self.assertEqual(self_t.identity.obj_id, 20)

    def test_delivery_hub_reachable_after_deny(self):
        """After deny, _handle_new_poi falls through to DDH when only
        rejected POIs are visible."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=9)
        vehicle.mission_items_count = 10
        controller = _create_controller(vehicle=vehicle)
        controller.mission.default_delivery_hub = Location(40.0, -74.0, 0.0)
        controller.mission.default_delivery_hub_active = False

        # Simulate rejected POI still visible in detector
        rejected = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(rejected, ConfirmationStatus.REJECTED)
        controller.detections.detected_pois = [rejected]

        # select_poi should return None for self_poi
        self_t, _ = controller.selector.select([rejected])
        self.assertIsNone(self_t)

        # handle_new_poi should fall through to DDH setup
        controller.navigation_task_action.handle_new_poi(self_t)
        self.assertTrue(controller.mission.default_delivery_hub_active)

    def test_full_deny_then_delivery_hub_flow(self):
        """Full flow: deny POI -> resume AUTO -> reach end of track -> delivery hub selected."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=7)
        vehicle.mission_items_count = 10
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, detector=detector, navigation=navigation)
        controller.mission.default_delivery_hub = Location(40.0, -74.0, 0.0)

        # Step 1: Deny — CONFIRM + REJECTED in GUIDED -> resume AUTO
        poi = _create_detected_poi(obj_id=42)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.phase.current = NavState.CONFIRM

        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)
        vehicle.set_mode.assert_called_with(FlightMode.AUTO)

        # Step 2: Vehicle continues in AUTO, reaches end of track
        vehicle.get_mode = FlightMode.AUTO
        vehicle.mission_items_next = 9  # at DDH WP (total - 1)

        # No new detections (rejected POI out of view)
        detector.get_detect_data.return_value = DetectResponse([])
        controller.sensor.sense()
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)

        # Step 3: act_detect should fall through to DDH
        controller.detect_action.act()
        self.assertTrue(controller.mission.default_delivery_hub_active)


# =============================================================================
# Recovery State Tests
# =============================================================================

class TestNavControllerRecovery(unittest.TestCase):
    """Tests for RECOVERY state behavior."""

    def test_decide_recovery_below_min_alt_from_detect(self):
        """DETECT → RECOVERY when alt < min_alt."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=30.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.DETECT

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)

    def test_reset_with_missing_altitude_stays_closed_without_pitch_command(self):
        """A missing pre-GPS location is unknown, never synthetic zero AGL."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        vehicle.location = Mock(return_value=None)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)
        controller.phase.current = NavState.RESET

        controller.decision.decide()
        controller.actions.act()

        self.assertEqual(controller.phase.current, NavState.RESET)
        vehicle.set_attitude.assert_not_called()
        navigation.pause_final_approach.assert_called()

    def test_decide_recovery_below_min_alt_from_nav(self):
        """NAV → RECOVERY when alt < min_alt."""
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED, next_wp=5, alt=30.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.phase.current = NavState.NAV

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.RECOVERY)

    def test_decide_recovery_stays_until_alt_recovered(self):
        """Stays in RECOVERY while alt < min_alt + ALT_HYST."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=51.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RECOVERY

        controller.decision.decide()

        # 51.0 < 50.0 + 3.0 = 53.0 → still RECOVERY
        self.assertEqual(controller.phase.current, NavState.RECOVERY)

    def test_decide_recovery_to_detect_when_alt_ok(self):
        """RECOVERY → DETECT when alt >= min_alt + ALT_HYST."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=53.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RECOVERY

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)

    def test_decide_recovery_to_onhold_on_bad_mode(self):
        """RECOVERY → ONHOLD when mode leaves active set."""
        vehicle = _create_mock_vehicle(mode=FlightMode.MANUAL, next_wp=5, alt=30.0)
        args = _create_mock_args(min_alt=50.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.phase.current = NavState.RECOVERY

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.ONHOLD)

    def test_act_recovery_commands_attitude(self):
        """_act_recovery commands pitch-up attitude."""
        vehicle = _create_mock_vehicle()
        vehicle.max_pitch = 25.0
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        controller.recovery.act()

        expected_pitch = math.radians(2 * 25.0)
        vehicle.set_attitude.assert_called_once_with(0, expected_pitch, 0, 1.0)

    def test_act_recovery_pauses_navigation(self):
        """_act_recovery pauses the final-approach loop."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.recovery.act()

        navigation.pause_final_approach.assert_called()

    def test_recovery_to_detect_sets_auto_and_restarts(self):
        """RECOVERY → DETECT triggers _restart_auto_mission."""
        vehicle = _create_mock_vehicle()
        controller = _create_controller(vehicle=vehicle)

        controller.transitions.on_change(NavState.RECOVERY, NavState.DETECT)

        vehicle.set_mode.assert_called_with(FlightMode.AUTO)
        vehicle.restart_mission.assert_called_with(1)

    def test_enter_recovery_pauses_and_logs(self):
        """Entering RECOVERY pauses navigation and logs."""
        navigation = _create_mock_navigation()
        logger = Mock()
        controller = _create_controller(navigation=navigation, logger=logger)

        controller.recovery.enter()

        navigation.pause_final_approach.assert_called()
        recovery_logged = any("RECOVERY" in str(call) for call in logger.info.call_args_list)
        self.assertTrue(recovery_logged)


# =============================================================================
# On State Change Tests
# =============================================================================

class TestNavControllerOnStateChange(unittest.TestCase):
    """Tests for _on_state_change dispatch."""

    def test_nav_to_reset_calls_exit_nav(self):
        """NAV → RESET calls _exit_nav (SNAP log)."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.transitions.on_change(NavState.NAV, NavState.RESET)

        # _exit_nav calls navigation.reset() for SNAP
        navigation.reset.assert_called_once()

    def test_nav_to_recovery_calls_exit_nav(self):
        """NAV → RECOVERY calls _exit_nav."""
        navigation = _create_mock_navigation()
        controller = _create_controller(navigation=navigation)

        controller.transitions.on_change(NavState.NAV, NavState.RECOVERY)

        navigation.reset.assert_called_once()

    def test_detect_to_nav_calls_enter_nav(self):
        """DETECT → NAV calls _enter_nav (navigation init)."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, navigation=navigation)

        controller.transitions.on_change(NavState.DETECT, NavState.NAV)

        navigation.init.assert_called_once()
        vehicle.set_mode.assert_called_with(FlightMode.GUIDED)

    def test_enter_nav_switches_zoom_to_tracking_floor(self):
        """NAV entry drops the recognition-size zoom demand: the POI
        is confirmed, so zoom only keeps the track alive (tracking floor)
        and the frame contained during the dive.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        controller.transitions.on_change(NavState.DETECT, NavState.NAV)

        detector.set_zoom_size_demand.assert_called_once_with(False)

    def test_final_approach_enter_nav_freezes_zoom_at_min(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        detector = _create_mock_detector()
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            navigation=navigation,
        )

        controller.transitions.on_change(NavState.DETECT, NavState.NAV)

        detector.freeze_final_approach_zoom_at_min.assert_called_once_with()
        detector.set_zoom_size_demand.assert_not_called()

    def test_set_recognition_zoom_demand_helper(self):
        """The shared helper drops the recognition-size zoom demand — called at
        confirm-capture (and re-affirmed at NAV) so the zoom stops chasing
        48px and does not hunt in/out through the operator-confirm wait
        (run 163049). Best-effort: a detector lacking the hook must not raise.
        """
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        controller.zoom.set_recognition_demand(False)
        detector.set_zoom_size_demand.assert_called_once_with(False)

        detector.set_zoom_size_demand = None  # detector without the hook
        controller.zoom.set_recognition_demand(False)  # must not raise

    def test_reset_to_detect_restarts_mission(self):
        """RESET → DETECT calls _restart_auto_mission."""
        vehicle = _create_mock_vehicle()
        controller = _create_controller(vehicle=vehicle)

        controller.transitions.on_change(NavState.RESET, NavState.DETECT)

        vehicle.set_mode.assert_called_with(FlightMode.AUTO)
        vehicle.restart_mission.assert_called_with(1)

    def test_reset_to_recovery_does_not_restart(self):
        """RESET → RECOVERY does NOT call _restart_auto_mission."""
        vehicle = _create_mock_vehicle()
        controller = _create_controller(vehicle=vehicle)

        controller.transitions.on_change(NavState.RESET, NavState.RECOVERY)

        vehicle.set_mode.assert_not_called()
        vehicle.restart_mission.assert_not_called()

    def test_onhold_to_detect_starts_task_actor(self):
        """Leaving ONHOLD starts task actor."""
        controller = _create_controller()
        controller.network.task_actor = Mock()

        controller.transitions.on_change(NavState.ONHOLD, NavState.DETECT)

        controller.network.task_actor.start.assert_called()

    def test_start_navigation_task_sets_sim_speedup(self):
        """navigation_task_action.start sets SIM_SPEEDUP when configured."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        args = _create_mock_args(nav_sim_speedup=10.0)
        controller = _create_controller(vehicle=vehicle, args=args)
        controller.speedup.original = 1.0

        poi = _create_detected_poi(obj_id=0)
        controller.navigation_task_action.start(poi)

        vehicle.set_parameter.assert_called_with("SIM_SPEEDUP", 10.0)

    def test_start_navigation_task_defers_all_mutation_until_pending_rollback_verifies(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        vehicle.set_parameter.side_effect = [False, False, True]
        args = _create_mock_args(nav_sim_speedup=10.0)
        detector = _create_mock_detector()
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            args=args,
        )
        poi = _create_detected_poi(obj_id=17)

        self.assertIs(controller.navigation_task_action.start(poi), False)
        detector.start_tracking.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIs(controller.navigation_task_action.start(poi), True)
        detector.start_tracking.assert_called_once_with(17)
        self.assertIs(controller.confirmation_manager.active_poi, poi)
        self.assertEqual(
            vehicle.set_parameter.call_args_list,
            [
                call("SIM_SPEEDUP", 10.0),
                call("SIM_SPEEDUP", 1.0),
                call("SIM_SPEEDUP", 1.0),
            ],
        )

    def test_start_navigation_task_retries_without_mutation_when_speedup_baseline_missing(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        vehicle.get_parameter.side_effect = [None, 1.0]
        detector = _create_mock_detector()
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            args=_create_mock_args(nav_sim_speedup=10.0),
        )
        poi = _create_detected_poi(obj_id=19)

        self.assertIs(controller.navigation_task_action.start(poi), False)
        detector.start_tracking.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.active_poi)

        self.assertIs(controller.navigation_task_action.start(poi), True)
        detector.start_tracking.assert_called_once_with(19)
        self.assertIs(controller.confirmation_manager.active_poi, poi)
        vehicle.set_parameter.assert_called_once_with("SIM_SPEEDUP", 10.0)

    def test_pending_rollback_keeps_nav_exit_transition_retryable(self):
        vehicle = _create_mock_vehicle()
        vehicle.set_parameter.side_effect = [False, False, False, True]
        controller = _create_controller(
            vehicle=vehicle,
            args=_create_mock_args(nav_sim_speedup=10.0),
        )
        self.assertFalse(controller.speedup.apply())
        controller.phase.previous = NavState.NAV
        controller.phase.current = NavState.RESET

        with self.assertRaises(ExceptionGroup):
            controller.actions.act()

        self.assertEqual(controller.phase.previous, NavState.NAV)
        self.assertIsNotNone(controller.speedup.original)

        controller.actions.act()

        self.assertEqual(controller.phase.previous, NavState.RESET)
        self.assertIsNone(controller.speedup.original)
        self.assertEqual(
            vehicle.set_parameter.call_args_list,
            [
                call("SIM_SPEEDUP", 10.0),
                call("SIM_SPEEDUP", 1.0),
                call("SIM_SPEEDUP", 1.0),
                call("SIM_SPEEDUP", 1.0),
            ],
        )

    def test_start_navigation_task_leaves_sim_speedup_unchanged_when_disabled(self):
        """The default -gsu 0 preserves the launch-time SITL speed."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO)
        args = _create_mock_args(nav_sim_speedup=0.0)
        controller = _create_controller(vehicle=vehicle, args=args)

        poi = _create_detected_poi(obj_id=0)
        controller.navigation_task_action.start(poi)

        self.assertIsNone(controller.speedup.original)
        self.assertFalse(any(
            call.args and call.args[0] == "SIM_SPEEDUP"
            for call in vehicle.set_parameter.call_args_list
        ))

    def test_exit_nav_restores_sim_speedup(self):
        """_exit_nav restores SIM_SPEEDUP to initial value."""
        vehicle = _create_mock_vehicle()
        vehicle.get_parameter = Mock(return_value=1.0)
        args = _create_mock_args(nav_sim_speedup=10.0)
        navigation = _create_mock_navigation()
        controller = _create_controller(vehicle=vehicle, args=args, navigation=navigation)
        controller.speedup.apply()
        vehicle.set_parameter.reset_mock()

        controller.nav_transition.exit()

        vehicle.set_parameter.assert_called_with("SIM_SPEEDUP", 1.0)


class TestNavControllerPeerGeoPointing(unittest.TestCase):
    """Pre-acquisition geo-pointing wired through NavController.

    Step 4 makes the detector layer's geo APIs live: peer-nav ORBIT
    setup arms geo, _act_detect ticks it while detection is not yet
    armed, _clear_state and _resume_auto_mission tear it down.
    """

    def setUp(self):
        self.vehicle = _create_mock_vehicle()
        # Distinct sentinels for the two altitude-frame branches so a
        # regression that swaps location(True) <-> location(False) is
        # caught by tests rather than only at runtime in SITL.
        self.uav_loc_relative = Location(40.0, -74.0, 200.0, is_absolute=False)
        self.uav_loc_absolute = Location(40.0, -74.0, 1500.0, is_absolute=True)
        self.vehicle.location = Mock(side_effect=lambda is_relative: (
            self.uav_loc_relative if is_relative else self.uav_loc_absolute
        ))
        # _save_loiter_rad / _restore_loiter_rad use get/set_parameter
        # — return a real float so format strings in restore work.
        self.vehicle.get_parameter = Mock(return_value=80.0)
        self.vehicle.set_parameter = Mock()
        self.detector = _create_mock_detector()
        self.navigation = _create_mock_navigation()
        self.controller = _create_controller(
            vehicle=self.vehicle, detector=self.detector, navigation=self.navigation,
            approach_kind=ApproachKind.ORBIT,
        )

    def _arm_peer_orbit(self, lat=40.5, lng=-74.5, alt=900.0):
        """Stub the bits _setup_peer_navigation needs to reach the
        geo-arming branch, then call it. Returns the cached
        _peer_poi_loc for assertions.
        """
        from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
        task_loc = Mock()
        task_loc.lat = lat
        task_loc.lng = lng
        task_loc.alt = alt
        task = Mock()
        task.location = task_loc
        task.class_id = 0
        self.controller.network.task_actor = Mock()
        self.controller.network.task_actor.selected_poi.return_value = task
        plan = ApproachPlan(
            kind=ApproachKind.ORBIT,
            approach_location=Location(lat, lng, alt, is_absolute=True),
            offset_distance=0.0,
            orbit_radius=686.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ):
            self.controller.peer_navigation.setup()

    def _arm_peer_offset(self, lat=40.5, lng=-74.5, alt=900.0):
        """Same shape as _arm_peer_orbit but produces an OFFSET plan
        (with a positive orbit_radius — to verify the gate is
        kind-based, not radius-based).
        """
        from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
        task_loc = Mock()
        task_loc.lat = lat
        task_loc.lng = lng
        task_loc.alt = alt
        task = Mock()
        task.location = task_loc
        task.class_id = 0
        self.controller.network.task_actor = Mock()
        self.controller.network.task_actor.selected_poi.return_value = task
        plan = ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=Location(lat, lng, alt, is_absolute=True),
            offset_distance=300.0,
            # OFFSET plans CAN carry a positive loiter radius — the
            # gate must depend on plan.kind, not on orbit_radius.
            orbit_radius=80.0,
        )
        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ):
            self.controller.peer_navigation.setup()

    # --- arm path --------------------------------------------------------

    def test_init_peer_poi_loc_is_none(self):
        self.assertIsNone(self.controller.geo_hold.poi_location)

    def test_setup_peer_navigation_arms_geo_for_orbit_plan(self):
        self._arm_peer_orbit(lat=40.5, lng=-74.5, alt=900.0)

        self.assertIsNotNone(self.controller.geo_hold.poi_location)
        cached = self.controller.geo_hold.poi_location
        self.assertEqual(cached.lat, 40.5)
        self.assertEqual(cached.lng, -74.5)
        self.assertEqual(cached.alt, 900.0)
        # Peer task locations are MSL absolute by convention; cache
        # must carry that flag so _act_detect's pose math pairs it
        # with vehicle.location(False).
        self.assertTrue(cached.is_absolute)
        self.detector.start_geo_tracking.assert_called_once_with(
            cached, self.navigation.legacy_pois.geo_ref,
        )

    def test_final_approach_peer_navigation_never_arms_geo_pointing(self):
        self.navigation.final_approach.is_active = True

        self._arm_peer_orbit(lat=40.5, lng=-74.5, alt=900.0)

        self.assertIsNone(self.controller.geo_hold.poi_location)
        self.assertEqual(
            self.controller.navigation_task.navigation_poi_location,
            Location(40.5, -74.5, 900.0, is_absolute=True),
        )
        self.navigation.vehicle_commands.peer_poi_loiter.assert_called_once_with(
            Location(40.5, -74.5, 900.0, is_absolute=True),
            686.0,
            None,
        )
        self.detector.start_geo_tracking.assert_not_called()
        self.detector.update_geo.assert_not_called()

    def test_setup_peer_navigation_plans_with_absolute_pose(self):
        from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
        task_loc = Mock()
        task_loc.lat = 40.5
        task_loc.lng = -74.5
        task_loc.alt = 900.0
        task = Mock()
        task.location = task_loc
        task.class_id = 0
        self.controller.network.task_actor = Mock()
        self.controller.network.task_actor.selected_poi.return_value = task
        plan = ApproachPlan(
            kind=ApproachKind.ORBIT,
            approach_location=Location(40.5, -74.5, 900.0, is_absolute=True),
            offset_distance=0.0,
            orbit_radius=686.0,
        )

        with patch(
            "navpy.modules.nav.nav_composition.calc_peer_approach_offset",
            return_value=plan,
        ) as calc:
            self.controller.peer_navigation.setup()

        poi_arg, drone_arg = calc.call_args.args[:2]
        self.assertTrue(poi_arg.is_absolute)
        # No mission scan altitude here (scan_altitude_rel None,
        # get_mission_item_location None) -> legacy sizing path -> the
        # real absolute drone pose is passed unchanged.
        self.assertIs(drone_arg, self.uav_loc_absolute)

    def test_setup_peer_navigation_primes_geo_before_dispatch(self):
        events = []
        with patch.object(
            self.controller.vehicle_navigation,
            "request_guided",
            side_effect=lambda: events.append("guided"),
        ), patch.object(
            self.controller.vehicle_navigation,
            "dispatch_approach",
            side_effect=lambda plan, loiter_alt_rel=None: events.append("dispatch"),
        ):
            self.detector.start_geo_tracking.side_effect = (
                lambda *args: events.append("start")
            )
            self.detector.update_geo.side_effect = (
                lambda *args: events.append("update")
            )

            self._arm_peer_orbit()

        self.assertEqual(events, ["guided", "start", "update", "dispatch"])

    def test_setup_peer_navigation_primes_geo_with_absolute_pose(self):
        self._arm_peer_orbit()

        self.detector.update_geo.assert_called_once()
        args = self.detector.update_geo.call_args.args
        self.assertIs(args[0], self.uav_loc_absolute)
        self.assertIs(args[1], self.vehicle.attitude)

    def test_setup_peer_navigation_skips_immediate_prime_without_location(self):
        self.vehicle.location = Mock(side_effect=lambda is_relative: (
            self.uav_loc_relative if is_relative else None
        ))

        with patch.object(self.controller.vehicle_navigation, "dispatch_approach") as dispatch:
            self._arm_peer_orbit()

        self.detector.start_geo_tracking.assert_called_once()
        self.detector.update_geo.assert_not_called()
        dispatch.assert_called_once()
        self.assertIsNotNone(self.controller.geo_hold.poi_location)

    def test_setup_peer_navigation_skips_immediate_prime_without_attitude(self):
        self.vehicle.attitude = None

        with patch.object(self.controller.vehicle_navigation, "dispatch_approach") as dispatch:
            self._arm_peer_orbit()

        self.detector.start_geo_tracking.assert_called_once()
        self.detector.update_geo.assert_not_called()
        dispatch.assert_called_once()
        self.assertIsNotNone(self.controller.geo_hold.poi_location)

    def test_peer_geo_acquisition_logs_cached_miss_reason(self):
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = False
        self.detector.mounts = [mount]
        self.controller.geo_hold.poi_location = Location(40.5, -74.5, 900.0, is_absolute=True)
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        logger = self.controller.logger
        logger.info.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            state = self.controller.peer_geo_acquisition.snapshot(
                self.uav_loc_absolute,
                self.vehicle.attitude,
            )
            self.controller.peer_geo_acquisition.log(state)

        logger.info.assert_called_once()
        msg = logger.info.call_args.args[0]
        self.assertIn("PEER_GEO_ACQ:", msg)
        self.assertIn("reason=out_of_fov", msg)
        self.assertIn("range_h=2m", msg)
        self.assertIn("range_v=3m", msg)
        self.assertIn("slant=4m", msg)
        self.assertEqual(logger.info.call_args.kwargs["key"], "peer_geo_acq")

    def test_peer_geo_acquisition_formats_none_uv(self):
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        self.detector.mounts = [mount]
        self.controller.geo_hold.poi_location = Location(40.5, -74.5, 900.0, is_absolute=True)
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (None, None)
        logger = self.controller.logger
        logger.info.reset_mock()
        logger.warning.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            state = self.controller.peer_geo_acquisition.snapshot(
                self.uav_loc_absolute,
                self.vehicle.attitude,
            )
            self.controller.peer_geo_acquisition.log(state)

        msg = logger.info.call_args.args[0]
        self.assertIn("uv=(None,None)", msg)
        self.assertIn("reason=behind_cam", msg)
        logger.warning.assert_not_called()

    def test_peer_geo_acquisition_log_is_bucketed_by_reason(self):
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = False
        self.detector.mounts = [mount]
        self.controller.geo_hold.poi_location = Location(40.5, -74.5, 900.0, is_absolute=True)
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        logger = self.controller.logger
        logger.info.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            side_effect=[
                np.array([1.0, 2.0, -3.0]),
                np.array([4.0, 5.0, -6.0]),
            ],
        ):
            first = self.controller.peer_geo_acquisition.snapshot(
                self.uav_loc_absolute, self.vehicle.attitude,
            )
            second = self.controller.peer_geo_acquisition.snapshot(
                self.uav_loc_absolute, self.vehicle.attitude,
            )
            self.controller.peer_geo_acquisition.log(first)
            self.controller.peer_geo_acquisition.log(second)

        logger.info.assert_called_once()

    def test_setup_peer_navigation_does_not_arm_geo_for_offset_plan(self):
        """Even when an OFFSET plan carries a positive orbit_radius
        (e.g. the OFFSET_LOITER_RADIUS_M constant), geo-pointing must
        NOT arm — OFFSET flies straight at the POI so the airframe
        nose already points the bore correctly.
        """
        self._arm_peer_offset()

        self.assertIsNone(self.controller.geo_hold.poi_location)
        self.detector.start_geo_tracking.assert_not_called()
        self.detector.update_geo.assert_not_called()

    def test_setup_peer_navigation_continues_on_geo_arm_failure(self):
        self.detector.start_geo_tracking.side_effect = RuntimeError("locked")
        logger = self.controller.logger

        with patch.object(self.controller.vehicle_navigation, "dispatch_approach") as dispatch:
            self._arm_peer_orbit()

        # Peer-nav state still set; geo cache cleared so tick path no-ops.
        self.assertTrue(self.controller.navigation_task.peer_navigation)
        self.assertIsNone(self.controller.geo_hold.poi_location)
        self.detector.update_geo.assert_not_called()
        dispatch.assert_called_once()
        logger.warning.assert_called()

    def test_setup_peer_navigation_continues_on_geo_prime_failure(self):
        self.detector.update_geo.side_effect = RuntimeError("pose")
        logger = self.controller.logger

        with patch.object(self.controller.vehicle_navigation, "dispatch_approach") as dispatch:
            self._arm_peer_orbit()

        self.detector.update_geo.assert_called_once()
        self.detector.stop_geo_tracking.assert_called_once()
        self.assertIsNone(self.controller.geo_hold.poi_location)
        dispatch.assert_called_once()
        logger.warning.assert_called()

    def test_setup_peer_navigation_cleans_geo_on_dispatch_failure(self):
        with patch.object(
            self.controller.vehicle_navigation,
            "dispatch_approach",
            side_effect=RuntimeError("dispatch"),
        ):
            with self.assertRaisesRegex(RuntimeError, "dispatch"):
                self._arm_peer_orbit()

        self.detector.update_geo.assert_called_once()
        self.detector.stop_geo_tracking.assert_called_once()
        self.assertIsNone(self.controller.geo_hold.poi_location)

    def test_setup_peer_navigation_preserves_dispatch_error_on_teardown_failure(self):
        self.detector.stop_geo_tracking.side_effect = RuntimeError("stop")
        logger = self.controller.logger

        with patch.object(
            self.controller.vehicle_navigation,
            "dispatch_approach",
            side_effect=RuntimeError("dispatch"),
        ):
            with self.assertRaisesRegex(RuntimeError, "dispatch"):
                self._arm_peer_orbit()

        self.detector.stop_geo_tracking.assert_called_once()
        self.assertIsNone(self.controller.geo_hold.poi_location)
        logger.warning.assert_called()

    def test_setup_peer_navigation_clears_stale_cache_before_decision(self):
        """A prior ORBIT cycle's cached POI must not leak into a new
        non-ORBIT cycle. The clear happens at the START of setup so
        the gate decides cleanly.
        """
        self.controller.geo_hold.poi_location = Location(0.1, 0.2, 0.3, is_absolute=True)

        self._arm_peer_offset()

        self.assertIsNone(self.controller.geo_hold.poi_location)

    # --- per-tick update_geo --------------------------------------------

    def test_act_detect_calls_update_geo_when_peer_armed(self):
        self._arm_peer_orbit()
        self.detector.update_geo.reset_mock()

        self.controller.detect_action.act()

        # Must use absolute UAV location (location(False)) to pair
        # with the absolute cached POI — not location(True).
        self.detector.update_geo.assert_called_once()
        args = self.detector.update_geo.call_args.args
        self.assertIs(args[0], self.uav_loc_absolute)
        self.assertIs(args[1], self.vehicle.attitude)

    def test_act_detect_prepares_geo_zoom_when_in_frame_but_too_small(self):
        self._arm_peer_orbit()
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = True
        self.detector.mounts = [mount]
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        self.detector.update_geo.reset_mock()
        self.detector.prepare_geo_acquisition.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            self.controller.detect_action.act()

        self.detector.prepare_geo_acquisition.assert_called_once()
        args = self.detector.prepare_geo_acquisition.call_args.args
        self.assertIs(args[0], self.uav_loc_absolute)
        self.assertIs(args[1], self.vehicle.attitude)
        self.assertEqual(args[2], 0)

    def test_act_detect_rechecks_geo_zoom_after_poi_is_large_enough(self):
        self._arm_peer_orbit()
        mount = Mock()
        mount.get_k.return_value = np.diag([1000.0, 1000.0, 1.0])
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = True
        self.detector.mounts = [mount]
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        self.detector.prepare_geo_acquisition.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            self.controller.detect_action.act()

        self.detector.prepare_geo_acquisition.assert_called_once()

    def test_act_detect_rechecks_geo_zoom_to_recover_wider_candidate(self):
        self._arm_peer_orbit()
        mount = Mock()
        mount.get_k.return_value = np.diag([1000.0, 1000.0, 1.0])
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = False
        self.detector.mounts = [mount]
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        self.detector.prepare_geo_acquisition.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            self.controller.detect_action.act()

        self.detector.prepare_geo_acquisition.assert_called_once()

    def test_act_detect_rechecks_geo_zoom_when_current_view_is_out_of_frame(self):
        self._arm_peer_orbit()
        mount = Mock()
        mount.get_k.return_value = np.eye(3)
        mount.get_gimbal_data.return_value = Mock()
        mount.is_valid.return_value = False
        self.detector.mounts = [mount]
        self.navigation.legacy_pois.geo_ref.calc_uv.return_value = (10, 10)
        self.detector.prepare_geo_acquisition.reset_mock()

        with patch(
            "navpy.modules.nav.peer_geo.pymap3d.geodetic2ned",
            return_value=np.array([1.0, 2.0, -3.0]),
        ):
            self.controller.detect_action.act()

        self.detector.prepare_geo_acquisition.assert_called_once()

    def test_act_detect_skips_update_geo_when_detection_armed(self):
        self._arm_peer_orbit()
        self.detector.is_detection_armed = True
        self.detector.update_geo.reset_mock()

        self.controller.detect_action.act()

        self.detector.update_geo.assert_not_called()

    def test_act_detect_skips_update_geo_when_no_peer_poi_cached(self):
        """Self-discovery flow: peer-nav not active, _peer_poi_loc
        stays None, update_geo never fires.
        """
        # No peer arming.
        self.detector.update_geo.reset_mock()

        self.controller.detect_action.act()

        self.detector.update_geo.assert_not_called()

    def test_act_detect_uses_absolute_uav_location_not_relative(self):
        """Regression guard: location(True) returns relative alt and
        location(False) returns absolute MSL (per VehicleMav.location).
        update_geo must be called with the absolute one.
        """
        self._arm_peer_orbit()
        self.detector.update_geo.reset_mock()
        self.vehicle.location.reset_mock()

        self.controller.detect_action.act()

        # Must have called location(False) at some point during
        # _act_detect to obtain absolute MSL for pose math.
        called_args = [c.args[0] for c in self.vehicle.location.call_args_list]
        self.assertIn(False, called_args)
        # And the actual loc passed to update_geo must be the absolute one.
        passed_loc = self.detector.update_geo.call_args.args[0]
        self.assertTrue(passed_loc.is_absolute)

    def test_act_detect_skips_update_geo_when_uav_location_unavailable(self):
        """Defensive: vehicle.location can return None pre-GPS-fix."""
        self._arm_peer_orbit()
        self.vehicle.location = Mock(return_value=None)
        self.detector.update_geo.reset_mock()

        self.controller.detect_action.act()

        self.detector.update_geo.assert_not_called()

    # --- teardown --------------------------------------------------------

    def test_clear_state_stops_geo_and_clears_cache(self):
        self._arm_peer_orbit()
        self.detector.stop_geo_tracking.reset_mock()

        self.controller.reset.clear()

        self.detector.stop_geo_tracking.assert_called_once()
        self.assertIsNone(self.controller.geo_hold.poi_location)

    def test_resume_auto_mission_stops_geo_and_clears_cache(self):
        self._arm_peer_orbit()
        self.vehicle.mission_items_next = 5
        self.detector.stop_geo_tracking.reset_mock()

        self.controller.auto_resume.run()

        self.detector.stop_geo_tracking.assert_called_once()
        self.assertIsNone(self.controller.geo_hold.poi_location)


class TestNavControllerVisionNavConfirm(unittest.TestCase):
    """Part B: vision-nav confirm honors the Auto/Manual switch, and the
    final-approach record + wind freeze happen at the right moment for each mode."""

    def _vt_navigation(self, can_confirm=True, record=True):
        navigation = _create_mock_navigation()
        navigation.final_approach.is_active = True
        navigation.final_approach.can_confirm_detection = Mock(return_value=can_confirm)
        navigation.final_approach.record_confirmed_detection = Mock(return_value=record)
        return navigation

    def _confirm_controller(self, auto_confirm, can_confirm=True, record=True,
                            with_frame=False):
        # MANUAL falls through to the shared operator-review path, which needs a
        # real recognition image before it will ask the operator to authorize a
        # task. AUTO self-commits before that path is ever reached.
        if with_frame:
            poi = _create_detected_poi(
                obj_id=1,
                detection_frame=np.zeros((480, 640, 3), dtype=np.uint8),
                bbox=(320, 240, 100, 80),
            )
        else:
            poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = self._vt_navigation(can_confirm=can_confirm, record=record)
        args = _create_mock_args(auto_confirm=auto_confirm)
        controller = _create_controller(detector=detector, navigation=navigation, args=args)
        controller.confirmation_manager.set_active_poi(poi)
        # Isolate the branch decision from the downstream review/threading.
        controller.review.confirm_local = Mock()
        controller.review.request_operator_review = Mock()
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        return controller, navigation, poi

    def test_auto_confirm_records_and_confirms_locally(self):
        controller, navigation, _ = self._confirm_controller(auto_confirm=True)
        controller.confirmation_action.act()

        navigation.final_approach.can_confirm_detection.assert_called_once()
        navigation.final_approach.record_confirmed_detection.assert_called_once()
        controller.review.confirm_local.assert_called_once()
        controller.review.request_operator_review.assert_not_called()
        self.assertTrue(controller.final_approach.confirmed_recorded)

    def test_manual_requests_confirmation_and_defers_record(self):
        controller, navigation, _ = self._confirm_controller(
            auto_confirm=False, with_frame=True)
        controller.confirmation_action.act()

        # Flyability gate still runs, but the record/wind-freeze is deferred and
        # the operator confirmation request is sent instead of a local commit.
        navigation.final_approach.can_confirm_detection.assert_called_once()
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        controller.review.request_operator_review.assert_called_once()
        controller.review.confirm_local.assert_not_called()
        self.assertFalse(controller.final_approach.confirmed_recorded)

    def test_manual_review_poison_proves_final_approach_path_never_uses_poi_geo(self):
        """Default OFFSET final-approach review uses neither POI nor vehicle geo."""
        vehicle = _create_mock_vehicle()
        vehicle.location = Mock(
            side_effect=AssertionError("vehicle geo entered final-approach review"),
        )
        navigation = self._vt_navigation()
        navigation.legacy_pois.ground_location = Mock(
            side_effect=AssertionError("legacy POI geo entered final-approach review"),
        )
        navigation.vehicle_commands.peer_poi = Mock(
            side_effect=AssertionError("final-approach review commanded a geo POI"),
        )
        navigation.vehicle_commands.peer_poi_loiter = Mock(
            side_effect=AssertionError("final-approach review commanded a geo hold"),
        )
        controller = _create_controller(
            vehicle=vehicle,
            navigation=navigation,
            args=_create_mock_args(auto_confirm=False),
        )
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.review = Mock()

        with patch.object(
            type(poi),
            "set_p_t_g_loc",
            side_effect=AssertionError("final-approach review mutated POI geo"),
        ) as set_poi_geo:
            controller.review.request_operator_review(poi)

        navigation.legacy_pois.ground_location.assert_not_called()
        vehicle.location.assert_not_called()
        navigation.vehicle_commands.peer_poi.assert_not_called()
        navigation.vehicle_commands.peer_poi_loiter.assert_not_called()
        set_poi_geo.assert_not_called()
        self.assertIsNone(poi.geo.projected_poi_location)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        controller.confirmation_manager.review.assert_called_once_with([poi])

    def test_manual_waits_for_recognition_image_before_requesting(self):
        """A task is never put to the operator on a bare detection: without a
        recognition image there is nothing to review, so MANUAL keeps waiting
        rather than requesting authorization."""
        controller, navigation, _ = self._confirm_controller(auto_confirm=False)
        controller.confirmation_action.act()

        navigation.final_approach.can_confirm_detection.assert_called_once()
        navigation.final_approach.record_confirmed_detection.assert_not_called()
        controller.review.request_operator_review.assert_not_called()
        controller.review.confirm_local.assert_not_called()
        self.assertFalse(controller.final_approach.confirmed_recorded)

    def test_confirm_gate_blocks_both_modes_when_not_flyable(self):
        for auto in (True, False):
            controller, navigation, _ = self._confirm_controller(
                auto_confirm=auto, can_confirm=False)
            controller.confirmation_action.act()
            navigation.final_approach.record_confirmed_detection.assert_not_called()
            controller.review.confirm_local.assert_not_called()
            controller.review.request_operator_review.assert_not_called()
            self.assertFalse(controller.final_approach.confirmed_recorded)

    def _nav_controller(self, record=True, already_recorded=False):
        poi = _create_detected_poi(obj_id=1)
        detector = _create_mock_detector(detections=[poi])
        navigation = self._vt_navigation(record=record)
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        args = _create_mock_args(auto_confirm=False)
        controller = _create_controller(
            vehicle=vehicle, detector=detector, navigation=navigation, args=args)
        controller.confirmation_manager.set_active_poi(poi)
        controller.final_approach.confirmed_recorded = already_recorded
        controller.sensor.sense()
        controller.phase.current = NavState.NAV
        return controller, navigation, poi

    def test_manual_records_once_at_nav_entry(self):
        controller, navigation, _ = self._nav_controller(already_recorded=False)
        controller.final_approach_nav.act_nav()

        # Dive-commit records the final-approach observation + freezes wind, then navigates.
        navigation.final_approach.record_confirmed_detection.assert_called_once()
        self.assertTrue(controller.final_approach.confirmed_recorded)
        navigation.nav.assert_called_once()

        # Subsequent ticks must not re-record.
        navigation.final_approach.record_confirmed_detection.reset_mock()
        controller.sensor.sense()
        controller.final_approach_nav.act_nav()
        navigation.final_approach.record_confirmed_detection.assert_not_called()

    def test_auto_does_not_re_record_at_nav(self):
        controller, navigation, _ = self._nav_controller(already_recorded=True)
        controller.final_approach_nav.act_nav()

        navigation.final_approach.record_confirmed_detection.assert_not_called()
        navigation.nav.assert_called_once()

    def test_nav_waits_when_deferred_record_unavailable(self):
        controller, navigation, _ = self._nav_controller(
            record=False, already_recorded=False)
        controller.final_approach_nav.act_nav()

        # Record failed this tick → do not navigate yet, stay unrecorded, retry next.
        navigation.final_approach.record_confirmed_detection.assert_called_once()
        self.assertFalse(controller.final_approach.confirmed_recorded)
        navigation.nav.assert_not_called()
        # Timer started for the bounded wait.
        self.assertIsNotNone(controller.final_approach.deferred_record_started_at)

    def test_nav_aborts_when_deferred_record_stays_unavailable(self):
        controller, navigation, _ = self._nav_controller(
            record=False, already_recorded=False)
        # Simulate the bound elapsing: first attempt was long ago.
        controller.final_approach.deferred_record_started_at = controller.clock.decision_s() - 999.0
        controller.final_approach_nav.act_nav()

        # Bounded out → abort via navigation failure instead of loitering forever
        # in GUIDED with no final-approach command.
        self.assertTrue(controller.navigation_failures.failed)
        navigation.nav.assert_not_called()
        self.assertFalse(controller.final_approach.confirmed_recorded)


if __name__ == '__main__':
    unittest.main()
