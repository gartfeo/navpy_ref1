"""
Tests for CONF-03: the recognition-gate blocked-state DRONE-dest STATUSTEXT
(D-15/D-16/D-17, Task 1) and the "Ask me anyway" one-shot gate-bypass
override (D-18/D-19/D-20, Task 3).
"""
import unittest
from unittest.mock import Mock

import numpy as np

from navpy.args.logger_args import LogStatusDest
from navpy.modules.comm.messages.swarm_request_msg import (
    REQUEST_TYPE_FORCE_CONFIRM,
    REQUEST_TYPE_RESOURCE,
    SUBJECT_TYPE_THUMBNAIL,
    SwarmRequestMsg,
)
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.confirm_override_listener import ConfirmOverrideListener
from navpy.modules.nav.nav_controller import NavController, NavState
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.vision_profiles import MIN_CONFIRM_PIXELS
from tests.detection_factory import make_detected_target
from tests.modules.nav.nav_test_rig import create_nav_test_rig

# Pixel bbox well below MIN_CONFIRM_PIXELS (diagonal ~17px) -- blocks the
# pixel gate regardless of the class threshold. A generously-sized bbox
# (diagonal >> MIN_CONFIRM_PIXELS) is used wherever the pixel gate must pass.
_SMALL_BBOX = (320, 240, 12, 12)
_LARGE_BBOX = (320, 240, 100, 100)


def _create_mock_vehicle():
    vehicle = Mock()
    vehicle.attitude = Attitude(0, 0, 0)
    vehicle.home_location = Location(40.0, -74.0, 100.0)
    vehicle.mission_items_count = 10
    vehicle.mission_items_next = 5
    vehicle.source_system = 1
    vehicle.get_mode = FlightMode.AUTO
    vehicle.is_armed = True
    vehicle.location = Mock(return_value=Location(40.0, -74.0, 200.0))
    vehicle.get_param_or_default = Mock(side_effect=lambda name, default: default)
    vehicle.get_mission_item_location = Mock(return_value=None)
    vehicle.set_mode = Mock()
    vehicle.restart_mission = Mock()
    vehicle.max_pitch = 25.0
    vehicle.set_attitude = Mock()
    vehicle.is_simulated_autopilot = Mock(return_value=True)
    return vehicle


def _create_mock_detector(detections=None, is_zoom_stable=True):
    detector = Mock()
    detector.get_detect_data = Mock(
        return_value=DetectResponse(detections or [], primary_target=None),
    )
    detector.drain_detection_events = Mock(return_value=[])
    detector.open_detection_event_lease = Mock(return_value=None)
    detector.has_source_driven_detection_events = False
    detector.target_uses_source_driven_events = Mock(return_value=False)
    detector.is_simulation = True
    detector.is_zoom_stable = is_zoom_stable
    detector.get_zoom_result = Mock(return_value=None)
    detector.is_detection_armed = False
    detector.is_geo_armed = False
    detector.refresh = Mock()
    detector.set_sim_target = Mock()
    detector.freeze_terminal_zoom_at_min = Mock()
    detector.start_geo_tracking = Mock()
    detector.stop_geo_tracking = Mock()
    detector.start_tracking = Mock()
    detector.update_geo = Mock()
    detector.prepare_geo_acquisition = Mock(return_value=False)
    detector.mounts = []
    detector.loss_hold_sec = 0.5
    return detector


def _create_mock_navigation():
    navigation = Mock()
    navigation.reset = Mock(return_value=Mock(status=Mock(return_value="OK")))
    navigation.algorithm_info = ("l1", 1.0)
    navigation.init = Mock()
    navigation.nav = Mock(return_value=True)
    navigation.pause_terminate = Mock()
    navigation.terminal = Mock()
    navigation.terminal.is_active = False
    navigation.terminal.clear_source_discontinuity = Mock()
    navigation.terminal.can_confirm_detection = Mock(return_value=True)
    navigation.terminal.record_confirmed_detection = Mock(return_value=True)
    navigation.terminal.nav_without_detection = Mock(return_value=True)
    navigation.terminal.target_passed_override = Mock(return_value=None)
    navigation.terminal.last_measured_lateral_bearing_deg = Mock(return_value=None)
    navigation.legacy_targets = Mock()
    navigation.legacy_targets.locked_distance = Mock(return_value=1000.0)
    navigation.legacy_targets.ground_location = Mock(
        return_value=Location(40.001, -74.001, 0.0)
    )
    navigation.legacy_targets.geo_ref = Mock(name="geo_ref_sentinel")
    navigation.vehicle_commands = Mock()
    navigation.vehicle_commands.peer_target = Mock()
    navigation.vehicle_commands.peer_target_loiter = Mock()
    return navigation


def _create_mock_args(confirm_gate_timeout=15.0):
    args = Mock()
    args.min_wp = 3
    args.min_alt = 50.0
    args.confirm_wait_time_sec = 2.0
    args.confirm_gate_timeout_sec = confirm_gate_timeout
    args.is_auto_confirm = False
    args.is_confirm_on_fail = True
    args.nav_sim_speedup = 0.0
    args.is_oneshot = False
    args.refresh = Mock()
    return args


def _create_detected_target(obj_id=1, detection_frame=None, bbox=None, task_id=None):
    return make_detected_target(
        obj_id=obj_id,
        task_id=obj_id if task_id is None else task_id,
        x_error=0.0,
        y_error=0.0,
        detection_frame=detection_frame,
        bbox_cxcywh=bbox,
        tracking_bbox_cxcywh=bbox,
        confidence=0.95,
        class_id=0,
        t_g_loc_debug=Location(40.001, -74.001, 0.0),
    )


def _create_controller(detector=None, navigation=None, args=None, logger=None):
    return create_nav_test_rig(
        _create_mock_vehicle(),
        detector or _create_mock_detector(),
        navigation or _create_mock_navigation(),
        args or _create_mock_args(),
        logger or Mock(),
        ApproachKind.OFFSET,
    )


def _drone_dest_texts(logger):
    """Extract every DRONE-dest status text logged via logger.info/warning."""
    texts = []
    for method in (logger.info, logger.warning):
        for call in method.call_args_list:
            args, kwargs = call
            if kwargs.get('dest') == LogStatusDest.DRONE:
                texts.append(args[0] if args else kwargs.get('msg'))
    return texts


class ConfirmBlockedStatusTests(unittest.TestCase):
    """Task 1: CONFIRM_BLOCKED:<reason>|<detail>|<task_id> DRONE-dest
    STATUSTEXT emitted live while the pixel/zoom gate blocks (D-15/D-16/D-17)."""

    def test_pixels_blocked_emits_reason_with_source_and_min(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=1, detection_frame=frame, bbox=_SMALL_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)

        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        blocked = [t for t in texts if t.startswith("CONFIRM_BLOCKED:pixels|")]
        self.assertEqual(len(blocked), 1, texts)
        self.assertTrue(blocked[0].endswith("|1"), blocked[0])  # task_id=1 rides the status
        self.assertIsNone(controller.confirmation_manager.get_status(target))

    def test_zoom_not_stable_emits_zoom_reason(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=2, detection_frame=frame, bbox=_LARGE_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=False)
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)

        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        self.assertIn("CONFIRM_BLOCKED:zoom||2", texts)

    def test_blocked_state_clears_when_gate_passes(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=1, detection_frame=frame, bbox=_LARGE_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)
        controller.blocked.reason = 'pixels'
        controller.blocked.target_id = 1

        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        self.assertIn("CONFIRM_BLOCKED:clear", texts)
        self.assertIsNone(controller.blocked.reason)

    def test_blocked_state_clears_when_target_lost(self):
        target = _create_detected_target(obj_id=1, detection_frame=None)
        detector = _create_mock_detector(detections=[])  # no fresh detection this tick
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)
        controller.confirmation_manager.set_active_target(target)
        controller.blocked.reason = 'pixels'
        controller.blocked.target_id = 1

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        self.assertIn("CONFIRM_BLOCKED:clear", texts)
        self.assertIsNone(controller.blocked.reason)

    def test_blocked_status_rate_limited_across_repeated_ticks(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=1, detection_frame=frame, bbox=_SMALL_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)
        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM

        controller.confirmation_action.act()
        controller.confirmation_action.act()
        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        blocked = [t for t in texts if t.startswith("CONFIRM_BLOCKED:pixels|")]
        self.assertEqual(len(blocked), 1, "rapid repeated ticks must be rate-limited, not spammed")

    def test_blocked_status_not_emitted_when_gate_passes_cleanly(self):
        """A target that never blocks must never emit CONFIRM_BLOCKED."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=1, detection_frame=frame, bbox=_LARGE_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        logger = Mock()
        controller = _create_controller(detector=detector, logger=logger)
        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM

        controller.confirmation_action.act()

        texts = _drone_dest_texts(logger)
        self.assertFalse(any(t.startswith("CONFIRM_BLOCKED:") for t in texts))


class ForceConfirmOverrideTests(unittest.TestCase):
    """Task 3: "Ask me anyway" one-shot gate bypass (D-18/D-19/D-20)."""

    def test_forced_target_bypasses_pixel_gate_and_marks_degraded(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=5, detection_frame=frame, bbox=_SMALL_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        logger = Mock()
        controller = _create_controller(detector=detector, navigation=navigation, logger=logger)
        controller.confirmation_manager.set_active_target(target)
        controller.application.force_confirm_override(5)

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        self.assertTrue(target.confirmation.degraded)
        navigation.vehicle_commands.peer_target_loiter.assert_called_once()
        # review() ran synchronously up to update_status(CONFIRMING); the
        # background worker (no network -> auto-resolves) may have already
        # advanced it further by the time we check, so assert only that a
        # review round actually started (status left None).
        self.assertIsNotNone(controller.confirmation_manager.get_status(target))

    def test_forced_target_bypasses_zoom_gate(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=9, detection_frame=frame, bbox=_LARGE_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=False)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.confirmation_manager.set_active_target(target)
        controller.application.force_confirm_override(9)

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        self.assertTrue(target.confirmation.degraded)
        navigation.vehicle_commands.peer_target_loiter.assert_called_once()

    def test_override_is_one_shot_not_reused(self):
        """A second blocked tick for the same target must NOT auto-request
        again once the one-shot force flag has been consumed."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=7, detection_frame=frame, bbox=_SMALL_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.confirmation_manager.set_active_target(target)
        controller.application.force_confirm_override(7)
        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM

        controller.confirmation_action.act()

        self.assertFalse(controller.overrides.contains(7))
        navigation.vehicle_commands.peer_target_loiter.assert_called_once()

        # Fresh navigation task for the same target_id; gate still blocking; the
        # one-shot force flag was already consumed -- must stay blocked.
        controller.confirmation_manager.reset()
        controller.confirmation_manager.set_active_target(target)
        controller.sensor.sense()

        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_target_loiter.assert_called_once()  # still just the one call
        self.assertIsNone(controller.confirmation_manager.get_status(target))

    def test_override_does_not_fire_when_gate_would_not_block(self):
        """A force flag set for a target whose gate already passes is simply
        unused -- no double-request, no crash."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        target = _create_detected_target(obj_id=3, detection_frame=frame, bbox=_LARGE_BBOX)
        detector = _create_mock_detector(detections=[target], is_zoom_stable=True)
        navigation = _create_mock_navigation()
        controller = _create_controller(detector=detector, navigation=navigation)
        controller.confirmation_manager.set_active_target(target)
        controller.application.force_confirm_override(3)

        controller.sensor.sense()
        controller.phase.current = NavState.CONFIRM
        controller.confirmation_action.act()

        navigation.vehicle_commands.peer_target_loiter.assert_called_once()
        self.assertFalse(target.confirmation.degraded)


class SetNetworkRegistersOverrideListenerTests(unittest.TestCase):
    def test_set_network_registers_confirm_override_listener(self):
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()

        controller.application.set_network(network)

        registered = [call.args[0] for call in network.set_listener.call_args_list]
        self.assertTrue(
            any(isinstance(listener, ConfirmOverrideListener) for listener in registered),
        )

    def test_force_confirm_override_via_registered_listener(self):
        """End-to-end: a SWARM_REQUEST(FORCE_CONFIRM) delivered through the
        listener registered by set_network reaches NavController's force set."""
        controller = _create_controller()
        self.addCleanup(controller.application.stop)
        network = Mock()
        controller.application.set_network(network)
        listener = next(
            call.args[0] for call in network.set_listener.call_args_list
            if isinstance(call.args[0], ConfirmOverrideListener)
        )

        msg = SwarmRequestMsg(
            sender_id=0, receiver_id=controller.vehicle.source_system,
            request_type=REQUEST_TYPE_FORCE_CONFIRM, subject_type=0, subject_id=42,
        )
        listener.on_message(msg)

        self.assertTrue(controller.overrides.contains(42))


class ConfirmOverrideListenerTests(unittest.TestCase):
    """Unit tests for the listener's narrow callback boundary."""

    def test_dispatches_force_confirm_to_callback(self):
        request_override = Mock()
        listener = ConfirmOverrideListener(101, request_override, Mock())
        msg = SwarmRequestMsg(
            sender_id=0, receiver_id=101,
            request_type=REQUEST_TYPE_FORCE_CONFIRM, subject_type=0, subject_id=42,
        )

        listener.on_message(msg)

        request_override.assert_called_once_with(42)

    def test_ignores_message_addressed_to_a_different_drone(self):
        request_override = Mock()
        listener = ConfirmOverrideListener(101, request_override, Mock())
        msg = SwarmRequestMsg(
            sender_id=0, receiver_id=202,  # a different companion's wire id
            request_type=REQUEST_TYPE_FORCE_CONFIRM, subject_type=0, subject_id=42,
        )

        listener.on_message(msg)

        request_override.assert_not_called()

    def test_ignores_non_force_confirm_request_types(self):
        request_override = Mock()
        listener = ConfirmOverrideListener(101, request_override, Mock())
        msg = SwarmRequestMsg(
            sender_id=0, receiver_id=101,
            request_type=REQUEST_TYPE_RESOURCE, subject_type=SUBJECT_TYPE_THUMBNAIL,
            subject_id=42,
        )

        listener.on_message(msg)

        request_override.assert_not_called()


if __name__ == "__main__":
    unittest.main()
