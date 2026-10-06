"""
Tests for the auto-confirm-on-timeout freshness gate (MISS-04, D-21/D-22,
Phase 2 Plan 02-06 Task 1).

Confirm-on-fail is config-driven (AAS_NAV_CM_FL / is_confirm_on_fail); the
3-UAV demo runs CONFIRM so a lost operator response does not permanently
strand a UAV. But an auto-confirm on timeout must fire ONLY when the target
is genuinely detected fresh at that instant -- never from a cached status.
This file covers two layers:

  1. ConfirmationManager.get_status_on_fail() / the injected freshness callback
     in isolation (fail-closed default, exception safety, policy
     interaction).
  2. NavController._is_target_fresh_for_confirm() -- the real callback
     NavController wires into ConfirmationManager, keyed off DetectedObject's
     frame timestamps measured in the producer's FRAME CLOCK domain, using
     timestamp_now_s when needed and unscaled wall time otherwise,
     and the end-to-end wiring at construction time.
"""
import time
import unittest
from dataclasses import replace
from unittest.mock import MagicMock, Mock

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_controller import (
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
    NavController,
)
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from navpy.modules.vision.models.detect_response import DetectResponse
from tests.detection_factory import make_detected_target
from tests.modules.nav.nav_test_rig import create_nav_test_rig


# =============================================================================
# Part 1: ConfirmationManager.get_status_on_fail() / freshness callback in isolation
# =============================================================================

class GetStatusOnFailFreshnessGateTests(unittest.TestCase):
    """Direct unit tests of get_status_on_fail()'s freshness gating, with a
    manually-injected freshness callback (isolated from NavController)."""

    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 0.1

        self.confirmation_manager = ConfirmationManager(
            sys_id=1, args=self.mock_args, logger=self.mock_logger,
        )

    def test_confirm_policy_and_fresh_resolves_confirmed(self):
        """Timeout + CONFIRM policy + fresh callback True -> CONFIRMED."""
        self.mock_args.is_confirm_on_fail = True
        self.confirmation_manager.set_target_freshness_check(lambda target_id: True)

        self.assertEqual(
            self.confirmation_manager.get_status_on_fail(target_id=42),
            ConfirmationStatus.CONFIRMED,
        )

    def test_confirm_policy_and_not_fresh_resolves_timeout_rejected(self):
        """Timeout + CONFIRM policy + fresh callback False -> TIMEOUT_REJECTED
        (not CONFIRMED) -- the Pitfall 5 guard."""
        self.mock_args.is_confirm_on_fail = True
        self.confirmation_manager.set_target_freshness_check(lambda target_id: False)

        self.assertEqual(
            self.confirmation_manager.get_status_on_fail(target_id=42),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

    def test_reject_policy_resolves_timeout_rejected_regardless_of_freshness(self):
        """Timeout + REJECT policy -> TIMEOUT_REJECTED even when the target
        IS fresh -- policy governs, freshness only gates the CONFIRM path."""
        self.mock_args.is_confirm_on_fail = False
        self.confirmation_manager.set_target_freshness_check(lambda target_id: True)

        self.assertEqual(
            self.confirmation_manager.get_status_on_fail(target_id=42),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

    def test_no_freshness_callback_injected_fails_closed(self):
        """A ConfirmationManager that never had set_target_freshness_check()
        called must NOT auto-confirm on timeout even under CONFIRM policy
        -- directly guarding Pitfall 5 (an un-wired freshness signal must
        never be interpreted as 'fresh')."""
        self.mock_args.is_confirm_on_fail = True
        # No set_target_freshness_check() call -- uses the default.

        self.assertEqual(
            self.confirmation_manager.get_status_on_fail(target_id=42),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

    def test_none_target_id_fails_closed(self):
        """A None target_id (the unresolved-identity edge case in
        _request_confirmation_from_network) must not be treated as fresh."""
        self.mock_args.is_confirm_on_fail = True
        self.confirmation_manager.set_target_freshness_check(lambda target_id: True)

        self.assertEqual(
            self.confirmation_manager.get_status_on_fail(target_id=None),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

    def test_freshness_callback_receives_correct_target_id(self):
        """The injected callback is called with the exact target_id being
        resolved."""
        self.mock_args.is_confirm_on_fail = True
        received = []

        def _check(target_id):
            received.append(target_id)
            return True

        self.confirmation_manager.set_target_freshness_check(_check)
        self.confirmation_manager.get_status_on_fail(target_id=777)

        self.assertEqual(received, [777])

    def test_freshness_callback_operational_error_fails_closed(self):
        """An unavailable detector chain fails closed without auto-confirm."""
        self.mock_args.is_confirm_on_fail = True

        def _raising_check(target_id):
            raise OSError("detector chain unavailable")

        self.confirmation_manager.set_target_freshness_check(_raising_check)

        status = self.confirmation_manager.get_status_on_fail(target_id=42)

        self.assertEqual(status, ConfirmationStatus.TIMEOUT_REJECTED)
        self.mock_logger.error.assert_called()


class RequestConfirmationTimeoutIntegrationTests(unittest.TestCase):
    """End-to-end through review() -> _request_confirmation_from_network()'s
    real wait-timeout branch, exercising the SAME code path production runs
    through (not a direct get_status_on_fail() call)."""

    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 0.05
        self.mock_args.is_confirm_on_fail = True

        self.mock_network = MagicMock(spec=NetworkAbc)
        self.confirmation_manager = ConfirmationManager(
            sys_id=1, args=self.mock_args, logger=self.mock_logger,
        )
        self.confirmation_manager.set_network(self.mock_network)

        self.target = make_detected_target(
            obj_id=901, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.target.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))

    def test_timeout_with_fresh_callback_true_confirms(self):
        self.confirmation_manager.set_target_freshness_check(lambda target_id: True)
        self.confirmation_manager.review([self.target])
        time.sleep(0.2)

        self.assertEqual(
            self.confirmation_manager.get_status(self.target), ConfirmationStatus.CONFIRMED,
        )

    def test_timeout_with_fresh_callback_false_times_out_rejected(self):
        self.confirmation_manager.set_target_freshness_check(lambda target_id: False)
        self.confirmation_manager.review([self.target])
        time.sleep(0.2)

        self.assertEqual(
            self.confirmation_manager.get_status(self.target), ConfirmationStatus.TIMEOUT_REJECTED,
        )


# =============================================================================
# Part 2: NavController._is_target_fresh_for_confirm() -- real freshness
# signal, keyed off DetectedObject's frame timestamps measured on the
# detection's own frame clock (timestamp_now_s provider; _now_s() fallback).
# =============================================================================

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


def _create_mock_detector(detections=None):
    detector = Mock()
    detector.get_detect_data = Mock(
        return_value=DetectResponse(detections or [], primary_target=None),
    )
    detector.drain_detection_events = Mock(return_value=[])
    detector.open_detection_event_lease = Mock(return_value=None)
    detector.has_source_driven_detection_events = False
    detector.target_uses_source_driven_events = Mock(return_value=False)
    detector.is_simulation = True
    detector.is_zoom_stable = True
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


def _create_mock_args():
    args = Mock()
    args.min_wp = 3
    args.min_alt = 50.0
    args.confirm_wait_time_sec = 2.0
    args.confirm_gate_timeout_sec = 15.0
    args.is_auto_confirm = False
    args.is_confirm_on_fail = True
    args.nav_sim_speedup = 0.0
    args.is_oneshot = False
    args.refresh = Mock()
    return args


def _create_controller(detector=None, navigation=None, args=None, logger=None,
                       scheduler_cadence=None):
    return create_nav_test_rig(
        _create_mock_vehicle(),
        detector or _create_mock_detector(),
        navigation or _create_mock_navigation(),
        args or _create_mock_args(),
        logger or Mock(),
        ApproachKind.OFFSET,
        scheduler_cadence=scheduler_cadence,
    )


def _create_detected_target(obj_id=1, task_id=None, camera_frame_timestamp_s=None,
                            tracker_timestamp_s=None):
    return make_detected_target(
        obj_id=obj_id,
        task_id=obj_id if task_id is None else task_id,
        camera_frame_timestamp_s=camera_frame_timestamp_s,
        tracker_timestamp_s=tracker_timestamp_s,
    )


class IsTargetFreshForConfirmTests(unittest.TestCase):
    """Direct unit tests of NavController._is_target_fresh_for_confirm()."""

    def test_fresh_camera_frame_timestamp_returns_true(self):
        controller = _create_controller()
        now = time.time()
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=now - 0.1,
        )
        controller.detections.detected_targets = [target]

        self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))

    def test_stale_camera_frame_timestamp_returns_false(self):
        controller = _create_controller()
        now = time.time()
        target = _create_detected_target(
            obj_id=55,
            camera_frame_timestamp_s=now - (CONFIRM_FRESH_DETECTION_MAX_AGE_S + 5.0),
        )
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(55))

    def test_exactly_at_boundary_is_fresh(self):
        """age == CONFIRM_FRESH_DETECTION_MAX_AGE_S is inclusive (<=).

        Pins time.time() so the comparison lands EXACTLY at the boundary --
        a live time.time() call here would drift the age past the boundary
        by however long test setup takes, making this flaky.
        """
        from unittest.mock import patch

        controller = _create_controller()
        fixed_now = 1_000_000.0
        target = _create_detected_target(
            obj_id=55,
            camera_frame_timestamp_s=fixed_now - CONFIRM_FRESH_DETECTION_MAX_AGE_S,
        )
        controller.detections.detected_targets = [target]

        with patch(
            "navpy.modules.nav.nav_clock.time.time", return_value=fixed_now,
        ):
            self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))

    def test_falls_back_to_tracker_timestamp_when_camera_frame_missing(self):
        controller = _create_controller()
        now = time.time()
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=None, tracker_timestamp_s=now - 0.1,
        )
        controller.detections.detected_targets = [target]

        self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))

    def test_no_matching_detection_returns_false(self):
        controller = _create_controller()
        controller.detections.detected_targets = []

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(55))

    def test_mismatched_target_id_returns_false(self):
        controller = _create_controller()
        target = _create_detected_target(obj_id=55, camera_frame_timestamp_s=time.time())
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(99))

    def test_no_timestamp_available_returns_false(self):
        controller = _create_controller()
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=None, tracker_timestamp_s=None,
        )
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(55))


class FreshnessGateClockDomainTests(unittest.TestCase):
    """Freshness uses the producer clock without scaling timestamps.

    Polling and hardware detections use wall time regardless of SIM_SPEEDUP.
    A source-driven producer may use a different epoch, such as ArduPilot boot
    time, and then carries ``timestamp_now_s`` so age remains like-for-like.
    """

    @staticmethod
    def _make_timebase(sim_now):
        timebase = Mock()
        timebase.now = Mock(return_value=sim_now)
        return timebase

    def test_wall_stamped_stale_frame_is_not_fresh_with_timebase(self):
        wall_now = time.time()
        timebase = self._make_timebase(wall_now + 100.0)
        controller = _create_controller(scheduler_cadence=timebase)
        target = _create_detected_target(
            obj_id=55,
            camera_frame_timestamp_s=(
                wall_now - CONFIRM_FRESH_DETECTION_MAX_AGE_S - 5.0
            ),
        )
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(55))
        timebase.now.assert_not_called()

    def test_wall_stamped_fresh_frame_ignores_timebase_clock(self):
        wall_now = time.time()
        timebase = self._make_timebase(wall_now + 100.0)
        controller = _create_controller(scheduler_cadence=timebase)
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=wall_now - 0.1,
        )
        controller.detections.detected_targets = [target]

        self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))
        timebase.now.assert_not_called()

    def test_no_timebase_keeps_wall_clock_domain(self):
        """Without a timebase (real detector path) the gate still measures
        against time.time() — the domain real frames are stamped with."""
        controller = _create_controller(scheduler_cadence=None)
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=time.time() - 0.1,
        )
        controller.detections.detected_targets = [target]

        self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))

    def test_detection_own_clock_provider_wins_over_missing_timebase(self):
        """A source such as ArduPilot boot time can use a non-wall epoch.

        The detection's own provider governs without converting or scaling it.
        """
        sim_now = time.time() + 100.0
        controller = _create_controller(scheduler_cadence=None)
        target = _create_detected_target(
            obj_id=55,
            camera_frame_timestamp_s=sim_now - (CONFIRM_FRESH_DETECTION_MAX_AGE_S + 5.0),
        )
        target.replace_timing(replace(target.timing, detection_now_s=lambda: sim_now))
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.freshness.is_target_fresh_for_confirm(55))

    def test_wall_stamped_detection_fresh_despite_controller_timebase(self):
        """An explicit wall-clock provider remains independent of cadence."""
        controller = _create_controller(
            scheduler_cadence=self._make_timebase(time.time() + 100.0),
        )
        target = _create_detected_target(
            obj_id=55, camera_frame_timestamp_s=time.time() - 0.1,
        )
        target.replace_timing(replace(target.timing, detection_now_s=time.time))
        controller.detections.detected_targets = [target]

        self.assertTrue(controller.freshness.is_target_fresh_for_confirm(55))


class NavControllerWiresFreshnessCallbackTests(unittest.TestCase):
    """Construction-time integration: NavController must inject its own
    _is_target_fresh_for_confirm into the ConfirmationManager it owns, and the
    live end-to-end timeout path must honor it."""

    def test_confirmation_manager_freshness_callback_is_controller_bound(self):
        controller = _create_controller()

        target = _create_detected_target(obj_id=321, camera_frame_timestamp_s=time.time())
        controller.detections.detected_targets = [target]

        self.assertTrue(
            controller.confirmation_manager._freshness.is_fresh(321),
            "ConfirmationManager's injected freshness callback must be backed by "
            "the owning NavController's _last_detections, not a stub",
        )

    def test_confirmation_manager_freshness_callback_reflects_stale_detections(self):
        controller = _create_controller()
        now = time.time()
        target = _create_detected_target(
            obj_id=321,
            camera_frame_timestamp_s=now - (CONFIRM_FRESH_DETECTION_MAX_AGE_S + 5.0),
        )
        controller.detections.detected_targets = [target]

        self.assertFalse(controller.confirmation_manager._freshness.is_fresh(321))

    def test_end_to_end_fresh_detection_at_timeout_confirms(self):
        """A target still fresh in _last_detections at the moment
        confirm_wait_time_sec elapses resolves CONFIRMED."""
        args = _create_mock_args()
        args.confirm_wait_time_sec = 0.1
        controller = _create_controller(args=args)
        self.addCleanup(controller.application.stop)
        network = Mock()
        controller.application.set_network(network)

        target = make_detected_target(
            obj_id=88, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        target.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        # Keep the detection fresh in controller state throughout the wait.
        controller.detections.detected_targets = [target]

        controller.confirmation_manager.review([target])
        time.sleep(0.05)
        # Refresh the timestamp right before the window elapses so the
        # detection is genuinely "fresh right now" at resolution time.
        target.replace_timing(replace(
            target.timing,
            camera_frame_timestamp_s=time.time(),
        ))
        time.sleep(0.15)

        self.assertEqual(
            controller.confirmation_manager.get_status(target), ConfirmationStatus.CONFIRMED,
        )

    def test_end_to_end_lost_detection_at_timeout_rejects_without_confirmation(self):
        """A target that disappears from _last_detections before the window
        elapses resolves TIMEOUT_REJECTED (no confirmation), not CONFIRMED."""
        args = _create_mock_args()
        args.confirm_wait_time_sec = 0.1
        controller = _create_controller(args=args)
        self.addCleanup(controller.application.stop)
        network = Mock()
        controller.application.set_network(network)

        target = make_detected_target(
            obj_id=89, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        target.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))
        controller.detections.detected_targets = []  # never detected via the controller

        controller.confirmation_manager.review([target])
        time.sleep(0.2)

        self.assertEqual(
            controller.confirmation_manager.get_status(target), ConfirmationStatus.TIMEOUT_REJECTED,
        )


if __name__ == '__main__':
    unittest.main()
