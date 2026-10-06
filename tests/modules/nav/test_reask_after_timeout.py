"""
Tests for the bounded re-ask after timeout (D-14, Phase 2 Plan 02-06 Task 2):
TIMEOUT_REJECTED distinguishes a
system/timeout-origin rejection from an operator's explicit REJECTED, so a
reacquired POI can start a fresh, bounded confirm round without EVER
reopening an operator's decision.

Covers:
  - ConfirmationManager.clear_status() in isolation.
  - NavController._can_reask()/_begin_reask() budget bookkeeping.
  - _select_poi()/_handle_new_poi() bounded reopen of a
    TIMEOUT_REJECTED POI, and permanent skip of an operator REJECTED
    POI.
  - _decide() tearing down both REJECTED and TIMEOUT_REJECTED to DETECT
    immediately, and reask-attempt-counter reset semantics (CONFIRMED /
    operator REJECTED reset it, TIMEOUT_REJECTED does not).
  - The _handle_task_confirm_response monotonic guard still holding after a
    re-ask round.
  - A full multi-round integration: bounded re-ask up to
    CONFIRM_REASK_MAX_ATTEMPTS, then permanently rejected.
"""
import unittest
from unittest.mock import MagicMock, Mock

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.available_task_msg import TaskConfirmResponseMsg
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_controller import (
    CONFIRM_REASK_MAX_ATTEMPTS,
    NavController,
    NavState,
)
from navpy.modules.nav.confirmation_manager import ConfirmationManager, ConfirmationStatus
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from tests.detection_factory import make_detected_poi
from navpy.modules.vision.models.detect_response import DetectResponse
from tests.modules.nav.nav_test_rig import create_nav_test_rig


# =============================================================================
# Part 1: ConfirmationManager.clear_status() in isolation
# =============================================================================

class ClearStatusTests(unittest.TestCase):
    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.confirm_wait_time_sec = 0.1
        self.confirmation_manager = ConfirmationManager(
            sys_id=1, args=self.mock_args, logger=self.mock_logger,
        )
        self.poi = make_detected_poi(
            obj_id=11, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )

    def test_clear_status_removes_entry(self):
        self.confirmation_manager.update_status(self.poi, ConfirmationStatus.TIMEOUT_REJECTED)
        self.assertEqual(
            self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED,
        )

        self.confirmation_manager.clear_status(self.poi)

        self.assertIsNone(self.confirmation_manager.get_status(self.poi))

    def test_clear_status_noop_for_unknown_poi(self):
        """Clearing a POI with no status entry must not raise."""
        self.confirmation_manager.clear_status(self.poi)
        self.assertIsNone(self.confirmation_manager.get_status(self.poi))

    def test_timeout_rejected_and_rejected_are_distinct_values(self):
        self.assertNotEqual(ConfirmationStatus.TIMEOUT_REJECTED, ConfirmationStatus.REJECTED)


class MonotonicGuardSurvivesReaskTests(unittest.TestCase):
    """The response-handler monotonic guard must still protect a POI
    after a re-ask round: a late response with no live token must not flip
    a resolved (even re-resolved) status."""

    def setUp(self):
        self.mock_logger = MagicMock(spec=ILogger)
        self.mock_args = MagicMock(spec=NavArgs)
        self.mock_args.is_auto_confirm = False
        self.mock_args.is_confirm_on_fail = False  # REJECT policy: deterministic timeout
        self.mock_args.confirm_wait_time_sec = 0.03
        self.mock_network = MagicMock(spec=NetworkAbc)
        self.confirmation_manager = ConfirmationManager(
            sys_id=1, args=self.mock_args, logger=self.mock_logger,
        )
        self.confirmation_manager.set_network(self.mock_network)
        self.poi = make_detected_poi(
            obj_id=12, size_class=DetectionSizeClass.S,
            x_error=0, y_error=0, reference_height_m=0, k=0,
            g_data=None, uas_att=None,
        )
        self.poi.set_p_t_g_loc(Location(lat=10, lng=20, alt=100))

    def test_late_response_after_reask_round_does_not_resurrect(self):
        from time import sleep

        # Round 1: times out -> TIMEOUT_REJECTED.
        self.confirmation_manager.review([self.poi])
        sleep(0.12)
        self.assertEqual(
            self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED,
        )

        # Reopen for a bounded re-ask round (mirrors NavController._begin_reask).
        self.confirmation_manager.clear_status(self.poi)
        self.confirmation_manager.review([self.poi])
        sleep(0.12)
        self.assertEqual(
            self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED,
            "Round 2 also times out under REJECT policy.",
        )

        # A late operator approve now arrives for the (long-resolved) task_id.
        self.confirmation_manager.on_message(TaskConfirmResponseMsg(
            receiver_id=1,
            task_id=self.poi.identity.obj_id,
            is_confirmed=True,
        ))

        self.assertEqual(
            self.confirmation_manager.get_status(self.poi), ConfirmationStatus.TIMEOUT_REJECTED,
            "A late response after a re-ask round must not resurrect the POI.",
        )
        self.assertNotIn(
            self.poi.identity.obj_id,
            self.confirmation_manager._state.pending_events(),
        )


# =============================================================================
# Part 2: NavController-level -- _can_reask/_begin_reask, _select_poi,
# _handle_new_poi, _decide.
# =============================================================================

def _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5, alt=200.0, is_armed=True):
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
    vehicle.is_simulated_autopilot = Mock(return_value=True)
    return vehicle


def _create_mock_detector(detections=None, primary_poi=None):
    detector = Mock()
    detector.get_detect_data = Mock(
        return_value=DetectResponse(detections or [], primary_poi=primary_poi),
    )
    detector.drain_detection_events = Mock(return_value=[])
    detector.open_detection_event_lease = Mock(return_value=None)
    detector.has_source_driven_detection_events = False
    detector.poi_uses_source_driven_events = Mock(return_value=False)
    detector.is_simulation = True
    detector.is_zoom_stable = True
    detector.get_zoom_result = Mock(return_value=None)
    detector.is_detection_armed = False
    detector.is_geo_armed = False
    detector.refresh = Mock()
    detector.set_sim_poi = Mock()
    detector.freeze_final_approach_zoom_at_min = Mock()
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
    navigation.pause_final_approach = Mock()
    navigation.final_approach = Mock()
    navigation.final_approach.is_active = False
    navigation.final_approach.clear_source_discontinuity = Mock()
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
    navigation.legacy_pois.geo_ref = Mock(name="geo_ref_sentinel")
    navigation.vehicle_commands = Mock()
    navigation.vehicle_commands.peer_poi = Mock()
    navigation.vehicle_commands.peer_poi_loiter = Mock()
    return navigation


def _create_mock_args(confirm_wait=2.0, auto_confirm=False, confirm_on_fail=True,
                      is_oneshot=False, confirm_gate_timeout=15.0):
    args = Mock()
    args.min_wp = 3
    args.min_alt = 50.0
    args.confirm_wait_time_sec = confirm_wait
    args.confirm_gate_timeout_sec = confirm_gate_timeout
    args.is_auto_confirm = auto_confirm
    args.is_confirm_on_fail = confirm_on_fail
    args.nav_sim_speedup = 0.0
    args.is_oneshot = is_oneshot
    args.refresh = Mock()
    return args


def _create_detected_poi(obj_id=1, task_id=None):
    return make_detected_poi(
        obj_id=obj_id,
        task_id=obj_id if task_id is None else task_id,
        x_error=0.0,
        y_error=0.0,
        confidence=0.95,
        class_id=0,
        t_g_loc_debug=Location(40.001, -74.001, 0.0),
    )


def _create_controller(vehicle=None, detector=None, navigation=None, args=None, logger=None):
    return create_nav_test_rig(
        vehicle or _create_mock_vehicle(),
        detector or _create_mock_detector(),
        navigation or _create_mock_navigation(),
        args or _create_mock_args(),
        logger or Mock(),
        ApproachKind.OFFSET,
    )


class CanReaskBeginReaskTests(unittest.TestCase):
    def test_can_reask_true_with_no_prior_attempts(self):
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)
        self.assertTrue(controller.retry_policy.can_reask(poi))

    def test_can_reask_false_once_budget_exhausted(self):
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)
        controller.retry_state.reask_attempts[1] = CONFIRM_REASK_MAX_ATTEMPTS

        self.assertFalse(controller.retry_policy.can_reask(poi))

    def test_can_reask_true_just_under_budget(self):
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)
        controller.retry_state.reask_attempts[1] = CONFIRM_REASK_MAX_ATTEMPTS - 1

        self.assertTrue(controller.retry_policy.can_reask(poi))

    def test_begin_reask_increments_attempts(self):
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)

        controller.retry_policy.begin_reask(poi)
        self.assertEqual(controller.retry_state.reask_attempts.get(1), 1)

        controller.retry_policy.begin_reask(poi)
        self.assertEqual(controller.retry_state.reask_attempts.get(1), 2)

    def test_begin_reask_clears_confirmation_manager_status(self):
        controller = _create_controller()
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)

        controller.retry_policy.begin_reask(poi)

        self.assertIsNone(controller.confirmation_manager.get_status(poi))


class SelectPoiReaskTests(unittest.TestCase):
    def test_timeout_rejected_poi_selected_when_under_budget(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)

        self_poi, peers = controller.selector.select([poi])

        self.assertIsNotNone(self_poi)
        self.assertEqual(self_poi.identity.obj_id, 10)

    def test_timeout_rejected_poi_skipped_when_budget_exhausted(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.network.task_actor = Mock()
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.retry_state.reask_attempts[10] = CONFIRM_REASK_MAX_ATTEMPTS

        self_poi, peers = controller.selector.select([poi])

        self.assertIsNone(self_poi)
        self.assertIn(poi, peers)

    def test_operator_rejected_poi_never_selected_even_with_full_budget(self):
        """An operator REJECTED POI must stay permanently skipped -- it
        never reaches the TIMEOUT_REJECTED reask-budget branch at all."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        # Budget is fully available (never consumed) -- must still be skipped.
        self.assertNotIn(10, controller.retry_state.reask_attempts)

        self_poi, peers = controller.selector.select([poi])

        self.assertIsNone(self_poi)

    def test_new_poi_returned_when_exhausted_timeout_rejected_also_present(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.network.task_actor = Mock()
        exhausted = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(exhausted, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.retry_state.reask_attempts[10] = CONFIRM_REASK_MAX_ATTEMPTS
        fresh = _create_detected_poi(obj_id=20)

        self_poi, peers = controller.selector.select([exhausted, fresh])

        self.assertEqual(self_poi.identity.obj_id, 20)
        self.assertIn(exhausted, peers)


class HandleNewPoiReaskTests(unittest.TestCase):
    def test_missing_speedup_baseline_does_not_consume_reask(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        vehicle.get_parameter.side_effect = [None, 1.0]
        vehicle.set_parameter.return_value = True
        detector = _create_mock_detector()
        args = _create_mock_args()
        args.nav_sim_speedup = 10.0
        controller = _create_controller(
            vehicle=vehicle,
            detector=detector,
            args=args,
        )
        poi = _create_detected_poi(obj_id=19)
        controller.confirmation_manager.update_status(
            poi,
            ConfirmationStatus.TIMEOUT_REJECTED,
        )

        controller.navigation_task_action.handle_new_poi(poi)

        detector.start_tracking.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertEqual(
            controller.confirmation_manager.get_status(poi),
            ConfirmationStatus.TIMEOUT_REJECTED,
        )
        self.assertNotIn(19, controller.retry_state.reask_attempts)

        controller.navigation_task_action.handle_new_poi(poi)

        detector.start_tracking.assert_called_once_with(19)
        self.assertIs(controller.confirmation_manager.active_poi, poi)
        self.assertIsNone(controller.confirmation_manager.get_status(poi))
        self.assertEqual(controller.retry_state.reask_attempts.get(19), 1)

    def test_reopens_timeout_rejected_poi_under_budget(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)

        controller.navigation_task_action.handle_new_poi(poi)

        detector.start_tracking.assert_called_once()
        self.assertIs(controller.confirmation_manager.active_poi, poi)
        # Status was cleared by _begin_reask (a fresh CONFIRM entry will
        # write CONFIRMING via review(), not yet run here).
        self.assertIsNone(controller.confirmation_manager.get_status(poi))
        self.assertEqual(controller.retry_state.reask_attempts.get(10), 1)

    def test_does_not_reopen_timeout_rejected_poi_over_budget(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.retry_state.reask_attempts[10] = CONFIRM_REASK_MAX_ATTEMPTS

        controller.navigation_task_action.handle_new_poi(poi)

        detector.start_tracking.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertEqual(
            controller.confirmation_manager.get_status(poi), ConfirmationStatus.TIMEOUT_REJECTED,
            "Status must stay TIMEOUT_REJECTED -- budget exhausted, never reopened.",
        )

    def test_never_reopens_operator_rejected_poi(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        poi = _create_detected_poi(obj_id=10)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        controller.navigation_task_action.handle_new_poi(poi)

        detector.start_tracking.assert_not_called()
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertEqual(
            controller.confirmation_manager.get_status(poi), ConfirmationStatus.REJECTED,
            "An operator REJECTED POI must never be reopened.",
        )


class DecideTimeoutRejectedTeardownTests(unittest.TestCase):
    def test_decide_tears_down_timeout_rejected_to_detect(self):
        """Mirrors the existing REJECTED teardown test: TIMEOUT_REJECTED
        gets the SAME immediate clear-active/stop-tracking/DETECT teardown."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.navigation_task.navigation_poi_location = poi.geo.truth_poi_location

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIsNone(controller.navigation_task.navigation_poi_location)
        detector.stop_tracking.assert_called()

    def test_decide_still_tears_down_operator_rejected_to_detect(self):
        """Regression guard: the pre-existing REJECTED teardown is unchanged."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)

        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)

        controller.decision.decide()

        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        detector.stop_tracking.assert_called()

    def test_decide_resets_reask_attempts_on_confirmed(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.CONFIRMED)
        controller.retry_state.reask_attempts[1] = 1

        controller.decision.decide()

        self.assertNotIn(1, controller.retry_state.reask_attempts)

    def test_decide_resets_reask_attempts_on_operator_rejected(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.REJECTED)
        controller.retry_state.reask_attempts[1] = 1

        controller.decision.decide()

        self.assertNotIn(1, controller.retry_state.reask_attempts)

    def test_decide_does_not_reset_reask_attempts_on_timeout_rejected(self):
        """The whole point of the budget is to survive this EXACT teardown
        across a reacquire -- it must NOT be cleared here."""
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        poi = _create_detected_poi(obj_id=1)
        controller.confirmation_manager.set_active_poi(poi)
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.retry_state.reask_attempts[1] = 1

        controller.decision.decide()

        self.assertEqual(controller.retry_state.reask_attempts.get(1), 1)


class FullReaskCycleIntegrationTests(unittest.TestCase):
    """Multi-tick integration: a POI that keeps timing out gets
    reopened up to CONFIRM_REASK_MAX_ATTEMPTS times, then stays rejected."""

    def _run_one_timeout_restart_navigation_task_cycle(self, controller, poi):
        """Simulate one full round: TIMEOUT_REJECTED -> _decide() tears down
        to DETECT -> _select_poi/_handle_new_poi reopens (bounded) if
        the POI is still present in _last_detections."""
        controller.confirmation_manager.update_status(poi, ConfirmationStatus.TIMEOUT_REJECTED)
        controller.decision.decide()
        self.assertEqual(controller.phase.current, NavState.DETECT)
        self.assertIsNone(controller.confirmation_manager.active_poi)

        self_poi, _ = controller.selector.select([poi])
        controller.navigation_task_action.handle_new_poi(self_poi)

    def test_bounded_reask_then_permanently_rejected(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        detector = _create_mock_detector()
        controller = _create_controller(vehicle=vehicle, detector=detector)
        poi = _create_detected_poi(obj_id=99)

        # Initial navigation_task.
        controller.confirmation_manager.set_active_poi(poi)

        for round_num in range(1, CONFIRM_REASK_MAX_ATTEMPTS + 1):
            self._run_one_timeout_restart_navigation_task_cycle(controller, poi)
            self.assertIs(
                controller.confirmation_manager.active_poi, poi,
                f"Round {round_num}: POI should be reopened for a fresh "
                f"confirm round (budget {round_num}/{CONFIRM_REASK_MAX_ATTEMPTS}).",
            )
            self.assertIsNone(
                controller.confirmation_manager.get_status(poi),
                f"Round {round_num}: status must be cleared for a fresh review.",
            )

        # One more timeout exhausts the budget -- must NOT reopen again.
        self._run_one_timeout_restart_navigation_task_cycle(controller, poi)

        self.assertIsNone(
            controller.confirmation_manager.active_poi,
            "Budget exhausted: POI must stay torn down, not reopened.",
        )
        self.assertEqual(
            controller.confirmation_manager.get_status(poi), ConfirmationStatus.TIMEOUT_REJECTED,
            "Budget exhausted: status stays TIMEOUT_REJECTED permanently.",
        )
        self.assertEqual(
            controller.retry_state.reask_attempts.get(99), CONFIRM_REASK_MAX_ATTEMPTS,
        )


if __name__ == '__main__':
    unittest.main()
