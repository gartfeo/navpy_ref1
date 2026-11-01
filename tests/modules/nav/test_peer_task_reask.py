"""Mechanism tests: an unanswered confirmation of the assigned dock keeps the
peer task and asks again (docs/design/swarm-task-assignment-ack.md,
"Unanswered confirmation of the assigned dock").

Delivery demo 20261025-140306: UAV 2's first round timed out, it released
task 2 and resumed AUTO while its owner had already forgotten the task.
"""

import time
import unittest
from dataclasses import replace
from unittest.mock import MagicMock, Mock

from navpy.core.flow.nav_constants import CONFIRM_REASK_MAX_ATTEMPTS
from navpy.core.flow.nav_state import NavState
from navpy.core.kernel.location import Location
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.task_confirmation_msg import (
    TaskConfirmRequestMsg,
    TaskConfirmResponseMsg,
)
from navpy.core.ports.detect_response import DetectResponse
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from tests.modules.nav.test_dock_roles import D1, D2, _dock, _orbit_planner
from tests.modules.nav.test_nav_controller import (
    _assigned_peer_task,
    _commit,
    _create_controller,
    _create_detected_poi,
    _create_mock_args,
    _create_mock_detector,
    _create_mock_vehicle,
)

TASK_DOCK = Location(40.001, -74.001, 0.0, is_absolute=True)


def _logged(logger, method="info"):
    return [call.args[0] for call in getattr(logger, method).call_args_list]


def _wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def _stamping_network(requests):
    """A network that gives each confirmation request its wire identity."""
    def broadcast(message):
        if not isinstance(message, TaskConfirmRequestMsg):
            return
        if message.meta is None:
            message.set_meta(MsgMeta(801, len(requests) + 1, 1000, 5000))
        if not requests or requests[-1] is not message:
            requests.append(message)

    network = MagicMock(spec=NetworkAbc)
    network.broadcast.side_effect = broadcast
    return network


def _response_to(request, confirmed):
    meta = request.meta
    return TaskConfirmResponseMsg(
        receiver_id=1,
        task_id=request.task.task_id,
        is_confirmed=confirmed,
        meta=MsgMeta(meta.boot_id, meta.msg_seq, meta.time_ms + 1, meta.ttl_ms),
    )


class TestUnansweredPeerTaskIsKept(unittest.TestCase):

    def setUp(self):
        self.logger = Mock()
        self.vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        self.detector = _create_mock_detector()

    def _flying_the_task(self, approach_kind=ApproachKind.ORBIT, args=None):
        with _orbit_planner():
            controller = _create_controller(
                vehicle=self.vehicle,
                detector=self.detector,
                logger=self.logger,
                approach_kind=approach_kind,
                args=args,
            )
            task_actor = _assigned_peer_task(controller)
            controller.detections.detected_pois = []
            _commit(controller, NavState.DETECT)
            controller.detect_action.act()
        self.assertTrue(controller.navigation_task.peer_navigation)
        controller.peer_navigation.near_poi = Mock(return_value=True)
        self.vehicle.set_mode.reset_mock()
        self.detector.start_geo_tracking.reset_mock()
        self.detector.stop_geo_tracking.reset_mock()
        return controller, task_actor

    def _ask(self, controller, dock):
        """The detection starts a round; it ends with the given status."""
        controller.detections.detected_pois = [dock]
        _commit(controller, NavState.DETECT)
        controller.detect_action.act()
        self.assertIs(controller.confirmation_manager.active_poi, dock)

    def _answer(self, controller, dock, status):
        controller.confirmation_manager.update_status(dock, status)
        _commit(controller, NavState.CONFIRM)
        controller.poi_status.dispatch()

    def _assert_kept(self, controller, task_actor):
        task_actor.clear_selected_poi.assert_not_called()
        task_actor.reset.assert_not_called()
        self.vehicle.set_mode.assert_not_called()
        self.assertTrue(controller.navigation_task.peer_navigation)
        self.assertEqual(controller.navigation_task.peer_poi_location, TASK_DOCK)
        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertIs(controller.phase.current, NavState.DETECT)
        # Still flying the task: the task is not set up again.
        self.assertFalse(controller.peer_task_priority.pending())

    def _assert_released(self, controller, task_actor):
        task_actor.clear_selected_poi.assert_called_once_with()
        self.vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)
        self.assertFalse(controller.navigation_task.peer_navigation)

    def test_a_timeout_keeps_the_task_and_points_at_its_dock_again(self):
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        self._ask(controller, dock)
        # Round 1 left review state behind: a geo hold on its own estimate.
        controller.geo_hold.active = True
        controller.geo_hold.last_own_poi_geo = Location(40.0011, -74.0011, 0.0)
        controller.geo_hold.acquisition_log_bucket = ("round 1",)

        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_kept(controller, task_actor)
        self.assertFalse(controller.geo_hold.active)
        self.assertIsNone(controller.geo_hold.last_own_poi_geo)
        # The re-pointed camera's first acquisition state is logged, as after setup.
        self.assertNotEqual(controller.geo_hold.acquisition_log_bucket, ("round 1",))
        self.assertEqual(controller.detections.detected_pois, [])
        self.detector.start_geo_tracking.assert_called_once_with(
            TASK_DOCK, controller.navigation.legacy_pois.geo_ref,
        )
        self.assertEqual(controller.geo_hold.poi_location, TASK_DOCK)
        self.assertIn(
            "POI: P2 got no answer; asking again for peer task 77 (kept)",
            _logged(self.logger),
        )

    def test_the_next_detection_asks_again_and_counts_the_reask(self):
        controller, _ = self._flying_the_task()
        dock = _dock(2, D2)
        self._ask(controller, dock)
        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._ask(controller, dock)

        self.assertIsNone(controller.confirmation_manager.get_status(dock))
        self.assertEqual(controller.retry_state.reask_attempts, {2: 1})

    def test_three_unanswered_rounds_release_the_task_once(self):
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        for _ in range(CONFIRM_REASK_MAX_ATTEMPTS):
            self._ask(controller, dock)
            self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)
            self._assert_kept(controller, task_actor)

        self._ask(controller, dock)
        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_released(controller, task_actor)
        self.assertIn(
            "POI: P2 got no answer after every re-ask; peer task 77 released "
            "locally, no automatic recovery",
            _logged(self.logger, "warning"),
        )
        self.assertEqual(controller.navigation_task.peer_unanswered_rounds, 0)

    def test_a_dock_back_under_a_new_id_does_not_restart_the_budget(self):
        # A lost track identity expires; the same dock comes back as a new
        # POI id, which has a fresh per-POI budget.
        controller, task_actor = self._flying_the_task()
        for poi_id in range(2, 2 + CONFIRM_REASK_MAX_ATTEMPTS):
            dock = _dock(poi_id, D2)
            self._ask(controller, dock)
            self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)
            self._assert_kept(controller, task_actor)

        dock = _dock(9, D2)
        self._ask(controller, dock)
        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_released(controller, task_actor)

    def test_a_spent_poi_budget_releases_the_task(self):
        # The dock timed out before as an own POI, so its re-asks are used.
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        controller.retry_state.reask_attempts[2] = CONFIRM_REASK_MAX_ATTEMPTS - 1
        controller.confirmation_manager.update_status(
            dock, ConfirmationStatus.TIMEOUT_REJECTED,
        )
        self._ask(controller, dock)

        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_released(controller, task_actor)

    def test_real_rounds_ask_again_and_ignore_the_late_answer(self):
        # The loop end to end: a round that really times out, the next
        # detection asks with a new request identity, a late deny of the
        # first round changes nothing, and the second answer starts NAV.
        args = _create_mock_args(confirm_wait=0.05, confirm_on_fail=False)
        controller, task_actor = self._flying_the_task(args=args)
        requests = []
        manager = controller.confirmation_manager
        manager.set_network(_stamping_network(requests))
        dock = _create_detected_poi(obj_id=2, supports_confirmation_frame=False)
        dock.replace_geo(replace(dock.geo, truth_poi_location=D2))
        self.detector.get_detect_data.return_value = DetectResponse([dock])

        def tick_until(done):
            for _ in range(5):
                controller.sensor.sense()
                controller.decision.decide()
                controller.actions.act()
                if done():
                    return
            self.fail("the loop did not get there")

        tick_until(lambda: len(requests) == 1)
        self.assertTrue(_wait_until(
            lambda: manager.get_status(dock) is ConfirmationStatus.TIMEOUT_REJECTED
        ))
        args.confirm_wait_time_sec = 30.0  # round 2 stays open for the answers
        tick_until(lambda: len(requests) == 2)
        self.assertNotEqual(requests[0].meta.msg_seq, requests[1].meta.msg_seq)

        manager.on_message(_response_to(requests[0], confirmed=False))
        self.assertIs(manager.get_status(dock), ConfirmationStatus.CONFIRMING)
        manager.on_message(_response_to(requests[1], confirmed=True))
        tick_until(lambda: controller.phase.current is NavState.NAV)

        task_actor.clear_selected_poi.assert_not_called()
        self.vehicle.set_mode.assert_not_called()
        self.assertEqual(controller.retry_state.reask_attempts, {})
        self.assertIn(
            "Ignoring late or duplicate confirmation for P2.",
            _logged(self.logger, "warning"),
        )

    def test_the_kept_task_takes_no_other_dock(self):
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        self._ask(controller, dock)
        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        controller.detections.detected_pois = [_dock(3, D1)]
        controller.detect_action.act()

        self.assertIsNone(controller.confirmation_manager.active_poi)
        task_actor.notify_pois.assert_not_called()

    def test_an_operator_rejection_still_releases_the_task(self):
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        self._ask(controller, dock)

        self._answer(controller, dock, ConfirmationStatus.REJECTED)

        self._assert_released(controller, task_actor)

    def test_a_failed_geo_arm_keeps_the_task(self):
        # As for the first round: the approach goes on without geo pointing.
        controller, task_actor = self._flying_the_task()
        dock = _dock(2, D2)
        self._ask(controller, dock)
        self.detector.start_geo_tracking.side_effect = RuntimeError("no gimbal")

        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_kept(controller, task_actor)
        self.assertIsNone(controller.geo_hold.poi_location)

    def test_an_offset_approach_does_not_point_the_camera(self):
        controller, task_actor = self._flying_the_task(ApproachKind.OFFSET)
        dock = _dock(2, D2)
        self._ask(controller, dock)

        self._answer(controller, dock, ConfirmationStatus.TIMEOUT_REJECTED)

        self._assert_kept(controller, task_actor)
        self.detector.start_geo_tracking.assert_not_called()
        self.detector.stop_geo_tracking.assert_called()
        self.assertIsNone(controller.geo_hold.poi_location)

    def test_an_own_poi_timeout_still_resumes_auto(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.GUIDED)
        controller = _create_controller(vehicle=vehicle)
        task_actor = Mock()
        task_actor.selected_poi.return_value = None
        controller.network.task_actor = task_actor
        own = _dock(5, D1)
        controller.confirmation_manager.set_active_poi(own)

        self._answer(controller, own, ConfirmationStatus.TIMEOUT_REJECTED)

        vehicle.set_mode.assert_called_once_with(FlightMode.AUTO)
        self.assertIsNone(controller.confirmation_manager.active_poi)


if __name__ == "__main__":
    unittest.main()
