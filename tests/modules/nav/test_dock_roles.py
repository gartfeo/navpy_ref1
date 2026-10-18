"""Mechanism tests for e5: which docks a UAV may take or offer by role.

docs/design/delivery_eval_scenario.md, "Remaining plan", step e5:
- the home dock is never offered (todo 29);
- on the way home, an unassigned delivery dock is asked for (ruling 4);
- a never-seen home dock keeps the orbit, with no confirmation and no command;
- the winner passing d1 neither locks d1 nor offers it (ruling 5).
"""

import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from navpy.core.ports.detect_response import DetectResponse
from navpy.core.flow.nav_state import NavState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.acquire.approach_strategy import (
    ApproachKind,
    ApproachPlan,
)
from navpy.core.flow.nav_constants import DOCK_ASSOCIATION_RADIUS_M
from navpy.core.kernel.location import Location
from navpy.core.skills.search.dock_roles import DockRolePorts, DockRoles
from navpy.core.ports.flight_mode import FlightMode
from tests.modules.nav.test_nav_controller import (
    _assigned_peer_task,
    _commit,
    _create_controller,
    _create_detected_poi,
    _create_mock_detector,
    _create_mock_navigation,
    _create_mock_vehicle,
)

HOME = Location(40.01, -74.0, 0.0)
# The assigned task's dock (_assigned_peer_task) and a dock 150 m from it,
# as d2 and d1 in the delivery scenario.
D2 = Location(40.001, -74.001, 0.0)
D1 = Location(40.001 - 150.0 / 111_195.0, -74.001, 0.0)
DELIVERY = Location(40.003, -74.003, 0.0)


def _orbit_around(poi, *_args, **_kwargs):
    """A camera-sized orbit centred on the planned dock."""
    return ApproachPlan(
        kind=ApproachKind.ORBIT,
        approach_location=poi,
        offset_distance=0.0,
        orbit_radius=300.0,
    )


def _orbit_planner():
    return patch(
        "navpy.app.flow.nav_composition.calc_peer_approach_offset",
        side_effect=_orbit_around,
    )


def _dock(obj_id, location):
    """A sim detection: the world dock's truth position, not yet offered."""
    poi = _create_detected_poi(obj_id=obj_id)
    poi.replace_geo(replace(poi.geo, truth_poi_location=location))
    return poi


def _offered(task_actor):
    return [
        poi
        for call in task_actor.notify_pois.call_args_list
        for poi in call.args[0]
    ]


def _idle_task_actor(controller):
    task_actor = Mock()
    task_actor.selected_poi.return_value = None
    controller.network.task_actor = task_actor
    return task_actor


def _homing_controller(approach_kind=ApproachKind.OFFSET):
    vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=9)
    vehicle.mission_items_count = 10
    vehicle.lim_roll = 45.0
    vehicle.min_pitch = -40.0
    vehicle.air_speed = 25.0
    detector = _create_mock_detector()
    navigation = _create_mock_navigation()
    controller = _create_controller(
        vehicle=vehicle, detector=detector, navigation=navigation,
        approach_kind=approach_kind,
    )
    controller.mission.default_delivery_hub = HOME
    controller.mission.scan_altitude_rel = 90.0
    task_actor = _idle_task_actor(controller)
    return controller, task_actor


class TestHomeDockIsNeverOffered(unittest.TestCase):

    def test_the_home_dock_is_not_offered_at_the_hub(self):
        # todo 29: at the hub the home dock, seen beside the UAV's own POI,
        # went to the peer list and was offered as a swarm task.
        controller, task_actor = _homing_controller()
        controller.fallback_navigation.setup()
        delivery = _dock(5, DELIVERY)
        home = _dock(4, HOME)
        controller.confirmation_manager.set_active_poi(delivery)
        controller.detections.detected_pois = [delivery, home]
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        self.assertNotIn(home, _offered(task_actor))

    def test_the_home_dock_is_not_offered_beside_an_active_poi(self):
        controller, task_actor = _homing_controller()
        controller.fallback_navigation.setup()
        delivery = _dock(5, DELIVERY)
        home = _dock(4, HOME)
        controller.confirmation_manager.set_active_poi(delivery)

        _, peers = controller.selector.select([delivery, home])

        self.assertEqual(peers, [])

    def test_the_home_dock_is_not_offered_during_search(self):
        vehicle = _create_mock_vehicle(mode=FlightMode.AUTO, next_wp=5)
        controller = _create_controller(vehicle=vehicle)
        controller.mission.default_delivery_hub = HOME
        _idle_task_actor(controller)
        delivery = _dock(5, DELIVERY)
        home = _dock(4, HOME)

        own, peers = controller.selector.select([delivery, home])

        self.assertIs(own, delivery)
        self.assertEqual(peers, [])

    def test_another_dock_beside_the_home_dock_is_still_offered(self):
        controller, _ = _homing_controller()
        controller.fallback_navigation.setup()
        delivery = _dock(5, DELIVERY)
        other = _dock(6, D2)
        home = _dock(4, HOME)

        own, peers = controller.selector.select([home, delivery, other])

        self.assertIs(own, delivery)
        self.assertEqual(peers, [other])


class TestWayHomeTakesAnUnassignedDock(unittest.TestCase):
    """Ruling 4: on the way home a delivery dock nobody took is asked for."""

    def test_the_delivery_dock_is_asked_for_before_the_home_dock(self):
        controller, task_actor = _homing_controller()
        controller.fallback_navigation.setup()
        home = _dock(4, HOME)
        delivery = _dock(5, DELIVERY)
        # The detector lists the home dock first.
        controller.detections.detected_pois = [home, delivery]
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        self.assertIs(controller.confirmation_manager.active_poi, delivery)
        self.assertNotIn(home, _offered(task_actor))

        controller.decision.decide()

        self.assertIs(controller.phase.current, NavState.CONFIRM)

    def test_the_delivery_dock_gets_its_own_circle_through_approval(self):
        # Codex e5 review: the home-dock circle must not stay in place, or
        # the delivery dock leaves the view while it waits for approval.
        with _orbit_planner():
            controller, _ = _homing_controller(ApproachKind.ORBIT)
            controller.fallback_navigation.setup()
            self.assertGreater(controller.navigation_task.orbit_radius_m, 0.0)
            home = _dock(4, HOME)
            delivery = _dock(5, DELIVERY)
            controller.detections.detected_pois = [home, delivery]
            _commit(controller, NavState.DETECT)

            controller.detect_action.act()

            loiter = controller.navigation.vehicle_commands.peer_poi_loiter
            circle_centre = loiter.call_args.args[0]
            self.assertLess(circle_centre.distance_to(DELIVERY), 1.0)
            target = controller.navigation_task.navigation_poi_location
            self.assertLess(target.distance_to(DELIVERY), 1.0)

            controller.decision.decide()
            controller.confirmation_manager.update_status(
                delivery, ConfirmationStatus.CONFIRMED,
            )
            controller.detections.detected_pois = [home, delivery]
            controller.decision.decide()

            self.assertIs(controller.confirmation_manager.active_poi, delivery)
            target = controller.navigation_task.navigation_poi_location
            self.assertLess(target.distance_to(DELIVERY), 1.0)

    def test_the_home_dock_keeps_the_home_circle(self):
        with _orbit_planner():
            controller, _ = _homing_controller(ApproachKind.ORBIT)
            controller.fallback_navigation.setup()
            orbit_r = controller.navigation_task.orbit_radius_m
            hub = controller.navigation_task.navigation_poi_location
            home = _dock(4, HOME)
            controller.detections.detected_pois = [home]
            _commit(controller, NavState.DETECT)

            controller.detect_action.act()

            self.assertIs(controller.confirmation_manager.active_poi, home)
            self.assertEqual(controller.navigation_task.orbit_radius_m, orbit_r)
            self.assertEqual(controller.navigation_task.navigation_poi_location, hub)

    def test_the_home_dock_alone_is_asked_for(self):
        controller, task_actor = _homing_controller()
        controller.fallback_navigation.setup()
        home = _dock(4, HOME)
        controller.detections.detected_pois = [home]
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        self.assertIs(controller.confirmation_manager.active_poi, home)
        task_actor.notify_pois.assert_not_called()


class TestNeverSeenHomeDockKeepsTheOrbit(unittest.TestCase):

    def test_orbit_holds_with_no_confirmation_and_no_command(self):
        controller, task_actor = _homing_controller()
        controller.detector.get_detect_data.return_value = DetectResponse([])
        _commit(controller, NavState.DETECT)
        controller.sensor.sense()
        controller.decision.decide()
        controller.detect_action.act()
        self.assertTrue(controller.mission.default_delivery_hub_active)
        orbit_r = controller.navigation_task.orbit_radius_m
        hub = controller.navigation_task.navigation_poi_location
        self.assertIsNotNone(hub)
        commands = controller.navigation.vehicle_commands
        approach_calls = (
            commands.peer_poi.call_count,
            commands.peer_poi_loiter.call_count,
        )
        self.assertNotEqual(approach_calls, (0, 0))
        guided_calls = controller.vehicle.set_mode.call_count

        for _ in range(50):
            controller.sensor.sense()
            controller.decision.decide()
            controller.actions.act()
            self.assertIs(controller.phase.current, NavState.DETECT)

        self.assertIsNone(controller.confirmation_manager.active_poi)
        self.assertEqual(controller.navigation_task.orbit_radius_m, orbit_r)
        self.assertEqual(controller.navigation_task.navigation_poi_location, hub)
        self.assertEqual(controller.vehicle.set_mode.call_count, guided_calls)
        self.assertEqual(
            (commands.peer_poi.call_count, commands.peer_poi_loiter.call_count),
            approach_calls,
        )
        controller.navigation.nav.assert_not_called()
        controller.navigation.final_approach.record_confirmed_detection.assert_not_called()
        task_actor.notify_pois.assert_not_called()


class TestWinnerGoesToItsAssignedDockOnly(unittest.TestCase):
    """Ruling 5: an assigned UAV neither locks nor offers another dock."""

    def _winner_near_its_dock(self):
        controller = _create_controller()
        task_actor = _assigned_peer_task(controller)
        controller.detections.detected_pois = []
        _commit(controller, NavState.DETECT)
        controller.detect_action.act()
        self.assertTrue(controller.navigation_task.peer_navigation)
        controller.peer_navigation.near_poi = Mock(return_value=True)
        return controller, task_actor

    def test_the_winner_passing_d1_does_not_lock_or_offer_it(self):
        controller, task_actor = self._winner_near_its_dock()
        d1 = _dock(1, D1)
        controller.detections.detected_pois = [d1]

        controller.detect_action.act()

        self.assertIsNone(controller.confirmation_manager.active_poi)
        task_actor.notify_pois.assert_not_called()

    def test_the_winner_locks_d2_when_d1_is_listed_first(self):
        controller, task_actor = self._winner_near_its_dock()
        d1 = _dock(1, D1)
        d2 = _dock(2, D2)
        controller.detections.detected_pois = [d1, d2]

        controller.detect_action.act()

        self.assertIs(controller.confirmation_manager.active_poi, d2)
        task_actor.notify_pois.assert_not_called()

    def test_the_winner_offers_nothing_while_confirming_d2(self):
        controller, task_actor = self._winner_near_its_dock()
        d1 = _dock(1, D1)
        d2 = _dock(2, D2)
        controller.confirmation_manager.set_active_poi(d2)

        _, peers = controller.selector.select([d2, d1])

        self.assertEqual(peers, [])

    def test_a_pending_assignment_offers_nothing(self):
        controller = _create_controller()
        task_actor = _assigned_peer_task(controller)
        own = _dock(6, DELIVERY)
        other = _dock(8, D1)
        controller.detections.detected_pois = [own, other]
        _commit(controller, NavState.DETECT)

        controller.detect_action.act()

        task_actor.notify_pois.assert_not_called()



def _north_of(location, metres):
    return Location(location.lat + metres / 111_195.0, location.lng, 0.0)


def _roles(*, assigned_dock=None, ground=None, simulation=True):
    return DockRoles(DockRolePorts(
        home_dock=lambda: HOME,
        homing=lambda: False,
        assigned=lambda: assigned_dock is not None,
        assigned_dock=lambda: assigned_dock,
        detector_is_simulation=lambda: simulation,
        ground_location=lambda *_a, **_k: ground,
    ))


class TestDockAssociationRadius(unittest.TestCase):

    def test_the_home_dock_ends_at_the_association_radius(self):
        roles = _roles()
        inside = _dock(1, _north_of(HOME, DOCK_ASSOCIATION_RADIUS_M - 0.5))
        outside = _dock(2, _north_of(HOME, DOCK_ASSOCIATION_RADIUS_M + 0.5))

        self.assertTrue(roles.is_home_dock(inside))
        self.assertFalse(roles.is_home_dock(outside))
        self.assertFalse(roles.may_offer(inside))
        self.assertTrue(roles.may_offer(outside))

    def test_the_assigned_dock_ends_at_the_association_radius(self):
        roles = _roles(assigned_dock=D2)
        inside = _dock(1, _north_of(D2, DOCK_ASSOCIATION_RADIUS_M - 0.5))
        outside = _dock(2, _north_of(D2, DOCK_ASSOCIATION_RADIUS_M + 0.5))

        self.assertTrue(roles.may_take(inside))
        self.assertFalse(roles.may_take(outside))

    def test_a_detection_without_a_position_is_neither_home_nor_taken(self):
        # Off the sim and with no ground estimate there is no position.
        roles = _roles(assigned_dock=D2, simulation=False)
        poi = _create_detected_poi(obj_id=3)

        self.assertFalse(roles.is_home_dock(poi))
        self.assertFalse(roles.may_take(poi))
        self.assertFalse(roles.may_offer(poi))

    def test_off_the_sim_the_ground_estimate_is_used(self):
        roles = _roles(ground=_north_of(HOME, 5.0), simulation=False)
        poi = _create_detected_poi(obj_id=3)

        self.assertTrue(roles.is_home_dock(poi))

if __name__ == "__main__":
    unittest.main()
