"""Peer camera tracking: geo pointing is allowed until NAV, never in it.

Mechanism tests for docs/design/peer_camera_tracking.md: the phase (not the
configured vision law) gates peer geo pointing, detection hands every camera
off from geo pointing, and NAV entry fences any leftover geo session.
"""

import unittest
from unittest.mock import Mock

from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts
from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
from navpy.core.command.gimbal.gimbal_navigation_state import (
    GimbalLossPolicy,
    GimbalTrackingSetup,
)
from navpy.core.command.gimbal.gimbal_tracking_constants import MODE_FOLLOW
from navpy.core.flow.nav_state import (
    GeoHoldState,
    NavigationTaskState,
    NavPhaseState,
    NavState,
)
from navpy.core.kernel.location import Location
from navpy.core.perception.geo_pointing_fleet import GeoPointingFleet
from navpy.core.perception.pointing.gimbal_rate_tracker import (
    GimbalRateTrackerConfig,
)
from navpy.core.perception.tracking_command_router import TrackingCommandRouter
from navpy.core.perception.tracking_geo_hand_off import GeoReleasingTracking
from navpy.core.skills.acquire.approach_strategy import ApproachKind, ApproachPlan
from navpy.core.skills.acquire.peer_approach_start import (
    PeerApproachPorts,
    PeerApproachStarter,
)
from navpy.core.skills.acquire.peer_geo_tick import (
    DetectPointingPorts,
    PeerGeoTick,
)
from tests.modules.navigation.test_gimbal_navigation import (
    _FakeGeoRef,
    _FakeLocation,
    _make_mount,
)

_TASK_LOCATION = Location(40.5, -74.5, 900.0, is_absolute=True)


def _final_approach_running(phase: NavPhaseState):
    return lambda: phase.is_current(NavState.NAV)


class TestPeerApproachStartGate(unittest.TestCase):
    def _starter(self, phase):
        plan = ApproachPlan(
            kind=ApproachKind.ORBIT,
            approach_location=_TASK_LOCATION,
            offset_distance=0.0,
            orbit_radius=686.0,
        )
        ports = PeerApproachPorts(
            current_absolute=Mock(return_value=None),
            plan_orbit=Mock(return_value=(plan, None)),
            request_guided=Mock(),
            dispatch_approach=Mock(),
            save_loiter_radius=Mock(return_value=True),
        )
        tracker = Mock()
        tracker.start.return_value = True
        tracker.prime.return_value = True
        starter = PeerApproachStarter(
            ports,
            NavigationTaskState(),
            tracker,
            _final_approach_running(phase),
            Mock(),
        )
        return starter, tracker

    def test_arms_geo_pointing_before_nav(self):
        for state in (NavState.ONHOLD, NavState.DETECT, NavState.CONFIRM):
            with self.subTest(state=state):
                starter, tracker = self._starter(NavPhaseState(current=state))

                starter.start(Mock(class_id=0), _TASK_LOCATION)

                tracker.start.assert_called_once_with(_TASK_LOCATION)
                tracker.prime.assert_called_once()

    def test_never_arms_geo_pointing_in_nav(self):
        starter, tracker = self._starter(NavPhaseState(current=NavState.NAV))

        starter.start(Mock(class_id=0), _TASK_LOCATION)

        tracker.start.assert_not_called()
        tracker.prime.assert_not_called()


class TestPeerGeoTickGate(unittest.TestCase):
    def _tick(self, phase):
        pointing = DetectPointingPorts(
            is_detection_armed=Mock(return_value=False),
            prepare_geo_acquisition=Mock(return_value=False),
            update_geo=Mock(),
        )
        acquisition = Mock()
        acquisition.snapshot.return_value = None
        tick = PeerGeoTick(
            NavigationTaskState(peer_navigation=True),
            GeoHoldState(poi_location=_TASK_LOCATION),
            pointing,
            acquisition,
            _final_approach_running(phase),
            Mock(return_value=Location(40.0, -74.0, 1500.0, is_absolute=True)),
            Mock(return_value=Mock()),
        )
        return tick, pointing

    def test_updates_geo_pointing_before_nav(self):
        tick, pointing = self._tick(NavPhaseState(current=NavState.DETECT))

        tick.update()

        pointing.update_geo.assert_called_once()

    def test_never_updates_geo_pointing_in_nav(self):
        tick, pointing = self._tick(NavPhaseState(current=NavState.NAV))

        tick.update()

        pointing.update_geo.assert_not_called()
        pointing.prepare_geo_acquisition.assert_not_called()


class _Camera:
    """One detector source over a real GimbalNavigation."""

    def __init__(self, source_name, navigation):
        self.source_name = source_name
        self._navigation = navigation

    def start_tracking(self, obj_id):
        self._navigation.start_tracking(obj_id)

    def stop_tracking(self, to_neutral=True):
        self._navigation.stop_tracking(to_neutral=to_neutral)

    def start_geo_tracking(self, poi_loc, geo_ref):
        self._navigation.start_geo_tracking(poi_loc, geo_ref)

    def update_geo(self, uav_loc, uav_att):
        self._navigation.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(self, uav_loc, uav_att, class_id, min_pixels):
        return self._navigation.prepare_geo_acquisition(
            uav_loc, uav_att, class_id, min_pixels,
        )

    def stop_geo_tracking(self):
        self._navigation.stop_geo_tracking()

    @property
    def is_geo_armed(self):
        return self._navigation.is_geo_armed

    @property
    def is_detection_armed(self):
        return self._navigation.is_detection_armed


class TestMultiCameraDetectionHandOff(unittest.TestCase):
    def _camera(self, name):
        mount, gimbal = _make_mount()
        mount.name = name
        logger = Mock()
        logger.is_enabled_for.return_value = False
        navigation = GimbalNavigation(
            mount,
            logger,
            tracking=GimbalTrackingSetup(
                GimbalRateTrackerConfig(
                    correction_bw=1.0,
                    max_rate=100.0,
                    command_lead_time=0.15,
                ),
                GimbalLossPolicy(hold_sec=0.5, repoint_sec=2.0),
            ),
            zoom_config=None,
            neutral_pitch_deg=-20.0,
        )
        return _Camera(name, navigation), gimbal

    def test_detection_on_one_camera_clears_geo_on_all_without_moving_it(self):
        tracking_camera, tracking_gimbal = self._camera("cam_a")
        other_camera, other_gimbal = self._camera("cam_b")
        cameras = [tracking_camera, other_camera]
        fleet = GeoPointingFleet(cameras, Mock())
        registry = Mock()
        registry.identity_for_task.return_value = Mock(
            source_name="cam_a",
            local_obj_id=7,
        )
        hand_off = GeoReleasingTracking(
            TrackingCommandRouter(cameras, registry, Mock()),
            fleet.stop_geo_tracking,
        )
        fleet.start_geo_tracking(_FakeLocation(40.5, -74.5, 900.0), _FakeGeoRef())
        self.assertTrue(tracking_camera.is_geo_armed)
        self.assertTrue(other_camera.is_geo_armed)
        tracking_gimbal.reset_mock()
        other_gimbal.reset_mock()

        hand_off.start_tracking(1)

        self.assertFalse(fleet.is_geo_armed)
        self.assertTrue(tracking_camera.is_detection_armed)
        self.assertFalse(other_camera.is_detection_armed)
        # The tracking camera is never sent to neutral/FOLLOW by the stop.
        tracking_gimbal.set_att.assert_not_called()
        self.assertNotIn(
            MODE_FOLLOW,
            [c.args[0] for c in tracking_gimbal.set_motion_mode.call_args_list],
        )
        # The other camera leaves geo pointing and returns to neutral.
        other_gimbal.set_motion_mode.assert_called_with(MODE_FOLLOW)
        other_gimbal.set_att.assert_called()

    def test_geo_stop_runs_only_after_a_successful_start(self):
        tracking = Mock()
        tracking.start_tracking.side_effect = RuntimeError("lock failed")
        stop_geo = Mock()
        hand_off = GeoReleasingTracking(tracking, stop_geo)

        with self.assertRaisesRegex(RuntimeError, "lock failed"):
            hand_off.start_tracking(1)

        stop_geo.assert_not_called()


class TestNavEntryGeoFence(unittest.TestCase):
    def _entry(self, *, stop_geo, is_geo_armed, events):
        ports = NavEntryPorts(
            request_guided=Mock(side_effect=lambda: events.append("guided")),
            navigation_init=Mock(),
            final_approach_active=Mock(return_value=True),
            arm_dock_contact=Mock(),
            stop_geo_tracking=stop_geo,
            is_geo_armed=is_geo_armed,
        )
        zoom = Mock()
        zoom.freeze_final_approach_wide.return_value = True
        return NavEntry(ports, NavigationTaskState(), zoom, Mock()), ports

    def test_leftover_geo_session_is_stopped_before_guided(self):
        events = []
        armed = [True]

        def stop():
            events.append("stop_geo")
            armed[0] = False

        entry, ports = self._entry(
            stop_geo=Mock(side_effect=stop),
            is_geo_armed=lambda: armed[0],
            events=events,
        )

        entry.run()

        self.assertEqual(events, ["stop_geo", "guided"])
        ports.navigation_init.assert_called_once()

    def test_entry_fails_when_geo_stays_armed(self):
        events = []
        entry, ports = self._entry(
            stop_geo=Mock(),
            is_geo_armed=lambda: True,
            events=events,
        )

        with self.assertRaisesRegex(RuntimeError, "geo pointing still armed"):
            entry.run()

        self.assertEqual(events, [])
        ports.navigation_init.assert_not_called()

    def test_entry_propagates_geo_stop_failure(self):
        events = []
        entry, ports = self._entry(
            stop_geo=Mock(side_effect=OSError("gimbal link down")),
            is_geo_armed=lambda: False,
            events=events,
        )

        with self.assertRaisesRegex(OSError, "gimbal link down"):
            entry.run()

        self.assertEqual(events, [])
        ports.navigation_init.assert_not_called()


if __name__ == "__main__":
    unittest.main()
