"""Integration tests for gimbal tracking wiring.

Tests that GimbalRateTracker is correctly wired through
DetectorSim → DetectionCoordinator → NavController.
"""

import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.modules.vision.gimbal_rate_tracker import (
    GimbalRateTracker,
    GimbalRateTrackerConfig,
    TrackingState,
)
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample


class TestGimbalRateTrackerThreadSafety(unittest.TestCase):
    """Verify concurrent update/reset does not crash."""

    def test_concurrent_update_and_reset_no_crash(self):
        gimbal = Mock()
        tracker = GimbalRateTracker(gimbal, Mock(), GimbalRateTrackerConfig())

        errors = []

        def updater():
            for index in range(200):
                try:
                    timestamp_s = float(index + 1)
                    tracker.update(
                        GimbalAngularSample(0.1, -0.1, timestamp_s)
                    )
                except Exception as e:
                    errors.append(e)

        def resetter():
            for _ in range(200):
                try:
                    tracker.reset()
                except Exception as e:
                    errors.append(e)

        t1 = threading.Thread(target=updater)
        t2 = threading.Thread(target=resetter)
        t1.start()
        t2.start()
        t1.join(timeout=5)
        t2.join(timeout=5)
        self.assertEqual(errors, [])

    def test_reset_sets_idle(self):
        gimbal = Mock()
        tracker = GimbalRateTracker(gimbal, Mock(), GimbalRateTrackerConfig())

        tracker.update(
            GimbalAngularSample(0.1, -0.1, 1.0)
        )
        self.assertEqual(tracker.state, TrackingState.TRACKING)

        tracker.reset()
        self.assertEqual(tracker.state, TrackingState.IDLE)


class TestDetectorSimTracking(unittest.TestCase):
    """Test DetectorSim start/stop_tracking and detect_targets integration."""

    def _make_detector(self, with_tracker=True):
        from navpy.modules.vision.sim.detector_sim import DetectorSim
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
        from navpy.args.uas_args import UasArgs
        from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
        from tests.conftest import create_mock_args

        gimbal = Mock()
        gimbal.get_data.return_value = GimbalData(att=Attitude(-30, 0, 0))

        mount = Mock()
        mount.name = "test_mount"
        mount.gimbal = gimbal
        mount.image_width = 1920
        mount.image_height = 1080
        mount.get_k.return_value = np.array(
            [[2000, 0, 960], [0, 2000, 540], [0, 0, 1]], dtype=np.float64
        )
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(-30, 0, 0), roll_stabilize=True, pitch_stabilize=True,
            max_detect_distance=3000.0,
        )
        mount.is_valid.return_value = True

        vehicle = Mock()
        vehicle.home_location = Location(32.0, 34.8, 0.0, is_absolute=True)
        vehicle.mission_items_count = 0
        vehicle.get_param_or_default.return_value = 50.0
        vehicle.get_parameter.return_value = 1.0
        vehicle.get_mission_item_location.return_value = Location(
            32.004, 34.8, 0.0, is_absolute=True,
        )
        detector = DetectorSim(
            vehicle,
            mount,
            GeoRefCalc(UasArgs()),
            Mock(),
            create_mock_args(),
            tracking_config=(
                GimbalTrackingSetup(GimbalRateTrackerConfig())
                if with_tracker
                else None
            ),
        )

        return detector, gimbal

    def test_start_tracking_sets_obj_id(self):
        detector, _ = self._make_detector()
        detector.start_tracking(42)
        self.assertEqual(detector.navigation.tracking_obj_id, 42)

    def test_stop_tracking_clears_obj_id(self):
        detector, gimbal = self._make_detector()
        detector.start_tracking(42)
        detector.stop_tracking()
        self.assertIsNone(detector.navigation.tracking_obj_id)
        gimbal.set_att.assert_called_once()  # return_to_neutral

    def test_start_tracking_noop_without_tracker(self):
        detector, _ = self._make_detector(with_tracker=False)
        detector.start_tracking(42)
        self.assertIsNone(detector.navigation)

    def test_tracker_update_called_when_tracking(self):
        detector, _ = self._make_detector()
        rate_tracker = detector.navigation._parts.status._trackers.rate

        detector.set_sim_target(
            0,
            Location(32.004, 34.8, 0.0, is_absolute=True),
        )
        rate_tracker.update = Mock(wraps=rate_tracker.update)

        # Not tracking — tracker not called
        detector.detect_targets(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            Attitude(0, 0, 0),
            frame_timestamp_s=1.0,
        )
        rate_tracker.update.assert_not_called()

        # Start tracking — tracker called with non-None
        detector.start_tracking(0)
        detector.detect_targets(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            Attitude(0, 0, 0),
            frame_timestamp_s=2.0,
        )
        rate_tracker.update.assert_called_once()
        self.assertIsNotNone(rate_tracker.update.call_args[0][0])

    def test_tracker_does_not_own_loss_when_target_not_visible(self):
        detector, _ = self._make_detector()
        rate_tracker = detector.navigation._parts.status._trackers.rate

        rate_tracker.update = Mock(wraps=rate_tracker.update)

        detector.start_tracking(99)
        detector.detect_targets(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            Attitude(0, 0, 0),
            frame_timestamp_s=1.0,
        )
        rate_tracker.update.assert_not_called()


class TestProjectTarget(unittest.TestCase):
    """Test project_target vs detect separation."""

    def test_project_target_ignores_is_valid(self):
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
        from navpy.modules.vision.simulation_object import SimulationObject
        from navpy.args.uas_args import UasArgs
        from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
        from navpy.modules.vision.sim.finite_target_projector import FiniteTargetProjector
        from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
        from navpy.modules.vision.sim.ideal_target_projector import (
            IdealTargetProjector,
            UasFrameConvention,
        )
        from navpy.modules.vision.sim.sim_camera_ports import (
            FrameSize,
            ProjectionCameraPort,
        )
        from navpy.modules.vision.sim.sim_target_projector import SimTargetProjector

        mount = Mock()
        mount.image_width = 1920
        mount.image_height = 1080
        mount.get_k.return_value = np.array(
            [[2000, 0, 960], [0, 2000, 540], [0, 0, 1]], dtype=np.float64
        )
        mount.get_gimbal_data.return_value = GimbalData(
            att=Attitude(-30, 0, 0), roll_stabilize=True, pitch_stabilize=True,
            max_detect_distance=3000.0,
        )
        mount.is_valid.return_value = False

        geo_ref = GeoRefCalc(UasArgs())
        frame_size = FrameSize(1920, 1080)
        finite = FiniteTargetProjector(
            ProjectionCameraPort(
                mount.get_k,
                mount.get_gimbal_data,
                frame_size,
                mount.is_valid,
            ),
            geo_ref.calc_uv,
            lambda: (),
            lambda: 1.0,
        )
        ideal = IdealTargetProjector(
            IdealCameraState(mount.get_gimbal_data),
            frame_size,
            UasFrameConvention(geo_ref.uas_seq, geo_ref.degrees),
            lambda: 1.0,
        )
        projector = SimTargetProjector(False, finite, ideal)

        target = SimulationObject(0, Location(32.004, 34.8, 0.0, is_absolute=True), 2.0)
        result = projector.project_target(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            target,
            Attitude(0, 0, 0),
        )
        self.assertIsNotNone(result)

        from navpy.modules.vision.models.detect_data import DetectStatus
        detect_result = projector.detect(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            target,
            Attitude(0, 0, 0),
        )
        self.assertNotEqual(detect_result.status, DetectStatus.DETECTED)


class TestGimbalPhysicsModeSwitch(unittest.TestCase):
    """Test yaw conversion when switching between FOLLOW and LOCK modes."""

    def test_follow_to_lock_preserves_pointing(self):
        from navpy.modules.vision.peripheral.siyi.sim.gimbal_physics import (
            GimbalPhysics, MODE_FOLLOW, MODE_LOCK,
        )
        physics = GimbalPhysics(initial_pitch=-15.0, initial_yaw=10.0)
        physics.set_motion_mode(MODE_FOLLOW)
        physics.set_vehicle_attitude(Attitude(0, 90, 0))

        physics.set_motion_mode(MODE_LOCK)
        self.assertAlmostEqual(physics.yaw, 100.0, places=1)

    def test_lock_to_follow_roundtrips(self):
        from navpy.modules.vision.peripheral.siyi.sim.gimbal_physics import (
            GimbalPhysics, MODE_FOLLOW, MODE_LOCK,
        )
        physics = GimbalPhysics(initial_yaw=10.0)
        physics.set_motion_mode(MODE_FOLLOW)
        physics.set_vehicle_attitude(Attitude(0, 90, 0))

        physics.set_motion_mode(MODE_LOCK)
        physics.set_motion_mode(MODE_FOLLOW)
        self.assertAlmostEqual(physics.yaw, 10.0, places=1)

    def test_same_mode_is_noop(self):
        from navpy.modules.vision.peripheral.siyi.sim.gimbal_physics import (
            GimbalPhysics, MODE_FOLLOW,
        )
        physics = GimbalPhysics(initial_yaw=5.0)
        physics.set_motion_mode(MODE_FOLLOW)
        physics.set_vehicle_attitude(Attitude(0, 90, 0))

        physics.set_motion_mode(MODE_FOLLOW)
        self.assertAlmostEqual(physics.yaw, 5.0, places=1)


class TestBodyFrameReporting(unittest.TestCase):
    """Test that GimbalSiyiSim reports body-frame angles in LOCK mode."""

    def test_lock_mode_reports_body_frame_yaw(self):
        from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_sim import GimbalSiyiSim
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData

        vehicle = Mock()
        vehicle.attitude = Attitude(0, 90, 0)

        data = GimbalData(att=Attitude(-15, 0, 0), roll_stabilize=True, pitch_stabilize=True)
        gimbal = GimbalSiyiSim(data, VehicleAttitudeReader(vehicle), Mock())
        gimbal.start()
        time.sleep(0.15)

        try:
            gimbal.set_motion_mode(0)  # MODE_LOCK
            time.sleep(0.1)
            g_data = gimbal.get_data()
            self.assertAlmostEqual(g_data.att.yaw, 0.0, delta=5.0)
        finally:
            gimbal.stop()


class TestDetectionCoordinatorTracking(unittest.TestCase):

    def test_start_tracking_forwards(self):
        from navpy.modules.vision.detection_identity_registry import (
            DetectionIdentityRegistry,
        )
        from navpy.modules.vision.tracking_command_router import TrackingCommandRouter

        d1, d2 = Mock(), Mock()
        coord = TrackingCommandRouter(
            [d1, d2], DetectionIdentityRegistry(), Mock(),
        )
        coord.start_tracking(7)
        d1.start_tracking.assert_called_once_with(7)
        d2.start_tracking.assert_called_once_with(7)

    def test_stop_tracking_forwards(self):
        from navpy.modules.vision.detection_identity_registry import (
            DetectionIdentityRegistry,
        )
        from navpy.modules.vision.tracking_command_router import TrackingCommandRouter

        d1, d2 = Mock(), Mock()
        coord = TrackingCommandRouter(
            [d1, d2], DetectionIdentityRegistry(), Mock(),
        )
        coord.stop_tracking()
        d1.stop_tracking.assert_called_once_with(to_neutral=True)
        d2.stop_tracking.assert_called_once_with(to_neutral=True)


class TestRealDetectorStartTracking(unittest.TestCase):
    """Guardrails for the real YOLO Detector.start_tracking."""

    def test_rejects_negative_obj_id(self):
        """Negative ids are reserved as GimbalNavigation arm() sentinels;
        the real-detector surface must reject them so a caller can't
        accidentally arm YOLO's lock on a non-existent track."""
        from navpy.modules.vision.real_detector_controls import (
            DetectorTrackingControl,
        )

        target_lock = Mock()
        control = DetectorTrackingControl(None, target_lock)

        with self.assertRaises(ValueError):
            control.start_tracking(-1)
        target_lock.force_lock.assert_not_called()


class _FakeGeoRef:
    """Returns a fixed world LOCK attitude — stands in for GeoRefCalc so the
    test exercises the GimbalNavigation.update_geo -> set_att path without the
    geodetic math."""

    def __init__(self, att: Attitude):
        self._att = att

    def calc_gimbal_lock_att_loc(self, uav_loc, target_loc, uav_att, g_data):
        return self._att


class TestSecondLaunchGeoPointing(unittest.TestCase):
    """End-to-end through GimbalNavigation + a REAL GimbalSiyiSim: a launch-1
    pixel-tracking session leaves the sim's stale-RATE guard armed
    (``_last_rate_wall`` set by ``set_rate``); on a SECOND launch the
    geo-pointing path (``start_geo_tracking`` -> ``update_geo`` -> ``set_att``)
    must re-point the gimbal at the target and NOT be canceled by that guard.

    This is the field scenario the user reported (peers aiming opposite the
    target / ``behind_cam`` on the second mission launch). Without the
    ``set_att`` ``_last_rate_wall`` reset, the 50 Hz guard fires every tick on
    launch 2 and the gimbal stays frozen at its launch-1 attitude.
    """

    def _build(self):
        from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_sim import (
            GimbalSiyiSim,
        )
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
        from navpy.modules.navigation.gimbal_navigation import GimbalNavigation

        vehicle = Mock()
        vehicle.attitude = Attitude(0, 0, 0)  # body == world for readback
        scheduler_cadence = SchedulerCadence(
            lambda: 10.0,
            high_resolution_timer=False,
        )
        gimbal = GimbalSiyiSim(
            GimbalData(att=Attitude(-45, 0, 0)),
            VehicleAttitudeReader(vehicle),
            Mock(),
            scheduler_cadence,
        )

        mount = Mock()
        mount.name = "siyi_zr10"
        mount.gimbal = gimbal
        mount.get_gimbal_data.side_effect = lambda: gimbal.get_data()
        mount.get_k.return_value = np.array(
            [[2000, 0, 1280], [0, 2000, 720], [0, 0, 1]], dtype=np.float64
        )
        mount.is_valid.return_value = True

        navigation = GimbalNavigation(
            mount,
            Mock(),
            tracking=GimbalTrackingSetup(GimbalRateTrackerConfig()),
            neutral_pitch_deg=-45.0,
        )
        return navigation, gimbal, vehicle

    def test_geo_repoints_on_second_launch_after_pixel_session(self):
        navigation, gimbal, vehicle = self._build()
        gimbal.start()
        try:
            # --- Launch 1: pixel tracking slews the gimbal to negative yaw and
            #     arms the stale-RATE guard (set_rate sets _last_rate_wall).
            navigation.start_tracking(0)
            gimbal.set_rate(-50, 0)
            time.sleep(0.15)
            navigation.stop_tracking()
            self.assertLess(
                gimbal.get_data().att.yaw, -5.0,
                "precondition: launch-1 pixel slew should leave negative yaw",
            )

            # --- Launch 2: geo-point at a target that requires +60deg world yaw.
            geo_ref = _FakeGeoRef(Attitude(-30, 60, 0))
            target = Location(40.30, 44.43, 1280.0, is_absolute=True)
            uav_loc = Location(40.29, 44.43, 1480.0, is_absolute=True)
            navigation.start_geo_tracking(target, geo_ref)
            for _ in range(12):
                navigation.update_geo(uav_loc, vehicle.attitude)
                time.sleep(0.05)

            self.assertGreater(
                gimbal.get_data().att.yaw, 30.0,
                "launch-2 geo-pointing was canceled by the stale-RATE guard — "
                "gimbal stuck at the launch-1 attitude (the behind_cam bug)",
            )
        finally:
            navigation.stop_tracking()
            gimbal.stop()

    def test_gimbal_returns_to_neutral_yaw_after_stop(self):
        """After stop_tracking the gimbal must return to neutral yaw (0) so the
        NEXT launch starts from neutral, not the prior navigation task's attitude.
        SITL showed a repeated launch starting at yaw~90deg (stale) -> the gimbal
        scans/acquires from the wrong direction -> bad acquisition / recentre.
        This is the reset-state audit: drive the gimbal far from neutral, stop,
        and assert it actually slews back."""
        navigation, gimbal, vehicle = self._build()
        gimbal.start()
        try:
            navigation.start_tracking(0)
            gimbal.set_rate(70, 0)            # slew yaw far from neutral
            time.sleep(0.5)                   # ~5 sim-s at 10x
            moved = gimbal.get_data().att.yaw
            self.assertGreater(
                abs(moved), 30.0,
                f"precondition: gimbal should be far from neutral (yaw={moved:.1f})",
            )
            navigation.stop_tracking()          # -> FOLLOW + return_to_neutral(yaw=0)
            time.sleep(1.0)                   # let the physics slew back
            rest = gimbal.get_data().att.yaw
            self.assertLess(
                abs(rest), 5.0,
                f"gimbal yaw did not return to neutral after stop_tracking "
                f"(yaw={rest:.1f}) — stale attitude would carry into the next launch",
            )
        finally:
            navigation.stop_tracking()
            gimbal.stop()


if __name__ == '__main__':
    unittest.main()
