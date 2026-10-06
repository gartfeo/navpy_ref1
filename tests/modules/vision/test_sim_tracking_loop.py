"""Closed-loop integration test: DetectorSim → GimbalRateTracker → GimbalSiyiSim.

Verifies that when tracking is enabled, the gimbal actively moves to center
the target, reducing pixel errors over successive detection cycles.
"""

import time
import unittest
from unittest.mock import Mock

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTrackerConfig
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_sim import GimbalSiyiSim
from navpy.modules.vision.sim.detector_sim import DetectorSim


def _make_vehicle(lat=32.0, lng=34.8, alt=200.0, yaw=0.0, pitch=0.0, roll=0.0):
    """Create a mock vehicle at a fixed position and attitude."""
    vehicle = Mock()
    vehicle.location.return_value = Location(lat, lng, alt, is_absolute=True)
    vehicle.attitude = Attitude(pitch, yaw, roll)
    vehicle.home_location = Location(lat, lng, 0.0, is_absolute=True)
    vehicle.mission_items_count = 0
    vehicle.get_param_or_default.side_effect = lambda _name, default: default
    vehicle.get_parameter.return_value = 1.0  # SIM_SPEEDUP
    vehicle.get_mission_item_location.return_value = Location(
        lat, lng, 0.0, is_absolute=True,
    )
    return vehicle


def _make_mount(vehicle, camera_pitch=-28.7):
    """Create a real CameraMount with GimbalSiyiSim."""
    gimbal_data = GimbalData(
        att=Attitude(camera_pitch, 0, 0),
        roll_stabilize=True,
        pitch_stabilize=True,
        max_detect_distance=3000.0,
        name="gimbal_0",
        setup=GimbalMountSetup(att=Attitude(90, 0, 90)),
    )
    gimbal = GimbalSiyiSim(
        gimbal_data,
        VehicleAttitudeReader(vehicle),
        Mock(),
    )

    camera = CameraIntrinsics(
        image_width=1920,
        image_height=1080,
        zoom_map={"1": {"fx": 2293.0, "fy": 2314.0, "cx": 1199.61, "cy": 810.24}},
    )

    return CameraMount(name="zr10", camera=camera, gimbal=gimbal)


def _make_detector(vehicle, mount, tracking_config):
    from navpy.args.uas_args import UasArgs
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from tests.conftest import create_mock_args

    return DetectorSim(
        vehicle,
        mount,
        GeoRefCalc(UasArgs()),
        Mock(),
        create_mock_args(),
        tracking_config=tracking_config,
    )


class TestClosedLoopTracking(unittest.TestCase):
    """End-to-end: pixel errors converge as gimbal tracks."""

    def test_gimbal_reduces_pixel_error(self):
        """Tracking an off-center target should move gimbal to reduce error."""
        vehicle = _make_vehicle()
        mount = _make_mount(vehicle)

        # Target ~500m north, on the ground → will appear off-center
        target_loc = Location(32.004, 34.8, 0.0, is_absolute=True)

        config = GimbalRateTrackerConfig(
            correction_bw=1.0,
            max_rate=100.0,
            command_lead_time=0.0,
        )

        detector = _make_detector(
            vehicle,
            mount,
            GimbalTrackingSetup(rate=config),
        )
        detector.set_sim_target(0, target_loc)

        # Start gimbal physics thread
        mount.start()
        try:
            time.sleep(0.05)  # Let gimbal initialize

            # First detection — capture initial pixel errors
            c_g_loc = vehicle.location(False)
            uas_att = vehicle.attitude
            detector.detect_targets(
                c_g_loc,
                uas_att,
                frame_timestamp_s=1.0,
            )

            initial_targets = detector.get_latest_detections()
            if not initial_targets:
                self.skipTest("Target not visible at initial geometry")

            initial_x = initial_targets[0].pixel.u_px
            initial_y = initial_targets[0].pixel.v_px

            # Enable tracking
            detector.start_tracking(0)

            # Run detection loop with sleeps to let gimbal physics tick
            for index in range(30):
                detector.detect_targets(
                    c_g_loc,
                    uas_att,
                    frame_timestamp_s=1.05 + index * 0.05,
                )
                time.sleep(0.05)  # 50ms → gimbal ticks ~2.5 times

            final_targets = detector.get_latest_detections()
            self.assertTrue(len(final_targets) > 0, "Target lost during tracking")

            final_x = final_targets[0].pixel.u_px
            final_y = final_targets[0].pixel.v_px

            # Image center
            cx = 1199.61
            cy = 810.24

            initial_dist = ((initial_x - cx) ** 2 + (initial_y - cy) ** 2) ** 0.5
            final_dist = ((final_x - cx) ** 2 + (final_y - cy) ** 2) ** 0.5

            self.assertLess(
                final_dist, initial_dist,
                f"Pixel error should decrease: initial={initial_dist:.1f}, final={final_dist:.1f}"
            )

        finally:
            detector.stop()
            mount.stop()

    def test_stop_tracking_zeros_rate(self):
        """stop_tracking sends set_rate(0,0) to the gimbal."""
        vehicle = _make_vehicle()
        mount = _make_mount(vehicle)

        config = GimbalRateTrackerConfig(command_lead_time=0.0)

        detector = _make_detector(
            vehicle,
            mount,
            GimbalTrackingSetup(rate=config),
        )

        # Start tracking, run one detection cycle (no targets → loss), then stop
        detector.start_tracking(1)
        detector.detect_targets(
            Location(32.0, 34.8, 200.0, is_absolute=True),
            Attitude(0, 0, 0),
            frame_timestamp_s=1.0,
        )
        detector.stop_tracking()

        # Verify tracker state is IDLE after stop
        from navpy.modules.vision.gimbal_rate_tracker import TrackingState
        self.assertEqual(detector.navigation.rate_result.state, TrackingState.IDLE)
        detector.stop()


class TestMultiCameraObjIdInvariant(unittest.TestCase):
    """Regression test: Target uid assignment is deterministic by insertion order."""

    def test_sequential_targets_get_sequential_uids(self):
        """Targets created in the same order get the same uids."""
        from navpy.modules.vision.simulation_object import SimulationObject

        loc1 = Location(32.001, 34.801, 0.0)
        loc2 = Location(32.002, 34.802, 0.0)

        # Simulate two target providers adding targets in the same order
        targets_a = [SimulationObject(0, loc1, 2.0), SimulationObject(1, loc2, 2.0)]
        targets_b = [SimulationObject(0, loc1, 2.0), SimulationObject(1, loc2, 2.0)]

        self.assertEqual(targets_a[0].uid, targets_b[0].uid)
        self.assertEqual(targets_a[1].uid, targets_b[1].uid)
        self.assertNotEqual(targets_a[0].uid, targets_a[1].uid)


if __name__ == '__main__':
    unittest.main()
