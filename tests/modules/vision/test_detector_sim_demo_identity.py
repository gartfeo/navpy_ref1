"""Simultaneous two-POI identity through DetectorSim -> DetectionCoordinator.

Live run 004227 permuted the 3-UAV demo assignments because NavController's
plain sensing request was built as ``DetectRequest(True)``: after the request
API change the first positional parameter is ``force_lock_id``, and Python
``True == 1`` forcibly selected local obj1 (WP3) as primary whenever it was
visible. With the plain ``DetectRequest()`` NavController now sends, the demo
identity contract must hold from geometry alone when obj0 (WP2, orbit-centered)
and obj1 (WP3, offset peer) are visible in the same frame:

- the orbit-centered POI obj0 is the primary/owner POI,
- task ids are allocated in navigation task order: obj0 -> task 1, obj1 -> task 2,

which is the fixed task-to-waypoint mapping the GCS demo evaluator checks.
"""
import unittest
from unittest.mock import Mock

import pymap3d

from navpy.args.uas_args import UasArgs
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detection_coordinator import DetectionCoordinator
from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.detector_sim import DetectorSim
from navpy.modules.vision.sim.gimbal_sim import GimbalSim
from tests.conftest import create_test_camera, create_mock_args

UAV_LAT = 40.30
UAV_LNG = 44.43
UAV_ALT = 1500.0


class TestSimultaneousTwoPoiDemoIdentity(unittest.TestCase):
    """obj0 centered + obj1 offset, both in one frame, real projection path."""

    def setUp(self):
        self._logger = ConsoleLogger()
        self._vehicle = Mock(spec=IVehicle)
        self._vehicle.get_param_or_default = Mock(return_value=1)
        self._vehicle.mission_items_count = 0
        self._vehicle.attitude = Attitude(0, 0, 0)
        self._vehicle.home_location = Location(UAV_LAT, UAV_LNG, 0.0)
        self._vehicle.get_mission_item_location = Mock(
            return_value=Location(UAV_LAT, UAV_LNG, 0.0),
        )

        camera = create_test_camera(
            image_width=2000, image_height=1000,
            fov=60, sensor_width=20, sensor_height=10,
        )
        gimbal = GimbalSim(
            GimbalData(att=Attitude(-45, 0, 0)),
            VehicleAttitudeReader(self._vehicle),
        )
        gimbal.set_att(Attitude(-45, 0, 0))
        mount = CameraMount(name="siyi_zr10", camera=camera, gimbal=gimbal)

        self._detector = DetectorSim(
            self._vehicle,
            mount,
            GeoRefCalc(UasArgs(), False),
            self._logger,
            create_mock_args(),
            zc_util=None,
        )

        # Demo frame: the owner orbits its own POI, so obj0 (WP2) sits on
        # the camera axis (level flight, gimbal -45deg, ground 1500 m below and
        # 1500 m ahead) while obj1 (WP3) is visible but offset to the side.
        self._detector.set_sim_poi(1, self._ground_poi(north_m=1500.0, east_m=0.0))
        self._detector.set_sim_poi(2, self._ground_poi(north_m=1500.0, east_m=350.0))

        self._detector.detect_pois(
            Location(UAV_LAT, UAV_LNG, UAV_ALT), Attitude(0, 0, 0),
            frame_timestamp_s=1.0,
        )

        self._coordinator = DetectionCoordinator([self._detector], Mock())

    def _ground_poi(self, north_m: float, east_m: float) -> Location:
        lat, lng, alt = pymap3d.ned2geodetic(
            north_m, east_m, UAV_ALT, UAV_LAT, UAV_LNG, UAV_ALT,
        )
        # home_location.alt is 0, so this relative altitude is also absolute.
        return Location(lat, lng, alt)

    def test_plain_request_keeps_centered_poi_as_owner(self):
        """The exact NavController sensing request must give owner WP2/obj0."""
        resp = self._coordinator.get_detect_data(DetectRequest())

        self.assertEqual(len(resp.detected_pois), 2)
        self.assertIsNotNone(resp.primary_poi)
        self.assertEqual(resp.primary_poi.identity.obj_id, 0)
        self.assertIs(resp.detected_pois[0], resp.primary_poi)

    def test_plain_request_allocates_demo_task_mapping(self):
        """Task ids must follow navigation task order: obj0->task1, obj1->task2."""
        resp = self._coordinator.get_detect_data(DetectRequest())

        task_by_obj = {
            poi.identity.obj_id: poi.identity.task_id
            for poi in resp.detected_pois
        }
        self.assertEqual(task_by_obj, {0: 1, 1: 2})
        self.assertEqual(resp.primary_poi.identity.task_id, 1)

    def test_boolean_request_reproduces_the_fixed_permutation_bug(self):
        """``DetectRequest(True)`` means force_lock_id=1 and permutes the owner.

        This is the exact run-004227 signature (start_tracking task_id=1 ->
        obj_id=1): the boolean request force-locks local obj1/WP3 even though
        obj0/WP2 is dead-center, and obj1 then receives task id 1. Guards the
        mechanism so a boolean never silently becomes a force-lock again.
        """
        resp = self._coordinator.get_detect_data(DetectRequest(True))

        self.assertEqual(resp.primary_poi.identity.obj_id, 1)
        self.assertEqual(resp.primary_poi.identity.task_id, 1)


if __name__ == "__main__":
    unittest.main()
