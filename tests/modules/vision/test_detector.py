import unittest
from unittest.mock import Mock

import numpy as np
import pymap3d

from navpy.args.uas_args import UasArgs
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.sim.detector_sim import DetectorSim
from navpy.modules.vision.sim.gimbal_sim import GimbalSim
from navpy.modules.vision.simulation_object import SimulationObject
from tests.conftest import create_test_camera, create_mock_args
from tests.detection_factory import make_detected_target


def _create_mount_from_gimbal(
    gimbal: GimbalSim,
    camera,
    name: str = "test_mount",
) -> CameraMount:
    """Create a CameraMount from a GimbalSim."""
    return CameraMount(name=name, camera=camera, gimbal=gimbal)


class DetectorTestCase(unittest.TestCase):
    def __init__(self, tests=()):
        super().__init__(tests)
        self._vehicle = Mock(spec=IVehicle)
        self._vehicle.get_param_or_default = Mock(return_value=1)
        self._vehicle.mission_items_count = 0
        self._logger = ConsoleLogger()

        att = Attitude(-45, 0, 0)

        self._camera = create_test_camera(
            image_width=2000,
            image_height=1000,
            fov=60,
            sensor_width=20,
            sensor_height=10,
        )

        self._gimbal = GimbalSim(
            GimbalData(att=Attitude(-40, 0, 0)),
            VehicleAttitudeReader(self._vehicle),
        )

        self._geo_ref_calc = GeoRefCalc(UasArgs(), False)

        # Create CameraMount from gimbal
        mount = _create_mount_from_gimbal(
            self._gimbal,
            self._camera,
            "test_mount",
        )

        # Create detector with single mount
        args = create_mock_args()
        self._detector = DetectorSim(
            self._vehicle,
            mount,
            self._geo_ref_calc,
            self._logger,
            args,
            zc_util=None,
        )
        self._zc_util = ZcUtil()

        locations = self.get_location(1800, att)
        self._current_location = locations[0]
        self._target_location = locations[1]
        self._target = SimulationObject(1, self._target_location, 0)

    def test_multi_detector_pick_closest_to_center(self):
        """Test that with multiple detectors (one per mount), we can pick the best detection."""
        uas_att = Attitude(-25, 0, 0)
        camera = create_test_camera(image_width=1920, image_height=1080, fov=60, sensor_width=4.8, sensor_height=3.6)
        uas_seq = 'ZYX'

        # gimbal 1 - pitch 30 (far from target)
        gimbal_data1 = GimbalData(att=Attitude(-40, 0, 0))
        attitude_reader = VehicleAttitudeReader(self._vehicle)
        gimbal1 = GimbalSim(gimbal_data1, attitude_reader, uas_seq)
        gimbal1.set_att(Attitude(30, 0, 0))

        # gimbal 2 - pitch -5 (closest to target)
        gimbal_data2 = GimbalData(att=Attitude(-40, 0, 0))
        gimbal2 = GimbalSim(gimbal_data2, attitude_reader, uas_seq)
        gimbal2.set_att(Attitude(-5, 0, 0))

        # gimbal 3 - pitch -15
        gimbal_data3 = GimbalData(att=Attitude(-40, 0, 0))
        gimbal3 = GimbalSim(gimbal_data3, attitude_reader, uas_seq)
        gimbal3.set_att(Attitude(-15, 0, 0))

        geo_ref_calc = GeoRefCalc(UasArgs(), False)

        # Create mounts and separate detectors for each
        mount1 = _create_mount_from_gimbal(gimbal1, camera, "mount1")
        mount2 = _create_mount_from_gimbal(gimbal2, camera, "mount2")
        mount3 = _create_mount_from_gimbal(gimbal3, camera, "mount3")

        args = create_mock_args()
        detector1 = DetectorSim(self._vehicle, mount1, geo_ref_calc, self._logger, args)
        detector2 = DetectorSim(self._vehicle, mount2, geo_ref_calc, self._logger, args)
        detector3 = DetectorSim(self._vehicle, mount3, geo_ref_calc, self._logger, args)

        current_loc = Location(40.3116676, 44.4551189, 1800)
        target_loc = self._zc_util.ray_to_terrain(current_loc, uas_att)
        target = SimulationObject(1, target_loc, 0)

        # Get detection from each detector
        detect1 = detector1.update(current_loc, target, uas_att)
        detect2 = detector2.update(current_loc, target, uas_att)
        detect3 = detector3.update(current_loc, target, uas_att)

        # Pick the detection closest to image center (simulating VisionController aggregation)
        detections = [(detect1, mount1), (detect2, mount2), (detect3, mount3)]
        valid_detections = [(d, m) for d, m in detections if d is not None]

        # Find best detection (closest to center)
        best_detection = None
        min_center_error = float('inf')
        for detect, mount in valid_detections:
            center_y = mount.image_height / 2
            error = abs(detect.pixel.v_px - center_y)
            if error < min_center_error:
                min_center_error = error
                best_detection = detect

        # gimbal2 with pitch -5 should be closest to center
        self.assertIsNotNone(best_detection)
        self.assertAlmostEqual(best_detection.pose.gimbal_data.att.pitch, -5)

    def test_reverse_compatibility_simple(self):
        uas_att = Attitude(-45, 0, 0)
        g_att = Attitude(0, 0, 0)

        target_att = Attitude(-45, 0, 0)

        locations = self.get_location(2000, target_att)
        current_location = locations[0]
        target_location = locations[1]
        target = SimulationObject(1, target_location, 0)

        self._camera.set_zoom(1)
        self._gimbal.set_att(g_att)

        detect_data = self._detector.update(current_location, target, uas_att)
        x_error, y_error = detect_data.pixel.u_px, detect_data.pixel.v_px

        g_data = self._gimbal.get_data()
        actual_ned = self._geo_ref_calc.calc_ned(
            x_error,
            y_error,
            detect_data.optics.camera_matrix(),
            g_data=g_data,
            uas_att=uas_att,
        )

        actual_loc = self._zc_util.ray_to_terrain_ned(current_location, actual_ned)

        # the NED is considered same as UAS
        self.assertAlmostEqual(actual_loc.lat, target_location.lat, delta=1e-8)
        self.assertAlmostEqual(actual_loc.lng, target_location.lng, delta=1e-8)
        self.assertAlmostEqual(current_location.alt - actual_loc.alt, current_location.alt - target_location.alt,
                               delta=1e-8)

        dist = np.linalg.norm(pymap3d.geodetic2ned(target_location.lat, target_location.lng, target_location.alt,
                                                   actual_loc.lat, actual_loc.lng, actual_loc.alt))
        self.assertLess(dist, 0.15)

    def test_pitch_error_same_location_gimbal(self):
        uas_att = Attitude(-45, 0, 0)
        g_att = Attitude(-45, 0, 0)
        locations = self.get_location(1300, Attitude(-90, 0, 0))
        target = SimulationObject(1, locations[1], 0)
        self._gimbal.set_att(g_att)

        detect_data = self._detector.update(locations[0], target, uas_att)

        self.assertAlmostEqual(1000, detect_data.pixel.u_px, delta=0.1)
        self.assertAlmostEqual(500, detect_data.pixel.v_px, delta=0.1)

    def test_pitch_error(self):
        uas_att = Attitude(-90, 0, 0)
        self._gimbal.set_att(Attitude(0, 0, 0))
        self._camera.set_zoom(1)

        detect_data = self._detector.update(self._current_location, self._target, uas_att)

        self.assertAlmostEqual(detect_data.pixel.v_px, 400)

    def test_pitch_error_with_actualPitch(self):
        uas_att = Attitude(-25, 0, 0)
        self._gimbal.set_att(Attitude(-15, 0, 0))
        self._camera.set_zoom(1)

        detect_data = self._detector.update(self._current_location, self._target, uas_att)
        self.assertAlmostEqual(detect_data.pixel.v_px, 509)

    def test_roll_should_not_change(self):
        uas_att = Attitude(-15, 0, 0)
        self._gimbal.set_att(Attitude(-30, 0, 0))
        self._camera.set_zoom(1)

        detect_data = self._detector.update(self._current_location, self._target, uas_att)
        self.assertAlmostEqual(detect_data.pixel.u_px, 1000)

    def test_roll_error_actual_roll(self):
        uas_att = Attitude(-35, 10, 0)
        self._gimbal.set_att(Attitude(0, 0, 0))
        self._camera.set_zoom(1)

        detect_data = self._detector.update(self._current_location, self._target, uas_att)
        self.assertAlmostEqual(detect_data.pixel.u_px, 987)

    def test_get_detect_data_returns_primary_target_first(self):
        import threading

        from navpy.modules.vision.sim.detection_publication_buffer import (
            DetectionPublicationBuffer,
        )
        from navpy.modules.vision.sim.detection_publication_store import (
            DetectionPublicationStore,
        )

        primary = make_detected_target(obj_id=2)
        peer = make_detected_target(obj_id=1)
        store = DetectionPublicationStore(source_driven=False, capacity=1)
        slot = store.reserve(threading.Event(), threading.Event())
        self.assertIsNotNone(slot)
        self.assertTrue(store.publish(
            slot,
            [peer, primary],
            primary_target=primary,
            source_timestamp_s=1.0,
            source_receipt_timestamp_s=None,
            source_name="sim",
            source_discontinuity=False,
        ))
        buffer = DetectionPublicationBuffer(
            store,
            record_outcome=lambda *_: None,
            fallback_source_name=lambda: "sim",
        )
        resp = buffer.get_detect_data(DetectRequest())

        self.assertIs(resp.primary_target, primary)
        self.assertEqual(
            [target.identity.obj_id for target in resp.detected_targets],
            [2, 1],
        )

    def get_location(self, current_alt, att: Attitude, current_lat=40.3116676, current_lng=44.4551189):
        current_loc = Location(current_lat, current_lng, current_alt)
        target_loc = self._zc_util.ray_to_terrain(current_loc, att)
        return current_loc, target_loc


class TestDetectorGeoForwarders(unittest.TestCase):
    """Behavior of the explicit gimbal-control owners."""

    def setUp(self):
        from unittest.mock import Mock

        from navpy.modules.common.models.location import Location
        from navpy.modules.vision.real_detector_controls import (
            DetectorGeoControl,
            DetectorTrackingControl,
        )
        from navpy.modules.vision.real_gimbal_adapters import (
            GimbalGeoAdapter,
            GimbalTrackingAdapter,
        )

        self.navigation = Mock()
        self.geo = DetectorGeoControl(GimbalGeoAdapter(self.navigation))
        self.tracking = DetectorTrackingControl(
            GimbalTrackingAdapter(self.navigation),
            Mock(),
        )
        self.target_loc = Location(40.3, 44.4, 1500)
        self.uav_loc = Location(40.31, 44.41, 1700)
        self.uav_att = Mock()
        self.geo_ref = Mock()

    def test_start_geo_tracking_forwards_to_navigation(self):
        self.geo.start_geo_tracking(self.target_loc, self.geo_ref)
        self.navigation.start_geo_tracking.assert_called_once_with(
            self.target_loc, self.geo_ref
        )

    def test_update_geo_forwards_to_navigation(self):
        self.geo.update_geo(self.uav_loc, self.uav_att)
        self.navigation.update_geo.assert_called_once_with(
            self.uav_loc, self.uav_att
        )

    def test_prepare_geo_acquisition_forwards_to_navigation(self):
        self.navigation.prepare_geo_acquisition.return_value = True

        prepared = self.geo.prepare_geo_acquisition(
            self.uav_loc, self.uav_att, class_id=0, min_pixels=15.0,
        )

        self.assertTrue(prepared)
        self.navigation.prepare_geo_acquisition.assert_called_once_with(
            self.uav_loc, self.uav_att, 0, 15.0,
        )

    def test_stop_geo_tracking_forwards_to_navigation(self):
        self.geo.stop_geo_tracking()
        self.navigation.stop_geo_tracking.assert_called_once()

    def test_terminal_zoom_freeze_forwards_to_navigation(self):
        self.navigation.freeze_terminal_zoom_at_min.return_value = True
        self.tracking.freeze_terminal_zoom_at_min()
        self.navigation.freeze_terminal_zoom_at_min.assert_called_once_with()

    def test_no_op_when_navigation_is_none(self):
        from navpy.modules.vision.real_detector_controls import (
            DetectorGeoControl,
            DetectorTrackingControl,
        )

        geo = DetectorGeoControl(None)
        tracking = DetectorTrackingControl(None, Mock())
        geo.start_geo_tracking(self.target_loc, self.geo_ref)
        geo.update_geo(self.uav_loc, self.uav_att)
        tracking.freeze_terminal_zoom_at_min()
        self.assertFalse(geo.prepare_geo_acquisition(self.uav_loc, self.uav_att, 0, 15.0))
        geo.stop_geo_tracking()
        self.assertFalse(geo.is_geo_armed)
        self.assertFalse(geo.is_detection_armed)

    def test_forwarder_propagates_start_failure(self):
        self.navigation.start_geo_tracking.side_effect = RuntimeError("locked")
        with self.assertRaises(RuntimeError):
            self.geo.start_geo_tracking(self.target_loc, self.geo_ref)

    def test_armed_properties_forward(self):
        self.navigation.is_geo_armed = True
        self.navigation.is_detection_armed = False
        self.assertTrue(self.geo.is_geo_armed)
        self.assertFalse(self.geo.is_detection_armed)

    def test_get_zoom_result_returns_active_navigation_result(self):
        result = object()
        self.navigation.tracking_obj_id = 7
        self.navigation.zoom_result = result

        self.assertIs(self.tracking.get_zoom_result(7), result)
        self.assertIs(self.tracking.get_zoom_result(), result)

    def test_get_zoom_result_returns_none_for_other_obj_id(self):
        self.navigation.tracking_obj_id = 7
        self.navigation.zoom_result = object()

        self.assertIsNone(self.tracking.get_zoom_result(8))


if __name__ == '__main__':
    unittest.main()
