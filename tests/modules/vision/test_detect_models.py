"""Tests for detection data models."""
import unittest
import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import (
    DetectStatus,
    DetectionSizeClass,
    DetectedObject,
    DetectResult,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.simulation_object import SimulationObject
from tests.detection_factory import make_detected_poi


class TestDetectStatus(unittest.TestCase):
    """Tests for DetectStatus enum."""

    def test_status_values(self):
        """DetectStatus has expected values."""
        self.assertEqual(DetectStatus.DETECTED.value, 0)
        self.assertEqual(DetectStatus.OutOfView.value, 2)


class TestDetectionSizeClass(unittest.TestCase):
    """Tests for DetectionSizeClass enum."""

    def test_size_class_values(self):
        """DetectionSizeClass has expected size values."""
        self.assertEqual(DetectionSizeClass.S.value, 1)
        self.assertEqual(DetectionSizeClass.M.value, 2)
        self.assertEqual(DetectionSizeClass.L.value, 3)
        self.assertEqual(DetectionSizeClass.XL.value, 4)


class TestDetectedObject(unittest.TestCase):
    """Tests for DetectedObject class."""

    def setUp(self):
        self.gimbal_data = GimbalData(att=Attitude(-45, 0, 0))
        self.uas_att = Attitude(0, 0, 0)
        self.k = np.eye(3)

    def test_init_required_fields(self):
        """DetectedObject initializes with required fields."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )

        self.assertEqual(poi.identity.obj_id, 1)
        self.assertEqual(poi.pixel.u_px, 100)
        self.assertEqual(poi.pixel.v_px, 50)
        self.assertEqual(poi.geo.reference_height_m, 2.0)

    def test_init_optional_fields_defaults(self):
        """DetectedObject has correct default values."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )

        self.assertIsNone(poi.geo.camera_location)
        self.assertEqual(poi.classification.size_class, DetectionSizeClass.L)
        self.assertEqual(poi.classification.class_id, 0)
        self.assertEqual(poi.classification.confidence, 1.0)
        self.assertIsNone(poi.geo.truth_poi_location)
        self.assertIsNone(poi.geo.projected_poi_location)
        self.assertIsNone(poi.confirmation.frame)
        self.assertIsNone(poi.confirmation.bbox_cxcywh)
        self.assertIsNone(poi.tracking.bbox_cxcywh)
        self.assertIsNone(poi.tracking.x_velocity_px_s)
        self.assertIsNone(poi.tracking.y_velocity_px_s)
        self.assertIsNone(poi.timing.detection_timestamp_s)
        self.assertIsNone(poi.timing.camera_frame_timestamp_s)
        self.assertIsNone(poi.optics.camera_frame_sequence)
        self.assertIsNone(poi.optics.sample_id)
        self.assertIsNone(poi.optics.zoom_command)
        self.assertIsNone(poi.timing.tracker_timestamp_s)
        self.assertIsNone(poi.pose.pose_timestamp_s)
        self.assertIsNone(poi.pose.vehicle_attitude_timestamp_s)
        self.assertIsNone(poi.pose.gimbal_attitude_timestamp_s)
        self.assertIsNone(poi.pose.pose_age_s)
        self.assertIsNone(poi.pose.is_frame_atomic)
        self.assertEqual(poi.pose.status, "unknown")
        self.assertTrue(poi.confirmation.supports_frame)

    def test_init_custom_optional_fields(self):
        """DetectedObject accepts custom optional fields."""
        c_loc = Location(40.0, -74.0, 100)
        t_loc = Location(40.1, -74.1, 50)

        poi = make_detected_poi(
            obj_id=5,
            x_error=200,
            y_error=150,
            reference_height_m=3.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att,
            c_g_loc=c_loc,
            size_class=DetectionSizeClass.S,
            class_id=2,
            confidence=0.85,
            t_g_loc_debug=t_loc,
            bbox_cxcywh=(180.0, 140.0, 60.0, 40.0),
            x_velocity=12.5,
            y_velocity=-3.0,
            timestamp=42.0,
            camera_frame_timestamp_s=41.9,
            camera_frame_sequence=17,
            camera_optics_sample_id="zoom:8",
            camera_zoom_command="2.0",
            tracker_timestamp_s=42.0,
            pose_timestamp_s=42.1,
            vehicle_attitude_timestamp_s=42.1,
            gimbal_attitude_timestamp_s=42.1,
            pose_age_s=0.2,
            pose_is_frame_atomic=False,
            pose_status="mixed_time_pose_snapshot",
            tracking_bbox_cxcywh=(200.0, 150.0, 80.0, 50.0),
        )

        self.assertEqual(poi.geo.camera_location, c_loc)
        self.assertEqual(poi.classification.size_class, DetectionSizeClass.S)
        self.assertEqual(poi.classification.class_id, 2)
        self.assertEqual(poi.classification.confidence, 0.85)
        self.assertEqual(poi.geo.truth_poi_location, t_loc)
        self.assertEqual(poi.confirmation.bbox_cxcywh, (180.0, 140.0, 60.0, 40.0))
        self.assertEqual(poi.tracking.bbox_cxcywh, (200.0, 150.0, 80.0, 50.0))
        self.assertEqual(poi.tracking.x_velocity_px_s, 12.5)
        self.assertEqual(poi.tracking.y_velocity_px_s, -3.0)
        self.assertEqual(poi.timing.detection_timestamp_s, 42.0)
        self.assertEqual(poi.timing.camera_frame_timestamp_s, 41.9)
        self.assertEqual(poi.optics.camera_frame_sequence, 17)
        self.assertEqual(poi.optics.sample_id, "zoom:8")
        self.assertEqual(poi.optics.zoom_command, "2.0")
        self.assertEqual(poi.timing.tracker_timestamp_s, 42.0)
        self.assertEqual(poi.pose.pose_timestamp_s, 42.1)
        self.assertEqual(poi.pose.vehicle_attitude_timestamp_s, 42.1)
        self.assertEqual(poi.pose.gimbal_attitude_timestamp_s, 42.1)
        self.assertEqual(poi.pose.pose_age_s, 0.2)
        self.assertFalse(poi.pose.is_frame_atomic)
        self.assertEqual(poi.pose.status, "mixed_time_pose_snapshot")

    def test_set_p_t_g_loc(self):
        """set_p_t_g_loc sets predicted POI location."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )

        location = Location(40.0, -74.0, 50)
        poi.set_p_t_g_loc(location)

        self.assertEqual(poi.geo.projected_poi_location, location)
        self.assertEqual(poi.geo.projected_poi_location.lat, 40.0)

    def test_to_dict(self):
        """to_dict returns dictionary representation."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )
        poi.set_p_t_g_loc(Location(40.0, -74.0, 50))

        result = poi.to_dict()

        self.assertIsInstance(result, dict)
        self.assertEqual(result['obj_id'], 1)
        self.assertEqual(result['x_error'], 100)
        self.assertEqual(result['y_error'], 50)
        self.assertEqual(result['reference_height_m'], 2.0)
        self.assertNotIn('poi_height', result)
        self.assertEqual(result['p_t_g_l']['lat'], 40.0)
        self.assertEqual(result['p_t_g_l']['lng'], -74.0)
        self.assertEqual(result['p_t_g_l']['alt'], 50)

    def test_to_dict_none_location(self):
        """to_dict handles None predicted location."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )

        result = poi.to_dict()

        self.assertIsNone(result['p_t_g_l']['lat'])
        self.assertIsNone(result['p_t_g_l']['lng'])
        self.assertIsNone(result['p_t_g_l']['alt'])

    def test_str_representation(self):
        """__str__ returns readable representation."""
        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=self.k,
            g_data=self.gimbal_data,
            uas_att=self.uas_att
        )

        result = str(poi)

        self.assertIn('1', result)
        self.assertIn('x_error', result)
        self.assertIn('100', result)
        self.assertIn('y_error', result)
        self.assertIn('50', result)


class TestDetectResult(unittest.TestCase):
    """Tests for DetectResult class."""

    def test_init_with_status_only(self):
        """DetectResult initializes with status only."""
        result = DetectResult(status=DetectStatus.OutOfView)

        self.assertEqual(result.status, DetectStatus.OutOfView)
        self.assertIsNone(result.poi)

    def test_init_with_poi(self):
        """DetectResult initializes with status and POI."""
        gimbal_data = GimbalData(att=Attitude(-45, 0, 0))
        uas_att = Attitude(0, 0, 0)
        k = np.eye(3)

        poi = make_detected_poi(
            obj_id=1,
            x_error=100,
            y_error=50,
            reference_height_m=2.0,
            k=k,
            g_data=gimbal_data,
            uas_att=uas_att
        )

        result = DetectResult(status=DetectStatus.DETECTED, poi=poi)

        self.assertEqual(result.status, DetectStatus.DETECTED)
        self.assertEqual(result.poi, poi)
        self.assertEqual(result.poi.identity.obj_id, 1)


class TestPoi(unittest.TestCase):
    """Tests for POI class."""

    def test_init(self):
        """POI initializes with all fields."""
        loc = Location(40.0, -74.0, 100)
        poi = SimulationObject(
            uid=1,
            loc_global=loc,
            height=5,
        )

        self.assertEqual(poi.uid, 1)
        self.assertEqual(poi.g_loc, loc)
        self.assertEqual(poi.height, 5)

    def test_multiple_pois(self):
        """Multiple POI instances are independent."""
        loc1 = Location(40.0, -74.0, 100)
        loc2 = Location(41.0, -75.0, 200)

        poi1 = SimulationObject(uid=1, loc_global=loc1, height=5)
        poi2 = SimulationObject(uid=2, loc_global=loc2, height=10)

        self.assertEqual(poi1.uid, 1)
        self.assertEqual(poi2.uid, 2)
        self.assertEqual(poi1.g_loc.lat, 40.0)
        self.assertEqual(poi2.g_loc.lat, 41.0)


if __name__ == '__main__':
    unittest.main()
