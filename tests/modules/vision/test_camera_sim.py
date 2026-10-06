"""Tests for create_test_camera helper and CameraIntrinsics."""
import unittest
import numpy as np

from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from tests.conftest import create_test_camera, FovCamera


class TestCreateFovCamera(unittest.TestCase):
    """Tests for create_test_camera helper function."""

    def test_returns_test_camera(self):
        """create_test_camera returns FovCamera instance."""
        camera = create_test_camera()

        self.assertIsInstance(camera, FovCamera)

    def test_default_values(self):
        """create_test_camera uses sensible defaults."""
        camera = create_test_camera()

        self.assertEqual(camera.image_width, 1920)
        self.assertEqual(camera.image_height, 1080)

    def test_custom_values(self):
        """create_test_camera accepts custom values."""
        camera = create_test_camera(
            image_width=1280,
            image_height=720,
            fov=90.0,
            sensor_width=6.4,
            sensor_height=4.8
        )

        self.assertEqual(camera.image_width, 1280)
        self.assertEqual(camera.image_height, 720)

    def test_get_k_returns_3x3_matrix(self):
        """get_k returns 3x3 intrinsic matrix."""
        camera = create_test_camera()
        k = camera.get_k()

        self.assertEqual(k.shape, (3, 3))
        self.assertIsInstance(k, np.ndarray)

    def test_get_k_has_correct_structure(self):
        """get_k returns matrix with correct structure."""
        camera = create_test_camera(
            image_width=1920,
            image_height=1080
        )
        k = camera.get_k()

        # Principal point at center
        self.assertEqual(k[0, 2], 1920 / 2)
        self.assertEqual(k[1, 2], 1080 / 2)
        # Zero skew
        self.assertEqual(k[0, 1], 0)
        # Bottom row
        self.assertEqual(k[2, 0], 0)
        self.assertEqual(k[2, 1], 0)
        self.assertEqual(k[2, 2], 1)

    def test_is_valid_center_point(self):
        """is_valid returns True for center point."""
        camera = create_test_camera(image_width=1920, image_height=1080)

        self.assertTrue(camera.is_valid(960, 540))

    def test_is_valid_corners(self):
        """is_valid returns True for corner points."""
        camera = create_test_camera(image_width=1920, image_height=1080)

        self.assertTrue(camera.is_valid(0, 0))
        self.assertTrue(camera.is_valid(1920, 0))
        self.assertTrue(camera.is_valid(0, 1080))
        self.assertTrue(camera.is_valid(1920, 1080))

    def test_is_valid_out_of_bounds(self):
        """is_valid returns False for out of bounds points."""
        camera = create_test_camera(image_width=1920, image_height=1080)

        self.assertFalse(camera.is_valid(-1, 540))
        self.assertFalse(camera.is_valid(1921, 540))
        self.assertFalse(camera.is_valid(960, -1))
        self.assertFalse(camera.is_valid(960, 1081))

    def test_is_valid_none_values(self):
        """is_valid returns False for None values."""
        camera = create_test_camera()

        self.assertFalse(camera.is_valid(None, 540))
        self.assertFalse(camera.is_valid(960, None))
        self.assertFalse(camera.is_valid(None, None))


class FovCameraIntrinsics(unittest.TestCase):
    """Tests for CameraIntrinsics class."""

    def test_init_with_direct_values(self):
        """CameraIntrinsics initializes with direct fx, fy, cx, cy."""
        camera = CameraIntrinsics(
            fx=1000.0,
            fy=1000.0,
            cx=960.0,
            cy=540.0,
            image_width=1920,
            image_height=1080
        )

        self.assertEqual(camera.image_width, 1920)
        self.assertEqual(camera.image_height, 1080)

    def test_init_requires_all_intrinsics_without_zoom_map(self):
        """CameraIntrinsics raises when missing intrinsics without zoom_map."""
        with self.assertRaises(ValueError) as ctx:
            CameraIntrinsics(fx=1000.0, fy=1000.0)  # Missing cx, cy

        self.assertIn("cx, cy are required", str(ctx.exception))

    def test_init_with_zoom_map(self):
        """CameraIntrinsics initializes with zoom_map."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0},
            "2": {"fx": 2000.0, "fy": 2000.0, "cx": 960.0, "cy": 540.0}
        }
        camera = CameraIntrinsics(
            zoom_map=zoom_map,
            image_width=1920,
            image_height=1080
        )

        self.assertEqual(camera.image_width, 1920)
        self.assertEqual(camera.image_height, 1080)

    def test_get_k_returns_correct_matrix(self):
        """get_k returns matrix with correct intrinsic values."""
        camera = CameraIntrinsics(
            fx=1000.0,
            fy=1200.0,
            cx=960.0,
            cy=540.0
        )
        k = camera.get_k()

        self.assertEqual(k[0, 0], 1000.0)
        self.assertEqual(k[1, 1], 1200.0)
        self.assertEqual(k[0, 2], 960.0)
        self.assertEqual(k[1, 2], 540.0)

    def test_get_k_returns_copy(self):
        """get_k returns a copy (modifying doesn't affect internal state)."""
        camera = CameraIntrinsics(fx=1000.0, fy=1000.0, cx=960.0, cy=540.0)
        k1 = camera.get_k()
        k1[0, 0] = 9999.0

        k2 = camera.get_k()
        self.assertEqual(k2[0, 0], 1000.0)

    def test_get_dist_default_zeros(self):
        """get_dist returns zeros when no distortion specified."""
        camera = CameraIntrinsics(fx=1000.0, fy=1000.0, cx=960.0, cy=540.0)
        dist = camera.get_dist()

        self.assertEqual(len(dist), 5)
        np.testing.assert_array_equal(dist, np.zeros(5))

    def test_get_dist_with_distortion(self):
        """get_dist returns specified distortion coefficients."""
        dist_coeffs = [0.1, -0.2, 0.001, 0.002, 0.05]
        camera = CameraIntrinsics(
            fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
            dist=dist_coeffs
        )
        dist = camera.get_dist()

        np.testing.assert_array_almost_equal(dist, dist_coeffs)

    def test_set_zoom_changes_intrinsics(self):
        """set_zoom changes intrinsics from zoom_map."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0},
            "2": {"fx": 2000.0, "fy": 2000.0, "cx": 960.0, "cy": 540.0}
        }
        camera = CameraIntrinsics(zoom_map=zoom_map, image_width=1920, image_height=1080)

        k1 = camera.get_k()
        self.assertEqual(k1[0, 0], 1000.0)

        result = camera.set_zoom("2")
        self.assertTrue(result)

        k2 = camera.get_k()
        self.assertEqual(k2[0, 0], 2000.0)

    def test_set_zoom_uncalibrated_level_extrapolates(self):
        """set_zoom extrapolates intrinsics for uncalibrated zoom level."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
        }
        camera = CameraIntrinsics(zoom_map=zoom_map, image_width=1920, image_height=1080)

        result = camera.set_zoom("5")

        self.assertTrue(result)
        self.assertAlmostEqual(camera.get_k()[0, 0], 5000.0)

    def test_set_zoom_non_numeric_returns_false(self):
        """set_zoom returns False for non-numeric zoom value."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
        }
        camera = CameraIntrinsics(zoom_map=zoom_map, image_width=1920, image_height=1080)

        result = camera.set_zoom("abc")

        self.assertFalse(result)

    def test_set_zoom_no_zoom_map(self):
        """set_zoom stores value when no zoom_map."""
        camera = CameraIntrinsics(fx=1000.0, fy=1000.0, cx=960.0, cy=540.0)

        result = camera.set_zoom(2.0)

        self.assertTrue(result)

    def test_is_valid_center_point(self):
        """is_valid returns True for center point."""
        camera = CameraIntrinsics(
            fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
            image_width=1920, image_height=1080
        )

        self.assertTrue(camera.is_valid(960, 540))

    def test_is_valid_out_of_bounds(self):
        """is_valid returns False for out of bounds points."""
        camera = CameraIntrinsics(
            fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
            image_width=1920, image_height=1080
        )

        self.assertFalse(camera.is_valid(-1, 540))
        self.assertFalse(camera.is_valid(1921, 540))

    def test_is_valid_none_dimensions_always_true(self):
        """is_valid returns True when dimensions not specified."""
        camera = CameraIntrinsics(fx=1000.0, fy=1000.0, cx=960.0, cy=540.0)

        self.assertTrue(camera.is_valid(9999, 9999))

    def test_is_valid_none_values(self):
        """is_valid returns False for None values."""
        camera = CameraIntrinsics(
            fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
            image_width=1920, image_height=1080
        )

        self.assertFalse(camera.is_valid(None, 540))
        self.assertFalse(camera.is_valid(960, None))

    def test_skew_in_matrix(self):
        """Skew coefficient is included in matrix."""
        camera = CameraIntrinsics(
            fx=1000.0, fy=1000.0, cx=960.0, cy=540.0,
            skew=0.5
        )
        k = camera.get_k()

        # skew * fx
        self.assertEqual(k[0, 1], 0.5 * 1000.0)

    def test_zoom_map_missing_keys_raises(self):
        """zoom_map entry missing required keys raises ValueError."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0}  # Missing cx, cy
        }

        with self.assertRaises(ValueError) as ctx:
            CameraIntrinsics(zoom_map=zoom_map)

        self.assertIn("missing keys", str(ctx.exception))

    def test_zoom_map_infers_dimensions(self):
        """zoom_map infers dimensions from cx, cy if not provided."""
        zoom_map = {
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0}
        }
        camera = CameraIntrinsics(zoom_map=zoom_map)

        # Dimensions inferred as 2 * cx, 2 * cy
        self.assertEqual(camera.image_width, 1920)
        self.assertEqual(camera.image_height, 1080)


class FovCameraIntrinsicsDistortion(unittest.TestCase):
    """Tests for CameraIntrinsics distortion handling."""

    def test_distortion_from_zoom_map(self):
        """Distortion is loaded from zoom_map entry."""
        dist_coeffs = [-0.3, 0.5, 0.001, 0.002, -0.6]
        zoom_map = {
            "1": {
                "fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0,
                "dist": dist_coeffs
            }
        }
        camera = CameraIntrinsics(zoom_map=zoom_map, image_width=1920, image_height=1080)

        dist = camera.get_dist()
        np.testing.assert_array_almost_equal(dist, dist_coeffs)

    def test_distortion_changes_with_zoom(self):
        """Distortion changes when zoom level changes."""
        zoom_map = {
            "1": {
                "fx": 1000.0, "fy": 1000.0, "cx": 960.0, "cy": 540.0,
                "dist": [0.1, 0.2, 0.0, 0.0, 0.0]
            },
            "2": {
                "fx": 2000.0, "fy": 2000.0, "cx": 960.0, "cy": 540.0,
                "dist": [0.3, 0.4, 0.0, 0.0, 0.0]
            }
        }
        camera = CameraIntrinsics(zoom_map=zoom_map, image_width=1920, image_height=1080)

        dist1 = camera.get_dist()
        self.assertAlmostEqual(dist1[0], 0.1)

        camera.set_zoom("2")
        dist2 = camera.get_dist()
        self.assertAlmostEqual(dist2[0], 0.3)


if __name__ == '__main__':
    unittest.main()
