"""Tests for CameraIntrinsics zoom interpolation/extrapolation."""
import unittest

import numpy as np

from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics


def _make_zoom_map():
    """Two calibrated zoom levels for interpolation tests."""
    return {
        "1": {
            "fx": 2000.0, "fy": 2000.0,
            "cx": 960.0, "cy": 540.0,
            "skew": 0.0,
            "dist": [-0.3, 0.5, 0.0, 0.0, -0.4],
        },
        "3": {
            "fx": 4000.0, "fy": 4000.0,
            "cx": 980.0, "cy": 530.0,
            "skew": 0.0,
            "dist": [-0.1, 0.2, 0.0, 0.0, -0.1],
        },
    }


class TestCalibratedZoom(unittest.TestCase):
    def test_calibrated_zoom_accepted(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertTrue(cam.set_zoom("1"))
        self.assertAlmostEqual(cam.get_k()[0, 0], 2000.0)
        self.assertTrue(cam.set_zoom("3"))
        self.assertAlmostEqual(cam.get_k()[0, 0], 4000.0)


class TestInterpolatedZoom(unittest.TestCase):
    def test_midpoint_interpolation(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertTrue(cam.set_zoom("2"))
        k = cam.get_k()
        self.assertAlmostEqual(k[0, 0], 3000.0)  # midpoint of 2000 and 4000
        self.assertAlmostEqual(k[1, 1], 3000.0)
        self.assertAlmostEqual(k[0, 2], 970.0)    # midpoint of 960 and 980
        self.assertAlmostEqual(k[1, 2], 535.0)    # midpoint of 540 and 530

    def test_quarter_interpolation(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertTrue(cam.set_zoom("1.5"))
        k = cam.get_k()
        self.assertAlmostEqual(k[0, 0], 2500.0)  # 25% between 2000 and 4000

    def test_extrapolation_beyond_max(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertTrue(cam.set_zoom("5"))
        k = cam.get_k()
        # Extrapolate: fx = 4000 + (5-3)/(3-1) * (4000-2000) = 4000 + 2000 = 6000
        self.assertAlmostEqual(k[0, 0], 6000.0)

    def test_extrapolation_below_min(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertTrue(cam.set_zoom("0.5"))
        k = cam.get_k()
        # t = (0.5 - 1) / (3 - 1) = -0.25
        # fx = 2000 + (-0.25) * 2000 = 1500
        self.assertAlmostEqual(k[0, 0], 1500.0)

    def test_distortion_uses_nearest(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        cam.set_zoom("1.5")  # closer to zoom 1
        dist = cam.get_dist()
        self.assertAlmostEqual(dist[0], -0.3)  # zoom 1 dist

        cam.set_zoom("2.5")  # closer to zoom 3
        dist = cam.get_dist()
        self.assertAlmostEqual(dist[0], -0.1)  # zoom 3 dist


class TestSingleLevelExtrapolation(unittest.TestCase):
    def test_single_level_scales_focal_length(self):
        zoom_map = {
            "1": {
                "fx": 2000.0, "fy": 2000.0,
                "cx": 960.0, "cy": 540.0,
                "skew": 0.0, "dist": None,
            },
        }
        cam = CameraIntrinsics(zoom_map=zoom_map)
        self.assertTrue(cam.set_zoom("2"))
        k = cam.get_k()
        self.assertAlmostEqual(k[0, 0], 4000.0)  # 2x scale
        self.assertAlmostEqual(k[0, 2], 960.0)    # cx unchanged

    def test_single_level_half_zoom(self):
        zoom_map = {
            "2": {
                "fx": 4000.0, "fy": 4000.0,
                "cx": 960.0, "cy": 540.0,
                "skew": 0.0, "dist": None,
            },
        }
        cam = CameraIntrinsics(zoom_map=zoom_map)
        self.assertTrue(cam.set_zoom("1"))
        k = cam.get_k()
        self.assertAlmostEqual(k[0, 0], 2000.0)  # 0.5x scale

    def test_subclass_scale_policy_remains_an_open_extension_point(self):
        class CustomScale(CameraIntrinsics):
            @staticmethod
            def _scale_entry(entry, scale):
                scaled = dict(entry)
                scaled["fx"] = 123.0
                scaled["fy"] = 456.0
                return scaled

        cam = CustomScale(zoom_map={"1": _make_zoom_map()["1"]})

        self.assertTrue(cam.set_zoom("2"))
        self.assertAlmostEqual(cam.get_k()[0, 0], 123.0)
        self.assertAlmostEqual(cam.get_k()[1, 1], 456.0)


class TestInterpolationExtensionPoints(unittest.TestCase):
    def test_subclass_scalar_interpolator_is_used(self):
        class KeepLowerCalibration(CameraIntrinsics):
            @staticmethod
            def _lerp(a, b, t):
                del b, t
                return a

        cam = KeepLowerCalibration(zoom_map=_make_zoom_map())

        self.assertTrue(cam.set_zoom("2"))
        self.assertAlmostEqual(cam.get_k()[0, 0], 2000.0)


class TestInvalidZoom(unittest.TestCase):
    def test_non_numeric_zoom_rejected(self):
        cam = CameraIntrinsics(zoom_map=_make_zoom_map())
        self.assertFalse(cam.set_zoom("abc"))

    def test_empty_zoom_map_accepts_anything(self):
        cam = CameraIntrinsics(fx=1000, fy=1000, cx=500, cy=500)
        self.assertTrue(cam.set_zoom("10"))


if __name__ == "__main__":
    unittest.main()
