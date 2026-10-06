import unittest
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.geo_ray_diagnostics import compute_geo_ray_diagnostic
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class _Loc:
    def __init__(self, lat=40.0, lng=44.0, alt=1000.0):
        self.lat = lat
        self.lng = lng
        self.alt = alt


class _GeoRef:
    def __init__(self, uv=(100.0, 200.0), ray=(2.0, 0.0, 1.0)):
        self.uv = uv
        self.ray = np.array(ray, dtype=float)
        self.uv_calls = []
        self.ned_calls = []

    def calc_uv(self, p_ned, k, g_data, uas_att):
        self.uv_calls.append((p_ned, k, g_data, uas_att))
        return self.uv

    def calc_ned(self, u, v, k, g_data, uas_att):
        self.ned_calls.append((u, v, k, g_data, uas_att))
        return self.ray


class TestGeoRayDiagnostics(unittest.TestCase):
    def setUp(self):
        self.poi_loc = _Loc(40.1, 44.1, 950.0)
        self.uav_loc = _Loc(40.0, 44.0, 1000.0)
        self.uav_att = Attitude(0.0, 0.0, 0.0)
        self.g_data = GimbalData(att=Attitude(-10.0, 20.0, 0.0))
        self.k = np.array([
            [1000.0, 0.0, 960.0],
            [0.0, 1000.0, 540.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)

    def test_uses_geo_ref_projection_and_center_ray_for_plane_hit(self):
        geo_ref = _GeoRef(uv=(960.0, 540.0), ray=(2.0, 0.0, 1.0))

        with patch(
            "navpy.modules.vision.geo_ray_diagnostics.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 20.0, 50.0]),
        ):
            diag = compute_geo_ray_diagnostic(
                poi_loc=self.poi_loc,
                uav_loc=self.uav_loc,
                uav_att=self.uav_att,
                k=self.k,
                g_data=self.g_data,
                geo_ref=geo_ref,
                is_valid_pixel=lambda _u, _v: True,
            )

        self.assertEqual(diag.projection_status, "in_frame")
        self.assertEqual(diag.intersection_status, "poi_alt_hit")
        self.assertEqual(diag.poi_ned, (100.0, 20.0, 50.0))
        self.assertEqual(diag.optical_axis_ned, (2.0, 0.0, 1.0))
        self.assertEqual(diag.plane_hit_ned, (100.0, 0.0, 50.0))
        self.assertAlmostEqual(diag.plane_scale, 50.0)
        self.assertAlmostEqual(diag.lateral_miss_m, 20.0)

        uv_p_ned, _, _, _ = geo_ref.uv_calls[0]
        np.testing.assert_allclose(uv_p_ned, np.array([100.0, 20.0, 50.0]))
        center_u, center_v, _, _, _ = geo_ref.ned_calls[0]
        self.assertEqual(center_u, 960.0)
        self.assertEqual(center_v, 540.0)

    def test_marks_out_of_fov_from_validator(self):
        geo_ref = _GeoRef(uv=(2000.0, 540.0), ray=(2.0, 0.0, 1.0))

        with patch(
            "navpy.modules.vision.geo_ray_diagnostics.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 50.0]),
        ):
            diag = compute_geo_ray_diagnostic(
                poi_loc=self.poi_loc,
                uav_loc=self.uav_loc,
                uav_att=self.uav_att,
                k=self.k,
                g_data=self.g_data,
                geo_ref=geo_ref,
                is_valid_pixel=lambda _u, _v: False,
            )

        self.assertEqual(diag.projection_status, "out_of_fov")
        self.assertFalse(diag.poi_in_frame)

    def test_marks_behind_camera_without_calling_validator(self):
        geo_ref = _GeoRef(uv=(None, None), ray=(2.0, 0.0, 1.0))
        validator = Mock()

        with patch(
            "navpy.modules.vision.geo_ray_diagnostics.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 50.0]),
        ):
            diag = compute_geo_ray_diagnostic(
                poi_loc=self.poi_loc,
                uav_loc=self.uav_loc,
                uav_att=self.uav_att,
                k=self.k,
                g_data=self.g_data,
                geo_ref=geo_ref,
                is_valid_pixel=validator,
            )

        self.assertEqual(diag.projection_status, "behind_cam")
        self.assertIsNone(diag.poi_in_frame)
        validator.assert_not_called()

    def test_parallel_optical_axis_has_no_plane_hit(self):
        geo_ref = _GeoRef(uv=(960.0, 540.0), ray=(1.0, 0.0, 0.0))

        with patch(
            "navpy.modules.vision.geo_ray_diagnostics.pymap3d.geodetic2ned",
            return_value=np.array([100.0, 0.0, 50.0]),
        ):
            diag = compute_geo_ray_diagnostic(
                poi_loc=self.poi_loc,
                uav_loc=self.uav_loc,
                uav_att=self.uav_att,
                k=self.k,
                g_data=self.g_data,
                geo_ref=geo_ref,
                is_valid_pixel=lambda _u, _v: True,
            )

        self.assertEqual(diag.intersection_status, "parallel_to_poi_alt")
        self.assertIsNone(diag.plane_hit_ned)
        self.assertIsNone(diag.lateral_miss_m)


if __name__ == "__main__":
    unittest.main()
