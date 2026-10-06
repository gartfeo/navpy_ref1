import math
import unittest
from unittest.mock import Mock

import numpy as np
import pymap3d
from numpy.testing import assert_allclose
from pymavlink.mavextra import wrap_180

from navpy.args.uas_args import UasArgs
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.rotation_utils import normalize
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.sim.detector_sim import DetectorSim
from navpy.modules.vision.sim.gimbal_sim import GimbalSim
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation
from tests.conftest import create_test_camera, create_mock_args


def _create_mount_from_gimbal(
    gimbal: GimbalSim,
    camera,
    name: str = "test_mount",
) -> CameraMount:
    """Create a CameraMount from a GimbalSim."""
    return CameraMount(name=name, camera=camera, gimbal=gimbal)


class GeoRefCalcTest(unittest.TestCase):
    def __init__(self, tests=()):
        super().__init__(tests)
        self._vehicle = Mock(spec=IVehicle)
        self._vehicle.get_param_or_default = Mock(return_value=1)
        self._vehicle.mission_items_count = 0
        self._logger = ConsoleLogger()

    def test_simple_reverse(self):
        camera = create_test_camera(image_width=1920, image_height=1080, fov=60, sensor_width=4.8, sensor_height=3.6)
        geo_ref = GeoRefCalc(UasArgs(), enable_log=False)

        uas_att = Attitude(-5, 15, 5)
        g_att = Attitude(-35, 0, 0)

        rot_att = Attitude(-45, 0, 15)
        seq = 'XYZ'
        r = Rotation.from_euler(seq, get_euler_by_sequence(rot_att, seq), degrees=True).as_matrix()
        expected_ned = r @ np.array([[100], [0], [0]])
        expected_ned = expected_ned[:, 0]

        k = camera.get_k()
        g_data = GimbalData(att=g_att)
        u, v = geo_ref.calc_uv(expected_ned, k, g_data, uas_att)
        actual_ned = geo_ref.calc_ned(u, v, k, g_data, uas_att)

        assert_allclose(normalize(actual_ned), normalize(expected_ned), atol=1e-3)

    def test_with_direct(self):
        camera = create_test_camera(image_width=1920, image_height=1080, fov=60, sensor_width=4.8, sensor_height=3.6)
        gimbal = GimbalSim(
            GimbalData(att=Attitude(-40, 0, 0)),
            VehicleAttitudeReader(self._vehicle),
        )

        geo_ref = GeoRefCalc(UasArgs())
        mount = _create_mount_from_gimbal(gimbal, camera, "test_mount")
        args = create_mock_args()
        detector = DetectorSim(self._vehicle, mount, geo_ref, self._logger, args, zc_util=None)

        current_loc = Location(40.311347, 44.452192, 1566.9)

        target_loc = Location(40.312205, 44.455597, 1293.0)
        target = SimulationObject(1, target_loc, 0)

        uas_att = Attitude(-0.2655273659243745, 57.745888979386166, -25.837936271619576)
        g_att = Attitude(-40, 0, 0)

        gimbal.set_att(g_att)
        detect_data = detector.update(current_loc, target, uas_att)

        self.assertIsNotNone(detect_data)
        self.assertIsNotNone(detect_data.pixel.u_px)
        self.assertIsNotNone(detect_data.pixel.v_px)

        expected_ned = pymap3d.geodetic2ned(target_loc.lat, target_loc.lng, target_loc.alt,
                                            current_loc.lat, current_loc.lng, current_loc.alt)
        g_data = GimbalData(att=g_att)
        actual_ned = geo_ref.calc_ned(
            detect_data.pixel.u_px,
            detect_data.pixel.v_px,
            detect_data.optics.camera_matrix(),
            g_data,
            uas_att,
        )

        assert_allclose(normalize(actual_ned), normalize(expected_ned), atol=1e-3)

        actual_yaw, actual_pitch = geo_ref.calc_yaw_pitch_proj(actual_ned, uas_att)

        self.assertLess(actual_yaw, 0)
        self.assertGreater(actual_pitch, 0)

    def test_get_vector_should_return_correct_sequence(self):
        # yzx
        att = Attitude(15, 30, 45)
        actual_vector = get_euler_by_sequence(att, 'XYZ')
        self.assertEqual(actual_vector, [45, 15, 30])

        actual_vector = get_euler_by_sequence(att, 'ZYX')
        self.assertEqual(actual_vector, [30, 15, 45])

    def test_ned_simple(self):
        camera = create_test_camera(image_width=2000, image_height=1000, fov=60, sensor_width=20, sensor_height=10)
        camera.set_zoom(1)
        k = camera.get_k()
        uas_args = UasArgs()
        uas_args.refresh()
        geo_ref = GeoRefCalc(uas_args)
        u = 1000
        v = 500

        g_data = GimbalData(att=Attitude(-90, 0, 0))
        actual_ned = geo_ref.calc_ned(u, v, k, g_data, Attitude(0, 0, 0))
        assert_allclose(actual_ned, [0, 0, 1], atol=1e-8)

    def test_uv_simple(self):
        camera = create_test_camera(image_width=2000, image_height=1000, fov=60, sensor_width=20, sensor_height=10)
        camera.set_zoom(12.5)
        k = camera.get_k()
        geo_ref = GeoRefCalc(UasArgs())

        u = 10
        v = 20

        g_data = GimbalData(att=Attitude(0, 0, 0))
        expected_ned = geo_ref.calc_ned(u, v, k=k, g_data=g_data, uas_att=Attitude(-90, 0, 0))

        actual_uv = geo_ref.calc_uv(expected_ned, k=k, g_data=g_data, uas_att=Attitude(-90, 0, 0))
        expected_uv = u, v
        assert_allclose(actual_uv, expected_uv)

    def test_ned_uv(self):
        cases = [
            [1391, 55, 0.66991199, 0.20249819, 9.81900698],
            [1396, 126, 0.60846897, 0.22941642, 9.82599566],
            [1297, 128, 0.5753323, 0.14164517, 9.82738023],
            [1293, 57, 0.6370921, 0.11561991, 9.82037957]
        ]

        camera = create_test_camera(image_width=1980, image_height=1088, fov=60,
                          sensor_width=3.97778183701707, sensor_height=2.186610335467904)

        camera.set_zoom(2.12)
        k = camera.get_k()
        geo_ref = GeoRefCalc(UasArgs(degrees=False, uas_seq='xyz',
                                     utm_x=0, utm_y=0, utm_z=8.88), enable_log=False)
        g_att = Attitude(0.00116, 0.00176, 0.00138)
        uas_att = Attitude(-1.466422693619277240, 6.046293468769378, - 0.1061368580805083922)

        g_data = GimbalData(
            att=g_att,
            g_seq='xyz', degrees=False,
            setup=GimbalMountSetup(
                att=Attitude(90, 90, 0), seq='zxy',
                degrees=True, dist=[-0.002, 0.023, 0.002]))

        for u, v, x, y, z in cases:
            actual_ned = geo_ref.calc_ned(u, v, k, g_data=g_data, uas_att=uas_att)
            expected_ned = [x, y, z]
            assert_allclose(actual_ned, expected_ned, atol=2e-4)

            actual_uv = geo_ref.calc_uv(expected_ned, k, g_data=g_data, uas_att=uas_att)
            expected_uv = u, v
            assert_allclose(actual_uv, expected_uv, atol=1)

    def test_ned_uv2(self):
        camera = create_test_camera(image_width=2448, image_height=2048, fov=60,
                          sensor_width=8.6, sensor_height=7.194771241830065)
        camera.set_zoom(12.5)
        k = camera.get_k()
        assert_allclose(k, [[3558.1395, 0, 1224], [0, 3558.1395, 1024], [0, 0, 1]])
        geo_ref = GeoRefCalc(UasArgs(degrees=False, uas_seq='xyz',
                                     utm_x=31.72212, utm_y=-6.55099, utm_z=42.44889), enable_log=False)
        u = 1095
        v = 1099
        g_att = Attitude(-math.pi / 3, -math.pi / 2, 0)
        u_att = Attitude(0, 0, 0)

        g_data = GimbalData(att=g_att, degrees=False, g_seq='xyz',
                            setup=GimbalMountSetup(
                                att=Attitude(0, 90, 90), seq='xyz',
                                dist=[0.3, 0, 0.2], degrees=True))
        actual_ned = geo_ref.calc_ned(u=u, v=v, k=k, g_data=g_data, uas_att=u_att)
        expected_ned = [31.9858651, -7.03273554, 43.52545462]
        assert_allclose(actual_ned, expected_ned, atol=1e-5)

        actual_uv = geo_ref.calc_uv(expected_ned, k=k, g_data=g_data, uas_att=u_att)
        expected_uv = u, v
        assert_allclose(actual_uv, expected_uv, atol=1e-3)

    def test_ned_uv_corner_cases(self):
        camera = create_test_camera(image_width=2000, image_height=1000, fov=60, sensor_width=20, sensor_height=10)
        camera.set_zoom(12.5)
        k = camera.get_k()
        geo_ref = GeoRefCalc(UasArgs(), False)
        g_data = GimbalData(att=Attitude(0, 0, 0))
        cases = [
            [1000, 500, 0, 0, 1],
            [1000, 500, 0, 0, 1],
            [0, 0, 0.4, -0.8, 1],
            [2000, 1000, -0.4, 0.8, 1],
            # [2001, 0, None, None, 1],
            # [0, 1001, None, None, 1],
            # [0, -1, None, None, 1],
            # [-1, 0, None, None, 1],
            # [None, None, 0, -0.81, 1],
            # [None, None, -0.41, 0, 1],
            # [None, None, 0, 0.81, 1],
            # [None, None, 0.41, 0, 1],
        ]
        att = Attitude(-90, 0, 0)
        for case in cases:
            u, v, x, y, z = case
            # print(u, v, x, y, z)
            expected_ned = [x, y, z]
            if u is not None or v is not None:
                actual_ned = geo_ref.calc_ned(u, v, k=k, g_data=g_data, uas_att=att)

                if x is None or y is None:
                    self.assertEqual(actual_ned[0], None, msg=f'case: {case}')
                    self.assertEqual(actual_ned[1], None, msg=f'case: {case}')
                    self.assertEqual(actual_ned[2], None, msg=f'case: {case}')
                else:
                    assert_allclose(actual_ned, expected_ned, atol=1e-5)

            if x is not None or y is not None:
                actual_uv = geo_ref.calc_uv(expected_ned, k=k, g_data=g_data, uas_att=att)

                expected_uv = u, v

                if u is None or v is None:
                    self.assertEqual(actual_uv[0], None)
                    self.assertEqual(actual_uv[1], None)
                else:
                    assert_allclose(actual_uv, expected_uv, atol=1e-5)

    def test_back_side_should_not_be_visible(self):
        geo_ref = GeoRefCalc(UasArgs(), False)
        p_ned = [0, 0, -200]
        uas_att = Attitude(pitch=0, yaw=0, roll=0)
        g_data = GimbalData(att=Attitude(0, 0, 0),
                            setup=GimbalMountSetup(att=Attitude(0, 0, 0)))
        k = np.array([[1000, 0, 1000], [0, 1000, 500], [0, 0, 1]])
        u, v = geo_ref.calc_uv(p_ned, k, g_data=g_data, uas_att=uas_att)
        self.assertIsNone(u)
        self.assertIsNone(v)

    def test_lock_command_projects_target_to_center_with_siyi_mount(self):
        geo_ref = GeoRefCalc(UasArgs(), False)
        target_ned = np.array([-461.8, -429.9, 158.7])
        uas_att = Attitude(-2.0, -48.0, 5.0)
        g_data = GimbalData(att=Attitude(0.0, 0.0, 0.0))
        k = np.array([
            [1000.0, 0.0, 960.0],
            [0.0, 1000.0, 540.0],
            [0.0, 0.0, 1.0],
        ])

        command = geo_ref.calc_gimbal_lock_att_ned(target_ned, uas_att, g_data)

        self.assertAlmostEqual(
            command.yaw,
            math.degrees(math.atan2(target_ned[1], target_ned[0])),
            places=2,
        )
        self.assertLess(command.pitch, 0.0)

        readback = geo_ref.calc_gimbal_lock_readback(command, uas_att, g_data)
        uv = geo_ref.calc_uv(target_ned, k, readback, uas_att)
        assert_allclose(uv, (960.0, 540.0), atol=1.0)

        axis_ned = geo_ref.calc_ned(960.0, 540.0, k, readback, uas_att)
        assert_allclose(normalize(axis_ned), normalize(target_ned), atol=1e-6)

    def test_lock_command_location_path_uses_world_yaw(self):
        geo_ref = GeoRefCalc(UasArgs(), False)
        current_loc = Location(40.0, 44.0, 1000.0)
        target_ned = np.array([0.0, 100.0, 20.0])
        target_lat, target_lng, target_alt = pymap3d.ned2geodetic(
            target_ned[0], target_ned[1], target_ned[2],
            current_loc.lat, current_loc.lng, current_loc.alt,
        )
        target_loc = Location(target_lat, target_lng, target_alt)
        uas_att = Attitude(0.0, 80.0, 0.0)
        g_data = GimbalData(att=Attitude(0.0, 0.0, 0.0))

        command = geo_ref.calc_gimbal_lock_att_loc(
            current_loc, target_loc, uas_att, g_data,
        )

        self.assertAlmostEqual(command.yaw, 90.0, places=2)
        self.assertAlmostEqual(
            command.pitch,
            -math.degrees(math.atan2(target_ned[2], np.linalg.norm(target_ned[:2]))),
            places=2,
        )

    # -6.1 (-4.4, -5.3), -82.4 (76.3, -75.1),
    # c_l: lat=-35.362600,lon=149.165148,alt=307.8, t_l: lat=-35.363262,lon=149.165237,alt=0.4,
    # t_n: (-74.24956570254349, 8.277773240457933, 306.9495247912603), s_l: lat=-35.363269,lon=149.165239,alt=0.8,
    # u: p=-75.12108426557654,y=178.00533022412415,r=-5.2502131626193, actual_diff: 0.9
    def test_center_gimbal(self):
        c_l = Location(-35.362600, 149.165148, 307.8)
        t_l = Location(-35.363262, 149.165237, 0.4)
        u = Attitude(-75.12108426557654, 178.00533022412415, -5.2502131626193)
        t_n = pymap3d.geodetic2ned(t_l.lat, t_l.lng, t_l.alt, c_l.lat, c_l.lng, c_l.alt)

        camera = create_test_camera(image_width=1920, image_height=1080, fov=60, sensor_width=4.8, sensor_height=3.6)

        # gimbal
        gimbal_args1 = GimbalData(att=Attitude(-5, 0, 0))
        gimbal1 = GimbalSim(
            gimbal_args1,
            VehicleAttitudeReader(self._vehicle),
        )

        geo_ref_calc = GeoRefCalc(UasArgs(), False)
        u, v = geo_ref_calc.calc_uv(
            t_n,
            camera.get_k(),
            gimbal1.get_data(),
            u,
        )

        self.assertIsNotNone(u)
        self.assertGreaterEqual(u, 0)
        self.assertIsNotNone(v)
        self.assertGreaterEqual(v, 0)

    #  2510.3 - 8, 374: -60 (-43.4, 12.3), 0 (14.5, -3.5),
    #  c_l: lat=-35.341213,lon=149.162383,alt=500.0, t_l: lat=-35.363262,lon=149.165237,alt=0.4,
    #  --t_n: [ 1.1621353  -0.45734057  0.32199172]--, s_l: lat=-35.324239,lon=149.154233,alt=-21.7,
    #  g: p=-30,y=0,r=0, u: p=-3.5138862277180283,y=21.933030038211367,r=12.336559612830094,
    #  actual_diff: 4443.6, pitch_diff: 195.43, yaw_diff: 2.97
    def test_back_target(self):
        c_l = Location(-35.341213, 149.162383, 500)
        t_l = Location(-35.363262, 149.165237, 0.4)
        u = Attitude(-3.5138862277180283, 21.933030038211367, 12.336559612830094)
        t_n = pymap3d.geodetic2ned(t_l.lat, t_l.lng, t_l.alt, c_l.lat, c_l.lng, c_l.alt)

        camera = create_test_camera(image_width=1920, image_height=1080, fov=60, sensor_width=4.8, sensor_height=3.6)
        uas_seq = 'ZYX'

        # gimbal 1
        gimbal_args1 = GimbalData(att=Attitude(-30, 0, 0))
        gimbal1 = GimbalSim(
            gimbal_args1,
            VehicleAttitudeReader(self._vehicle),
            uas_seq,
        )

        geo_ref_calc = GeoRefCalc(UasArgs(), enable_log=False)

        i = geo_ref_calc.calc_uv(
            t_n,
            camera.get_k(),
            gimbal1.get_data(),
            u,
        )
        self.assertIsNone(i[0], 0)
        self.assertIsNone(i[1], 0)

    def test_pitch_sign_north_south(self):
        geo_ref = GeoRefCalc(UasArgs(), enable_log=False)

        for yaw in [0, 180]:
            target_att_u = Attitude(70, yaw, 0)
            target_att_d = Attitude(-70, yaw, 0)

            uas_att_u = Attitude(30, yaw, 0)
            uas_att_d = Attitude(-30, yaw, 0)

            _, pitch = geo_ref.calc_yaw_pitch_proj_att(target_att_u, uas_att_u)
            self.assertAlmostEqual(pitch, -40, msg=f'Yaw: {yaw}, pitch: {pitch}')

            _, pitch = geo_ref.calc_yaw_pitch_proj_att(target_att_u, uas_att_d)
            self.assertAlmostEqual(pitch, -100, msg=f'Yaw: {yaw}, pitch: {pitch}')

            _, pitch = geo_ref.calc_yaw_pitch_proj_att(target_att_d, uas_att_u)
            self.assertAlmostEqual(pitch, 100, msg=f'Yaw: {yaw}, pitch: {pitch}')

            _, pitch = geo_ref.calc_yaw_pitch_proj_att(target_att_d, uas_att_d)
            self.assertAlmostEqual(pitch, 40, msg=f'Yaw: {yaw}, pitch: {pitch}')

    def test_calc_yaw_pitch_yaw_pitch_45(self):
        wind_att = Attitude(0, 0, 0)
        uas_att = Attitude(0, 45, 0)
        yaw, pitch = GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj_att(wind_att, uas_att)
        self.assertAlmostEqual(yaw, 45)

    def test_rotation(self):
        p_ned = [1, 0, 0]

        # Create rotation from NED to UAS body frame
        r_uas_ned = Rotation.from_euler("ZYX", [-30, -60, 0], degrees=True)
        # Transform vector to body frame
        c_ned = r_uas_ned.apply(p_ned)

        # Recover the inverse rotation (body to NED)
        r_ned_uas = r_uas_ned.inv()
        # Apply full inverse rotation to bring back to NED
        a_ned = r_ned_uas.apply(c_ned)

        # Verify we get the original vector
        assert_allclose(a_ned, p_ned, atol=1e-7)

        # Optional: Verify Euler angles
        euler = r_ned_uas.as_euler('ZYZ', degrees=True)
        # Should be [30, 60, 0]
        assert_allclose(euler, [0, 60, 30], atol=1e-7)

    def test_all_possible_att(self):
        for wind_yaw in range(0, 1, 1):
            wind_att = Attitude(0, wind_yaw, 0)
            for pitch in range(-180, 181, 30):
                for roll in range(0, 1, 1):
                    for yaw in range(30, 31, 1):
                        uas_att = Attitude(pitch, yaw, roll)
                        GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj_att(wind_att, uas_att)
        print('done')

    def test_calc_yaw_pitch_yaw_aligned(self):
        # check yaw
        target_yaw = 0
        target_pitch = 0

        curr_att = Attitude(0, 0, 0)
        target_att = Attitude(91, 0, 0)
        yaw, pitch = GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj_att(curr_att, target_att)

        if (not np.isclose(yaw, target_yaw)
                or not np.isclose(pitch, target_pitch)):
            print(f'initial: {0},{0}, '
                  f'yaw angle: {target_yaw}: {round(yaw, 2)}; '
                  f'pitch: {round(pitch, 2)}; ')

    def test_calc_yaw_pitch_yaw_180_pitch(self):
        # check yaw
        target_yaw = 0
        target_pitch = 180

        curr_att = Attitude(0, 150, 0)
        target_att = Attitude(0, -30, 0)
        yaw, pitch = GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj_att(curr_att, target_att)

        if (not np.isclose(yaw, target_yaw)
                or not np.isclose(pitch, target_pitch)):
            # we expect pitch to be 180 in case yaw is 180 or -180 as we set yaw 0 manually
            print(f'initial: {150},{-30}, '
                  f'yaw angle: {target_yaw}: {round(yaw, 2)}; '
                  f'pitch: {round(pitch, 2)}; ')

    def test_optimize(self):
        """Simple check of rotation difference minimization."""

        def rotation_difference(angles, vector_a, vector_b):
            rot = Rotation.from_euler('ZY', angles, degrees=True)
            return np.linalg.norm(rot.apply(vector_a) - vector_b)

        vector_a = np.array([1, 0, 0])
        vector_b = Rotation.from_euler('Y', [57], degrees=True).apply(vector_a)

        best = None
        best_val = 1e9
        for yaw in np.linspace(-180, 180, 73):
            for pitch in np.linspace(-90, 90, 37):
                val = rotation_difference([yaw, pitch], vector_a, vector_b)
                if val < best_val:
                    best_val = val
                    best = (yaw, pitch)

        yaw, pitch = best
        Rotation.from_euler('ZY', [yaw, pitch], degrees=True).apply(vector_b)

    def test_calc_yaw_pitch_pitch(self):
        for initial_pitch in range(-180, 181, 30):
            for angle in range(-180, 181, 30):
                # check pitch
                target_pitch = wrap_180(angle - initial_pitch)
                target_ned = Rotation.from_euler('Y', initial_pitch, degrees=True).apply([1, 0, 0])
                uas_att = Attitude(angle, 0, 0)
                yaw, pitch = GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj(target_ned, uas_att)

                if not np.isclose(pitch, target_pitch) or not np.isclose(yaw, 0):
                    print(f'initial: {initial_pitch},{angle}, '
                          f'pitch angle: {target_pitch}: {round(pitch, 2)}; '
                          f'Yaw: {round(yaw, 2)}; ')

    def test_calc_yaw_pitch_yaw(self):
        for initial_yaw in range(-180, 181, 30):
            for angle in range(-180, 181, 30):
                # check pitch
                target_yaw = wrap_180(angle - initial_yaw)
                target_ned = Rotation.from_euler('Z', initial_yaw, degrees=True).apply([1, 0, 0])
                uas_att = Attitude(0, angle, 0)
                yaw, pitch = GeoRefCalc(UasArgs(), enable_log=False).calc_yaw_pitch_proj(target_ned, uas_att)

                if not np.isclose(yaw, target_yaw) or not np.isclose(pitch, 0):
                    print(f'initial: {initial_yaw},{angle}, '
                          f'yaw angle: {target_yaw}: {round(pitch, 2)}; '
                          f'pitch: {round(yaw, 2)}; ')


class CalcPitchLosTest(unittest.TestCase):
    """True-LOS pitch error for the vertical navigation channel."""

    def setUp(self):
        self._geo_ref = GeoRefCalc(UasArgs(), enable_log=False)

    def test_matches_projection_when_nose_on_target(self):
        # With zero heading error the body-XZ projection IS the LOS
        # elevation relative to body pitch: both formulations must agree.
        for elevation_deg in (5.0, 9.3, 20.0, 45.0):
            for uas_pitch in (0.0, -10.0, -36.8):
                horizontal = math.cos(math.radians(elevation_deg))
                down = math.sin(math.radians(elevation_deg))
                target_ned = np.array([horizontal, 0.0, down])
                uas_att = Attitude(uas_pitch, 0.0, 0.0)

                _, proj_pitch = self._geo_ref.calc_yaw_pitch_proj(target_ned, uas_att)
                los_pitch = self._geo_ref.calc_pitch_los(target_ned, uas_att)

                self.assertAlmostEqual(
                    los_pitch, proj_pitch, places=5,
                    msg=f"elev={elevation_deg} pitch={uas_pitch}",
                )

    def test_bounded_when_target_behind(self):
        # Field geometry of the 2026-06-12 NAV-entry aborts: target
        # ~9.3 deg below the horizon but ~97 deg off the nose. The XZ
        # projection exceeds 90 deg and railed the dive; the LOS pitch
        # must stay shallow (|los + pitch| bounded by real geometry).
        elevation = math.radians(9.3)
        bearing = math.radians(97.0)
        target_ned = np.array([
            math.cos(elevation) * math.cos(bearing),
            math.cos(elevation) * math.sin(bearing),
            math.sin(elevation),
        ])
        uas_att = Attitude(-36.8, 0.0, 0.0)

        _, proj_pitch = self._geo_ref.calc_yaw_pitch_proj(target_ned, uas_att)
        los_pitch = self._geo_ref.calc_pitch_los(target_ned, uas_att)

        # The projection reports a near-vertical (~90 deg) nose-down
        # rotation for a 9.3 deg elevation target — the dive-railing
        # artifact. The LOS pitch reports the real geometry.
        self.assertGreater(abs(proj_pitch), 80.0)
        self.assertAlmostEqual(los_pitch, 9.3 + (-36.8), places=3)

    def test_sign_conventions(self):
        level = Attitude(0.0, 0.0, 0.0)
        below = np.array([1.0, 0.0, 0.2])   # target below the horizon
        above = np.array([1.0, 0.0, -0.2])  # target above the horizon

        # Positive = nose-down rotation needed.
        self.assertGreater(self._geo_ref.calc_pitch_los(below, level), 0.0)
        self.assertLess(self._geo_ref.calc_pitch_los(above, level), 0.0)

        # Nose already on the LOS: no rotation needed.
        elevation = math.degrees(math.atan2(0.2, 1.0))
        on_los = Attitude(-elevation, 0.0, 0.0)
        self.assertAlmostEqual(
            self._geo_ref.calc_pitch_los(below, on_los), 0.0, places=6,
        )

    def test_straight_down_is_bounded(self):
        target_ned = np.array([0.0, 0.0, 1.0])
        result = self._geo_ref.calc_pitch_los(target_ned, Attitude(0.0, 0.0, 0.0))
        self.assertAlmostEqual(result, 90.0, places=3)


if __name__ == '__main__':
    unittest.main()
