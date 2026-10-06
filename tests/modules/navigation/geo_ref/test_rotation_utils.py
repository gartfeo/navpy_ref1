import unittest

import numpy as np
from numpy.testing import assert_allclose
from pymavlink.mavextra import wrap_180
from navpy.utils.simple_rotation import Rotation

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.rotation_utils import get_euler_rotation_angles, normalize, \
    calculate_euler_angles, _calculate_yaw, _calculate_pitch_after_yaw, angle_between_vectors_proj
from navpy.utils.euler_utils import get_euler_by_sequence


class RotationUtilsTestCase(unittest.TestCase):
    def test_calc_yaw_90(self):
        v1 = [1, 0, 0]
        v2 = [0, 1, 0]
        yaw = _calculate_yaw(v1, v2)
        self.assertEqual(yaw, 90)

    def test_calc_yaw_0(self):
        v1 = [1, 0, 0]
        v2 = [0, 0, -1]
        yaw = _calculate_yaw(v1, v2)
        self.assertEqual(yaw, 0)

    def test_calc_yaw_180(self):
        v1 = [1, 0, 0]
        v2 = [-1, 0, 0]
        yaw = _calculate_yaw(v1, v2)
        self.assertEqual(yaw, 180)

    def test_calc_pitch_90(self):
        v1 = [1, 0, 0]
        v2 = [0, 0, 1]
        pitch = _calculate_pitch_after_yaw(v1, v2, 0)
        self.assertEqual(pitch, -90)

    def test_calc_pitch_0(self):
        v1 = [1, 0, 0]
        v2 = [0, 1, 0]
        pitch = _calculate_pitch_after_yaw(v1, v2, 0)
        self.assertEqual(pitch, 0)

    def test_calc_pitch_180(self):
        v1 = [1, 0, 0]
        v2 = [-1, 0, 0]
        pitch = _calculate_pitch_after_yaw(v1, v2, 180)
        self.assertEqual(pitch, 0)

    def test_convert_extrinsic(self):
        setup_angle_seq = 'xyz'
        att = Attitude(0, 90, 90)
        r = Rotation.from_euler(setup_angle_seq, get_euler_by_sequence(att, setup_angle_seq), degrees=True)
        expected = r.as_matrix()

        new_seq = r.as_euler('XYZ', degrees=True)
        actual = Rotation.from_euler('XYZ', new_seq, degrees=True).as_matrix()
        print(new_seq)
        assert_allclose(actual, expected, atol=1e-7)

    def test_euler_simple(self):
        i_v = [0, 1, 0]
        b_v = [1, 1, 0]

        euler = get_euler_rotation_angles(i_v, b_v, seq='YZX', degrees=True)

        r_i_b = Rotation.from_euler('YZX', euler, degrees=True).as_matrix()
        actual_b_v = r_i_b @ i_v

        assert_allclose(actual_b_v, normalize(b_v), atol=1e-7)

    def test_euler(self):
        p_base = [100, 0, 300]
        r_b_to_uas = Rotation.from_euler('YZX', [30, 20, 0], degrees=True).as_matrix()
        p_uas = r_b_to_uas @ p_base

        r_uas_to_target = Rotation.from_euler('ZYX', [10, 40, 0], degrees=True).as_matrix()
        p_target = r_uas_to_target @ p_uas

        seq = 'XYZ'
        euler = get_euler_rotation_angles(p_uas, p_target, seq=seq, degrees=True)

        # test
        actual_r_uas_to_target = Rotation.from_euler(seq, euler, degrees=True).as_matrix()
        actual_p_target = actual_r_uas_to_target @ p_uas

        assert_allclose(actual_p_target, p_target, atol=1e-7)

    def test_calculate_euler_angles(self):
        uas_euler_angles = np.array([0, 10, -10])
        # uas_euler_angles = np.array([0, 0.2, 0.1])
        # target_euler_angles = np.array([10, -30, 0])
        target_euler_angles = np.array([0, -20, 0])
        seq = 'XYZ'

        gimbal_euler_angles_to_target = calculate_euler_angles('ZYX', uas_euler_angles,
                                                               seq, target_euler_angles,
                                                               True)

        i = [1, 0, 0]

        r_uas_v = Rotation.from_euler('ZYX', uas_euler_angles, degrees=True).as_matrix()
        p_uas = r_uas_v @ i

        r_g_v = Rotation.from_euler(seq, target_euler_angles, degrees=True).as_matrix()
        p_g = r_g_v @ i

        r_uas_g = Rotation.from_euler(seq, gimbal_euler_angles_to_target, degrees=True).as_matrix()
        p_uas_g = r_uas_g @ p_uas

        np.testing.assert_allclose(p_g, p_uas_g, rtol=1e-5, atol=1e-8)

    def test_gimbal(self):
        seq = 'XYZ'
        uas_euler = [1, -2, 3]

        i = [1, 0, 0]

        r = Rotation.from_euler(seq, uas_euler).as_matrix()
        p = r @ i

        r = Rotation.from_euler(seq[::-1], [-x for x in uas_euler[::-1]]).as_matrix()
        p_actual = r @ p

        assert_allclose(p_actual, i, rtol=1e-5, atol=1e-8)

    def test_angle_between_vectors(self):
        v1 = [1, 1, 0]
        v2 = [1, 0, 0]

        angle = angle_between_vectors_proj(v1, v2, 'yaw')
        self.assertAlmostEqual(angle, -45)

    def test_calculate_yaw(self):
        v1 = [1, 0, 1]
        v2 = [-1, 0, 1]
        yaw = wrap_180(_calculate_yaw(v1, v2))
        self.assertAlmostEqual(yaw, 180)

    @staticmethod
    def equations(p, a, b, h1, h2):
        x, y, z = p
        eq1 = (x ** 2 + y ** 2) / a ** 2 - (z - h1) ** 2
        eq2 = (x ** 2 + y ** 2) / b ** 2 - (z + h2) ** 2
        return [eq1, eq2, eq1 - eq2]

if __name__ == '__main__':
    unittest.main()
