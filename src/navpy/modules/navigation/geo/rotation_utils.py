import numpy as np
from pymavlink.mavextra import wrap_180
from navpy.utils.simple_rotation import Rotation


def normalize(v):
    dist = np.linalg.norm(v)
    if dist == 0:
        return v
    return v / np.linalg.norm(v)


def calculate_yaw_pitch(v1, v2):
    v1_n, v2_n = normalize(v1), normalize(v2)
    yaw = _calculate_yaw(v1_n, v2_n)
    pitch = _calculate_pitch(v1_n, v2_n)
    # pitch = _calculate_pitch_after_yaw(v1_n, v2_n, yaw)

    return yaw, pitch


def _calculate_yaw(v1_n, v2_n):
    yaw = angle_between_vectors_proj(v1_n, v2_n, 'yaw')
    return yaw


def _calculate_pitch_after_yaw(v1_n, v2_n, yaw):
    if yaw != 0:
        v1_rotated_n = Rotation.from_euler('Z', yaw, degrees=True).apply(v1_n)
    else:
        v1_rotated_n = v1_n

    if np.allclose(v1_rotated_n, v2_n):
        return 0

    return _calculate_pitch(v1_rotated_n, v2_n)


def _calculate_pitch(v1_n, v2_n):
    return angle_between_vectors_proj(v1_n, v2_n, 'pitch')


def angle_between_vectors_proj(v1, v2, axis):
    if axis == 'yaw':
        # Project onto XY plane by ignoring Z
        v1_projected = np.array([v1[0], v1[1], 0])
        v2_projected = np.array([v2[0], v2[1], 0])
        if (v1[0] == 0 and v1[1] == 0) or (v2[0] == 0 and v2[1] == 0):
            return 0
        normal = np.array([0, 0, 1])
    elif axis == 'pitch':
        # Project onto XZ plane by ignoring Y
        v1_projected = np.array([v1[0], 0, v1[2]])
        v2_projected = np.array([v2[0], 0, v2[2]])
        if (v1[0] == 0 and v1[2] == 0) or (v2[0] == 0 and v2[2] == 0):
            return 0
        normal = np.array([0, 1, 0])
    else:
        raise ValueError("Invalid plane specified. Choose 'yaw' or 'pitch'.")

    # Check if either vector is zero
    if np.allclose(v1_projected, 0) or np.allclose(v2_projected, 0):
        return 0  # Returning 0 when one or both vectors are zero

    # Calculate the angle using arctan2 for better numerical stability
    cross_product = np.cross(v1_projected, v2_projected)
    dot_product = np.dot(v1_projected, v2_projected)
    angle_rad = np.arctan2(np.linalg.norm(cross_product), dot_product)

    # No need to determine the sign with arctan2, as it inherently considers the sign
    # But adjust the angle based on the specified axis if needed
    if np.dot(cross_product, normal) < 0:
        angle_rad = -angle_rad

    # Convert angle to degrees
    angle_deg = np.degrees(angle_rad)

    return angle_deg


def get_euler_rotation_angles(start_look_at_vector, target_look_at_vector,
                              seq, degrees=True,
                              start_up_vector=None, target_up_vector=None):
    def find_additional_vertical_vector(vector):
        ez = np.array([0, 0, 1])
        look_at_vector = normalize(vector)
        up_vector = normalize(ez - np.dot(look_at_vector, ez) * look_at_vector)
        return up_vector

    def calc_rotation_matrix(v1_start, v2_start, v1_target, v2_target):
        """
        calculating M the rotation matrix from base U to base V
        M @ U = V
        M = V @ U^-1
        """

        def get_base_matrices():
            u1_start = normalize(v1_start)
            u2_start = normalize(v2_start)
            u3_start = normalize(np.cross(u1_start, u2_start))

            u1_target = normalize(v1_target)
            u2_target = normalize(v2_target)
            u3_target = normalize(np.cross(u1_target, u2_target))

            U = np.hstack([u1_start.reshape(3, 1), u2_start.reshape(3, 1), u3_start.reshape(3, 1)])
            V = np.hstack([u1_target.reshape(3, 1), u2_target.reshape(3, 1), u3_target.reshape(3, 1)])

            return U, V

        def calc_base_transition_matrix():
            return np.dot(V, np.linalg.inv(U))

        if not np.isclose(np.dot(v1_target, v2_target), 0, atol=1e-03):
            raise ValueError("v1_target and v2_target must be vertical")

        U, V = get_base_matrices()
        return calc_base_transition_matrix()

    if start_up_vector is None:
        start_up_vector = find_additional_vertical_vector(start_look_at_vector)

    if target_up_vector is None:
        target_up_vector = find_additional_vertical_vector(target_look_at_vector)

    rot_mat = calc_rotation_matrix(start_look_at_vector, start_up_vector, target_look_at_vector, target_up_vector)
    # is_equal = np.allclose(rot_mat @ start_look_at_vector, target_look_at_vector, atol=1e-03)
    # print(f"rot_mat @ start_look_at_vector1 == target_look_at_vector1 is {is_equal}")
    rotation = Rotation.from_matrix(rot_mat)
    return rotation.as_euler(seq, degrees)


def calculate_euler_angles(uas_seq, uas_euler_angles, target_seq, target_euler_angles, degrees):
    uas_rotation = Rotation.from_euler(uas_seq, uas_euler_angles, degrees)
    target_rotation = Rotation.from_euler(target_seq, target_euler_angles, degrees)

    uas_rotation_matrix = uas_rotation.as_matrix()
    target_rotation_matrix = target_rotation.as_matrix()

    gimbal_rotation_matrix = target_rotation_matrix @ np.transpose(uas_rotation_matrix)
    gimbal_rotation = Rotation.from_matrix(gimbal_rotation_matrix)
    # Wrap the Euler angles to the range [-pi, pi] (or [-180°, 180°] for degrees)
    gimbal_euler_angles = gimbal_rotation.as_euler(target_seq, degrees)
    wrapped_gimbal_euler_angles = np.array([wrap_180(angle) for angle in gimbal_euler_angles])
    #
    # x1, y, x2 = gimbal_rotation.as_euler(target_seq, degrees)
    # wrapped_gimbal_euler_angles = np.array([wrap_angle(x1 + x2, degrees), wrap_angle(y, degrees), 0])

    return wrapped_gimbal_euler_angles
