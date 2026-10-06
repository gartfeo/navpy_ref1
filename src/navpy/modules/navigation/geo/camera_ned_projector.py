"""Bidirectional camera-pixel and NED-ray projection."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.uas_frame_transform import UasFrameTransform
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation

if TYPE_CHECKING:
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class CameraNedProjector:
    """Project through camera, gimbal, aircraft, and NED frames."""

    def __init__(
        self,
        frame: UasFrameTransform,
        enable_log: bool = False,
    ) -> None:
        self._frame = frame
        self._enable_log = enable_log
        self._inv_k = None
        self._cached_k = None

    def calc_ned(
        self,
        u: float,
        v: float,
        k: np.ndarray,
        g_data: GimbalData,
        uas_att: Attitude,
    ) -> np.ndarray:
        self._show("k", k)
        image = np.array([[u], [v], [1]])
        self._show("i", image)

        if self._inv_k is None or not np.array_equal(k, self._cached_k):
            self._inv_k = np.linalg.inv(k)
            self._cached_k = k.copy()
        camera_point = self._inv_k @ image
        self._show("p_c_prime", camera_point)

        camera_to_gimbal_translation = np.array([[0], [0], [0]])
        self._show("t_c_to_g", camera_to_gimbal_translation)
        camera_to_gimbal = Rotation.from_euler(
            g_data.args.setup_seq,
            get_euler_by_sequence(
                g_data.args.setup_att,
                g_data.args.setup_seq,
            ),
            degrees=g_data.args.setup_degrees,
        ).as_matrix()
        self._show("r_c_to_g", camera_to_gimbal)
        gimbal_point = (
            camera_to_gimbal @ camera_point + camera_to_gimbal_translation
        )
        self._show("p_g_prime", gimbal_point)

        gimbal_to_uas_translation = np.reshape(
            g_data.args.setup_dist,
            (3, 1),
        )
        self._show("t_g_to_uas", gimbal_to_uas_translation)
        gimbal_to_uas = Rotation.from_euler(
            g_data.args.g_seq,
            get_euler_by_sequence(g_data.att, g_data.args.g_seq),
            degrees=g_data.args.degrees,
        ).as_matrix()
        self._show("r_g_to_uas", gimbal_to_uas)
        uas_point = gimbal_to_uas @ gimbal_point + gimbal_to_uas_translation
        self._show("p_uas_prime", uas_point)

        uas_to_ned_translation = self._frame.translation_column()
        self._show("t_uas_to_ned", uas_to_ned_translation)
        uas_to_ned = self._frame.rotation_to_ned(uas_att)
        self._show("r_uas_to_ned", uas_to_ned)
        ned_point = uas_to_ned @ uas_point + uas_to_ned_translation
        self._show("p_ned_prime", ned_point)
        return ned_point[:, 0]

    def calc_uv(
        self,
        p_ned: Sequence[float] | np.ndarray,
        k: np.ndarray,
        g_data: GimbalData,
        uas_att: Attitude,
    ) -> tuple[int | None, int | None]:
        ned_point = np.array(p_ned).reshape(3, 1)
        self._show("p_ned_prime", ned_point)

        ned_to_uas_translation = self._frame.translation_column()
        self._show("t_ned_to_uas", ned_to_uas_translation)
        uas_to_ned = self._frame.rotation_to_ned(uas_att)
        self._show("r_ned_to_uas", uas_to_ned)
        uas_point = np.transpose(uas_to_ned) @ (
            ned_point - ned_to_uas_translation
        )
        self._show("p_uas_prime", uas_point)

        gimbal_to_uas_translation = np.reshape(
            g_data.args.setup_dist,
            (3, 1),
        )
        self._show("t_g_to_uas", gimbal_to_uas_translation)
        gimbal_to_uas = Rotation.from_euler(
            g_data.args.g_seq,
            get_euler_by_sequence(g_data.att, g_data.args.g_seq),
            degrees=g_data.args.degrees,
        ).as_matrix()
        self._show("r_uas_to_g", gimbal_to_uas)
        gimbal_point = np.transpose(gimbal_to_uas) @ (
            uas_point - gimbal_to_uas_translation
        )
        self._show("p_g_prime", gimbal_point)

        gimbal_to_camera_translation = np.array([[0], [0], [0]])
        self._show("t_g_to_c", gimbal_to_camera_translation)
        camera_to_gimbal = Rotation.from_euler(
            g_data.args.setup_seq,
            get_euler_by_sequence(
                g_data.args.setup_att,
                g_data.args.setup_seq,
            ),
            degrees=g_data.args.setup_degrees,
        ).as_matrix()
        self._show("r_g_to_c", camera_to_gimbal)
        camera_point = np.transpose(camera_to_gimbal) @ (
            gimbal_point - gimbal_to_camera_translation
        )
        self._show("p_c_prime", camera_point)
        self._show("k", k)

        if round(camera_point[2][0], 8) <= 0:
            if self._enable_log:
                print(
                    "Out of detect. "
                    f"p_c: {np.array(camera_point).reshape(1, 3)}, "
                    f"g_att: {g_data.att}, uas_att: {uas_att}"
                )
            return None, None

        image = k @ (camera_point / camera_point[2][0])
        self._show("i", image)
        image = image[:, 0]
        return round(image[0]), round(image[1])

    def _show(self, name: str, value: object) -> None:
        if self._enable_log:
            print(f"{name}: {value}")
