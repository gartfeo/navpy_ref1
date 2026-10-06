"""Compatibility checks for the decomposed public camera-mount facade."""

from __future__ import annotations

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.fixed_gimbal import FixedGimbal
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable


def _camera(focal: float) -> CameraIntrinsics:
    return CameraIntrinsics(
        fx=focal,
        fy=focal,
        cx=50.0,
        cy=25.0,
        image_width=100,
        image_height=50,
    )


def test_public_camera_and_gimbal_replacements_remain_live() -> None:
    mount = CameraMount(
        "test",
        _camera(100.0),
        FixedGimbal(GimbalData(att=Attitude(-10.0, 0.0, 0.0))),
    )
    replacement_camera = _camera(200.0)
    replacement_gimbal = FixedGimbal(
        GimbalData(att=Attitude(-20.0, 0.0, 0.0))
    )

    mount.camera = replacement_camera
    mount.gimbal = replacement_gimbal

    assert mount.get_k()[0, 0] == 200.0
    assert mount.get_gimbal_data().att.pitch == -20.0


def test_public_calibration_replacement_changes_readback_mapping() -> None:
    class ReadbackGimbal(FixedGimbal):
        def get_zoom_level(self) -> float:
            return 1.5

    first = ZoomCalibrationTable.from_profile({"1": 1.0, "2": 2.0})
    second = ZoomCalibrationTable.from_profile({"10": 1.0, "20": 2.0})
    mount = CameraMount(
        "test",
        _camera(100.0),
        ReadbackGimbal(GimbalData(att=Attitude(0.0, 0.0, 0.0))),
        first,
    )

    assert mount.get_current_zoom_command() == "1.50"
    mount.zoom_calibration = second
    assert mount.get_current_zoom_command() == "15.00"
