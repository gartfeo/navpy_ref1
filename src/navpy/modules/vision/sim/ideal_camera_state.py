"""Static aircraft-relative camera state for ideal simulator rendering."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import (
    GimbalData,
    GimbalMountSetup,
)
from navpy.modules.vision.sim.ideal_pose_source import copy_attitude
from navpy.modules.vision.sim.sim_camera_ports import GimbalDataReader


class IdealCameraReader(Protocol):
    def read(self) -> GimbalData: ...


class IdealCameraState:
    """Capture one neutral, static camera transform for an ideal run."""

    def __init__(self, read_gimbal: GimbalDataReader) -> None:
        self._read_gimbal = read_gimbal
        self._data: GimbalData | None = None

    def prepare(self) -> None:
        data = copy_gimbal_data(self._read_gimbal())
        data.att = Attitude(0.0, 0.0, 0.0)
        self._data = data

    def read(self) -> GimbalData:
        if self._data is None:
            raise RuntimeError("ideal camera must be prepared before rendering")
        return copy_gimbal_data(self._data)

    def reset(self) -> None:
        self._data = None


def copy_gimbal_data(gimbal_data: GimbalData) -> GimbalData:
    return GimbalData(
        att=copy_attitude(gimbal_data.att),
        setup=GimbalMountSetup(
            att=copy_attitude(gimbal_data.setup_att),
            seq=str(gimbal_data.setup_seq),
            degrees=bool(gimbal_data.setup_degrees),
            dist=list(gimbal_data.setup_dist),
        ),
        g_seq=str(gimbal_data.g_seq),
        degrees=bool(gimbal_data.degrees),
        roll_stabilize=bool(gimbal_data.roll_stabilize),
        pitch_stabilize=bool(gimbal_data.pitch_stabilize),
        max_detect_distance=float(gimbal_data.max_detect_distance),
        name=str(gimbal_data.name),
        timestamp_s=gimbal_data.timestamp_s,
    )


__all__ = ["IdealCameraReader", "IdealCameraState", "copy_gimbal_data"]
