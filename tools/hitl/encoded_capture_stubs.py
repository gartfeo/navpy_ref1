"""Stand-ins of the encoded capture's live check: a level aircraft and a static camera with the ZR10's size."""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import Mock


def stub_vehicle() -> Any:
    from navpy.core.kernel.attitude import Attitude

    attitude = Attitude(pitch=0.0, yaw=0.0, roll=0.0)
    vehicle = Mock()
    vehicle.attitude = attitude
    vehicle.attitude_history.return_value = ()
    vehicle.air_speed = 20.0
    vehicle.link_identity = "encoded-capture-check"
    vehicle.location.return_value = None
    type(vehicle).attitude_sample = property(lambda _self: SimpleNamespace(
        attitude=attitude, body_rates_rad_s=(0.0, 0.0, 0.0), time_boot_s=time.monotonic(), receipt_time_s=time.time(),
    ))
    return vehicle


def stub_mount() -> Any:
    import numpy as np

    from navpy.core.kernel.attitude import Attitude
    from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
    from navpy.core.ports.gimbal_abc import GimbalData

    def state(width: int, height: int) -> CameraMountFrameState:
        return CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
            k=np.array([[1400.0, 0.0, width / 2], [0.0, 1400.0, height / 2], [0.0, 0.0, 1.0]]),
            dist=np.zeros(5),
            gimbal_timestamp_s=None,
            gimbal_is_static=True,
            zoom_command=None,
            zoom_sample_id="static:encoded-capture-check",
            zoom_sample_age_s=0.0,
        )

    mount = Mock()
    mount.capture_frame_state.side_effect = state
    return mount
