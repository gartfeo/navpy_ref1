from __future__ import annotations

from dataclasses import replace

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalAbc, GimbalData


class FixedGimbal(GimbalAbc):
    """No-op gimbal for body-fixed cameras.

    Used when a camera is rigidly mounted to the vehicle body with no
    gimbal stabilization. The gimbal attitude is fixed based on the
    profile configuration.
    """

    def __init__(self, data: GimbalData) -> None:
        self._data = data

    def get_data(self) -> GimbalData:
        return self._data

    def set_att(self, att: Attitude) -> None:
        """Set gimbal attitude (updates the fixed attitude)."""
        self._data = replace(self._data, att=att)

    def state_is_static(self) -> bool:
        return True

    def raise_if_failed(self) -> None:
        """A body-fixed mount has no background worker to report."""
