"""Neutral-pose return owned outside the angular rate estimator."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTracker


MODE_FOLLOW = 1


class NeutralGimbalActuator(Protocol):
    def set_motion_mode(self, mode: int) -> None: ...

    def set_att(self, attitude: Attitude) -> None: ...

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None: ...


class GimbalNeutralReturn:
    def __init__(
        self,
        actuator: NeutralGimbalActuator,
        rate_tracker: GimbalRateTracker,
        neutral_pitch_deg: float,
    ) -> None:
        self._actuator = actuator
        self._rate_tracker = rate_tracker
        self._neutral_attitude = Attitude(neutral_pitch_deg, 0.0, 0.0)

    @property
    def pitch_deg(self) -> float:
        return self._neutral_attitude.pitch

    def execute(self) -> None:
        try:
            self._actuator.set_motion_mode(MODE_FOLLOW)
            self._actuator.set_att(self._neutral_attitude)
        finally:
            self._rate_tracker.reset()

    def hold(self) -> None:
        """Stop angular motion without changing mode or pointing attitude."""
        try:
            self._actuator.set_rate(0.0, 0.0)
        finally:
            self._rate_tracker.reset()


__all__ = ["GimbalNeutralReturn", "NeutralGimbalActuator"]
