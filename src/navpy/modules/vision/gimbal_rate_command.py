"""Pure rate-command law for one angular estimate."""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.modules.vision.gimbal_rate_types import GimbalRateTrackerConfig
from navpy.modules.vision.target_angle_estimator import (
    TargetAngleEstimate,
    project_estimate,
)


@dataclass(frozen=True)
class GimbalRateCommand:
    yaw_units: float
    pitch_units: float
    projected_yaw_rad: float
    projected_pitch_rad: float


class GimbalRateCommandLaw:
    def __init__(self, config: GimbalRateTrackerConfig) -> None:
        self._config = config

    def command(self, estimate: TargetAngleEstimate) -> GimbalRateCommand:
        yaw_rad, pitch_rad = project_estimate(
            estimate,
            self._config.command_lead_time,
        )
        yaw_error_deg = math.degrees(yaw_rad)
        pitch_error_deg = -math.degrees(pitch_rad)
        bandwidth = self._config.correction_bw
        return GimbalRateCommand(
            self._to_units(bandwidth * yaw_error_deg, yaw_error_deg),
            self._to_units(bandwidth * pitch_error_deg, pitch_error_deg),
            yaw_rad,
            pitch_rad,
        )

    def _to_units(self, desired_dps: float, error_deg: float) -> float:
        config = self._config
        units = desired_dps / config.max_slew_dps * 100.0
        units = max(-config.max_rate, min(config.max_rate, units))
        if (
            abs(error_deg) > config.settle_tolerance_deg
            and 0.0 < abs(units) < 1.0
        ):
            return math.copysign(1.0, units)
        return units


__all__ = ["GimbalRateCommand", "GimbalRateCommandLaw"]
