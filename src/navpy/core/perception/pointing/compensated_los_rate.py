"""Camera-motion-compensated visual LOS rate (HITL step 24, design N7).

The measured pixel ray goes through the frame's camera-to-body transform and
the aircraft's pitch/roll into a heading-free level frame. Its elevation there
is world elevation. Its azimuth there turns with the aircraft, so each step
adds the gyro turn rate (trapezoid) to the wrapped azimuth change: the sum is
the azimuth relative to the first sample, never an absolute heading. A
constant-velocity estimator over the two angles gives the LOS rate that the
gimbal, rate-commanded in LOCK about the world vertical, must turn at; that
rate is the feedforward.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.euler_utils import get_euler_by_sequence
from navpy.core.kernel.simple_rotation import Rotation
from navpy.core.perception.pointing.camera_motion_state import CameraMotionState
from navpy.core.perception.pointing.poi_angle_estimator import (
    PoiAngleEstimator,
    PoiAngleEstimatorConfig,
    PoiAnglePlan,
)


@dataclass(frozen=True)
class CameraFeedforwardConfig:
    # geo_ref.uas_seq; the frame projector uses the same convention.
    aircraft_sequence: str = "ZYX"
    aircraft_degrees: bool = True
    # Longer gaps between camera states restart the estimate: the gyro
    # trapezoid and the estimator's constant-velocity step both assume short steps.
    max_gap_s: float = 0.25
    # Above this |elevation| azimuth is ill-conditioned (it is undefined at
    # the nadir): the feedforward is withheld and the estimate restarts.
    max_abs_elevation_deg: float = 80.0
    # None reuses the tracker's own estimator configuration.
    estimator: PoiAngleEstimatorConfig | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_gap_s) or self.max_gap_s <= 0.0:
            raise ValueError("max_gap_s must be finite and > 0")
        if not 0.0 < self.max_abs_elevation_deg < 90.0:
            raise ValueError("max_abs_elevation_deg must be in (0, 90)")


@dataclass(frozen=True)
class _LosMemory:
    timestamp_s: float
    level_azimuth_rad: float
    relative_azimuth_rad: float
    turn_rate_rad_s: float


@dataclass(frozen=True)
class LosRatePlan:
    """A previewed step; commit it only after the command was sent."""

    yaw_rate_rad_s: float
    pitch_rate_rad_s: float
    _memory: _LosMemory | None
    _estimate: PoiAnglePlan | None


class CompensatedLosRate:
    """Preview/commit estimator of the LOS rate in a stabilised frame."""

    def __init__(
        self,
        config: CameraFeedforwardConfig,
        estimator: PoiAngleEstimatorConfig,
    ) -> None:
        self._config = config
        self._estimator = PoiAngleEstimator(config.estimator or estimator)
        self._memory: _LosMemory | None = None

    def reset(self) -> None:
        self._estimator.reset()
        self._memory = None

    def preview(
        self,
        camera_ray: np.ndarray,
        camera: CameraMotionState | None,
        timestamp_s: float,
    ) -> LosRatePlan:
        if camera is None:
            return _NO_FEEDFORWARD
        azimuth, elevation = level_angles(camera_ray, camera, self._config)
        if abs(elevation) > math.radians(self._config.max_abs_elevation_deg):
            return _NO_FEEDFORWARD
        previous = self._memory
        dt_s = None if previous is None else timestamp_s - previous.timestamp_s
        restart = dt_s is None or not 0.0 < dt_s <= self._config.max_gap_s
        if restart:
            relative = azimuth
        else:
            turn = 0.5 * (previous.turn_rate_rad_s + camera.aircraft_turn_rate_rad_s)
            relative = (
                previous.relative_azimuth_rad
                + _wrap(azimuth - previous.level_azimuth_rad)
                + turn * dt_s
            )
        plan = self._estimator.preview_update(
            relative, elevation, timestamp_s, reset=restart
        )
        if plan is None:
            return _NO_FEEDFORWARD
        memory = _LosMemory(
            timestamp_s, azimuth, relative, camera.aircraft_turn_rate_rad_s
        )
        estimate = plan.estimate
        return LosRatePlan(
            estimate.yaw_rate_rad_s, estimate.pitch_rate_rad_s, memory, plan
        )

    def commit(self, plan: LosRatePlan) -> None:
        if plan._memory is None or plan._estimate is None:
            self.reset()
            return
        self._estimator.commit(plan._estimate)
        self._memory = plan._memory


_NO_FEEDFORWARD = LosRatePlan(0.0, 0.0, None, None)


def level_angles(
    camera_ray: np.ndarray,
    camera: CameraMotionState,
    config: CameraFeedforwardConfig,
) -> tuple[float, float]:
    """Azimuth (right +) and elevation (up +) of a camera ray, heading-free."""
    body = camera.camera_to_body_matrix() @ np.asarray(camera_ray, dtype=float)
    attitude = Attitude(
        pitch=camera.aircraft_pitch_deg,
        yaw=0.0,
        roll=camera.aircraft_roll_deg,
    )
    level = Rotation.from_euler(
        config.aircraft_sequence,
        get_euler_by_sequence(attitude, config.aircraft_sequence),
        degrees=config.aircraft_degrees,
    ).as_matrix() @ body
    # Level frame: x along the aircraft's horizontal forward, z down.
    forward, right, down = (float(value) for value in level)
    return math.atan2(right, forward), math.atan2(-down, math.hypot(forward, right))


def _wrap(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


__all__ = [
    "CameraFeedforwardConfig",
    "CompensatedLosRate",
    "LosRatePlan",
    "level_angles",
]
