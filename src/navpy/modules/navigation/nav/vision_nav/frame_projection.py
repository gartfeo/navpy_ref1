"""Convert a narrow pixel observation into the primitive final-approach DTO."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.vision.models.pixel_observation import VisualDetection
from navpy.modules.vision.visual_ray_projection import observation_body_ray, unit_vector
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


@dataclass(frozen=True)
class FinalApproachProjectionConfig:
    aircraft_sequence: str
    aircraft_degrees: bool


class FinalApproachFrameProjector:
    """Accept only frame-local pixels and immutable camera state."""

    def __init__(self, config: FinalApproachProjectionConfig) -> None:
        self._config = config

    @staticmethod
    def source_name(detection: VisualDetection) -> str | None:
        name = detection.observation.source_name
        return name if type(name) is str and bool(name) else None

    def project(
        self,
        detection: VisualDetection,
        source_generation: int,
        air_speed_mps: float | None = 25.0,
    ) -> FinalApproachVisionFrame | None:
        observation = detection.observation
        if observation.pose_is_frame_atomic is not True:
            return None
        source_name = self.source_name(detection)
        identity = _identity(detection)
        if source_name is None or identity is None:
            return None
        try:
            body_ray = observation_body_ray(observation)
            attitude = Attitude(
                pitch=_finite(observation.aircraft_pitch_deg),
                yaw=0.0,
                roll=_finite(observation.aircraft_roll_deg),
            )
            source_timestamp_s = _finite(observation.source_timestamp_s)
            air_speed = _finite(air_speed_mps)
            if air_speed <= 0.0:
                return None
            # DELIBERATE and separate from the try/except below.
            #
            # `aircraft_yaw_rate_rad_s` is declared OPTIONAL on PixelObservation
            # (`float | None = None`), and `aircraft_yaw_rate_rad_s()` returns
            # None whenever the conversion has no answer -- missing body rates,
            # or cos(pitch) singular near vertical. Calling `_finite` on it
            # raised TypeError, the broad handler swallowed that, and the WHOLE
            # frame vanished with no diagnostic: identical, from the outside, to
            # a malformed observation.
            #
            # Keeping the refusal (the lateral channel needs this term to remove
            # the aircraft's own turn) but making it explicit, so it is
            # attributable rather than accidental. That the aircraft goes blind
            # near vertical pitch -- exactly where it is diving -- is a real
            # hazard, and it is now visible instead of hidden.
            if observation.aircraft_yaw_rate_rad_s is None:
                return None
            yaw_rate = _finite(observation.aircraft_yaw_rate_rad_s)
            control_ray = _control_ray(body_ray, attitude, self._config)
        except (TypeError, ValueError):
            return None
        task_id, obj_id = identity
        return FinalApproachVisionFrame(
            source_name=source_name,
            source_generation=source_generation,
            task_id=task_id,
            obj_id=obj_id,
            source_timestamp_s=source_timestamp_s,
            body_x=float(body_ray[0]),
            body_y=float(body_ray[1]),
            body_z=float(body_ray[2]),
            control_x=float(control_ray[0]),
            control_y=float(control_ray[1]),
            control_z=float(control_ray[2]),
            aircraft_roll_deg=float(attitude.roll),
            air_speed_mps=air_speed,
            aircraft_yaw_rate_rad_s=yaw_rate,
            aircraft_pitch_deg=float(attitude.pitch),
        )


def _identity(detection: VisualDetection) -> tuple[int, int] | None:
    values = (detection.task_id, detection.obj_id)
    if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
        return None
    return values


def _control_ray(
    body_ray: np.ndarray,
    attitude: Attitude,
    config: FinalApproachProjectionConfig,
) -> np.ndarray:
    rotation = Rotation.from_euler(
        config.aircraft_sequence,
        get_euler_by_sequence(attitude, config.aircraft_sequence),
        degrees=config.aircraft_degrees,
    ).as_matrix()
    return unit_vector(rotation @ body_ray)


def _finite(value: object) -> float:
    if isinstance(value, bool):
        raise ValueError("boolean is not a measurement")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("measurement must be finite")
    return number


__all__ = ["FinalApproachFrameProjector", "FinalApproachProjectionConfig"]
