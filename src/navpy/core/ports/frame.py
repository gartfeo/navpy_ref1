"""Primitive, immutable input accepted by final approach."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FinalApproachVisionFrame:
    """One measured frame-local line of sight, with no truth dependencies."""

    source_name: str
    source_generation: int
    task_id: int
    obj_id: int
    source_timestamp_s: float
    body_x: float
    body_y: float
    body_z: float
    control_x: float
    control_y: float
    control_z: float
    aircraft_roll_deg: float = 0.0
    air_speed_mps: float = 25.0
    aircraft_yaw_rate_rad_s: float = 0.0
    aircraft_pitch_deg: float = 0.0

    def __post_init__(self) -> None:
        if type(self.source_name) is not str or not self.source_name:
            raise ValueError("source_name must not be empty")
        identities = (self.source_generation, self.task_id, self.obj_id)
        if any(type(value) is not int for value in identities):
            raise ValueError("final-approach frame identities must be exact integers")
        if self.source_generation < 0:
            raise ValueError("source_generation must be non-negative")
        values = self.body_ray + self.control_ray + (
            self.source_timestamp_s,
            self.aircraft_roll_deg,
            self.air_speed_mps,
            self.aircraft_yaw_rate_rad_s,
            self.aircraft_pitch_deg,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("final-approach frame values must be finite")
        if self.air_speed_mps <= 0.0:
            raise ValueError("final-approach frame airspeed must be positive")
        self._validate_unit(self.body_ray, "body")
        self._validate_unit(self.control_ray, "control")

    @property
    def body_ray(self) -> tuple[float, float, float]:
        return self.body_x, self.body_y, self.body_z

    @property
    def control_ray(self) -> tuple[float, float, float]:
        return self.control_x, self.control_y, self.control_z

    @property
    def continuity_key(self) -> tuple[str, int, int, int]:
        return self.source_name, self.source_generation, self.task_id, self.obj_id

    @staticmethod
    def _validate_unit(ray: tuple[float, float, float], name: str) -> None:
        norm = math.sqrt(sum(component * component for component in ray))
        if not math.isclose(norm, 1.0, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError(f"{name} ray must be a unit vector")


__all__ = ["FinalApproachVisionFrame"]
