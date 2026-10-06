"""Legacy wind feed-forward calculation for mission navigation."""

from __future__ import annotations

import numpy as np
from typing import Protocol

from navpy.modules.navigation.geo.rotation_utils import normalize
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.wind import Wind
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


class WindTelemetry(Protocol):
    @property
    def wind(self) -> Wind: ...

    @property
    def air_speed(self) -> float | None: ...

    @property
    def attitude(self) -> Attitude | None: ...


class WindFeedForwardCalculator:
    """Calculate compass-referenced wind feed-forward coefficients.

    This collaborator belongs to legacy mission navigation.  Its compass and
    world-frame inputs must not be passed into the pure-vision nav law.
    """

    def __init__(self, min_air_speed: float) -> None:
        self._min_air_speed = min_air_speed

    def calculate(self, vehicle: WindTelemetry) -> tuple[float, float]:
        wind = vehicle.wind
        if (
            wind is None
            or wind.speed is None
            or wind.direction is None
            or wind.speed < self._min_air_speed
        ):
            return 0, 0

        uas_speed_m_s = vehicle.air_speed
        uas_att = vehicle.attitude
        if uas_speed_m_s is None or uas_att is None:
            return 0, 0

        wind_ned = Rotation.from_euler(
            "Z", wind.direction, degrees=True
        ).apply([1, 0, 0])
        uas_to_ned = Rotation.from_euler(
            "ZYX",
            get_euler_by_sequence(uas_att, "ZYX"),
            degrees=True,
        ).as_matrix()
        wind_uas = np.transpose(uas_to_ned) @ wind_ned
        wind_unit = normalize(wind_uas)

        rate = wind.speed / uas_speed_m_s
        wind_yaw = rate * wind_unit[1]
        wind_pitch = rate * wind_unit[2]
        if abs(wind_yaw) < 1e-10:
            wind_yaw = 0.0
        if abs(wind_pitch) < 1e-10:
            wind_pitch = 0.0
        return wind_yaw, wind_pitch
