"""Convert aircraft Euler attitude to a forward NED ray."""

from __future__ import annotations

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


class AttitudeRayConverter:
    def __init__(self, degrees: bool) -> None:
        self._degrees = degrees

    def to_ned(self, attitude: Attitude, sequence: str) -> np.ndarray:
        rotation = Rotation.from_euler(
            sequence,
            get_euler_by_sequence(attitude, sequence),
            degrees=self._degrees,
        )
        return np.round(rotation.apply(np.array([1.0, 0.0, 0.0])), 10)
