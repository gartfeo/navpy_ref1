"""Aircraft-frame transform shared by geo-reference calculators."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


@dataclass(frozen=True)
class UasFrameTransform:
    sequence: str
    degrees: bool
    translation_ned: tuple[float, float, float]

    def rotation_to_ned(self, attitude: Attitude) -> np.ndarray:
        return Rotation.from_euler(
            self.sequence,
            get_euler_by_sequence(attitude, self.sequence),
            degrees=self.degrees,
        ).as_matrix()

    def translation_column(self) -> np.ndarray:
        return np.asarray(self.translation_ned).reshape(3, 1)
