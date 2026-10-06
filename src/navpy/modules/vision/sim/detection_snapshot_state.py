"""Thread-confined latest simulator detection snapshot state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class DetectionSnapshot:
    detected_pois: tuple[DetectedObject, ...]
    debug_pois: tuple[DetectedObject, ...]
    primary_poi: Optional[DetectedObject]


__all__ = ["DetectionSnapshot"]
