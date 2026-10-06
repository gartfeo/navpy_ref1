"""Thread-confined latest simulator detection snapshot state."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class DetectionSnapshot:
    detected_targets: tuple[DetectedObject, ...]
    debug_targets: tuple[DetectedObject, ...]
    primary_target: Optional[DetectedObject]


__all__ = ["DetectionSnapshot"]
