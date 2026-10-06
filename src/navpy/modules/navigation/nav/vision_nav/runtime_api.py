"""Narrow final-approach capabilities used outside the final-approach package."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from navpy.modules.vision.models.detect_data import DetectedObject


@runtime_checkable
class VisionNavStatus(Protocol):
    def poi_passed_override(self) -> bool: ...

    def last_measured_lateral_bearing_deg(self) -> float | None: ...


@runtime_checkable
class VisionNavConfirmation(Protocol):
    def can_confirm_detection(self, detect_data: DetectedObject) -> bool: ...

    def record_final_approach_confirmed_detection(
        self,
        detect_data: DetectedObject,
    ) -> bool: ...


__all__ = ["VisionNavConfirmation", "VisionNavStatus"]
