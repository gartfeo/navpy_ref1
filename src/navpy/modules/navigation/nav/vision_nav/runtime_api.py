"""Narrow terminal capabilities used outside the terminal package."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from navpy.modules.vision.models.detect_data import DetectedObject


@runtime_checkable
class VisionNavStatus(Protocol):
    def target_passed_override(self) -> bool: ...

    def last_measured_lateral_bearing_deg(self) -> float | None: ...


@runtime_checkable
class VisionNavConfirmation(Protocol):
    def can_confirm_detection(self, detect_data: DetectedObject) -> bool: ...

    def record_terminal_confirmed_detection(
        self,
        detect_data: DetectedObject,
    ) -> bool: ...


__all__ = ["VisionNavConfirmation", "VisionNavStatus"]
