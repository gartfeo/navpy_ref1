"""Narrow final-approach capabilities used outside the final-approach package."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity


@runtime_checkable
class VisionNavStatus(Protocol):
    def poi_passed_override(self) -> bool: ...

    def begin_dock_episode(self, identity: DockContactIdentity) -> None: ...

    def latch_docked(self, event: DockContactEvent) -> bool: ...

    def latch_failure(self) -> bool: ...

    def consume_missed_dock(self) -> bool: ...

    def last_measured_lateral_bearing_deg(self) -> float | None: ...


@runtime_checkable
class VisionNavConfirmation(Protocol):
    def can_confirm_detection(self, detect_data: DetectedObject) -> bool: ...

    def record_final_approach_confirmed_detection(
        self,
        detect_data: DetectedObject,
    ) -> bool: ...


__all__ = ["VisionNavConfirmation", "VisionNavStatus"]
