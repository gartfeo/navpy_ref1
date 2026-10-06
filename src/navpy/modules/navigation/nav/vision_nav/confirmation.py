"""Side-effect-free final-approach confirmation followed by explicit recording."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Protocol

from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
)
from navpy.modules.navigation.nav.vision_nav.law import FinalApproachLawPlan
from navpy.modules.navigation.nav.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)
from navpy.modules.vision.models.detect_data import DetectedObject


class FinalApproachConfirmationLaw(Protocol):
    def preview(self, frame: FinalApproachVisionFrame) -> FinalApproachLawPlan | None: ...

    def seed(self, frame: FinalApproachVisionFrame, roll_deg: float = 0.0) -> None: ...


@dataclass(frozen=True)
class FinalApproachConfirmationPorts:
    lock: threading.RLock
    projector: FinalApproachFrameProjector
    epochs: SourceEpochLedger
    law: FinalApproachConfirmationLaw
    roll_anchor_deg: Callable[[], float] = lambda: 0.0


class FinalApproachConfirmation:
    """Validate current measured authority without issuing a command."""

    def __init__(self, ports: FinalApproachConfirmationPorts) -> None:
        self._ports = ports

    def can_confirm_detection(self, poi: DetectedObject) -> bool:
        with self._ports.lock:
            frame = self._project(poi)
            return frame is not None and self._can_confirm(frame)

    def record_final_approach_confirmed_detection(self, poi: DetectedObject) -> bool:
        with self._ports.lock:
            frame = self._project(poi)
            if frame is None or not self._can_record(frame):
                return False
            admission = self._ports.epochs.admit(frame)
            if admission is FrameAdmission.REGRESSION:
                return False
            if admission is FrameAdmission.FRESH:
                self._ports.law.seed(frame, self._ports.roll_anchor_deg())
            return True

    def _project(self, poi: DetectedObject) -> FinalApproachVisionFrame | None:
        visual = poi.visual_detection()
        source_name = self._ports.projector.source_name(visual)
        if source_name is None:
            return None
        return self._ports.projector.project(
            visual,
            self._ports.epochs.generation(source_name),
            getattr(poi.timing, "source_air_speed_mps", None),
        )

    def _can_confirm(self, frame: FinalApproachVisionFrame) -> bool:
        plan = self._ports.law.preview(frame)
        return (
            self._ports.epochs.classify(frame) is FrameAdmission.FRESH
            and plan is not None
            and plan.within_limits
        )

    def _can_record(self, frame: FinalApproachVisionFrame) -> bool:
        """Accept the current valid frame after a flyable frame was reviewed."""
        return (
            self._ports.epochs.classify(frame) is FrameAdmission.FRESH
            and self._ports.law.preview(frame) is not None
        )


__all__ = [
    "FinalApproachConfirmation",
    "FinalApproachConfirmationLaw",
    "FinalApproachConfirmationPorts",
]
