"""Side-effect-free guidance confirmation followed by explicit recording."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.frame import GuidanceVisionFrame
from navpy.core.approach.vision_nav.frame_projection import (
    GuidanceFrameProjector,
)
from navpy.plugins.laws.vision_pn.law_plan import GuidanceLawPlan
from navpy.core.approach.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)
from navpy.core.ports.detect_data import DetectedObject


class GuidanceConfirmationLaw(Protocol):
    def preview(self, frame: GuidanceVisionFrame) -> GuidanceLawPlan | None: ...

    def seed(self, frame: GuidanceVisionFrame, roll_deg: float = 0.0) -> None: ...


@dataclass(frozen=True)
class GuidanceConfirmationPorts:
    lock: rt.RLock
    projector: GuidanceFrameProjector
    epochs: SourceEpochLedger
    law: GuidanceConfirmationLaw
    roll_anchor_deg: Callable[[], float] = lambda: 0.0


class GuidanceConfirmation:
    """Validate current measured authority without issuing a command."""

    def __init__(self, ports: GuidanceConfirmationPorts) -> None:
        self._ports = ports

    def can_confirm_detection(self, poi: DetectedObject) -> bool:
        with self._ports.lock:
            frame = self._project(poi)
            return frame is not None and self._can_confirm(frame)

    def record_guidance_confirmed_detection(self, poi: DetectedObject) -> bool:
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

    def _project(self, poi: DetectedObject) -> GuidanceVisionFrame | None:
        visual = poi.visual_detection()
        source_name = self._ports.projector.source_name(visual)
        if source_name is None:
            return None
        return self._ports.projector.project(
            visual,
            self._ports.epochs.generation(source_name),
            getattr(poi.timing, "source_air_speed_mps", None),
        )

    def _can_confirm(self, frame: GuidanceVisionFrame) -> bool:
        plan = self._ports.law.preview(frame)
        return (
            self._ports.epochs.classify(frame) is FrameAdmission.FRESH
            and plan is not None
            and plan.within_limits
        )

    def _can_record(self, frame: GuidanceVisionFrame) -> bool:
        """Accept the current valid frame after a flyable frame was reviewed."""
        return (
            self._ports.epochs.classify(frame) is FrameAdmission.FRESH
            and self._ports.law.preview(frame) is not None
        )


__all__ = [
    "GuidanceConfirmation",
    "GuidanceConfirmationLaw",
    "GuidanceConfirmationPorts",
]
