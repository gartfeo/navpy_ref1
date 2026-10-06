"""Raw source-clock admission and explicit discontinuity epochs."""

from __future__ import annotations

from enum import Enum
from typing import Iterable

from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame


class FrameAdmission(Enum):
    FRESH = "fresh"
    DUPLICATE = "duplicate"
    REGRESSION = "regression"


class SourceEpochLedger:
    """Preserve raw producer time across phase resets."""

    def __init__(self) -> None:
        self._generations: dict[str, int] = {}
        self._timestamps: dict[tuple[str, int, int, int], float] = {}

    def generation(self, source_name: str) -> int:
        return self._generations.setdefault(source_name, 0)

    def note_discontinuity(self, source_name: str) -> None:
        self.note_discontinuities((source_name,))

    def note_discontinuities(
        self,
        source_names: Iterable[str],
    ) -> tuple[str, ...]:
        ordered_names = tuple(dict.fromkeys(source_names))
        if any(not isinstance(name, str) or not name for name in ordered_names):
            raise ValueError("discontinuity source names must be non-empty strings")
        self._generations.update({
            name: self._generations.get(name, 0) + 1
            for name in ordered_names
        })
        return ordered_names

    def classify(self, frame: FinalApproachVisionFrame) -> FrameAdmission:
        previous = self._timestamps.get(frame.continuity_key)
        if previous is None or frame.source_timestamp_s > previous:
            return FrameAdmission.FRESH
        if frame.source_timestamp_s == previous:
            return FrameAdmission.DUPLICATE
        return FrameAdmission.REGRESSION

    def admit(self, frame: FinalApproachVisionFrame) -> FrameAdmission:
        result = self.classify(frame)
        if result is FrameAdmission.FRESH:
            self._timestamps[frame.continuity_key] = frame.source_timestamp_s
        return result


__all__ = ["FrameAdmission", "SourceEpochLedger"]
