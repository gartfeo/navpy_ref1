"""Simulator-only forced detection-gap policy."""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from typing import Optional

from navpy.modules.vision.sim.sim_detector_state import ForcedGapState
from navpy.modules.vision.sim.sim_runtime_ports import (
    GapSpecificationReader,
    TrackingStatusPort,
)


@dataclass(frozen=True)
class ForcedGapPlan:
    frame_timestamp_s: float


def read_forced_gap_specification() -> Optional[str]:
    return os.environ.get("NAVPY_SIM_FORCE_DETECTION_GAP")


class ForcedDetectionGapPolicy:
    """Commit a deterministic test gap inside the frame transaction fence."""

    def __init__(
        self,
        tracking: TrackingStatusPort,
        state: ForcedGapState,
        specification: GapSpecificationReader = read_forced_gap_specification,
    ) -> None:
        self._tracking = tracking
        self._state = state
        self._specification = specification
        self._lock = threading.Lock()

    def plan(self, frame_timestamp_s: float) -> ForcedGapPlan:
        """Describe the frame decision without changing shared state."""
        return ForcedGapPlan(float(frame_timestamp_s))

    def commit(self, plan: ForcedGapPlan) -> bool:
        """Apply anchor state only inside the caller's generation fence."""
        with self._lock:
            return self._commit_timestamp(plan.frame_timestamp_s)

    def _commit_timestamp(self, frame_timestamp_s: float) -> bool:
        window = self._parse_window(self._specification())
        if window is None:
            return False
        start_s, duration_s = window

        anchor_s = self._state.anchor_timestamp_s
        if anchor_s is None:
            if self._tracking.tracking_obj_id is None:
                return False
            anchor_s = frame_timestamp_s
            self._state.set_anchor_timestamp_s(anchor_s)

        elapsed_s = frame_timestamp_s - anchor_s
        return start_s <= elapsed_s < (start_s + duration_s)

    @staticmethod
    def _parse_window(specification: Optional[str]) -> Optional[tuple[float, float]]:
        if not specification:
            return None
        try:
            start_s_text, duration_s_text = specification.split(":", 1)
            start_s = float(start_s_text)
            duration_s = float(duration_s_text)
        except (ValueError, AttributeError):
            return None
        if start_s < 0.0 or duration_s <= 0.0:
            return None
        return start_s, duration_s


__all__ = [
    "ForcedDetectionGapPolicy",
    "ForcedGapPlan",
    "read_forced_gap_specification",
]
