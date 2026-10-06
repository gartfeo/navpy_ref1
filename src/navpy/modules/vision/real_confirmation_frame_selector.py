"""Confirmation-frame ranking and retrieval for real detections."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from navpy.modules.vision.confirmation_frame import (
    should_replace_confirmation_frame,
)
from navpy.modules.vision.models.detection_components import BoundingBox
from navpy.modules.vision.real_detector_state import ConfirmationFrameStore


@dataclass(frozen=True)
class SelectedConfirmationFrame:
    """Stored evidence selected for one emitted POI."""

    frame: np.ndarray | None
    bbox_cxcywh: BoundingBox | None


class ConfirmationFrameSelector:
    """Own confirmation ranking while the store owns copying and eviction."""

    def __init__(self, store: ConfirmationFrameStore) -> None:
        self._store = store

    def select(
            self,
            obj_id: int,
            frame: np.ndarray | None,
            bbox: BoundingBox,
            center_score: float,
    ) -> SelectedConfirmationFrame:
        if frame is not None:
            stored = self._store.get(obj_id)
            if (
                    stored is None
                    or should_replace_confirmation_frame(
                        bbox,
                        center_score,
                        stored[1],
                        stored[2],
                    )
            ):
                self._store.put(obj_id, frame, bbox, center_score)

        stored = self._store.get(obj_id)
        if stored is None:
            return SelectedConfirmationFrame(None, None)
        return SelectedConfirmationFrame(stored[0], stored[1])

    def evict_except(self, live_ids: set[int]) -> None:
        self._store.evict_except(live_ids)


__all__ = ["ConfirmationFrameSelector", "SelectedConfirmationFrame"]
