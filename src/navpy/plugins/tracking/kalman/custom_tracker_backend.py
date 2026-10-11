"""Tracker backend wrapping the built-in Kalman multi-object tracker."""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from navpy.modules.vision.multi_object_tracker import MultiObjectTracker
from navpy.core.ports.tracks import TrackedObject
from navpy.modules.vision.tracker_config import TrackerBackendConfig
from navpy.core.ports.detection import Detection


class CustomTrackerBackend:
    def __init__(self, config: TrackerBackendConfig):
        self._tracker = MultiObjectTracker(
            max_age=config.custom.max_age,
            min_hits=config.custom.min_hits,
            max_center_dist_px=config.custom.max_center_dist_px,
            revive_seconds=config.custom.revive_seconds,
        )

    def update(
            self,
            detections: List[Detection],
            frame_w: int,
            frame_h: int,
            frame: Optional[np.ndarray] = None,
    ) -> List[TrackedObject]:
        return self._tracker.update(detections, frame_w, frame_h)

    def reset(self) -> None:
        self._tracker.reset()

    def close(self) -> None:
        self._tracker.reset()
