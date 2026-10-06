"""Selectable multi-object tracker backends (public surface).

All backends normalize their output to ``TrackedObject`` so the detector,
POI lock, navigation, and UI paths stay independent from a specific tracker
library.

Backends:
  - ``custom``   : the built-in Kalman ``MultiObjectTracker`` (no GPU, no ReID).
  - ``botsort``  : BoxMOT BoT-SORT (appearance ReID + camera-motion compensation).
  - ``strongsort`` : BoxMOT StrongSORT (appearance ReID memory).

The library adapters live in ``tracker_backends_boxmot`` and the coasting
memory in ``track_export_memory``.
"""
from __future__ import annotations

from typing import List, Optional, Protocol

import numpy as np

from navpy.modules.vision.device import DeviceT
from navpy.modules.vision.multi_object_tracker import MultiObjectTracker, TrackedObject
from navpy.modules.vision.tracker_backends_boxmot import (
    BotSortTrackerBackend,
    StrongSortTrackerBackend,
)
from navpy.modules.vision.tracker_config import (
    TrackerBackendConfig,
    tracker_config_from_settings,
)
from navpy.modules.vision.yolo_detector import Detection

__all__ = [
    "TrackerBackend",
    "TrackerBackendConfig",
    "CustomTrackerBackend",
    "BotSortTrackerBackend",
    "StrongSortTrackerBackend",
    "tracker_config_from_settings",
    "create_tracker_backend",
]


class TrackerBackend(Protocol):
    def update(
            self,
            detections: List[Detection],
            frame_w: int,
            frame_h: int,
            frame: Optional[np.ndarray] = None,
    ) -> List[TrackedObject]:
        ...

    def reset(self) -> None:
        ...

    def close(self) -> None:
        """Release any held GPU/model resources. Safe to call more than once."""
        ...


def create_tracker_backend(
        settings: Optional[dict],
        *,
        detector_device: DeviceT = "auto",
        detector_conf: float = 0.25,
        logger=None,
) -> TrackerBackend:
    config = tracker_config_from_settings(settings, detector_conf=detector_conf)
    backend = config.runtime.backend
    if backend in ("custom", "kalman", "mot"):
        return CustomTrackerBackend(config)
    if backend in ("botsort", "bot-sort", "bot_sort"):
        return BotSortTrackerBackend(config, detector_device=detector_device, logger=logger)
    if backend in ("strongsort", "strong-sort", "strong_sort"):
        return StrongSortTrackerBackend(config, detector_device=detector_device, logger=logger)
    raise ValueError(
        "tracker backend must be one of: custom, botsort, strongsort "
        f"(got {config.runtime.backend!r})"
    )


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
