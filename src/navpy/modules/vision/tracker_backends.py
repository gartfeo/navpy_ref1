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

from navpy.core.ports.tracks import TrackedObject
from navpy.core.ports.detection import Detection

__all__ = [
    "TrackerBackend",
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
