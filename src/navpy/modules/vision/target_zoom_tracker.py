"""Compatibility facade for zoom tracking of a selected visual reference.

TargetZoomTracker can follow a delivery-dock observation; tracking alone does
not identify an authorized recipient. Public target-prefixed names stay stable.
"""

from navpy.modules.vision.target_zoom_orchestrator import TargetZoomTracker
from navpy.modules.vision.target_zoom_types import (
    TargetZoomTrackerConfig,
    ZoomTrackResult,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState

__all__ = [
    "TargetZoomTracker",
    "TargetZoomTrackerConfig",
    "ZoomTrackResult",
    "ZoomTrackingState",
]
