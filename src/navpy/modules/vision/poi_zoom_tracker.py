"""Compatibility facade for zoom tracking of a selected visual reference.

PoiZoomTracker can follow a delivery-dock observation; tracking alone does
not identify an authorized recipient. Public POI-prefixed names stay stable.
"""

from navpy.modules.vision.poi_zoom_orchestrator import PoiZoomTracker
from navpy.modules.vision.poi_zoom_types import (
    PoiZoomTrackerConfig,
    ZoomTrackResult,
)
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState

__all__ = [
    "PoiZoomTracker",
    "PoiZoomTrackerConfig",
    "ZoomTrackResult",
    "ZoomTrackingState",
]
