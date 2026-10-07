"""Facade for zoom tracking of a selected POI.

PoiZoomTracker follows a dock observation; tracking preserves POI
association only.
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
