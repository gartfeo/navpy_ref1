"""Confirmation-frame validation and source-size extraction."""

from __future__ import annotations

import math
from typing import Optional

from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_size import characteristic_pixels


def extract_confirmation_size(poi: DetectedObject) -> Optional[float]:
    bbox = poi.confirmation.bbox_cxcywh
    if bbox is None or len(bbox) < 4:
        return None
    try:
        width = float(bbox[2])
        height = float(bbox[3])
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(width) and math.isfinite(height)):
        return None
    if width <= 0.0 or height <= 0.0:
        return None
    return characteristic_pixels(width, height)


class ConfirmationFramePolicy:
    def __init__(self, debug: ConfirmDebugReporter) -> None:
        self._debug = debug

    def is_ready(self, poi: DetectedObject) -> bool:
        if (
            poi.confirmation.frame is not None
            or not poi.confirmation.supports_frame
        ):
            return True
        self._debug.log(
            f"no_frame obj={poi.identity.obj_id} "
            f"px=({poi.pixel.u_px:.0f},{poi.pixel.v_px:.0f})"
        )
        return False


__all__ = ["ConfirmationFramePolicy", "extract_confirmation_size"]
