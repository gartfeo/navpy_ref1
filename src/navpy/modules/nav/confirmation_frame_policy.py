"""Confirmation-frame validation and source-size extraction."""

from __future__ import annotations

import math
from typing import Optional

from navpy.modules.nav.confirmation_reporting import ConfirmDebugReporter
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_size import characteristic_pixels


def extract_confirmation_size(target: DetectedObject) -> Optional[float]:
    bbox = target.confirmation.bbox_cxcywh
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

    def is_ready(self, target: DetectedObject) -> bool:
        if (
            target.confirmation.frame is not None
            or not target.confirmation.supports_frame
        ):
            return True
        self._debug.log(
            f"no_frame obj={target.identity.obj_id} "
            f"px=({target.pixel.u_px:.0f},{target.pixel.v_px:.0f})"
        )
        return False


__all__ = ["ConfirmationFramePolicy", "extract_confirmation_size"]
