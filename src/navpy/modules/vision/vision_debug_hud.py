"""Rate-tracker and zoom HUD for one simulator debug panel."""

from __future__ import annotations

import math

import cv2
import numpy as np

from navpy.modules.vision.gimbal_rate_tracker import GimbalTrackResult
from navpy.modules.vision.target_zoom_types import ZoomTrackResult
from navpy.modules.vision.vision_ui_ports import SimDebugSnapshot


class SimTrackerHudRenderer:
    def draw(
        self,
        image: np.ndarray,
        y_offset: int,
        height: int,
        snapshot: SimDebugSnapshot,
    ) -> None:
        if snapshot.rate_result is None:
            cv2.putText(
                image,
                f"det: {len(snapshot.detections)}",
                (5, y_offset + height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (150, 150, 150),
                1,
            )
        else:
            self._draw_rate_tracker(
                image,
                y_offset,
                height,
                snapshot.rate_result,
            )
        if snapshot.zoom_result is not None:
            self._draw_zoom_tracker(
                image,
                y_offset,
                height,
                snapshot.zoom_result,
                snapshot.zoom_target_pixels,
            )

    def _draw_rate_tracker(
        self,
        image: np.ndarray,
        y_offset: int,
        height: int,
        result: GimbalTrackResult,
    ) -> None:
        state_name = result.state.value.upper()
        color = {
            "IDLE": (120, 120, 120),
            "TRACKING": (0, 255, 0),
            "HOLDING": (0, 140, 255),
        }.get(state_name, (150, 150, 150))
        if result.yaw_error is not None and result.pitch_error is not None:
            text = (
                f"{state_name}  err:("
                f"{math.degrees(result.yaw_error):+.1f},"
                f"{math.degrees(result.pitch_error):+.1f})"
            )
        else:
            text = state_name
        cv2.putText(
            image,
            text,
            (5, y_offset + height - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
        )
        if result.yaw_rate is None or result.pitch_rate is None:
            return
        cv2.putText(
            image,
            f"rate:({result.yaw_rate:+.1f},{result.pitch_rate:+.1f})",
            (5, y_offset + height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (150, 150, 150),
            1,
        )

    def _draw_zoom_tracker(
        self,
        image: np.ndarray,
        y_offset: int,
        height: int,
        result: ZoomTrackResult,
        target_pixels: float | None,
    ) -> None:
        state_name = result.state.value.upper()
        color = {
            "IDLE": (120, 120, 120),
            "HOLDING": (0, 255, 0),
            "ZOOMING_IN": (0, 200, 255),
            "ZOOMING_OUT": (200, 200, 0),
            "UNSUPPORTED": (80, 80, 80),
        }.get(state_name, (150, 150, 150))
        text = f"zoom:{state_name}"
        if result.size_px is not None:
            target_px = target_pixels or 0.0
            ratio = result.size_px / target_px if target_px > 0 else 0.0
            text = (
                f"zoom:{state_name}  size:{result.size_px:.0f}/"
                f"{target_px:.0f}px ({ratio:.2f}x)"
            )
        cv2.putText(
            image,
            text,
            (5, y_offset + height - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            color,
            1,
        )


__all__ = ["SimTrackerHudRenderer"]
