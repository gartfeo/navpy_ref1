"""Render simulator detections and optional NED diagnostics."""

from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.vision_ui_ports import NedVectorCalculator


class SimTargetOverlayRenderer:
    def __init__(self, calc_ned: NedVectorCalculator) -> None:
        self._calc_ned = calc_ned

    def draw(
        self,
        image: np.ndarray,
        y_offset: int,
        scale_x: float,
        scale_y: float,
        image_center_x: float,
        image_center_y: float,
        detections: Sequence[DetectedObject],
    ) -> None:
        for target in detections:
            x = int(target.pixel.u_px * scale_x)
            y = y_offset + int(target.pixel.v_px * scale_y)
            dx = target.pixel.u_px - image_center_x
            dy = target.pixel.v_px - image_center_y
            ned_text = self._target_ned_text(target)
            color = (0, 255, 0)
            cv2.circle(image, (x, y), 8, color, 2)
            cv2.line(image, (x - 12, y), (x + 12, y), color, 1)
            cv2.line(image, (x, y - 12), (x, y + 12), color, 1)
            cv2.putText(
                image,
                f"{target.identity.obj_id}:({int(dx)},{int(dy)})",
                (x + 10, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
            )
            if ned_text:
                cv2.putText(
                    image,
                    ned_text,
                    (x + 10, y + 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (150, 200, 150),
                    1,
                )

    def _target_ned_text(self, target: DetectedObject) -> str:
        if (
            target.optics.calibration is None
            or target.pose.gimbal_data is None
            or target.pose.aircraft_attitude is None
        ):
            return ""
        try:
            target_ned = self._calc_ned(
                u=target.pixel.u_px,
                v=target.pixel.v_px,
                k=target.optics.camera_matrix(),
                g_data=target.pose.gimbal_data,
                uas_att=target.pose.aircraft_attitude,
            )
            if target_ned is None:
                return ""
            norm = float(np.linalg.norm(target_ned))
            if norm <= 0:
                return ""
            unit = target_ned / norm
            return f"ned:({unit[0]:.2f},{unit[1]:.2f},{unit[2]:.2f})"
        except Exception:
            return ""


__all__ = ["SimTargetOverlayRenderer"]
