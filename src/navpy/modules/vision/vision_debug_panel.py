"""Render one simulator camera panel without retaining the controller."""

from __future__ import annotations

import cv2
import numpy as np

from navpy.modules.vision.vision_debug_hud import SimTrackerHudRenderer
from navpy.modules.vision.vision_debug_pois import SimPoiOverlayRenderer
from navpy.modules.vision.vision_ui_ports import (
    AircraftAttitudeReader,
    SimDebugSnapshot,
)


class SimPanelRenderer:
    def __init__(
        self,
        attitude: AircraftAttitudeReader,
        tracker_hud: SimTrackerHudRenderer,
        pois: SimPoiOverlayRenderer,
    ) -> None:
        self._attitude = attitude
        self._tracker_hud = tracker_hud
        self._pois = pois

    def draw(
        self,
        image: np.ndarray,
        y_offset: int,
        width: int,
        height: int,
        snapshot: SimDebugSnapshot,
        camera_pitch_deg: float,
    ) -> None:
        image[y_offset:y_offset + height, :] = (30, 30, 30)
        cv2.rectangle(
            image,
            (0, y_offset),
            (width - 1, y_offset + height - 1),
            (60, 60, 60),
            1,
        )
        image_width = snapshot.image_width
        image_height = snapshot.image_height
        scale_x = width / image_width
        scale_y = height / image_height
        center_x, center_y = width // 2, y_offset + height // 2
        cv2.line(
            image,
            (center_x - 20, center_y),
            (center_x + 20, center_y),
            (60, 60, 60),
            1,
        )
        cv2.line(
            image,
            (center_x, center_y - 20),
            (center_x, center_y + 20),
            (60, 60, 60),
            1,
        )
        intrinsics = snapshot.intrinsics
        image_center_x = (
            float(intrinsics[0, 2]) if intrinsics is not None else image_width / 2
        )
        image_center_y = (
            float(intrinsics[1, 2]) if intrinsics is not None else image_height / 2
        )
        self._draw_horizon(
            image,
            y_offset,
            width,
            height,
            snapshot,
            intrinsics,
            image_center_y,
            scale_y,
            camera_pitch_deg,
        )
        self._pois.draw(
            image,
            y_offset,
            scale_x,
            scale_y,
            image_center_x,
            image_center_y,
            snapshot.detections,
        )
        self._draw_mount_header(image, y_offset, snapshot)
        self._tracker_hud.draw(
            image,
            y_offset,
            height,
            snapshot,
        )

    def _draw_horizon(
        self,
        image: np.ndarray,
        y_offset: int,
        width: int,
        height: int,
        snapshot: SimDebugSnapshot,
        intrinsics: np.ndarray | None,
        image_center_y: float,
        scale_y: float,
        camera_pitch_deg: float,
    ) -> None:
        if intrinsics is None or abs(camera_pitch_deg) >= 85:
            return
        horizon_y = (
            image_center_y
            + float(intrinsics[1, 1])
            * np.tan(np.radians(camera_pitch_deg))
        ) * scale_y
        gimbal = snapshot.gimbal
        aircraft_attitude = self._attitude()
        effective_roll = (
            0.0
            if gimbal.roll_stabilize
            else aircraft_attitude.roll if aircraft_attitude is not None else 0.0
        )
        delta_y = (width / 2) * np.tan(np.radians(effective_roll))
        point_left = (0, int(horizon_y + delta_y))
        point_right = (width, int(horizon_y - delta_y))
        panel = image[y_offset:y_offset + height, 0:width]
        sky = np.array(
            [[0, 0], [width, 0], point_right, point_left],
            dtype=np.int32,
        )
        ground = np.array(
            [point_left, point_right, [width, height], [0, height]],
            dtype=np.int32,
        )
        cv2.fillPoly(panel, [sky], (60, 40, 20))
        cv2.fillPoly(panel, [ground], (20, 40, 30))
        cv2.line(panel, point_left, point_right, (0, 140, 255), 1, cv2.LINE_AA)

    @staticmethod
    def _draw_mount_header(
        image: np.ndarray,
        y_offset: int,
        snapshot: SimDebugSnapshot,
    ) -> None:
        gimbal = snapshot.gimbal
        zoom = snapshot.current_zoom
        zoom_text = f"  Z:{zoom}x" if zoom else ""
        text = (
            f"{snapshot.source_name}  Y:{gimbal.att.yaw:+.1f}  "
            f"P:{gimbal.att.pitch:+.1f}  R:{gimbal.att.roll:+.1f}{zoom_text}"
        )
        cv2.putText(
            image,
            text,
            (5, y_offset + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (200, 200, 200),
            1,
        )


__all__ = ["SimPanelRenderer"]
