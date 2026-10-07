"""Synthetic confirmation-frame rendering for simulator POIs."""

from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from navpy.modules.vision.sim.dock_artwork import create_platform_vehicle_sprite, create_dock_sprite


_LOCATION_SPRITE_FILES = {
    "building": "building.png",
    "vehicle": "platform_vehicle.png",
    "antenna": "antenna.png",
    "operations_site": "operations_site.png",
    "bridge": "bridge.png",
    "fuel": "fuel.png",
    "other": "other.png",
}


class SimFrameGenerator:
    """Generate simulated review frames with dock or location-type overlays."""

    def __init__(
            self,
            assets_dir: str,
            frame_size: Tuple[int, int] = (1920, 1080),
    ) -> None:
        self._frame_size = frame_size
        self._background = None
        self._sprites: Dict[str, np.ndarray] = {
            "dock": create_dock_sprite(),
            "vehicle": create_platform_vehicle_sprite(),
        }
        background_path = os.path.join(assets_dir, "background.png")
        if os.path.exists(background_path):
            background = cv2.imread(background_path)
            if background is not None:
                self._background = cv2.resize(background, frame_size)
        dock_path = os.path.join(assets_dir, "dock.png")
        if os.path.exists(dock_path):
            sprite = cv2.imread(dock_path, cv2.IMREAD_UNCHANGED)
            if sprite is not None:
                self._sprites["dock"] = sprite
        for location_type, filename in _LOCATION_SPRITE_FILES.items():
            path = os.path.join(assets_dir, filename)
            if not os.path.exists(path):
                continue
            sprite = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if sprite is not None:
                self._sprites[location_type] = sprite

    @property
    def is_available(self) -> bool:
        return self._background is not None and bool(self._sprites)

    def sprite_height_for(self, location_type: Optional[str]) -> int:
        sprite = self._get_sprite(location_type)
        return sprite.shape[0] if sprite is not None else 100

    def sprite_width_for(self, location_type: Optional[str]) -> int:
        sprite = self._get_sprite(location_type)
        return sprite.shape[1] if sprite is not None else 100

    @property
    def sprite_height(self) -> int:
        return self.sprite_height_for(None)

    def _get_sprite(self, location_type: Optional[str]) -> Optional[np.ndarray]:
        if location_type and location_type in self._sprites:
            return self._sprites[location_type]
        return self._sprites.get("dock")

    def generate_frame(
            self,
            detections: List[Tuple[float, float, float]],
            location_type: Optional[str] = None,
    ) -> Optional[Tuple[np.ndarray, List[Tuple]]]:
        if not self.is_available:
            return None
        sprite = self._get_sprite(location_type)
        if sprite is None:
            return None
        frame = self._background.copy()
        boxes = []
        width, height = self._frame_size
        for x_fraction, y_fraction, scale in detections:
            center_x = int(width * x_fraction)
            center_y = int(height * y_fraction)
            frame, box = self._composite(
                frame, sprite, center_x, center_y, scale,
            )
            boxes.append(box)
        return frame, boxes

    @staticmethod
    def _composite(
            background: np.ndarray,
            sprite: np.ndarray,
            center_x: int,
            center_y: int,
            scale: float,
    ) -> Tuple[np.ndarray, Tuple]:
        source_height, source_width = sprite.shape[:2]
        width = int(source_width * scale)
        height = int(source_height * scale)
        resized = cv2.resize(
            sprite, (width, height), interpolation=cv2.INTER_AREA,
        )
        if resized.shape[2] == 4:
            color = resized[:, :, :3]
            alpha = resized[:, :, 3] / 255.0
        else:
            color = resized
            alpha = np.ones((height, width))
        x = center_x - width // 2
        y = center_y - height // 2
        background_height, background_width = background.shape[:2]
        x1, y1 = max(0, x), max(0, y)
        x2, y2 = min(background_width, x + width), min(background_height, y + height)
        tx1, ty1 = x1 - x, y1 - y
        tx2, ty2 = tx1 + (x2 - x1), ty1 + (y2 - y1)
        for channel in range(3):
            background[y1:y2, x1:x2, channel] = (
                alpha[ty1:ty2, tx1:tx2]
                * color[ty1:ty2, tx1:tx2, channel]
                + (1 - alpha[ty1:ty2, tx1:tx2])
                * background[y1:y2, x1:x2, channel]
            )
        return background, (center_x, center_y, width, height)


__all__ = ["SimFrameGenerator"]
