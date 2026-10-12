"""Code-rendered D-pad symbol for synthetic confirmation images."""

import cv2
import numpy as np


# Keep the existing confirmation sprite's canvas and aspect ratio. These are
# image pixels, not physical dock dimensions or detection parameters.
DOCK_SPRITE_WIDTH_PX = 652
DOCK_SPRITE_HEIGHT_PX = 442
VEHICLE_SPRITE_SIZE_PX = 600


def create_dock_sprite() -> np.ndarray:
    """Return a fresh BGRA demo pad; this symbol is not a trained detector."""
    sprite = np.zeros((DOCK_SPRITE_HEIGHT_PX, DOCK_SPRITE_WIDTH_PX, 4), np.uint8)
    cv2.rectangle(sprite, (0, 0), (651, 441), (74, 54, 22, 255), -1)
    cv2.rectangle(sprite, (14, 14), (637, 427), (224, 224, 224, 255), 10)
    # Center the same D symbol used by the GCS dock marker.
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 9.0, 18
    (width, height), _ = cv2.getTextSize("D", font, scale, thickness)
    origin = ((DOCK_SPRITE_WIDTH_PX - width) // 2,
              (DOCK_SPRITE_HEIGHT_PX + height) // 2)
    cv2.putText(sprite, "D", origin, font, scale, (255, 255, 255, 255), thickness, cv2.LINE_AA)
    return sprite


def create_platform_vehicle_sprite() -> np.ndarray:
    """Draw a moving platform vehicle carrying a dock on the existing square canvas."""
    sprite = np.zeros((VEHICLE_SPRITE_SIZE_PX, VEHICLE_SPRITE_SIZE_PX, 4), np.uint8)
    # Top view: four wheels, a plain cargo platform, and a cab at the front.
    for x in (70, 470):
        for y in (100, 410):
            cv2.rectangle(sprite, (x, y), (x + 60, y + 95), (40, 40, 40, 255), -1)
    cv2.rectangle(sprite, (115, 35), (485, 565), (210, 210, 210, 255), -1)
    cv2.rectangle(sprite, (135, 55), (465, 165), (115, 150, 170, 255), -1)
    pad = cv2.resize(create_dock_sprite(), (330, 224), interpolation=cv2.INTER_AREA)
    sprite[255:479, 135:465] = pad
    return sprite
