"""Pure crop geometry and annotation rendering for GCS thumbnails."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class TargetInfo:
    target_id: int
    bbox: tuple[float, float, float, float]
    confidence: float = 1.0
    class_name: str = "Detection"


def crop_target(
    frame: np.ndarray,
    bbox: tuple[float, float, float, float],
    size: tuple[int, int],
) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """Crop around one target and return its coordinates in the crop."""
    frame_height, frame_width = frame.shape[:2]
    cx, cy, bbox_width, bbox_height = bbox
    bbox_max = max(bbox_width, bbox_height, 1)
    half_height = max(bbox_max * 1.25, 75)
    half_width = max(bbox_max * 1.25, 100)

    thumbnail_width, thumbnail_height = size
    target_aspect = thumbnail_width / thumbnail_height
    current_aspect = half_width / half_height
    if current_aspect > target_aspect:
        half_height = half_width / target_aspect
    else:
        half_width = half_height * target_aspect

    x1 = cx - half_width
    y1 = cy - half_height
    x2 = cx + half_width
    y2 = cy + half_height
    if x1 < 0:
        x2 -= x1
        x1 = 0
    if y1 < 0:
        y2 -= y1
        y1 = 0
    if x2 > frame_width:
        x1 -= x2 - frame_width
        x2 = frame_width
    if y2 > frame_height:
        y1 -= y2 - frame_height
        y2 = frame_height

    x1 = int(max(0, x1))
    y1 = int(max(0, y1))
    x2 = int(min(frame_width, x2))
    y2 = int(min(frame_height, y2))
    cropped = frame[y1:y2, x1:x2].copy()
    adjusted = (bbox[0] - x1, bbox[1] - y1, bbox[2], bbox[3])
    return cropped, adjusted


def draw_low_res_banner(img: np.ndarray) -> None:
    label = "LOW-RES"
    height, width = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = max(0.5, height / 400)
    thickness = max(1, round(height / 250))
    (text_width, text_height), baseline = cv2.getTextSize(
        label, font, font_scale, thickness
    )
    pad = max(4, height // 100)
    x2 = width - pad
    y1 = pad
    x1 = x2 - text_width - 2 * pad
    y2 = y1 + text_height + 2 * pad
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 220), -1)
    cv2.putText(
        img,
        label,
        (x1 + pad, y2 - pad - baseline // 2),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def draw_target_box(
    img: np.ndarray,
    target_id: int,
    bbox: tuple[float, float, float, float],
    confidence: float = 1.0,
    class_name: str = "Detection",
) -> None:
    frame_height, frame_width = img.shape[:2]
    cx, cy, width, height = bbox
    pad = max(max(width, height) * 0.5, 20)
    x1 = max(0, min(int(cx - width * 0.5 - pad), frame_width - 1))
    y1 = max(0, min(int(cy - height * 0.5 - pad), frame_height - 1))
    x2 = max(0, min(int(cx + width * 0.5 + pad), frame_width - 1))
    y2 = max(0, min(int(cy + height * 0.5 + pad), frame_height - 1))

    color = (0, 69, 255)
    thickness = max(1, round(frame_height / 400))
    font_scale = max(0.35, frame_height / 600)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

    label = f"D{target_id} {class_name} {int(confidence * 100)}%"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_thickness = max(1, round(frame_height / 400))
    (label_width, label_height), _ = cv2.getTextSize(
        label, font, font_scale, font_thickness
    )
    label_x = x1
    label_y = max(label_height + 10, y1 - 8)
    cv2.rectangle(
        img,
        (label_x, label_y - label_height - 6),
        (label_x + label_width + 4, label_y + 4),
        color,
        -1,
    )
    cv2.putText(
        img,
        label,
        (label_x + 2, label_y),
        font,
        font_scale,
        (255, 255, 255),
        font_thickness,
    )


__all__ = ["TargetInfo", "crop_target", "draw_low_res_banner", "draw_target_box"]
