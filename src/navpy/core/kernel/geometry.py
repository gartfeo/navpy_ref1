"""Shared bounding-box geometry helpers for the vision package.

Single source of truth for IoU and box-format conversion so detector,
poi_lock, deep_search, and the trackers cannot drift apart.

Box formats:
  - xyxy:   (x1, y1, x2, y2) corner coordinates in pixels
  - cxcywh: (cx, cy, w, h)   center + size in pixels
"""
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

BBox = Tuple[float, float, float, float]


def cxcywh_to_xyxy(bbox: Sequence[float]) -> BBox:
    cx, cy, w, h = (float(v) for v in bbox)
    half_w = w * 0.5
    half_h = h * 0.5
    return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)



def iou_xyxy(a: Sequence[float], b: Sequence[float]) -> float:
    """IoU of two xyxy boxes. Returns 0.0 for malformed boxes (x2<x1 or y2<y1)."""
    ax1, ay1, ax2, ay2 = (float(v) for v in a)
    bx1, by1, bx2, by2 = (float(v) for v in b)
    if ax2 < ax1 or ay2 < ay1 or bx2 < bx1 or by2 < by1:
        return 0.0

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0.0 else 0.0


def iou_cxcywh(a: Sequence[float], b: Sequence[float]) -> float:
    """IoU of two cxcywh boxes."""
    return iou_xyxy(cxcywh_to_xyxy(a), cxcywh_to_xyxy(b))


def clip_bbox_xyxy(bb: np.ndarray, w: int, h: int) -> np.ndarray:
    """Clip an xyxy box to the [0, w-1] x [0, h-1] image bounds (copy)."""
    bb = bb.copy()
    bb[0] = np.clip(bb[0], 0, w - 1)
    bb[2] = np.clip(bb[2], 0, w - 1)
    bb[1] = np.clip(bb[1], 0, h - 1)
    bb[3] = np.clip(bb[3], 0, h - 1)
    return bb
