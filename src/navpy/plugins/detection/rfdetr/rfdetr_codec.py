"""RF-DETR's own pre- and post-processing (HITL step 23b; docs/design/hitl_dock_detector.md section 3).

Copied in rule from gimbal-controller ``gimbal/rfdetr_detect.py`` (its docs/CAR-MOT-LIVE.md "RF-DETR on orin1"
holds the parity evidence against rfdetr's own ``predict()``):

- pre: a BGR uint8 picture squashed to the model's square (no letterbox), bilinear without antialias, RGB,
  ImageNet-normalised, as (1, 3, size, size) float32. The resize rounds each pixel to 8 bits where rfdetr resizes in
  float (at most 1/255 before normalisation), kept as there;
- post: the float32 sigmoid of every query/class pair of the (Q, 91) logits, the ``num_select`` best ranked by score
  then by flattened index (rfdetr's stable sort), then score > the floor; no NMS. Class slots are COCO category ids
  (car = 3); only COCO's 80 are mapped to the 0-79 index, the rest are dropped.

Boxes come back in pixels of the picture that was squashed, clipped to it: rows (cx, cy, w, h, score, class 0-79).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
LOGIT_SLOTS = 91
# COCO's category ids in index order: the model's logit slot (category id) -> the COCO 0-79 index.
_COCO_CATEGORY_IDS = (
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 31, 32, 33, 34, 35,
    36, 37, 38, 39, 40, 41, 42, 43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
    67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90,
)
COCO91_TO_80 = {category: index for index, category in enumerate(_COCO_CATEGORY_IDS)}
# The COCO 0-79 names the profile may name (phase A keeps only ``truck``).
COCO80_NAMES = {"person": 0, "bicycle": 1, "car": 2, "motorcycle": 3, "bus": 5, "truck": 7}
# (3, 256): channel c (RGB) of an 8-bit value v, ImageNet-normalised.
_LUT = ((np.arange(256, dtype=np.float32)[None] / np.float32(255.0) - MEAN[:, None]) / STD[:, None]).astype(np.float32)
_SLOT_TO_80 = np.full(LOGIT_SLOTS, -1, np.int64)
for _slot, _index in COCO91_TO_80.items():
    _SLOT_TO_80[_slot] = _index


class RfDetrOutputError(ValueError):
    """The model's outputs are not (Q, 4) boxes and (Q, 91) logits, or are not finite."""


def preprocess(image_bgr: np.ndarray, size: int, out: Optional[np.ndarray] = None) -> np.ndarray:
    """A BGR uint8 picture -> (1, 3, size, size) float32 RGB, ImageNet-normalised, squashed to the square."""
    import cv2
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.dtype != np.uint8:
        raise ValueError("the frame is not a 3-channel 8-bit picture")
    small = cv2.resize(image_bgr, (size, size), interpolation=cv2.INTER_LINEAR)
    if out is None:
        out = np.empty((1, 3, size, size), np.float32)
    for channel in range(3):
        np.take(_LUT[channel], small[:, :, 2 - channel], out=out[0, channel])
    return out


def preprocess_tiles(tiles_bgr: np.ndarray) -> np.ndarray:
    """(N, size, size, 3) BGR uint8 native tiles -> (N, 3, size, size) float32 RGB, ImageNet-normalised; the same
    values as ``preprocess`` of each tile (a resize to its own size is the identity), in one pass per channel."""
    if tiles_bgr.ndim != 4 or tiles_bgr.shape[3] != 3 or tiles_bgr.dtype != np.uint8 \
            or tiles_bgr.shape[1] != tiles_bgr.shape[2]:
        raise ValueError("the tiles are not square 3-channel 8-bit pictures")
    out = np.empty((tiles_bgr.shape[0], 3, tiles_bgr.shape[1], tiles_bgr.shape[2]), np.float32)
    for channel in range(3):
        np.take(_LUT[channel], tiles_bgr[..., 2 - channel], out=out[:, channel])
    return out


def _top(score_all: np.ndarray, k: int) -> np.ndarray:
    """The k best flattened pairs, by score then by index (rfdetr's stable sort)."""
    if k < score_all.size:
        kth = np.partition(score_all, score_all.size - k)[score_all.size - k]
        above = np.flatnonzero(score_all > kth)
        at = np.flatnonzero(score_all == kth)[:k - len(above)]
        top = np.concatenate([above, at])
    else:
        top = np.arange(score_all.size)
    return top[np.lexsort((top, -score_all[top]))]


def _squeeze(array: np.ndarray) -> np.ndarray:
    return array.reshape(array.shape[-2:]) if array.ndim == 3 and array.shape[0] == 1 else array


def decode(boxes: np.ndarray, logits: np.ndarray, width: int, height: int, floor: float, num_select: Optional[int] = None) -> np.ndarray:
    """One picture's raw outputs -> (N, 6) rows (cx, cy, w, h, score, COCO 0-79 class) in its pixels, best first."""
    b = _squeeze(np.asarray(boxes))
    lg = _squeeze(np.asarray(logits))
    if b.ndim != 2 or b.shape[1] != 4 or lg.ndim != 2 or lg.shape[0] != b.shape[0] or lg.shape[1] != LOGIT_SLOTS:
        raise RfDetrOutputError(f"the model's outputs have shapes {np.shape(boxes)} and {np.shape(logits)}, "
                                f"not (Q, 4) and (Q, {LOGIT_SLOTS})")
    b = b.astype(np.float32)
    lg = lg.astype(np.float32)
    if not (np.isfinite(b).all() and np.isfinite(lg).all()):
        raise RfDetrOutputError("the model's output is not finite")
    queries, classes = lg.shape
    k = queries if num_select is None else min(int(num_select), queries * classes)
    score_all = (1.0 / (1.0 + np.exp(-np.clip(lg.reshape(-1), -88.0, 88.0)))).astype(np.float32)
    top = _top(score_all, k)
    score = score_all[top]
    keep = score > floor
    top, score = top[keep], score[keep]
    query, slot = top // classes, top % classes
    cls = _SLOT_TO_80[slot]
    coco = cls >= 0
    query, score, cls = query[coco], score[coco], cls[coco]
    if not len(query):
        return np.zeros((0, 6), np.float32)
    cx, cy, w, h = b[query].T
    x0 = np.clip((cx - w / 2) * width, 0.0, width)
    y0 = np.clip((cy - h / 2) * height, 0.0, height)
    x1 = np.clip((cx + w / 2) * width, 0.0, width)
    y1 = np.clip((cy + h / 2) * height, 0.0, height)
    return np.stack([(x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, score, cls.astype(np.float32)],
                    axis=1).astype(np.float32)


__all__ = ["COCO80_NAMES", "COCO91_TO_80", "LOGIT_SLOTS", "MEAN", "STD", "RfDetrOutputError", "decode", "preprocess",
           "preprocess_tiles"]
