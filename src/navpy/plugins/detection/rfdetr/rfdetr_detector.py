"""The rfdetr frame detector: RF-DETR-L, profile classes only (HITL step 23b; hitl_dock_detector.md section 3).

One detector with one job: the frame (squashed whole or as fixed tiles, rfdetr_tiling.py) goes through RF-DETR's own
pre- and post-processing (rfdetr_codec.py); rows below the profile's confidence and every class the profile does not
map are dropped; the kept class becomes its navpy class (phase A: COCO ``car`` -> ``dock``). No fallback model and
no other backend behind it: a frame it misses has no detection. Stateless per frame, like the other backends; the
box is in full-frame pixels.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.detection import Detection
from navpy.plugins.detection.rfdetr.rfdetr_codec import decode, preprocess, preprocess_tiles
from navpy.plugins.detection.rfdetr.rfdetr_runtime import RfDetrRuntime
from navpy.plugins.detection.rfdetr.rfdetr_settings import INPUT_TILES, RfDetrSettings
from navpy.plugins.detection.rfdetr.rfdetr_tiling import merge_tiles, tiles

# rfdetr's decode floor (gimbal-controller's DET_FLOOR); the profile's confidence is applied after it.
DECODE_FLOOR = 0.05


class RfDetrDetector:
    def __init__(self, runtime: RfDetrRuntime, settings: RfDetrSettings, min_confidence: float,
                 logger: Optional[ILogger] = None) -> None:
        self._runtime = runtime
        self._settings = settings
        self._min_confidence = float(min_confidence)
        self.device = runtime.device
        if logger is not None:
            logger.info(f"RF-DETR detector: variant={settings.variant} input={settings.input_mode} "
                        f"size={settings.input_px} classes={dict(settings.class_map)} "
                        f"conf={self._min_confidence:.2f} device={self.device}")

    def close(self) -> None:
        self._runtime.close()

    def detect(self, frame: np.ndarray) -> list[Detection]:
        height, width = frame.shape[:2]
        rows = self._tiled(frame, width, height) if self._settings.input_mode == INPUT_TILES \
            else self._squashed(frame, width, height)
        return self._detections(rows)

    def _squashed(self, frame: np.ndarray, width: int, height: int) -> np.ndarray:
        boxes, logits = self._runtime.infer(preprocess(frame, self._settings.input_px))
        return self._kept(decode(boxes[0], logits[0], width, height, DECODE_FLOOR, self._settings.num_select))

    def _tiled(self, frame: np.ndarray, width: int, height: int) -> np.ndarray:
        grid = tiles(width, height, self._settings.input_px)
        size = self._settings.input_px
        if all((t.width, t.height) == (size, size) for t in grid):
            blob = preprocess_tiles(np.stack([frame[t.y0:t.y0 + size, t.x0:t.x0 + size] for t in grid]))
        else:   # a frame shorter than a tile on an axis: that axis is squashed to the square
            blob = np.concatenate([preprocess(np.ascontiguousarray(frame[t.y0:t.y0 + t.height, t.x0:t.x0 + t.width]),
                                              size) for t in grid])
        boxes, logits = self._runtime.infer(blob)
        per_tile = [self._kept(decode(boxes[i], logits[i], t.width, t.height, DECODE_FLOOR,
                                      self._settings.num_select)) for i, t in enumerate(grid)]
        return merge_tiles(per_tile, grid, width, height)

    def _kept(self, rows: np.ndarray) -> np.ndarray:
        """Only the profile's classes, at or above its confidence, with a box (before any tile merge)."""
        keep = [self._settings.navpy_class(int(row[5])) is not None and row[4] >= self._min_confidence
                and row[2] > 0.0 and row[3] > 0.0 for row in rows]
        return rows[np.asarray(keep, bool)] if len(rows) else rows

    def _detections(self, rows: np.ndarray) -> list[Detection]:
        out = []
        for cx, cy, w, h, score, coco in rows:
            navpy_class: Optional[int] = self._settings.navpy_class(int(coco))
            out.append(Detection(cx=float(cx), cy=float(cy), w=float(w), h=float(h), confidence=float(score),
                                 class_id=int(navpy_class)))
        return out


__all__ = ["DECODE_FLOOR", "RfDetrDetector"]
