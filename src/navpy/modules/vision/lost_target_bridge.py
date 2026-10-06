"""Visual bridge for the locked target across detection gaps.

When the locked object's detection drops (small, blurred, low-conf — YOLO
simply does not fire), nothing downstream can re-bind what was never detected.
This bridge follows the actual pixels with a normalized-cross-correlation
template search in a local window, so the identity layer's re-bind anchor
tracks the real object instead of a stale constant-velocity prediction.

It is deliberately NOT a tracker: it emits position *hints* only. Detections
never come from here, and the identity layer's appearance gates still apply,
so template drift cannot steal the id — at worst a hint is wrong and the
re-bind falls back to fail-closed behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class BridgeHit:
    cx: float
    cy: float


class LostTargetBridge:
    """NCC template follower for ONE target (the locked one)."""

    def __init__(
            self,
            *,
            min_score: float = 0.55,
            search_margin: float = 1.5,
            max_bridge_seconds: float = 8.0,
            min_template_std: float = 6.0,
    ):
        # Below this correlation the target is considered not visible (e.g.
        # occluded) and NO hint is emitted — a low-confidence match must not
        # drag the re-bind anchor onto background.
        self._min_score = float(min_score)
        # A near-uniform crop carries no information and breaks the fail-closed
        # contract: TM_CCOEFF_NORMED scores a zero-variance template 1.0 against
        # ANYTHING, so min_score cannot reject it. Refuse to track such crops.
        self._min_template_std = float(min_template_std)
        # Search window half-extent, in units of the template size.
        self._search_margin = float(search_margin)
        self._max_bridge_seconds = float(max_bridge_seconds)
        # Per-search scale steps RELATIVE to the current cumulative scale; the
        # winner updates the cumulative scale so zoom drift is followed
        # incrementally. The template itself is re-derived each search from the
        # crisp ORIGINAL crop (not the previous scaled one) so repeated zoom
        # bridging does not accumulate resampling blur, and the cumulative
        # scale is clamped so the template can never grow/shrink unboundedly.
        self._search_scales: Tuple[float, ...] = (0.93, 1.0, 1.075)
        self._scale_min, self._scale_max = 0.3, 3.0
        self._template0: Optional[np.ndarray] = None   # original-resolution crop
        self._scale: float = 1.0                       # cumulative scale vs original
        self._center: Optional[Tuple[float, float]] = None
        self._lost_since: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._template0 is not None

    def observe(self, frame: np.ndarray, bbox_cxcywh, now: float) -> None:
        """Refresh the template while the target is VISIBLE (cheap crop copy)."""
        crop = _crop(frame, bbox_cxcywh)
        if crop is None or float(np.std(crop)) < self._min_template_std:
            return  # no/flat crop: keep the previous template (or stay inactive)
        self._template0 = crop      # crisp original; scaled copies derive from it
        self._scale = 1.0
        self._center = (float(bbox_cxcywh[0]), float(bbox_cxcywh[1]))
        self._lost_since = None

    def search(self, frame: np.ndarray, now: float) -> Optional[BridgeHit]:
        """Follow the target while it is NOT detected. Returns a hint or None."""
        if self._template0 is None or self._center is None or frame is None:
            return None
        if self._lost_since is None:
            self._lost_since = now
        elif now - self._lost_since > self._max_bridge_seconds:
            self.reset()
            return None

        th0, tw0 = self._template0.shape[:2]
        # current apparent size = original * cumulative scale
        cur_w, cur_h = int(tw0 * self._scale), int(th0 * self._scale)
        cx, cy = self._center
        mx, my = int(cur_w * self._search_margin), int(cur_h * self._search_margin)
        x1 = max(0, int(cx - cur_w / 2) - mx)
        y1 = max(0, int(cy - cur_h / 2) - my)
        x2 = min(frame.shape[1], int(cx + cur_w / 2) + mx)
        y2 = min(frame.shape[0], int(cy + cur_h / 2) + my)
        window = frame[y1:y2, x1:x2]

        # Multi-scale search around the current cumulative scale. Each candidate
        # template is resized from the ORIGINAL crop (no compounding blur), and
        # the cumulative scale is clamped so it can never run away.
        best: Optional[Tuple[float, Tuple[int, int], int, int, float]] = None
        for s in self._search_scales:
            cscale = min(self._scale_max, max(self._scale_min, self._scale * s))
            sw, sh = max(8, int(tw0 * cscale)), max(8, int(th0 * cscale))
            if window.shape[0] < sh or window.shape[1] < sw:
                continue
            tpl = self._template0 if cscale == 1.0 else cv2.resize(self._template0, (sw, sh))
            result = cv2.matchTemplate(window, tpl, cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(result)
            if not np.isfinite(score):
                continue
            if best is None or score > best[0]:
                best = (float(score), loc, sw, sh, cscale)
        if best is None or best[0] < self._min_score:
            return None  # occluded / gone: do not drag the anchor
        _, loc, sw, sh, cscale = best
        hit_cx = x1 + loc[0] + sw / 2.0
        hit_cy = y1 + loc[1] + sh / 2.0
        # Follow the target: position AND cumulative scale track the winner.
        self._scale = cscale
        self._center = (hit_cx, hit_cy)
        return BridgeHit(cx=hit_cx, cy=hit_cy)

    def reset(self) -> None:
        self._template0 = None
        self._scale = 1.0
        self._center = None
        self._lost_since = None


def _crop(frame: np.ndarray, bbox_cxcywh) -> Optional[np.ndarray]:
    if frame is None or bbox_cxcywh is None:
        return None
    cx, cy, w, h = (float(v) for v in bbox_cxcywh)
    x1 = max(0, int(cx - w / 2))
    y1 = max(0, int(cy - h / 2))
    x2 = min(frame.shape[1], int(cx + w / 2))
    y2 = min(frame.shape[0], int(cy + h / 2))
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None
    return frame[y1:y2, x1:x2].copy()
