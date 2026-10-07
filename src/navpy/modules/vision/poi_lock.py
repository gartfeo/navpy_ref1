"""Selected POI ownership for POI tracking.

A stable scene ID preserves track association only.

``PoiLock`` records *which* stable scene identity the operator/auto-selector
has chosen and follows it across frames. Identity continuity (re-recognising a
POI after occlusion, not confusing similar objects) is owned by the upstream
tracker + :class:`TrackIdentityResolver`; the lock simply trusts the stable id
it is given.

Design rules (post-redesign):
  - The lock does NOT perform its own spatial re-association or id rewriting.
    When the locked stable id is absent from the current tracks it stays lost
    and waits for the identity layer to reissue that id. This is *fail-closed*:
    we never silently jump the lock onto a different (possibly wrong) object.
  - All mutable state is guarded by a single mutex because force-lock requests
    arrive on the operator/API thread while ``select`` runs on the track thread.
"""
from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING, List, Optional

from navpy.modules.vision.geometry import iou_cxcywh

if TYPE_CHECKING:
    from navpy.modules.vision.multi_object_tracker import TrackedObject


class PoiLock:
    """Keeps ownership of a selected stable track id."""

    # How many select() cycles to keep trying a force request before giving up
    # (lets the tracker confirm the clicked/forced track).
    _FORCE_GRACE = 60

    def __init__(self, max_lost_frames: int = 120, auto_lock: bool = True):
        self._mutex = threading.RLock()
        self.locked_id: Optional[int] = None
        self.lost = 0
        self.max_lost = max_lost_frames
        self.auto_lock = bool(auto_lock)
        self._force_id: Optional[int] = None
        self._force_bbox: Optional[tuple[float, float, float, float]] = None
        self._force_ttl: int = 0

    # -- selection -------------------------------------------------------

    def select(
            self,
            tracks: List["TrackedObject"],
            frame_w: int,
            frame_h: int,
    ) -> Optional["TrackedObject"]:
        with self._mutex:
            return self._select_locked(tracks, frame_w, frame_h)

    def _select_locked(self, tracks, frame_w, frame_h):
        # 1) Pending bbox-click force: commit only when the matched track is
        #    confirmed, otherwise keep waiting (F22 — don't consume the click on
        #    an unconfirmed track and then drift).
        if self._force_bbox is not None:
            match = self._match_bbox(tracks, frame_w, frame_h)
            if match is not None and match.is_confirmed:
                self._commit(match)
                self._clear_force()
                return match
            self._tick_force_ttl()
            return None

        # 2) Pending id force.
        if self._force_id is not None:
            match = next((t for t in tracks if t.id == self._force_id), None)
            if match is not None and match.is_confirmed:
                self._commit(match)
                self._clear_force()
                return match
            self._tick_force_ttl()
            return None

        # 3) Follow the locked stable id. The identity layer owns continuity, so
        #    we only ever follow our exact id. If it is not present we stay lost
        #    (fail-closed) instead of guessing a nearby track. Once the loss
        #    budget expires the lock resets and we fall through to auto-lock.
        if self.locked_id is not None:
            match = next(
                (t for t in tracks if t.id == self.locked_id and t.is_confirmed),
                None,
            )
            if match is not None:
                self._commit(match)
                return match
            self._mark_lost()
            if self.locked_id is not None:
                return None

        # 4) Auto-lock: pick the most central confirmed track.
        if not self.auto_lock:
            return None
        confirmed = [t for t in tracks if t.is_confirmed]
        if not confirmed:
            return None
        cx0, cy0 = frame_w * 0.5, frame_h * 0.5
        best = min(confirmed, key=lambda t: (t.cx - cx0) ** 2 + (t.cy - cy0) ** 2)
        self._commit(best)
        return best

    # -- force requests --------------------------------------------------

    def force_lock(self, obj_id: int):
        """Request a lock on a specific stable id.

        Like :meth:`force_lock_bbox`, this leaves ``locked_id`` None until a
        confirmed track resolves the request (so a request for an id that never
        appears cannot leave a phantom lock). Use :attr:`has_pending_force`, not
        ``locked_id``, to test "is a POI being acquired?".
        """
        with self._mutex:
            self._force_id = obj_id
            self._force_bbox = None
            self._force_ttl = self._FORCE_GRACE
            self.lost = 0

    def force_lock_bbox(self, bbox_cxcywh):
        """Force-lock the track that best overlaps a clicked bbox (cx,cy,w,h)."""
        if bbox_cxcywh is None:
            return
        cx, cy, w, h = bbox_cxcywh
        with self._mutex:
            self._force_bbox = (float(cx), float(cy), float(w), float(h))
            self._force_id = None
            self._force_ttl = self._FORCE_GRACE
            self.lost = 0

    @property
    def has_pending_force(self) -> bool:
        with self._mutex:
            return self._force_id is not None or self._force_bbox is not None

    def reset(self):
        with self._mutex:
            self.locked_id = None
            self.lost = 0
            self._clear_force()

    # -- internals -------------------------------------------------------

    def _match_bbox(self, tracks, frame_w, frame_h):
        """Rank candidates by overlap with the clicked box.

        IoU is the primary signal (a strongly-overlapping POI beats a tiny
        track that merely has its centre inside an operator-inflated click box —
        F21). Containment is a secondary bonus; centre distance (normalised by
        the *frame* diagonal, not the click box) breaks remaining ties.
        """
        if self._force_bbox is None:
            return None
        fcx, fcy, fw, fh = self._force_bbox
        fx1, fy1, fx2, fy2 = fcx - fw * 0.5, fcy - fh * 0.5, fcx + fw * 0.5, fcy + fh * 0.5
        frame_diag = max(math.hypot(float(frame_w), float(frame_h)), 1.0)

        candidates = []
        for track in tracks:
            iou = iou_cxcywh(self._force_bbox, (track.cx, track.cy, track.w, track.h))
            center_inside = fx1 <= track.cx <= fx2 and fy1 <= track.cy <= fy2
            if iou <= 0.0 and not center_inside:
                continue
            center_dist = math.hypot(track.cx - fcx, track.cy - fcy) / frame_diag
            # IoU is strictly primary: a materially higher-IoU POI always wins.
            # Containment only breaks ties between equal-IoU candidates; centre
            # distance (frame-normalised) breaks any remaining tie.
            candidates.append((
                (-iou, 0 if center_inside else 1, center_dist, -track.confidence),
                track,
            ))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _commit(self, track: "TrackedObject"):
        self.locked_id = track.id
        self.lost = 0

    def _mark_lost(self):
        if self.locked_id is None:
            return
        self.lost += 1
        if self.lost > self.max_lost:
            self.reset()

    def _tick_force_ttl(self):
        self._force_ttl -= 1
        if self._force_ttl <= 0:
            self._clear_force()

    def _clear_force(self):
        self._force_id = None
        self._force_bbox = None
        self._force_ttl = 0
