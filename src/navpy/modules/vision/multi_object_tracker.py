"""Multi-object tracker orchestration and compatibility exports.

Manages persistent track IDs across frames, handling track birth,
confirmation, occlusion (coasting), death, and revival. Runs at a
different rate than inference and interpolates between sparse detections.
"""
from __future__ import annotations

import time
from typing import List, Optional, Tuple

import numpy as np

from navpy.modules.common.linear_assignment import hungarian
from navpy.modules.vision.geometry import clip_bbox_xyxy, iou_xyxy
from navpy.modules.vision.mot_state import (
    KalmanCV,
    TrackedObject,
    _Track,
    export_tracks,
)
from navpy.modules.vision.yolo_detector import Detection

__all__ = [
    "TrackedObject",
    "MultiObjectTracker",
    "KalmanCV",
    "hungarian",
    "iou_xyxy",
    "clip_bbox_xyxy",
]


class MultiObjectTracker:
    def __init__(
            self,
            max_age: int = 60,
            min_hits: int = 3,
            iou_weight: float = 0.65,
            dist_weight: float = 0.35,
            max_center_dist_px: float = 500.0,
            min_iou_gate: float = 0.02,
            class_mismatch_penalty: float = 0.35,
            revive_seconds: float = 2.5,
    ):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_w = iou_weight
        self.dist_w = dist_weight
        self.max_center_dist = max_center_dist_px
        self.min_iou_gate = min_iou_gate
        self.class_penalty = class_mismatch_penalty
        self.revive_seconds = revive_seconds

        self._tracks: List[_Track] = []
        self._lost: List[Tuple[_Track, float]] = []
        self._next_id = 1

    def _cleanup_lost(self, ts_now: float):
        if self.revive_seconds <= 0:
            self._lost = []
            return
        keep = []
        for t, ts_lost in self._lost:
            if (ts_now - ts_lost) <= self.revive_seconds:
                keep.append((t, ts_lost))
        self._lost = keep

    def _try_revive(self, det: Detection, ts_now: float) -> Optional[_Track]:
        if self.revive_seconds <= 0 or not self._lost:
            return None

        best_i = -1
        best_dist = 1e18

        for i, (t, ts_lost) in enumerate(self._lost):
            if det.class_id != t.class_id:
                continue
            dt = max(0.0, ts_now - t.ts)
            pred_cx = t.kf.cx + t.kf.vx * dt
            pred_cy = t.kf.cy + t.kf.vy * dt
            # A track that has been lost a while may carry an unreliable velocity;
            # cap the forward projection to the gate so it cannot teleport far
            # past the last known position and match the wrong returning object.
            disp = float(np.hypot(pred_cx - t.kf.cx, pred_cy - t.kf.cy))
            if disp > self.max_center_dist:
                scale = self.max_center_dist / disp
                pred_cx = t.kf.cx + (pred_cx - t.kf.cx) * scale
                pred_cy = t.kf.cy + (pred_cy - t.kf.cy) * scale
            dc = float(np.hypot(pred_cx - det.cx, pred_cy - det.cy))
            if dc < best_dist:
                best_dist = dc
                best_i = i

        if best_i < 0:
            return None

        if best_dist > self.max_center_dist:
            return None

        t, _ = self._lost.pop(best_i)
        t.reinit(det, ts_now)
        return t

    def _cost(self, tracks: List[_Track], dets: List[Detection]) -> np.ndarray:
        n, m = len(tracks), len(dets)
        cost = np.zeros((n, m), dtype=np.float32)
        for i, t in enumerate(tracks):
            tb = t.bbox_xyxy()
            tcx, tcy = t.kf.cx, t.kf.cy
            for j, d in enumerate(dets):
                db = d.xyxy
                iou = iou_xyxy(tb, db)
                dc = float(np.hypot(tcx - d.cx, tcy - d.cy))
                nd = min(1.0, dc / max(1.0, self.max_center_dist))
                c = self.iou_w * (1.0 - iou) + self.dist_w * nd
                if d.class_id != t.class_id:
                    c += self.class_penalty
                cost[i, j] = c
        return cost

    def update(self, dets: List[Detection], frame_w: int,
               frame_h: int) -> List[TrackedObject]:
        ts_now = time.time()
        self._cleanup_lost(ts_now)

        for track in self._tracks:
            track.predict(ts_now)

        if not self._tracks:
            self._spawn_or_revive(dets, ts_now)
            return self._export()

        if not dets:
            self._prune(ts_now, frame_w, frame_h, confirm=False)
            return self._export()

        assigned_dets = self._assign(dets, ts_now)
        self._spawn_or_revive(dets, ts_now, assigned_dets)
        self._prune(ts_now, frame_w, frame_h, confirm=True)
        return self._export()

    def _assign(self, dets: List[Detection], ts_now: float) -> set[int]:
        cost = self._cost(self._tracks, dets)
        pairs = hungarian(cost)
        assigned_dets: set[int] = set()
        for track_index, detection_index in pairs:
            track = self._tracks[track_index]
            detection = dets[detection_index]
            track_bbox = track.bbox_xyxy()
            detection_bbox = detection.xyxy
            iou = iou_xyxy(track_bbox, detection_bbox)
            center_distance = float(np.hypot(
                track.kf.cx - detection.cx, track.kf.cy - detection.cy))
            if (
                center_distance > self.max_center_dist
                and iou < self.min_iou_gate
            ):
                continue
            track.update(detection, ts_now)
            assigned_dets.add(detection_index)
        return assigned_dets

    def _spawn_or_revive(
        self,
        dets: List[Detection],
        ts_now: float,
        assigned_dets: set[int] | None = None,
    ) -> None:
        for index, detection in enumerate(dets):
            if assigned_dets is not None and index in assigned_dets:
                continue
            revived = self._try_revive(detection, ts_now)
            if revived is not None:
                self._tracks.append(revived)
            else:
                self._tracks.append(_Track(self._next_id, detection, ts_now))
                self._next_id += 1

    def _prune(
        self,
        ts_now: float,
        frame_w: int,
        frame_h: int,
        *,
        confirm: bool,
    ) -> None:
        alive = []
        for track in self._tracks:
            if confirm and track.hits >= self.min_hits:
                track.is_confirmed = True
            if self._should_keep_active(track, frame_w, frame_h):
                alive.append(track)
            else:
                self._lost.append((track, ts_now))
        self._tracks = alive

    def _should_keep_active(self, track: _Track, frame_w: int,
                            frame_h: int) -> bool:
        if track.missed > self.max_age:
            return False
        bbox = clip_bbox_xyxy(track.bbox_xyxy(), frame_w, frame_h)
        return (bbox[2] - bbox[0]) > 2 and (bbox[3] - bbox[1]) > 2

    def reset(self):
        self._tracks = []
        self._lost = []
        self._next_id = 1

    def _export(self) -> List[TrackedObject]:
        return export_tracks(self._tracks)
