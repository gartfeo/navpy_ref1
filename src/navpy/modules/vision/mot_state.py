"""State objects owned by the built-in multi-object tracker."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from navpy.modules.vision.yolo_detector import Detection


@dataclass(frozen=True)
class TrackedObject:
    id: int
    cx: float
    cy: float
    w: float
    h: float
    confidence: float
    class_id: int
    age: int
    hits: int
    missed: int
    is_confirmed: bool
    timestamp: float
    vx: float
    vy: float


class KalmanCV:
    def __init__(self, cx: float, cy: float) -> None:
        self.x = np.array([cx, cy, 0.0, 0.0], dtype=np.float32)
        self.P = np.diag([50.0, 50.0, 500.0, 500.0]).astype(np.float32)

        self.R = np.diag([8.0, 8.0]).astype(np.float32)
        self.q_pos = 2.0
        self.q_vel = 40.0

    def predict(self, dt: float) -> None:
        dt = max(1e-3, float(dt))
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float32)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel]).astype(np.float32)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray) -> None:
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=np.float32)
        y = z - (H @ self.x)
        S = H @ self.P @ H.T + self.R
        # Use solve instead of inv for numerical stability (handles near-singular S)
        # K = P @ H.T @ S^-1  =>  K.T = S^-1.T @ (P @ H.T).T  =>  solve(S.T, (P @ H.T).T).T
        K = np.linalg.solve(S.T, (self.P @ H.T).T).T
        self.x = self.x + K @ y
        I = np.eye(4, dtype=np.float32)
        self.P = (I - K @ H) @ self.P

    @property
    def cx(self) -> float: return float(self.x[0])

    @property
    def cy(self) -> float: return float(self.x[1])

    @property
    def vx(self) -> float: return float(self.x[2])

    @property
    def vy(self) -> float: return float(self.x[3])


class _Track:
    def __init__(self, tid: int, det: Detection, ts: float) -> None:
        self.id = tid
        self.kf = KalmanCV(det.cx, det.cy)
        self.w = det.w
        self.h = det.h
        self.class_id = det.class_id
        self.confidence = det.confidence

        self.age = 1
        self.hits = 1
        self.missed = 0
        self.ts = ts
        self.is_confirmed = False

    def predict(self, ts_now: float) -> None:
        dt = ts_now - self.ts
        self.kf.predict(dt)
        self.age += 1
        self.missed += 1
        self.ts = ts_now

    def update(self, det: Detection, ts_now: float) -> None:
        dt = ts_now - self.ts
        self.kf.predict(dt)
        self.kf.update(np.array([det.cx, det.cy], dtype=np.float32))

        a = 0.7
        self.w = a * det.w + (1 - a) * self.w
        self.h = a * det.h + (1 - a) * self.h

        self.confidence = det.confidence
        self.class_id = det.class_id

        self.hits += 1
        self.missed = 0
        self.age += 1
        self.ts = ts_now

    def reinit(self, det: Detection, ts_now: float) -> None:
        # revive from lost cache with same ID
        self.kf = KalmanCV(det.cx, det.cy)
        self.w = det.w
        self.h = det.h
        self.class_id = det.class_id
        self.confidence = det.confidence
        self.age += 1
        self.hits = max(self.hits, 1)
        self.missed = 0
        self.ts = ts_now

    def bbox_xyxy(self) -> np.ndarray:
        cx, cy = self.kf.cx, self.kf.cy
        x1 = cx - self.w * 0.5
        y1 = cy - self.h * 0.5
        x2 = cx + self.w * 0.5
        y2 = cy + self.h * 0.5
        return np.array([x1, y1, x2, y2], dtype=np.float32)


def export_tracks(tracks: list[_Track]) -> list[TrackedObject]:
    return [
        TrackedObject(
            id=track.id,
            cx=track.kf.cx,
            cy=track.kf.cy,
            w=float(track.w),
            h=float(track.h),
            confidence=float(track.confidence),
            class_id=int(track.class_id),
            age=int(track.age),
            hits=int(track.hits),
            missed=int(track.missed),
            is_confirmed=bool(track.is_confirmed),
            timestamp=float(track.ts),
            vx=float(track.kf.vx),
            vy=float(track.kf.vy),
        )
        for track in tracks
    ]


__all__ = ["KalmanCV", "TrackedObject"]
