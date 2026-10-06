"""Per-track kinematic memory for library tracker backends.

Coasts unseen tracks across empty detection batches and estimates velocity.
The caller passes one consistent ``timestamp`` for all of export/coast/prune in
a cycle. ``export`` additionally guards against non-positive steps (it keeps the
previous velocity instead of dividing by ~0), so a backward clock jump cannot
synthesize a huge spurious velocity even though the adapters stamp wall-clock
time.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from navpy.modules.vision.multi_object_tracker import TrackedObject

# Max seconds a track keeps being emitted (coasted) after empty detections.
_EMPTY_DETECTION_COAST_SECONDS = 0.30


@dataclass(frozen=True)
class _MemoryState:
    cx: float
    cy: float
    w: float
    h: float
    confidence: float
    class_id: int
    timestamp: float
    vx: float = 0.0
    vy: float = 0.0
    age: int = 0
    hits: int = 0
    missed: int = 0
    is_confirmed: bool = True


class _TrackExportMemory:
    def __init__(self, max_age_seconds: float):
        self.max_age_seconds = max_age_seconds
        self._state: Dict[int, _MemoryState] = {}

    def export(
            self,
            track_id: int,
            x1: float,
            y1: float,
            x2: float,
            y2: float,
            confidence,
            class_id,
            is_confirmed: bool,
            missed: int,
            timestamp: float,
    ) -> TrackedObject:
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        prev = self._state.get(track_id)
        if prev is None:
            vx = 0.0
            vy = 0.0
            age = 1
            hits = 1 if is_confirmed else 0
        else:
            dt = timestamp - prev.timestamp
            if dt <= 1e-3:
                # Sub-millisecond / non-positive step: keep the previous velocity
                # instead of dividing by a near-zero (or negative) dt, which would
                # synthesize an enormous spurious velocity.
                vx = prev.vx
                vy = prev.vy
            else:
                vx = (cx - prev.cx) / dt
                vy = (cy - prev.cy) / dt
            age = prev.age + 1
            hits = prev.hits + (1 if missed == 0 else 0)
        track = TrackedObject(
            id=track_id,
            cx=cx,
            cy=cy,
            w=w,
            h=h,
            confidence=float(confidence if confidence is not None else 0.0),
            class_id=int(class_id if class_id is not None else -1),
            age=age,
            hits=hits,
            missed=missed,
            is_confirmed=is_confirmed,
            timestamp=timestamp,
            vx=vx,
            vy=vy,
        )
        self._state[track_id] = _MemoryState(
            cx=track.cx,
            cy=track.cy,
            w=track.w,
            h=track.h,
            confidence=track.confidence,
            class_id=track.class_id,
            timestamp=track.timestamp,
            vx=track.vx,
            vy=track.vy,
            age=track.age,
            hits=track.hits,
            missed=track.missed,
            is_confirmed=track.is_confirmed,
        )
        return track

    def coast(self, timestamp: float, *, max_age_seconds: float) -> List[TrackedObject]:
        out: List[TrackedObject] = []
        for track_id, state in self._state.items():
            age_s = max(0.0, timestamp - state.timestamp)
            if age_s > max_age_seconds:
                continue
            out.append(
                TrackedObject(
                    id=track_id,
                    cx=state.cx + state.vx * age_s,
                    cy=state.cy + state.vy * age_s,
                    w=state.w,
                    h=state.h,
                    confidence=state.confidence,
                    class_id=state.class_id,
                    age=state.age + 1,
                    hits=state.hits,
                    missed=state.missed + 1,
                    is_confirmed=state.is_confirmed,
                    timestamp=timestamp,
                    vx=state.vx,
                    vy=state.vy,
                )
            )
        return out

    def prune(self, seen: set[int], timestamp: float) -> None:
        for track_id, state in list(self._state.items()):
            if track_id in seen:
                continue
            if timestamp - state.timestamp > self.max_age_seconds:
                del self._state[track_id]

    def clear(self) -> None:
        self._state.clear()
