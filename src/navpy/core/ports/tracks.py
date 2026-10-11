"""Neutral tracked-object record every tracker backend produces."""

from __future__ import annotations

from dataclasses import dataclass


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
