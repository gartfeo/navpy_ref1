"""Paths, constants, and positional CLI parsing for tracking validation."""

from __future__ import annotations

import os
from collections.abc import Sequence
from types import SimpleNamespace


ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
EVID = os.path.join(ROOT, "validation_evidence")
os.makedirs(EVID, exist_ok=True)

URL = "rtsp://192.168.144.25:8554/main.264"
MODEL = os.path.join(ROOT, ".models", "yolov8s.pt")
CLIP = os.path.join(EVID, "siyi_scene.mp4")
CAR, BUS, TRUCK = 2, 5, 7

LOG = SimpleNamespace(
    info=lambda *args, **kwargs: None,
    warning=lambda *args, **kwargs: None,
    error=lambda *args, **kwargs: None,
)
METRICS_PATH = os.path.join(EVID, "metrics.txt")


def parse_record_seconds(argv: Sequence[str]) -> float:
    return float(argv[1]) if len(argv) > 1 else 90.0


__all__ = [
    "BUS",
    "CAR",
    "CLIP",
    "EVID",
    "LOG",
    "METRICS_PATH",
    "MODEL",
    "ROOT",
    "TRUCK",
    "URL",
    "parse_record_seconds",
]
