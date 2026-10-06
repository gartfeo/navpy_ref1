"""Comprehensive REAL-SIYI tracking validation.

Records a live SIYI clip, replays the production tracking stack, and writes
recognition, identity-stability, occlusion, and camera-motion evidence under
``validation_evidence/``.

Usage: python validate_tracking.py [record_seconds]   (default 90)
"""

from __future__ import annotations

import os
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np


ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "src"))

from scripts.python.validate_tracking_config import (
    BUS,
    CAR,
    CLIP,
    EVID,
    LOG,
    METRICS_PATH,
    MODEL,
    TRUCK,
    URL,
    parse_record_seconds,
)


REC_SECS = parse_record_seconds(sys.argv)
_METRICS = METRICS_PATH

from navpy.modules.vision.appearance import create_appearance_embedder
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.geometry import cxcywh_to_xyxy
from navpy.modules.vision.lost_target_bridge import LostTargetBridge
from navpy.modules.vision.target_lock import TargetLock
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import create_tracker_backend
from navpy.modules.vision.yolo_detector import YoloDetector
from scripts.python.validate_tracking_camera_motion import (
    camera_motion as _camera_motion,
)
from scripts.python.validate_tracking_metrics import (
    append_metric_line,
    associate,
    find_occ,
    iou,
)
from scripts.python.validate_tracking_occlusion import (
    annotate,
    report_occurrence,
)
from scripts.python.validate_tracking_orchestration import run_validation
from scripts.python.validate_tracking_runtime import (
    build_stack,
    record_clip,
    reset_all,
    step,
)

def emit(s: str = "") -> None:
    append_metric_line(s, metrics_path=_METRICS)


def build():
    return build_stack(MODEL, LOG)


def record():
    return record_clip(URL, CLIP, REC_SECS, LOG, emit)


def report_occ(ev, c, fps):
    return report_occurrence(ev, c, fps, CLIP, EVID, emit)


def camera_motion(c, fps):
    return _camera_motion(c, fps, CLIP, emit)


def main() -> None:
    run_validation(
        REC_SECS,
        _METRICS,
        URL,
        MODEL,
        CLIP,
        EVID,
        LOG,
    )


if __name__ == "__main__":
    main()
