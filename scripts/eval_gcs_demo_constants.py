"""Shared constants for the exact three-UAV GCS demo evaluator."""

from __future__ import annotations

import hashlib
import os
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

GCS_LAUNCH = ROOT / "scripts" / "gcs_launch.py"
GCS_STOP = ROOT / "scripts" / "gcs_stop.py"
BACKEND_LOG = ROOT / ".logs" / "gcs" / "backend.log"
SCENARIO_MANIFEST_PATH = ROOT / "scripts" / "eval_gcs_demo_scenario.json"

EXPECTED_UAV_COUNT = 3
EXPECTED_NAV_LAST_WP_ORDINAL = 2
SIM_SPEEDUP = 10.0
NAVIGATION_SPEEDUP_OVERRIDE = 0.0
FINAL_APPROACH_ROLL_LIMIT_DEG = 45.0
VISION_PROFILE = "siyi_zr10"
VISION_MOUNT_PITCH_DEG = -14.0
FORBIDDEN_VISION_PROFILE = "ideal_360"
VISION_DETECT_HZ = 30.0

_ROOT_LOCK_KEY = hashlib.sha256(
    os.path.normcase(str(ROOT.resolve())).encode("utf-8")
).hexdigest()[:16]
EVALUATOR_LOCK_PATH = (
    Path(tempfile.gettempdir()) / f"navpy-gcs-demo-{_ROOT_LOCK_KEY}.lock"
)


__all__ = [
    "BACKEND_LOG",
    "EVALUATOR_LOCK_PATH",
    "EXPECTED_UAV_COUNT",
    "EXPECTED_NAV_LAST_WP_ORDINAL",
    "FORBIDDEN_VISION_PROFILE",
    "GCS_LAUNCH",
    "GCS_STOP",
    "NAVIGATION_SPEEDUP_OVERRIDE",
    "ROOT",
    "SCENARIO_MANIFEST_PATH",
    "SIM_SPEEDUP",
    "SRC",
    "FINAL_APPROACH_ROLL_LIMIT_DEG",
    "VISION_MOUNT_PITCH_DEG",
    "VISION_DETECT_HZ",
    "VISION_PROFILE",
]
