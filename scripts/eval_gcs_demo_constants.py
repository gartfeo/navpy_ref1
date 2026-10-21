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
#: The run's GCS log dir (GCS_LOG_DIR): backend.log and diagnostics sessions.
GCS_LOG_SUBDIR = "gcs"
SCENARIO_MANIFEST_PATH = ROOT / "scripts" / "eval_gcs_demo_scenario.json"

EXPECTED_NAV_LAST_WP_ORDINAL = 1
SIM_SPEEDUP = 10.0
NAVIGATION_SPEEDUP_OVERRIDE = 0.0
FINAL_APPROACH_ROLL_LIMIT_DEG = 45.0
#: A parameter request's HTTP timeout and the server budget it names
#: (``timeout_s``), kept 5 s below it (``docs/design/radio_budget.md``, todo
#: 97). In wall mode the server answers first; under lockstep the budget
#: counts sim seconds, so a stalled run can still hit the HTTP timeout.
PARAM_HTTP_TIMEOUT_S = 60.0
PARAM_REQUEST_BUDGET_S = PARAM_HTTP_TIMEOUT_S - 5.0
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


def backend_log_path(run_dir: Path) -> Path:
    """The run's complete backend log (no rotation, one backend per run)."""
    return run_dir / GCS_LOG_SUBDIR / "backend.log"


__all__ = [
    "GCS_LOG_SUBDIR",
    "backend_log_path",
    "EVALUATOR_LOCK_PATH",
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
    "PARAM_HTTP_TIMEOUT_S",
    "PARAM_REQUEST_BUDGET_S",
    "VISION_MOUNT_PITCH_DEG",
    "VISION_DETECT_HZ",
    "VISION_PROFILE",
]
