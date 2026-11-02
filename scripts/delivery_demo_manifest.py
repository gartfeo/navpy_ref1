"""The hand demo's commit and input record, written into its run folder.

``write_input_manifest`` records whatever environment it is given, and a hand
demo inherits the owner's whole shell, so only the names below reach it: exact
names, no prefixes, so that a new secret variable cannot slip in. The record is
evidence, not a launch step: a failure warns and the launch goes on.
"""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping
from pathlib import Path

from scripts.delivery_demo_plan import PLAN_PATH
from scripts.eval_gcs_demo_inputs import write_input_manifest
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_sitl_root import sha256_file

#: What a run's behaviour or evidence depends on. ``NAVPY_SIM_VIDEO_URI`` stays
#: out: a video URI can carry a password. No value here should ever hold a secret.
MANIFEST_ENV = frozenset({
    # This run's own paths and names (``delivery_demo.RUN_ENV``).
    "GCS_SETTINGS_PATH", "NAVPY_LOG_DIR", "GCS_LOG_DIR", "NAVPY_POSE_CADENCE_DEBUG",
    "NAVPY_POSE_CADENCE_NAMING", "GCS_DIAGNOSTICS_SESSION_ID",
    # Switches a shell can leave set.
    "GCS_SIM_NAVIGATION_SPEEDUP", "NAVPY_LOG_ROTATION", "NAVPY_SIM_LOG_LEVEL",
    "NAVPY_SIM_CV_THREADS", "NAVPY_SIM_DOCKS", "NAVPY_SIM_DOCK_NET_RADIUS_M",
    "NAVPY_SIM_FORCE_DETECTION_GAP", "NAVPY_DETERMINISM_TRACE", "NAVPY_DETERMINISM_TRACE_ROWS",
    "AAS_LAT_GYRO_TERM", "AAS_LAT_GYRO_DELAY_S", "AAS_TRUTH_POSE_TIME_AXIS",
    "PYTHONHASHSEED", "PYTHONUTF8", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
})


def manifest_environment(env: Mapping[str, str]) -> dict[str, str]:
    return {key: value for key, value in env.items() if key in MANIFEST_ENV}


def _warn_on_stderr(text: str) -> None:
    print(text, file=sys.stderr)


def record_run_inputs(
    run_dir: Path, env: Mapping[str, str], sitl: Mapping[str, JsonValue], sitl_root_host: str,
    scenario: DemoScenario, cpu_share_min: float,
    warn: Callable[[str], None] = _warn_on_stderr,
) -> None:
    """Write the run's manifest. ``offered_plan`` is the file the Ready text names;
    the operator may load another, and the run keeps no copy of what was loaded."""
    try:
        options: dict[str, JsonValue] = {
            "hand_demo": True,
            "cpu_share_min": cpu_share_min,
            "offered_plan_sha256": sha256_file(PLAN_PATH),
        }
        write_input_manifest(run_dir, manifest_environment(env), sitl, sitl_root_host, options, scenario)
    except Exception as error:  # noqa: BLE001 - evidence only; it must not fail the launch
        warn(f"WARNING: this run has no complete commit and input record: {error}. The launch goes on.")


__all__ = ["MANIFEST_ENV", "manifest_environment", "record_run_inputs"]
