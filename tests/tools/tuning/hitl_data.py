"""Synthetic aircraft data for the tuning tests, and where the real (private) data is.

The aircraft's launch profile and HITL parameter files are private (tools/tuning/private_data.py). Every test
in this folder reads the synthetic set below instead: conftest.py points NAVPY_HITL_TOOLS_DIR at a folder
written by write_hitl_data. The real set is checked only by test_hitl_private_data.py, which skips where it is
absent. The numbers are neutral: a plausible launcher and aircraft, not any real one.
"""
from __future__ import annotations

import json
from pathlib import Path

from tools.tuning.hitl_bridge.launch_profile import GROUND_REST_KEY, RAIL_KEY, rail_spec
from tools.tuning.hitl_bridge.rail import RailSpec
from tools.tuning.private_data import (AIRCRAFT_PARAMS_NAME, HITL_DATA_DIR_NAME, LAUNCH_PARAMS_NAME,
                                       LAUNCH_PROFILE_NAME, private_dir)

# Read when this module is first imported (by conftest.py), before any test changes the environment.
REAL_PRIVATE_DIR = private_dir()

SYNTHETIC_RAIL = {
    "accel_table_bin_s": 0.010,
    "accel_table_g": [0.1, 3.0, 6.0, 7.0, 6.5, 6.0, 5.5, 5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0],
    "travel_m": 2.5,
    "release_speed_m_s": 11.0,
    "elevation_deg": 12.0,
    "carriage_angle_to_rail_deg": 2.0,
    "height_m": 0.8,
}
SYNTHETIC_GROUND_REST = {"height_agl_ft": 0.5, "pitch_deg": -2.0}
# Different from the stock board defaults the fake board starts with (test_board.BRIDGE_DEFAULTS), so a test
# sees that the files were loaded.
SYNTHETIC_AIRCRAFT_PARAMS = {"AIRSPEED_MIN": 18, "AIRSPEED_CRUISE": 23, "AIRSPEED_MAX": 31, "TKOFF_LVL_PITCH": 14,
                             "TKOFF_THR_MAX": 0, "TKOFF_THR_SLEW": 0}
SYNTHETIC_LAUNCH_PARAMS = {"TKOFF_THR_MINACC": 3.0, "TKOFF_THR_DELAY": 4}


def _parm(values: dict) -> str:
    return "# synthetic test data\n" + "".join(f"{name} {value}\n" for name, value in values.items())


def write_hitl_data(root: Path) -> Path:
    """The synthetic private folder under root (its hitl subfolder filled); returns root."""
    hitl = Path(root) / HITL_DATA_DIR_NAME
    hitl.mkdir(parents=True, exist_ok=True)
    (hitl / AIRCRAFT_PARAMS_NAME).write_text(_parm(SYNTHETIC_AIRCRAFT_PARAMS), encoding="utf-8")
    (hitl / LAUNCH_PARAMS_NAME).write_text(_parm(SYNTHETIC_LAUNCH_PARAMS), encoding="utf-8")
    (hitl / LAUNCH_PROFILE_NAME).write_text(json.dumps({RAIL_KEY: SYNTHETIC_RAIL,
                                                        GROUND_REST_KEY: SYNTHETIC_GROUND_REST}), encoding="utf-8")
    return Path(root)


def synthetic_rail() -> RailSpec:
    return rail_spec({RAIL_KEY: SYNTHETIC_RAIL})
