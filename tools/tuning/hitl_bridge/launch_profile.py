"""The aircraft's launch data for the bridge: its catapult rail and its rest pose on the skids.

These are the owner's measurements (the launcher's logs, the model's ground contacts), so they live in the
private folder (tools/tuning/private_data.py), in a JSON file the bridge reads with --launch-file:

    {"rail": {"accel_table_bin_s": s, "accel_table_g": [g, ...], "travel_m": m, "release_speed_m_s": m/s,
              "elevation_deg": deg, "carriage_angle_to_rail_deg": deg, "height_m": m},
     "ground_rest": {"height_agl_ft": ft, "pitch_deg": deg}}

Other keys (sources, the measured ranges the private-data tests check) are ignored here. --launch rail needs
"rail", --launch ground needs "ground_rest"; --launch launcher needs neither file.
"""
from __future__ import annotations

import json
import math
from dataclasses import fields
from pathlib import Path
from typing import Any, Mapping

from tools.tuning.hitl_bridge.plant_options import StartPose
from tools.tuning.hitl_bridge.rail import RailSpec

RAIL_KEY = "rail"
GROUND_REST_KEY = "ground_rest"
GROUND_REST_FIELDS = ("height_agl_ft", "pitch_deg")


def _number(section: str, name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"launch profile: {section}.{name} must be a finite number, not {value!r}")
    return float(value)


def _section(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    section = data.get(key)
    if not isinstance(section, Mapping):
        raise ValueError(f"launch profile: no {key!r} section")
    return section


def rail_spec(data: Mapping[str, Any]) -> RailSpec:
    """The "rail" section as a RailSpec; ValueError when a field is missing or not a finite number."""
    section = _section(data, RAIL_KEY)
    values = {}
    for f in fields(RailSpec):
        if f.name not in section:
            raise ValueError(f"launch profile: {RAIL_KEY}.{f.name} is missing")
        raw = section[f.name]
        if f.name == "accel_table_g":
            if not isinstance(raw, list) or not raw:
                raise ValueError(f"launch profile: {RAIL_KEY}.accel_table_g must be a non-empty list")
            values[f.name] = tuple(_number(RAIL_KEY, f.name, a) for a in raw)
        else:
            values[f.name] = _number(RAIL_KEY, f.name, raw)
    return RailSpec(**values)


def ground_rest(data: Mapping[str, Any]) -> StartPose:
    """The "ground_rest" section: the model's reference point height and pitch at rest on its skids."""
    section = _section(data, GROUND_REST_KEY)
    missing = [name for name in GROUND_REST_FIELDS if name not in section]
    if missing:
        raise ValueError(f"launch profile: {GROUND_REST_KEY}.{missing[0]} is missing")
    return StartPose(height_agl_ft=_number(GROUND_REST_KEY, "height_agl_ft", section["height_agl_ft"]),
                     pitch_deg=_number(GROUND_REST_KEY, "pitch_deg", section["pitch_deg"]), on_ground=True)


def read_launch_data(path: Path) -> Mapping[str, Any]:
    """The JSON object in path; FileNotFoundError names the private folder, ValueError a malformed file."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"no launch profile {path}: the aircraft's launch data is private "
                                f"(tools/tuning/private_data.py); pass --launch-file, or --launch launcher")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"launch profile {path}: {exc}") from None
    if not isinstance(data, Mapping):
        raise ValueError(f"launch profile {path}: not a JSON object")
    return data
