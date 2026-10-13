"""Generate the delivery 3-UAV eval fixture with the GCS planner's own code.

The owner's delivery scenario (docs/design/delivery_eval_scenario.md): one
drawn area split into three search routes by the GCS planner, two delivery
docks on one route, and one home dock per route. The planner exists only in
JavaScript, so this script runs it through Node and writes
``scripts/eval_delivery_scenario.json``. The eval reads only that JSON.

    python scripts/eval_delivery_scenario_gen.py           # rewrite the fixture
    python scripts/eval_delivery_scenario_gen.py --check   # fail if it is stale
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
UTILS_DIR = ROOT / "src" / "gcs" / "frontend" / "src" / "utils"
FIXTURE_PATH = ROOT / "scripts" / "eval_delivery_scenario.json"

# Planner modules in dependency order, as tests/gcs/test_planner_js.py loads them.
_JS_FILES = (
    "projection.js",
    "plannerConfig.js",
    "geometry.js",
    "clipping.js",
    "trackGenerator.js",
    "launchZone.js",
    "planner.js",
)
# The hub assignment the GCS runs before upload. It gets its own scope because
# it declares helper names the planner modules also declare.
_HUB_JS_FILE = "deliveryHubAssignment.js"

# Mission items carry lat/lon as int(degrees * 1e7); todo 26 truncates.
_COORDINATE_SCALE = 10_000_000
_EARTH_RADIUS_M = 6_371_000.0
# How far normalisation may move a planner point: the nearest grid value can
# fail the float round trip, and then a neighbour (1.5 quanta away) is used.
ENCODING_TOLERANCE_DEG = 1.5 / _COORDINATE_SCALE

# Fixed inputs. The area lies just south of the SITL home, where the ground is
# within about 40 m of home height. Its centre is due south of the launch
# point: the GCS passes the launch point as the set's approach point
# (usePlanGeneration.js:57-62), and the planner aims the lines at it
# (planner.js:176-182), so they run north-south and the three routes are equal.
# Routes are 500 m apart, so a dock on the middle line of one route is 1 km
# from every other route: beyond the ZR10's 829 m detection limit for the
# 4.3 m dock at search zoom 1 (vision_class_profile.py).
INPUTS: dict[str, Any] = {
    "home": {"lat": 40.3117414, "lon": 44.4552111, "alt_m": 1294.86},
    "launch_point": {"lat": 40.3118106, "lon": 44.45604},
    "polygon": [
        {"lat": 40.302, "lon": 44.42954},
        {"lat": 40.302, "lon": 44.48254},
        {"lat": 40.275, "lon": 44.48254},
        {"lat": 40.275, "lon": 44.42954},
    ],
    "search_pattern": "distributed",
    "uav_count": 3,
    "partition_angle_deg": 90.0,
    "track_spacing_override_m": 500.0,
    # Above home. The planner's 363 m ZR10 preset cannot see a dock (Q2).
    "dock_preset_altitude_m": 220.0,
    "altitude_separation_m": 20.0,
    "delivery_hubs": [
        {"name": "home-west", "type": "building", "lat": 40.3165, "lon": 44.4384},
        {"name": "home-middle", "type": "building", "lat": 40.3165, "lon": 44.4560},
        {"name": "home-east", "type": "building", "lat": 40.3165, "lon": 44.4737},
    ],
    # d1 sits on the middle line of the offerer's route; d2 follows it 150 m
    # along the same line. Assumed (not proven here; the e2b SITL slice
    # checks it): the view band at about 260 m is some 340 m deep, so both
    # are in view together and the offerer offers d2 at once.
    "dock_placement": {
        "offerer_zone_index": 0,
        "line_index": 1,
        "d1_fraction": 0.45,
        "d2_after_m": 150.0,
    },
}

# Runtime settings the oracle assumes; the eval must apply them (e4).
RUNTIME = {
    # Detection starts after the first track row, so launch and transit
    # (which pass the middle home dock) cannot select or offer a dock.
    "nav_last_wp": 1,
    # One approach per flight: after it the UAV does no more delivery work.
    "nav_oneshot": True,
    # Search runs at zoom 1; DETECT_LIMIT_M holds only there.
    "search_zoom": 1,
}

# 8 px dock diagonal at ZR10 zoom-1 fy, times the 1.1 detection margin.
DETECT_LIMIT_M = 829.0
CLEARANCE_MARGIN_M = 150.0


def _strip_es_modules(src: str) -> str:
    out = []
    for line in src.split("\n"):
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        out.append(re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line))
    return "\n".join(out)


def _source(name: str) -> str:
    return _strip_es_modules((UTILS_DIR / name).read_text(encoding="utf-8"))


def _planner_bundle() -> str:
    planner = "\n".join(_source(name) for name in _JS_FILES)
    hubs = (
        "const autoAssignDeliveryHubs = (() => {\n"
        f"{_source(_HUB_JS_FILE)}\n"
        "return autoAssignDeliveryHubs;\n})();"
    )
    return f"{planner}\n{hubs}"


def _run_planner(inputs: dict[str, Any]) -> dict[str, Any]:
    call = {
        "polygon": inputs["polygon"],
        "pattern": inputs["search_pattern"],
        "uavCount": inputs["uav_count"],
        "launchPoint": inputs["launch_point"],
        "approachPoints": [inputs["launch_point"]],
        "angle": inputs["partition_angle_deg"],
        "spacing": inputs["track_spacing_override_m"],
        "settings": {"flight": {"altitude_separation_m": inputs["altitude_separation_m"]},
                     "camera": {"vision_profile": "eval"}},
        "catalog": {"eval": {"devices": [], "dock_presets": {
            "dock": {"altitude_m": inputs["dock_preset_altitude_m"]}}}},
        "hubs": inputs["delivery_hubs"],
    }
    code = _planner_bundle() + f"""
const c = {json.dumps(call)};
configurePlanner(c.settings, c.catalog, null);
setTrackSpacingOverride(c.spacing);
const plan = generatePlan(c.polygon, c.pattern, c.uavCount, c.launchPoint, null, c.approachPoints, c.angle);
const hubs = autoAssignDeliveryHubs(plan.zones, c.hubs);
process.stdout.write(JSON.stringify({{plan, hubs}}));
"""
    result = subprocess.run(
        ["node"], input=code, capture_output=True, text=True, encoding="utf-8", timeout=60,
    )
    if result.returncode != 0:
        raise RuntimeError(f"planner failed: {result.stderr.strip()}")
    return json.loads(result.stdout)


def encodable(value: float) -> float:
    """The nearest n / 1e7 that survives the builder's int(v * 1e7) truncation.

    The fixture is the planner's output normalised to this grid: each point
    moves by at most ENCODING_TOLERANCE_DEG (about 1.7 cm). The live GCS would
    truncate the raw float instead, which can land one quantum lower.
    """
    target = round(value * _COORDINATE_SCALE)
    for item in (target, target + 1, target - 1):
        candidate = item / _COORDINATE_SCALE
        if int(candidate * _COORDINATE_SCALE) / _COORDINATE_SCALE == candidate:
            return candidate
    raise ValueError(f"cannot encode {value!r} exactly")


def _point(lat: float, lon: float) -> dict[str, float]:
    return {"lat": encodable(lat), "lon": encodable(lon)}


def ground_distance_m(a: dict[str, float], b: dict[str, float]) -> float:
    lat1, lat2 = math.radians(a["lat"]), math.radians(b["lat"])
    dlat = lat2 - lat1
    dlon = math.radians(b["lon"] - a["lon"])
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(h))


def _route_lines(track: list[dict[str, float]]) -> list[tuple[dict, dict]]:
    """The planner's zigzag: each search line is a pair of consecutive points."""
    return [(track[i], track[i + 1]) for i in range(0, len(track) - 1, 2)]


def _place_delivery_docks(zone: dict[str, Any], placement: dict[str, Any]) -> list[dict[str, Any]]:
    """Interpolate along the line in the direction it is flown."""
    start, end = _route_lines(zone["track"])[placement["line_index"]]
    length = ground_distance_m(start, end)
    docks = []
    for name, along in (
        ("d1", placement["d1_fraction"] * length),
        ("d2", placement["d1_fraction"] * length + placement["d2_after_m"]),
    ):
        t = along / length
        lat = start["lat"] + (end["lat"] - start["lat"]) * t
        lon = start["lon"] + (end["lon"] - start["lon"]) * t
        docks.append({"name": name, **_point(lat, lon), "height_m": 0.0, "location_type": None})
    return docks


def _mission(zone: dict[str, Any], hub: dict[str, Any], inputs: dict[str, Any]) -> dict[str, Any]:
    """The upload body the GCS builds per zone (useMissionUpload.js:101-140)."""
    return {
        "waypoints": [_point(p["lat"], p["lon"]) for p in zone["track"]],
        "altitude_m": zone["altitude_m"],
        "corridor_count": 0,
        "corridor_altitude_m": None,
        "search_pattern": inputs["search_pattern"],
        "polygon": [dict(p) for p in inputs["polygon"]],
        "corridor_backbone": [],
        "launch_point": dict(inputs["launch_point"]),
        "default_delivery_hub": {"lat": hub["lat"], "lon": hub["lon"], "type": hub["type"]},
    }


def build_fixture(inputs: dict[str, Any] = INPUTS) -> dict[str, Any]:
    result = _run_planner(inputs)
    # Planner order: the GCS pairs zones with vehicles and hubs by position.
    zones = result["plan"]["zones"]
    if [zone["zone_index"] for zone in zones] != list(range(len(zones))):
        raise ValueError("planner zones are not in zone_index order")
    if len(zones) != inputs["uav_count"]:
        raise ValueError(f"planner made {len(zones)} zones, expected {inputs['uav_count']}")
    hub_indices = result["hubs"]
    if sorted(hub_indices) != list(range(len(zones))):
        raise ValueError(f"each zone needs its own home dock, got {hub_indices}")
    hubs = [inputs["delivery_hubs"][index] for index in hub_indices]
    placement = inputs["dock_placement"]
    delivery = _place_delivery_docks(zones[placement["offerer_zone_index"]], placement)
    home_docks = [
        {"name": hub["name"], **_point(hub["lat"], hub["lon"]), "height_m": 0.0,
         "location_type": hub["type"]}
        for hub in hubs
    ]
    return {
        "schema_version": 1,
        "generator": "scripts/eval_delivery_scenario_gen.py",
        "inputs": inputs,
        "runtime": RUNTIME,
        "plan": {
            "grid_angle_deg": result["plan"]["grid_angle_deg"],
            "base_altitude_m": result["plan"]["altitude_m"],
            "zones": [
                {"zone_index": z["zone_index"], "altitude_m": z["altitude_m"],
                 "strip_count": z["strip_count"], "total_distance_km": z["total_distance_km"]}
                for z in zones
            ],
        },
        "missions": [_mission(zone, hub, inputs) for zone, hub in zip(zones, hubs)],
        "docks": {"schema_version": 1, "docks": delivery + home_docks},
        "expected": {
            "offerer_zone_index": placement["offerer_zone_index"],
            "kept_dock": "d1",
            "offered_dock": "d2",
            "home_dock_by_zone": [hub["name"] for hub in hubs],
        },
    }


def render(fixture: dict[str, Any]) -> str:
    return json.dumps(fixture, indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if the fixture is stale")
    args = parser.parse_args(argv)
    text = render(build_fixture())
    if args.check:
        current = FIXTURE_PATH.read_text(encoding="utf-8") if FIXTURE_PATH.exists() else ""
        if current != text:
            print(f"{FIXTURE_PATH} is stale; rerun without --check", file=sys.stderr)
            return 1
        return 0
    FIXTURE_PATH.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
