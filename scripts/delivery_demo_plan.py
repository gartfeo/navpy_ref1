"""The hand delivery demo's plan as a GCS plan file the operator loads in the UI.

Built from the delivery fixture's inputs: the same area height, launch point,
pattern, UAV count, partition angle and hubs. The UI marks sim docks only on
track waypoints, so the area is narrowed and the lines are 150 m apart: two
south-end turn waypoints of the lowest zone become the docks, as close as the
fixture's d1 and d2. See docs/design/delivery_demo_prep.md, "Normal-run demo".

A second plan file keeps all of that but flies the area the owner drew on the
map on 2026-11-03: a 1.26 x 0.77 km box along the A327 road, south-west of the
launch point. The planner aims its lines at the launch point, so they cross the
box diagonally; the docks fall on its west edge.

    python scripts/delivery_demo_plan.py           # rewrite both plan files
    python scripts/delivery_demo_plan.py --check   # fail if either is stale
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping
from pathlib import Path

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for _import_root in (_REPOSITORY_ROOT, _REPOSITORY_ROOT / "src"):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))

from scripts.eval_delivery_fixture import FIXTURE_PATH, load_fixture  # noqa: E402
from scripts.eval_gcs_demo_ports import JsonValue  # noqa: E402

PLAN_PATH = Path(__file__).resolve().parent / "delivery_demo_plan.json"
DRAWN_PLAN_PATH = PLAN_PATH.with_name("delivery_demo_plan_drawn.json")
#: The owner's drawn area, corners read off the map drawing (about 4 m a pixel).
DRAWN_AREA = (
    {"lat": 40.310307, "lon": 44.45409}, {"lat": 40.303663, "lon": 44.456537},
    {"lat": 40.300652, "lon": 44.442203}, {"lat": 40.307263, "lon": 44.4398},
)
#: Line spacing: the distance between the two dock waypoints, the fixture's
#: d1-d2 distance. Both stay in the ZR10's zoom-1 frame on the way to the first.
LINE_SPACING_M = 150.0
#: Area width: three lines per UAV, as in the fixture.
AREA_WIDTH_M = 1350.0
#: The zone that carries the docks: the lowest. With no camera profile in the
#: settings the planner flies zones 0-2 at 240/220/200 m; with the ZR10 profile
#: at 403/383/363 m, where only 363 m (with the fixed -14 deg search pitch)
#: leaves a window, about 682-730 m out, with both docks in frame and in range.
OFFERER_ZONE = 2
#: Its track waypoints (0-based) that carry the docks: the south-end turn of
#: its first two lines.
DOCK_TRACK_WAYPOINTS = (1, 2)
_METRES_PER_DEGREE = 111_320.0


def plan_file(fixture: Mapping[str, JsonValue]) -> dict[str, JsonValue]:
    inputs = fixture["inputs"]
    hubs = inputs["delivery_hubs"]
    # nav_last_wp is a 1-based NAV ordinal; the plan has no corridor.
    detect_after = fixture["runtime"]["nav_last_wp"] - 1
    return {
        "polygon": _narrowed_area(inputs["polygon"], inputs["launch_point"]["lon"]),
        "search_pattern": inputs["search_pattern"],
        "uav_count": inputs["uav_count"],
        "partition_angle_deg": inputs["partition_angle_deg"],
        "route_offset_m": LINE_SPACING_M,
        "set_launch_points": [inputs["launch_point"]],
        "delivery_hub_assignments": [
            _hub_index(hubs, mission["default_delivery_hub"]) for mission in fixture["missions"]
        ],
        "sim_dock_wps": {str(OFFERER_ZONE): list(DOCK_TRACK_WAYPOINTS)},
        "detect_after_wps": {str(zone): detect_after for zone in range(inputs["uav_count"])},
        "default_delivery_hubs": hubs,
    }


def _narrowed_area(polygon: list, centre_lon: float) -> list[dict[str, float]]:
    """The fixture's area rectangle, AREA_WIDTH_M wide around *centre_lon*."""
    north = max(p["lat"] for p in polygon)
    south = min(p["lat"] for p in polygon)
    half = AREA_WIDTH_M / 2 / (_METRES_PER_DEGREE * math.cos(math.radians((north + south) / 2)))
    west, east = round(centre_lon - half, 7), round(centre_lon + half, 7)
    return [
        {"lat": north, "lon": west}, {"lat": north, "lon": east},
        {"lat": south, "lon": east}, {"lat": south, "lon": west},
    ]


def _hub_index(hubs: list, hub: Mapping[str, JsonValue]) -> int:
    matches = [i for i, h in enumerate(hubs) if (h["lat"], h["lon"]) == (hub["lat"], hub["lon"])]
    if len(matches) != 1:
        raise ValueError(f"mission hub {hub} matches {len(matches)} fixture hubs")
    return matches[0]


def drawn_plan_file(fixture: Mapping[str, JsonValue]) -> dict[str, JsonValue]:
    """plan_file over DRAWN_AREA; everything else, hubs included, is the same."""
    return {**plan_file(fixture), "polygon": [dict(corner) for corner in DRAWN_AREA]}


def render(fixture: Mapping[str, JsonValue]) -> str:
    return json.dumps(plan_file(fixture), indent=2) + "\n"


def render_drawn(fixture: Mapping[str, JsonValue]) -> str:
    return json.dumps(drawn_plan_file(fixture), indent=2) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if a plan file is stale")
    args = parser.parse_args(argv)
    fixture = load_fixture(FIXTURE_PATH)
    outputs = {PLAN_PATH: render(fixture), DRAWN_PLAN_PATH: render_drawn(fixture)}
    if args.check:
        stale = [path for path, text in outputs.items()
                 if not path.exists() or path.read_text(encoding="utf-8") != text]
        for path in stale:
            print(f"{path} is stale; run: python scripts/delivery_demo_plan.py", file=sys.stderr)
        return 1 if stale else 0
    for path, text in outputs.items():
        path.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
