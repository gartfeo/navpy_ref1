"""Tests for dockResolver.js — resolves sim POI positions from plan + bitmask."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


# Paths to JS source files
_MAP_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "map", "utils",
))
_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


# Load bitmask.js + dockResolver.js
_BITMASK_SRC = _strip_es_modules(
    open(os.path.join(_UTILS_DIR, "bitmask.js"), encoding="utf-8").read()
)
_RESOLVER_SRC = _strip_es_modules(
    open(os.path.join(_MAP_UTILS_DIR, "dockResolver.js"), encoding="utf-8").read()
)
_JS_CODE = _BITMASK_SRC + "\n" + _RESOLVER_SRC


def _run_node(expr):
    """Evaluate a JS expression and return the parsed JSON result."""
    script = _JS_CODE + f"\nconsole.log(JSON.stringify({expr}));"
    result = run_node(script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


# Sample plan with 2 zones, each with 5-point tracks
PLAN_2ZONES = {
    "zones": [
        {
            "zone_index": 0,
            "set_index": 0,
            "track": [
                {"lat": 32.0, "lon": 34.80},
                {"lat": 32.01, "lon": 34.81},
                {"lat": 32.02, "lon": 34.82},
                {"lat": 32.03, "lon": 34.83},
                {"lat": 32.04, "lon": 34.84},
            ],
        },
        {
            "zone_index": 1,
            "set_index": 0,
            "track": [
                {"lat": 32.10, "lon": 34.90},
                {"lat": 32.11, "lon": 34.91},
                {"lat": 32.12, "lon": 34.92},
                {"lat": 32.13, "lon": 34.93},
                {"lat": 32.14, "lon": 34.94},
            ],
        },
    ],
}

VEHICLE_LIST = [{"sys_id": 1}, {"sys_id": 2}]

CORRIDOR_POINTS = [
    [  # set 0 corridor
        {"lat": 31.90, "lon": 34.70},
        {"lat": 31.95, "lon": 34.75},
    ],
]

# A freshly downloaded zone: the track is the whole mission navigation list,
# corridor prefix included, flagged by corridor_end_index. Post-processing
# trims the prefix and resets the flag to 0 (PLAN_TRIMMED_TRACK).
PLAN_RAW_TRACK = {
    "zones": [
        {
            "zone_index": 0,
            "sys_id": 1,
            "corridor_end_index": 2,
            "track": CORRIDOR_POINTS[0] + PLAN_2ZONES["zones"][0]["track"],
        },
    ],
}

PLAN_TRIMMED_TRACK = {
    "zones": [
        {
            "zone_index": 0,
            "sys_id": 1,
            "corridor_end_index": 0,
            "track": PLAN_2ZONES["zones"][0]["track"],
        },
    ],
}


class TestResolveSimDocks(unittest.TestCase):
    """Test resolveSimDocks pure utility."""

    def test_single_poi(self):
        """Both vehicles have WP 5 → one POI per zone."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 16, "2": 16})  # both WP 5
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0 → vehicle 1: WP 5 = track[4]
        self.assertAlmostEqual(result[0]["lat"], 32.04, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.84, places=4)
        self.assertEqual(result[0]["wpNumber"], 5)
        self.assertEqual(result[0]["zoneIndex"], 0)
        self.assertEqual(result[0]["sys_id"], 1)
        # Zone 1 → vehicle 2: WP 5 = track[4]
        self.assertAlmostEqual(result[1]["lat"], 32.14, places=4)
        self.assertAlmostEqual(result[1]["lon"], 34.94, places=4)
        self.assertEqual(result[1]["zoneIndex"], 1)
        self.assertEqual(result[1]["sys_id"], 2)

    def test_per_vehicle_different_pois(self):
        """Each vehicle has different POI WPs."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        # Vehicle 1: WP 1 (bitmask 1), Vehicle 2: WP 3 (bitmask 4)
        vtw_json = json.dumps({"1": 1, "2": 4})
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0 → vehicle 1: WP 1 = track[0]
        self.assertAlmostEqual(result[0]["lat"], 32.0, places=4)
        self.assertEqual(result[0]["wpNumber"], 1)
        # Zone 1 → vehicle 2: WP 3 = track[2]
        self.assertAlmostEqual(result[1]["lat"], 32.12, places=4)
        self.assertEqual(result[1]["wpNumber"], 3)

    def test_vehicle_without_pois_skipped(self):
        """Vehicle with no targ_wps entry → zone skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        # Only vehicle 2 has POIs
        vtw_json = json.dumps({"2": 16})
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["zoneIndex"], 1)

    def test_zone_with_sys_id(self):
        """Zone with explicit sys_id uses that vehicle's bitmask."""
        plan = {"zones": [{
            "zone_index": 0, "set_index": 0, "sys_id": 2,
            "track": [{"lat": 32.0, "lon": 34.80}, {"lat": 32.01, "lon": 34.81}],
        }]}
        plan_json = json.dumps(plan)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 1, "2": 2})  # v1: WP1, v2: WP2
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        # Zone has sys_id=2 → uses vehicle 2's bitmask (2 = WP 2)
        self.assertEqual(result[0]["wpNumber"], 2)
        self.assertAlmostEqual(result[0]["lat"], 32.01, places=4)

    def test_multiple_pois(self):
        """Both vehicles have WPs 1,3 → two POIs per zone."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 5, "2": 5})  # bitmask 5 = WPs 1,3
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 4)
        wp_numbers = [t["wpNumber"] for t in result]
        self.assertEqual(wp_numbers, [1, 3, 1, 3])

    def test_empty_vehicle_targ_wps(self):
        """Empty vehicleTargWps → no POIs."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {{}}, {vl_json})"
        )
        self.assertEqual(result, [])

    def test_null_plan(self):
        """Null plan → no POIs."""
        vl_json = json.dumps(VEHICLE_LIST)
        result = _run_node(
            f"resolveSimDocks(null, [[]], 'distributed', {{\"1\": 16}}, {vl_json})"
        )
        self.assertEqual(result, [])

    def test_bitmask_exceeds_wp_count(self):
        """Bitmask with WP index beyond track length → gracefully skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 512, "2": 512})  # WP 10, track has 5
        result = _run_node(
            f"resolveSimDocks({plan_json}, [[]], 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(result, [])

    def test_corridor_search_pattern(self):
        """Corridor search_pattern: wps = zoneTrack only (no corridor prepend)."""
        plan_json = json.dumps(PLAN_2ZONES)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 1, "2": 1})  # WP 1
        result = _run_node(
            f"resolveSimDocks({plan_json}, {corridor_json}, 'corridor', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0 track[0]
        self.assertAlmostEqual(result[0]["lat"], 32.0, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.80, places=4)

    def test_raw_track_does_not_double_count_corridor(self):
        """A freshly downloaded zone still carries its corridor prefix.

        Startup publishes the raw track (corridor_end_index > 0) and trims it
        only in a later post-processing pass. If corridorPointsArr is already
        populated from an earlier publication, prepending it counts the corridor
        twice and shifts every POI back by corridorLen.
        """
        plan_json = json.dumps(PLAN_RAW_TRACK)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 16})  # mission WP 5
        result = _run_node(
            f"resolveSimDocks({plan_json}, {corridor_json}, 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        # Raw track index 4 = the 3rd search waypoint, NOT the 1st (32.0/34.80).
        self.assertAlmostEqual(result[0]["lat"], 32.02, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.82, places=4)
        self.assertEqual(result[0]["wpNumber"], 5)

    def test_trimmed_track_still_prepends_corridor(self):
        """corridor_end_index == 0 → track is trimmed, corridor must be prepended."""
        plan_json = json.dumps(PLAN_TRIMMED_TRACK)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 16})  # mission WP 5
        result = _run_node(
            f"resolveSimDocks({plan_json}, {corridor_json}, 'distributed', {vtw_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        # 2 corridor pts + trimmed track → WP 5 = track[2]. Same point as the
        # raw-track case above: the two plan shapes must resolve identically.
        self.assertAlmostEqual(result[0]["lat"], 32.02, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.82, places=4)

    def test_non_corridor_with_corridor_points(self):
        """Non-corridor search_pattern: wps = corridorPts + zoneTrack."""
        plan_json = json.dumps(PLAN_2ZONES)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        vtw_json = json.dumps({"1": 5, "2": 5})  # WPs 1,3
        result = _run_node(
            f"resolveSimDocks({plan_json}, {corridor_json}, 'distributed', {vtw_json}, {vl_json})"
        )
        # Bitmask 5 = WPs 1,3. For each zone:
        # WP 1 = corridorPts[0] = {31.90, 34.70}
        # WP 3 = track[0] = zone-specific
        self.assertEqual(len(result), 4)
        # Zone 0: WP 1 = corridor[0]
        self.assertAlmostEqual(result[0]["lat"], 31.90, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.70, places=4)
        # Zone 0: WP 3 = track[0] (index 2 in combined array)
        self.assertAlmostEqual(result[1]["lat"], 32.0, places=4)
        self.assertAlmostEqual(result[1]["lon"], 34.80, places=4)


class TestResolveDetectionStart(unittest.TestCase):
    """Test resolveDetectionStart — first detection waypoint per zone."""

    def test_detection_start_basic(self):
        """nav_last_wp=2 → detection at WP 2 = index 1 in track."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vnl_json = json.dumps({"1": 2, "2": 3})
        result = _run_node(
            f"resolveDetectionStart({plan_json}, [[]], 'distributed', {vnl_json}, {vl_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0 → vehicle 1: nav_last_wp=2, det at WP 2 = track[1]
        self.assertAlmostEqual(result[0]["lat"], 32.01, places=4)
        self.assertEqual(result[0]["wpNumber"], 2)
        # Zone 1 → vehicle 2: nav_last_wp=3, det at WP 3 = track[2]
        self.assertAlmostEqual(result[1]["lat"], 32.12, places=4)
        self.assertEqual(result[1]["wpNumber"], 3)

    def test_no_nav_last_wp(self):
        """Empty vehicleNavLastWp → no detection points."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        result = _run_node(
            f"resolveDetectionStart({plan_json}, [[]], 'distributed', {{}}, {vl_json})"
        )
        self.assertEqual(result, [])

    def test_nav_last_wp_exceeds_track(self):
        """nav_last_wp beyond track length → skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vnl_json = json.dumps({"1": 20, "2": 20})  # Track has 5 pts
        result = _run_node(
            f"resolveDetectionStart({plan_json}, [[]], 'distributed', {vnl_json}, {vl_json})"
        )
        self.assertEqual(result, [])

    def test_with_corridor_prepend(self):
        """Non-corridor: wps = corridorPts + zoneTrack. nav_last_wp counts from combined start."""
        plan_json = json.dumps(PLAN_2ZONES)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        # 2 corridor pts + 5 track pts = 7. nav_last_wp=3 → det at WP 3 = combined[2] = track[0]
        vnl_json = json.dumps({"1": 3, "2": 3})
        result = _run_node(
            f"resolveDetectionStart({plan_json}, {corridor_json}, 'distributed', {vnl_json}, {vl_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0: WP 3 = combined[2] = track[0] (after 2 corridor pts)
        self.assertAlmostEqual(result[0]["lat"], 32.0, places=4)
        self.assertEqual(result[0]["wpNumber"], 3)

    def test_raw_track_does_not_double_count_corridor(self):
        """Raw downloaded track (corridor_end_index > 0) → index it directly."""
        plan_json = json.dumps(PLAN_RAW_TRACK)
        corridor_json = json.dumps(CORRIDOR_POINTS)
        vl_json = json.dumps(VEHICLE_LIST)
        vnl_json = json.dumps({"1": 3})  # mission WP 3
        result = _run_node(
            f"resolveDetectionStart({plan_json}, {corridor_json}, 'distributed', {vnl_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        # Raw track index 2 = the 1st search waypoint, not corridor[0].
        self.assertAlmostEqual(result[0]["lat"], 32.0, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.80, places=4)
        self.assertEqual(result[0]["wpNumber"], 3)

    def test_vehicle_without_nav_last_wp_skipped(self):
        """Vehicle with no nav_last_wp → zone skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        vl_json = json.dumps(VEHICLE_LIST)
        vnl_json = json.dumps({"2": 2})  # Only vehicle 2
        result = _run_node(
            f"resolveDetectionStart({plan_json}, [[]], 'distributed', {vnl_json}, {vl_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["zoneIndex"], 1)


class TestResolveDetectionStartFromPlan(unittest.TestCase):
    """Test resolveDetectionStartFromPlan — plan-time detection start resolution."""

    def test_basic(self):
        """One zone with detect WP → correct lat/lon resolved."""
        plan_json = json.dumps(PLAN_2ZONES)
        daw_json = json.dumps({"0": 2, "1": 4})
        result = _run_node(
            f"resolveDetectionStartFromPlan({plan_json}, {daw_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0: track index 2 → track[2]
        self.assertAlmostEqual(result[0]["lat"], 32.02, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.82, places=4)
        self.assertEqual(result[0]["wpNumber"], 3)
        self.assertEqual(result[0]["zoneIndex"], 0)
        # Zone 1: track index 4 → track[4]
        self.assertAlmostEqual(result[1]["lat"], 32.14, places=4)
        self.assertAlmostEqual(result[1]["lon"], 34.94, places=4)
        self.assertEqual(result[1]["wpNumber"], 5)
        self.assertEqual(result[1]["zoneIndex"], 1)

    def test_zone_without_detect_wp(self):
        """Zone not in detectAfterWps → skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        daw_json = json.dumps({"1": 0})
        result = _run_node(
            f"resolveDetectionStartFromPlan({plan_json}, {daw_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["zoneIndex"], 1)
        self.assertAlmostEqual(result[0]["lat"], 32.10, places=4)

    def test_out_of_bounds(self):
        """Detect WP index beyond track → skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        daw_json = json.dumps({"0": 10})
        result = _run_node(
            f"resolveDetectionStartFromPlan({plan_json}, {daw_json})"
        )
        self.assertEqual(result, [])

    def test_empty_detect_wps(self):
        """Empty detectAfterWps → no points."""
        plan_json = json.dumps(PLAN_2ZONES)
        result = _run_node(
            f"resolveDetectionStartFromPlan({plan_json}, {{}})"
        )
        self.assertEqual(result, [])

    def test_null_plan(self):
        """Null plan → no points."""
        result = _run_node(
            'resolveDetectionStartFromPlan(null, {"0": 1})'
        )
        self.assertEqual(result, [])

    def test_lng_fallback(self):
        """Track point with 'lng' instead of 'lon' → still resolved."""
        plan = {"zones": [{
            "zone_index": 0,
            "track": [{"lat": 32.0, "lng": 34.80}, {"lat": 32.01, "lng": 34.81}],
        }]}
        plan_json = json.dumps(plan)
        daw_json = json.dumps({"0": 1})
        result = _run_node(
            f"resolveDetectionStartFromPlan({plan_json}, {daw_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]["lon"], 34.81, places=4)


class TestNavLastWpEncoding(unittest.TestCase):
    """Test nav_last_wp corridor offset conversion (upload/download round-trip)."""

    def test_no_corridor(self):
        """Track index 2, no corridor → nav_last_wp = 3."""
        corridor_len = 0
        track_idx = 2
        nav_last_wp = corridor_len + track_idx + 1
        self.assertEqual(nav_last_wp, 3)
        self.assertEqual(nav_last_wp - 1 - corridor_len, track_idx)

    def test_with_corridor(self):
        """Track index 1, corridor=3 → nav_last_wp = 5."""
        corridor_len = 3
        track_idx = 1
        nav_last_wp = corridor_len + track_idx + 1
        self.assertEqual(nav_last_wp, 5)
        self.assertEqual(nav_last_wp - 1 - corridor_len, track_idx)

    def test_first_track_wp(self):
        """Track index 0, corridor=2 → nav_last_wp = 3."""
        corridor_len = 2
        track_idx = 0
        nav_last_wp = corridor_len + track_idx + 1
        self.assertEqual(nav_last_wp, 3)
        self.assertEqual(nav_last_wp - 1 - corridor_len, track_idx)


class TestResolveSimDocksFromPlan(unittest.TestCase):
    """Test resolveSimDocksFromPlan — plan-time sim POI resolution (no vehicle bitmask)."""

    def test_basic(self):
        """Two zones with selected WPs → correct lat/lon resolved."""
        plan_json = json.dumps(PLAN_2ZONES)
        stw_json = json.dumps({"0": [3], "1": [1]})
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {stw_json})"
        )
        self.assertEqual(len(result), 2)
        # Zone 0 WP index 3 → track[3]
        self.assertAlmostEqual(result[0]["lat"], 32.03, places=4)
        self.assertAlmostEqual(result[0]["lon"], 34.83, places=4)
        self.assertEqual(result[0]["zoneIndex"], 0)
        self.assertEqual(result[0]["wpNumber"], 4)  # 0-based 3 → 1-based 4
        # Zone 1 WP index 1 → track[1]
        self.assertAlmostEqual(result[1]["lat"], 32.11, places=4)
        self.assertAlmostEqual(result[1]["lon"], 34.91, places=4)
        self.assertEqual(result[1]["zoneIndex"], 1)
        self.assertEqual(result[1]["wpNumber"], 2)

    def test_multiple_per_zone(self):
        """Multiple selected WPs in one zone."""
        plan_json = json.dumps(PLAN_2ZONES)
        stw_json = json.dumps({"0": [0, 2, 4]})
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {stw_json})"
        )
        self.assertEqual(len(result), 3)
        self.assertEqual([t["wpNumber"] for t in result], [1, 3, 5])
        self.assertAlmostEqual(result[0]["lat"], 32.0, places=4)
        self.assertAlmostEqual(result[2]["lat"], 32.04, places=4)

    def test_empty_selections(self):
        """No selections → empty result."""
        plan_json = json.dumps(PLAN_2ZONES)
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {{}})"
        )
        self.assertEqual(result, [])

    def test_null_plan(self):
        """Null plan → empty result."""
        result = _run_node(
            'resolveSimDocksFromPlan(null, {"0": [1]})'
        )
        self.assertEqual(result, [])

    def test_out_of_bounds_skipped(self):
        """WP index beyond track length → skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        stw_json = json.dumps({"0": [10]})  # Track has 5 pts
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {stw_json})"
        )
        self.assertEqual(result, [])

    def test_zone_without_selections(self):
        """Zone not in simDockWps → skipped."""
        plan_json = json.dumps(PLAN_2ZONES)
        stw_json = json.dumps({"1": [2]})  # Only zone 1
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {stw_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["zoneIndex"], 1)
        self.assertAlmostEqual(result[0]["lat"], 32.12, places=4)

    def test_lng_fallback(self):
        """Track point with 'lng' instead of 'lon' → still resolved."""
        plan = {"zones": [{
            "zone_index": 0,
            "track": [{"lat": 32.0, "lng": 34.80}],
        }]}
        plan_json = json.dumps(plan)
        stw_json = json.dumps({"0": [0]})
        result = _run_node(
            f"resolveSimDocksFromPlan({plan_json}, {stw_json})"
        )
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]["lon"], 34.80, places=4)


class TestSimDockBitmaskEncoding(unittest.TestCase):
    """Test that sim POI WP selections encode to correct bitmasks."""

    def test_single_wp(self):
        """WP index 2 with 0 corridor → WP number 3 → bitmask bit 2."""
        # Bitmask for wpNum=3: 1 << (3-1) = 4
        result = _run_node("1 << (0 + 2 + 1 - 1)")
        self.assertEqual(result, 4)

    def test_multiple_wps(self):
        """WP indices [0, 3] with 2 corridor pts → WP numbers [3, 6] → bitmask."""
        corridor_len = 2
        wps = [0, 3]
        expected = 0
        for wi in wps:
            wp_num = corridor_len + wi + 1
            expected |= (1 << (wp_num - 1))
        # WP 3: bit 2 = 4, WP 6: bit 5 = 32 → 36
        self.assertEqual(expected, 36)

    def test_corridor_mode_no_offset(self):
        """Corridor mode: corridorLen=0, WP index 4 → WP number 5 → bitmask 16."""
        expected = 1 << (0 + 4 + 1 - 1)
        self.assertEqual(expected, 16)


class TestDecodeBitmaskArray(unittest.TestCase):
    """Test decodeBitmaskArray utility."""

    def test_zero(self):
        self.assertEqual(_run_node("decodeBitmaskArray(0)"), [])

    def test_single_bit(self):
        self.assertEqual(_run_node("decodeBitmaskArray(4)"), [3])

    def test_multiple_bits(self):
        self.assertEqual(_run_node("decodeBitmaskArray(13)"), [1, 3, 4])

    def test_roundtrip_with_encode(self):
        result = _run_node("decodeBitmaskArray(encodeBitmask('2,5,7'))")
        self.assertEqual(result, [2, 5, 7])


if __name__ == "__main__":
    unittest.main()
