"""Tests for fallbackLocationAssignment.js — optimal DOCK-to-zone auto-assignment."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


# Path to the JS utils directory
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


_JS_FILE = os.path.join(_UTILS_DIR, "fallbackLocationAssignment.js")
_DOCK_JS = _strip_es_modules(open(_JS_FILE, encoding="utf-8").read())


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _DOCK_JS + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestAutoAssignDocks(unittest.TestCase):
    """autoAssignFallbackLocations optimal assignment."""

    def test_empty_zones(self):
        result = _run_js("""
        const r = autoAssignFallbackLocations([], [{name:'A', lat:40, lon:44}]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [])

    def test_empty_docks(self):
        result = _run_js("""
        const zones = [{track: [{lat:40, lon:44}]}];
        const r = autoAssignFallbackLocations(zones, []);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [None])

    def test_single_zone_single_dock(self):
        result = _run_js("""
        const zones = [{track: [{lat:40, lon:44}, {lat:40.1, lon:44}]}];
        const fallbackLocations = [{name:'HQ', lat:40.15, lon:44}];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [0])

    def test_two_zones_two_docks_nearest(self):
        """Each zone gets the nearest unassigned DOCK."""
        result = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}, {lat:40.05, lon:44}]},  // ends at 40.05
          {track: [{lat:40, lon:44}, {lat:40.5, lon:44}]},   // ends at 40.5
        ];
        const fallbackLocations = [
          {name:'Near', lat:40.06, lon:44},  // closest to zone 0
          {name:'Far', lat:40.49, lon:44},   // closest to zone 1
        ];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [0, 1])

    def test_more_zones_than_docks(self):
        """All zones get nearest DOCK even when shared (1 DOCK, 3 zones)."""
        result = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}]},
          {track: [{lat:40.1, lon:44}]},
          {track: [{lat:40.2, lon:44}]},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        # All 3 zones should get the only DOCK (index 0)
        self.assertEqual(result, [0, 0, 0])

    def test_prefer_unique_when_enough_docks(self):
        """With enough Docks, each zone gets a unique one even if not nearest."""
        result = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}]},
          {track: [{lat:40.001, lon:44}]},
        ];
        // Both Docks near zone 0, but each zone should get a different one
        const fallbackLocations = [
          {name:'A', lat:40, lon:44},
          {name:'B', lat:40.002, lon:44},
        ];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        # Each zone gets a unique DOCK
        self.assertEqual(len(set(result)), 2)
        self.assertEqual(len(result), 2)

    def test_zone_without_track(self):
        """Zone with empty track gets null assignment."""
        result = _run_js("""
        const zones = [
          {track: []},
          {track: [{lat:40, lon:44}]},
        ];
        const fallbackLocations = [{name:'A', lat:40, lon:44}];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        self.assertIsNone(result[0])
        self.assertEqual(result[1], 0)


    def test_no_crossing_lines(self):
        """Greedy-nearest would assign Z0→DOCK1, Z1→DOCK0 (crossing lines).
        Optimal DP should assign Z0→DOCK0, Z1→DOCK1, Z2→DOCK2 (no crossings).

        Layout (lat is x-axis, lon is y-axis):
          Z0=(32.0, 34.8)  Z1=(32.05, 34.8)  Z2=(32.1, 34.8)
          DOCK0=(32.04, 34.9) DOCK1=(32.06, 34.9) DOCK2=(32.08, 34.9)

        Greedy picks Z1→DOCK0 first (closest pair), which forces Z0 to reach
        across to DOCK1, creating a crossing. DP finds the global optimum.
        """
        result = _run_js("""
        const zones = [
          {track: [{lat:32.0,  lon:34.8}]},
          {track: [{lat:32.05, lon:34.8}]},
          {track: [{lat:32.1,  lon:34.8}]},
        ];
        const fallbackLocations = [
          {name:'A', lat:32.04, lon:34.9},
          {name:'B', lat:32.06, lon:34.9},
          {name:'C', lat:32.08, lon:34.9},
        ];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        # Optimal: Z0→DOCK0, Z1→DOCK1, Z2→DOCK2 — no crossing
        self.assertEqual(result, [0, 1, 2])

    def test_optimal_total_cost(self):
        """Verify DP picks the assignment with minimum total distance."""
        result = _run_js("""
        const zones = [
          {track: [{lat:32.0, lon:34.8}]},
          {track: [{lat:32.1, lon:34.8}]},
        ];
        const fallbackLocations = [
          {name:'A', lat:32.1, lon:34.9},
          {name:'B', lat:32.0, lon:34.9},
        ];
        // Z0 is near DOCK_B, Z1 is near DOCK_A → optimal: [1, 0]
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [1, 0])

    def test_more_docks_than_zones(self):
        """With surplus Docks, each zone picks the best from the full set."""
        result = _run_js("""
        const zones = [
          {track: [{lat:40, lon:44}]},
        ];
        const fallbackLocations = [
          {name:'Far',  lat:41, lon:44},
          {name:'Near', lat:40.001, lon:44},
          {name:'Mid',  lat:40.5, lon:44},
        ];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result, [1])  # nearest DOCK

    def test_sharing_with_unique_preference(self):
        """4 zones, 2 Docks: 2 zones get unique Docks, 2 share nearest."""
        result = _run_js("""
        const zones = [
          {track: [{lat:40.0, lon:44}]},
          {track: [{lat:40.1, lon:44}]},
          {track: [{lat:40.2, lon:44}]},
          {track: [{lat:40.3, lon:44}]},
        ];
        const fallbackLocations = [
          {name:'South', lat:40.0, lon:44.1},
          {name:'North', lat:40.3, lon:44.1},
        ];
        const r = autoAssignFallbackLocations(zones, fallbackLocations);
        console.log(JSON.stringify(r));
        """)
        # 2 unique assignments + 2 shared (each picks nearest)
        self.assertEqual(len(result), 4)
        self.assertIn(0, result)
        self.assertIn(1, result)
        # Z0 should get DOCK 0 (south), Z3 should get DOCK 1 (north)
        self.assertEqual(result[0], 0)
        self.assertEqual(result[3], 1)


class TestZoneToDockDistance(unittest.TestCase):
    """zoneToFallbackLocationDistance helper."""

    def test_basic_distance(self):
        result = _run_js("""
        const zone = {track: [{lat:40, lon:44}, {lat:40.001, lon:44}]};
        const dock = {lat:40.001, lon:44};
        const d = zoneToFallbackLocationDistance(zone, dock);
        console.log(JSON.stringify(d));
        """)
        # Same point — should be ~0
        self.assertAlmostEqual(result, 0, delta=5)

    def test_no_track_returns_infinity(self):
        result = _run_js("""
        const zone = {track: []};
        const dock = {lat:40, lon:44};
        const d = zoneToFallbackLocationDistance(zone, dock);
        console.log(JSON.stringify(d === Infinity));
        """)
        self.assertTrue(result)


class TestBuildDocksFromDownload(unittest.TestCase):
    """buildFallbackLocationsFromDownload — DOCK extraction from downloaded fallback locations."""

    def test_no_pois_all_null(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([null, null, null]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result["fallbackLocations"], [])
        self.assertEqual(result["assignments"], [None, None, None])

    def test_single_poi(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([{lat: 32.0, lon: 34.8}]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["fallbackLocations"]), 1)
        self.assertEqual(result["fallbackLocations"][0]["name"], "Fallback delivery location 1")
        self.assertEqual(result["fallbackLocations"][0]["type"], "other")
        self.assertAlmostEqual(result["fallbackLocations"][0]["lat"], 32.0)
        self.assertAlmostEqual(result["fallbackLocations"][0]["lon"], 34.8)
        self.assertEqual(result["assignments"], [0])

    def test_single_poi_with_downloaded_type(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([{lat: 32.0, lon: 34.8, type: 'bridge'}]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result["fallbackLocations"][0]["type"], "bridge")

    def test_dedup_shared_coords(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([
          {lat: 32.0, lon: 34.8},
          {lat: 32.0, lon: 34.8},
        ]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["fallbackLocations"]), 1)
        self.assertEqual(result["assignments"], [0, 0])

    def test_mixed_null_and_pois(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([
          null,
          {lat: 32.0, lon: 34.8},
          null,
          {lat: 33.0, lon: 35.0},
        ]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["fallbackLocations"]), 2)
        self.assertEqual(result["assignments"], [None, 0, None, 1])
        self.assertEqual(result["fallbackLocations"][0]["name"], "Fallback delivery location 1")
        self.assertEqual(result["fallbackLocations"][1]["name"], "Fallback delivery location 2")

    def test_empty_array(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result["fallbackLocations"], [])
        self.assertEqual(result["assignments"], [])

    def test_null_input(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload(null);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(result["fallbackLocations"], [])
        self.assertEqual(result["assignments"], [])

    def test_distinct_pois(self):
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([
          {lat: 32.0, lon: 34.8},
          {lat: 33.0, lon: 35.0},
          {lat: 34.0, lon: 36.0},
        ]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["fallbackLocations"]), 3)
        self.assertEqual(result["assignments"], [0, 1, 2])

    def test_near_but_distinct_coords_not_deduped(self):
        """Coords differing by more than 1e-7 should NOT be deduped."""
        result = _run_js("""
        const r = buildFallbackLocationsFromDownload([
          {lat: 32.0, lon: 34.8},
          {lat: 32.001, lon: 34.8},
        ]);
        console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["fallbackLocations"]), 2)
        self.assertEqual(result["assignments"], [0, 1])


if __name__ == "__main__":
    unittest.main()
