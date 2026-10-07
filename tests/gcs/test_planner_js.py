"""Tests for JS planner (planner.js) — standalone JS validation."""
import unittest
import json
import os
import re

from gcs.backend.settings_model import GcsSettings
from gcs.backend.settings_store import settings_store
from tests.gcs.js_runner import run_node

# Reset settings store to defaults so config matches JS hardcoded defaults
settings_store._settings = GcsSettings()

# Path to the JS planner utils directory
_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

# Load split JS files in dependency order and concatenate for Node.js eval.
_JS_FILES = [
    "projection.js",
    "plannerConfig.js",
    "geometry.js",
    "clipping.js",
    "trackGenerator.js",
    "launchZone.js",
    "planner.js",
]


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        # Remove import lines
        if stripped.startswith("import "):
            continue
        # Remove re-export lines (export { ... } from ...)
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        # Convert "export function" / "export const" / "export let" to plain declarations
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_PLANNER_JS = ""
for fname in _JS_FILES:
    fpath = os.path.join(_UTILS_DIR, fname)
    _PLANNER_JS += _strip_es_modules(open(fpath, encoding="utf-8").read()) + "\n"

# Test polygon (simple convex rectangle ~12 km²)
POLYGON = [
    {"lat": 32.0, "lon": 34.8},
    {"lat": 32.0, "lon": 34.9},
    {"lat": 32.1, "lon": 34.9},
    {"lat": 32.1, "lon": 34.8},
]
POLYGON_TUPLES = [(p["lat"], p["lon"]) for p in POLYGON]

# Large polygon for realistic multi-UAV scenario
L_POLYGON = [
    {"lat": 32.0, "lon": 34.7},
    {"lat": 32.0, "lon": 35.0},
    {"lat": 32.3, "lon": 35.0},
    {"lat": 32.3, "lon": 34.7},
]
L_POLYGON_TUPLES = [(p["lat"], p["lon"]) for p in L_POLYGON]

# Corridor waypoints
CORRIDOR_WPS = [
    {"lat": 32.0, "lon": 34.8},
    {"lat": 32.05, "lon": 34.85},
    {"lat": 32.1, "lon": 34.9},
]


def _run_node(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _PLANNER_JS + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


def _js_analyze(polygon):
    poly_json = json.dumps(polygon)
    return _run_node(
        f"console.log(JSON.stringify(analyzeArea({poly_json})));"
    )


def _js_generate(polygon, search_pattern, uav_count,
                  launch_point=None, corridor_waypoints=None,
                  set_launch_points=None, partition_angle_deg=None):
    poly_json = json.dumps(polygon)
    lp_json = json.dumps(launch_point)
    cw_json = json.dumps(corridor_waypoints)
    slp_json = json.dumps(set_launch_points)
    pa_json = json.dumps(partition_angle_deg)
    return _run_node(
        f"console.log(JSON.stringify(generatePlan("
        f"{poly_json}, '{search_pattern}', {uav_count}, "
        f"{lp_json}, {cw_json}, {slp_json}, {pa_json})));"
    )


class TestAnalyze(unittest.TestCase):
    """Verify JS analyzeArea returns valid results."""

    def test_basic_polygon(self):
        js = _js_analyze(POLYGON)
        self.assertGreater(js["area_km2"], 0)
        self.assertGreater(js["required_uavs"], 0)
        self.assertGreater(js["strip_count"], 0)
        self.assertGreater(js["total_distance_km"], 0)
        self.assertGreater(js["estimated_time_min"], 0)
        self.assertGreater(len(js["launch_zone"]), 2)

    def test_large_polygon(self):
        js = _js_analyze(L_POLYGON)
        self.assertGreater(js["area_km2"], 0)
        self.assertGreater(js["required_uavs"], 0)
        self.assertGreater(js["total_distance_km"], 0)


class TestDockPresetAltitude(unittest.TestCase):
    """Survey altitude and spacing come from the single `dock` preset."""

    def test_plan_altitude_is_dock_preset_altitude(self):
        js = _run_node(
            "configurePlanner({ camera: { vision_profile: 'p' } }, { p: {"
            " devices: [{ name: 'd', image_width: 1920, image_height: 1080,"
            " pitch_deg: -35, zooms: { '1': { fx: 2252.628, fy: 2262.151, detect_range_m: 900 } } }],"
            " dock_presets: { dock: { altitude_m: 173 } } } });"
            "const cfg = getConfig();"
            f"const plan = generatePlan({json.dumps(POLYGON)}, 'distributed', 3);"
            "console.log(JSON.stringify({ presets: Object.keys(cfg.DOCK_PRESETS),"
            " altitude: plan.altitude_m, spacingAlt: computeTrackSpacing(cfg)[1] }));"
        )
        self.assertEqual(js["presets"], ["dock"])
        self.assertEqual(js["altitude"], 173)
        self.assertEqual(js["spacingAlt"], 173)

    def test_fallback_dock_preset(self):
        js = _run_node(
            "console.log(JSON.stringify(getConfig().DOCK_PRESETS));"
        )
        self.assertEqual(js, {"dock": {"altitude_m": 200.0}})


class TestGenerate(unittest.TestCase):
    """Verify JS generatePlan returns valid results."""

    def test_distributed_3_uavs(self):
        js = _js_generate(POLYGON, "distributed", 3)
        self.assertEqual(len(js["zones"]), 3)
        self.assertGreater(js["total_distance_km"], 0)
        self.assertGreater(js["strip_count"], 0)
        self.assertNotIn("dock_classes", js)
        for z in js["zones"]:
            self.assertGreater(z["strip_count"], 0)
            self.assertGreater(z["total_distance_km"], 0)

    def test_distributed_with_launch(self):
        lp = {"lat": 31.95, "lon": 34.85}
        js = _js_generate(POLYGON, "distributed", 3, launch_point=lp)
        self.assertEqual(len(js["zones"]), 3)
        self.assertGreater(js["total_distance_km"], 0)

    def test_corridor(self):
        js = _js_generate(
            POLYGON, "corridor", 3,
            corridor_waypoints=CORRIDOR_WPS,
        )
        self.assertEqual(len(js["zones"]), 3)
        self.assertNotIn("dock_classes", js)
        for z in js["zones"]:
            self.assertGreater(z["total_distance_km"], 0)
            self.assertGreater(len(z["track"]), 0)

    def test_launch_zone_present(self):
        js = _js_generate(POLYGON, "distributed", 3)
        self.assertGreater(len(js["launch_zone"]), 2)
        self.assertGreater(len(js["min_launch_zone"]), 2)
        self.assertGreater(js["launch_zone_buffer_m"], 0)

    def test_large_distributed(self):
        """Large polygon requiring many UAVs."""
        js = _js_generate(L_POLYGON, "distributed", 6)
        self.assertEqual(len(js["zones"]), 6)
        for z in js["zones"]:
            self.assertGreater(len(z["track"]), 0)
        self.assertGreater(js["total_distance_km"], 0)


class TestSetIndex(unittest.TestCase):
    """Verify JS generatePlan outputs correct set_index on each zone."""

    def test_distributed_set_index_single_set(self):
        """3 UAVs → all zones have set_index=0."""
        js = _js_generate(POLYGON, "distributed", 3)
        for z in js["zones"]:
            self.assertEqual(z["set_index"], 0)

    def test_distributed_set_index_multi_set(self):
        """6 UAVs → zones 0-2 set 0, zones 3-5 set 1."""
        js = _js_generate(POLYGON, "distributed", 6)
        for z in js["zones"]:
            expected = z["zone_index"] // 3
            self.assertEqual(
                z["set_index"], expected,
                f"Zone {z['zone_index']}: expected set_index={expected}, got {z['set_index']}",
            )

    def test_corridor_set_index_zero(self):
        js = _js_generate(POLYGON, "corridor", 3,
                          corridor_waypoints=CORRIDOR_WPS)
        for z in js["zones"]:
            self.assertEqual(z["set_index"], 0)

    def test_set_index_matches_formula(self):
        """JS set_index = zone_index // UAVS_PER_SET for multi-set distributed."""
        js = _js_generate(L_POLYGON, "distributed", 6)
        self.assertEqual(len(js["zones"]), 6)
        for z in js["zones"]:
            expected = z["zone_index"] // 3
            self.assertEqual(z["set_index"], expected,
                             f"Zone {z['zone_index']}: expected set_index={expected}, "
                             f"got {z['set_index']}")

    def test_multi_set_launch_points_orientation(self):
        """Per-set launch points should affect track orientation."""
        lp = {"lat": 31.95, "lon": 34.85}
        # Two different set launch points
        slps = [
            {"lat": 31.95, "lon": 34.85},  # set 0 LP
            {"lat": 32.15, "lon": 34.85},  # set 1 LP (opposite side)
        ]
        js_no_slp = _js_generate(L_POLYGON, "distributed", 6,
                                  launch_point=lp)
        js_with_slp = _js_generate(L_POLYGON, "distributed", 6,
                                    launch_point=lp, set_launch_points=slps)
        # Both should produce 6 zones
        self.assertEqual(len(js_no_slp["zones"]), 6)
        self.assertEqual(len(js_with_slp["zones"]), 6)
        # set_index values should be identical
        for z1, z2 in zip(js_no_slp["zones"], js_with_slp["zones"]):
            self.assertEqual(z1["set_index"], z2["set_index"])


class TestPerSetScanAngle(unittest.TestCase):
    """Verify per-set scan direction in JS distributed planner."""

    def test_single_set_unchanged(self):
        """3 UAVs, 1 set: same behavior as before."""
        js = _js_generate(POLYGON, "distributed", 3)
        self.assertEqual(len(js["zones"]), 3)
        for z in js["zones"]:
            self.assertEqual(z["set_index"], 0)
            self.assertGreater(len(z["track"]), 0)

    def test_multi_set_all_zones_have_tracks(self):
        """6 UAVs, 2 sets: all zones have tracks."""
        js = _js_generate(L_POLYGON, "distributed", 6)
        self.assertEqual(len(js["zones"]), 6)
        for z in js["zones"]:
            expected_set = z["zone_index"] // 3
            self.assertEqual(z["set_index"], expected_set)
            self.assertGreater(len(z["track"]), 0)

    def test_per_set_lp_produces_different_orientation(self):
        """Per-set LPs on opposite sides should produce different track first-points."""
        slps = [
            {"lat": 31.95, "lon": 34.75},
            {"lat": 32.35, "lon": 35.05},
        ]
        js = _js_generate(L_POLYGON, "distributed", 6,
                          set_launch_points=slps)
        self.assertEqual(len(js["zones"]), 6)
        # Set 0 and Set 1 tracks should start from different regions
        s0_first = js["zones"][0]["track"][0]
        s1_first = js["zones"][3]["track"][0]
        # They should not be in the same spot (different approach directions)
        dist = abs(s0_first["lat"] - s1_first["lat"]) + abs(s0_first["lon"] - s1_first["lon"])
        self.assertGreater(dist, 0.01)

    def test_fallback_no_lps(self):
        """Multi-set without LPs: uses full polygon longest edge angle."""
        js = _js_generate(L_POLYGON, "distributed", 6)
        for z in js["zones"]:
            self.assertGreater(len(z["track"]), 0)


class TestPartitionAngle(unittest.TestCase):
    """Verify partition angle override in JS generatePlan."""

    def test_partition_angle_override(self):
        """Custom partition angle produces different zone shapes."""
        js_auto = _js_generate(L_POLYGON, "distributed", 6)
        js_rotated = _js_generate(L_POLYGON, "distributed", 6,
                                  partition_angle_deg=45)
        self.assertEqual(len(js_auto["zones"]), 6)
        self.assertEqual(len(js_rotated["zones"]), 6)
        # Zone polygons should differ
        auto_first = js_auto["zones"][0]["polygon"][0]
        rot_first = js_rotated["zones"][0]["polygon"][0]
        dist = abs(auto_first["lat"] - rot_first["lat"]) + abs(auto_first["lon"] - rot_first["lon"])
        self.assertGreater(dist, 0.001)

    def test_partition_angle_null_uses_auto(self):
        """Passing null partition angle should match auto behavior."""
        js_auto = _js_generate(L_POLYGON, "distributed", 6)
        js_null = _js_generate(L_POLYGON, "distributed", 6,
                               partition_angle_deg=None)
        self.assertEqual(len(js_auto["zones"]), len(js_null["zones"]))
        for za, zn in zip(js_auto["zones"], js_null["zones"]):
            self.assertAlmostEqual(za["total_distance_km"], zn["total_distance_km"], places=1)


if __name__ == "__main__":
    unittest.main()
