"""Tests for hasGpsFix / collectFitPoints in useCesiumViewer.js.

These back the "center view" behavior: the framed rectangle must include the
plan geometry and only GPS-fixed UAVs, so a no-fix vehicle's (0, 0) / stale
fused position can't blow the view out to span the globe.
"""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_HOOKS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks",
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
        if re.match(r"^export\s+default\s+function\s", stripped):
            line = re.sub(r"^export\s+default\s+(function)\s", r"\1 ", line)
            out.append(line)
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_FILE = os.path.join(_HOOKS_DIR, "useCesiumViewer.js")
_SRC = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestHasGpsFix(unittest.TestCase):
    def test_null_vehicle(self):
        self.assertEqual(_run_js("console.log(hasGpsFix(null));"), "false")

    def test_null_fix(self):
        self.assertEqual(_run_js("console.log(hasGpsFix({gps_fix: null}));"), "false")

    def test_missing_fix(self):
        self.assertEqual(_run_js("console.log(hasGpsFix({}));"), "false")

    def test_no_fix_and_2d_excluded(self):
        # 0/1 = no fix, 2 = 2D — all below the 3D launch-gate threshold.
        for fix in (0, 1, 2):
            self.assertEqual(
                _run_js(f"console.log(hasGpsFix({{gps_fix: {fix}}}));"),
                "false", msg=f"gps_fix={fix} should be excluded",
            )

    def test_3d_and_better_included(self):
        # 3 = 3D, 4 = DGPS, 5/6 = RTK float/fixed.
        for fix in (3, 4, 5, 6):
            self.assertEqual(
                _run_js(f"console.log(hasGpsFix({{gps_fix: {fix}}}));"),
                "true", msg=f"gps_fix={fix} should be included",
            )


class TestCollectFitPoints(unittest.TestCase):
    def test_no_args_returns_empty(self):
        self.assertEqual(_run_js("console.log(collectFitPoints().length);"), "0")

    def test_empty_inputs_returns_empty(self):
        self.assertEqual(_run_js("console.log(collectFitPoints({}).length);"), "0")

    def test_fixed_vehicle_included(self):
        out = _run_js(
            "console.log(JSON.stringify(collectFitPoints("
            "{vehicleList: [{sys_id: 1, gps_fix: 3, lat: 10, lon: 20}]})));"
        )
        self.assertEqual(out, '[{"lat":10,"lon":20}]')

    def test_no_fix_vehicle_excluded(self):
        # A vehicle with a valid-looking position but no 3D fix must not count.
        out = _run_js(
            "console.log(collectFitPoints("
            "{vehicleList: [{sys_id: 1, gps_fix: 1, lat: 0, lon: 0}]}).length);"
        )
        self.assertEqual(out, "0")

    def test_fixed_vehicle_missing_position_excluded(self):
        out = _run_js(
            "console.log(collectFitPoints("
            "{vehicleList: [{sys_id: 1, gps_fix: 3, lat: null, lon: null}]}).length);"
        )
        self.assertEqual(out, "0")

    def test_mixed_vehicles_only_fixed_kept(self):
        out = _run_js(
            "console.log(JSON.stringify(collectFitPoints({vehicleList: ["
            "{sys_id: 1, gps_fix: 3, lat: 1, lon: 2},"
            "{sys_id: 2, gps_fix: null, lat: 3, lon: 4},"
            "{sys_id: 3, gps_fix: 6, lat: 5, lon: 6}"
            "]})));"
        )
        self.assertEqual(out, '[{"lat":1,"lon":2},{"lat":5,"lon":6}]')

    def test_polygon_lon_and_lng_fallback(self):
        # Polygon vertices may carry `lon` or the Leaflet-style `lng`.
        out = _run_js(
            "console.log(JSON.stringify(collectFitPoints("
            "{polygon: [{lat: 1, lon: 2}, {lat: 3, lng: 4}]})));"
        )
        self.assertEqual(out, '[{"lat":1,"lon":2},{"lat":3,"lon":4}]')

    def test_plan_tracks_launch_and_corridor(self):
        out = _run_js(
            "console.log(JSON.stringify(collectFitPoints({"
            "plan: {zones: [{track: [{lat: 1, lon: 1}, {lat: 2, lon: 2}]}]},"
            "launchPoint: {lat: 3, lng: 3},"
            "corridorPoints: [{lat: 4, lon: 4}]"
            "})));"
        )
        self.assertEqual(
            out,
            '[{"lat":1,"lon":1},{"lat":2,"lon":2},{"lat":3,"lon":3},{"lat":4,"lon":4}]',
        )


if __name__ == "__main__":
    unittest.main()
