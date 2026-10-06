"""Tests for the fly-to-location camera path.

Two layers:

1. ``flyToOptions`` (src/gcs/frontend/src/utils/cameraFlyTo.js) is exercised with a
   mock Cesium via Node.js, asserting the lat/lon -> (lon, lat, height) axis swap,
   orientation, duration, and exact call counts. Reads the real source (strip
   ``export``) so the test guards the shipped helper, not a copy.

2. Wiring guard: static assertions that ``flyToRef`` is threaded App -> CesiumMap ->
   useCesiumViewer and that neither legacy global (``window.Cesium``,
   ``__CESIUM_VIEWER__``) survives anywhere in the frontend source. The helper test
   alone would pass even if the wiring were broken, which is exactly the failure
   mode that made this action silently dead.
"""
import json
import math
import os
import re
import unittest

from tests.gcs.js_runner import run_node


_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_FRONTEND_SRC = os.path.join(_ROOT, "src", "gcs", "frontend", "src")
_CAMERA_FLYTO = os.path.join(_FRONTEND_SRC, "utils", "cameraFlyTo.js")


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


# Real helper source, export-stripped for CommonJS-style Node eval.
_HELPER_JS = _read(_CAMERA_FLYTO).replace("export function ", "function ")

# Mock Cesium that records every call, then invoke the helper twice
# (default height, explicit height) and dump the results + call log.
_HARNESS_JS = _HELPER_JS + r"""
const calls = { fromDegrees: [], toRadians: [] };
const Cesium = {
  Cartesian3: {
    fromDegrees: (lon, lat, h) => {
      calls.fromDegrees.push([lon, lat, h]);
      return { kind: 'cart3', args: [lon, lat, h] };
    },
  },
  Math: {
    toRadians: (deg) => {
      calls.toRadians.push(deg);
      return deg * Math.PI / 180;
    },
  },
};
const optsDefault = flyToOptions(Cesium, 32.5, 34.8);
const optsHeight = flyToOptions(Cesium, 10, 20, 500);
console.log(JSON.stringify({ optsDefault, optsHeight, calls }));
"""


class TestFlyToOptions(unittest.TestCase):
    """flyToOptions builds correct camera.flyTo options from (lat, lon)."""

    @classmethod
    def setUpClass(cls):
        result = run_node(_HARNESS_JS, timeout=10)
        if result.returncode != 0:
            raise RuntimeError(f"Node.js error: {result.stderr}")
        cls.out = json.loads(result.stdout.strip())

    def test_axis_swap_and_height_default(self):
        """(lat, lon) at the boundary -> fromDegrees(lon, lat, 2000)."""
        self.assertEqual(self.out["optsDefault"]["destination"]["args"], [34.8, 32.5, 2000])

    def test_explicit_height_passes_through(self):
        self.assertEqual(self.out["optsHeight"]["destination"]["args"], [20, 10, 500])

    def test_orientation_pitch_is_minus_60(self):
        orient = self.out["optsDefault"]["orientation"]
        self.assertEqual(orient["heading"], 0)
        self.assertEqual(orient["roll"], 0)
        self.assertAlmostEqual(orient["pitch"], math.radians(-60), places=9)

    def test_duration(self):
        self.assertEqual(self.out["optsDefault"]["duration"], 1.0)

    def test_call_counts_and_args(self):
        """Exactly one fromDegrees + one toRadians per invocation, correct args."""
        calls = self.out["calls"]
        self.assertEqual(calls["fromDegrees"], [[34.8, 32.5, 2000], [20, 10, 500]])
        self.assertEqual(calls["toRadians"], [-60, -60])


class TestFlyToWiring(unittest.TestCase):
    """Guard the App -> CesiumMap -> useCesiumViewer ref threading and dead globals."""

    APP = _read(os.path.join(_FRONTEND_SRC, "App.jsx"))
    CESIUM_MAP = _read(os.path.join(_FRONTEND_SRC, "components", "map", "CesiumMap.jsx"))
    HOOK = _read(os.path.join(_FRONTEND_SRC, "components", "map", "hooks", "useCesiumViewer.js"))

    def test_app_creates_and_passes_ref(self):
        self.assertRegex(self.APP, r"flyToRef\s*=\s*useRef\(")
        self.assertRegex(self.APP, r"flyToRef\.current\?\.\(")
        self.assertIn("flyToRef={flyToRef}", self.APP)

    def test_cesium_map_forwards_ref_as_second_hook_arg(self):
        self.assertRegex(
            self.CESIUM_MAP,
            r"useCesiumViewer\(\s*resetViewRef\s*,\s*flyToRef\s*,\s*mapDefaults\s*\)",
        )

    def test_hook_assigns_ref_and_calls_flyto(self):
        self.assertRegex(self.HOOK, r"flyToRef\.current\s*=")
        self.assertRegex(self.HOOK, r"camera\.flyTo\(\s*flyToOptions\(")

    def test_no_legacy_globals_in_frontend_source(self):
        """window.Cesium was never set; __CESIUM_VIEWER__ was removed. Neither may return."""
        offenders = []
        for root, _dirs, files in os.walk(_FRONTEND_SRC):
            for name in files:
                if not name.endswith((".js", ".jsx")):
                    continue
                path = os.path.join(root, name)
                text = _read(path)
                if "window.Cesium" in text or "__CESIUM_VIEWER__" in text:
                    offenders.append(os.path.relpath(path, _ROOT))
        self.assertEqual(offenders, [], f"legacy Cesium globals resurfaced in: {offenders}")


if __name__ == "__main__":
    unittest.main()
