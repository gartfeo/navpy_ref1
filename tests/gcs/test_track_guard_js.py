"""Tests that JS track generation guards prevent infinite loops.

Exercises computeTrackSpacing, generateZigzagTrack, and generateStrips
with camera configs that would previously cause infinite loops (e.g.
positive pitch making bz_near negative, or 100% overlap).
"""
import json
import os
import re
import unittest

from tests.gcs.js_runner import run_node

# Path to the JS planner utils directory
_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

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


_PLANNER_JS = ""
for fname in _JS_FILES:
    fpath = os.path.join(_UTILS_DIR, fname)
    _PLANNER_JS += _strip_es_modules(open(fpath, encoding="utf-8").read()) + "\n"


def _run_node(script):
    full = _PLANNER_JS + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


# Config with positive pitch (+35 deg) — camera above horizon, bz_near < 0
_BAD_PITCH_CFG = """{
  DOCK_PRESETS: { small: { altitude_m: 150 } },
  FOV_HORIZONTAL_DEG: 46.2,
  FOV_VERTICAL_DEG: 26.8,
  CAMERA_PITCH_DEG: 35.0,
  OVERLAP_FRACTION: 0.1,
  DEG2RAD: Math.PI / 180
}"""

# Config with full overlap (1.0) — spacing becomes zero
_FULL_OVERLAP_CFG = """{
  DOCK_PRESETS: { small: { altitude_m: 150 } },
  FOV_HORIZONTAL_DEG: 46.2,
  FOV_VERTICAL_DEG: 26.8,
  CAMERA_PITCH_DEG: -35.0,
  OVERLAP_FRACTION: 1.0,
  DEG2RAD: Math.PI / 180
}"""

# Normal config
_NORMAL_CFG = """{
  DOCK_PRESETS: { small: { altitude_m: 150 } },
  FOV_HORIZONTAL_DEG: 46.2,
  FOV_VERTICAL_DEG: 26.8,
  CAMERA_PITCH_DEG: -35.0,
  OVERLAP_FRACTION: 0.1,
  DEG2RAD: Math.PI / 180
}"""


class TestComputeTrackSpacingGuardsJS(unittest.TestCase):
    """JS computeTrackSpacing guards prevent negative/zero spacing."""

    def test_positive_pitch_returns_zero(self):
        result = _run_node(
            f"console.log(JSON.stringify(computeTrackSpacing(['small'], {_BAD_PITCH_CFG})));"
        )
        spacing, alt, half_swath = result
        self.assertEqual(spacing, 0)
        self.assertEqual(half_swath, 0)
        self.assertEqual(alt, 150)

    def test_full_overlap_returns_zero(self):
        result = _run_node(
            f"console.log(JSON.stringify(computeTrackSpacing(['small'], {_FULL_OVERLAP_CFG})));"
        )
        spacing, alt, half_swath = result
        self.assertEqual(spacing, 0)

    def test_normal_config_positive_spacing(self):
        result = _run_node(
            f"console.log(JSON.stringify(computeTrackSpacing(['small'], {_NORMAL_CFG})));"
        )
        spacing, alt, half_swath = result
        self.assertGreater(spacing, 0)
        self.assertGreater(half_swath, 0)

    def test_spacing_override_replaces_computed_spacing(self):
        # Demo-mode manual route offset: TRACK_SPACING_OVERRIDE_M replaces the
        # camera-derived spacing but leaves altitude/halfSwath untouched.
        result = _run_node(
            "const cfg = Object.assign(" + _NORMAL_CFG + ", { TRACK_SPACING_OVERRIDE_M: 123 });"
            "console.log(JSON.stringify(computeTrackSpacing(['small'], cfg)));"
        )
        spacing, alt, half_swath = result
        self.assertEqual(spacing, 123)
        self.assertEqual(alt, 150)
        self.assertGreater(half_swath, 0)

    def test_null_override_keeps_computed_spacing(self):
        result = _run_node(
            "const cfg = Object.assign(" + _NORMAL_CFG + ", { TRACK_SPACING_OVERRIDE_M: null });"
            "console.log(JSON.stringify(computeTrackSpacing(['small'], cfg)));"
        )
        spacing, alt, half_swath = result
        self.assertGreater(spacing, 0)
        self.assertNotEqual(spacing, 0)
        self.assertAlmostEqual(spacing, 2 * half_swath * 0.9, delta=1e-6)


class TestGenerateZigzagTrackGuardsJS(unittest.TestCase):
    """JS generateZigzagTrack returns empty on invalid spacing (no hang)."""

    def test_zero_spacing_returns_empty(self):
        result = _run_node(
            "var poly = [[0,0],[1000,0],[1000,1000],[0,1000]];\n"
            "console.log(JSON.stringify(generateZigzagTrack(poly, 0, 0, 0)));"
        )
        track, count = result
        self.assertEqual(track, [])
        self.assertEqual(count, 0)

    def test_negative_spacing_returns_empty(self):
        result = _run_node(
            "var poly = [[0,0],[1000,0],[1000,1000],[0,1000]];\n"
            "console.log(JSON.stringify(generateZigzagTrack(poly, -100, 0, 0)));"
        )
        track, count = result
        self.assertEqual(track, [])
        self.assertEqual(count, 0)


class TestGenerateStripsGuardsJS(unittest.TestCase):
    """JS generateStrips returns empty on invalid spacing (no hang)."""

    def test_zero_spacing_returns_empty(self):
        result = _run_node(
            "var poly = [[0,0],[1000,0],[1000,1000],[0,1000]];\n"
            "console.log(JSON.stringify(generateStrips(poly, 0, 0, 0)));"
        )
        self.assertEqual(result, [])

    def test_negative_spacing_returns_empty(self):
        result = _run_node(
            "var poly = [[0,0],[1000,0],[1000,1000],[0,1000]];\n"
            "console.log(JSON.stringify(generateStrips(poly, -50, 0, 0)));"
        )
        self.assertEqual(result, [])


class TestAnalyzeAreaBadCameraJS(unittest.TestCase):
    """analyzeArea with bad camera pitch returns valid (zero-strip) result."""

    def test_positive_pitch_analyze_no_hang(self):
        """analyzeArea with pitch=+35 should return result without hanging."""
        # Override global config to use bad pitch
        result = _run_node(
            "CAMERA_PITCH_DEG = 35.0;\n"
            "FOV_HORIZONTAL_DEG = 46.2;\n"
            "FOV_VERTICAL_DEG = 26.8;\n"
            "var poly = [{lat:32,lon:34.8},{lat:32,lon:34.9},{lat:32.1,lon:34.9},{lat:32.1,lon:34.8}];\n"
            "var r = analyzeArea(poly, ['small']);\n"
            "console.log(JSON.stringify({strip_count: r.strip_count, spacing: r.track_spacing_m}));"
        )
        self.assertEqual(result["strip_count"], 0)
        self.assertEqual(result["spacing"], 0)


if __name__ == "__main__":
    unittest.main()
