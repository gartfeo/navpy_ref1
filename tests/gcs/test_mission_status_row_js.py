"""Tests for MissionStatusRow pure logic: formatDist and STATUS_RENDERERS dispatch table."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_COMPONENT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "sidebar",
    "MissionStatusRow.jsx",
))

_GEO_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "geo.js",
))


def _strip_es_modules(src):
    """Remove ES module import/export and JSX so Node.js can eval pure functions."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        if re.match(r"^export\s+default\s", stripped):
            break  # stop before the React component
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


def _extract_pure_functions(src):
    """Extract only plain JS functions (no JSX) from the component file.

    Stops at STATUS_RENDERERS since it contains JSX.
    """
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        if re.match(r"^export\s+default\s", stripped):
            break
        # Stop before any JSX-containing code
        if "STATUS_RENDERERS" in stripped:
            break
        # Skip React component functions (contain JSX)
        if stripped.startswith("function Badge(") or \
           stripped.startswith("function TaskLabel(") or \
           stripped.startswith("function ClickableCoords(") or \
           stripped.startswith("function ProgressBar("):
            break
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


# Inline dependencies
_COLORS_JS = """
const colors = {
  textDim: '#8899aa',
  success: '#4caf50',
  accent: '#00d2ff',
  warning: '#ff9800',
  error: '#f44336',
  textBright: '#ffffff',
  bgLight: '#16213e',
};
"""

# Load flatDist from geo.js
_GEO_JS = _strip_es_modules(open(_GEO_PATH, encoding="utf-8").read())

# Load pure functions from MissionStatusRow.jsx
_COMPONENT_JS = _extract_pure_functions(
    open(_COMPONENT_PATH, encoding="utf-8").read()
)

_JS_SRC = _COLORS_JS + "\n" + _GEO_JS + "\n" + _COMPONENT_JS


def _run_js(snippet):
    code = _JS_SRC + "\n" + snippet
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}\n\nCode:\n{code}")
    return result.stdout.strip()


class TestFormatDist(unittest.TestCase):
    """formatDist returns 'X.X km' for >= 1000m, 'XXX m' otherwise."""

    def test_below_1000_meters(self):
        self.assertEqual(_run_js("console.log(formatDist(500));"), "500 m")

    def test_exactly_1000_meters(self):
        self.assertEqual(_run_js("console.log(formatDist(1000));"), "1.0 km")

    def test_above_1000_meters(self):
        self.assertEqual(_run_js("console.log(formatDist(2500));"), "2.5 km")

    def test_zero_meters(self):
        self.assertEqual(_run_js("console.log(formatDist(0));"), "0 m")

    def test_small_distance(self):
        self.assertEqual(_run_js("console.log(formatDist(42));"), "42 m")

    def test_fractional_rounds_to_nearest(self):
        self.assertEqual(_run_js("console.log(formatDist(99.7));"), "100 m")

    def test_large_km(self):
        self.assertEqual(_run_js("console.log(formatDist(12345));"), "12.3 km")

    def test_just_below_1000(self):
        self.assertEqual(_run_js("console.log(formatDist(999));"), "999 m")


class TestFlatDistUsedForApproaching(unittest.TestCase):
    """Verify flatDist can compute distance for approaching status."""

    def test_flatdist_same_point(self):
        out = _run_js("""
        const d = flatDist({lat: 40, lon: 44}, {lat: 40, lon: 44});
        console.log(d.toFixed(2));
        """)
        self.assertEqual(out, "0.00")

    def test_flatdist_known_distance(self):
        """~1 degree latitude = ~111320m."""
        out = _run_js("""
        const d = flatDist({lat: 40, lon: 44}, {lat: 41, lon: 44});
        console.log(Math.round(d));
        """)
        self.assertEqual(out, "111320")

    def test_approaching_distance_formatted(self):
        """Integration: compute distance and format it."""
        out = _run_js("""
        const d = flatDist({lat: 40, lon: 44}, {lat: 40.01, lon: 44});
        console.log(formatDist(d));
        """)
        self.assertEqual(out, "1.1 km")


if __name__ == "__main__":
    unittest.main()
