"""Tests for StatusBarIndicator utility functions."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_COMPONENT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "hud",
    "StatusBarIndicator.jsx",
))


def _strip_es_modules(src):
    """Remove ES module import/export and JSX so Node.js can eval the code."""
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


_COLORS_JS = """
const colors = {
  textDim: '#8899aa',
  success: '#4caf50',
  warning: '#ff9800',
  error: '#f44336',
  accent: '#00d2ff',
  text: '#e0e0e0',
};
"""

_JS_SRC = _COLORS_JS + _strip_es_modules(
    open(_COMPONENT_PATH, encoding="utf-8").read()
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


# Color constants matching styles.js
C_SUCCESS = '#4caf50'
C_WARNING = '#ff9800'
C_ERROR = '#f44336'
C_ACCENT = '#00d2ff'


class TestGpsFixLabel(unittest.TestCase):
    """gpsFixLabel returns correct label string per fix type."""

    def test_null(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(null));"), '--')

    def test_undefined(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(undefined));"), '--')

    def test_no_fix_0(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(0));"), '--')

    def test_no_fix_1(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(1));"), '--')

    def test_2d(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(2));"), '2D')

    def test_3d(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(3));"), '3D')

    def test_dgps(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(4));"), 'DGPS')

    def test_rtk_float(self):
        out = _run_js("console.log(gpsFixLabel(5));")
        # Node may normalize thin space (\u2009) to regular space
        self.assertIn('RTK', out)
        self.assertTrue(out.endswith('F'))

    def test_rtk_fixed(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(6));"), 'RTK')

    def test_unknown_value(self):
        self.assertEqual(_run_js("console.log(gpsFixLabel(99));"), '--')


class TestGpsColor(unittest.TestCase):
    """gpsColor returns correct color per fix type."""

    def test_null(self):
        self.assertEqual(_run_js("console.log(gpsColor(null));"), C_ERROR)

    def test_undefined(self):
        self.assertEqual(_run_js("console.log(gpsColor(undefined));"), C_ERROR)

    def test_no_fix_0(self):
        self.assertEqual(_run_js("console.log(gpsColor(0));"), C_ERROR)

    def test_no_fix_1(self):
        self.assertEqual(_run_js("console.log(gpsColor(1));"), C_ERROR)

    def test_2d_warning(self):
        self.assertEqual(_run_js("console.log(gpsColor(2));"), C_WARNING)

    def test_3d_success(self):
        self.assertEqual(_run_js("console.log(gpsColor(3));"), C_SUCCESS)

    def test_dgps_success(self):
        self.assertEqual(_run_js("console.log(gpsColor(4));"), C_SUCCESS)

    def test_rtk_float_accent(self):
        self.assertEqual(_run_js("console.log(gpsColor(5));"), C_ACCENT)

    def test_rtk_fixed_accent(self):
        self.assertEqual(_run_js("console.log(gpsColor(6));"), C_ACCENT)


class TestLinkBars(unittest.TestCase):
    """linkBars maps quality 0-100 to 0-4 bars."""

    def test_null(self):
        self.assertEqual(_run_js("console.log(linkBars(null));"), '0')

    def test_zero(self):
        self.assertEqual(_run_js("console.log(linkBars(0));"), '0')

    def test_low(self):
        self.assertEqual(_run_js("console.log(linkBars(15));"), '1')

    def test_medium_low(self):
        self.assertEqual(_run_js("console.log(linkBars(30));"), '2')

    def test_medium(self):
        self.assertEqual(_run_js("console.log(linkBars(55));"), '3')

    def test_high(self):
        self.assertEqual(_run_js("console.log(linkBars(80));"), '4')

    def test_full(self):
        self.assertEqual(_run_js("console.log(linkBars(100));"), '4')


class TestLinkBarColor(unittest.TestCase):
    """linkBarColor returns correct color per bar count."""

    def test_0_bars(self):
        self.assertEqual(_run_js("console.log(linkBarColor(0));"), C_ERROR)

    def test_1_bar(self):
        self.assertEqual(_run_js("console.log(linkBarColor(1));"), C_ERROR)

    def test_2_bars(self):
        self.assertEqual(_run_js("console.log(linkBarColor(2));"), C_WARNING)

    def test_3_bars(self):
        self.assertEqual(_run_js("console.log(linkBarColor(3));"), C_SUCCESS)

    def test_4_bars(self):
        self.assertEqual(_run_js("console.log(linkBarColor(4));"), C_SUCCESS)


class TestHaccLabel(unittest.TestCase):
    """haccLabel formats horizontal accuracy for display."""

    def test_null(self):
        self.assertEqual(_run_js("console.log(haccLabel(null));"), '--')

    def test_undefined(self):
        self.assertEqual(_run_js("console.log(haccLabel(undefined));"), '--')

    def test_above_1m(self):
        self.assertEqual(_run_js("console.log(haccLabel(1.5));"), '1.5 m')

    def test_below_1m_cm(self):
        self.assertEqual(_run_js("console.log(haccLabel(0.03));"), '3 cm')

    def test_exact_1m(self):
        self.assertEqual(_run_js("console.log(haccLabel(1));"), '1.0 m')

    def test_rtk_accuracy(self):
        self.assertEqual(_run_js("console.log(haccLabel(0.014));"), '1 cm')


if __name__ == "__main__":
    unittest.main()
