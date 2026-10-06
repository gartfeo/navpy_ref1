"""Tests for BatteryIndicator batteryColor function."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_COMPONENT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "hud",
    "BatteryIndicator.jsx",
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


# Inline the colors constants so the function can resolve them
_COLORS_JS = """
const colors = {
  textDim: '#8899aa',
  success: '#4caf50',
  warning: '#ff9800',
  error: '#f44336',
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


class TestBatteryColor(unittest.TestCase):
    """batteryColor returns correct color per battery percentage bracket."""

    def test_null_returns_dim(self):
        self.assertEqual(_run_js("console.log(batteryColor(null));"), colors_textDim)

    def test_undefined_returns_dim(self):
        self.assertEqual(_run_js("console.log(batteryColor(undefined));"), colors_textDim)

    def test_high_battery(self):
        self.assertEqual(_run_js("console.log(batteryColor(85));"), colors_success)

    def test_boundary_51(self):
        self.assertEqual(_run_js("console.log(batteryColor(51));"), colors_success)

    def test_boundary_50(self):
        """50% is NOT > 50, so it falls into warning."""
        self.assertEqual(_run_js("console.log(batteryColor(50));"), colors_warning)

    def test_mid_battery(self):
        self.assertEqual(_run_js("console.log(batteryColor(35));"), colors_warning)

    def test_boundary_20(self):
        self.assertEqual(_run_js("console.log(batteryColor(20));"), colors_warning)

    def test_boundary_19(self):
        self.assertEqual(_run_js("console.log(batteryColor(19));"), colors_error)

    def test_low_battery(self):
        self.assertEqual(_run_js("console.log(batteryColor(5));"), colors_error)

    def test_zero_battery(self):
        self.assertEqual(_run_js("console.log(batteryColor(0));"), colors_error)


# Color constants matching styles.js
colors_textDim = '#8899aa'
colors_success = '#4caf50'
colors_warning = '#ff9800'
colors_error = '#f44336'


if __name__ == "__main__":
    unittest.main()
