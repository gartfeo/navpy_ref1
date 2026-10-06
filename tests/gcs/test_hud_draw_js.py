"""Tests for hudDraw.js pure utility functions."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


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


_HUD_DRAW_JS = _strip_es_modules(
    open(os.path.join(_UTILS_DIR, "hudDraw.js"), encoding="utf-8").read()
)


def _run_js(script):
    code = _HUD_DRAW_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestNormalizeHeading(unittest.TestCase):
    def test_positive(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(90));"), "90")

    def test_zero(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(0));"), "0")

    def test_360(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(360));"), "0")

    def test_negative(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(-90));"), "270")

    def test_large_positive(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(720));"), "0")

    def test_large_negative(self):
        self.assertEqual(_run_js("console.log(normalizeHeading(-450));"), "270")

    def test_fractional(self):
        out = _run_js("console.log(normalizeHeading(365.5));")
        self.assertAlmostEqual(float(out), 5.5, places=5)


class TestHeadingLabel(unittest.TestCase):
    def test_north(self):
        self.assertEqual(_run_js("console.log(headingLabel(0));"), "N")

    def test_east(self):
        self.assertEqual(_run_js("console.log(headingLabel(90));"), "E")

    def test_south(self):
        self.assertEqual(_run_js("console.log(headingLabel(180));"), "S")

    def test_west(self):
        self.assertEqual(_run_js("console.log(headingLabel(270));"), "W")

    def test_padded(self):
        self.assertEqual(_run_js("console.log(headingLabel(30));"), "030")

    def test_three_digits(self):
        self.assertEqual(_run_js("console.log(headingLabel(150));"), "150")

    def test_wrap_negative(self):
        self.assertEqual(_run_js("console.log(headingLabel(-90));"), "W")

    def test_360_is_north(self):
        self.assertEqual(_run_js("console.log(headingLabel(360));"), "N")


class TestClamp(unittest.TestCase):
    def test_within(self):
        self.assertEqual(_run_js("console.log(clamp(5, 0, 10));"), "5")

    def test_below(self):
        self.assertEqual(_run_js("console.log(clamp(-5, 0, 10));"), "0")

    def test_above(self):
        self.assertEqual(_run_js("console.log(clamp(15, 0, 10));"), "10")

    def test_at_min(self):
        self.assertEqual(_run_js("console.log(clamp(0, 0, 10));"), "0")

    def test_at_max(self):
        self.assertEqual(_run_js("console.log(clamp(10, 0, 10));"), "10")


class TestFmtVal(unittest.TestCase):
    def test_integer(self):
        self.assertEqual(_run_js("console.log(fmtVal(42));"), "42")

    def test_decimal(self):
        self.assertEqual(_run_js("console.log(fmtVal(3.14159, 2));"), "3.14")

    def test_null(self):
        self.assertEqual(_run_js("console.log(fmtVal(null));"), "--")

    def test_undefined(self):
        self.assertEqual(_run_js("console.log(fmtVal(undefined));"), "--")

    def test_nan(self):
        self.assertEqual(_run_js("console.log(fmtVal(NaN));"), "--")

    def test_zero(self):
        self.assertEqual(_run_js("console.log(fmtVal(0, 1));"), "0.0")


if __name__ == "__main__":
    unittest.main()
