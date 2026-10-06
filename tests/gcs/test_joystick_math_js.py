"""Tests for joystickMath.js pure utility functions via Node.js subprocess."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

_JS_PATH = os.path.join(_UTILS_DIR, "joystickMath.js")


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


_JS_CODE = _strip_es_modules(open(_JS_PATH, encoding="utf-8").read())


def _run_js(snippet):
    """Evaluate JS snippet with joystickMath loaded, return parsed JSON."""
    full = _JS_CODE + "\n" + snippet
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestClampToSquare(unittest.TestCase):
    def test_inside(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(0.5, 0.3)))')
        self.assertAlmostEqual(r["x"], 0.5, places=5)
        self.assertAlmostEqual(r["y"], 0.3, places=5)

    def test_origin(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(0, 0)))')
        self.assertEqual(r["x"], 0)
        self.assertEqual(r["y"], 0)

    def test_on_edge(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(1, 0)))')
        self.assertAlmostEqual(r["x"], 1.0, places=5)
        self.assertAlmostEqual(r["y"], 0.0, places=5)

    def test_outside_clamped(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(2, 0)))')
        self.assertAlmostEqual(r["x"], 1.0, places=5)
        self.assertAlmostEqual(r["y"], 0.0, places=5)

    def test_corner_reachable(self):
        """Square clamp allows full deflection on both axes simultaneously."""
        r = _run_js('console.log(JSON.stringify(clampToSquare(1, 1)))')
        self.assertAlmostEqual(r["x"], 1.0, places=5)
        self.assertAlmostEqual(r["y"], 1.0, places=5)

    def test_negative_corner(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(-1, -1)))')
        self.assertAlmostEqual(r["x"], -1.0, places=5)
        self.assertAlmostEqual(r["y"], -1.0, places=5)

    def test_overflow_clamped_independently(self):
        r = _run_js('console.log(JSON.stringify(clampToSquare(-3, 4)))')
        self.assertAlmostEqual(r["x"], -1.0, places=5)
        self.assertAlmostEqual(r["y"], 1.0, places=5)


class TestAxisToMavlink(unittest.TestCase):
    def test_center(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(0)))')
        self.assertEqual(r, 0)

    def test_full_positive(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(1)))')
        self.assertEqual(r, 1000)

    def test_full_negative(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(-1)))')
        self.assertEqual(r, -1000)

    def test_half(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(0.5)))')
        self.assertEqual(r, 500)

    def test_clamps_above(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(2.5)))')
        self.assertEqual(r, 1000)

    def test_clamps_below(self):
        r = _run_js('console.log(JSON.stringify(axisToMavlink(-1.5)))')
        self.assertEqual(r, -1000)


class TestThrottleToMavlink(unittest.TestCase):
    def test_center(self):
        r = _run_js('console.log(JSON.stringify(throttleToMavlink(0)))')
        self.assertEqual(r, 500)

    def test_full_up(self):
        r = _run_js('console.log(JSON.stringify(throttleToMavlink(1)))')
        self.assertEqual(r, 1000)

    def test_full_down(self):
        r = _run_js('console.log(JSON.stringify(throttleToMavlink(-1)))')
        self.assertEqual(r, 0)

    def test_clamps_above(self):
        r = _run_js('console.log(JSON.stringify(throttleToMavlink(5)))')
        self.assertEqual(r, 1000)

    def test_clamps_below(self):
        r = _run_js('console.log(JSON.stringify(throttleToMavlink(-5)))')
        self.assertEqual(r, 0)


class TestRcPwmToStickY(unittest.TestCase):
    def test_min_pwm(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(1000)))')
        self.assertAlmostEqual(r, -1.0, places=5)

    def test_mid_pwm(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(1500)))')
        self.assertAlmostEqual(r, 0.0, places=5)

    def test_max_pwm(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(2000)))')
        self.assertAlmostEqual(r, 1.0, places=5)

    def test_null_defaults_to_min(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(null)))')
        self.assertAlmostEqual(r, -1.0, places=5)

    def test_undefined_defaults_to_min(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(undefined)))')
        self.assertAlmostEqual(r, -1.0, places=5)

    def test_below_range_clamped(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(800)))')
        self.assertAlmostEqual(r, -1.0, places=5)

    def test_above_range_clamped(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(2200)))')
        self.assertAlmostEqual(r, 1.0, places=5)

    def test_quarter_throttle(self):
        r = _run_js('console.log(JSON.stringify(rcPwmToStickY(1250)))')
        self.assertAlmostEqual(r, -0.5, places=5)


class TestBuildManualControlPayload(unittest.TestCase):
    def test_neutral(self):
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:0,y:0},{x:0,y:0})))'
        )
        self.assertEqual(r["x"], 0)    # pitch
        self.assertEqual(r["y"], 0)    # roll
        self.assertEqual(r["z"], 500)  # throttle at center
        self.assertEqual(r["r"], 0)    # yaw

    def test_full_forward(self):
        """Right stick full up → pitch = 1000."""
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:0,y:0},{x:0,y:1})))'
        )
        self.assertEqual(r["x"], 1000)

    def test_full_right_roll(self):
        """Right stick full right → roll = 1000."""
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:0,y:0},{x:1,y:0})))'
        )
        self.assertEqual(r["y"], 1000)

    def test_full_throttle(self):
        """Left stick full up → throttle = 1000."""
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:0,y:1},{x:0,y:0})))'
        )
        self.assertEqual(r["z"], 1000)

    def test_full_yaw_left(self):
        """Left stick full left → yaw = -1000."""
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:-1,y:0},{x:0,y:0})))'
        )
        self.assertEqual(r["r"], -1000)

    def test_combined(self):
        """Both sticks at half deflection."""
        r = _run_js(
            'console.log(JSON.stringify(buildManualControlPayload({x:0.5,y:-0.5},{x:-0.5,y:0.5})))'
        )
        self.assertEqual(r["x"], 500)    # pitch (right y=0.5)
        self.assertEqual(r["y"], -500)   # roll (right x=-0.5)
        self.assertEqual(r["z"], 250)    # throttle (left y=-0.5)
        self.assertEqual(r["r"], 500)    # yaw (left x=0.5)


if __name__ == "__main__":
    unittest.main()
