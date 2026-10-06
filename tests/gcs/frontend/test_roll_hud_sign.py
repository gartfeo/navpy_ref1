"""Tests for AttitudeIndicator roll sign correctness.

Runs test_roll_hud_sign_logic.js via Node.js subprocess to verify that
the horizon rotation and bank pointer use -rollRad (positive roll = right
bank should tilt the horizon counter-clockwise).
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_roll_hud_sign_logic.js",
))


class TestRollHudSign(unittest.TestCase):

    def test_roll_hud_sign_logic_js(self):
        """Run Node.js roll sign tests and verify PASS output."""
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("PASS", result.stdout, "Expected PASS in stdout")
