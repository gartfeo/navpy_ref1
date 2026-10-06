"""Tests for corridor arrow heading logic in useCorridorWaypoints.js.

Runs test_corridor_arrows_logic.js via Node.js subprocess to verify the
headingRad function used to orient directional arrow billboards at
corridor waypoints.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_corridor_arrows_logic.js",
))


class TestCorridorArrows(unittest.TestCase):

    def test_corridor_arrows_logic_js(self):
        """Run Node.js corridor arrow heading tests and verify PASS output."""
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("PASS", result.stdout, "Expected PASS in stdout")


if __name__ == "__main__":
    unittest.main()
