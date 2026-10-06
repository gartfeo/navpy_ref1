"""Tests for ground-marker SVG icon helpers.

Runs test_ground_marker_billboards_logic.js via Node.js subprocess.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_ground_marker_billboards_logic.js",
))


class TestGroundMarkerBillboards(unittest.TestCase):

    def test_ground_marker_billboards_logic_js(self):
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
