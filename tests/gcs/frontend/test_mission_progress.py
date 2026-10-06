"""Tests for mission progress utility functions.

Runs test_mission_progress_logic.js via Node.js subprocess to verify
computeZoneDistances and interpolatedDistance work correctly.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_mission_progress_logic.js",
))


class TestMissionProgress(unittest.TestCase):

    def test_mission_progress_logic_js(self):
        """Run Node.js mission progress tests and verify pass."""
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("passed", result.stdout, "Expected 'passed' in stdout")


if __name__ == "__main__":
    unittest.main()
