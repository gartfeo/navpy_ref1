"""Tests for mission assembly utility functions.

Runs test_mission_assembly_logic.js via Node.js subprocess to verify
assembleMissionsFromResults works correctly.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_mission_assembly_logic.js",
))


class TestMissionAssembly(unittest.TestCase):

    def test_mission_assembly_logic_js(self):
        """Run Node.js mission assembly tests and verify pass."""
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
