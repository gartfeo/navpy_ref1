"""Tests for monitor phase state machine logic.

Runs test_monitor_phase_logic.js via Node.js subprocess to verify the
computeMonitorPhase function returns the correct phase for all combinations
of vehicleList state and containerLaunching flag.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_monitor_phase_logic.js",
))


class TestMonitorPhase(unittest.TestCase):

    def test_monitor_phase_logic_js(self):
        """Run Node.js monitor phase tests and verify PASS."""
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
