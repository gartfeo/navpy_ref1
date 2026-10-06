"""Tests for restart/start mission button visibility logic.

Runs test_restart_button_logic.js via Node.js subprocess to verify the
restart button is shown whenever at least one vehicle is armed (including
in GUIDED mode), not only when ALL vehicles are armed.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_restart_button_logic.js",
))


class TestRestartButton(unittest.TestCase):

    def test_restart_button_visibility_js(self):
        """Run Node.js restart button visibility tests and verify PASS."""
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
