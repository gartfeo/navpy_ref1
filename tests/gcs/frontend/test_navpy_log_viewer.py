"""Tests for NavPy sim log viewer visibility logic.

Runs test_navpy_log_viewer_logic.js via Node.js subprocess to verify the
navpyLogViewerVisibility function returns correct link/panel visibility
for all combinations of instance state and showLogs flag.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_navpy_log_viewer_logic.js",
))


class TestNavpyLogViewer(unittest.TestCase):

    def test_navpy_log_viewer_logic_js(self):
        """Run Node.js log viewer tests and verify PASS."""
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
