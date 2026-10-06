"""Tests for DOCK-preserving POI reconstruction in handleConnect.

Runs test_connect_dock_logic.js via Node.js subprocess to verify that
existing zones preserve their DOCK assignments while the newly connected
zone gets the downloaded fallback_delivery_location.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_connect_dock_logic.js",
))


class TestConnectDock(unittest.TestCase):

    def test_connect_dock_logic_js(self):
        """Run Node.js connect DOCK logic tests and verify PASS output."""
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
