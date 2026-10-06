"""Tests for the zoomToHeight utility used in useCesiumViewer.

Runs test_zoom_to_height.js via Node.js subprocess.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_zoom_to_height.js",
))


class TestZoomToHeight(unittest.TestCase):

    def test_zoom_to_height_js(self):
        """Run Node.js zoom-to-height tests and verify PASS output."""
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
