"""Tests for useUavTrails static position updates."""
import os
import re
import unittest


_TRAILS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "useUavTrails.js",
))


class TestUavTrailsUpdate(unittest.TestCase):
    def setUp(self):
        with open(_TRAILS_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_no_callback_property(self):
        self.assertNotIn("CallbackProperty", self.source)

    def test_positions_assigned(self):
        self.assertRegex(self.source, r"polyline\.positions\s*=")


if __name__ == "__main__":
    unittest.main()
