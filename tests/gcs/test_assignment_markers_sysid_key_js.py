"""Tests for useAssignmentMarkers sys_id key dependency."""
import os
import re
import unittest


_ASSIGNMENT_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "useAssignmentMarkers.js",
))


class TestAssignmentMarkersSysIdKey(unittest.TestCase):
    def setUp(self):
        with open(_ASSIGNMENT_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_imports_store_hook(self):
        self.assertIn("useTelemetryStore", self.source)

    def test_uses_sysid_key(self):
        self.assertIn("getSysIdKey", self.source)
        self.assertRegex(self.source, r"\[[^\]]*sysIdKey[^\]]*\]")


if __name__ == "__main__":
    unittest.main()
