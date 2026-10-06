"""Tests to avoid O(n^2) vehicle index lookups in map hooks."""
import os
import unittest


_HOOK_FILES = [
    os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src", "components",
        "map", "hooks", "useUavMarkers.js",
    )),
    os.path.normpath(os.path.join(
        os.path.dirname(__file__),
        "..", "..", "src", "gcs", "frontend", "src", "components",
        "map", "hooks", "useCoverageLayer.js",
    )),
]


class TestVehicleIndexOptimizations(unittest.TestCase):
    def test_no_indexof_v(self):
        for path in _HOOK_FILES:
            with open(path, encoding="utf-8") as f:
                source = f.read()
            self.assertNotIn("indexOf(v)", source)


if __name__ == "__main__":
    unittest.main()
