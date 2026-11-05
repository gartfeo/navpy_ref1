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

    def test_only_the_live_segment_uses_a_callback_property(self):
        # The recorded trail pieces take static positions; the one per-frame
        # CallbackProperty is the live segment to the model (todos 264, 267).
        self.assertEqual(self.source.count("new Cesium.CallbackProperty"), 1)
        live = self.source.index("function addLiveEntity")
        self.assertGreater(self.source.index("new Cesium.CallbackProperty"), live)
        self.assertLess(self.source.index("new Cesium.CallbackProperty"),
                        self.source.index("refreshRef.current = (sid)"))

    def test_pieces_are_polylines_of_a_collection(self):
        # A changed static entity polyline is rebuilt in the background and
        # the old one shown meanwhile, which made the trail head blink; the
        # pieces are PolylineCollection polylines, set directly (todo 293).
        # The one entity is the live segment.
        self.assertIn("new Cesium.PolylineCollection()", self.source)
        self.assertEqual(self.source.count("viewer.entities.add("), 1)
        self.assertRegex(self.source, r"line\.positions\s*=")


if __name__ == "__main__":
    unittest.main()
