import unittest

import cv2
import numpy as np

from navpy.modules.vision.lost_poi_bridge import LostPoiBridge


def _scene(cx, cy, patch):
    frame = np.full((720, 1280, 3), 25, np.uint8)
    x1, y1 = int(cx - 45), int(cy - 30)
    frame[y1:y1 + 60, x1:x1 + 90] = patch
    return frame


class TestLostPoiBridge(unittest.TestCase):
    def setUp(self):
        self.patch = np.random.default_rng(1).integers(0, 255, (60, 90, 3)).astype(np.uint8)

    def test_follows_poi_across_a_gap(self):
        b = LostPoiBridge(min_score=0.5)
        b.observe(_scene(300, 360, self.patch), (300, 360, 90, 60), now=0.0)
        # POI moved while undetected; the bridge should find it nearby
        hit = b.search(_scene(360, 380, self.patch), now=0.05)
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.cx, 360, delta=12)
        self.assertAlmostEqual(hit.cy, 380, delta=12)

    def test_no_hint_when_poi_absent(self):
        b = LostPoiBridge(min_score=0.6)
        b.observe(_scene(300, 360, self.patch), (300, 360, 90, 60), now=0.0)
        blank = np.full((720, 1280, 3), 25, np.uint8)   # POI gone (occluded)
        self.assertIsNone(b.search(blank, now=0.05))

    def test_gives_up_after_max_bridge_seconds(self):
        b = LostPoiBridge(min_score=0.5, max_bridge_seconds=1.0)
        b.observe(_scene(300, 360, self.patch), (300, 360, 90, 60), now=0.0)
        b.search(_scene(300, 360, self.patch), now=0.1)
        self.assertIsNone(b.search(_scene(300, 360, self.patch), now=2.0))
        self.assertFalse(b.active)

    def test_follows_zoom_drift(self):
        # During a zoom the POI grows a few %/frame while undetected; the
        # multi-scale search must keep matching and grow the template with it.
        b = LostPoiBridge(min_score=0.5)
        # Structured patch (low-frequency): pure noise decorrelates under
        # resampling, but real objects (cars) have structure that survives it.
        structured = cv2.GaussianBlur(self.patch, (9, 9), 3)
        b.observe(_scene(640, 360, structured), (640, 360, 90, 60), now=0.0)
        scale = 1.0
        hit = None
        for k in range(1, 9):       # ~1.48x total growth by the end
            scale *= 1.05
            big = cv2.resize(structured, (int(90 * scale), int(60 * scale)))
            frame = np.full((720, 1280, 3), 25, np.uint8)
            x1, y1 = int(640 - big.shape[1] / 2), int(360 - big.shape[0] / 2)
            frame[y1:y1 + big.shape[0], x1:x1 + big.shape[1]] = big
            hit = b.search(frame, now=0.04 * k)
            self.assertIsNotNone(hit, f"lost the POI at zoom step {k} (x{scale:.2f})")
        self.assertAlmostEqual(hit.cx, 640, delta=15)
        self.assertGreater(b._scale, 1.15)   # cumulative scale grew with the zoom

    def test_cumulative_scale_is_clamped(self):
        # A long zoom-in bridge must not grow the template unboundedly: the
        # cumulative scale is capped (and the template derives from the crisp
        # original, not a compounding chain of resizes).
        b = LostPoiBridge(min_score=0.4)
        structured = cv2.GaussianBlur(self.patch, (9, 9), 3)
        b.observe(_scene(640, 360, structured), (640, 360, 90, 60), now=0.0)
        scale = 1.0
        for k in range(1, 60):           # drive way past the cap
            scale = min(4.0, scale * 1.05)
            big = cv2.resize(structured, (int(90 * scale), int(60 * scale)))
            frame = np.full((720, 1280, 3), 25, np.uint8)
            x1, y1 = max(0, int(640 - big.shape[1] / 2)), max(0, int(360 - big.shape[0] / 2))
            frame[y1:y1 + big.shape[0], x1:x1 + big.shape[1]] = big
            b.search(frame, now=0.02 * k)
        self.assertLessEqual(b._scale, b._scale_max + 1e-9)   # never exceeds the cap

    def test_flat_template_is_rejected_fail_closed(self):
        # TM_CCOEFF_NORMED scores a zero-variance template 1.0 against ANY
        # window, so min_score cannot reject it — the bridge must refuse to
        # track an information-free crop in the first place (Codex finding).
        b = LostPoiBridge(min_score=0.55)
        flat = np.full((720, 1280, 3), 50, np.uint8)
        b.observe(flat, (640, 360, 20, 20), now=0.0)
        self.assertFalse(b.active)
        self.assertIsNone(b.search(np.full((720, 1280, 3), 200, np.uint8), now=0.1))

    def test_flat_observe_keeps_previous_good_template(self):
        b = LostPoiBridge(min_score=0.5)
        b.observe(_scene(300, 360, self.patch), (300, 360, 90, 60), now=0.0)
        # a later flat crop (e.g. occluder fills the box) must not overwrite
        b.observe(np.full((720, 1280, 3), 50, np.uint8), (300, 360, 90, 60), now=0.1)
        self.assertTrue(b.active)
        hit = b.search(_scene(310, 362, self.patch), now=0.2)
        self.assertIsNotNone(hit)

    def test_reset_clears_template(self):
        b = LostPoiBridge()
        b.observe(_scene(300, 360, self.patch), (300, 360, 90, 60), now=0.0)
        self.assertTrue(b.active)
        b.reset()
        self.assertFalse(b.active)
        self.assertIsNone(b.search(_scene(300, 360, self.patch), now=0.1))


if __name__ == "__main__":
    unittest.main()
