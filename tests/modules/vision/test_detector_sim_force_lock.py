"""DetectorSim must honor click/id force-lock so it matches the real detector
for the GCS (F06: previously force-lock was a silent no-op in sim)."""
import unittest

from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.sim.detection_publication_buffer import (
    DetectionPublicationBuffer,
)
from tests.detection_factory import make_detected_poi


def _poi(obj_id, cx, cy, w=40.0, h=40.0):
    return make_detected_poi(
        obj_id=obj_id,
        task_id=obj_id,
        tracking_bbox_cxcywh=(float(cx), float(cy), float(w), float(h)),
    )


class TestResolveForcedPoi(unittest.TestCase):
    def setUp(self):
        self.pois = [_poi(1, 200, 200), _poi(2, 900, 500)]

    def test_no_request_returns_none(self):
        self.assertIsNone(DetectionPublicationBuffer.resolve_forced_poi(DetectRequest(), self.pois))

    def test_force_lock_id_selects_that_poi(self):
        forced = DetectionPublicationBuffer.resolve_forced_poi(
            DetectRequest(force_lock_id=2), self.pois,
        )
        self.assertEqual(forced.identity.obj_id, 2)

    def test_force_lock_bbox_selects_overlapping_poi(self):
        forced = DetectionPublicationBuffer.resolve_forced_poi(
            DetectRequest(force_lock_bbox_cxcywh=(905, 505, 60, 60)), self.pois,
        )
        self.assertEqual(forced.identity.obj_id, 2)

    def test_force_lock_bbox_prefers_overlap_over_distance(self):
        # A click overlapping POI 1 must pick 1 even though 2 exists.
        forced = DetectionPublicationBuffer.resolve_forced_poi(
            DetectRequest(force_lock_bbox_cxcywh=(205, 205, 50, 50)), self.pois,
        )
        self.assertEqual(forced.identity.obj_id, 1)

    def test_force_lock_bbox_with_no_overlap_returns_none(self):
        forced = DetectionPublicationBuffer.resolve_forced_poi(
            DetectRequest(force_lock_bbox_cxcywh=(1600, 900, 10, 10)), self.pois,
        )
        self.assertIsNone(forced)


if __name__ == "__main__":
    unittest.main()
