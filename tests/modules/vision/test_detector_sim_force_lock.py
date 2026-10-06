"""DetectorSim must honor click/id force-lock so it matches the real detector
for the GCS (F06: previously force-lock was a silent no-op in sim)."""
import unittest

from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.sim.detection_publication_buffer import (
    DetectionPublicationBuffer,
)
from tests.detection_factory import make_detected_target


def _target(obj_id, cx, cy, w=40.0, h=40.0):
    return make_detected_target(
        obj_id=obj_id,
        task_id=obj_id,
        tracking_bbox_cxcywh=(float(cx), float(cy), float(w), float(h)),
    )


class TestResolveForcedTarget(unittest.TestCase):
    def setUp(self):
        self.targets = [_target(1, 200, 200), _target(2, 900, 500)]

    def test_no_request_returns_none(self):
        self.assertIsNone(DetectionPublicationBuffer.resolve_forced_target(DetectRequest(), self.targets))

    def test_force_lock_id_selects_that_target(self):
        forced = DetectionPublicationBuffer.resolve_forced_target(
            DetectRequest(force_lock_id=2), self.targets,
        )
        self.assertEqual(forced.identity.obj_id, 2)

    def test_force_lock_bbox_selects_overlapping_target(self):
        forced = DetectionPublicationBuffer.resolve_forced_target(
            DetectRequest(force_lock_bbox_cxcywh=(905, 505, 60, 60)), self.targets,
        )
        self.assertEqual(forced.identity.obj_id, 2)

    def test_force_lock_bbox_prefers_overlap_over_distance(self):
        # A click overlapping target 1 must pick 1 even though 2 exists.
        forced = DetectionPublicationBuffer.resolve_forced_target(
            DetectRequest(force_lock_bbox_cxcywh=(205, 205, 50, 50)), self.targets,
        )
        self.assertEqual(forced.identity.obj_id, 1)

    def test_force_lock_bbox_with_no_overlap_returns_none(self):
        forced = DetectionPublicationBuffer.resolve_forced_target(
            DetectRequest(force_lock_bbox_cxcywh=(1600, 900, 10, 10)), self.targets,
        )
        self.assertIsNone(forced)


if __name__ == "__main__":
    unittest.main()
