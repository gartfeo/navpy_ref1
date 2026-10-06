import unittest
from unittest.mock import patch

from navpy.modules.vision.multi_object_tracker import MultiObjectTracker
from navpy.modules.vision.yolo_detector import Detection


def _det(cx: float, cy: float = 500.0) -> Detection:
    return Detection(cx=cx, cy=cy, w=80.0, h=40.0, confidence=0.8, class_id=2)


def _update_at(tracker: MultiObjectTracker, ts: float, detections):
    with patch("navpy.modules.vision.multi_object_tracker.time.time", return_value=ts):
        return tracker.update(detections, frame_w=1920, frame_h=1080)


class TestMultiObjectTrackerRevive(unittest.TestCase):
    def test_offscreen_track_is_revived_with_same_id_when_it_returns(self):
        tracker = MultiObjectTracker(
            max_age=20,
            min_hits=1,
            max_center_dist_px=500.0,
            revive_seconds=5.0,
        )

        _update_at(tracker, 0.0, [_det(1700.0)])
        tracks = _update_at(tracker, 0.1, [_det(1720.0)])
        original_id = tracks[0].id

        tracker._tracks[0].kf.x[2] = 1000.0

        tracks = _update_at(tracker, 0.4, [])

        self.assertEqual(tracks, [])
        self.assertEqual(len(tracker._lost), 1)

        tracks = _update_at(tracker, 0.5, [_det(1870.0)])

        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].id, original_id)

    def test_revive_during_assignment_while_another_track_stays_active(self):
        # The other revive entry point: an unassigned detection during normal
        # assignment revives a lost track while a separate track stays active.
        # max_age=1 -> the left object becomes lost after 2 missed frames.
        tracker = MultiObjectTracker(max_age=1, min_hits=1, max_center_dist_px=400.0,
                                     revive_seconds=5.0)
        _update_at(tracker, 0.0, [_det(300.0), _det(1500.0)])
        tracks = _update_at(tracker, 0.1, [_det(305.0), _det(1500.0)])
        lost_id = next(t.id for t in tracks if t.cx < 800)
        kept_id = next(t.id for t in tracks if t.cx > 800)

        _update_at(tracker, 0.2, [_det(1500.0)])   # left missed=1 (active)
        _update_at(tracker, 0.3, [_det(1500.0)])   # left missed=2>max_age -> lost
        self.assertTrue(any(t.id == lost_id for t, _ in tracker._lost))

        # Left object returns near its last position while right stays active.
        out = _update_at(tracker, 0.4, [_det(310.0), _det(1500.0)])
        ids = {t.id for t in out}
        self.assertIn(lost_id, ids)   # revived, same id
        self.assertIn(kept_id, ids)   # never lost

    def test_returning_far_beyond_gate_gets_new_id(self):
        tracker = MultiObjectTracker(max_age=1, min_hits=1, max_center_dist_px=200.0,
                                     revive_seconds=5.0)
        _update_at(tracker, 0.0, [_det(300.0)])
        first = _update_at(tracker, 0.1, [_det(305.0)])[0]
        _update_at(tracker, 0.2, [])   # missed=1 (active, coasting)
        _update_at(tracker, 0.3, [])   # missed=2>max_age -> lost
        # returns far away (> max_center_dist) -> must NOT revive
        out = _update_at(tracker, 0.4, [_det(1500.0)])
        self.assertNotEqual(out[0].id, first.id)

    def test_revive_projection_is_capped_to_gate(self):
        # F27: an unreliable large velocity must not teleport the predicted
        # position past the gate and revive onto a wrong-position detection.
        tracker = MultiObjectTracker(max_age=20, min_hits=1, max_center_dist_px=200.0,
                                     revive_seconds=5.0)
        _update_at(tracker, 0.0, [_det(300.0)])
        _update_at(tracker, 0.1, [_det(320.0)])
        tracker._tracks[0].kf.x[2] = 100000.0   # absurd velocity
        _update_at(tracker, 0.2, [])             # lost with huge vx
        # A detection far to the right would be "reachable" by the absurd
        # projection if uncapped; capped projection keeps it beyond the gate.
        out = _update_at(tracker, 0.3, [_det(1500.0)])
        self.assertNotEqual(out[0].id, 1)


if __name__ == "__main__":
    unittest.main()
