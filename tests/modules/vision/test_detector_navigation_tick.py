"""Navigation must not be fed a STALE cached target as a fresh measurement when
detection stalls — otherwise the loss timer never fires (Codex review fix)."""
import unittest
from unittest.mock import patch

from navpy.modules.vision.real_detector_state import FreshnessPolicy
from navpy.modules.vision.real_tracking import TrackingNavigationSink
from tests.detection_factory import make_detected_target


class _Navigation:
    def __init__(self, tracking_obj_id):
        self.tracking_obj_id = tracking_obj_id
        self.updated = []
        self.fail_next = False

    def update(self, target):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("actuation failed")
        self.updated.append(target)


def _target(task_id, timestamp):
    return make_detected_target(obj_id=task_id, timestamp=timestamp)


class TestTickNavigation(unittest.TestCase):
    def _sink(self, navigation):
        return TrackingNavigationSink(navigation, FreshnessPolicy(1.0 / 60.0))

    def test_stale_target_is_passed_as_none(self):
        g = _Navigation(tracking_obj_id=7)
        sink = self._sink(g)
        with patch("navpy.modules.vision.real_detector_state.time.time", return_value=100.0):
            sink.update([_target(7, timestamp=98.0)])  # 2s old -> stale
        self.assertEqual(g.updated, [None])

    def test_fresh_target_is_passed_through(self):
        g = _Navigation(tracking_obj_id=7)
        sink = self._sink(g)
        fresh = _target(7, timestamp=99.99)
        with patch("navpy.modules.vision.real_detector_state.time.time", return_value=100.0):
            sink.update([fresh])
        self.assertEqual(g.updated, [fresh])

    def test_no_navigation_is_noop(self):
        sink = self._sink(None)
        sink.update([_target(7, timestamp=100.0)])  # must not raise

    def test_untracked_id_passes_none(self):
        g = _Navigation(tracking_obj_id=99)
        sink = self._sink(g)
        with patch("navpy.modules.vision.real_detector_state.time.time", return_value=100.0):
            sink.update([_target(7, timestamp=100.0)])
        self.assertEqual(g.updated, [None])

    def test_duplicate_timestamp_is_fed_once(self):
        # Coast ticks re-deliver the SAME cached target; feeding it twice as a
        # fresh measurement would drag the Kalman rate estimate to zero. The
        # second delivery must be a true no-op, not a synthetic loss tick.
        g = _Navigation(tracking_obj_id=7)
        sink = self._sink(g)
        fresh = _target(7, timestamp=99.99)
        with patch("navpy.modules.vision.real_detector_state.time.time", return_value=100.0):
            sink.update([fresh])   # delivered
            sink.update([fresh])   # same ts -> no navigation call
        self.assertEqual(g.updated, [fresh])

    def test_failed_delivery_can_retry_exact_timestamp(self):
        g = _Navigation(tracking_obj_id=7)
        g.fail_next = True
        sink = self._sink(g)
        fresh = _target(7, timestamp=99.99)

        with patch(
            "navpy.modules.vision.real_detector_state.time.time",
            return_value=100.0,
        ):
            with self.assertRaisesRegex(RuntimeError, "actuation failed"):
                sink.update([fresh])
            sink.update([fresh])

        self.assertEqual(g.updated, [fresh])

    def test_none_timestamp_target_is_never_deduped(self):
        # A target without a timestamp can't be dedup-keyed; it must always be
        # delivered (the dedupe guard is `timestamp is not None`).
        g = _Navigation(tracking_obj_id=7)
        sink = self._sink(g)
        with patch("navpy.modules.vision.real_detector_state.time.time", return_value=100.0):
            sink.update([_target(7, timestamp=None)])
            sink.update([_target(7, timestamp=None)])
        self.assertEqual(len(g.updated), 2)
        self.assertTrue(all(u is not None for u in g.updated))

    def test_real_detected_target_uses_grouped_source_timestamp(self):
        g = _Navigation(tracking_obj_id=7)
        sink = self._sink(g)
        fresh = make_detected_target(obj_id=7, timestamp=99.99)

        with patch(
            "navpy.modules.vision.real_detector_state.time.time",
            return_value=100.0,
        ):
            sink.update([fresh])
            sink.update([fresh])

        self.assertEqual(g.updated, [fresh])


if __name__ == "__main__":
    unittest.main()
