import threading
import unittest
from types import SimpleNamespace

from navpy.modules.vision.poi_lock import PoiLock


def _track(
    track_id: int,
    cx: float,
    cy: float,
    *,
    w: float = 200.0,
    h: float = 300.0,
    class_id: int = 0,
    confidence: float = 0.9,
    timestamp: float = 1.0,
    vx: float = 0.0,
    vy: float = 0.0,
    is_confirmed: bool = True,
):
    return SimpleNamespace(
        id=track_id, cx=cx, cy=cy, w=w, h=h, confidence=confidence,
        class_id=class_id, age=5, hits=5, missed=0, is_confirmed=is_confirmed,
        timestamp=timestamp, vx=vx, vy=vy,
    )


class TestAutoLock(unittest.TestCase):
    def test_selects_center_poi_when_unlocked(self):
        lock = PoiLock(max_lost_frames=3)
        selected = lock.select([_track(1, 200, 540), _track(2, 960, 540)], 1920, 1080)
        self.assertEqual(selected.id, 2)
        self.assertEqual(lock.locked_id, 2)

    def test_auto_lock_can_be_disabled(self):
        lock = PoiLock(max_lost_frames=3, auto_lock=False)
        self.assertIsNone(lock.select([_track(2, 960, 540)], 1920, 1080))
        self.assertIsNone(lock.locked_id)

    def test_keeps_exact_locked_id(self):
        lock = PoiLock(max_lost_frames=3)
        lock.select([_track(4, 900, 500)], 1920, 1080)
        moved = _track(4, 1200, 520, vx=400, timestamp=1.1)
        selected = lock.select([_track(5, 960, 540), moved], 1920, 1080)
        self.assertEqual(selected.id, 4)
        self.assertEqual(lock.locked_id, 4)


class TestFailClosed(unittest.TestCase):
    def test_does_not_reacquire_a_different_id(self):
        # The identity layer owns continuity. If our exact id is gone, the lock
        # stays lost rather than jumping to a nearby same-class track.
        lock = PoiLock(max_lost_frames=5)
        lock.select([_track(4, 900, 500)], 1920, 1080)
        # id 4 absent; a same-class track at the predicted spot appears as id 9.
        selected = lock.select([_track(9, 905, 500)], 1920, 1080)
        self.assertIsNone(selected)
        self.assertEqual(lock.locked_id, 4)  # still ours, just lost

    def test_reclaims_when_identity_layer_reissues_the_id(self):
        lock = PoiLock(max_lost_frames=5)
        lock.select([_track(4, 900, 500)], 1920, 1080)
        self.assertIsNone(lock.select([_track(9, 905, 500)], 1920, 1080))
        # identity layer re-binds the reappearing object to stable id 4
        selected = lock.select([_track(4, 905, 500)], 1920, 1080)
        self.assertEqual(selected.id, 4)
        self.assertEqual(lock.lost, 0)

    def test_resets_and_auto_locks_after_loss_budget_expires(self):
        lock = PoiLock(max_lost_frames=1)
        lock.select([_track(4, 1400, 600)], 1920, 1080)
        far = _track(8, 100, 540)
        self.assertIsNone(lock.select([far], 1920, 1080))   # lost=1
        # budget exceeded -> reset -> fall through to auto-lock in same call
        selected = lock.select([far], 1920, 1080)
        self.assertEqual(selected.id, 8)
        self.assertEqual(lock.locked_id, 8)


class TestForceLock(unittest.TestCase):
    def test_force_lock_commits_on_confirmation(self):
        lock = PoiLock(max_lost_frames=3, auto_lock=False)
        lock.force_lock(2)
        self.assertTrue(lock.has_pending_force)
        self.assertIsNone(lock.locked_id)  # no phantom lock before confirm
        selected = lock.select([_track(2, 960, 540)], 1920, 1080)
        self.assertEqual(selected.id, 2)
        self.assertEqual(lock.locked_id, 2)

    def test_force_lock_waits_for_unconfirmed_track(self):
        lock = PoiLock(max_lost_frames=3, auto_lock=False)
        lock.force_lock(2)
        # track present but not yet confirmed -> keep waiting, don't commit
        self.assertIsNone(lock.select([_track(2, 960, 540, is_confirmed=False)], 1920, 1080))
        self.assertTrue(lock.has_pending_force)
        selected = lock.select([_track(2, 960, 540, is_confirmed=True)], 1920, 1080)
        self.assertEqual(selected.id, 2)

    def test_force_lock_overrides_current(self):
        lock = PoiLock(max_lost_frames=3)
        t1, t2 = _track(1, 960, 540), _track(2, 200, 200)
        lock.select([t1, t2], 1920, 1080)
        self.assertEqual(lock.locked_id, 1)
        lock.force_lock(2)
        selected = lock.select([t1, t2], 1920, 1080)
        self.assertEqual(selected.id, 2)

    def test_force_lock_to_nonexistent_expires_then_auto_locks(self):
        lock = PoiLock(max_lost_frames=3)
        t1 = _track(1, 960, 540)
        lock.force_lock(99)
        self.assertIsNone(lock.select([t1], 1920, 1080))  # waiting for id 99
        lock._force_ttl = 0
        self.assertIsNone(lock.select([t1], 1920, 1080))  # grace expires, force cleared
        # no phantom lock left behind -> next call auto-locks the center POI
        self.assertEqual(lock.select([t1], 1920, 1080).id, 1)


class TestForceBbox(unittest.TestCase):
    def test_prefers_high_iou_over_tiny_center_inside_track(self):
        # F21: a strongly-overlapping POI must beat a tiny track that merely
        # has its centre inside an inflated click box.
        lock = PoiLock(max_lost_frames=3, auto_lock=False)
        big_overlap = _track(2, 905, 505, w=130, h=100)        # high IoU, centre just off
        tiny_inside = _track(3, 900, 500, w=8, h=8)            # centre inside, tiny IoU
        lock.force_lock_bbox((900, 500, 140, 110))
        selected = lock.select([tiny_inside, big_overlap], 1920, 1080)
        self.assertEqual(selected.id, 2)

    def test_waits_for_unconfirmed_clicked_track(self):
        # F22: clicking a tentative track must not consume the click and drift.
        lock = PoiLock(max_lost_frames=3, auto_lock=False)
        clicked = _track(2, 900, 500, w=120, h=90, is_confirmed=False)
        lock.force_lock_bbox((900, 500, 140, 110))
        self.assertIsNone(lock.select([clicked], 1920, 1080))
        self.assertTrue(lock.has_pending_force)
        confirmed = _track(2, 900, 500, w=120, h=90, is_confirmed=True)
        self.assertEqual(lock.select([confirmed], 1920, 1080).id, 2)


class TestThreadSafety(unittest.TestCase):
    def test_concurrent_force_and_select_do_not_corrupt_state(self):
        lock = PoiLock(max_lost_frames=3)
        tracks = [_track(i, 100 * i, 200) for i in range(1, 6)]
        stop = threading.Event()

        def hammer_force():
            while not stop.is_set():
                lock.force_lock(3)
                lock.force_lock_bbox((300, 200, 50, 50))

        def hammer_select():
            for _ in range(2000):
                lock.select(tracks, 1920, 1080)

        forcer = threading.Thread(target=hammer_force, daemon=True)
        forcer.start()
        hammer_select()
        stop.set()
        forcer.join(timeout=1.0)
        # No assertion on the exact id (it races by design); the point is that
        # no torn read raised and the object is still usable.
        self.assertIn(lock.locked_id, {None, 1, 2, 3, 4, 5})


if __name__ == "__main__":
    unittest.main()
