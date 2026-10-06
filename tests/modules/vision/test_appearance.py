import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.vision import appearance
from navpy.modules.vision.appearance import (
    BoxmotReidEmbedder,
    cosine_distance,
    create_appearance_embedder,
    l2_normalize,
)


class _FakeBackend:
    def __init__(self):
        self.calls = []

    def get_features(self, boxes, frame):
        self.calls.append((np.asarray(boxes), frame))
        # return non-normalised features, one row per box
        return np.asarray([[3.0, 4.0]] * len(boxes), dtype=np.float32)


class TestMath(unittest.TestCase):
    def test_cosine_distance_identical_is_zero(self):
        v = np.array([1.0, 0.0], dtype=np.float32)
        self.assertAlmostEqual(cosine_distance(v, v), 0.0)

    def test_cosine_distance_orthogonal_is_one(self):
        a = np.array([1.0, 0.0], dtype=np.float32)
        b = np.array([0.0, 1.0], dtype=np.float32)
        self.assertAlmostEqual(cosine_distance(a, b), 1.0)

    def test_cosine_distance_handles_none_and_mismatch(self):
        self.assertEqual(cosine_distance(None, np.array([1.0])), 1.0)
        self.assertEqual(cosine_distance(np.array([1.0, 0.0]), np.array([1.0])), 1.0)

    def test_l2_normalize_rows_are_unit(self):
        out = l2_normalize(np.array([[3.0, 4.0], [0.0, 2.0]], dtype=np.float32))
        np.testing.assert_allclose(np.linalg.norm(out, axis=1), [1.0, 1.0], atol=1e-6)


class TestEmbedder(unittest.TestCase):
    def _embedder(self):
        with patch.object(BoxmotReidEmbedder, "_build_backend", return_value=_FakeBackend()):
            return BoxmotReidEmbedder(reid_device="cpu", detector_device="cpu")

    def test_embed_returns_l2_normalised_rows(self):
        emb = self._embedder()
        out = emb.embed(np.zeros((10, 10, 3), np.uint8), [[0, 0, 4, 8], [1, 1, 5, 9]])
        self.assertEqual(out.shape, (2, 2))
        np.testing.assert_allclose(np.linalg.norm(out, axis=1), [1.0, 1.0], atol=1e-6)

    def test_embed_empty_boxes_returns_empty(self):
        emb = self._embedder()
        out = emb.embed(np.zeros((10, 10, 3), np.uint8), [])
        self.assertEqual(out.shape[0], 0)

    def test_close_is_safe(self):
        emb = self._embedder()
        emb.close()
        self.assertIsNone(emb._backend)


class TestFactory(unittest.TestCase):
    def test_none_or_disabled_returns_none(self):
        self.assertIsNone(create_appearance_embedder(None))
        self.assertIsNone(create_appearance_embedder({"enabled": False}))

    def test_enabled_builds_embedder(self):
        with patch.object(BoxmotReidEmbedder, "_build_backend", return_value=_FakeBackend()):
            emb = create_appearance_embedder({"enabled": True, "reid_device": "cpu"},
                                             detector_device="cpu")
        self.assertIsInstance(emb, BoxmotReidEmbedder)


class TestEmbeddingProvenance(unittest.TestCase):
    """A served embedding must still describe the CURRENT track (Codex
    blocker): bbox provenance + an age bound, so a stalled batch cannot vouch
    for a raw id whose box moved or whose object was swapped."""

    def _async(self, **kw):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
        backend = _FakeBackend()

        class _Emb:
            def embed(self, frame, boxes):
                return backend.get_features(boxes, frame)

            def close(self):
                pass

        clock = {"t": 0.0}
        emb = AsyncAppearanceEmbedder(_Emb(), sync=True,
                                      clock=lambda: clock["t"], **kw)
        return emb, clock

    def test_same_box_is_served(self):
        emb, clock = self._async()
        frame = np.zeros((100, 100, 3), np.uint8)
        emb.submit(frame, [(10, 10, 50, 50)], [1])
        out = emb.latest({1: (10, 10, 50, 50)})
        self.assertIn(1, out)

    def test_moved_box_is_not_served(self):
        # the raw id now covers a different region: the old crop's embedding
        # must not vouch for it
        emb, clock = self._async()
        frame = np.zeros((100, 100, 3), np.uint8)
        emb.submit(frame, [(10, 10, 30, 30)], [1])
        out = emb.latest({1: (60, 60, 90, 90)})   # zero overlap
        self.assertNotIn(1, out)

    def test_stale_batch_is_not_served(self):
        emb, clock = self._async(max_age=0.3)
        frame = np.zeros((100, 100, 3), np.uint8)
        emb.submit(frame, [(10, 10, 50, 50)], [1])
        clock["t"] = 0.5   # batch is now older than max_age
        out = emb.latest({1: (10, 10, 50, 50)})
        self.assertEqual(out, {})

    def test_unknown_key_is_not_served(self):
        emb, clock = self._async()
        frame = np.zeros((100, 100, 3), np.uint8)
        emb.submit(frame, [(10, 10, 50, 50)], [1])
        out = emb.latest({2: (10, 10, 50, 50)})
        self.assertEqual(out, {})


if __name__ == "__main__":
    unittest.main()


class TestAsyncAppearanceEmbedder(unittest.TestCase):
    def _fake(self):
        class _F:
            def __init__(s): s.calls = 0; s.closed = False
            def embed(s, frame, boxes):
                s.calls += 1
                return np.ones((len(boxes), 2), dtype=np.float32)
            def close(s): s.closed = True
        return _F()

    def test_sync_mode_computes_inline(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
        f = self._fake()
        counts = []
        a = AsyncAppearanceEmbedder(f, sync=True, on_batch=counts.append)
        a.submit(np.zeros((4, 4, 3), np.uint8), [[0, 0, 2, 2], [1, 1, 3, 3]], [7, 9])
        self.assertEqual(set(a.latest().keys()), {7, 9})
        self.assertEqual(counts, [2])
        a.close()
        self.assertTrue(f.closed)

    def test_async_worker_eventually_produces_latest(self):
        import time
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
        f = self._fake()
        a = AsyncAppearanceEmbedder(f)            # real worker thread
        self.assertEqual(a.latest(), {})          # nothing yet -> non-blocking
        a.submit(np.zeros((4, 4, 3), np.uint8), [[0, 0, 2, 2]], [5])
        deadline = time.time() + 2.0
        while time.time() < deadline and 5 not in a.latest():
            time.sleep(0.01)
        self.assertIn(5, a.latest())
        a.close()

    def test_embed_failure_is_swallowed(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
        class _Boom:
            def embed(self, frame, boxes): raise RuntimeError("boom")
            def close(self): pass
        a = AsyncAppearanceEmbedder(_Boom(), sync=True)
        a.submit(np.zeros((4, 4, 3), np.uint8), [[0, 0, 2, 2]], [1])
        self.assertEqual(a.latest(), {})          # no crash, no result
        a.close()

    def test_async_submit_after_close_does_not_restart_inline(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        backend = self._fake()
        embedder = AsyncAppearanceEmbedder(backend)
        embedder.close()

        embedder.submit(
            np.zeros((4, 4, 3), np.uint8),
            [[0, 0, 2, 2]],
            [1],
        )

        self.assertEqual(backend.calls, 0)

    def test_close_waits_for_inflight_embed_before_backend_close(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        class _BlockingBackend:
            def __init__(s):
                s.entered = threading.Event()
                s.release = threading.Event()
                s.exited = threading.Event()
                s.close_calls = 0
                s.closed_while_embedding = False

            def embed(s, frame, boxes):
                s.entered.set()
                s.release.wait(timeout=2.0)
                s.exited.set()
                return np.ones((len(boxes), 2), dtype=np.float32)

            def close(s):
                s.close_calls += 1
                s.closed_while_embedding = not s.exited.is_set()

        backend = _BlockingBackend()
        batches = []
        embedder = AsyncAppearanceEmbedder(backend, on_batch=batches.append)
        embedder.submit(
            np.zeros((4, 4, 3), np.uint8),
            [[0, 0, 2, 2]],
            [1],
        )
        self.assertTrue(backend.entered.wait(timeout=1.0))
        closed = threading.Event()
        closer = threading.Thread(target=lambda: (embedder.close(), closed.set()))
        closer.start()

        self.assertFalse(closed.wait(timeout=0.05))
        self.assertEqual(backend.close_calls, 0)
        self.assertIsNotNone(embedder._worker)
        backend.release.set()
        closer.join(timeout=2.0)

        self.assertFalse(closer.is_alive())
        self.assertEqual(backend.close_calls, 1)
        self.assertFalse(backend.closed_while_embedding)
        self.assertEqual(batches, [1])
        self.assertEqual(set(embedder.latest()), {1})

    def test_submit_after_close_is_noop_in_both_modes(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        for sync in (False, True):
            with self.subTest(sync=sync):
                backend = self._fake()
                embedder = AsyncAppearanceEmbedder(backend, sync=sync)
                embedder.close()
                embedder.submit(
                    np.zeros((4, 4, 3), np.uint8),
                    [[0, 0, 2, 2]],
                    [1],
                )
                self.assertEqual(backend.calls, 0)

    def test_close_is_idempotent(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        class _CountingBackend:
            def __init__(s):
                s.close_calls = 0

            def embed(s, frame, boxes):
                return np.ones((len(boxes), 2), dtype=np.float32)

            def close(s):
                s.close_calls += 1

        backend = _CountingBackend()
        embedder = AsyncAppearanceEmbedder(backend, sync=True)
        embedder.close()
        embedder.close()
        self.assertEqual(backend.close_calls, 1)

    def test_warning_logger_failure_cannot_orphan_worker(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        class _BlockingBackend:
            def __init__(s):
                s.entered = threading.Event()
                s.release = threading.Event()
                s.close_calls = 0

            def embed(s, frame, boxes):
                s.entered.set()
                s.release.wait(timeout=2.0)
                return np.ones((len(boxes), 2), dtype=np.float32)

            def close(s):
                s.close_calls += 1

        logger = Mock()
        logger.warning.side_effect = RuntimeError("logger failed")
        backend = _BlockingBackend()
        embedder = AsyncAppearanceEmbedder(backend, logger=logger)
        embedder.submit(
            np.zeros((4, 4, 3), np.uint8),
            [[0, 0, 2, 2]],
            [1],
        )
        self.assertTrue(backend.entered.wait(timeout=1.0))
        started_s = time.perf_counter()
        with patch(
            "navpy.modules.vision.appearance_worker._WORKER_WARNING_S",
            0.01,
        ):
            with self.assertRaisesRegex(TimeoutError, "cleanup remains owned"):
                embedder.close()
        self.assertLess(time.perf_counter() - started_s, 0.2)
        self.assertIsNotNone(embedder._worker)
        self.assertEqual(backend.close_calls, 0)

        backend.release.set()
        embedder._worker._thread.join(timeout=2.0)
        embedder.close()

        self.assertIsNone(embedder._worker)
        self.assertEqual(backend.close_calls, 1)
        logger.warning.assert_called_once_with(
            "Appearance worker did not stop within timeout; "
            "retaining ownership for retry"
        )

    def test_cleanup_error_is_replayed_without_double_close(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        backend = self._fake()
        backend.close = Mock(side_effect=RuntimeError("close failed"))
        embedder = AsyncAppearanceEmbedder(backend, sync=True)
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, "close failed"):
                embedder.close()
        backend.close.assert_called_once_with()

    def test_worker_start_failure_closes_acquired_backend(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        backend = self._fake()
        with patch(
            "navpy.modules.vision.appearance_worker.threading.Thread.start",
            side_effect=RuntimeError("thread start failed"),
        ):
            with self.assertRaisesRegex(RuntimeError, "thread start failed"):
                AsyncAppearanceEmbedder(backend)

        self.assertTrue(backend.closed)

    def test_callback_can_close_without_orphaning_worker_or_backend(self):
        from navpy.modules.vision.appearance import AsyncAppearanceEmbedder

        class _ReentrantBackend:
            def __init__(s):
                s.close_calls = 0

            def embed(s, frame, boxes):
                return np.ones((len(boxes), 2), dtype=np.float32)

            def close(s):
                s.close_calls += 1

        backend = _ReentrantBackend()
        callback_returned = threading.Event()
        callback_errors = []
        owner = {}

        def close_from_callback(_count):
            try:
                owner["embedder"].close()
            except BaseException as error:
                callback_errors.append(error)
            finally:
                callback_returned.set()

        embedder = AsyncAppearanceEmbedder(
            backend,
            on_batch=close_from_callback,
        )
        owner["embedder"] = embedder
        embedder.submit(
            np.zeros((4, 4, 3), np.uint8),
            [[0, 0, 2, 2]],
            [1],
        )

        self.assertTrue(callback_returned.wait(timeout=1.0))
        embedder.close()
        self.assertEqual(callback_errors, [])
        self.assertEqual(backend.close_calls, 1)


def _capture_error(action, errors):
    try:
        action()
    except BaseException as error:
        errors.append(error)
