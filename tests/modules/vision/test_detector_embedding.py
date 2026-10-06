"""On-demand appearance embedding: computed only when a lock is active and only
for the locked class — a handful of crops, not every box every frame."""
import unittest
from unittest.mock import Mock

import numpy as np

from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_state import RuntimeMetrics
from navpy.modules.vision.real_tracking import TrackingRecovery


def _track(obj_id, cls=0, cx=100.0):
    return TrackedObject(
        id=obj_id, cx=cx, cy=100.0, w=20.0, h=20.0, confidence=0.9, class_id=cls,
        age=5, hits=5, missed=0, is_confirmed=True, timestamp=0.0, vx=0.0, vy=0.0,
    )


class _FakeEmbedder:
    def __init__(self):
        self.calls = 0

    def embed(self, frame, boxes):
        self.calls += 1
        return np.ones((len(boxes), 2), dtype=np.float32)


class TestComputeEmbeddings(unittest.TestCase):
    def _recovery(self, appearance):
        metrics = RuntimeMetrics()
        # Embedding now runs through the async wrapper; sync=True makes it
        # deterministic (computes inline on submit) for the test.
        wrapped = (
            AsyncAppearanceEmbedder(appearance, sync=True,
                                    on_batch=lambda n: metrics.bump("embeddings", n))
            if appearance is not None else None
        )
        return (
            TrackingRecovery(Mock(), wrapped, Mock(), Mock(), True, metrics),
            metrics,
        )

    def test_no_embedder_returns_none(self):
        recovery, _ = self._recovery(None)
        self.assertIsNone(recovery.compute_embeddings([_track(1)], np.zeros((4, 4, 3), np.uint8)))

    def test_embeds_all_tracks_scene_wide_without_a_lock(self):
        # Scene-wide id stability: every raw track is embedded even when no lock
        # is active, so the identity layer can re-bind ANY object by appearance.
        emb = _FakeEmbedder()
        recovery, metrics = self._recovery(emb)
        tracks = [_track(1, cls=0), _track(2, cls=0), _track(3, cls=4)]
        out = recovery.compute_embeddings(tracks, np.zeros((4, 4, 3), np.uint8))
        self.assertEqual(set(out.keys()), {1, 2, 3})   # all classes, keyed by raw id
        self.assertEqual(metrics.snapshot()["embeddings"], 3)

    def test_empty_tracks_returns_none(self):
        emb = _FakeEmbedder()
        recovery, _ = self._recovery(emb)
        self.assertIsNone(recovery.compute_embeddings([], np.zeros((4, 4, 3), np.uint8)))
        self.assertEqual(emb.calls, 0)

    def test_embed_failure_is_swallowed(self):
        class _Boom:
            def embed(self, frame, boxes):
                raise RuntimeError("boom")
        recovery, _ = self._recovery(_Boom())
        self.assertIsNone(recovery.compute_embeddings([_track(1)], np.zeros((4, 4, 3), np.uint8)))


if __name__ == "__main__":
    unittest.main()
