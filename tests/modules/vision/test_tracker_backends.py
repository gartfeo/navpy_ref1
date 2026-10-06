import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.vision.track_export_memory import _TrackExportMemory
from navpy.modules.vision.tracker_backends import (
    BotSortTrackerBackend,
    CustomTrackerBackend,
    StrongSortTrackerBackend,
    create_tracker_backend,
    tracker_config_from_settings,
)
from navpy.modules.vision.yolo_detector import Detection


# ---------------------------------------------------------------------------
# Faithful fakes: their __init__ signatures mirror the REAL libraries
# (boxmot 19/21) so that passing an unsupported kwarg raises
# TypeError here instead of being silently swallowed by **kwargs. This is what
# makes the conversion/kwarg contract actually tested.
# ---------------------------------------------------------------------------


class _FakeReIDBackend:
    """Stand-in for the boxmot ReID *backend* (the object with get_features)."""

    def get_features(self, xyxys, img):  # pragma: no cover - not exercised by fakes
        return np.zeros((len(xyxys), 4), dtype=np.float32)


class _FakeReID:
    # Mirrors boxmot.reid.core.ReID.__init__ (keyword-only after path).
    def __init__(self, path=None, *, weights=None, device='cpu', half=False,
                 preprocess_name=None):
        self.kwargs = dict(path=path, weights=weights, device=device, half=half)
        self._backend = _FakeReIDBackend()

    def get_backend(self):
        return self._backend


class _FakeBotSort:
    # Mirrors boxmot.trackers.bbox.botsort.botsort.BotSort.__init__ EXACTLY
    # (no **kwargs) so an unsupported kwarg raises TypeError.
    def __init__(self, reid_model=None, track_high_thresh=0.5, track_low_thresh=0.1,
                 new_track_thresh=0.6, track_buffer=30, match_thresh=0.8,
                 proximity_thresh=0.5, appearance_thresh=0.25, cmc_method='ecc',
                 frame_rate=30, fuse_first_associate=False, with_reid=True):
        self.kwargs = dict(
            reid_model=reid_model, track_high_thresh=track_high_thresh,
            track_low_thresh=track_low_thresh, new_track_thresh=new_track_thresh,
            track_buffer=track_buffer, match_thresh=match_thresh,
            proximity_thresh=proximity_thresh, appearance_thresh=appearance_thresh,
            cmc_method=cmc_method, frame_rate=frame_rate,
            fuse_first_associate=fuse_first_associate, with_reid=with_reid,
        )
        self.dets = None
        self.frame = None
        self.reset_called = False

    def update(self, dets, frame):
        self.dets = dets
        self.frame = frame
        return np.array([[10.0, 20.0, 50.0, 60.0, 9.0, 0.85, 2.0, 0.0]], dtype=np.float32)

    def reset(self):
        self.reset_called = True


class _FakeStrongSort:
    # Mirrors boxmot.trackers.bbox.strongsort.strongsort.StrongSort.__init__
    def __init__(self, reid_model=None, min_conf=0.1, max_cos_dist=0.2,
                 max_iou_dist=0.7, n_init=3, nn_budget=100, mc_lambda=0.98,
                 ema_alpha=0.9, **kwargs):
        self.kwargs = dict(
            reid_model=reid_model, min_conf=min_conf, max_cos_dist=max_cos_dist,
            max_iou_dist=max_iou_dist, n_init=n_init, nn_budget=nn_budget,
            mc_lambda=mc_lambda, ema_alpha=ema_alpha, **kwargs,
        )
        self.dets = None
        self.frame = None
        self.embs = None
        self.reset_called = False
        self.cmc = object()

    def update(self, dets, frame, embs=None, masks=None):
        self.dets = dets
        self.frame = frame
        self.embs = embs
        return np.array([[10.0, 20.0, 50.0, 60.0, 11.0, 0.88, 2.0, 0.0]], dtype=np.float32)

    def reset(self):
        self.reset_called = True


def _patch_botsort():
    return patch(
        "navpy.modules.vision.tracker_backends_boxmot._load_botsort_deps",
        return_value=(_FakeReID, _FakeBotSort),
    )


def _patch_strongsort():
    return patch(
        "navpy.modules.vision.tracker_backends_boxmot._load_strongsort_deps",
        return_value=(_FakeReID, _FakeStrongSort),
    )


class TestTrackerBackendConfig(unittest.TestCase):
    def test_tracker_config_defaults_to_custom(self):
        config = tracker_config_from_settings(None)
        self.assertEqual(config.runtime.backend, "custom")
        self.assertEqual(config.custom.max_age, 60)
        self.assertEqual(config.reid.cmc_method, "sof")
        self.assertTrue(config.reid.enabled)

    def test_factory_creates_custom_backend(self):
        tracker = create_tracker_backend({"backend": "custom"}, detector_device="cpu")
        self.assertIsInstance(tracker, CustomTrackerBackend)
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        tracks = tracker.update([Detection(20, 30, 10, 8, 0.8, 2)], 100, 100, frame=frame)
        self.assertEqual(len(tracks), 1)

    def test_unknown_backend_is_rejected(self):
        with self.assertRaises(ValueError):
            create_tracker_backend({"backend": "unknown"}, detector_device="cpu")

    def test_thresholds_derive_from_detector_conf(self):
        # The profile lowers detector conf to 0.2; BoT-SORT must treat 0.2 as a
        # track-starting confidence, not the library's 0.6 default (F04).
        config = tracker_config_from_settings({"backend": "botsort"}, detector_conf=0.2)
        th = config.association.resolved_botsort_thresholds()
        self.assertAlmostEqual(th["track_high_thresh"], 0.2)
        self.assertAlmostEqual(th["new_track_thresh"], 0.2)
        self.assertLessEqual(th["track_low_thresh"], 0.2)

    def test_explicit_thresholds_override_derivation(self):
        config = tracker_config_from_settings(
            {"backend": "botsort", "new_track_thresh": 0.4, "track_high_thresh": 0.5},
            detector_conf=0.2,
        )
        th = config.association.resolved_botsort_thresholds()
        self.assertAlmostEqual(th["new_track_thresh"], 0.4)
        self.assertAlmostEqual(th["track_high_thresh"], 0.5)


class TestStrongSortTrackerBackend(unittest.TestCase):
    def _make(self, **settings):
        settings.setdefault("backend", "strongsort")
        settings.setdefault("min_hits", 1)
        settings.setdefault("reid_device", "cpu")
        settings.setdefault("cmc_method", None)
        with _patch_strongsort():
            return StrongSortTrackerBackend(
                tracker_config_from_settings(settings, detector_conf=0.2),
                detector_device="cpu",
            )

    def test_passes_reid_backend_and_settings(self):
        tracker = self._make(appearance_thresh=0.19, match_thresh=0.71, max_age=90)
        kwargs = tracker._tracker.kwargs
        self.assertIsInstance(kwargs["reid_model"], _FakeReIDBackend)
        self.assertAlmostEqual(kwargs["min_conf"], 0.2)
        self.assertAlmostEqual(kwargs["max_cos_dist"], 0.19)
        self.assertAlmostEqual(kwargs["max_iou_dist"], 0.71)
        self.assertEqual(kwargs["n_init"], 1)
        self.assertEqual(kwargs["max_age"], 90)

    def test_reid_is_required(self):
        with self.assertRaises(ValueError):
            self._make(with_reid=False)

    def test_disables_cmc_when_configured_none(self):
        tracker = self._make()
        warp = tracker._tracker.cmc.apply(np.zeros((10, 10, 3), dtype=np.uint8),
                                          np.empty((0, 4), dtype=np.float32))
        np.testing.assert_array_equal(warp, np.eye(2, 3, dtype=np.float32))

    def test_converts_detection_to_xyxy_input(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)
        row = [float(v) for v in tracker._tracker.dets[0]]
        # cx=30,cy=40,w=20,h=10 -> x1=20,y1=35,x2=40,y2=45,conf=0.8,cls=2
        self.assertEqual(row[:4], [20.0, 35.0, 40.0, 45.0])
        self.assertAlmostEqual(row[4], 0.8, places=5)
        self.assertEqual(row[5], 2.0)

    def test_converts_output_to_tracked_object(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        tracks = tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].id, 11)
        self.assertEqual(tracks[0].class_id, 2)
        self.assertTrue(tracks[0].is_confirmed)
        self.assertAlmostEqual(tracks[0].cx, 30.0)

    def test_requires_frame(self):
        tracker = self._make()
        with self.assertRaises(ValueError):
            tracker.update([], 100, 100, frame=None)

    def test_coasts_without_calling_reid_when_no_detections(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        first = tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)[0]
        tracker._tracker.dets = "unchanged"
        coasted = tracker.update([], 100, 100, frame=frame)
        self.assertEqual(tracker._tracker.dets, "unchanged")
        self.assertEqual(len(coasted), 1)
        self.assertEqual(coasted[0].id, first.id)
        self.assertGreater(coasted[0].missed, first.missed)

    def test_close_releases_tracker_and_reid(self):
        tracker = self._make()
        tracker.close()
        self.assertIsNone(tracker._tracker)
        self.assertIsNone(tracker._reid)

    def test_reset_after_close_does_not_rebuild(self):
        tracker = self._make()
        tracker.close()
        tracker.reset()
        self.assertIsNone(tracker._tracker)


class TestBotSortTrackerBackend(unittest.TestCase):
    def _make(self, **settings):
        settings.setdefault("backend", "botsort")
        settings.setdefault("reid_device", "cpu")
        with _patch_botsort():
            return BotSortTrackerBackend(
                tracker_config_from_settings(settings, detector_conf=0.2),
                detector_device="cpu",
            )

    def test_passes_only_supported_kwargs_and_derived_thresholds(self):
        tracker = self._make()
        kwargs = tracker._tracker.kwargs
        # conf-derived (detector_conf=0.2)
        self.assertAlmostEqual(kwargs["new_track_thresh"], 0.2)
        self.assertAlmostEqual(kwargs["track_high_thresh"], 0.2)
        self.assertEqual(kwargs["track_buffer"], 60)
        self.assertEqual(kwargs["cmc_method"], "sof")
        self.assertTrue(kwargs["with_reid"])

    def test_reid_model_is_the_backend_not_the_runtime(self):
        # Regression: the wrapper must pass ReID().get_backend() (which has
        # get_features), NOT the ReID runtime or a non-existent .model attr.
        tracker = self._make()
        self.assertIsInstance(tracker._tracker.kwargs["reid_model"], _FakeReIDBackend)

    def test_converts_detection_to_xyxy_input(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)
        row = [float(v) for v in tracker._tracker.dets[0]]
        # cx=30,cy=40,w=20,h=10 -> x1=20,y1=35,x2=40,y2=45,conf=0.8,cls=2
        # (BoT-SORT input is float32, so conf is compared with tolerance.)
        self.assertEqual(row[:4], [20.0, 35.0, 40.0, 45.0])
        self.assertAlmostEqual(row[4], 0.8, places=5)
        self.assertEqual(row[5], 2.0)

    def test_converts_output_to_tracked_object(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        tracks = tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].id, 9)
        self.assertEqual(tracks[0].class_id, 2)
        self.assertAlmostEqual(tracks[0].cx, 30.0)

    def test_cmc_method_can_be_configured(self):
        tracker = self._make(cmc_method="ecc")
        self.assertEqual(tracker._tracker.kwargs["cmc_method"], "ecc")

    def test_reid_can_be_disabled(self):
        tracker = self._make(with_reid=False)
        self.assertFalse(tracker._tracker.kwargs["with_reid"])
        self.assertIsNone(tracker._tracker.kwargs["reid_model"])
        self.assertIsNone(tracker._reid)

    def test_requires_frame(self):
        tracker = self._make()
        with self.assertRaises(ValueError):
            tracker.update([], 100, 100, frame=None)

    def test_coasts_without_calling_tracker_when_no_detections(self):
        tracker = self._make()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        first = tracker.update([Detection(30, 40, 20, 10, 0.8, 2)], 100, 100, frame=frame)[0]
        tracker._tracker.dets = "unchanged"
        coasted = tracker.update([], 100, 100, frame=frame)
        self.assertEqual(tracker._tracker.dets, "unchanged")
        self.assertEqual(coasted[0].id, first.id)
        self.assertGreater(coasted[0].missed, first.missed)

    def test_close_releases_tracker_and_reid(self):
        tracker = self._make()
        tracker.close()
        self.assertIsNone(tracker._tracker)
        self.assertIsNone(tracker._reid)

    def test_reset_after_close_does_not_rebuild(self):
        tracker = self._make()
        tracker.close()
        tracker.reset()                       # must not resurrect GPU resources
        self.assertIsNone(tracker._tracker)


class TestTrackExportMemory(unittest.TestCase):
    def _box(self, cx, w=20.0):
        return (cx - w / 2, 90.0, cx + w / 2, 110.0)  # x1,y1,x2,y2 around cy=100

    def test_velocity_from_motion_and_coast_extrapolation(self):
        m = _TrackExportMemory(max_age_seconds=1.0)
        m.export(1, *self._box(100), 0.9, 0, True, 0, 0.0)
        t = m.export(1, *self._box(120), 0.9, 0, True, 0, 0.5)
        self.assertAlmostEqual(t.vx, 40.0)   # (120-100)/0.5
        coasted = m.coast(0.75, max_age_seconds=2.0)
        self.assertEqual(len(coasted), 1)
        self.assertAlmostEqual(coasted[0].cx, 130.0)  # 120 + 40*0.25
        self.assertGreater(coasted[0].missed, t.missed)

    def test_non_positive_dt_keeps_previous_velocity(self):
        # F05: a zero/negative clock step must not synthesize a huge velocity.
        m = _TrackExportMemory(max_age_seconds=1.0)
        m.export(1, *self._box(100), 0.9, 0, True, 0, 1.0)
        t2 = m.export(1, *self._box(120), 0.9, 0, True, 0, 1.0)  # same timestamp
        self.assertEqual(t2.vx, 0.0)

    def test_prune_evicts_after_max_age(self):
        m = _TrackExportMemory(max_age_seconds=1.0)
        m.export(1, *self._box(100), 0.9, 0, True, 0, 0.0)
        m.prune(set(), 2.0)   # 2.0 - 0.0 > max_age
        self.assertEqual(m.coast(2.0, max_age_seconds=10.0), [])


@unittest.skipUnless(
    os.environ.get("NAVPY_BOXMOT_INTEGRATION") == "1",
    "set NAVPY_BOXMOT_INTEGRATION=1 to run the real boxmot construction test",
)
class TestRealLibraryConstruction(unittest.TestCase):
    """Opt-in: construct the REAL backends to prove kwargs/wiring are valid.

    Downloads ReID weights and may need a GPU; off by default so unit tests
    stay deterministic and offline.
    """

    def test_real_botsort_constructs_and_runs_one_frame(self):
        tracker = BotSortTrackerBackend(
            tracker_config_from_settings({"backend": "botsort", "reid_device": "cpu"},
                                         detector_conf=0.2),
            detector_device="cpu",
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        out = tracker.update([Detection(320, 240, 40, 60, 0.9, 0)], 640, 480, frame=frame)
        self.assertIsInstance(out, list)
        tracker.close()

    def test_real_strongsort_constructs_and_runs_one_frame(self):
        tracker = StrongSortTrackerBackend(
            tracker_config_from_settings(
                {"backend": "strongsort", "reid_device": "cpu", "cmc_method": None},
                detector_conf=0.2,
            ),
            detector_device="cpu",
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        out = tracker.update([Detection(320, 240, 40, 60, 0.9, 0)], 640, 480, frame=frame)
        self.assertIsInstance(out, list)
        tracker.close()


if __name__ == "__main__":
    unittest.main()
