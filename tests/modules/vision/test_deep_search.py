import unittest
from unittest.mock import patch

from navpy.modules.vision.deep_search import (
    DeepSearchConfig,
    DeepSearchDetector,
    deep_search_config_from_settings,
    merge_deep_search_detections,
)
from navpy.modules.vision.yolo_detector import Detection


class _FakeYolo:
    instances = []

    def __init__(self, model_path, *, imgsz, conf, device, classes, logger=None):
        self.kwargs = dict(model_path=model_path, imgsz=imgsz, conf=conf,
                           device=device, classes=classes)
        _FakeYolo.instances.append(self)

    def detect(self, frame):
        return [Detection(1, 1, 1, 1, 0.5, 0)]


class TestDeepSearchConfig(unittest.TestCase):
    def test_disabled_settings_return_none(self):
        self.assertIsNone(deep_search_config_from_settings(None))
        self.assertIsNone(deep_search_config_from_settings({"enabled": False}))

    def test_enabled_settings_parse_budget(self):
        config = deep_search_config_from_settings({
            "enabled": True,
            "hz": 3,
            "imgsz": 1280,
            "conf": 0.1,
            "stale_seconds": 0.7,
            "device": 0,
            "classes": [2],
        })

        self.assertIsInstance(config, DeepSearchConfig)
        self.assertAlmostEqual(config.period, 1.0 / 3.0)
        self.assertEqual(config.imgsz, 1280)
        self.assertEqual(config.classes, [2])


class TestDeepSearchMerge(unittest.TestCase):
    def test_deep_detection_not_overlapping_fast_detection_is_appended(self):
        base = [Detection(100, 100, 40, 20, 0.8, 2)]
        deep = [Detection(300, 100, 40, 20, 0.2, 2)]

        merged = merge_deep_search_detections(base, deep)

        self.assertEqual(merged, base + deep)

    def test_duplicate_keeps_higher_confidence_box(self):
        # F30: a low-confidence deep box must NOT clobber a confident fast box.
        base = [Detection(100, 100, 40, 20, 0.8, 2)]
        deep = [Detection(102, 101, 40, 20, 0.2, 2)]

        merged = merge_deep_search_detections(base, deep)

        self.assertEqual(merged, base)

    def test_more_confident_deep_box_replaces_fast_box(self):
        base = [Detection(100, 100, 40, 20, 0.3, 2)]
        deep = [Detection(102, 101, 40, 20, 0.9, 2)]

        merged = merge_deep_search_detections(base, deep)

        self.assertEqual(merged, deep)

    def test_different_class_is_not_considered_duplicate(self):
        base = [Detection(100, 100, 40, 20, 0.8, 2)]
        deep = [Detection(102, 101, 40, 20, 0.2, 0)]

        merged = merge_deep_search_detections(base, deep)

        self.assertEqual(merged, base + deep)

    def test_iou_at_duplicate_threshold_is_a_duplicate(self):
        # Two equal 30x20 boxes offset by w/3 have IoU == 0.5 exactly; with the
        # default duplicate_iou=0.5 (>=) they merge, keeping the higher-conf box.
        base = [Detection(100, 100, 30, 20, 0.8, 2)]
        deep = [Detection(110, 100, 30, 20, 0.2, 2)]
        merged = merge_deep_search_detections(base, deep)
        self.assertEqual(len(merged), 1)
        self.assertEqual(merged, base)


class TestDeepSearchDetector(unittest.TestCase):
    def setUp(self):
        _FakeYolo.instances = []

    def _build(self, config, **kwargs):
        kwargs.setdefault("default_model_path", "base.pt")
        kwargs.setdefault("default_device", 0)
        kwargs.setdefault("default_classes", [2])
        with patch("navpy.modules.vision.deep_search.YoloDetector", _FakeYolo):
            return DeepSearchDetector(config, **kwargs)

    def test_auto_device_inherits_detector_device(self):
        det = self._build(DeepSearchConfig(device="auto"))
        self.assertEqual(_FakeYolo.instances[0].kwargs["device"], 0)
        self.assertEqual(det.detect(None)[0].class_id, 0)

    def test_explicit_device_and_model_override_defaults(self):
        cfg = DeepSearchConfig(device="cpu", model_path="deep.pt", classes=[5])
        self._build(cfg)
        kw = _FakeYolo.instances[0].kwargs
        self.assertEqual(kw["device"], "cpu")
        self.assertEqual(kw["model_path"], "deep.pt")
        self.assertEqual(kw["classes"], [5])
        self.assertEqual(kw["imgsz"], 960)


if __name__ == "__main__":
    unittest.main()
