import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.vision.yolo_detector import YoloDetector


class _FakeYOLO:
    def __init__(self, model_path):
        self.model_path = model_path


def _torch_with_cuda(available):
    return SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: bool(available)),
    )


class TestYoloDetectorConfiguration(unittest.TestCase):
    def test_auto_device_uses_cuda_half_when_available(self):
        logger = Mock()
        modules = {
            "ultralytics": SimpleNamespace(YOLO=_FakeYOLO),
            "torch": _torch_with_cuda(True),
        }

        with patch.dict(sys.modules, modules):
            detector = YoloDetector("model.pt", device="auto", logger=logger)

        self.assertEqual(detector.device, 0)
        self.assertTrue(detector.half)
        logger.info.assert_called_once()
        logger.warning.assert_not_called()

    def test_auto_device_warns_when_only_cpu_available(self):
        logger = Mock()
        modules = {
            "ultralytics": SimpleNamespace(YOLO=_FakeYOLO),
            "torch": _torch_with_cuda(False),
        }

        with patch.dict(sys.modules, modules):
            detector = YoloDetector("model.pt", device="auto", logger=logger)

        self.assertEqual(detector.device, "cpu")
        self.assertFalse(detector.half)
        logger.info.assert_called_once()
        logger.warning.assert_called_once()

    def test_explicit_cuda_falls_back_to_cpu_when_unavailable(self):
        logger = Mock()
        modules = {
            "ultralytics": SimpleNamespace(YOLO=_FakeYOLO),
            "torch": _torch_with_cuda(False),
        }

        with patch.dict(sys.modules, modules):
            detector = YoloDetector("model.pt", device="cuda", logger=logger)

        self.assertEqual(detector.device, "cpu")
        self.assertFalse(detector.half)
        logger.info.assert_called_once()
        logger.error.assert_called_once()
        logger.warning.assert_not_called()

    def test_detect_converts_boxes_from_vectorized_arrays(self):
        model = _PredictingYOLO(
            boxes=_FakeBoxes(
                xyxy=np.asarray([[10.0, 20.0, 30.0, 50.0], [1.0, 2.0, 5.0, 8.0]]),
                conf=np.asarray([0.75, 0.25]),
                cls=np.asarray([2.0, 7.0]),
            )
        )

        detector = YoloDetector.__new__(YoloDetector)
        detector.model = model
        detector.imgsz = 640
        detector.conf = 0.2
        detector.classes = [2]
        detector.device = 0
        detector.half = True
        detector._logger = Mock()

        dets = detector.detect(np.zeros((10, 10, 3), dtype=np.uint8))

        self.assertEqual(len(dets), 2)
        self.assertEqual(model.predict_kwargs["half"], True)
        self.assertAlmostEqual(dets[0].cx, 20.0)
        self.assertAlmostEqual(dets[0].cy, 35.0)
        self.assertAlmostEqual(dets[0].w, 20.0)
        self.assertAlmostEqual(dets[0].h, 30.0)
        self.assertAlmostEqual(dets[0].confidence, 0.75)
        self.assertEqual(dets[0].class_id, 2)
        self.assertEqual(dets[1].class_id, 7)

    def test_detect_retries_fp32_when_fp16_fails(self):
        model = _FailingHalfYOLO(
            boxes=_FakeBoxes(
                xyxy=np.asarray([[10.0, 20.0, 30.0, 50.0]]),
                conf=np.asarray([0.75]),
                cls=np.asarray([2.0]),
            )
        )
        logger = Mock()
        detector = YoloDetector.__new__(YoloDetector)
        detector.model = model
        detector.imgsz = 640
        detector.conf = 0.2
        detector.classes = [2]
        detector.device = 0
        detector.half = True
        detector._logger = logger

        dets = detector.detect(np.zeros((10, 10, 3), dtype=np.uint8))

        self.assertEqual(len(dets), 1)
        self.assertFalse(detector.half)
        self.assertEqual(model.half_values, [True, False])
        logger.warning.assert_called_once()


class TestYoloDetectorEdges(unittest.TestCase):
    def _detector(self, model):
        detector = YoloDetector.__new__(YoloDetector)
        detector.model = model
        detector.imgsz = 640
        detector.conf = 0.2
        detector.classes = None
        detector.device = 0
        detector.half = True
        detector._logger = Mock()
        return detector

    def test_none_boxes_returns_empty(self):
        detector = self._detector(_PredictingYOLO(boxes=None))
        self.assertEqual(detector.detect(np.zeros((10, 10, 3), np.uint8)), [])

    def test_empty_boxes_returns_empty(self):
        empty = _FakeBoxes(
            xyxy=np.empty((0, 4)), conf=np.empty((0,)), cls=np.empty((0,)),
        )
        detector = self._detector(_PredictingYOLO(boxes=empty))
        self.assertEqual(detector.detect(np.zeros((10, 10, 3), np.uint8)), [])

    def test_non_half_error_is_not_swallowed(self):
        # A CUDA-OOM-like error (no half/fp16/dtype marker) must propagate, not
        # silently drop to FP32 forever (F29).
        class _OOMYOLO(_PredictingYOLO):
            def predict(self, **kwargs):
                raise RuntimeError("CUDA out of memory")
        detector = self._detector(_OOMYOLO(boxes=None))
        with self.assertRaises(RuntimeError):
            detector.detect(np.zeros((10, 10, 3), np.uint8))
        self.assertTrue(detector.half)  # not downgraded by an unrelated error

    def test_fp32_fallback_is_permanent(self):
        model = _FailingHalfYOLO(
            boxes=_FakeBoxes(xyxy=np.asarray([[1.0, 2.0, 3.0, 4.0]]),
                             conf=np.asarray([0.5]), cls=np.asarray([0.0])),
        )
        detector = self._detector(model)
        detector.detect(np.zeros((10, 10, 3), np.uint8))   # fails half, retries fp32
        detector.detect(np.zeros((10, 10, 3), np.uint8))   # must start at fp32
        self.assertFalse(detector.half)
        self.assertEqual(model.half_values, [True, False, False])
        detector._logger.warning.assert_called_once()      # only one downgrade log


class _FakeBoxes:
    def __init__(self, *, xyxy, conf, cls):
        self.xyxy = xyxy
        self.conf = conf
        self.cls = cls

    def __iter__(self):
        raise AssertionError("detect should read vectorized box arrays, not iterate boxes")


class _PredictingYOLO:
    def __init__(self, *, boxes):
        self._boxes = boxes
        self.predict_kwargs = None

    def predict(self, **kwargs):
        self.predict_kwargs = kwargs
        return [SimpleNamespace(boxes=self._boxes)]


class _FailingHalfYOLO(_PredictingYOLO):
    def __init__(self, *, boxes):
        super().__init__(boxes=boxes)
        self.half_values = []

    def predict(self, **kwargs):
        self.half_values.append(bool(kwargs.get("half")))
        if kwargs.get("half"):
            raise RuntimeError("fp16 unsupported")
        return super().predict(**kwargs)


if __name__ == "__main__":
    unittest.main()
