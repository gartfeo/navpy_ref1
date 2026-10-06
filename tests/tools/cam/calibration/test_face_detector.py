"""Unit tests for FaceDetector (YOLO-based)."""

import sys
from pathlib import Path
from unittest.mock import patch, MagicMock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src"))

from cam.calibration.face_detector import FaceDetector, _DEFAULT_MODEL
from cam.calibration.detector_abc import DetectorAbc
from navpy.modules.vision.detector import Detection


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_detector_with_mock_yolo():
    """Create a FaceDetector with a mocked YoloDetector (no model file needed)."""
    with patch("cam.calibration.face_detector.YoloDetector") as MockYolo:
        mock_yolo = MagicMock()
        mock_yolo.detect.return_value = []
        MockYolo.return_value = mock_yolo
        det = FaceDetector(model_path="dummy.pt")
    det._yolo = mock_yolo
    return det


def _make_detection(cx, cy, w, h, conf=0.9, cls=0):
    return Detection(cx=cx, cy=cy, w=w, h=h, confidence=conf, class_id=cls)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFaceDetectorInterface:
    """Verify FaceDetector implements DetectorAbc."""

    def test_is_subclass(self):
        assert issubclass(FaceDetector, DetectorAbc)

    def test_default_model_path_points_to_face_model(self):
        assert _DEFAULT_MODEL.name == "yolov8n-face-lindevs.pt"


class TestFaceDetectorOnBlank:
    """No face in frame -> returns None."""

    def test_detect_returns_none_when_no_detections(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = []
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        assert det.detect(frame) is None

    def test_detect_bounds_returns_none_when_no_detections(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = []
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        assert det.detect_bounds(frame) is None


class TestFaceDetectorReturnTypes:
    """Verify return value contracts using mocked YOLO detections."""

    def test_detect_returns_center(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = [
            _make_detection(cx=320.0, cy=240.0, w=100.0, h=120.0),
        ]
        result = det.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        assert result is not None
        cx, cy = result
        assert cx == 320.0
        assert cy == 240.0

    def test_detect_bounds_returns_half_sizes(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = [
            _make_detection(cx=320.0, cy=240.0, w=100.0, h=120.0),
        ]
        result = det.detect_bounds(np.zeros((480, 640, 3), dtype=np.uint8))
        assert result is not None
        half_w, half_h = result
        assert half_w == 50.0   # 100/2
        assert half_h == 60.0   # 120/2

    def test_detect_none_when_no_face(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = []
        assert det.detect(np.zeros((480, 640, 3), dtype=np.uint8)) is None

    def test_detect_bounds_none_when_no_face(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = []
        assert det.detect_bounds(np.zeros((480, 640, 3), dtype=np.uint8)) is None


class TestFaceDetectorSelection:
    """Verify highest-confidence face is selected."""

    def test_selects_highest_confidence(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = [
            _make_detection(cx=100.0, cy=100.0, w=50.0, h=50.0, conf=0.4),
            _make_detection(cx=400.0, cy=300.0, w=120.0, h=140.0, conf=0.95),
            _make_detection(cx=500.0, cy=100.0, w=80.0, h=80.0, conf=0.6),
        ]
        result = det.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        assert result is not None
        cx, cy = result
        assert cx == 400.0
        assert cy == 300.0

    def test_bounds_from_highest_confidence(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = [
            _make_detection(cx=100.0, cy=100.0, w=50.0, h=50.0, conf=0.4),
            _make_detection(cx=400.0, cy=300.0, w=120.0, h=140.0, conf=0.95),
        ]
        result = det.detect_bounds(np.zeros((480, 640, 3), dtype=np.uint8))
        assert result is not None
        half_w, half_h = result
        assert half_w == 60.0   # 120/2
        assert half_h == 70.0   # 140/2

    def test_single_detection_selected(self):
        det = _make_detector_with_mock_yolo()
        det._yolo.detect.return_value = [
            _make_detection(cx=250.0, cy=180.0, w=90.0, h=110.0, conf=0.7),
        ]
        result = det.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        assert result == (250.0, 180.0)


class TestFaceDetectorWithModel:
    """Integration tests that require the actual YOLO model file."""

    @pytest.fixture
    def detector(self):
        pytest.importorskip("ultralytics")
        if not _DEFAULT_MODEL.exists():
            pytest.skip(f"YOLO model not found: {_DEFAULT_MODEL}")
        return FaceDetector()

    def test_blank_frame_returns_none(self, detector):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        assert detector.detect(frame) is None

    def test_detect_returns_tuple_or_none(self, detector):
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        result = detector.detect(frame)
        assert result is None or (isinstance(result, tuple) and len(result) == 2)

    def test_detect_bounds_returns_tuple_or_none(self, detector):
        frame = np.full((480, 640, 3), 128, dtype=np.uint8)
        result = detector.detect_bounds(frame)
        assert result is None or (isinstance(result, tuple) and len(result) == 2)
