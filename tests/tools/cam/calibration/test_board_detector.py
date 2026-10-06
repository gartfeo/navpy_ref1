"""Tests for BoardDetector detection logic."""

import cv2
import numpy as np

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
from cam.calibration.board_detector import BoardDetector


def _make_checkerboard_image(cols: int, rows: int, square_px: int = 40,
                             margin: int = 80) -> np.ndarray:
    """Create a synthetic checkerboard BGR image."""
    board_w = (cols + 1) * square_px
    board_h = (rows + 1) * square_px
    img_w = board_w + 2 * margin
    img_h = board_h + 2 * margin

    img = np.full((img_h, img_w), 255, dtype=np.uint8)

    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 1:
                x0 = margin + c * square_px
                y0 = margin + r * square_px
                img[y0:y0 + square_px, x0:x0 + square_px] = 0

    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def _jpeg_compress(bgr: np.ndarray, quality: int = 20) -> np.ndarray:
    """Compress and decompress via JPEG to simulate compression artifacts."""
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


class TestDetectClean:
    """Detection on a clean synthetic checkerboard."""

    def test_detects_8x6_checkerboard(self):
        cols, rows = 8, 6
        img = _make_checkerboard_image(cols, rows)
        det = BoardDetector(cols, rows)
        center = det.detect(img)
        assert center is not None

    def test_detect_corners_returns_full_array(self):
        cols, rows = 8, 6
        img = _make_checkerboard_image(cols, rows)
        det = BoardDetector(cols, rows)
        corners = det.detect_corners(img)
        assert corners is not None
        assert len(corners) == cols * rows

    def test_detects_flipped_orientation(self):
        """Detector should find the board when cols/rows are swapped."""
        cols, rows = 8, 6
        img = _make_checkerboard_image(cols, rows)
        det = BoardDetector(rows, cols)
        center = det.detect(img)
        assert center is not None

    def test_returns_none_on_blank(self):
        det = BoardDetector(8, 6)
        blank = np.full((480, 640, 3), 128, dtype=np.uint8)
        center = det.detect(blank)
        assert center is None

    def test_detect_corners_returns_none_on_blank(self):
        det = BoardDetector(8, 6)
        blank = np.full((480, 640, 3), 128, dtype=np.uint8)
        corners = det.detect_corners(blank)
        assert corners is None


class TestDetectCompressed:
    """Detection on JPEG-compressed checkerboard."""

    def test_detects_jpeg_compressed_checkerboard(self):
        cols, rows = 8, 6
        img = _make_checkerboard_image(cols, rows, square_px=50, margin=100)
        compressed = _jpeg_compress(img, quality=20)
        det = BoardDetector(cols, rows)
        center = det.detect(compressed)
        assert center is not None
        corners = det.detect_corners(compressed)
        assert corners is not None
        assert len(corners) == cols * rows

    def test_detects_heavily_compressed_checkerboard(self):
        cols, rows = 8, 6
        img = _make_checkerboard_image(cols, rows, square_px=50, margin=100)
        compressed = _jpeg_compress(img, quality=5)
        det = BoardDetector(cols, rows)
        corners = det.detect_corners(compressed)
        assert corners is not None
        assert len(corners) == cols * rows


class TestCornerAccuracy:
    """Verify cornerSubPix produces sub-pixel accurate corners."""

    def test_center_near_image_center(self):
        cols, rows = 8, 6
        sq = 40
        margin = 80
        img = _make_checkerboard_image(cols, rows, sq, margin)
        det = BoardDetector(cols, rows)
        center = det.detect(img)
        assert center is not None

        img_h, img_w = img.shape[:2]
        # Board is centered in the image; center should be near image center
        assert abs(center[0] - img_w / 2) < sq
        assert abs(center[1] - img_h / 2) < sq
