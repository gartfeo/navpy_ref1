"""Tests for camer_calibration3 image selection (no camera required)."""

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "tools"))
from cam.calibration import camer_calibration3 as calib

COLS, ROWS = 7, 5
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def _checkerboard(width: int, height: int, square_px: int = 40) -> np.ndarray:
    """A COLSxROWS-inner-corner board centred on a white frame of the given size."""
    img = np.full((height, width), 255, dtype=np.uint8)
    x0 = (width - (COLS + 1) * square_px) // 2
    y0 = (height - (ROWS + 1) * square_px) // 2
    for r in range(ROWS + 1):
        for c in range(COLS + 1):
            if (r + c) % 2 == 1:
                ys, xs = y0 + r * square_px, x0 + c * square_px
                img[ys:ys + square_px, xs:xs + square_px] = 0
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def _write(path: Path, img: np.ndarray) -> str:
    cv2.imwrite(str(path), img)
    return str(path)


def test_list_images_includes_png_and_jpg(tmp_path):
    board = _checkerboard(640, 480)
    _write(tmp_path / "a.png", board)
    _write(tmp_path / "b.JPG", board)
    (tmp_path / "intrinsics.json").write_text("{}")

    names = [Path(p).name for p in calib._list_images(tmp_path)]

    assert names == ["a.png", "b.JPG"]


def test_cropped_image_is_excluded_from_fit_even_when_listed_first(tmp_path):
    # The crop sorts first; the calibration size must still be the full frame,
    # and the crop must not be fitted although its corners are detectable.
    images = [_write(tmp_path / "0_crop.png", _checkerboard(500, 480))]
    images += [_write(tmp_path / f"{i}_full.png", _checkerboard(640, 480)) for i in range(1, 4)]
    objp = calib._build_objp(COLS, ROWS, 30.0)

    objpoints, imgpoints, img_shape = calib._collect_detections(
        images, (COLS, ROWS), objp, CRITERIA
    )

    assert img_shape == (640, 480)
    assert len(imgpoints) == 3
    assert len(objpoints) == 3
