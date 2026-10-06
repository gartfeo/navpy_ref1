"""Mechanism tests for the default ChArUco board frame detector.

Frames are synthetic: the board is rendered by OpenCV at a known pixel scale
(1 px per mm, so the 250 x 350 mm board is 250 x 350 px) and pasted or warped
into a white frame at a known pose, so the expected outline box is exact.
"""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from navpy.modules.vision.charuco_board import (
    CHARUCO_DOCK_CLASS_ID,
    CharucoBoardSpec,
    charuco_board_spec_from_settings,
)
from navpy.modules.vision.charuco_detector import (
    MIN_CHARUCO_CORNERS,
    CharucoBoardDetector,
    build_charuco_board,
)

FRAME_W, FRAME_H = 1280, 960
BOX_TOLERANCE_PX = 3.0


def _board_image(spec: CharucoBoardSpec = CharucoBoardSpec()) -> np.ndarray:
    width_mm, height_mm = spec.size_mm
    return build_charuco_board(spec).generateImage(
        (int(width_mm), int(height_mm)), marginSize=0,
    )


def _pasted_frame(x0: int, y0: int, board: np.ndarray) -> np.ndarray:
    frame = np.full((FRAME_H, FRAME_W, 3), 255, np.uint8)
    h, w = board.shape[:2]
    frame[y0:y0 + h, x0:x0 + w] = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    return frame


def _box(detection) -> np.ndarray:
    return detection.xyxy.astype(np.float64)


def test_board_spec_defaults_are_the_a3_bench_board():
    spec = CharucoBoardSpec()

    assert spec.dictionary == "DICT_4X4_50"
    assert (spec.squares_x, spec.squares_y) == (5, 7)
    assert (spec.square_mm, spec.marker_mm) == (50.0, 37.0)
    assert spec.interior_corner_count == 24
    assert spec.size_mm == (250.0, 350.0)
    assert CHARUCO_DOCK_CLASS_ID == 0


def test_fronto_parallel_board_yields_one_full_confidence_dock_box():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    frame = _pasted_frame(400, 200, _board_image())

    detections = detector.detect(frame)

    assert len(detections) == 1
    detection = detections[0]
    assert detection.class_id == CHARUCO_DOCK_CLASS_ID
    assert detection.confidence == pytest.approx(1.0)
    np.testing.assert_allclose(
        _box(detection), [400, 200, 650, 550], atol=BOX_TOLERANCE_PX,
    )


def test_perspective_board_box_encloses_projected_outline():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    board = _board_image()
    outline = np.float32([[0, 0], [250, 0], [250, 350], [0, 350]])
    image_quad = np.float32([[500, 150], [820, 210], [780, 700], [460, 620]])
    homography = cv2.getPerspectiveTransform(outline, image_quad)
    warped = cv2.warpPerspective(
        board, homography, (FRAME_W, FRAME_H),
        flags=cv2.INTER_LINEAR, borderValue=255,
    )
    frame = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)

    detections = detector.detect(frame)

    assert len(detections) == 1
    np.testing.assert_allclose(
        _box(detections[0]), [460, 150, 820, 700], atol=BOX_TOLERANCE_PX,
    )
    assert detections[0].confidence == pytest.approx(1.0)


def test_partially_occluded_board_lowers_confidence_but_keeps_board_extent():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    frame = _pasted_frame(400, 200, _board_image())
    # Grey out the lower half of the board (board rows below y = 175 mm).
    frame[200 + 175:200 + 350, 400:650] = 128

    detections = detector.detect(frame)

    assert len(detections) == 1
    detection = detections[0]
    assert MIN_CHARUCO_CORNERS / 24 <= detection.confidence < 1.0
    # The homography extrapolates the whole physical board outline.
    np.testing.assert_allclose(
        _box(detection), [400, 200, 650, 550], atol=BOX_TOLERANCE_PX,
    )


def test_board_outline_is_clipped_to_the_frame():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    board = _board_image()
    frame = np.full((FRAME_H, FRAME_W, 3), 255, np.uint8)
    # Board extends 100 px beyond the right edge of the frame.
    x0 = FRAME_W - 150
    frame[300:650, x0:FRAME_W] = cv2.cvtColor(board[:, :150], cv2.COLOR_GRAY2BGR)

    detections = detector.detect(frame)

    assert len(detections) == 1
    np.testing.assert_allclose(
        _box(detections[0]), [x0, 300, FRAME_W, 650], atol=BOX_TOLERANCE_PX,
    )


def test_min_confidence_rejects_partial_board():
    frame = _pasted_frame(400, 200, _board_image())
    frame[200 + 175:200 + 350, 400:650] = 128
    partial = CharucoBoardDetector(CharucoBoardSpec()).detect(frame)[0]

    strict = CharucoBoardDetector(
        CharucoBoardSpec(), min_confidence=partial.confidence + 0.01,
    )

    assert strict.detect(frame) == []


@pytest.mark.parametrize(
    "frame",
    [
        np.full((FRAME_H, FRAME_W, 3), 255, np.uint8),
        np.zeros((FRAME_H, FRAME_W), np.uint8),
        np.random.default_rng(7).integers(
            0, 256, (FRAME_H, FRAME_W, 3), dtype=np.uint8,
        ),
    ],
    ids=["white", "black-gray", "noise"],
)
def test_frame_without_board_yields_no_detections(frame):
    assert CharucoBoardDetector(CharucoBoardSpec()).detect(frame) == []


def test_board_from_another_dictionary_is_not_detected():
    other = CharucoBoardSpec(dictionary="DICT_5X5_50")
    frame = _pasted_frame(400, 200, _board_image(other))

    assert CharucoBoardDetector(CharucoBoardSpec()).detect(frame) == []


def test_grayscale_and_bgra_frames_are_accepted():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    bgr = _pasted_frame(300, 300, _board_image())

    gray = detector.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    bgra = detector.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA))

    assert len(gray) == 1 and len(bgra) == 1
    np.testing.assert_allclose(_box(gray[0]), _box(bgra[0]), atol=1e-3)


def test_detector_runs_on_cpu_and_refuses_detection_after_close():
    detector = CharucoBoardDetector(CharucoBoardSpec())
    assert detector.device == "cpu"

    detector.close()
    detector.close()

    with pytest.raises(RuntimeError, match="closed"):
        detector.detect(np.zeros((10, 10), np.uint8))


def test_profile_charuco_block_overrides_board_geometry():
    spec = charuco_board_spec_from_settings({
        "conf": 0.35,
        "charuco": {"squares_x": 4, "squares_y": 6, "square_mm": 40.0,
                    "marker_mm": 30.0, "dictionary": "DICT_4X4_100"},
    })

    assert spec == CharucoBoardSpec(
        dictionary="DICT_4X4_100", squares_x=4, squares_y=6,
        square_mm=40.0, marker_mm=30.0,
    )
    assert charuco_board_spec_from_settings({}) == CharucoBoardSpec()


@pytest.mark.parametrize(
    "block, message",
    [
        ({"square_size": 50}, "Unknown detector.charuco"),
        ({"marker_mm": 60.0}, "smaller than square_mm"),
        ({"squares_x": 1}, "at least 2 x 2"),
        ("DICT_4X4_50", "must be a mapping"),
    ],
)
def test_invalid_profile_charuco_block_is_rejected(block, message):
    with pytest.raises(ValueError, match=message):
        charuco_board_spec_from_settings({"charuco": block})


def test_unknown_aruco_dictionary_is_rejected():
    with pytest.raises(ValueError, match="Unknown ArUco dictionary"):
        CharucoBoardDetector(CharucoBoardSpec(dictionary="DICT_FACES"))
