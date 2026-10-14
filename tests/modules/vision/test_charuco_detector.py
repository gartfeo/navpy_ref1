"""Mechanism tests for the default ChArUco board frame detector.

Frames are synthetic: the board is rendered by OpenCV at a known pixel scale
(1 px per mm, so the 250 x 350 mm board is 250 x 350 px) and pasted or warped
into a white frame at a known pose, so the expected outline box is exact.
"""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from navpy.plugins.detection.charuco.charuco_board import (
    CHARUCO_DOCK_CLASS_ID,
    CHARUCO_MIN_MARKER_PERIMETER_RATE,
    CHARUCO_PIXELS_PER_CELL,
    CHARUCO_REFINE_MARKERS,
    CharucoBoardSpec,
    CharucoDetectionSettings,
    charuco_board_spec_from_settings,
    charuco_detection_settings_from_settings,
)
from navpy.plugins.detection.charuco.charuco_detector import (
    MIN_CHARUCO_CORNERS,
    CharucoBoardDetector,
    build_charuco_board,
    build_charuco_detector,
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


# A far, foreshortened board as the ZR10 at zoom 1 sees the Unreal dock's board at about 40 m slant and 35 deg
# depression (outline with its white margin, NW NE SE SW, px): its markers' outlines are about 30 px, under
# OpenCV's default minimum of 3 % of 1280 px.
FAR_OUTLINE = np.array([[630.1, 338.6], [711.8, 338.6], [714.6, 403.0], [627.3, 403.0]])
_FAR_SQUARE_PX = 64        # texture pixels per square
_FAR_SUPERSAMPLE = 4       # rendered at 4x, then area-averaged: the pixel footprint of a camera


def _far_texture() -> np.ndarray:
    margin = _FAR_SQUARE_PX // 2
    size = (5 * _FAR_SQUARE_PX + 2 * margin, 7 * _FAR_SQUARE_PX + 2 * margin)
    return build_charuco_board(CharucoBoardSpec()).generateImage(size, marginSize=margin)


def _far_frame(scale: float, blur_px: float) -> tuple[np.ndarray, np.ndarray]:
    """A grey 1280 x 720 frame with the board at FAR_OUTLINE scaled about its centre, and the homography from
    texture pixels to frame pixels (the truth)."""
    texture = _far_texture()
    th, tw = texture.shape
    centre = FAR_OUTLINE.mean(axis=0)
    quad = np.float32(centre + (FAR_OUTLINE - centre) * scale)
    corners = np.float32([[0, 0], [tw, 0], [tw, th], [0, th]])
    ss = _FAR_SUPERSAMPLE
    canvas = cv2.warpPerspective(
        texture.astype(np.float32), cv2.getPerspectiveTransform(corners, (quad + 0.5) * ss - 0.5),
        (FRAME_W * ss, FRAME_H * ss), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=-1,
    )
    image = np.where(canvas >= 0, 40 + canvas * 175 / 255, 110.0)
    image = cv2.resize(image, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
    if blur_px:
        image = cv2.GaussianBlur(image, (0, 0), blur_px)
    return np.clip(image, 0, 255).astype(np.uint8), cv2.getPerspectiveTransform(corners, quad)


def _corners(detector, gray) -> tuple[np.ndarray, np.ndarray]:
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None:
        return np.zeros((0, 2)), np.zeros(0, int)
    return np.asarray(corners).reshape(-1, 2), np.asarray(ids).reshape(-1)


def _truth_corner_error_px(ids: np.ndarray, corners: np.ndarray, truth: np.ndarray) -> float:
    board_mm = np.asarray(build_charuco_board(CharucoBoardSpec()).getChessboardCorners())[:, :2]
    margin = _FAR_SQUARE_PX // 2
    texture_px = margin + board_mm[ids] / CharucoBoardSpec().square_mm * _FAR_SQUARE_PX
    expected = cv2.perspectiveTransform(texture_px.reshape(-1, 1, 2).astype(np.float32), truth).reshape(-1, 2)
    return float(np.linalg.norm(expected - corners, axis=1).max())


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


def test_detection_settings_defaults_are_the_long_range_settings():
    settings = CharucoDetectionSettings()

    assert settings.min_marker_perimeter_rate == CHARUCO_MIN_MARKER_PERIMETER_RATE == 0.015
    assert settings.pixels_per_cell == CHARUCO_PIXELS_PER_CELL == 8
    assert settings.refine_markers is CHARUCO_REFINE_MARKERS is True


def test_detection_settings_reach_the_opencv_detector():
    settings = CharucoDetectionSettings(
        min_marker_perimeter_rate=0.02, pixels_per_cell=6, refine_markers=False,
    )

    detector = build_charuco_detector(build_charuco_board(CharucoBoardSpec()), settings)

    marker = detector.getDetectorParameters()
    assert marker.minMarkerPerimeterRate == pytest.approx(0.02)
    assert marker.perspectiveRemovePixelPerCell == 6
    assert detector.getCharucoParameters().tryRefineMarkers is False
    # everything else stays OpenCV's default
    default = cv2.aruco.DetectorParameters()
    assert marker.cornerRefinementMethod == default.cornerRefinementMethod
    assert marker.errorCorrectionRate == pytest.approx(default.errorCorrectionRate)
    assert detector.getCharucoParameters().minMarkers == cv2.aruco.CharucoParameters().minMarkers


def test_far_board_is_found_only_with_the_long_range_settings():
    """At about 40 m on the ZR10 at zoom 1, OpenCV's default marker size floor drops every marker."""
    frame, truth = _far_frame(1.0, 0.0)
    board = build_charuco_board(CharucoBoardSpec())

    _, opencv_ids = _corners(cv2.aruco.CharucoDetector(board), frame)
    corners, ids = _corners(build_charuco_detector(board), frame)

    assert len(opencv_ids) == 0
    assert len(ids) >= 6
    square_px = (FAR_OUTLINE[1, 0] - FAR_OUTLINE[0, 0]) / 6
    assert _truth_corner_error_px(ids, corners, truth) < 0.25 * square_px
    detections = CharucoBoardDetector(CharucoBoardSpec()).detect(frame)
    assert len(detections) == 1
    assert detections[0].cx == pytest.approx(FAR_OUTLINE[:, 0].mean(), abs=2.0)
    assert detections[0].cy == pytest.approx(FAR_OUTLINE[:, 1].mean(), abs=2.0)


def test_blurred_board_needs_marker_refinement():
    """A soft frame (lens and compression blur) keeps a few markers readable; the board's layout recovers the
    rest."""
    frame, truth = _far_frame(1.6, 0.8)
    board = build_charuco_board(CharucoBoardSpec())
    without = build_charuco_detector(board, CharucoDetectionSettings(refine_markers=False))

    _, unrefined = _corners(without, frame)
    corners, ids = _corners(build_charuco_detector(board), frame)

    assert len(unrefined) < MIN_CHARUCO_CORNERS
    assert len(ids) >= 12
    square_px = 1.6 * (FAR_OUTLINE[1, 0] - FAR_OUTLINE[0, 0]) / 6
    assert _truth_corner_error_px(ids, corners, truth) < 0.25 * square_px


@pytest.mark.parametrize(
    "frame",
    [
        cv2.GaussianBlur(np.random.default_rng(3).integers(0, 256, (FRAME_H, FRAME_W), dtype=np.uint8), (0, 0), 1.0),
        np.kron((np.indices((FRAME_H // 40, FRAME_W // 40)).sum(axis=0) % 2 * 255).astype(np.uint8),
                np.ones((40, 40), np.uint8)),
        np.kron((np.indices((FRAME_H // 9, FRAME_W // 9)).sum(axis=0) % 2 * 255).astype(np.uint8),
                np.ones((9, 9), np.uint8)),
    ],
    ids=["soft-noise", "checkerboard-40px", "checkerboard-9px"],
)
def test_long_range_settings_find_no_board_without_one(frame):
    assert CharucoBoardDetector(CharucoBoardSpec()).detect(frame) == []


def test_profile_charuco_detection_block_overrides_settings():
    settings = charuco_detection_settings_from_settings({
        "charuco_detection": {"min_marker_perimeter_rate": 0.03, "pixels_per_cell": 4,
                              "refine_markers": False},
    })

    assert settings == CharucoDetectionSettings(
        min_marker_perimeter_rate=0.03, pixels_per_cell=4, refine_markers=False,
    )
    assert charuco_detection_settings_from_settings({}) == CharucoDetectionSettings()


@pytest.mark.parametrize(
    "block, message",
    [
        ({"min_perimeter": 0.01}, "Unknown detector.charuco_detection"),
        ({"min_marker_perimeter_rate": 0.0}, "in \\(0, 1\\)"),
        ({"pixels_per_cell": 0}, "at least 1"),
        ({"pixels_per_cell": 8.9}, "must be an integer"),
        ({"pixels_per_cell": True}, "must be an integer"),
        ({"min_marker_perimeter_rate": "0.01"}, "must be a number"),
        ({"refine_markers": "yes"}, "true or false"),
        ([0.01], "must be a mapping"),
    ],
)
def test_invalid_profile_charuco_detection_block_is_rejected(block, message):
    with pytest.raises(ValueError, match=message):
        charuco_detection_settings_from_settings({"charuco_detection": block})
