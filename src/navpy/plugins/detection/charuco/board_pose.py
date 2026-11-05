"""ChArUco board pose: OpenCV ``solvePnP`` on the known board (HITL step 22).

The board's interior corners are object points in metres, centred on the
board, so the solved translation is the board centre in the camera frame.
Planar IPPE gives both mirror solutions; the better one is refined with
Levenberg-Marquardt. A fit is refused when it is too sparse or collinear,
behind the camera, seen from the back, reprojects badly, is ambiguous with
the mirror solution's centre far away, or leaves the centre uncertain.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from navpy.core.ports.board_pose import (
    BoardPoseFit,
    BoardPoseRejection,
    BoardPoseResult,
)
from navpy.core.ports.detection import BoardCorners
from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec

# Fewer corners than this give a centre too weakly constrained to place an
# orbit (6 corners of a 4 x 6 grid still span two rows and two columns).
MIN_POSE_CORNERS = 6
# Plan step 22's done limit: a fit is used only when its reprojection p95 is
# within 2 pixels of the measured corners.
MAX_REPROJECTION_P95_PX = 2.0
# Corner noise assumed when a fit's own residual is smaller: a ChArUco corner
# is sub-pixel, but encoded video and blur leave about half a pixel.
CORNER_NOISE_FLOOR_PX = 0.5
# Limit on the board centre's largest one-sigma axis from the fit's
# covariance (at least CORNER_NOISE_FLOOR_PX of noise). In the synthetic
# sweep the true centre error's p95 was about 1.5 sigma, so 0.3 m keeps
# accepted fits inside step 22's 0.5 m centre-error limit.
MAX_CENTRE_SIGMA_M = 0.3
# The mirror IPPE solution counts as a rival when its error is within this
# ratio of the best one ...
AMBIGUITY_ERROR_RATIO = 1.5
# ... and the fit is ambiguous only if the rival puts the centre this far
# away (an orientation flip alone does not move the orbit centre).
AMBIGUOUS_CENTRE_GAP_M = 0.25

_MM_PER_M = 1000.0
_P95 = 95.0


class CharucoBoardPoseSolver:
    """Fit one ChArUco board to its found corners with the frame's optics."""

    def __init__(self, spec: CharucoBoardSpec) -> None:
        self._columns = spec.squares_x - 1
        board = cv2.aruco.CharucoBoard(
            (spec.squares_x, spec.squares_y),
            float(spec.square_mm),
            float(spec.marker_mm),
            cv2.aruco.getPredefinedDictionary(
                int(getattr(cv2.aruco, spec.dictionary))
            ),
        )
        corners_mm = np.asarray(board.getChessboardCorners(), dtype=np.float64)
        width_mm, height_mm = spec.size_mm
        centre_mm = np.array([0.5 * width_mm, 0.5 * height_mm, 0.0])
        self._object_m = (corners_mm - centre_mm) / _MM_PER_M

    @property
    def board_points_m(self) -> np.ndarray:
        """The interior corners by id, board-centred, in metres (a copy)."""
        return self._object_m.copy()

    def solve(
        self,
        corners: BoardCorners,
        intrinsic: np.ndarray,
        distortion: np.ndarray,
    ) -> BoardPoseResult:
        count = len(corners.ids)
        if count < MIN_POSE_CORNERS:
            return BoardPoseRejection("too_few_corners", count)
        ids = np.asarray(corners.ids, dtype=np.int64)
        if (
            ids.min() < 0
            or ids.max() >= len(self._object_m)
            or len(set(corners.ids)) != count
        ):
            return BoardPoseRejection("bad_corner_ids", count)
        if (
            len(set(ids % self._columns)) < 2
            or len(set(ids // self._columns)) < 2
        ):
            return BoardPoseRejection("collinear_corners", count)
        image = np.asarray(corners.points_px, dtype=np.float64).reshape(-1, 1, 2)
        k = np.asarray(intrinsic, dtype=np.float64)
        dist = np.asarray(distortion, dtype=np.float64).reshape(-1)
        if not (
            np.all(np.isfinite(image))
            and np.all(np.isfinite(k))
            and np.all(np.isfinite(dist))
        ):
            return BoardPoseRejection("non_finite_input", count)
        objects = self._object_m[ids].reshape(-1, 1, 3)
        return _fit(objects, image, k, dist, count)


def _fit(
    objects: np.ndarray,
    image: np.ndarray,
    k: np.ndarray,
    dist: np.ndarray,
    count: int,
) -> BoardPoseResult:
    try:
        solved, rvecs, tvecs, errors = cv2.solvePnPGeneric(
            objects, image, k, dist, flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        return BoardPoseRejection("solve_failed", count)
    if not solved or not rvecs:
        return BoardPoseRejection("solve_failed", count)
    errors = [float(np.ravel(error)[0]) for error in np.ravel(errors)]
    order = sorted(range(len(rvecs)), key=lambda index: errors[index])
    best = order[0]
    rvec, tvec = cv2.solvePnPRefineLM(
        objects, image, k, dist, rvecs[best].copy(), tvecs[best].copy(),
    )
    centre = np.ravel(tvec).astype(np.float64)
    if not (np.all(np.isfinite(centre)) and np.all(np.isfinite(rvec))):
        return BoardPoseRejection("solve_failed", count)
    if centre[2] <= 0.0:
        return BoardPoseRejection("behind_camera", count)
    rotation, _ = cv2.Rodrigues(rvec)
    # The board's z axis points into its printed face, away from a viewer.
    if float(rotation[:, 2] @ centre) <= 0.0:
        return BoardPoseRejection("board_faces_away", count)
    projected, jacobian = cv2.projectPoints(objects, rvec, tvec, k, dist)
    residuals = np.linalg.norm(
        projected.reshape(-1, 2) - image.reshape(-1, 2), axis=1,
    )
    rms = float(math.sqrt(float(np.mean(residuals ** 2))))
    p95 = float(np.percentile(residuals, _P95))
    if not p95 <= MAX_REPROJECTION_P95_PX:
        return BoardPoseRejection("reprojection", count)
    ratio = math.inf
    if len(order) > 1 and errors[best] > 0.0:
        ratio = errors[order[1]] / errors[best]
    if ratio < AMBIGUITY_ERROR_RATIO:
        rival = np.ravel(tvecs[order[1]]).astype(np.float64)
        if float(np.linalg.norm(rival - centre)) > AMBIGUOUS_CENTRE_GAP_M:
            return BoardPoseRejection("ambiguous", count)
    sigma = _centre_sigma_m(jacobian, max(rms, CORNER_NOISE_FLOOR_PX))
    if not sigma <= MAX_CENTRE_SIGMA_M:
        return BoardPoseRejection("centre_uncertain", count)
    return BoardPoseFit(
        centre_camera_m=(float(centre[0]), float(centre[1]), float(centre[2])),
        corner_count=count,
        reprojection_rms_px=rms,
        reprojection_p95_px=p95,
        centre_sigma_m=sigma,
        ambiguity_ratio=ratio,
    )


def _centre_sigma_m(jacobian: np.ndarray, noise_px: float) -> float:
    """Largest one-sigma axis of the translation from J^T J (Gauss-Newton)."""
    pose = np.asarray(jacobian, dtype=np.float64)[:, :6]
    try:
        covariance = np.linalg.inv(pose.T @ pose) * noise_px ** 2
    except np.linalg.LinAlgError:
        return math.inf
    eigen = np.linalg.eigvalsh(covariance[3:6, 3:6])
    if not np.all(np.isfinite(eigen)):
        return math.inf
    return float(math.sqrt(max(float(eigen.max()), 0.0)))


__all__ = [
    "AMBIGUITY_ERROR_RATIO",
    "AMBIGUOUS_CENTRE_GAP_M",
    "CORNER_NOISE_FLOOR_PX",
    "CharucoBoardPoseSolver",
    "MAX_CENTRE_SIGMA_M",
    "MAX_REPROJECTION_P95_PX",
    "MIN_POSE_CORNERS",
]
