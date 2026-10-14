"""Camera and board geometry of the dock-pass report (dock_pass_report.py): where the Unreal dock's ChArUco board is
in a frame of the fixed body camera from the truth pose, and the board pose OpenCV detects in that frame. Offline
scoring only; nothing here reaches navpy's navigation."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from navpy.plugins.detection.charuco.charuco_board import CharucoBoardSpec
from navpy.plugins.detection.charuco.charuco_detector import build_charuco_board

IMAGE_W, IMAGE_H = 1280, 720          # NativeCameraWidth/Height (the project's DefaultGame.ini)
FOV_H_DEG = 90.0                      # the gimbal camera's FOV (Unreal log "fov 90.0 deg")
MOUNT_PITCH_DEG = -35.0               # InitialPitchDeg of the fixed body camera
# Where the camera sits from the commanded vehicle position, body FRD (m): the level places the gimbal camera actor
# ahead of the vehicle's pivot. Measured 2026-10-14 from four static bench poses 6-15 m from the board
# (npyc_live_check --pose-at, ChArUco solvePnP of 20 frames each, pitch 0 and 5 deg, roll 0 and 10 deg): 2.33-2.61 m
# forward, -0.04-0.01 m right, 0.10-0.38 m down. Without it the truth outline sat tens of pixels above the board.
# A calibration with the same detector: pose errors below are against this camera position.
CAMERA_OFFSET_BODY_M = (2.5, 0.0, 0.3)
# The Unreal board: 5 x 7 squares of 40 cm, markers 0.74 of a square, half a square of margin (charuco_board.json),
# so 2.4 m east-west by 3.2 m north-south lying flat (Unreal +X east, +Y south, yaw 0).
BOARD_SPEC = CharucoBoardSpec(square_mm=400.0, marker_mm=296.0)
BOARD_HALF_EAST_M, BOARD_HALF_NORTH_M = 1.2, 1.6
MIN_PNP_CORNERS = 6                   # fewer ChArUco corners give no trustworthy board pose
MIN_PNP_SPAN = 2                      # ... nor corners all in one row or one column of the grid (collinear)


def metres_per_degree(lat_deg: float) -> tuple[float, float]:
    """(north, east) metres per degree on the WGS84 ellipsoid at this latitude."""
    phi = math.radians(lat_deg)
    north = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    east = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return north, east


def camera_matrix() -> np.ndarray:
    f = (IMAGE_W / 2) / math.tan(math.radians(FOV_H_DEG / 2))
    return np.array([[f, 0, IMAGE_W / 2], [0, f, IMAGE_H / 2], [0, 0, 1]], dtype=float)


def body_to_ned(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return rz @ ry @ rx


def camera_to_body() -> np.ndarray:
    """Columns: the OpenCV camera axes (x right, y down, z forward) in body FRD, pitched MOUNT_PITCH_DEG."""
    down = math.radians(-MOUNT_PITCH_DEG)
    z = np.array([math.cos(down), 0.0, math.sin(down)])
    x = np.array([0.0, 1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


@dataclass(frozen=True)
class Truth:
    lat: float
    lon: float
    alt_msl: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class View:
    in_view: bool
    slant_m: float                     # camera to board centre
    agl_m: float                       # the aircraft's height above the board
    centre_cam: np.ndarray             # board centre in the camera frame (m)
    normal_cam: np.ndarray             # board's up normal in the camera frame
    corners_px: np.ndarray             # the four outline corners projected (4 x 2)


def board_view(truth: Truth, board: tuple[float, float, float]) -> View:
    """Where the board is in this frame's camera, from the truth pose only."""
    blat, blon, bmsl = board
    mn, me = metres_per_degree(truth.lat)
    body = body_to_ned(truth.roll, truth.pitch, truth.yaw)
    rel_ned = np.array([(blat - truth.lat) * mn, (blon - truth.lon) * me, -(bmsl - truth.alt_msl)])
    rel_ned -= body @ np.array(CAMERA_OFFSET_BODY_M)
    ned_to_cam = (body @ camera_to_body()).T
    k = camera_matrix()
    corners = []
    in_front = True
    for dn, de in ((BOARD_HALF_NORTH_M, -BOARD_HALF_EAST_M), (BOARD_HALF_NORTH_M, BOARD_HALF_EAST_M),
                   (-BOARD_HALF_NORTH_M, BOARD_HALF_EAST_M), (-BOARD_HALF_NORTH_M, -BOARD_HALF_EAST_M)):
        cam = ned_to_cam @ (rel_ned + np.array([dn, de, 0.0]))
        in_front &= cam[2] > 0
        pix = k @ cam
        corners.append(pix[:2] / pix[2] if cam[2] > 0 else np.array([np.nan, np.nan]))
    corners = np.array(corners)
    inside = in_front and bool(np.all((corners[:, 0] >= 0) & (corners[:, 0] < IMAGE_W)
                                      & (corners[:, 1] >= 0) & (corners[:, 1] < IMAGE_H)))
    centre = ned_to_cam @ rel_ned
    return View(inside, float(np.linalg.norm(rel_ned)), truth.alt_msl - bmsl, centre,
                ned_to_cam @ np.array([0.0, 0.0, -1.0]), corners)


def spans_grid(ids: np.ndarray) -> bool:
    """True when the ChArUco corner ids lie on at least MIN_PNP_SPAN rows and columns of the corner grid: corners in
    one row or one column are collinear and leave the board's pose undetermined."""
    per_row = BOARD_SPEC.squares_x - 1
    return len(set(ids // per_row)) >= MIN_PNP_SPAN and len(set(ids % per_row)) >= MIN_PNP_SPAN


def detected_pose(image: np.ndarray) -> Optional[dict]:
    """The board's centre and up normal in the camera frame from its ChArUco corners (solvePnP), or None."""
    board = build_charuco_board(BOARD_SPEC)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4
                                                      else cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
    if ids is None or len(ids) < MIN_PNP_CORNERS:
        return None
    flat = np.asarray(ids).reshape(-1)
    if not spans_grid(flat):
        return None
    objects = np.asarray(board.getChessboardCorners(), dtype=float)[flat] / 1000.0
    try:
        ok, rvec, tvec = cv2.solvePnP(objects, np.asarray(corners, dtype=float).reshape(-1, 2), camera_matrix(), None)
    except cv2.error:
        return None
    if not ok:
        return None
    rot, _ = cv2.Rodrigues(rvec)
    grid_centre = np.array([BOARD_SPEC.squares_x, BOARD_SPEC.squares_y, 0.0]) * BOARD_SPEC.square_mm / 2000.0
    centre = rot @ grid_centre + tvec.reshape(3)
    normal = rot @ np.array([0.0, 0.0, 1.0])
    # OpenCV's board z points into the board; the face seen is the one facing the camera
    normal = -normal if normal @ centre > 0 else normal
    return {"centre": centre, "normal": normal, "corners": np.asarray(corners).reshape(-1, 2), "count": len(ids)}


def pose_error(view: View, pose: dict) -> dict:
    error = pose["centre"] - view.centre_cam
    cos = float(np.clip(pose["normal"] @ view.normal_cam, -1.0, 1.0))
    return {"position_m": float(np.linalg.norm(error)),
            "range_m": float(np.linalg.norm(pose["centre"]) - np.linalg.norm(view.centre_cam)),
            "tilt_deg": math.degrees(math.acos(cos))}
