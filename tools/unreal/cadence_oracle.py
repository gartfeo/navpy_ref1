"""The capture-cadence check's independent oracle and reprojection: the camera transform rebuilt from
the admitted poses, the board clock map and the emulator's true gimbal, and the board corners
projected with each frame's logged camera. Scoring only."""
from __future__ import annotations

import bisect
import math
import statistics
from dataclasses import dataclass
from itertools import accumulate
from typing import Optional, Sequence

import numpy as np

from navpy.core.perception.camera.board_clock_map import (BOARD_CLOCK_HISTORY_COUNT, BOARD_CLOCK_MAX_GAP_S,
                                                          BOARD_CLOCK_MIN_SAMPLES)
from tools.unreal.cadence_log import Admit, BoardPose, GimbalStep, Mount, UnrealLog, parse_vec
from tools.unreal.charuco_board_texture import MARGIN_SQUARES
from tools.unreal.dock_pass_geometry import BOARD_SPEC, CameraModel
from tools.unreal.pose_stamp_log import _angle_deg as quat_angle_deg, _slerp as quat_slerp
from tools.unreal.unreal_math import Quat, Vec, compose_gimbal, quat_conj, quat_mul, quat_rotate

ANGLE_TOLERANCE_DEG = 0.01            # step 4: transform fixtures within 0.01 deg / 1 mm
LOCATION_TOLERANCE_CM = 0.1
LOGGED_TIME_TOLERANCE_S = 1.0e-6      # the plugin logs PC times with 7 decimals: two roundings of 0.05 us
WARMUP_S = 2.0                        # the clock map needs 10 poses; brackets are scored after this
REPROJECTION_P95_PX = 2.0             # step 3's limit
MIN_REPROJECTION_CORNERS = 6          # dock_pass_geometry.MIN_PNP_CORNERS
NEIGHBOUR_MOVE_PX = 2.0               # a neighbour's camera moved the projected corners this much: a timing probe
MIN_NEIGHBOUR_FRAMES = 30
GIMBAL_MAX_BRACKET_GAP_S = 0.030      # FNavPyGimbalHistory::MaxBracketGapS
POSE_MAX_BRACKET_GAP_US = 100000      # FNavPyBoardTimeInterpolator::MaxBracketGapUs


# --- sources -------------------------------------------------------------------------------------------------------
class StreamSources:
    """One stream generation's admitted poses. ``n`` (a capture's map_n) selects the first n admitted, the ones
    the interpolator and the clock map held at that capture."""

    def __init__(self, admits: Sequence[Admit]) -> None:
        self.admits = list(admits)
        self.order = sorted(range(len(self.admits)), key=lambda i: self.admits[i].board_us)
        self.times = [self.admits[i].board_us for i in self.order]
        self.newest = list(accumulate((a.board_us for a in self.admits), max))

    def clock_offset(self, n: int, board_us: int) -> Optional[float]:
        """FNavPyBoardClockMap::Map over the first n: min(receipt - board) of the newest 64, or None."""
        if n < BOARD_CLOCK_MIN_SAMPLES or n > len(self.admits):
            return None
        if abs(board_us - self.newest[n - 1]) * 1.0e-6 > BOARD_CLOCK_MAX_GAP_S:
            return None
        return min(a.receipt_qpc - a.board_us * 1.0e-6 for a in self.admits[max(0, n - BOARD_CLOCK_HISTORY_COUNT):n])

    def pose(self, n: int, board_us: int) -> Optional[tuple[Vec, Quat, bool]]:
        """The interpolator's pose at board_us from the first n (lerp / slerp, valid within its bracket gap)."""
        if not 0 < n <= len(self.admits):
            return None
        i = bisect.bisect_right(self.times, board_us) - 1
        while i >= 0 and self.order[i] >= n:
            i -= 1
        if i < 0:
            return None
        a = self.admits[self.order[i]]
        if a.board_us == board_us:
            return a.location, a.quat, True
        newest, j = self.newest[n - 1], i + 1
        while j < len(self.times) and self.times[j] <= newest and self.order[j] >= n:
            j += 1
        if j >= len(self.times) or self.times[j] > newest:
            return None
        b = self.admits[self.order[j]]
        w = (board_us - a.board_us) / (b.board_us - a.board_us)
        loc = tuple(p + w * (q - p) for p, q in zip(a.location, b.location))
        return loc, quat_slerp(a.quat, b.quat, w), b.board_us - a.board_us <= POSE_MAX_BRACKET_GAP_US


def gimbal_at(steps: Sequence[GimbalStep], times: Sequence[float], qpc_s: float) -> Optional[tuple[GimbalStep, bool]]:
    """FNavPyGimbalHistory::Query on the emulator's steps: a lerp inside a valid bracket (consecutive sequences at
    most 30 ms apart), else the nearest step, not valid."""
    i = bisect.bisect_right(times, qpc_s) - 1
    if i < 0 or i + 1 >= len(steps) and steps[i].t != qpc_s:
        return None
    a = steps[i]
    if a.t == qpc_s:
        return a, True
    b = steps[i + 1]
    if b.seq != a.seq + 1 or b.t - a.t > GIMBAL_MAX_BRACKET_GAP_S:
        return (a if qpc_s - a.t <= b.t - qpc_s else b), False
    w = (qpc_s - a.t) / (b.t - a.t)
    return GimbalStep(qpc_s, a.seq, a.yaw + w * (b.yaw - a.yaw), a.pitch + w * (b.pitch - a.pitch),
                      a.roll + w * (b.roll - a.roll), a.roll_stabilized, a.zoom), True


@dataclass(frozen=True)
class Expected:
    location: Vec
    quat: Quat
    pose_valid: bool
    gimbal_valid: bool


def expected_camera(pose: tuple[Vec, Quat, bool], gimbal: tuple[GimbalStep, bool], mount: Mount) -> Expected:
    """The capture component's world transform from the vehicle pose, the gimbal state and the mount."""
    vehicle_loc, vehicle_quat, pose_valid = pose
    step, gimbal_valid = gimbal
    pitch = min(max(step.pitch, mount.min_pitch), mount.max_pitch)
    pivot_rot = compose_gimbal(vehicle_quat, step.yaw, pitch, step.roll, mount.stabilize_roll and step.roll_stabilized)
    pivot = quat_rotate(vehicle_quat, mount.pivot_cm)
    lever = quat_rotate(pivot_rot, mount.camera_in_pitch_cm)
    location = tuple(v + p + c for v, p, c in zip(vehicle_loc, pivot, lever))
    return Expected(location, quat_mul(pivot_rot, mount.camera_in_pitch_quat), pose_valid, gimbal_valid)


# --- projection ----------------------------------------------------------------------------------------------------
def board_corners_world(board: BoardPose, flip_x: float, flip_y: float) -> np.ndarray:
    """World positions (cm) of the ChArUco inner corners by id. The board texture spans the plane's X (squares_x)
    and Y (squares_y); which way each runs is the texture mapping's, so flip_x/flip_y (+-1) choose it."""
    sx, sy = BOARD_SPEC.squares_x, BOARD_SPEC.squares_y
    square = board.width_cm / (sx + 2 * MARGIN_SQUARES)
    points = []
    for corner_id in range((sx - 1) * (sy - 1)):
        row, col = divmod(corner_id, sx - 1)
        local = (flip_x * ((col + 1) - sx / 2) * square, flip_y * ((row + 1) - sy / 2) * square, 0.0)
        offset = quat_rotate(board.quat, local)
        points.append([board.location[i] + offset[i] for i in range(3)])
    return np.array(points)


def project(points_cm: np.ndarray, location: Vec, quat: Quat, camera: CameraModel) -> np.ndarray:
    """Pixels (OpenCV convention) of world points in an Unreal camera (X forward, Y right, Z up) with the native
    camera's pinhole (NavPyGimbalCameraActor::BuildNativeCameraIntrinsicsProjection)."""
    inverse = quat_conj(quat)
    out = []
    for p in points_cm:
        x, y, z = quat_rotate(inverse, (p[0] - location[0], p[1] - location[1], p[2] - location[2]))
        out.append([camera.fx * y / x + camera.cx, -camera.fy * z / x + camera.cy] if x > 0 else [math.nan, math.nan])
    return np.array(out)


def _quantile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def record_transform(record: dict) -> tuple[Vec, Quat]:
    return parse_vec(record["cam_loc"]), parse_vec(record["cam_quat"])


def check_oracle(log: UnrealLog, sent: Sequence[dict], steps: Sequence[GimbalStep], mount: Mount) -> dict:
    sources: dict[int, list[Admit]] = {}
    for a in log.admits:
        if a.port == mount.receiver_port and a.admitted:
            sources.setdefault(a.gen, []).append(a)
    streams = {gen: StreamSources(admits) for gen, admits in sources.items()}
    step_times = [s.t for s in steps]
    first_board: dict[int, int] = {}
    for s in sent:
        gen = int(s["gen"])
        first_board[gen] = min(first_board.get(gen, int(s["board_us"])), int(s["board_us"]))

    angle_err, loc_err = [], []
    unscored = invalid = clock_mismatch = 0
    for s in sent:
        gen, board_us, n = int(s["gen"]), int(s["board_us"]), int(s["map_n"])
        if board_us - first_board[gen] < WARMUP_S * 1e6:
            continue
        stream = streams.get(gen)
        offset = None if stream is None else stream.clock_offset(n, board_us)
        # The plugin's own map and query must be what the poses it admitted give.
        if (offset is None) != (s["mapped"] != "1") or offset is not None and (
                abs(float(s["offset_s"]) - offset) > LOGGED_TIME_TOLERANCE_S
                or abs(float(s["gimbal_qpc"]) - (board_us * 1e-6 + offset)) > LOGGED_TIME_TOLERANCE_S):
            clock_mismatch += 1
        pose = None if stream is None else stream.pose(n, board_us)
        gimbal = None if offset is None else gimbal_at(steps, step_times, board_us * 1e-6 + offset)
        if pose is None or gimbal is None:
            unscored += 1
            continue
        expected = expected_camera(pose, gimbal, mount)
        loc, quat = record_transform(s)
        angle_err.append(quat_angle_deg(quat, expected.quat))
        loc_err.append(math.dist(loc, expected.location))
        invalid += not (expected.pose_valid and expected.gimbal_valid and s["pose_valid"] == "1"
                        and s["gimbal_valid"] == "1")
    return {"scored": len(angle_err), "unscored": unscored, "invalid_brackets": invalid,
            "clock_mismatch": clock_mismatch, "max_angle_deg": max(angle_err, default=None),
            "max_location_cm": max(loc_err, default=None),
            "pass": bool(angle_err) and unscored == 0 and invalid == 0 and clock_mismatch == 0
            and max(angle_err) <= ANGLE_TOLERANCE_DEG and max(loc_err) <= LOCATION_TOLERANCE_CM}


def _corner_errors(record: dict, frame: dict, corners: np.ndarray, camera: CameraModel) -> Optional[np.ndarray]:
    detected = frame.get("corners") or []
    if len(detected) < MIN_REPROJECTION_CORNERS:
        return None
    loc, quat = record_transform(record)
    projected = project(corners[[int(c[0]) for c in detected]], loc, quat, camera)
    return np.linalg.norm(projected - np.array([[c[1], c[2]] for c in detected]), axis=1)


def check_reprojection(log: UnrealLog, sent: Sequence[dict], recorded: dict[int, dict], camera: CameraModel) -> dict:
    if log.board is None:
        return {"pass": False, "reason": "no NavPyBoardPose line"}
    ordered = sorted((s for s in sent if int(s["frame"]) in recorded), key=lambda s: int(s["frame"]))
    best = None
    for flips in ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)):
        corners = board_corners_world(log.board, *flips)
        errors = [e for s in ordered if (e := _corner_errors(s, recorded[int(s["frame"])], corners, camera)) is not None]
        median = statistics.median(float(np.median(e)) for e in errors) if errors else math.inf
        if best is None or median < best[0]:
            best = (median, flips, corners, errors)
    _median, flips, corners, own = best
    if not own:
        return {"pass": False, "reason": "no frame with enough detected corners"}
    residuals = [float(r) for e in own for r in e]
    # Timing probe: the frame's own record against its neighbours' records, where their cameras moved the board.
    own_moving, neighbour_moving = [], []
    for i, s in enumerate(ordered):
        frame = recorded[int(s["frame"])]
        mine = _corner_errors(s, frame, corners, camera)
        if mine is None:
            continue
        loc, quat = record_transform(s)
        mine_px = project(corners, loc, quat, camera)
        for j in (i - 1, i + 1):
            if not 0 <= j < len(ordered):
                continue
            other_loc, other_quat = record_transform(ordered[j])
            shift = np.nanmean(np.linalg.norm(project(corners, other_loc, other_quat, camera) - mine_px, axis=1))
            if shift > NEIGHBOUR_MOVE_PX:
                own_moving.append(float(np.mean(mine)))
                neighbour_moving.append(float(np.mean(_corner_errors(ordered[j], frame, corners, camera))))
    p95 = _quantile(residuals, 0.95)
    timing_pass = (len(own_moving) >= MIN_NEIGHBOUR_FRAMES
                   and statistics.median(own_moving) < statistics.median(neighbour_moving))
    return {"frames": len(own), "corners": len(residuals), "flips": flips, "p95_px": p95,
            "moving_pairs": len(own_moving),
            "own_median_px": statistics.median(own_moving) if own_moving else None,
            "neighbour_median_px": statistics.median(neighbour_moving) if neighbour_moving else None,
            "pass": p95 is not None and p95 <= REPROJECTION_P95_PX and timing_pass}
