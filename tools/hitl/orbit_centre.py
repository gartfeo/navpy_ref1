"""Where the HITL orbit is centred, for the pre-series orbit check (scenario_checks.py, step 23).

The circle's centre is navpy's own geolocation of the board from the first frame that finds it (only ``prepare``
places the orbit). Step 22's pose fit places it within POSE_CENTRE_LIMIT_M when the fit is accepted; a refused fit
leaves the height-zero ground ray, D * board_height / h beyond the board, D the first detection's horizontal
distance (the reading slant's reach, never less than r). The prediction runs the product's solver on seeded noisy
corners (POSE_DRAWS draws per yaw, CORNER_NOISE_FLOOR_PX) at the farthest slant the board can be read from; only if
every draw is accepted is the centre the board pose.
"""
from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np

from navpy.core.perception.zoom.fixed_zoom_steps import ZoomStep
from navpy.core.ports.board_pose import BoardPoseFit
from navpy.core.ports.detection import BoardCorners
from navpy.core.skills.geo_approach.self_detected_approach import WAITING_CIRCLE_REFERENCE_HEIGHT_M
from navpy.plugins.detection.charuco.board_pose import CORNER_NOISE_FLOOR_PX, CharucoBoardPoseSolver
from tools.hitl.orbit_plan import OrbitPlan, height_above_board, planned_step, reading_slant_m

# Step 22's done limit on the own-detected centre when its pose fit is accepted.
POSE_CENTRE_LIMIT_M = 0.5
# My value: seeded draws of the pose prediction (deterministic; every one must be accepted).
POSE_DRAWS = 40
POSE_SEED = 23
# Board yaws the prediction tries, relative to the line of sight (the scene's yaw is drawn per seed).
POSE_YAWS_DEG = (0.0, 45.0, 90.0)


def _board_in_camera(points_m: np.ndarray, slant_m: float, h: float, yaw_deg: float) -> np.ndarray:
    """The flat board's corners in the camera frame (x right, y down, z forward), centred on the optical axis and
    seen from depression asin(h / slant), turned yaw_deg about its own normal."""
    sin_d = h / slant_m
    cos_d = math.sqrt(max(0.0, 1.0 - sin_d * sin_d))
    normal = np.array([0.0, cos_d, sin_d])          # into the printed face: down, away from the camera
    along = np.array([0.0, sin_d, -cos_d])
    yaw = math.radians(yaw_deg)
    x_axis = math.cos(yaw) * np.array([1.0, 0.0, 0.0]) + math.sin(yaw) * along
    y_axis = np.cross(normal, x_axis)
    return (np.array([0.0, 0.0, slant_m]) + points_m[:, :1] * x_axis + points_m[:, 1:2] * y_axis
            + points_m[:, 2:3] * normal)


def pose_accepted_at(plan: OrbitPlan, step: ZoomStep, slant_m: float) -> bool:
    """Whether the product's solver accepts every seeded noisy fit of the board at this slant and step."""
    solver = CharucoBoardPoseSolver(plan.board.spec)
    points = solver.board_points_m
    k, dist = step.optics.k(), step.optics.dist_coeffs()
    rng = np.random.default_rng(POSE_SEED)
    for yaw in POSE_YAWS_DEG:
        image, _ = cv2.projectPoints(_board_in_camera(points, slant_m, height_above_board(plan), yaw),
                                     np.zeros(3), np.zeros(3), k, dist)
        image = image.reshape(-1, 2)
        for _ in range(POSE_DRAWS):
            noisy = image + rng.normal(0.0, CORNER_NOISE_FLOOR_PX, image.shape)
            corners = BoardCorners(tuple(range(len(noisy))), tuple((float(u), float(v)) for u, v in noisy))
            if not isinstance(solver.solve(corners, k, dist), BoardPoseFit):
                return False
    return True


def pose_accepted(plan: OrbitPlan) -> bool:
    """Whether the orbit centre is the board pose (accepted at the reading slant), else the ground ray."""
    step = planned_step(plan)
    return step is not None and pose_accepted_at(plan, step, reading_slant_m(plan, step))


def detection_distance_m(plan: OrbitPlan, step: Optional[ZoomStep]) -> float:
    """The worst horizontal distance of the first detection: the board is first found as soon as it can be read,
    at the reading slant's horizontal reach, which may lie beyond the orbit radius."""
    h = height_above_board(plan)
    if step is None:
        return plan.radius_m
    reach = math.sqrt(max(0.0, reading_slant_m(plan, step) ** 2 - h * h))
    return max(plan.radius_m, reach)


def centre_error_m(plan: OrbitPlan) -> float:
    """The worst horizontal error of the orbit centre: the accepted pose limit, or the height-zero ground ray."""
    if pose_accepted(plan):
        return POSE_CENTRE_LIMIT_M
    return abs(detection_distance_m(plan, planned_step(plan)) * plan.board_height_m / height_above_board(plan))


def descent_height_m(plan: OrbitPlan) -> float:
    """The height the planner sizes the descent from: the lane over the orbit centre's height, the board's for an
    accepted pose, the ground ray's (self_detected_approach.WAITING_CIRCLE_REFERENCE_HEIGHT_M) otherwise."""
    centre = plan.board_height_m if pose_accepted(plan) else WAITING_CIRCLE_REFERENCE_HEIGHT_M
    return plan.lane_height_m - centre
