"""Where the HITL orbit is centred, for the pre-series orbit check (scenario_checks.py, steps 23 and 23b).

The circle's centre is navpy's own geolocation of the dock from its first eligible detection: with the whole-dock
detector there is never a board pose, so it is always the height-zero ground ray (hitl_dock_detector.md section 4.1).
Its worst horizontal error e is the plain sum of the declared bounds (orbit_plan.GroundRayBounds) at the farthest
first detection D_max, slant S, h the lane over the POI (the dock's mid-height):

    D hd / h (the POI's height above home) + D dh / h (lane height) + S^2 / h * depression + S * azimuth
    + (S^2 / (h fy) + S / fx) * box-centre px (at the search step) + airspeed * capture skew

D_max is where the dock's box, at its most favourable yaw and plus the declared box-size error, shrinks to the
acquisition gate's floor at the search step (the widest: nothing is acquired before the gate passes, so nothing
zooms in before it); never less than r.

The dock's box: a box L x W x H on the ground, seen from depression d with its long axis at yaw psi to the line of
sight, spans L|sin psi| + W|cos psi| across and (L|cos psi| + W|sin psi|) sin d + H cos d down (orthographic, the
picture at these ranges).
"""
from __future__ import annotations

import math

from navpy.core.perception.zoom.fixed_zoom_steps import ZoomStep
from navpy.core.skills.geo_approach.self_detected_approach import WAITING_CIRCLE_REFERENCE_HEIGHT_M
from tools.hitl.orbit_plan import Dock, OrbitPlan, height_above_poi, search_step

# Dock yaws to the line of sight the checks try (my value: every 15 deg over a half turn; the box is symmetric).
DOCK_YAWS_DEG = tuple(range(0, 180, 15))
# The bisection's horizontal search span and tolerance for D_max (my values).
FARTHEST_SEARCH_M = 20_000.0
DISTANCE_TOLERANCE_M = 0.1


def dock_box_px(step: ZoomStep, dock: Dock, slant_m: float, h: float, yaw_deg: float) -> tuple[float, float]:
    """The dock's box (width, height) in full-frame pixels at this step, slant and yaw."""
    sin_d = h / slant_m
    cos_d = math.sqrt(max(0.0, 1.0 - sin_d * sin_d))
    s, c = abs(math.sin(math.radians(yaw_deg))), abs(math.cos(math.radians(yaw_deg)))
    across = dock.length_m * s + dock.width_m * c
    down = (dock.length_m * c + dock.width_m * s) * sin_d + dock.height_m * cos_d
    return step.optics.fx * across / slant_m, step.optics.fy * down / slant_m


def smaller_side_input_px(plan: OrbitPlan, step: ZoomStep, slant_m: float, yaw_deg: float) -> float:
    """The box's smaller side in the detector's input pixels."""
    width, height = dock_box_px(step, plan.dock, slant_m, height_above_poi(plan), yaw_deg)
    scale_x, scale_y = plan.detector.input_scale(plan.steps.stream.image_width, plan.steps.stream.image_height)
    return min(width * scale_x, height * scale_y)


def _largest_side(plan: OrbitPlan, step: ZoomStep, horizontal_m: float) -> float:
    slant = math.hypot(horizontal_m, height_above_poi(plan))
    return max(smaller_side_input_px(plan, step, slant, yaw) for yaw in DOCK_YAWS_DEG)


def acquirable_beyond_radius(plan: OrbitPlan) -> bool:
    """Whether the gate's floor can be met at the search step at the circle's radius or farther (D_max is not just
    the radius it is clamped to): a floor the dock's box never reaches would never acquire."""
    floor = plan.detector.acq_min_box_px - plan.bounds.box_size_error_px
    return _largest_side(plan, search_step(plan), plan.radius_m) >= floor


def first_detection_distance_m(plan: OrbitPlan) -> float:
    """D_max: the farthest horizontal distance where the gate can acquire (never less than the radius)."""
    step, floor = search_step(plan), plan.detector.acq_min_box_px - plan.bounds.box_size_error_px
    if _largest_side(plan, step, plan.radius_m) < floor:
        return plan.radius_m
    near, far = plan.radius_m, FARTHEST_SEARCH_M
    if _largest_side(plan, step, far) >= floor:
        return far
    while far - near > DISTANCE_TOLERANCE_M:
        middle = (near + far) / 2.0
        near, far = (middle, far) if _largest_side(plan, step, middle) >= floor else (near, middle)
    return far


def centre_error_terms(plan: OrbitPlan) -> dict[str, float]:
    """Each declared term of the ground ray's horizontal error at D_max (m)."""
    d, h, b = first_detection_distance_m(plan), height_above_poi(plan), plan.bounds
    slant, optics = math.hypot(d, h), search_step(plan).optics
    return {
        "poi_height": d * plan.dock.poi_height_m / h,
        "lane_height": d * b.lane_height_error_m / h,
        "depression": slant * slant / h * math.radians(b.depression_bias_deg),
        "azimuth": slant * math.radians(b.azimuth_bias_deg),
        "box_centre": (slant * slant / (h * optics.fy) + slant / optics.fx) * b.box_centre_bias_px,
        "capture_skew": plan.airspeed_m_s * b.capture_skew_s,
    }


def centre_error_m(plan: OrbitPlan) -> float:
    """The bound e: the plain sum of the declared terms."""
    return sum(centre_error_terms(plan).values())


def descent_height_m(plan: OrbitPlan) -> float:
    """The height the planner sizes the descent from: the lane over the ground ray's height
    (self_detected_approach.WAITING_CIRCLE_REFERENCE_HEIGHT_M)."""
    return plan.lane_height_m - WAITING_CIRCLE_REFERENCE_HEIGHT_M
