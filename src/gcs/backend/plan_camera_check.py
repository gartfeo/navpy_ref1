"""Plan/camera compatibility: can the search camera ever detect each sim dock?

Design: docs/design/plan_camera_check.md. Every number comes from NavPy's own
camera model: the first-zoom intrinsics (``build_camera_model``), the mount
transform and detection distance (``build_gimbal_data``), and the sim
detection pixel gate (``FinitePoiProjector``). The aircraft flies level along
each route segment with the camera at its search pose; a result is a "never"
in that model, not a proof that a run works.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

import numpy as np
import pymap3d

from navpy.app.perception.vision_camera_factory import build_camera_model
from navpy.app.perception.vision_gimbal_profile import build_gimbal_data
from navpy.core.perception.profile.vision_class_profile import (
    MIN_DETECT_PIXELS,
    compute_detect_slant_range,
    get_class_detect_size,
)
from navpy.core.perception.profile.vision_detector_profile import get_detector_settings
from navpy.core.perception.profile.vision_profile_loader import get_devices
from navpy.core.perception.visual_ray_projection import camera_to_body_from_gimbal

# Mirror of the class the sim detector publishes for a dock (see
# routes/vision_profiles.py: DetectorSim publishes detection class 0).
_DOCK_CLASS_ID = 0


@dataclass(frozen=True)
class SearchCamera:
    """One camera mount at its search pose (first zoom, neutral gimbal)."""

    name: str
    fx: float
    fy: float
    cx: float
    cy: float
    width: float
    height: float
    body_to_camera: np.ndarray  # 3x3, body (fwd, right, down) -> camera (x right, y down, z fwd)
    range_m: float  # max slant distance at which the dock is detected


def _detect_range(fy: float, max_detect_distance: float, sim_finite: bool) -> float:
    """Slant range of the dock detection gate.

    Sim (finite FOV): ``FinitePoiProjector.project`` keeps a POI when its
    pixel size reaches MIN_DETECT_PIXELS *or* it is within the mount's
    ``max_detect_distance``; the gate is the larger of the two. Real: only the
    nominal pixel budget, an estimate (the real detector has no such gate).
    """
    pixel_range = compute_detect_slant_range(fy, get_class_detect_size(_DOCK_CLASS_ID), MIN_DETECT_PIXELS)
    return max(pixel_range, float(max_detect_distance)) if sim_finite else pixel_range


def search_cameras(profile: Mapping[str, object], *, sim_finite: bool) -> list[SearchCamera]:
    """Build every detecting mount of a profile the way NavPy builds it."""
    detector_settings = get_detector_settings(profile)
    cameras = []
    for index, device in enumerate(get_devices(profile)):
        gimbal = build_gimbal_data(device, index, detector_settings)
        if gimbal is None:  # NavPy skips a device without a gimbal block
            continue
        intrinsics = build_camera_model(device, None, log=False)
        k = intrinsics.get_k()
        fy = float(k[1, 1])
        cameras.append(SearchCamera(
            name=str(device.get("name", f"mount_{index}")),
            fx=float(k[0, 0]),
            fy=fy,
            cx=float(k[0, 2]),
            cy=float(k[1, 2]),
            width=float(intrinsics.image_width),
            height=float(intrinsics.image_height),
            body_to_camera=camera_to_body_from_gimbal(gimbal).matrix().T,
            range_m=_detect_range(fy, gimbal.max_detect_distance, sim_finite),
        ))
    return cameras


def _corner_rays(cam: SearchCamera) -> list[np.ndarray]:
    corners = ((0.0, 0.0), (cam.width, 0.0), (cam.width, cam.height), (0.0, cam.height))
    return [np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0]) for u, v in corners]


def _in_image(cam: SearchCamera, ray: np.ndarray) -> bool:
    if ray[2] <= 0.0:
        return False
    u = cam.cx + cam.fx * ray[0] / ray[2]
    v = cam.cy + cam.fy * ray[1] / ray[2]
    return 0.0 <= u <= cam.width and 0.0 <= v <= cam.height


def max_depression_sin(cam: SearchCamera) -> float:
    """sin of the steepest below-horizon angle of any ray in the image.

    The in-image rays form a convex cone, so the maximum of
    down . r / |r| lies at the nadir (if it is in the image) or on an image
    edge; along an edge r = a + t*b the stationary point has a closed form.
    """
    down = cam.body_to_camera @ np.array([0.0, 0.0, 1.0])
    if _in_image(cam, down):
        return 1.0
    rays = _corner_rays(cam)
    best = -1.0
    for i in range(4):
        a, end = rays[i], rays[(i + 1) % 4]
        b = end - a
        p, q = float(down @ a), float(down @ b)
        aa, ab, bb = float(a @ a), float(a @ b), float(b @ b)
        candidates = [0.0, 1.0]
        denom = q * ab - p * bb
        if abs(denom) > 1e-15:
            t = (p * ab - q * aa) / denom
            if 0.0 < t < 1.0:
                candidates.append(t)
        for t in candidates:
            r = a + t * b
            best = max(best, float(down @ r) / float(np.linalg.norm(r)))
    return best


def max_search_height(cam: SearchCamera) -> float:
    """Highest height above the dock at which some in-image ground point is within range."""
    s = max_depression_sin(cam)
    return cam.range_m * s if s > 0.0 else 0.0


def search_window(cam: SearchCamera, height_m: float) -> Optional[tuple[float, float]]:
    """(near, range edge) ground distance for a dock height_m below: where the
    steepest image ray meets the ground, and where detection range ends (a
    range bound only; the image may end sooner). None when there is no window."""
    s = max_depression_sin(cam)
    if height_m <= 0.0 or s <= 0.0 or height_m > cam.range_m * s:
        return None
    near = height_m * math.sqrt(1.0 - s * s) / s
    return near, math.sqrt(cam.range_m ** 2 - height_m ** 2)


def segment_in_view(
    cam: SearchCamera,
    height_m: float,
    start_ne: Sequence[float],
    end_ne: Sequence[float],
    dock_ne: Sequence[float],
) -> Optional[tuple[float, float]]:
    """Exact along-segment interval [s0, s1] (m from start) where the dock is in view.

    Level flight, heading along the segment. In body axes the dock is at
    (f0 - s, c, h), so its camera coordinates are linear in s: every image
    edge is a linear inequality in s and the range a quadratic one.
    """
    start = np.asarray(start_ne, dtype=float)
    seg = np.asarray(end_ne, dtype=float) - start
    length = float(np.hypot(*seg))
    if length <= 0.0:
        return None
    fwd = seg / length
    right = np.array([-fwd[1], fwd[0]])
    rel = np.asarray(dock_ne, dtype=float) - start
    f0, c = float(rel @ fwd), float(rel @ right)
    lateral_sq = c * c + height_m * height_m
    if lateral_sq > cam.range_m ** 2:
        return None
    reach = math.sqrt(cam.range_m ** 2 - lateral_sq)
    lo, hi = max(0.0, f0 - reach), min(length, f0 + reach)
    v0 = cam.body_to_camera @ np.array([f0, c, height_m])
    v1 = cam.body_to_camera @ np.array([-1.0, 0.0, 0.0])
    # alpha + beta*s >= 0 for: in front, u >= 0, u <= W, v >= 0, v <= H.
    rows = (
        (np.array([0.0, 0.0, 1.0])),
        (np.array([cam.fx, 0.0, cam.cx])),
        (np.array([-cam.fx, 0.0, cam.width - cam.cx])),
        (np.array([0.0, cam.fy, cam.cy])),
        (np.array([0.0, -cam.fy, cam.height - cam.cy])),
    )
    for row in rows:
        alpha, beta = float(row @ v0), float(row @ v1)
        if abs(beta) < 1e-12:
            if alpha < 0.0:
                return None
            continue
        bound = -alpha / beta
        if beta > 0.0:
            lo = max(lo, bound)
        else:
            hi = min(hi, bound)
    if lo > hi:
        return None
    # The projector rejects zero distance: some point must be strictly in front.
    # Depth is linear in s, so it is largest at one end of the interval.
    depth = lambda s: float(v0[2] + v1[2] * s)  # noqa: E731
    return (lo, hi) if max(depth(lo), depth(hi)) > 0.0 else None


# --------------------------------------------------------------------------
# Plan check
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class ZoneInput:
    """One zone as it will be flown, resolved by the frontend route adapter.

    ``route`` starts at the waypoint detection starts from; ``docks`` are the
    zone's ``targ_wps`` waypoints. ``dock_alt_m`` is the vehicle's
    AAS_TARG_ALT (None = unknown); ``use_terrain`` its AAS_USE_TRN (None =
    unknown).
    """

    zone_index: int
    altitude_m: float
    dock_alt_m: Optional[float]
    use_terrain: Optional[bool]
    route: Sequence[tuple[float, float]]
    docks: Sequence[tuple[float, float, int]]  # (lat, lon, mission waypoint number)


def _to_ne(lat: float, lon: float, origin: tuple[float, float]) -> tuple[float, float]:
    n, e, _ = pymap3d.geodetic2ned(lat, lon, 0.0, origin[0], origin[1], 0.0)
    return float(n), float(e)


def _dive_deg(height_m: float, ground_m: float) -> float:
    return math.degrees(math.atan2(height_m, ground_m))


def _dock_detectable(cams: Sequence[SearchCamera], height_m: float,
                     route_ne: Sequence[tuple[float, float]], dock_ne: tuple[float, float]) -> bool:
    return any(
        segment_in_view(cam, height_m, route_ne[i], route_ne[i + 1], dock_ne) is not None
        for cam in cams
        for i in range(len(route_ne) - 1)
    )


def check_zone(cams: Sequence[SearchCamera], zone: ZoneInput, *, exact_sensor: bool) -> dict:
    """Facts and problems for one zone; the frontend words the messages."""
    out: dict = {"zone_index": zone.zone_index, "dock_count": len(zone.docks)}
    if not zone.docks:
        return {**out, "checked": False, "reason": "no_docks"}
    if zone.dock_alt_m is None:
        return {**out, "checked": False, "reason": "dock_alt_unknown"}
    height = float(zone.altitude_m) - float(zone.dock_alt_m)
    # Terrain on (or unknown) moves the dock by its ground's difference to
    # home, which the GCS cannot see: the result is then an estimate.
    estimate = (not exact_sensor) or zone.use_terrain is not False
    severity = "warning" if estimate else "error"
    windows = []
    for cam in cams:
        win = search_window(cam, height)
        if win is not None:
            windows.append({
                "camera": cam.name, "near_m": win[0], "range_edge_m": win[1],
                "dive_near_deg": _dive_deg(height, win[0]), "dive_range_edge_deg": _dive_deg(height, win[1]),
            })
    best = max(cams, key=max_search_height)
    out.update({
        "checked": True, "estimate": estimate, "height_m": height, "windows": windows,
        "max_height_m": max_search_height(best), "range_m": best.range_m,
    })
    problems = []
    if height > 0.0 and not windows:
        s = max_depression_sin(best)
        near = height * math.sqrt(1.0 - s * s) / s if s > 0.0 else None
        problems.append({
            "kind": "no_search_window", "severity": severity, "camera": best.name,
            "near_m": near, "range_m": best.range_m, "max_height_m": max_search_height(best),
        })
    else:
        origin = zone.route[0] if zone.route else (zone.docks[0][0], zone.docks[0][1])
        route_ne = [_to_ne(lat, lon, origin) for lat, lon in zone.route]
        for lat, lon, wp in zone.docks:
            if not _dock_detectable(cams, height, route_ne, _to_ne(lat, lon, origin)):
                problems.append({"kind": "dock_never_in_view", "severity": severity, "wp": int(wp)})
    out["problems"] = problems
    return out


def check_plan(profile: Optional[Mapping[str, object]], zones: Sequence[ZoneInput], *, sim_mode: bool) -> dict:
    """Check every zone against the profile's search cameras."""
    if profile is None:
        return {"status": "skipped", "reason": "no_profile", "zones": []}
    if sim_mode and bool(get_detector_settings(profile).get("ideal_360", False)):
        # ideal_360 has no finite frame (SimPoiProjector bypasses the finite gate).
        return {"status": "skipped", "reason": "ideal_360", "zones": []}
    cams = search_cameras(profile, sim_finite=sim_mode)
    if not cams:
        return {"status": "skipped", "reason": "no_camera", "zones": []}
    results = [check_zone(cams, zone, exact_sensor=sim_mode) for zone in zones]
    has_problem = any(r.get("problems") for r in results)
    return {"status": "problems" if has_problem else "ok", "sim_mode": sim_mode, "zones": results}
