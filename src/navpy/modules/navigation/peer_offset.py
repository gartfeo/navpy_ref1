"""Compute approach points for peer-assigned or fallback delivery location navigation based on camera geometry.

Two strategies (see ``approach_strategy.py``):
- OFFSET: offset point behind target, bank-corrected for fixed cameras.
- ORBIT: orbit directly around target. With a zoom mount and navigation
  limits, the radius is the FURTHEST standoff that still tracks at 1x and
  confirms at max zoom (``min`` of the two camera-pixel bounds), floored by
  the dive-feasibility minimum (``orbit_geometry.r_nav_min``); otherwise
  it falls back to camera-pixel-range sizing.
"""

import logging
import math
from typing import List, Optional

import pymap3d
from geopy.distance import geodesic

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.navigation.orbit_geometry import (
    OrbitNavigationLimits,
    r_nav_min,  # compatibility re-export
)
from navpy.modules.navigation.peer_orbit_plan import (
    MIN_ACQUIRE_STANDOFF_M,
    MIN_APPROACH_STANDOFF_M,
    MIN_TRACK_PIXELS,
    _ORBIT_PIXEL_MARGIN,
    compute_orbit_plan,
)
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.vision_class_profile import (
    MIN_CONFIRM_PIXELS,  # compatibility re-export
    MIN_DETECT_PIXELS,  # compatibility re-export
    compute_approach_interval,
    compute_detect_slant_range,
    get_class_detect_size,
)


_log = logging.getLogger(__name__)

OFFSET_LOITER_RADIUS_M = 80.0  # GUIDED-loiter radius for fixed-camera approach
_CRUISE_AIRSPEED = 25.0  # m/s — typical fixed-wing cruise
_GRAVITY = 9.81


def select_approach_mount(mounts: List[CameraMount]) -> Optional[CameraMount]:
    """Pick the mount with the smallest absolute negative pitch.

    Only downward-looking cameras (pitch < 0) can see the ground ahead.
    Among those, the least-negative pitch looks furthest forward, giving
    the longest preview distance — so the offset point is placed where
    that camera's boresight intersects the ground.

    Returns None if no downward-looking mount exists.
    """
    best: Optional[CameraMount] = None
    best_pitch: float = -math.inf
    for mount in mounts:
        pitch = mount.get_gimbal_data().att.pitch
        if pitch >= 0:
            continue
        if pitch > best_pitch:
            best_pitch = pitch
            best = mount
    return best


def calc_peer_approach_offset(
    target: Location,
    drone_loc: Location,
    mounts: List[CameraMount],
    class_id: int = 0,
    kind: ApproachKind = ApproachKind.OFFSET,
    *,
    orbit_limits: Optional[OrbitNavigationLimits] = None,
    recognition_px: Optional[float] = None,
) -> ApproachPlan:
    """Compute approach point based on camera geometry and strategy.

    Parameters
    ----------
    target : Location
        Peer-assigned or fallback delivery location position.
    drone_loc : Location
        Current drone position (used for approach bearing and altitude).
    mounts : list[CameraMount]
        Available camera mounts — ``select_approach_mount`` picks the best.
    class_id : int
        Numeric detection class used to look up the configured dimensions.
    kind : ApproachKind
        Which approach strategy to use.
    orbit_limits : OrbitNavigationLimits | None
        Vehicle envelope supplying the dive-feasibility FLOOR
        (``r_nav_min``) for the ORBIT radius. When supplied (and the chosen
        mount has zoom), the radius is the FURTHEST standoff that still tracks
        at 1x and confirms at max zoom, never inside this floor. ``None`` (or
        a no-zoom mount) keeps the legacy camera-pixel-range sizing.
    recognition_px : float | None
        Per-class operator-ID pixel demand (``get_min_pixels_for_class``),
        the max-zoom recognition bound for the ORBIT radius. ``None`` falls
        back to ``MIN_CONFIRM_PIXELS``. Geometry-only here; the caller owns
        the profile policy.

    Returns
    -------
    ApproachPlan
        The computed approach plan with location, offset, and orbit radius.
    """
    mount = select_approach_mount(mounts)
    if mount is None:
        return ApproachPlan(kind=kind, approach_location=target, offset_distance=0.0)

    pitch_deg = mount.get_gimbal_data().att.pitch
    alt_diff = drone_loc.alt - target.alt
    alt = alt_diff if alt_diff > 10 else drone_loc.alt
    if alt <= 0:
        return ApproachPlan(kind=kind, approach_location=target, offset_distance=0.0)

    if kind == ApproachKind.ORBIT:
        return _compute_orbit_plan(
            target, mount, alt, class_id, orbit_limits, recognition_px,
        )

    return _compute_offset_plan(target, drone_loc, mount, alt, pitch_deg, class_id)


def _compute_offset_plan(
    target: Location, drone_loc: Location, mount: CameraMount,
    alt: float, pitch_deg: float, class_id: int,
) -> ApproachPlan:
    """Compute bank-corrected offset point for fixed cameras."""
    try:
        k = mount.get_k()
        fy = float(k[1, 1])
        cy = float(k[1, 2])
        img_h = mount.image_height or 1080
        class_size = get_class_detect_size(class_id)

        interval = compute_approach_interval(fy, cy, img_h, abs(pitch_deg), alt, class_size)
        if interval is not None:
            g_min, g_max = interval
            # Fixed camera: account for bank-induced boresight shift.
            phi = math.atan2(_CRUISE_AIRSPEED ** 2, OFFSET_LOITER_RADIUS_M * _GRAVITY)
            boresight_ground = (g_min + g_max) / 2
            forward_banked = boresight_ground / math.cos(phi)
            lateral_banked = alt * math.tan(phi)
            offset_dist = math.sqrt(
                (OFFSET_LOITER_RADIUS_M + lateral_banked) ** 2
                + forward_banked ** 2
            )
        else:
            boresight_angle = abs(pitch_deg)
            max_slant = compute_detect_slant_range(fy, class_size)
            if boresight_angle > 0:
                boresight_dist = alt / math.tan(math.radians(boresight_angle))
                if max_slant > alt:
                    max_ground = math.sqrt(max_slant ** 2 - alt ** 2)
                    offset_dist = min(boresight_dist, max_ground)
                else:
                    offset_dist = min(boresight_dist, max_slant / 2)
            else:
                offset_dist = max_slant / 2
    except (TypeError, IndexError, AttributeError, ValueError, ZeroDivisionError):
        max_detect = mount.get_gimbal_data().max_detect_distance
        if max_detect > alt:
            offset_dist = math.sqrt(max_detect ** 2 - alt ** 2)
        else:
            return ApproachPlan(
                kind=ApproachKind.OFFSET,
                approach_location=target,
                offset_distance=0.0,
                orbit_radius=OFFSET_LOITER_RADIUS_M,
            )
        _log.info(f"OFFSET(fallback): alt={alt:.0f}m max_detect={max_detect:.0f}m → offset={offset_dist:.0f}m")
        offset_loc = _offset_location(target, drone_loc, offset_dist)
        return ApproachPlan(
            kind=ApproachKind.OFFSET,
            approach_location=offset_loc,
            offset_distance=offset_dist,
            orbit_radius=OFFSET_LOITER_RADIUS_M,
        )

    interval_str = f"interval=[{interval[0]:.0f},{interval[1]:.0f}]m " if interval else ""
    _log.info(
        f"OFFSET: alt={alt:.0f}m pitch={pitch_deg:.0f}° fy={fy:.0f} "
        f"{interval_str}→ offset={offset_dist:.0f}m loiter_r={OFFSET_LOITER_RADIUS_M:.0f}m"
    )
    offset_loc = _offset_location(target, drone_loc, offset_dist)
    # Carry the fixed-camera radius so navigation cannot retain a much larger
    # radius from a preceding ORBIT navigation_task.
    return ApproachPlan(
        kind=ApproachKind.OFFSET,
        approach_location=offset_loc,
        offset_distance=offset_dist,
        orbit_radius=OFFSET_LOITER_RADIUS_M,
    )


def _compute_orbit_plan(
    target: Location,
    mount: CameraMount,
    alt: float,
    class_id: int,
    orbit_limits: Optional[OrbitNavigationLimits] = None,
    recognition_px: Optional[float] = None,
) -> ApproachPlan:
    """Compatibility wrapper over the focused orbit planner."""
    return compute_orbit_plan(
        target,
        mount,
        alt,
        class_id,
        orbit_limits,
        recognition_px,
        fallback_radius_m=OFFSET_LOITER_RADIUS_M,
        logger=_log,
    )


def _offset_location(
    target: Location,
    drone_loc: Location,
    offset_dist: float,
) -> Location:
    """Compute offset point behind target along approach bearing."""
    az, _, _ = pymap3d.geodetic2aer(
        target.lat,
        target.lng,
        0,
        drone_loc.lat,
        drone_loc.lng,
        drone_loc.alt,
    )
    back_bearing = (az + 180) % 360
    origin = (target.lat, target.lng)
    dest = geodesic(meters=offset_dist).destination(
        origin,
        bearing=back_bearing,
    )
    return Location(dest.latitude, dest.longitude, target.alt)
