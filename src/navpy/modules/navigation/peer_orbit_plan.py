"""Orbit sizing for peer-assigned or fallback delivery location approach planning."""

from __future__ import annotations

import logging
import math
from typing import Optional, Protocol

import numpy as np

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits, r_nav_min
from navpy.modules.vision.vision_class_profile import (
    MIN_CONFIRM_PIXELS,
    MIN_DETECT_PIXELS,
    MIN_TRACK_PIXELS,
    get_class_detect_size,
)


class OrbitMount(Protocol):
    """The optics and zoom state required to size an orbit."""

    @property
    def has_zoom(self) -> bool: ...

    @property
    def base_fy(self) -> Optional[float]: ...

    @property
    def zoom_ratio(self) -> float: ...

    def get_k(self) -> np.ndarray: ...


# Vision-nav ORBIT floor, applied inside r_nav_min for the navigation-law
# dive (zoom mounts with orbit_limits, and the NavController terminal override).
# The raw dive-feasibility minimum can fall to ~185-250 m at normal altitudes;
# floor the orbit here so the tangential orbit-exit has enough run-in to finish
# the roll-out BEFORE the dive starts.
#
# 500 m (was 300 m): the extra ~200 m of run-in lets the orbit-exit bank settle
# to wings-level before the terminal dive, so the terminal LOS is not corrupted
# by residual roll. Live-confirmed (3-UAV eval): bank at the terminal bearing
# zero-crossing fell from 33-40 deg at 300 m to <5 deg at 500 m, collapsing the
# LATERAL miss from ~1 m to ~0.006 m. Camera is not the constraint: the
# siyi_zr10 10x zoom holds the target at ~48 px at 500 m (>>20 px confirm).
#
# NOTE (2026-07-14): 500 m does NOT on its own reach the <0.5 m 3D goal. Live
# 500 m SNAP is 1.6-3.4 m, now VERTICAL-dominated (long ~2.4 m, vert ~2.3 m,
# lat ~0). That vertical residual is a real-airframe terminal-flare limit
# (pitch/flight-path-response lag + TECS in the steep dive) that the offline
# point-mass certs do NOT reproduce (they hit <0.05 m at both 300 m and 500 m);
# the earlier "offline repro: 500 m -> 0.11 m" prediction was an artifact of
# that idealized plant. The lateral benefit above is the real, retained reason
# to keep 500 m; closing the vertical axis needs a live-SITL pitch/geometry
# change, not more standoff. approach_ready_dist = orbit_radius + PEER_APPROACH_MARGIN_M.
MIN_APPROACH_STANDOFF_M = 500.0

# Legacy no-zoom / no-orbit-limits acquisition floor ("never park closer than
# this"). Kept at 300 m: a fixed-camera mount confirms the target at its 1x
# MIN_CONFIRM_PIXELS slant (~426 m for class 0 at alt 100 m), which must NOT be
# pushed out to the 500 m vision-nav dive floor above -- doing so would put
# the target beyond the fixed camera's confirm range. Only the navigation-law dive
# (zoom + orbit_limits) uses the larger MIN_APPROACH_STANDOFF_M.
MIN_ACQUIRE_STANDOFF_M = 300.0
_ORBIT_PIXEL_MARGIN = 0.9

_DEFAULT_LOGGER = logging.getLogger("navpy.modules.navigation.peer_offset")


def compute_orbit_plan(
    target: Location,
    mount: OrbitMount,
    alt: float,
    class_id: int,
    orbit_limits: Optional[OrbitNavigationLimits] = None,
    recognition_px: Optional[float] = None,
    *,
    fallback_radius_m: float,
    logger: logging.Logger = _DEFAULT_LOGGER,
) -> ApproachPlan:
    """Compute an orbit-around-target plan.

    Gimbal pitch is ignored because the gimbal tracks the target at any angle.

    A zoom mount with ``orbit_limits`` uses the furthest standoff that still
    tracks reliably at 1x and reaches operator-identification size at maximum
    zoom, floored by the terminal dive-feasibility minimum::

        orbit_slant  = min(fy_1x  * size / MIN_TRACK_PIXELS,
                           fy_max * size / recognition_px)
                       * _ORBIT_PIXEL_MARGIN
        orbit_radius = max(sqrt(orbit_slant^2 - alt^2), r_nav_min)

    Otherwise the legacy camera-pixel sizing applies:
    a zoom mount uses ``MIN_DETECT_PIXELS`` and a fixed mount uses
    ``MIN_CONFIRM_PIXELS``.
    """
    if mount.has_zoom and orbit_limits is not None:
        return _compute_zoom_navigation_orbit_plan(
            target,
            mount,
            alt,
            class_id,
            orbit_limits,
            recognition_px,
            logger=logger,
        )
    return _compute_legacy_orbit_plan(
        target,
        mount,
        alt,
        class_id,
        fallback_radius_m=fallback_radius_m,
        logger=logger,
    )


def _compute_zoom_navigation_orbit_plan(
    target: Location,
    mount: OrbitMount,
    alt: float,
    class_id: int,
    orbit_limits: OrbitNavigationLimits,
    recognition_px: Optional[float],
    *,
    logger: logging.Logger,
) -> ApproachPlan:
    """Size the furthest zoom orbit while respecting the terminal dive floor."""
    nav_floor = r_nav_min(
        orbit_limits,
        alt,
        floor_m=MIN_APPROACH_STANDOFF_M,
    )
    recog_px = (
        float(recognition_px)
        if recognition_px and recognition_px > 0
        else float(MIN_CONFIRM_PIXELS)
    )
    fy_1x = mount.base_fy or 0.0
    if fy_1x <= 0:
        try:
            fy_1x = float(mount.get_k()[1, 1])
        except Exception:  # noqa: BLE001 - a flaky get_k must not crash planning
            fy_1x = 0.0
    zoom_ratio = mount.zoom_ratio
    if fy_1x > 0 and zoom_ratio > 0:
        class_size = get_class_detect_size(class_id)
        track_1x_slant = fy_1x * class_size / MIN_TRACK_PIXELS
        recog_max_slant = fy_1x * zoom_ratio * class_size / recog_px
        orbit_slant = (
            min(track_1x_slant, recog_max_slant) * _ORBIT_PIXEL_MARGIN
        )
        camera_radius = (
            math.sqrt(orbit_slant ** 2 - alt ** 2)
            if orbit_slant > alt
            else 0.0
        )
        orbit_radius = max(camera_radius, nav_floor)
        actual_slant = math.sqrt(orbit_radius ** 2 + alt ** 2)
        if actual_slant > track_1x_slant + 1.0:
            logger.warning(
                f"ORBIT(furthest): dive floor {nav_floor:.0f}m puts the "
                f"target below {MIN_TRACK_PIXELS}px at 1x "
                f"(slant {actual_slant:.0f}m > track {track_1x_slant:.0f}m, "
                f"alt={alt:.0f}m) — 1x tracking margin reduced."
            )
        if actual_slant > recog_max_slant + 1.0:
            logger.warning(
                f"ORBIT(furthest): dive floor {nav_floor:.0f}m exceeds "
                f"the max-zoom recognition range (slant {actual_slant:.0f}m "
                f"> recog {recog_max_slant:.0f}m, {recog_px:.0f}px) — "
                f"recognition NOT reachable at max zoom."
            )
        logger.info(
            f"ORBIT(furthest): alt={alt:.0f}m fy_1x={fy_1x:.0f} "
            f"zoom={zoom_ratio:.1f}x class={class_size:.1f}m "
            f"track_slant={track_1x_slant:.0f}m "
            f"recog_slant={recog_max_slant:.0f}m floor={nav_floor:.0f}m "
            f"→ orbit_r={orbit_radius:.0f}m"
        )
    else:
        orbit_radius = nav_floor
        logger.warning(
            f"ORBIT(furthest): unusable focal length "
            f"(base_fy={mount.base_fy}, zoom_ratio={zoom_ratio:.1f}); "
            f"using dive floor orbit_r={nav_floor:.0f}m."
        )
    return ApproachPlan(
        kind=ApproachKind.ORBIT,
        approach_location=target,
        offset_distance=0.0,
        orbit_radius=orbit_radius,
    )


def _compute_legacy_orbit_plan(
    target: Location,
    mount: OrbitMount,
    alt: float,
    class_id: int,
    *,
    fallback_radius_m: float,
    logger: logging.Logger,
) -> ApproachPlan:
    """Size an orbit using the legacy camera-pixel range."""
    try:
        k = mount.get_k()
        fy = float(k[1, 1])
        class_size = get_class_detect_size(class_id)

        if mount.has_zoom:
            pixel_threshold = MIN_DETECT_PIXELS
            threshold_label = "detect"
        else:
            pixel_threshold = MIN_CONFIRM_PIXELS
            threshold_label = "confirm"

        slant = fy * class_size / pixel_threshold * _ORBIT_PIXEL_MARGIN
        if slant > alt:
            orbit_radius = math.sqrt(slant ** 2 - alt ** 2)
        else:
            orbit_radius = fallback_radius_m
            logger.warning(
                f"ORBIT GEOMETRY UNREACHABLE: alt={alt:.0f}m > "
                f"slant={slant:.0f}m "
                f"(fy={fy:.0f}, class={class_size:.1f}m, "
                f"{threshold_label}_px={pixel_threshold}). "
                f"Falling back to OFFSET_LOITER_RADIUS_M="
                f"{fallback_radius_m}m — target may not be detectable."
            )

        orbit_radius = max(orbit_radius, MIN_ACQUIRE_STANDOFF_M)
        logger.info(
            f"ORBIT: alt={alt:.0f}m fy={fy:.0f} class={class_size:.1f}m "
            f"zoom={'Y' if mount.has_zoom else 'N'} "
            f"{threshold_label}_px={pixel_threshold} "
            f"slant={slant:.0f}m → orbit_r={orbit_radius:.0f}m"
        )
    except (TypeError, IndexError, AttributeError, ValueError, ZeroDivisionError):
        orbit_radius = fallback_radius_m

    return ApproachPlan(
        kind=ApproachKind.ORBIT,
        approach_location=target,
        offset_distance=0.0,
        orbit_radius=orbit_radius,
    )


__all__ = [
    "MIN_ACQUIRE_STANDOFF_M",
    "MIN_APPROACH_STANDOFF_M",
    "MIN_TRACK_PIXELS",
    "OrbitMount",
    "compute_orbit_plan",
]
