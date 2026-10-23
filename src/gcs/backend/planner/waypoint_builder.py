"""Convert track points to MAVLink mission items for VehicleMav upload."""
from __future__ import annotations

import logging
import math
import struct
from dataclasses import dataclass, field
from typing import Callable

from pymavlink.mavwp import MAVWPLoader
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_WAYPOINT,
    MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
)

# Shared mission metadata encoding — single source of truth
from navpy.plugins.vehicle.ardupilot.mission_encoding import (  # noqa: E402
    CORRIDOR_END_MARKER,
    SEARCH_PATTERN_IDS, SEARCH_PATTERN_NAMES,
    META_POLYGON_VERTEX, META_CORRIDOR_VERTEX, META_LAUNCH_POINT, META_DEFAULT_DELIVERY_HUB,
    encode_meta_z, decode_meta_z, encode_location_type_into_z, decode_location_type_from_z,
)


from gcs.backend.exchange_budget import held_within
from gcs.backend.planner.mission_items import build_mission
from navpy.core.ports.parameter_exchange import ExchangePolicy
from navpy.core.ports.vehicle_control_interfaces import VehicleMissionAccess


def load_mission_to_vehicle(vehicle, track_latlon: list[dict], altitude_m: float,
                            corridor_count: int = 0,
                            search_pattern: str = "distributed",
                            polygon: list[dict] | None = None,
                            corridor_backbone: list[dict] | None = None,
                            launch_point: dict | None = None,
                            corridor_altitude_m: float | None = None,
                            default_delivery_hub: dict | None = None,
                            takeoff_altitude_m: float | None = None) -> bool:
    """Build and upload a mission to a VehicleMav instance.

    Args:
        vehicle: VehicleMav instance
        track_latlon: list of {"lat": float, "lon": float}
        altitude_m: relative altitude in meters
        corridor_count: number of leading corridor waypoints
        search_pattern: search pattern name
        polygon: original planning polygon vertices (for round-trip)
        corridor_backbone: corridor backbone waypoints (for round-trip)
        launch_point: separate launch/home position (for round-trip)

    Returns:
        True on success
    """
    wp_loader = build_mission(track_latlon, altitude_m, vehicle.target_system,
                              corridor_count, search_pattern, polygon, corridor_backbone,
                              launch_point, corridor_altitude_m,
                              default_delivery_hub, takeoff_altitude_m)

    # Clear + load + upload is one transaction: hold the per-vehicle mission
    # lock so a concurrent probe/download can't clear or rebuild the loader
    # between loading the new items and uploading them.
    with vehicle.mission_lock:
        vehicle.clear_mission()
        vehicle.load_mission_items(wp_loader)
        return vehicle.upload_mission()


_log = logging.getLogger(__name__)

# Fields compared during post-upload verification.
# x/y use a tolerance because lat/lon round-trip through int(val*1e7)/1e7.
_EXACT_FIELDS = ("command", "param1", "z")
_LATLON_TOL = 2  # integer units of 1e-7 degrees (~0.01 mm)


@dataclass
class UploadResult:
    success: bool
    uploaded_count: int
    expected_count: int
    attempts: int
    error: str | None = None
    link_lost: bool = False


def upload_mission_with_retry(
    vehicle,
    track_latlon: list[dict],
    altitude_m: float,
    corridor_count: int = 0,
    search_pattern: str = "distributed",
    polygon: list[dict] | None = None,
    corridor_backbone: list[dict] | None = None,
    launch_point: dict | None = None,
    corridor_altitude_m: float | None = None,
    default_delivery_hub: dict | None = None,
    takeoff_altitude_m: float | None = None,
    max_retries: int = 3,
    on_progress: Callable[[str, int], None] | None = None,
    on_wp_progress: Callable[[int, int], None] | None = None,
    policy: ExchangePolicy | None = None,
) -> UploadResult:
    """Build, upload, and verify a mission with retries.

    ``policy`` bounds every transfer and the mission-lock wait (``radio_budget.md``,
    todo 105). A failed upload may still be stored (its final ACK lost), so it is
    read back before another clear and upload, and counts only on an exact match.

    Args:
        vehicle: VehicleMav instance
        track_latlon: list of {"lat": float, "lon": float}
        altitude_m: relative altitude in meters
        max_retries: number of attempts before giving up
        on_progress: callback(stage, attempt) for progress reporting
        on_wp_progress: callback(wp_sent, wp_total) for per-waypoint progress

    Returns:
        UploadResult with success/failure details
    """
    if not track_latlon:
        return UploadResult(success=True, uploaded_count=0, expected_count=0, attempts=0)

    wp_loader = build_mission(
        track_latlon, altitude_m, vehicle.target_system,
        corridor_count, search_pattern, polygon, corridor_backbone,
        launch_point, corridor_altitude_m, default_delivery_hub,
        takeoff_altitude_m,
    )
    expected_count = wp_loader.count()

    for attempt in range(1, max_retries + 1):
        # The whole clear -> load -> upload -> verify-download -> param-verify
        # sequence is one transaction: hold the per-vehicle mission lock so a
        # concurrent probe/download can't clear or rebuild the shared loader
        # mid-upload (which would corrupt the upload or the verification).
        with held_within(vehicle.mission_lock, policy) as held:
            if not held:
                return UploadResult(
                    success=False, uploaded_count=0, expected_count=expected_count,
                    attempts=attempt, error="Request deadline passed before the mission lock was free",
                )
            if on_progress:
                on_progress("clearing", attempt)
            vehicle.clear_mission()

            if on_progress:
                on_progress("uploading", attempt)
            vehicle.load_mission_items(wp_loader)
            ok = vehicle.upload_mission(on_wp_sent=on_wp_progress, policy=policy)

            downloaded_count = 0
            param_errors = None
            stored_anyway = False
            if ok or vehicle.link_ok:
                if on_progress:
                    on_progress("verifying", attempt)
                downloaded_count = vehicle.download_mission(policy=policy)
                if downloaded_count == expected_count:
                    if ok:
                        param_errors = verify_mission_params(vehicle, wp_loader)
                    else:
                        stored_anyway = not mission_mismatches(vehicle, wp_loader)

        # Interpret the attempt outside the lock (logging / retry control flow).
        if not ok:
            if not vehicle.link_ok:
                _log.warning("Upload attempt %d: link lost", attempt)
                return UploadResult(
                    success=False, uploaded_count=0, expected_count=expected_count,
                    attempts=attempt, error="Link lost during upload", link_lost=True,
                )
            if stored_anyway:
                _log.warning("Upload attempt %d: unconfirmed, but the readback matches exactly",
                             attempt)
                return UploadResult(
                    success=True, uploaded_count=downloaded_count,
                    expected_count=expected_count, attempts=attempt,
                )
            _log.warning("Upload attempt %d/%d failed, retrying...", attempt, max_retries)
            continue

        if downloaded_count != expected_count:
            _log.warning(
                "Count mismatch attempt %d/%d: expected %d, got %d",
                attempt, max_retries, expected_count, downloaded_count,
            )
            continue

        # Count matches — verify individual fields (warn only, don't block)
        for err in (param_errors or []):
            _log.warning("Param verify: %s", err)
        return UploadResult(
            success=True, uploaded_count=downloaded_count,
            expected_count=expected_count, attempts=attempt,
        )

    return UploadResult(
        success=False, uploaded_count=0, expected_count=expected_count,
        attempts=max_retries,
        error=f"Upload failed after {max_retries} attempts",
    )


#: What a stored item must equal for an unconfirmed upload to count (todo 105).
_STORED_FIELDS = ("frame", "command", "autocontinue", "param1", "param2", "param3", "param4",
                  "x", "y", "z")


def _float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def _same(expected: object, actual: object) -> bool:
    if isinstance(expected, float) or isinstance(actual, float):
        a, b = _float32(expected), _float32(actual)
        return a == b or (math.isnan(a) and math.isnan(b))
    return expected == actual


def mission_mismatches(vehicle: VehicleMissionAccess, wp_loader: MAVWPLoader) -> list[str]:
    """Every difference between the upload and the readback but home (seq 0).

    Exact, floats as float32 (the wire type): any difference, autopilot
    normalization included, means the upload is not known to be stored.
    """
    errors: list[str] = []
    for i in range(1, wp_loader.count()):
        expected = wp_loader.wp(i)
        actual = vehicle.get_mission_item(i)
        if actual is None:
            errors.append(f"seq {i}: missing after download")
            continue
        for name in _STORED_FIELDS:
            ev, av = getattr(expected, name, None), getattr(actual, name, None)
            if ev is None or av is None or not _same(ev, av):
                errors.append(f"seq {i}: {name} expected={ev} got={av}")
    return errors


def verify_mission_params(vehicle, wp_loader: MAVWPLoader) -> list[str]:
    """Compare uploaded mission items against downloaded ones field-by-field.

    Returns a list of mismatch descriptions (empty = all OK).
    """
    errors: list[str] = []
    count = wp_loader.count()
    for i in range(count):
        if i == 0:
            continue  # Home waypoint — vehicle overwrites with actual position
        expected = wp_loader.wp(i)
        actual = vehicle.get_mission_item(i)
        if actual is None:
            errors.append(f"seq {i}: missing after download")
            continue
        for field in _EXACT_FIELDS:
            ev = getattr(expected, field, None)
            av = getattr(actual, field, None)
            if ev is None:
                continue
            # Compare as float with small tolerance for z (encoded bitmask)
            if abs(float(ev) - float(av)) > 0.01:
                errors.append(f"seq {i}: {field} expected={ev} got={av}")
        # x/y are lat/lon * 1e7 as integers
        for field in ("x", "y"):
            ev = getattr(expected, field, 0)
            av = getattr(actual, field, 0)
            if abs(int(ev) - int(av)) > _LATLON_TOL:
                errors.append(f"seq {i}: {field} expected={ev} got={av}")
    return errors
