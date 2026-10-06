"""Mission validation for probing existing vehicle missions on connect."""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_TAKEOFF
from gcs.backend.planner.waypoint_builder import (
    CORRIDOR_END_MARKER,
    META_POLYGON_VERTEX, META_CORRIDOR_VERTEX, META_LAUNCH_POINT, META_FALLBACK_DELIVERY_LOCATION,
    decode_meta_z, decode_location_type_from_z,
)
from navpy.args.navigation_target_args import is_nav_target_command

log = logging.getLogger(__name__)


@dataclass
class MissionProbe:
    valid: bool
    item_count: int = 0
    track_count: int = 0
    search_pattern: str = "distributed"
    polygon: list[dict] = field(default_factory=list)


@dataclass
class FleetValidation:
    valid: bool
    zone_count: int = 0
    invalid_vehicles: list[int] = field(default_factory=list)
    search_pattern_mismatch: bool = False


def parse_mission_items(vehicle, count: int, sys_id: int) -> dict:
    """Parse already-downloaded mission items into a full response dict.

    This is the shared parsing logic used by both the probe (on connect)
    and the GET download endpoint.  The caller must have called
    ``vehicle.download_mission()`` first so that ``get_mission_item``
    returns valid data.
    """
    waypoints = []
    altitude_m = None
    corridor_end_index = None
    polygon_vertices = []
    corridor_backbone = []
    launch_point = None
    fallback_delivery_location = None
    last_meta_z = 0.0
    route_index = 0
    target_ordinal = 0
    in_metadata = False
    for i in range(count):
        wp = vehicle.get_mission_item(i)
        if wp is None:
            continue
        if i == 0 or wp.command == MAV_CMD_NAV_TAKEOFF:
            continue
        if wp.command == CORRIDOR_END_MARKER:
            if not in_metadata:
                corridor_end_index = route_index
                in_metadata = True
            if wp.z != 0:
                last_meta_z = wp.z
            meta_type = int(wp.param1)
            lat = wp.x / 1e7
            lon = wp.y / 1e7
            if meta_type == META_POLYGON_VERTEX:
                polygon_vertices.append({"lat": lat, "lon": lon})
            elif meta_type == META_CORRIDOR_VERTEX:
                corridor_backbone.append({"lat": lat, "lon": lon})
            elif meta_type == META_LAUNCH_POINT:
                launch_point = {"lat": lat, "lon": lon}
            elif meta_type == META_FALLBACK_DELIVERY_LOCATION:
                fallback_delivery_location = {"lat": lat, "lon": lon}
                location_type = decode_location_type_from_z(wp.z)
                if location_type:
                    fallback_delivery_location["type"] = location_type
            continue
        in_metadata = False
        loc = vehicle.get_mission_item_location(i)
        if loc is None:
            continue
        route_index += 1
        row = {
            "lat": loc.lat,
            "lon": loc.lng,
            "alt": loc.alt,
            "mission_sequence": i,
            "command": int(wp.command),
        }
        if is_nav_target_command(wp.command):
            target_ordinal += 1
            row["nav_waypoint_ordinal"] = target_ordinal
        waypoints.append(row)
        if altitude_m is None:
            altitude_m = loc.alt

    search_pattern, dock_classes = decode_meta_z(last_meta_z)

    # Trim trailing fallback-location NAV_WAYPOINT duplicate
    if fallback_delivery_location and waypoints:
        last = waypoints[-1]
        if (abs(last["lat"] - fallback_delivery_location["lat"]) < 1e-5 and
                abs(last["lon"] - fallback_delivery_location["lon"]) < 1e-5):
            waypoints.pop()

    return {
        "sys_id": sys_id,
        "waypoints": waypoints,
        "altitude_m": altitude_m or 100,
        "mission_count": count,
        "corridor_end_index": corridor_end_index,
        "search_pattern": search_pattern,
        "dock_classes": dock_classes,
        "polygon": polygon_vertices,
        "corridor_backbone": corridor_backbone,
        "launch_point": launch_point,
        "fallback_delivery_location": fallback_delivery_location,
    }


def validate_vehicle_mission(vehicle, sys_id: int, on_progress=None) -> tuple[MissionProbe, dict | None]:
    """Download and parse a vehicle's mission.

    *on_progress*, if given, is called as ``on_progress(current, total)`` as
    items arrive during the download (see VehicleMav.download_mission).

    Returns (probe, parsed_response_or_None).
    """
    # Hold the mission lock across download AND parse: parse_mission_items reads
    # the live shared loader, so without this a concurrent probe/download for the
    # same vehicle could clear/rebuild it mid-parse and yield a partial result.
    with vehicle.mission_lock:
        count = vehicle.download_mission(on_progress=on_progress)
        if count == 0:
            return MissionProbe(valid=False, item_count=0), None
        parsed = parse_mission_items(vehicle, count, sys_id)
    track_count = len(parsed["waypoints"])
    search_pattern = parsed["search_pattern"]
    polygon = parsed["polygon"]

    probe = MissionProbe(
        valid=track_count > 0,
        item_count=count,
        track_count=track_count,
        search_pattern=search_pattern,
        polygon=polygon,
    )
    return probe, parsed if probe.valid else None


def validate_fleet_missions(probes: dict[int, MissionProbe]) -> FleetValidation:
    """Cross-validate mission probes across all vehicles in a fleet."""
    if not probes:
        return FleetValidation(valid=False)

    invalid_vehicles = [sid for sid, p in probes.items() if not p.valid]
    valid_probes = {sid: p for sid, p in probes.items() if p.valid}

    if not valid_probes:
        return FleetValidation(
            valid=False,
            zone_count=0,
            invalid_vehicles=invalid_vehicles,
        )

    search_patterns = {p.search_pattern for p in valid_probes.values()}
    search_pattern_mismatch = len(search_patterns) > 1

    return FleetValidation(
        valid=len(invalid_vehicles) == 0 and not search_pattern_mismatch,
        zone_count=len(valid_probes),
        invalid_vehicles=invalid_vehicles,
        search_pattern_mismatch=search_pattern_mismatch,
    )
