"""Scenario missions: strict upload-API fixtures, home, and sysid binding."""

from __future__ import annotations

import math
import struct
from collections.abc import Mapping, Sequence
from copy import deepcopy
from typing import cast

from scripts.eval_gcs_demo_models import positive_int
from scripts.eval_gcs_demo_ports import JsonValue
from navpy.plugins.vehicle.ardupilot.nav_waypoints import is_nav_poi_command


# Upload-API mission fields: models.VehicleAssignment minus sys_id/zone_index.
MISSION_FIELDS = frozenset({
    "waypoints",
    "altitude_m",
    "corridor_count",
    "corridor_altitude_m",
    "search_pattern",
    "polygon",
    "corridor_backbone",
    "launch_point",
    "default_delivery_hub",
})
# Mission items carry lat/lon as int(degrees * 1e7).
_COORDINATE_SCALE = 10_000_000
_EARTH_RADIUS_M = 6_371_000.0
# One encoding quantum (1e-7 degree) of latitude, in metres.
_COORDINATE_QUANTUM_M = math.radians(1 / _COORDINATE_SCALE) * _EARTH_RADIUS_M


def exact_keys(
    payload: Mapping[str, JsonValue],
    expected: set[str],
    label: str,
) -> None:
    actual = set(payload)
    if actual != expected:
        raise ValueError(f"{label} keys must be {sorted(expected)}, got {sorted(actual)}")


def _finite(value: JsonValue, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _coordinate(value: JsonValue, label: str, *, typed: bool = False) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    exact_keys(value, {"lat", "lon", "type"} if typed else {"lat", "lon"}, label)
    lat = _finite(value["lat"], f"{label}.lat")
    lon = _finite(value["lon"], f"{label}.lon")
    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        raise ValueError(f"{label} is out of range")
    for name, degrees in (("lat", lat), ("lon", lon)):
        # The builder truncates, so only exactly encodable values read back unchanged.
        if int(degrees * _COORDINATE_SCALE) / _COORDINATE_SCALE != degrees:
            raise ValueError(f"{label}.{name} does not survive mission-item encoding")
    if typed and (type(value["type"]) is not str or not value["type"]):
        raise ValueError(f"{label}.type must be a non-empty string")


def _coordinates(value: JsonValue, label: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    for index, item in enumerate(value):
        _coordinate(item, f"{label}[{index}]")
    return value


def _mission(raw: JsonValue, label: str) -> Mapping[str, JsonValue]:
    """Validate one upload-API mission so its onboard readback is exact."""
    if not isinstance(raw, dict):
        raise ValueError(f"{label} must be an object")
    exact_keys(raw, set(MISSION_FIELDS), label)
    waypoints = _coordinates(raw["waypoints"], f"{label}.waypoints")
    if not waypoints:
        raise ValueError(f"{label} needs at least one waypoint")
    altitude = _finite(raw["altitude_m"], f"{label}.altitude_m")
    if altitude <= 0.0:
        raise ValueError(f"{label}.altitude_m must be positive")
    # MAVLink carries altitude as float32.
    if struct.unpack("<f", struct.pack("<f", altitude))[0] != altitude:
        raise ValueError(f"{label}.altitude_m does not survive float32 encoding")
    corridor_count = raw["corridor_count"]
    if type(corridor_count) is not int or not 0 <= corridor_count <= len(waypoints):
        raise ValueError(f"{label}.corridor_count is invalid")
    # The mission parser reads the corridor altitude back as the zone altitude.
    corridor_altitude = raw["altitude_m"] if corridor_count else None
    if raw["corridor_altitude_m"] != corridor_altitude:
        raise ValueError(f"{label}.corridor_altitude_m must be {corridor_altitude!r}")
    if type(raw["search_pattern"]) is not str or not raw["search_pattern"]:
        raise ValueError(f"{label}.search_pattern must be a non-empty string")
    _coordinates(raw["polygon"], f"{label}.polygon")
    _coordinates(raw["corridor_backbone"], f"{label}.corridor_backbone")
    if raw["launch_point"] is not None:
        _coordinate(raw["launch_point"], f"{label}.launch_point")
    if raw["default_delivery_hub"] is not None:
        _coordinate(raw["default_delivery_hub"], f"{label}.default_delivery_hub", typed=True)
    return raw


def _distance_m(a: Mapping[str, float], b: Mapping[str, float]) -> float:
    lat_a, lat_b = math.radians(a["lat"]), math.radians(b["lat"])
    d_lat = lat_b - lat_a
    d_lon = math.radians(b["lon"] - a["lon"])
    h = math.sin(d_lat / 2) ** 2 + math.cos(lat_a) * math.cos(lat_b) * math.sin(d_lon / 2) ** 2
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(h))


def search_path_m(waypoints: Sequence[Mapping[str, float]]) -> float:
    """Length of one search track, first row to last (transit excluded)."""
    return sum(_distance_m(a, b) for a, b in zip(waypoints, waypoints[1:]))


def _require_equal_search_paths(missions: Sequence[Mapping[str, JsonValue]]) -> None:
    """The planner requirement: every UAV gets an equal search path.

    Exact up to encoding: each row may sit one quantum off the planner's
    point, so two tracks may differ by two quanta per row.
    """
    tracks = [cast(list[dict[str, float]], m["waypoints"]) for m in missions]
    lengths = [search_path_m(track) for track in tracks]
    tolerance = 2 * _COORDINATE_QUANTUM_M * max(len(track) for track in tracks)
    if max(lengths) - min(lengths) > tolerance:
        rounded = [round(length, 2) for length in lengths]
        raise ValueError(f"scenario search paths must be equal, got {rounded} m")


def scenario_missions(
    raw: JsonValue,
    slot_names: Sequence[str],
    owner_slot: str,
    poi_ordinals: Sequence[int],
) -> tuple[Mapping[str, JsonValue], ...]:
    """One mission per slot; the POI ordinals select distinct owner track rows."""
    if not isinstance(raw, list) or len(raw) != len(slot_names):
        raise ValueError("scenario manifest requires exactly one mission per slot")
    missions = tuple(
        _mission(item, f"scenario mission {name}") for name, item in zip(slot_names, raw)
    )
    _require_equal_search_paths(missions)
    track = cast(list[dict[str, float]], missions[list(slot_names).index(owner_slot)]["waypoints"])
    # The owner mission also ends with an appended default-delivery-hub NAV row,
    # which the GCS mission parser drops but NavPy still counts.
    if max(poi_ordinals) > len(track):
        raise ValueError(
            "scenario POI ordinals must select owner track rows, "
            "never the appended default-delivery-hub row"
        )
    coordinates = {(track[o - 1]["lat"], track[o - 1]["lon"]) for o in poi_ordinals}
    if len(coordinates) != len(poi_ordinals):
        raise ValueError("the scenario requires one unique POI coordinate per ordinal")
    return missions


def parse_home(raw: JsonValue) -> str:
    """SITL start point, LAT,LON,ALT,HEADING, as gcs_launch --home takes it."""
    if type(raw) is not str or len(raw.split(",")) != 4:
        raise ValueError("scenario home must be a LAT,LON,ALT,HEADING string")
    try:
        lat, lon, _alt, _heading = (
            _finite(float(field), "scenario home") for field in raw.split(",")
        )
    except ValueError as error:
        raise ValueError(f"invalid scenario home {raw!r}") from error
    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        raise ValueError("scenario home is out of range")
    return raw


def bind_missions(
    missions: Sequence[Mapping[str, JsonValue]],
    sys_ids: Sequence[int],
) -> list[dict[str, JsonValue]]:
    """Bind each slot's mission to its live sysid, in the upload-API shape."""
    if len(sys_ids) != len(missions):
        raise ValueError("one sysid per scenario mission is required")
    return [
        {
            "sys_id": positive_int("sys_id", sys_id),
            "zone_index": zone_index,
            **deepcopy(dict(mission)),
        }
        for zone_index, (sys_id, mission) in enumerate(zip(sys_ids, missions))
    ]


def owner_nav_coordinate(waypoint: JsonValue, ordinal: int) -> tuple[float, float]:
    if not isinstance(waypoint, dict):
        raise ValueError(f"owner NAV waypoint {ordinal} must be an object")
    try:
        raw_lat, raw_lon = waypoint["lat"], waypoint["lon"]
    except KeyError as error:
        raise ValueError(f"invalid owner NAV waypoint {ordinal}: {waypoint!r}") from error
    if any(
        isinstance(value, bool) or not isinstance(value, (int, float))
        for value in (raw_lat, raw_lon)
    ):
        raise ValueError(f"invalid owner NAV waypoint {ordinal}: {waypoint!r}")
    lat, lon = float(raw_lat), float(raw_lon)
    if not math.isfinite(lat) or not math.isfinite(lon) or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        raise ValueError(f"invalid owner NAV waypoint {ordinal} coordinate")
    return lat, lon


def owner_nav_waypoint_rows(
    owner_mission: Mapping[str, JsonValue],
) -> dict[int, Mapping[str, JsonValue]]:
    raw_waypoints = owner_mission.get("waypoints")
    if not isinstance(raw_waypoints, list):
        raise ValueError("owner mission waypoints must be a list")
    rows: dict[int, Mapping[str, JsonValue]] = {}
    previous_sequence = 0
    for index, raw in enumerate(raw_waypoints, start=1):
        if not isinstance(raw, Mapping):
            raise ValueError(f"owner mission waypoint row {index} must be an object")
        required = {
            "lat",
            "lon",
            "alt",
            "mission_sequence",
            "command",
            "nav_waypoint_ordinal",
        }
        if set(raw) != required:
            raise ValueError(
                f"owner mission waypoint row {index} keys must be {sorted(required)}"
            )
        sequence = positive_int("mission sequence", raw["mission_sequence"])
        ordinal = positive_int("NAV waypoint ordinal", raw["nav_waypoint_ordinal"])
        if sequence <= previous_sequence:
            raise ValueError("owner mission waypoint sequences must be increasing")
        if ordinal != index:
            raise ValueError("owner mission NAV waypoint ordinals must be contiguous")
        if not is_nav_poi_command(raw["command"]):
            raise ValueError("owner mission row is not MAV_CMD_NAV_WAYPOINT")
        previous_sequence = sequence
        rows[ordinal] = raw
    return rows


__all__ = [
    "MISSION_FIELDS",
    "bind_missions",
    "exact_keys",
    "owner_nav_coordinate",
    "owner_nav_waypoint_rows",
    "parse_home",
    "scenario_missions",
    "search_path_m",
]
