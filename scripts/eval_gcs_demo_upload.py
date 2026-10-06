"""Mission-upload helpers for the already-running companion workflow."""

from __future__ import annotations

import hashlib
import json
import math
from copy import deepcopy
from typing import Iterable

from navpy.args.navigation_poi_args import is_nav_poi_command
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonValue


MISSION_SHIFT_LAT_DEG = 0.0002
MISSION_SHIFT_LON_DEG = -0.0002


def _object(value: JsonValue, label: str) -> dict[str, JsonValue]:
    if not isinstance(value, dict):
        raise TypeError(f"{label} must be an object")
    return value


def _list(value: JsonValue, label: str) -> list[JsonValue]:
    if not isinstance(value, list):
        raise TypeError(f"{label} must be a list")
    return value


def _number(value: JsonValue, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    return number


def _coordinate(value: JsonValue, label: str) -> dict[str, JsonValue]:
    source = _object(value, label)
    coordinate: dict[str, JsonValue] = {
        "lat": round(_number(source.get("lat"), f"{label}.lat"), 7),
        "lon": round(_number(source.get("lon"), f"{label}.lon"), 7),
    }
    location_type = source.get("type")
    if location_type is not None:
        if type(location_type) is not str or not location_type:
            raise TypeError(f"{label}.type must be a non-empty string")
        coordinate["type"] = location_type
    return coordinate


def _coordinates(value: JsonValue, label: str) -> list[JsonValue]:
    return [
        _coordinate(item, f"{label}[{index}]")
        for index, item in enumerate(_list(value, label))
    ]


def _optional_coordinate(value: JsonValue, label: str) -> JsonValue:
    return None if value is None else _coordinate(value, label)


def mission_assignment(
    mission_value: JsonValue,
    *,
    expected_sys_id: int,
    zone_index: int,
) -> dict[str, JsonValue]:
    """Translate the downloaded mission shape back into the frontend API shape."""
    mission = _object(mission_value, f"vehicle {expected_sys_id} mission")
    if mission.get("sys_id") != expected_sys_id:
        raise ValueError(
            f"mission sys_id {mission.get('sys_id')!r} != {expected_sys_id}"
        )
    raw_waypoints = _list(mission.get("waypoints"), "mission waypoints")
    if not raw_waypoints:
        raise RegressionError(f"vehicle {expected_sys_id} mission is empty")
    waypoints: list[JsonValue] = []
    for index, raw in enumerate(raw_waypoints):
        row = _object(raw, f"mission waypoint {index}")
        if not is_nav_poi_command(row.get("command")):
            raise RegressionError(
                f"vehicle {expected_sys_id} mission waypoint {index} is not "
                "a NAV_WAYPOINT and cannot be losslessly re-uploaded"
            )
        waypoints.append(_coordinate(row, f"mission waypoint {index}"))
    corridor_end = mission.get("corridor_end_index")
    if corridor_end is None:
        corridor_count = 0
    elif type(corridor_end) is int and 0 <= corridor_end <= len(waypoints):
        corridor_count = corridor_end
    else:
        raise ValueError("mission corridor_end_index is invalid")
    altitude_m = _number(mission.get("altitude_m"), "mission altitude_m")
    return {
        "sys_id": expected_sys_id,
        "zone_index": zone_index,
        "waypoints": waypoints,
        "altitude_m": altitude_m,
        "corridor_count": corridor_count,
        "corridor_altitude_m": altitude_m if corridor_count else None,
        "search_pattern": mission.get("search_pattern") or "distributed",
        "polygon": _coordinates(mission.get("polygon"), "mission polygon"),
        "corridor_backbone": _coordinates(
            mission.get("corridor_backbone"),
            "mission corridor_backbone",
        ),
        "launch_point": _optional_coordinate(
            mission.get("launch_point"),
            "mission launch_point",
        ),
        "default_delivery_hub": _optional_coordinate(
            mission.get("default_delivery_hub"),
            "mission default_delivery_hub",
        ),
    }


def capture_mission_assignments(
    context: StackContext,
) -> list[dict[str, JsonValue]]:
    return [
        mission_assignment(
            context.api.get_json(
                f"/api/vehicles/{sys_id}/mission",
                timeout=90.0,
            ),
            expected_sys_id=sys_id,
            zone_index=index,
        )
        for index, sys_id in enumerate(context.sys_ids)
    ]


def capture_nav_start_bindings(
    context: StackContext,
    nav_last_wp_by_sys_id: dict[int, int],
) -> list[dict[str, JsonValue]]:
    """Resolve each configured NAV ordinal to its uploaded MAVLink sequence."""
    expected_ids = set(context.sys_ids)
    if set(nav_last_wp_by_sys_id) != expected_ids:
        raise ValueError("nav-start binding sysids do not match the stack")
    bindings: list[dict[str, JsonValue]] = []
    for sys_id in context.sys_ids:
        mission = _object(
            context.api.get_json(
                f"/api/vehicles/{sys_id}/mission",
                timeout=90.0,
            ),
            f"vehicle {sys_id} uploaded mission",
        )
        if mission.get("sys_id") != sys_id:
            raise ValueError(f"uploaded mission sys_id mismatch for {sys_id}")
        mission_count = mission.get("mission_count")
        if type(mission_count) is not int or mission_count <= 0:
            raise RegressionError(
                f"vehicle {sys_id} uploaded mission has invalid mission_count"
            )
        ordinal = nav_last_wp_by_sys_id[sys_id]
        if type(ordinal) is not int or ordinal <= 0:
            raise ValueError(f"vehicle {sys_id} nav_last_wp must be positive")
        matches: list[dict[str, JsonValue]] = []
        for index, raw in enumerate(
            _list(mission.get("waypoints"), "uploaded mission waypoints")
        ):
            row = _object(raw, f"uploaded mission waypoint {index}")
            if (
                row.get("nav_waypoint_ordinal") == ordinal
                and is_nav_poi_command(row.get("command"))
            ):
                matches.append(row)
        if len(matches) != 1:
            raise RegressionError(
                f"vehicle {sys_id} NAV ordinal {ordinal} resolved to "
                f"{len(matches)} mission rows"
            )
        sequence = matches[0].get("mission_sequence")
        if type(sequence) is not int or sequence <= 0:
            raise RegressionError(
                f"vehicle {sys_id} NAV ordinal {ordinal} has invalid "
                "mission_sequence"
            )
        bindings.append({
            "sys_id": sys_id,
            "nav_waypoint_ordinal": ordinal,
            "mission_sequence": sequence,
            "mission_count": mission_count,
        })
    return bindings


def shifted_mission_assignments(
    assignments: Iterable[dict[str, JsonValue]],
    *,
    lat_delta: float = MISSION_SHIFT_LAT_DEG,
    lon_delta: float = MISSION_SHIFT_LON_DEG,
) -> list[dict[str, JsonValue]]:
    """Translate a mission so stale companion-local mission state is observable."""
    shifted = deepcopy(list(assignments))
    for assignment in shifted:
        for field in ("waypoints", "polygon", "corridor_backbone"):
            for coordinate in _list(assignment.get(field), field):
                _shift_coordinate(_object(coordinate, field), lat_delta, lon_delta)
        for field in ("launch_point", "default_delivery_hub"):
            coordinate = assignment.get(field)
            if coordinate is not None:
                _shift_coordinate(
                    _object(coordinate, field),
                    lat_delta,
                    lon_delta,
                )
    return shifted


def _shift_coordinate(
    coordinate: dict[str, JsonValue],
    lat_delta: float,
    lon_delta: float,
) -> None:
    coordinate["lat"] = round(
        _number(coordinate.get("lat"), "coordinate.lat") + lat_delta,
        7,
    )
    coordinate["lon"] = round(
        _number(coordinate.get("lon"), "coordinate.lon") + lon_delta,
        7,
    )


def upload_mission_assignments(
    context: StackContext,
    assignments: list[dict[str, JsonValue]],
) -> None:
    status, raw = context.api.post_json(
        "/api/vehicles/upload",
        {"assignments": assignments},
        timeout=240.0,
    )
    payload = _object(raw, "mission upload response")
    results = _list(payload.get("results"), "mission upload results")
    failures = []
    for result_value in results:
        result = _object(result_value, "mission upload result")
        if result.get("done") is not True or result.get("error") is not None:
            failures.append(result)
    if (
        status != 200
        or payload.get("status") != "complete"
        or len(results) != len(assignments)
        or failures
    ):
        raise RegressionError(f"mission upload failed: {payload}")


def mission_assignment_digest(
    assignments: Iterable[dict[str, JsonValue]],
) -> str:
    payload = json.dumps(
        list(assignments),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


__all__ = [
    "MISSION_SHIFT_LAT_DEG",
    "MISSION_SHIFT_LON_DEG",
    "capture_mission_assignments",
    "capture_nav_start_bindings",
    "mission_assignment",
    "mission_assignment_digest",
    "shifted_mission_assignments",
    "upload_mission_assignments",
]
