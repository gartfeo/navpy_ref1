"""Mission download and target-coordinate resolution."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from typing import Any

from pymavlink import mavutil

from eval_navigation_models import MissionItem, TargetExpectation, TargetLocation
from eval_navigation_telemetry import command_long, message_from_target, recv_target_message


def mission_item_from_message(message: Any) -> MissionItem:
    """Convert either MAVLink mission-item encoding into a complete record."""
    message_type = message.get_type()
    raw_x = getattr(message, "x", None)
    raw_y = getattr(message, "y", None)
    if raw_x is None or raw_y is None:
        latitude = longitude = None
    elif message_type == "MISSION_ITEM_INT":
        latitude = float(raw_x) / 1e7
        longitude = float(raw_y) / 1e7
    else:
        latitude = float(raw_x)
        longitude = float(raw_y)

    def number(name: str) -> float | None:
        raw = getattr(message, name, None)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return None
        return float(raw)

    def integer(name: str) -> int | None:
        value = number(name)
        return None if value is None else int(value)

    raw_altitude = getattr(message, "z", None)
    return MissionItem(
        seq=int(message.seq),
        command=int(message.command),
        frame=int(message.frame),
        lat_deg=latitude,
        lon_deg=longitude,
        mission_alt_m=(
            float(raw_altitude) if raw_altitude is not None else None
        ),
        param1=number("param1"),
        param2=number("param2"),
        param3=number("param3"),
        param4=number("param4"),
        current=integer("current"),
        autocontinue=integer("autocontinue"),
        mission_type=integer("mission_type"),
    )


def resolve_target_expectation(
    mission: Sequence[MissionItem],
    *,
    target_wp: int,
    target_rel_alt_m: float,
    home_abs_alt_m: float,
) -> TargetExpectation:
    """Resolve the requested NAV_WAYPOINT ordinal to detector-local target zero."""
    nav_items = [
        item
        for item in mission
        if item.seq > 0
        and item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    ]
    if target_wp < 1 or target_wp > len(nav_items):
        raise ValueError(
            f"target WP {target_wp} is unavailable; mission has "
            f"{len(nav_items)} NAV_WAYPOINT items"
        )
    selected = nav_items[target_wp - 1]
    if selected.lat_deg is None or selected.lon_deg is None:
        raise ValueError(
            f"target WP {target_wp} (mission seq {selected.seq}) "
            "has no coordinates"
        )
    location = TargetLocation(
        lat_deg=selected.lat_deg,
        lon_deg=selected.lon_deg,
        rel_alt_m=float(target_rel_alt_m),
        abs_alt_m=float(home_abs_alt_m) + float(target_rel_alt_m),
    )
    return TargetExpectation(target_wp, selected.seq, 1, 0, location)


def download_mission(
    master: Any,
    *,
    timeout_s: float = 5.0,
    retries: int = 3,
) -> list[MissionItem]:
    """Download every mission item from the evaluator vehicle."""
    count_message = None
    for _ in range(retries):
        master.mav.mission_request_list_send(master.target_system, 0)
        count_message = recv_target_message(master, "MISSION_COUNT", timeout_s)
        if count_message is not None:
            break
    if count_message is None:
        raise RuntimeError("mission download failed: no MISSION_COUNT")
    return [
        _download_item(master, sequence, timeout_s=timeout_s, retries=retries)
        for sequence in range(int(count_message.count))
    ]


def _download_item(
    master: Any,
    sequence: int,
    *,
    timeout_s: float,
    retries: int,
) -> MissionItem:
    for _ in range(retries):
        master.mav.mission_request_int_send(master.target_system, 0, sequence)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            candidate = master.recv_match(
                type=["MISSION_ITEM_INT", "MISSION_ITEM"],
                blocking=True,
                timeout=0.5,
            )
            if candidate is None or not message_from_target(
                candidate, master.target_system
            ):
                continue
            if int(candidate.seq) == sequence:
                return mission_item_from_message(candidate)
    raise RuntimeError(
        f"mission download failed: item {sequence} was not received"
    )


def resolve_home_abs_alt_m(
    master: Any,
    *,
    timeout_s: float = 5.0,
) -> float:
    """Resolve prelaunch home altitude from a fixed raw GPS sample."""
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        remaining_s = deadline_s - time.monotonic()
        gps = recv_target_message(
            master,
            "GPS_RAW_INT",
            min(0.5, max(0.0, remaining_s)),
        )
        candidate = _fixed_gps_alt_m(gps)
        if candidate is not None:
            return candidate

    raise RuntimeError(
        "home altitude unavailable: no fixed GPS_RAW_INT before timeout"
    )


def _fixed_gps_alt_m(gps: Any) -> float | None:
    if gps is None:
        return None
    try:
        fix_type = int(gps.fix_type)
        lat = int(gps.lat)
        lon = int(gps.lon)
        altitude_m = float(gps.alt) / 1000.0
    except (AttributeError, TypeError, ValueError):
        return None
    if fix_type < 3 or (lat == 0 and lon == 0):
        return None
    return altitude_m if math.isfinite(altitude_m) else None
