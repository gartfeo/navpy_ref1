"""The offline truth cross-check of one approach (hitl_final_approach.md section 7 "Truth cross-check, offline" and
section 9 "Every ten-run series"): the closest approach of the bridge's ``link.csv`` path to the board centre Unreal
built, and whether it joins the contact the service reported.

Scoring only: it reads truth (the scenario's private files through contact_scene.read_dock and the plant's path) and
runs after every process stopped. Nothing in navpy reads it. The path is the bridge's own log (one row per board
step), its altitude MSL plus the bridge's ellipsoid offset (the vehicle_state datum the contact service judges in).
Each step is a straight segment between two rows no more than MAX_SEGMENT_S apart in board time; the distance is the segment's closest point to the centre, in a
local level frame at the centre (metres; the run's few kilometres make the flat frame exact to millimetres).
"""
from __future__ import annotations

import csv
import math
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, Optional

from navpy.core.kernel.location import Location

TRUTH_MISS_MAX_M = 0.5          # plan: a pass needs an offline cross-check below 0.5 m
# Two 20 ms pose periods (step 26's segment join): the service judged the pose stream the bridge also logs, so its
# contact time and the logged crossing are within one joined segment of each other.
CONTACT_JOIN_S = 0.04
EARTH_RADIUS_M = 6_378_137.0    # WGS84 equatorial radius: the local frame's scale
# The longest step joined into a segment (my value: two 20 ms pose periods, as the contact service joins). The bridge
# logs every 5 ms; a longer hole, a board time going back (a reboot) or a repeated time is never bridged.
MAX_SEGMENT_S = 0.04
BOARD_TIME_RESOLUTION_S = 1e-6     # board time is whole microseconds: seconds carry float rounding


def _local(lat: float, lon: float, height: float, centre: Location) -> tuple[float, float, float]:
    north = math.radians(lat - centre.lat) * EARTH_RADIUS_M
    east = math.radians(lon - centre.lng) * EARTH_RADIUS_M * math.cos(math.radians(centre.lat))
    return east, north, height - centre.alt


def _segment_closest(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, float]:
    """The distance from the origin to segment ab and the fraction along it where it is closest."""
    d = [b[i] - a[i] for i in range(3)]
    length2 = sum(x * x for x in d)
    u = 0.0 if length2 == 0.0 else max(0.0, min(1.0, -sum(a[i] * d[i] for i in range(3)) / length2))
    return math.dist((0.0, 0.0, 0.0), [a[i] + u * d[i] for i in range(3)]), u


def path_rows(rows: Iterable[Mapping[str, str]], alt_offset_m: float,
              skipped: Optional[list] = None) -> list[tuple[float, float, float, float]]:
    """(board time s, lat, lon, ellipsoid height) of each logged step with a full pose; the others are counted in
    ``skipped`` (their neighbours join only when still within MAX_SEGMENT_S of each other)."""
    out = []
    for row in rows:
        try:
            values = [float(row[key]) for key in ("board_time_us", "lat_deg", "lon_deg", "alt_msl_m")]
        except (KeyError, TypeError, ValueError):
            values = [math.nan]
        if all(math.isfinite(v) for v in values):
            out.append((values[0] * 1e-6, values[1], values[2], values[3] + alt_offset_m))
        elif skipped is not None:
            skipped.append(row)
    return out


def closest_pass(path: list[tuple[float, float, float, float]], centre: Location) -> Optional[dict[str, Any]]:
    """The closest joined segment of the path to the centre: its distance and board time; None without one."""
    best = None
    points = [(t, _local(lat, lon, h, centre)) for t, lat, lon, h in path]
    for (t0, a), (t1, b) in zip(points, points[1:]):
        if not 0.0 < t1 - t0 <= MAX_SEGMENT_S + BOARD_TIME_RESOLUTION_S:          # a hole, a repeat or a reboot: never bridged
            continue
        distance, u = _segment_closest(a, b)
        if best is None or distance < best["distance_m"]:
            best = {"distance_m": distance, "board_time_s": t0 + u * (t1 - t0), "segment_s": [t0, t1]}
    return best


def cross_check(link_csv: Path, centre: Location, alt_offset_m: float, contact_board_time_s: Optional[float]) -> dict:
    """The run's truth verdict: closest pass below TRUTH_MISS_MAX_M and joined to the service's contact."""
    try:
        with open(link_csv, encoding="utf-8", newline="") as handle:
            skipped: list = []
            path = path_rows(csv.DictReader(handle), alt_offset_m, skipped)
    except OSError as exc:
        return {"ok": False, "problems": [f"bridge path unreadable: {exc}"]}
    closest = closest_pass(path, centre)
    problems = []
    if closest is None:
        problems.append("the bridge path has no segment of two poses within MAX_SEGMENT_S")
    elif not closest["distance_m"] < TRUTH_MISS_MAX_M:
        problems.append(f"closest truth pass {closest['distance_m']:.3f} m, not below {TRUTH_MISS_MAX_M} m")
    if contact_board_time_s is None:
        problems.append("no contact time to join")
    elif closest is not None and abs(closest["board_time_s"] - contact_board_time_s) > CONTACT_JOIN_S:
        problems.append(f"the closest pass at {closest['board_time_s']:.3f} s is not the contact at "
                        f"{contact_board_time_s:.3f} s (join {CONTACT_JOIN_S} s)")
    return {"ok": not problems, "problems": problems, "closest": closest, "poses": len(path), "skipped": len(skipped),
            "contact_board_time_s": contact_board_time_s}


__all__ = ["CONTACT_JOIN_S", "MAX_SEGMENT_S", "TRUTH_MISS_MAX_M", "closest_pass", "cross_check", "path_rows"]
