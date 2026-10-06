"""Truth closest-approach evidence from navigation, compact, and debug logs."""

from __future__ import annotations

import csv
import re
from pathlib import Path


_DISTANCE = r"(?:[0-9]+(?:\.[0-9]+)?)|inf|nan"
_COMPACT_SNAP_RE = re.compile(rf"3d=(?P<distance>{_DISTANCE})", re.IGNORECASE)
_NAVIGATION_SNAP_RE = re.compile(
    rf"SNAP\(VISION-NAV[^)]*\):\s*3d=(?P<distance>{_DISTANCE})",
    re.IGNORECASE,
)


def _payload(payload: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in payload.split(";"):
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key in result:
            raise ValueError(f"duplicate debug payload field {key!r}")
        result[key] = value
    return result


def navigation_snap_distance(navigation_text: str) -> float | None:
    matches = list(_NAVIGATION_SNAP_RE.finditer(navigation_text))
    return float(matches[-1].group("distance")) if matches else None


def parse_compact_snap_distances(path: Path) -> list[float]:
    distances: list[float] = []
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 3 or not row[1].startswith("SNAP("):
                continue
            match = _COMPACT_SNAP_RE.search(row[2])
            if match is None:
                raise ValueError("compact SNAP row lacks a 3d distance")
            distances.append(float(match.group("distance")))
    return distances


def parse_debug_snap_distances(path: Path) -> list[float]:
    distances: list[float] = []
    with path.open("r", encoding="utf-8", errors="strict", newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 3 or row[1] != "EVENT:SNAP_COMPONENTS":
                continue
            payload = _payload(row[2])
            raw = payload.get("dist_3d_m")
            if raw is None:
                raise ValueError("debug SNAP_COMPONENTS lacks dist_3d_m")
            try:
                distances.append(float(raw))
            except ValueError as error:
                raise ValueError(f"invalid debug SNAP distance {raw!r}") from error
    return distances


__all__ = [
    "navigation_snap_distance",
    "parse_compact_snap_distances",
    "parse_debug_snap_distances",
]
