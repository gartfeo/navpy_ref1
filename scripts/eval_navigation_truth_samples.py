"""SIM_STATE-to-sample conversion and the truth-track record format.

The strict half of truth scoring: which SIM_STATE messages are allowed to
become authoritative samples, and how every message (accepted or rejected) is
persisted to ``truth_track.csv`` so a run can be re-scored offline.
"""

from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import PositionSample  # noqa: E402

from navpy.modules.vehicle.pose_telemetry import (  # noqa: E402
    simulator_truth_source_time_s,
)


# "converted", not "accepted": the column says the message decoded into an
# authoritative sample, nothing about how the scorer later treated it
# (duplicate/reorder dispositions are stream-level and live in the verdict's
# scorer_* and collapsed_duplicate_arrivals counters; offline re-scoring from
# this CSV reproduces them exactly).
TRACK_FIELDS = (
    "received_wall_time_s",
    "source_time_s",
    "lat_deg",
    "lon_deg",
    "abs_alt_m",
    "scoring_active",
    "converted",
    "reason",
)


def _deg_e7_coordinates(message: Any) -> tuple[float, float] | None:
    """Coordinates from the int32 degE7 fields only, never the float fallback.

    The runtime parser (``sim_state_coordinates_deg``) tolerates stock
    firmware by falling back to the float32 ``lat``/``lon`` fields, whose
    ~0.6 m resolution would silently poison a centimetre-level score.  Truth
    scoring therefore requires the extension fields outright.
    """
    lat_int = getattr(message, "lat_int", None)
    lon_int = getattr(message, "lon_int", None)
    explicit = (
        isinstance(lat_int, int)
        and not isinstance(lat_int, bool)
        and isinstance(lon_int, int)
        and not isinstance(lon_int, bool)
        and (lat_int != 0 or lon_int != 0)
    )
    if not explicit:
        return None
    latitude = float(lat_int) * 1e-7
    longitude = float(lon_int) * 1e-7
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        return None
    return latitude, longitude


def truth_position_sample(
    message: Any,
    received_wall_time_s: float,
    home_abs_alt_m: float,
) -> tuple[PositionSample | None, str | None]:
    """Convert one SIM_STATE message to a scoreable sample, or say why not."""
    coordinates = _deg_e7_coordinates(message)
    if coordinates is None:
        return None, "sim_state degE7 coordinates absent"
    source_time_s = simulator_truth_source_time_s(message)
    if source_time_s is None:
        return None, "sim_state time_us absent or invalid"
    raw_alt = getattr(message, "alt", None)
    if isinstance(raw_alt, bool) or not isinstance(raw_alt, (int, float)):
        return None, "sim_state altitude absent"
    abs_alt_m = float(raw_alt)
    if not math.isfinite(abs_alt_m):
        return None, "sim_state altitude not finite"
    return (
        PositionSample(
            lat_deg=coordinates[0],
            lon_deg=coordinates[1],
            abs_alt_m=abs_alt_m,
            rel_alt_m=abs_alt_m - home_abs_alt_m,
            received_wall_time_s=received_wall_time_s,
            source_time_s=source_time_s,
        ),
        None,
    )


def _deg_e7_field(value: Any, limit_deg: float) -> float | None:
    """One degE7 int decoded independently, for audit rows only."""
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    decoded = float(value) * 1e-7
    return decoded if -limit_deg <= decoded <= limit_deg else None


def partial_truth_fields(
    message: Any,
) -> tuple[float | None, float | None, float | None, float | None]:
    """Whatever is independently decodable from a rejected message.

    A message missing ``time_us`` still carries auditable coordinates and
    vice versa; discarding them would make the track CSV useless for
    diagnosing exactly the messages that failed conversion.  Each field is
    decoded on its own -- a malformed longitude must not blank a valid
    latitude -- and the pair rules of the conversion path (not-both-zero) do
    not apply here: this is audit data, never a score input.
    """
    source_time_s = simulator_truth_source_time_s(message)
    lat_deg = _deg_e7_field(getattr(message, "lat_int", None), 90.0)
    lon_deg = _deg_e7_field(getattr(message, "lon_int", None), 180.0)
    raw_alt = getattr(message, "alt", None)
    abs_alt_m = None
    if (
        not isinstance(raw_alt, bool)
        and isinstance(raw_alt, (int, float))
        and math.isfinite(float(raw_alt))
    ):
        abs_alt_m = float(raw_alt)
    return source_time_s, lat_deg, lon_deg, abs_alt_m


@dataclass(frozen=True)
class TruthRecord:
    received_wall_time_s: float
    source_time_s: float | None
    lat_deg: float | None
    lon_deg: float | None
    abs_alt_m: float | None
    scoring_active: bool
    converted: bool
    reason: str | None


def _optional_float(text: str) -> float | None:
    return float(text) if text not in ("", "None") else None


def read_track_records(path: Path) -> list[TruthRecord]:
    """The inverse of ``write_track_csv``, for offline re-scoring.

    Reads every persisted row back so an offline consumer (the SIM_CPA
    comparison, a re-scoring tool) works from the same episode evidence
    the live run recorded -- one code path for both.
    """
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if tuple(header or ()) != TRACK_FIELDS:
            raise ValueError(
                f"unrecognized truth track header in {path}: {header!r}"
            )
        records: list[TruthRecord] = []
        for row in reader:
            if len(row) != len(TRACK_FIELDS):
                raise ValueError(f"malformed truth track row: {row!r}")
            records.append(TruthRecord(
                received_wall_time_s=float(row[0]),
                source_time_s=_optional_float(row[1]),
                lat_deg=_optional_float(row[2]),
                lon_deg=_optional_float(row[3]),
                abs_alt_m=_optional_float(row[4]),
                scoring_active=bool(int(row[5])),
                converted=bool(int(row[6])),
                reason=row[7] or None,
            ))
        return records


def write_track_csv(records: list[TruthRecord], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(TRACK_FIELDS)
        for record in records:
            writer.writerow((
                record.received_wall_time_s,
                record.source_time_s,
                record.lat_deg,
                record.lon_deg,
                record.abs_alt_m,
                int(record.scoring_active),
                int(record.converted),
                record.reason or "",
            ))


__all__ = [
    "TRACK_FIELDS",
    "TruthRecord",
    "partial_truth_fields",
    "read_track_records",
    "truth_position_sample",
    "write_track_csv",
]
