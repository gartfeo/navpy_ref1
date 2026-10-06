"""Measured ground-track recording for wind analysis.

Wind only means something relative to where a vehicle is actually MOVING.
Indexing a wind matrix by compass direction silently assumes every vehicle
approaches on the same heading; peers can approach a POI from any bearing, so
the same world wind is a headwind for one and a tailwind for another and the
two average into noise.

This records the ground velocity vector straight off GLOBAL_POSITION_INT
(``vx``/``vy``/``vz``, NED, cm/s) and reports the wind resolved into that
frame.  Evaluation only: nothing here feeds navigation.  AGENTS.md forbids the
command path from using ground speed, and this deliberately lives outside it,
in the evaluator, alongside the other scoring instruments.
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

from eval_navigation_models import PoiLocation  # noqa: E402
from eval_navigation_scoring import horizontal_distance_m  # noqa: E402


# Distance from the POI inside which the track is the final-approach track.
# Matches the window used for LOS-rate diagnosis: far enough out that the
# trajectory is still correctable, close enough that the vehicle has committed
# to its run rather than still turning onto it.
FINAL_APPROACH_TRACK_WINDOW_M = 200.0
# Below this ground speed the velocity vector's direction is dominated by noise
# rather than motion, so it is not a meaningful track.
MIN_TRACK_GROUND_SPEED_M_S = 1.0


@dataclass(frozen=True)
class TrackSample:
    source_time_s: float | None
    north_m_s: float
    east_m_s: float
    down_m_s: float
    range_m: float


def _wrap_180(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


class GroundTrackRecorder:
    """Accumulate the measured ground velocity vector during an scoring interval."""

    def __init__(self, poi: PoiLocation) -> None:
        self._poi = poi
        self._samples: list[TrackSample] = []

    def add(self, message: Any) -> bool:
        """Record one GLOBAL_POSITION_INT. Returns whether it was usable."""
        try:
            north = float(message.vx) / 100.0
            east = float(message.vy) / 100.0
            down = float(message.vz) / 100.0
            lat = float(message.lat) / 1e7
            lon = float(message.lon) / 1e7
            abs_alt_m = float(message.alt) / 1000.0
        except (AttributeError, TypeError, ValueError):
            return False
        if not all(map(math.isfinite, (north, east, down, lat, lon))):
            return False
        try:
            source_time_s = float(message.time_boot_ms) / 1000.0
        except (AttributeError, TypeError, ValueError):
            source_time_s = None
        horizontal = horizontal_distance_m(
            lat, lon, self._poi.lat_deg, self._poi.lon_deg
        )
        vertical = abs_alt_m - self._poi.abs_alt_m
        self._samples.append(TrackSample(
            source_time_s=source_time_s,
            north_m_s=north,
            east_m_s=east,
            down_m_s=down,
            range_m=math.hypot(horizontal, vertical),
        ))
        return True

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def _approach_samples(self) -> list[TrackSample]:
        """Everything up to the closest approach: the run-in, not the departure."""
        if not self._samples:
            return []
        closest_m = min(sample.range_m for sample in self._samples)
        last = max(
            index for index, sample in enumerate(self._samples)
            if sample.range_m == closest_m
        )
        return self._samples[: last + 1]

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ("src_t", "vn_m_s", "ve_m_s", "vd_m_s", "range_m")
            )
            for sample in self._samples:
                writer.writerow((
                    "" if sample.source_time_s is None else sample.source_time_s,
                    sample.north_m_s,
                    sample.east_m_s,
                    sample.down_m_s,
                    sample.range_m,
                ))

    def summary(
        self,
        wind_speed_mps: float,
        wind_dir_deg: float,
        *,
        window_m: float = FINAL_APPROACH_TRACK_WINDOW_M,
    ) -> dict[str, float | int | None]:
        """Resolve the wind into the measured final-approach track frame.

        ``wind_dir_deg`` follows the ArduPilot/meteorological convention: the
        direction the wind blows FROM.  A vehicle tracking toward ``track_deg``
        therefore meets a headwind when the wind comes from ahead of it, so

            headwind = speed * cos(wind_from - track)

        is positive for a headwind and negative for a tailwind, and

            crosswind = speed * sin(wind_from - track)

        is positive when the wind comes from the vehicle's right.
        """
        # The window below is a RANGE band, and a level fly-by leaves the
        # POI and re-enters that band on the way out. Inbound and outbound
        # then partly cancel: measured on a clean 0.08 m approach, samples
        # averaging 18.10 m/s of ground speed produced a 1.80 m/s resultant
        # pointing 284 deg, on a run flown due north. It squeaked past the
        # cancellation guard below and was reported as a track.
        #
        # An approach track ends at the closest approach, so cut there. Samples
        # tied at the minimum cannot be split into inbound and outbound, so
        # they stay in and the guard speaks instead of a rule inventing an
        # answer.
        approach = self._approach_samples()
        moving = [
            sample for sample in approach
            if math.hypot(sample.north_m_s, sample.east_m_s)
            >= MIN_TRACK_GROUND_SPEED_M_S
        ]
        final_approach = [
            sample for sample in moving if sample.range_m <= window_m
        ]
        used = final_approach or moving
        if not used:
            raise RuntimeError(
                f"no moving ground-track samples in {len(self._samples)} records"
            )
        # Average the VECTOR, not the heading: a circular quantity averaged as
        # scalars folds across the 0/360 wrap and yields a track the vehicle
        # never flew.
        mean_north = sum(s.north_m_s for s in used) / len(used)
        mean_east = sum(s.east_m_s for s in used) / len(used)
        resultant_m_s = math.hypot(mean_north, mean_east)
        # Every individual sample can be moving while the RESULTANT cancels --
        # an out-and-back leg averages to zero.  atan2 would still return a
        # confident angle, and the wind would be resolved against a track the
        # vehicle never flew.  Refuse instead of inventing one.
        if resultant_m_s < MIN_TRACK_GROUND_SPEED_M_S:
            raise RuntimeError(
                f"ground-track samples cancel: resultant {resultant_m_s:.3f} m/s "
                f"over {len(used)} moving samples; no single track was flown"
            )
        track_deg = math.degrees(math.atan2(mean_east, mean_north)) % 360.0
        relative_deg = _wrap_180(wind_dir_deg - track_deg)
        return {
            "sample_count": len(self._samples),
            "track_sample_count": len(used),
            "used_terminal_window": bool(final_approach),
            "window_m": window_m,
            "track_deg": track_deg,
            "ground_speed_m_s": resultant_m_s,
            "wind_speed_m_s": wind_speed_mps,
            "wind_from_deg": wind_dir_deg,
            "relative_wind_deg": relative_deg,
            "headwind_m_s": wind_speed_mps * math.cos(math.radians(relative_deg)),
            "crosswind_m_s": wind_speed_mps * math.sin(math.radians(relative_deg)),
        }


__all__ = [
    "GroundTrackRecorder",
    "MIN_TRACK_GROUND_SPEED_M_S",
    "FINAL_APPROACH_TRACK_WINDOW_M",
    "TrackSample",
]
