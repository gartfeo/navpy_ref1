"""Capture what NavPy sent, ArduPlane accepted, and the airframe achieved."""

from __future__ import annotations

import csv
import time
from pathlib import Path


class FlightControlTrace:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._rows: list[tuple[object, ...]] = []

    def sample(self, vehicle: object) -> None:
        target = getattr(vehicle, "attitude_target_debug", None)
        nav = getattr(vehicle, "nav_controller_output_debug", None)
        attitude = getattr(vehicle, "attitude", None)
        self._rows.append((
            time.time(),
            getattr(target, "roll", None),
            getattr(target, "pitch", None),
            getattr(target, "type_mask", None),
            getattr(target, "age_ms", None),
            getattr(nav, "nav_roll", None),
            getattr(nav, "nav_pitch", None),
            getattr(nav, "age_ms", None),
            getattr(attitude, "roll", None),
            getattr(attitude, "pitch", None),
        ))

    def write(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        with self._path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow((
                "wall_s", "sent_roll_deg", "sent_pitch_deg", "type_mask",
                "sent_age_ms", "nav_roll_deg", "nav_pitch_deg", "nav_age_ms",
                "actual_roll_deg", "actual_pitch_deg",
            ))
            writer.writerows(self._rows)


__all__ = ["FlightControlTrace"]
