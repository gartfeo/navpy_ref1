from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PassObservation:
    entered_close_basin: bool
    passed: bool


class LegacyPassTracker:
    """Coordinate-distance pass memory for non-pure-vision navigation."""

    def __init__(
        self,
        *,
        close_distance_m: float,
        distance_epsilon_m: float,
        increase_samples: int,
        behind_bearing_deg: float,
    ) -> None:
        self._close_distance_m = close_distance_m
        self._distance_epsilon_m = distance_epsilon_m
        self._increase_samples = increase_samples
        self._behind_bearing_deg = behind_bearing_deg
        self.reset()

    def observe(self, distance_m: float) -> PassObservation:
        entered = distance_m <= self._close_distance_m and not self.close_observed
        if entered:
            self.close_observed = True
        if not self.approach_started:
            self.approach_started = True
            self.previous_distance_m = distance_m
            self.increase_count = 0
            return PassObservation(entered, False)
        if (
            self.close_observed
            and distance_m
            > float(self.previous_distance_m) + self._distance_epsilon_m
        ):
            self.increase_count += 1
        else:
            self.increase_count = 0
        self.previous_distance_m = distance_m
        return PassObservation(
            entered,
            self.increase_count >= self._increase_samples,
        )

    def has_crossing_evidence(self, measured_bearing_deg: object) -> bool:
        if self.increase_count >= 1:
            return True
        if not isinstance(measured_bearing_deg, (int, float)):
            return False
        return abs(float(measured_bearing_deg)) > self._behind_bearing_deg

    def reset(self) -> None:
        self.previous_distance_m: Optional[float] = None
        self.approach_started = False
        self.close_observed = False
        self.increase_count = 0
