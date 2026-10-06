"""Source-ordered coordinate scoring for navigation evaluation."""

from __future__ import annotations

import heapq
import math

from eval_navigation_models import ClosestApproach, PositionSample, PoiLocation


EARTH_RADIUS_M = 6_371_008.8
COORDINATE_SCORE_RATE_HZ = 30.0
COORDINATE_SCORE_MAX_SOURCE_DT_S = 0.15
COORDINATE_SCORE_MAX_SPATIAL_GAP_M = 5.0
COORDINATE_SCORE_MIN_SAMPLES = 3
COORDINATE_SCORE_REORDER_WINDOW_S = COORDINATE_SCORE_MAX_SOURCE_DT_S


def horizontal_distance_m(
    first_lat: float,
    first_lon: float,
    second_lat: float,
    second_lon: float,
) -> float:
    """Return great-circle horizontal distance between two coordinates."""
    lat1 = math.radians(first_lat)
    lat2 = math.radians(second_lat)
    latitude_delta = lat2 - lat1
    longitude_delta = math.radians(second_lon - first_lon)
    haversine = (
        math.sin(latitude_delta / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(longitude_delta / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(
        min(1.0, math.sqrt(haversine))
    )


def _local_vector(
    sample: PositionSample,
    poi: PoiLocation,
) -> tuple[float, float, float]:
    north = math.radians(sample.lat_deg - poi.lat_deg) * EARTH_RADIUS_M
    east = (
        math.radians(sample.lon_deg - poi.lon_deg)
        * EARTH_RADIUS_M
        * math.cos(math.radians(poi.lat_deg))
    )
    return north, east, sample.abs_alt_m - poi.abs_alt_m


def _closest_on_segment(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> ClosestApproach:
    delta = tuple(end[index] - start[index] for index in range(3))
    denominator = sum(component * component for component in delta)
    fraction = 0.0
    if denominator > 1e-12:
        fraction = max(
            0.0,
            min(
                1.0,
                -sum(start[index] * delta[index] for index in range(3))
                / denominator,
            ),
        )
    closest = tuple(
        start[index] + fraction * delta[index] for index in range(3)
    )
    horizontal = math.hypot(closest[0], closest[1])
    vertical = abs(closest[2])
    return ClosestApproach(
        math.hypot(horizontal, vertical),
        horizontal,
        vertical,
    )


def _same_position(first: PositionSample, second: PositionSample) -> bool:
    return (
        first.lat_deg == second.lat_deg
        and first.lon_deg == second.lon_deg
        and first.abs_alt_m == second.abs_alt_m
        and first.rel_alt_m == second.rel_alt_m
    )


class CoordinateScorer:
    """Score source-time-ordered telemetry against one requested coordinate."""

    def __init__(self, poi: PoiLocation) -> None:
        self._poi = poi
        self._pending: list[tuple[float, int, PositionSample]] = []
        self._arrival_seq = 0
        self._max_seen_source_s: float | None = None
        self._watermark_s: float | None = None
        self._previous: tuple[float, float, float] | None = None
        self._previous_sample: PositionSample | None = None
        self._best: ClosestApproach | None = None
        self._received_count = 0
        self._accepted_count = 0
        self._duplicate_count = 0
        self._reordered_count = 0
        self._late_count = 0
        self._max_source_gap_s = 0.0
        self._max_spatial_gap_m = 0.0
        self._invalid_reasons: list[str] = []

    @property
    def result(self) -> ClosestApproach | None:
        self._release(math.inf)
        return self._best

    @property
    def sample_count(self) -> int:
        self._release(math.inf)
        return self._accepted_count

    @property
    def received_sample_count(self) -> int:
        return self._received_count

    @property
    def duplicate_sample_count(self) -> int:
        self._release(math.inf)
        return self._duplicate_count

    @property
    def reordered_sample_count(self) -> int:
        self._release(math.inf)
        return self._reordered_count

    @property
    def late_sample_count(self) -> int:
        self._release(math.inf)
        return self._late_count

    @property
    def max_source_gap_s(self) -> float:
        self._release(math.inf)
        return self._max_source_gap_s

    @property
    def max_spatial_gap_m(self) -> float:
        self._release(math.inf)
        return self._max_spatial_gap_m

    @property
    def certification_error(self) -> str | None:
        self._release(math.inf)
        reasons = list(self._invalid_reasons)
        if self._accepted_count < COORDINATE_SCORE_MIN_SAMPLES:
            reasons.append(
                "insufficient position cadence evidence: "
                f"{self._accepted_count} samples "
                f"(need {COORDINATE_SCORE_MIN_SAMPLES})"
            )
        return "; ".join(reasons) if reasons else None

    @property
    def certifiable(self) -> bool:
        return self.certification_error is None

    def _invalidate(self, reason: str) -> None:
        if reason not in self._invalid_reasons:
            self._invalid_reasons.append(reason)

    def add(self, sample: PositionSample) -> None:
        self._received_count += 1
        source = sample.source_time_s
        if source is None or not math.isfinite(source):
            self._invalidate("position sample has no finite MAVLink source timestamp")
            return
        if (
            self._max_seen_source_s is not None
            and source
            < self._max_seen_source_s - COORDINATE_SCORE_REORDER_WINDOW_S
        ):
            self._invalidate(
                "position source clock jumped backwards beyond the reorder window"
            )
            return
        if self._max_seen_source_s is not None and source < self._max_seen_source_s:
            self._reordered_count += 1
        self._max_seen_source_s = (
            source
            if self._max_seen_source_s is None
            else max(self._max_seen_source_s, source)
        )
        heapq.heappush(self._pending, (source, self._arrival_seq, sample))
        self._arrival_seq += 1
        self._release(
            self._max_seen_source_s - COORDINATE_SCORE_REORDER_WINDOW_S
        )

    def _release(self, horizon_s: float) -> None:
        while self._pending and self._pending[0][0] <= horizon_s:
            _, _, sample = heapq.heappop(self._pending)
            self._score(sample)

    def _score(self, sample: PositionSample) -> None:
        source = sample.source_time_s
        assert source is not None
        if self._watermark_s is not None and source <= self._watermark_s:
            self._handle_old_sample(sample, source)
            return
        current = _local_vector(sample, self._poi)
        candidate = self._candidate(current, source)
        if self._best is None or candidate.dist_3d_m < self._best.dist_3d_m:
            self._best = candidate
        self._previous = current
        self._previous_sample = sample
        self._watermark_s = source
        self._accepted_count += 1

    def _handle_old_sample(self, sample: PositionSample, source: float) -> None:
        if source < self._watermark_s:
            self._late_count += 1
        elif self._previous_sample is not None and _same_position(
            self._previous_sample, sample
        ):
            self._duplicate_count += 1
        else:
            self._invalidate(
                "two position samples share a source timestamp with different positions"
            )

    def _candidate(
        self,
        current: tuple[float, float, float],
        source: float,
    ) -> ClosestApproach:
        if self._previous is None:
            return _closest_on_segment(current, current)
        delta = tuple(current[index] - self._previous[index] for index in range(3))
        spatial_gap = math.sqrt(sum(value * value for value in delta))
        source_gap = source - self._watermark_s
        self._max_spatial_gap_m = max(self._max_spatial_gap_m, spatial_gap)
        self._max_source_gap_s = max(self._max_source_gap_s, source_gap)
        valid = self._segment_valid(source_gap, spatial_gap)
        return _closest_on_segment(self._previous if valid else current, current)

    def _segment_valid(self, source_gap: float, spatial_gap: float) -> bool:
        valid = True
        if source_gap > COORDINATE_SCORE_MAX_SOURCE_DT_S:
            self._invalidate(
                f"position source-time gap {source_gap:.3f}s exceeds "
                f"{COORDINATE_SCORE_MAX_SOURCE_DT_S:.3f}s"
            )
            valid = False
        if spatial_gap > COORDINATE_SCORE_MAX_SPATIAL_GAP_M:
            self._invalidate(
                f"position spatial gap {spatial_gap:.3f}m exceeds "
                f"{COORDINATE_SCORE_MAX_SPATIAL_GAP_M:.3f}m"
            )
            valid = False
        return valid
