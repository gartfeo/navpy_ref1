"""Certification and scoring behavior of the simulator-truth recorder."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from types import SimpleNamespace

from scripts import eval_navigation_truth as truth
from scripts.eval_navigation_models import PoiLocation


EARTH_RADIUS_M = 6_371_008.8
M_PER_DEG_LAT = math.radians(1.0) * EARTH_RADIUS_M
POI = PoiLocation(
    lat_deg=43.0, lon_deg=34.0, rel_alt_m=60.0, abs_alt_m=500.0
)
HOME_ABS_ALT_M = 440.0


class _Clock:
    """Injectable monotonic clock so trailing-freshness tests are exact."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _message(
    north_m: float,
    east_m: float,
    alt_m: float,
    time_s: float,
) -> SimpleNamespace:
    lat_deg = POI.lat_deg + north_m / M_PER_DEG_LAT
    lon_deg = POI.lon_deg + east_m / (
        M_PER_DEG_LAT * math.cos(math.radians(POI.lat_deg))
    )
    return SimpleNamespace(
        lat_int=round(lat_deg * 1e7),
        lon_int=round(lon_deg * 1e7),
        alt=alt_m,
        time_us=round(time_s * 1e6),
    )


def _feed_pass(
    recorder: truth.TruthRecorder,
    *,
    start_time_s: float = 100.0,
    dt_s: float = 0.025,
    count: int = 400,
    east_offset_m: float = 0.05,
    alt_offset_m: float = 0.03,
    scoring_active: bool = True,
    speed_mps: float = 40.0,
    arrivals_per_sample: int = 1,
) -> float:
    """Fly a straight north-to-south pass abeam the POI; return last wall."""
    wall = 1000.0
    half = count // 2
    for index in range(count):
        north = (half - index) * speed_mps * dt_s
        message = _message(
            north,
            east_offset_m,
            POI.abs_alt_m + alt_offset_m,
            start_time_s + index * dt_s,
        )
        wall = 1000.0 + index * dt_s
        for _ in range(arrivals_per_sample):
            recorder.add_message(message, wall, scoring_active=scoring_active)
    return wall


def _recorder(clock: _Clock | None = None) -> truth.TruthRecorder:
    if clock is None:
        return truth.TruthRecorder(POI, HOME_ABS_ALT_M)
    return truth.TruthRecorder(POI, HOME_ABS_ALT_M, monotonic_now=clock)


def test_nominal_track_is_certified_and_scores_the_pass() -> None:
    recorder = _recorder()
    _feed_pass(recorder)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["certification_error"] is None
    assert recorder.certified
    expected = math.hypot(0.05, 0.03)
    assert abs(verdict["dist_3d_m"] - expected) < 0.03
    assert 35.0 < verdict["delivered_rate_hz"] < 45.0
    assert verdict["closure_post_samples"] >= truth.TRUTH_CLOSURE_MIN_POST_SAMPLES
    assert verdict["closure_rise_m"] > 1.0
    assert verdict["cpa_source_time_s"] is not None
    assert verdict["cpa_segment_fraction"] is not None
    assert verdict["collapsed_duplicate_arrivals"] == 0
    assert verdict["finalized"] is True
    assert verdict["rejected_samples"] == 0
    assert recorder.closure_ready()


def test_pre_scoring_interval_close_pass_does_not_score() -> None:
    recorder = _recorder()
    _feed_pass(recorder, scoring_active=False, east_offset_m=0.05)
    _feed_pass(recorder, start_time_s=120.0, east_offset_m=5.0)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["certification_error"] is None
    assert verdict["dist_3d_m"] > 4.0
    assert verdict["pre_engagement_samples"] == 400


def test_duplicate_arrivals_after_a_source_stall_are_not_fresh() -> None:
    """A stalled source that re-sends its final sample must fail the
    trailing-freshness gate: exact duplicates collapse out of every other
    certification check (rate, gap, span all read the canonical times), so
    mere ARRIVAL must not advance the live edge (Codex round-5 finding 3
    reproduced this against the arrival-driven code)."""
    clock = _Clock()
    recorder = _recorder(clock)
    _feed_pass(recorder)
    # The exact final sample of the pass: index 399 of 400 at 40 m/s, 25 ms.
    final = _message(
        (200 - 399) * 40.0 * 0.025,
        0.05,
        POI.abs_alt_m + 0.03,
        100.0 + 399 * 0.025,
    )
    for index in range(10):
        clock.now += 1.0
        recorder.add_message(final, 1010.0 + index, scoring_active=True)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["certification_error"] is not None
    assert "stale at finalize" in verdict["certification_error"]


def test_duplicates_of_a_live_source_do_not_break_freshness() -> None:
    """Same duplicates, but the source clock keeps advancing between them:
    the live edge follows the source, so certification stays clean."""
    clock = _Clock()
    recorder = _recorder(clock)
    _feed_pass(recorder, arrivals_per_sample=2)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["certification_error"] is None
    assert verdict["collapsed_duplicate_arrivals"] == 400


def test_never_finalized_fails_certification() -> None:
    recorder = _recorder()
    _feed_pass(recorder)
    verdict = recorder.verdict()
    assert verdict["finalized"] is False
    assert "never finalized" in verdict["certification_error"]


def test_finalize_first_declaration_binds_the_episode_end() -> None:
    """A fleet verdict pass or a failure salvage re-declares finalize long
    after the aircraft's own episode ended; the stamp must not move. The
    first fleet acceptance flight lost 2 of 3 aircraft exactly here: their
    re-stamped freshness gates measured the fleet's ~12 s finish-time
    spread instead of stream liveness."""
    clock = _Clock()
    recorder = _recorder(clock)
    _feed_pass(recorder)
    recorder.finalize()
    clock.now += 12.0
    recorder.finalize()  # bookkeeping echo; must not move the stamp
    assert recorder.verdict()["certification_error"] is None

    control_clock = _Clock()
    control = _recorder(control_clock)
    _feed_pass(control)
    control_clock.now += 12.0
    control.finalize()  # a genuinely late FIRST declaration stays stale
    error = control.verdict()["certification_error"]
    assert error is not None and "stale at finalize" in error


def test_degraded_rate_fails_certification() -> None:
    recorder = _recorder()
    _feed_pass(recorder, dt_s=0.05, count=400)
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None and "delivered rate" in error


def test_duplicated_slow_stream_is_not_certified_as_nominal() -> None:
    """A 20 Hz stream with every packet doubled is 40 Hz only by arrivals."""
    recorder = _recorder()
    _feed_pass(recorder, dt_s=0.05, count=300, arrivals_per_sample=2)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["engaged_samples"] == 300
    assert verdict["collapsed_duplicate_arrivals"] == 300
    assert verdict["scorer_duplicates"] == 300
    error = verdict["certification_error"]
    assert error is not None and "delivered rate" in error


def test_reordered_arrivals_do_not_corrupt_closure() -> None:
    """A swap inside the tolerance window must not fake or break the tail."""
    recorder = _recorder()
    half = 400 // 2
    messages = [
        _message(
            (half - index) * 40.0 * 0.025,
            0.05,
            POI.abs_alt_m + 0.03,
            100.0 + index * 0.025,
        )
        for index in range(400)
    ]
    # Deliver the chronologically last sample one slot early: in arrival
    # order the track would end on an older point.
    messages[-1], messages[-2] = messages[-2], messages[-1]
    for index, message in enumerate(messages):
        recorder.add_message(message, 1000.0 + index * 0.025, scoring_active=True)
    recorder.finalize()
    verdict = recorder.verdict()
    assert verdict["certification_error"] is None
    assert verdict["scorer_reordered"] == 1
    assert verdict["collapsed_duplicate_arrivals"] == 0
    assert verdict["closure_post_samples"] >= truth.TRUTH_CLOSURE_MIN_POST_SAMPLES


def test_single_gap_fails_the_truth_gate_but_not_the_scorer() -> None:
    recorder = _recorder()
    for index in range(300):
        time_s = 100.0 + index * 0.025 + (0.1 if index >= 150 else 0.0)
        north = (150 - index) * 1.0
        recorder.add_message(
            _message(north, 0.05, POI.abs_alt_m + 0.03, time_s),
            1000.0 + index * 0.025,
            scoring_active=True,
        )
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None
    assert "truth source gap" in error
    assert "truth stream:" not in error


def test_trailing_stall_fails_certification() -> None:
    clock = _Clock()
    recorder = _recorder(clock)
    _feed_pass(recorder)
    clock.now = 10.0
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None and "stale at finalize" in error


def test_missing_time_us_is_rejected_per_sample() -> None:
    recorder = _recorder()
    message = _message(100.0, 0.05, POI.abs_alt_m, 100.0)
    del message.time_us
    recorder.add_message(message, 1000.0, scoring_active=True)
    verdict = recorder.verdict()
    assert verdict["rejected_samples"] == 1
    assert "insufficient scoring-window truth samples" in verdict["certification_error"]


def test_float_only_coordinates_are_never_authoritative() -> None:
    recorder = _recorder()
    message = SimpleNamespace(
        lat_int=0, lon_int=0, lat=43.001, lon=34.0,
        alt=500.0, time_us=100_000_000,
    )
    recorder.add_message(message, 1000.0, scoring_active=True)
    assert recorder.verdict()["rejected_samples"] == 1


def test_boolean_deg_e7_fields_are_rejected() -> None:
    sample, reason = truth.truth_position_sample(
        SimpleNamespace(lat_int=True, lon_int=1, alt=500.0, time_us=1_000_000),
        1000.0,
        HOME_ABS_ALT_M,
    )
    assert sample is None
    assert "degE7" in reason


def test_truncated_track_fails_closure() -> None:
    recorder = _recorder()
    for index in range(300):
        recorder.add_message(
            _message(300.0 - index, 0.05, POI.abs_alt_m, 100.0 + index * 0.025),
            1000.0 + index * 0.025,
            scoring_active=True,
        )
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None and "samples after the minimum" in error
    assert not recorder.closure_ready()


def test_cpa_endpoint_sample_is_not_post_cpa_evidence() -> None:
    """A monotonic approach ends AT its minimum; the final sample is the CPA
    itself (projection fraction 1.0), never tail evidence."""
    from scripts.eval_navigation_truth_cpa import closure_evidence
    from scripts.eval_navigation_models import PositionSample

    samples = [
        PositionSample(
            lat_deg=POI.lat_deg + (300.0 - index) / M_PER_DEG_LAT,
            lon_deg=POI.lon_deg,
            abs_alt_m=POI.abs_alt_m,
            rel_alt_m=POI.abs_alt_m - HOME_ABS_ALT_M,
            received_wall_time_s=1000.0 + index * 0.025,
            source_time_s=100.0 + index * 0.025,
        )
        for index in range(20)
    ]
    post_samples, rise, cpa = closure_evidence(samples, POI)
    assert cpa is not None and cpa.fraction == 1.0
    assert post_samples == 0
    assert rise == 0.0


def test_flat_tail_fails_closure_rise() -> None:
    recorder = _recorder()
    for index in range(300):
        north = max(1.0, 150.0 - index)
        recorder.add_message(
            _message(north, 0.05, POI.abs_alt_m, 100.0 + index * 0.025),
            1000.0 + index * 0.025,
            scoring_active=True,
        )
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None and "distance rises" in error
    assert not recorder.closure_ready()


def test_overflow_invalidates_the_score(monkeypatch) -> None:
    monkeypatch.setattr(truth, "TRUTH_TRACK_MAX_RECORDS", 50)
    recorder = _recorder()
    _feed_pass(recorder)
    recorder.finalize()
    error = recorder.verdict()["certification_error"]
    assert error is not None and "exceeded 50 records" in error


def test_write_track_records_rejections(tmp_path: Path) -> None:
    recorder = _recorder()
    recorder.add_message(
        _message(100.0, 0.05, POI.abs_alt_m, 100.0), 1000.0, scoring_active=False
    )
    bad = _message(99.0, 0.05, POI.abs_alt_m, 100.025)
    del bad.time_us
    recorder.add_message(bad, 1000.025, scoring_active=True)
    path = tmp_path / "truth_track.csv"
    recorder.write_track(path)
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2
    assert rows[0]["engaged"] == "0" and rows[0]["converted"] == "1"
    assert rows[1]["converted"] == "0"
    assert "time_us" in rows[1]["reason"]
    # A rejected message keeps every field that still decodes: here the
    # coordinates and altitude survive while the missing source time is blank.
    assert rows[1]["lat_deg"] != ""
    assert rows[1]["abs_alt_m"] != ""
    assert rows[1]["source_time_s"] == ""


def test_partial_fields_decode_each_coordinate_independently() -> None:
    from scripts.eval_navigation_truth_samples import partial_truth_fields

    source_time_s, lat_deg, lon_deg, abs_alt_m = partial_truth_fields(
        SimpleNamespace(
            lat_int=430000000, lon_int="garbage", alt=500.0,
            time_us=1_000_000,
        )
    )
    assert lat_deg == 43.0
    assert lon_deg is None
    assert abs_alt_m == 500.0
    assert source_time_s == 1.0
