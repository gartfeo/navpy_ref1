"""Common-episode alignment, reproduction admissibility, disagreement."""

from __future__ import annotations

import math

from scripts import eval_sim_cpa_bin as bin_mod
from scripts import eval_sim_cpa_compare as compare
from scripts.eval_navigation_models import PoiLocation
from scripts.eval_navigation_truth_cpa import segment_cpa
from scripts.eval_navigation_truth_samples import TruthRecord
from scripts.eval_sim_cpa_bin import EpochEvidence
from scripts.eval_sim_cpa_block import compare_scores

POI = PoiLocation(
    lat_deg=43.0, lon_deg=34.0, rel_alt_m=60.0, abs_alt_m=500.0
)
EPOCH_US = 60_000_000
# One degree of latitude in metres, matching the scorer's spherical model.
_LAT_M = math.radians(1.0) * 6_371_000.0


def _record(time_s: float, north_m: float, scoring_active: bool = True,
            converted: bool = True) -> TruthRecord:
    """A sample flying a straight north line over the POI."""
    return TruthRecord(
        received_wall_time_s=time_s,
        source_time_s=time_s,
        lat_deg=POI.lat_deg + north_m / _LAT_M,
        lon_deg=POI.lon_deg,
        abs_alt_m=POI.abs_alt_m + 5.0,
        scoring_active=scoring_active,
        converted=converted,
        reason=None,
    )


def _flyby_records(
    start_s: float = 100.0, end_s: float = 140.0, cpa_s: float = 120.0,
    step_s: float = 0.04, speed_mps: float = 25.0,
) -> list[TruthRecord]:
    records = []
    t = start_s
    while t <= end_s + 1e-9:
        records.append(_record(t, (t - cpa_s) * speed_mps))
        t += step_s
    return records


def _evidence_for(records: list[TruthRecord]) -> EpochEvidence:
    """Module rows synthesized from the same straight-line geometry."""
    canonical = compare.scoring_active_samples_from_records(records)
    start_us = round(canonical[0].source_time_s * 1e6)
    end_us = round(canonical[-1].source_time_s * 1e6)
    first = -(-(start_us - EPOCH_US) // bin_mod.INTERVAL_US)
    last = (end_us - EPOCH_US) // bin_mod.INTERVAL_US - 1
    rows = []
    for seq in range(first, last + 1):
        # The firmware logs the interpolated minimum WITHIN the interval,
        # so the model is the continuous minimum of the distance curve
        # over [start, end]: the clamp of the true CPA time.
        start_s = (EPOCH_US + seq * bin_mod.INTERVAL_US) / 1e6
        end_s = (EPOCH_US + (seq + 1) * bin_mod.INTERVAL_US) / 1e6
        t_min = min(max(120.0, start_s), end_s)
        north = (t_min - 120.0) * 25.0
        d3 = math.sqrt(north * north + 25.0)  # 5 m vertical offset
        rows.append({
            "TimeUS": EPOCH_US + (seq + 1) * bin_mod.INTERVAL_US,
            "Ep": 1, "Seq": seq, "Fl": 0, "CpaUS": round(t_min * 1e6),
            "D3": d3, "DH": abs(north), "DV": 5.0,
            "GCpUS": round(t_min * 1e6), "GD3": d3,
            "GDH": abs(north), "GDV": 5.0,
        })
    return EpochEvidence(
        status="accepted", epoch=1, epoch_us=EPOCH_US, rows=tuple(rows),
        first_seq=first, last_seq=last,
    )


def test_scoring_active_samples_filter_and_canonicalize() -> None:
    records = [
        _record(10.0, 0.0, scoring_active=False),
        _record(11.0, 0.0, converted=False),
        _record(13.0, 50.0),
        _record(12.0, 25.0),
        _record(12.0, 25.0),  # duplicate timestamp collapses
    ]
    canonical = compare.scoring_active_samples_from_records(records)
    assert [s.source_time_s for s in canonical] == [12.0, 13.0]


def test_common_episode_aligns_inward_to_the_grid() -> None:
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    episode = compare.common_episode(canonical, EPOCH_US)
    assert episode is not None
    # Boundaries land ON the grid, inside the scored span.
    assert (episode["start_us"] - EPOCH_US) % bin_mod.INTERVAL_US == 0
    assert (episode["end_us"] - EPOCH_US) % bin_mod.INTERVAL_US == 0
    assert episode["start_us"] >= round(canonical[0].source_time_s * 1e6)
    assert episode["end_us"] <= round(canonical[-1].source_time_s * 1e6)
    span = episode["end_us"] - episode["start_us"]
    assert episode["last_seq"] - episode["first_seq"] + 1 == (
        span // bin_mod.INTERVAL_US
    )


def test_common_episode_refuses_an_epoch_opened_mid_scoring_interval() -> None:
    """Shrinking the window would reintroduce the episode mismatch."""
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    late_epoch_us = round(canonical[0].source_time_s * 1e6) + 1
    assert compare.common_episode(canonical, late_epoch_us) is None


def test_module_window_score_requires_every_full_interval() -> None:
    records = _flyby_records()
    evidence = _evidence_for(records)
    canonical = compare.scoring_active_samples_from_records(records)
    episode = compare.common_episode(canonical, EPOCH_US)

    score, problems = compare.module_window_score(evidence, episode)
    assert problems == []
    assert abs(score["cpa_time_s"] - 120.0) < 0.02
    assert score["d3_m"] < 5.2

    rows = [dict(r) for r in evidence.rows]
    victim = episode["first_seq"] + 5
    gapped = EpochEvidence(
        status="accepted", epoch=1, epoch_us=EPOCH_US,
        rows=tuple(r for r in rows if r["Seq"] != victim),
        first_seq=evidence.first_seq, last_seq=evidence.last_seq,
    )
    score, problems = compare.module_window_score(gapped, episode)
    assert score is None and "missing" in problems[0]

    rows[6]["Fl"] = bin_mod.FLAG_PARTIAL
    partial = EpochEvidence(
        status="accepted", epoch=1, epoch_us=EPOCH_US, rows=tuple(rows),
        first_seq=evidence.first_seq, last_seq=evidence.last_seq,
    )
    score, problems = compare.module_window_score(partial, episode)
    assert score is None and "partial" in problems[0]


def test_stream_recompute_reproduces_an_interior_cpa() -> None:
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    official_cpa = segment_cpa(canonical, POI)
    official = {
        "dist_3d_m": official_cpa.dist_3d_m,
        "horizontal_m": 0.0,
        "vertical_m": 5.0,
        "cpa_source_time_s": official_cpa.source_time_s,
    }
    episode = compare.common_episode(canonical, EPOCH_US)
    score, problems = compare.stream_window_score(
        canonical, POI, episode, official
    )
    assert problems == []
    assert score["d3_m"] == official_cpa.dist_3d_m
    assert score["cpa_time_s"] == official_cpa.source_time_s


def test_stream_recompute_refuses_a_cpa_outside_the_episode() -> None:
    """A CPA the trim removed (or that sits at the very edge) means the two
    sides no longer score the same event."""
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    episode = compare.common_episode(canonical, EPOCH_US)
    outside = {
        "dist_3d_m": 5.0,
        "horizontal_m": 0.0,
        "vertical_m": 5.0,
        "cpa_source_time_s": episode["end_us"] / 1e6 + 1.0,
    }
    score, problems = compare.stream_window_score(
        canonical, POI, episode, outside
    )
    assert score is None
    assert "not interior" in problems[0]


def test_stream_recompute_refuses_a_wrong_official_value() -> None:
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    episode = compare.common_episode(canonical, EPOCH_US)
    wrong = {
        "dist_3d_m": 7.7,  # not what the clipped track reproduces
        "horizontal_m": 0.0,
        "vertical_m": 5.0,
        "cpa_source_time_s": 120.0,
    }
    score, problems = compare.stream_window_score(
        canonical, POI, episode, wrong
    )
    assert score is None
    assert "does not reproduce" in problems[0]


def test_disagreement_predicate_thresholds() -> None:
    # Zero-based stream values so the deltas are the module values
    # bit-for-bit: "exactly at threshold" is only meaningful when the
    # computed delta IS the threshold constant, not a decimal that float64
    # rounds past it.
    stream = {"d3_m": 0.0, "dh_m": 0.040, "dv_m": 0.030, "cpa_time_s": 0.0}

    at_threshold = compare_scores(
        {"d3_m": 0.010, "dh_m": 0.040, "dv_m": 0.030,
         "cpa_time_s": 0.001}, stream,
    )
    assert at_threshold["disagreement"] is False  # exactly-at agrees

    d3_over = compare_scores(
        {"d3_m": 0.0101, "dh_m": 0.040, "dv_m": 0.030,
         "cpa_time_s": 0.0}, stream,
    )
    assert d3_over["disagreement"] is True

    time_over = compare_scores(
        {"d3_m": 0.0, "dh_m": 0.040, "dv_m": 0.030,
         "cpa_time_s": 0.0011}, stream,
    )
    assert time_over["disagreement"] is True

    # The observed 2026-09-03 shape: components repartition by more than a
    # centimetre while d3 agrees -- recorded, never flagged.
    components_only = compare_scores(
        {"d3_m": 0.005, "dh_m": 0.026, "dv_m": 0.044,
         "cpa_time_s": 0.0}, stream,
    )
    assert components_only["disagreement"] is False
    assert abs(components_only["deltas"]["dv_m"]) > 0.010


def test_compare_scores_fails_closed_on_non_finite_values() -> None:
    """90_review finding 1, last line of defense: a NaN delta satisfies no
    threshold, so without this guard corruption reads as agreement and
    walks into A/B eligibility."""
    good = {"d3_m": 0.05, "dh_m": 0.04, "dv_m": 0.03, "cpa_time_s": 100.0}
    comparison = compare_scores(dict(good, d3_m=float("nan")), good)
    assert comparison["status"] == "inadmissible"
    assert comparison["disagreement"] is None
    assert "module.d3_m" in comparison["error"]

    comparison = compare_scores(good, dict(good, cpa_time_s=float("inf")))
    assert comparison["status"] == "inadmissible"
    assert "stream.cpa_time_s" in comparison["error"]


def test_windowed_comparison_end_to_end_agrees() -> None:
    records = _flyby_records()
    canonical = compare.scoring_active_samples_from_records(records)
    evidence = _evidence_for(records)
    official_cpa = segment_cpa(canonical, POI)
    truth_block = {
        "dist_3d_m": official_cpa.dist_3d_m,
        "horizontal_m": 0.0,
        "vertical_m": 5.0,
        "cpa_source_time_s": official_cpa.source_time_s,
        "certification_error": None,
    }
    episode, module_score, comparison = compare.windowed_comparison(
        evidence, records, POI, truth_block
    )
    assert episode is not None and module_score is not None
    assert comparison["status"] == "compared"
    assert comparison["disagreement"] is False
    assert abs(comparison["deltas"]["cpa_time_s"]) < 0.02


def test_windowed_comparison_passes_through_rejected_evidence() -> None:
    rejected = EpochEvidence(status="fault", errors=("boom",))
    episode, module_score, comparison = compare.windowed_comparison(
        rejected, [], POI, {}
    )
    assert episode is None and module_score is None
    assert comparison["status"] == "inadmissible"
    assert "fault" in comparison["error"]
