"""Format realism against a committed real DataFlash prefix.

Synthetic rows exercise every rejection state cheaply, but only a real
BIN proves FMT/FMTU decoding, logger ordering, and field packing (the
plan's test-boundary split).  See tests/fixtures/sim_cpa/README.md for
the fixture's provenance.
"""

from __future__ import annotations

from pathlib import Path

from scripts.eval_navigation_models import PositionSample
from scripts.eval_sim_cpa_bin import (
    EV_ENABLE, certify_epoch, parse_sim_cpa_records,
)
from scripts.eval_sim_cpa_block import (
    EVIDENCE_ACCEPTED, EVIDENCE_POI_MISMATCH,
)
from scripts.eval_sim_cpa_compare import common_episode, module_window_score

FIXTURE = (
    Path(__file__).resolve().parent.parent
    / "fixtures" / "sim_cpa" / "valA_370_prefix.BIN"
)
# The 2026-09-03 validation POI, bit-for-bit.
POI = (430242544, 340000000, 6010)


def test_real_prefix_parses_scpc_and_scpa() -> None:
    scpc, scpa = parse_sim_cpa_records(FIXTURE)
    assert len(scpc) == 1
    event = scpc[0]
    assert event["Ev"] == EV_ENABLE
    assert (event["LatE7"], event["LngE7"], event["AltCM"]) == POI
    assert len(scpa) == 246
    # Real rows honor the contract fields this harness certifies on.
    sequences = [row["Seq"] for row in scpa]
    assert sequences == sorted(sequences)
    assert all(isinstance(row["D3"], float) for row in scpa)


def test_real_prefix_certifies_with_real_interval_arithmetic() -> None:
    scpc, scpa = parse_sim_cpa_records(FIXTURE)
    evidence = certify_epoch(scpc, scpa, POI)
    assert evidence.status == EVIDENCE_ACCEPTED
    assert evidence.missing_seqs == ()
    assert evidence.epoch_us == scpc[0]["EpUS"]
    assert len(evidence.rows) == 246


def test_real_prefix_poi_mismatch_still_fails_closed() -> None:
    scpc, scpa = parse_sim_cpa_records(FIXTURE)
    wrong = (POI[0] + 1, POI[1], POI[2])
    evidence = certify_epoch(scpc, scpa, wrong)
    assert evidence.status == EVIDENCE_POI_MISMATCH


def test_prefix_coverage_reads_as_incomplete_never_truncated() -> None:
    """R15: a prefix has incomplete episode coverage; nothing may claim
    detected truncation from pymavlink's silence about the missing tail."""
    scpc, scpa = parse_sim_cpa_records(FIXTURE)
    evidence = certify_epoch(scpc, scpa, POI)
    # An scored window beyond the prefix's rows yields missing interval
    # rows -- reported as exactly that, never as a torn or truncated file.
    last_row_end_us = evidence.epoch_us + (evidence.last_seq + 1) * 20_000
    beyond = [
        PositionSample(
            lat_deg=43.0, lon_deg=34.0, abs_alt_m=500.0, rel_alt_m=0.0,
            received_wall_time_s=0.0,
            source_time_s=(last_row_end_us + offset_us) / 1e6,
        )
        for offset_us in (10_000_000, 50_000_000)
    ]
    episode = common_episode(beyond, evidence.epoch_us)
    assert episode is not None
    score, problems = module_window_score(evidence, episode)
    assert score is None
    assert "missing" in problems[0]
    assert not any("truncat" in problem for problem in problems)
