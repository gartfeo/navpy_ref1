"""SCPC/SCPA certification state machine on synthetic parsed rows."""

from __future__ import annotations

from scripts import eval_sim_cpa_bin as bin_mod

POI = (430242544, 340000000, 6010)
EPOCH_US = 63_000_000


def _scpc(ev: int, ep: int = 1, ep_us: int = EPOCH_US,
          poi: tuple[int, int, int] = POI,
          time_us: int | None = None) -> dict:
    return {
        "TimeUS": EPOCH_US if time_us is None else time_us,
        "Ep": ep, "EpUS": ep_us, "Ev": ev,
        "LatE7": poi[0], "LngE7": poi[1], "AltCM": poi[2],
    }


def _row(seq: int, ep: int = 1, ep_us: int = EPOCH_US, fl: int = 0,
         d3: float = 100.0, time_us: int | None = None,
         cpa_us: int | None = None) -> dict:
    boundary = ep_us + (seq + 1) * bin_mod.INTERVAL_US
    if time_us is None:
        time_us = boundary
    if cpa_us is None:
        cpa_us = ep_us + seq * bin_mod.INTERVAL_US + 10_000
    return {
        "TimeUS": time_us, "Ep": ep, "Seq": seq, "Fl": fl,
        "CpaUS": cpa_us, "D3": d3, "DH": d3 * 0.9, "DV": d3 * 0.1,
        "GCpUS": cpa_us, "GD3": d3, "GDH": d3 * 0.9, "GDV": d3 * 0.1,
    }


def test_no_scpc_rejects() -> None:
    result = bin_mod.certify_epoch([], [_row(0)], POI)
    assert result.status == "no_scpc"


def test_poi_mismatch_rejects_with_observed_pois() -> None:
    other = (430242545, 340000000, 6010)  # one degE7 unit off
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE, poi=other)], [_row(0)], POI
    )
    assert result.status == "poi_mismatch"
    assert "430242545" in result.errors[0]


def test_two_matching_epochs_reject_as_ambiguous_never_by_recency() -> None:
    scpc = [
        _scpc(bin_mod.EV_ENABLE, ep=1),
        _scpc(bin_mod.EV_ENABLE, ep=2, ep_us=EPOCH_US + 10_000_000),
    ]
    result = bin_mod.certify_epoch(scpc, [_row(0)], POI)
    assert result.status == "ambiguous_epoch"


def test_fault_anywhere_in_the_epoch_rejects_it_whole() -> None:
    scpc = [_scpc(bin_mod.EV_ENABLE), _scpc(bin_mod.EV_FAULT)]
    result = bin_mod.certify_epoch(scpc, [_row(0)], POI)
    assert result.status == "fault"


def test_no_interval_rows_rejects() -> None:
    result = bin_mod.certify_epoch([_scpc(bin_mod.EV_ENABLE)], [], POI)
    assert result.status == "no_rows"


def test_duplicate_seq_rejects_outright() -> None:
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [_row(3), _row(3)], POI
    )
    assert result.status == "bad_interval_arithmetic"


def test_out_of_order_physical_rows_reject_never_repair() -> None:
    """90_review finding 2: sorting before validation would silently
    repair the exact corruption this check exists to catch."""
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [_row(1), _row(0)], POI
    )
    assert result.status == "bad_interval_arithmetic"
    assert "out-of-order" in result.errors[0]


def test_non_finite_or_negative_distances_reject() -> None:
    """90_review finding 1: abs(nan) > threshold is False downstream, so
    a NaN that reaches the comparison reads as agreement.  Certification
    is the primary gate; every distance field is held to it."""
    for value in (float("nan"), float("inf"), -1.0):
        bad = _row(0)
        bad["D3"] = value
        result = bin_mod.certify_epoch(
            [_scpc(bin_mod.EV_ENABLE)], [bad], POI
        )
        assert result.status == "bad_interval_arithmetic"
        assert "D3" in result.errors[0]

    poisoned_global = _row(0)
    poisoned_global["GDV"] = float("nan")
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [poisoned_global], POI
    )
    assert result.status == "bad_interval_arithmetic"
    assert "GDV" in result.errors[0]


def test_off_boundary_time_rejects_the_contract() -> None:
    bad = _row(2)
    bad["TimeUS"] += 1
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [bad], POI
    )
    assert result.status == "bad_interval_arithmetic"
    assert "exact interval boundary" in result.errors[0]


def test_partial_row_close_time_must_stay_inside_its_interval() -> None:
    start = EPOCH_US + 5 * bin_mod.INTERVAL_US
    ok = _row(5, fl=bin_mod.FLAG_PARTIAL | bin_mod.FLAG_FINAL,
              time_us=start + 7_000, cpa_us=start + 3_000)
    accepted = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [_row(4), ok], POI
    )
    assert accepted.status == "accepted"

    bad = dict(ok, TimeUS=start)  # closed AT its own start: impossible
    rejected = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [_row(4), bad], POI
    )
    assert rejected.status == "bad_interval_arithmetic"


def test_internal_gap_is_recorded_not_rejected_here() -> None:
    """Whether a gap matters depends on the comparison episode, which only
    the comparison layer knows; certification records it and rides on."""
    result = bin_mod.certify_epoch(
        [_scpc(bin_mod.EV_ENABLE)], [_row(2), _row(5)], POI
    )
    assert result.status == "accepted"
    assert result.missing_seqs == (3, 4)
    assert result.first_seq == 2 and result.last_seq == 5


def test_rows_from_other_epochs_never_leak_in() -> None:
    rows = [_row(0, ep=1), _row(0, ep=2, ep_us=EPOCH_US + 1_000_000)]
    result = bin_mod.certify_epoch([_scpc(bin_mod.EV_ENABLE)], rows, POI)
    assert result.status == "accepted"
    assert len(result.rows) == 1


def test_raw_epoch_global_reads_the_last_row() -> None:
    rows = (
        _row(0, d3=50.0),
        _row(1, d3=0.05, fl=bin_mod.FLAG_FINAL),
    )
    raw = bin_mod.raw_epoch_global(rows)
    assert raw["d3_m"] == 0.05
    assert raw["final_flagged"] is True
    assert raw["cpa_time_s"] == rows[-1]["GCpUS"] / 1e6
