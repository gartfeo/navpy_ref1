"""Post-teardown orchestration: gating, provenance, and offline rescore."""

from __future__ import annotations

import json
from pathlib import Path

from scripts import eval_sim_cpa_score as score
from scripts.eval_sim_cpa_block import sim_cpa_block


def _configured() -> dict:
    return {
        "mode": "auto", "status": "configured", "expected_target": None,
        "requested_params": [], "acknowledged_params": [],
        "failed_param": None, "error": None,
    }


def _binding() -> dict:
    return {
        "mode": "lastlog_successor", "status": "armed", "sysid": 121,
        "logs_dir": "/x", "lastlog_pre": 3, "expected_number": 4,
        "error": None, "recorded_wall_time_s": 0.0,
    }


def test_non_configured_case_skips_binding_exit_and_parsing(
    tmp_path, monkeypatch
) -> None:
    """Disabled/unsupported/failed configuration armed nothing: chasing a
    module-silent BIN would only dress absence up as rejection."""
    def _never(*a, **k):
        raise AssertionError("must not be called")

    monkeypatch.setattr(score, "wait_for_sitl_exit", _never)
    monkeypatch.setattr(score, "acquire_bin", _never)
    disabled = dict(_configured(), status="disabled_by_operator")

    block = score.score_after_teardown(
        tmp_path, sysid=121, binding=_binding(), configuration=disabled,
        truth_block=None, case_start_wall_s=0.0, arduplane_sha_before="a",
    )

    assert block["configuration"]["status"] == "disabled_by_operator"
    assert block["artifact"]["status"] == "not_attempted"
    assert block["artifact"]["binding"] == _binding()
    assert block["evidence"]["status"] == "not_attempted"


def test_unconfirmed_sitl_exit_never_reads_the_bin(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setattr(
        score, "wait_for_sitl_exit", lambda sysid: (False, "still up")
    )

    def _never(*a, **k):
        raise AssertionError("a live BIN must never be read")

    monkeypatch.setattr(score, "acquire_bin", _never)

    block = score.score_after_teardown(
        tmp_path, sysid=121, binding=_binding(),
        configuration=_configured(), truth_block=None,
        case_start_wall_s=0.0, arduplane_sha_before="a",
    )

    assert block["artifact"]["status"] == "sitl_exit_timeout"
    assert "still up" in block["artifact"]["error"]


def test_binary_identity_change_rejects_the_evidence(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setattr(
        score, "wait_for_sitl_exit", lambda sysid: (True, "")
    )
    monkeypatch.setattr(
        score, "arduplane_sha256", lambda: ("b" * 64, "")
    )
    monkeypatch.setattr(
        score, "acquire_bin",
        lambda binding, case_dir, case_start_wall_s: {
            "status": "acquired", "source_path": "/x/4.BIN",
            "copied_path": str(tmp_path / "sim_cpa.BIN"),
            "size_bytes": 10, "sha256": "c" * 64, "binding": binding,
            "error": None,
        },
    )

    def _never(*a, **k):
        raise AssertionError("evidence from an unidentified binary")

    monkeypatch.setattr(score, "parse_sim_cpa_records", _never)

    block = score.score_after_teardown(
        tmp_path, sysid=121, binding=_binding(),
        configuration=_configured(), truth_block=None,
        case_start_wall_s=0.0, arduplane_sha_before="a" * 64,
    )

    assert block["evidence"]["status"] == "provenance_mismatch"
    assert block["provenance"]["arduplane_sha256_before"] == "a" * 64
    assert block["provenance"]["arduplane_sha256_after"] == "b" * 64


def test_uncertified_stream_yields_no_comparison(
    tmp_path, monkeypatch
) -> None:
    """Module evidence against an uncertified stream is diagnostic only:
    the evidence layer stays, the comparison refuses (R/g)."""
    (tmp_path / "case.json").write_text(json.dumps({
        "target": {
            "lat_deg": 43.0, "lon_deg": 34.0,
            "rel_alt_m": 60.0, "abs_alt_m": 500.0,
        },
    }), encoding="utf-8")
    monkeypatch.setattr(
        score, "parse_sim_cpa_records",
        lambda p: ([{
            "TimeUS": 1, "Ep": 1, "EpUS": 0, "Ev": 1,
            "LatE7": 430000000, "LngE7": 340000000, "AltCM": 50000,
        }], [{
            "TimeUS": 20000, "Ep": 1, "Seq": 0, "Fl": 0, "CpaUS": 10000,
            "D3": 1.0, "DH": 0.9, "DV": 0.1,
            "GCpUS": 10000, "GD3": 1.0, "GDH": 0.9, "GDV": 0.1,
        }]),
    )
    evidence, comparison = score._score_bin(
        tmp_path / "any.BIN", tmp_path,
        {"certification_error": "stream died", "dist_3d_m": 0.05},
    )
    assert evidence["status"] == "accepted"
    assert comparison["status"] == "stream_uncertified"


def test_offline_rescore_preserves_config_and_provenance_layers(
    tmp_path, monkeypatch
) -> None:
    (tmp_path / "case.json").write_text(json.dumps({
        "target": {
            "lat_deg": 43.0, "lon_deg": 34.0,
            "rel_alt_m": 60.0, "abs_alt_m": 500.0,
        },
    }), encoding="utf-8")
    bin_path = tmp_path / "some.BIN"
    bin_path.write_bytes(b"payload")
    monkeypatch.setattr(
        score, "parse_sim_cpa_records", lambda p: ([], [])
    )
    original = sim_cpa_block()
    original["configuration"]["status"] = "configured"
    original["provenance"]["arduplane_sha256_before"] = "d" * 64

    block = score.score_offline(
        tmp_path, bin_path,
        verdict={"sim_cpa": original, "truth": None},
    )

    # A rescore cannot re-observe the flight: those layers pass through.
    assert block["configuration"]["status"] == "configured"
    assert block["provenance"]["arduplane_sha256_before"] == "d" * 64
    assert block["artifact"]["binding"]["mode"] == "offline_explicit"
    assert block["evidence"]["status"] == "no_scpc"
    import hashlib

    assert block["artifact"]["sha256"] == hashlib.sha256(
        b"payload"
    ).hexdigest()
