"""LASTLOG successor binding and BIN acquisition, with slot IO faked out."""

from __future__ import annotations

from scripts import eval_sim_cpa_artifact as artifact


def _pre(monkeypatch, lastlog, bins=0, logs_dir="/home/u/ardupilot/121/logs"):
    monkeypatch.setattr(
        artifact, "resolve_logs_dir", lambda sysid: (logs_dir, "")
    )
    monkeypatch.setattr(
        artifact, "read_lastlog",
        lambda d: (lastlog, "") if lastlog is not None else (None, "absent"),
    )
    monkeypatch.setattr(artifact, "count_bins", lambda d: bins)


def test_binding_normal_successor(monkeypatch) -> None:
    _pre(monkeypatch, lastlog=371)
    binding = artifact.binding_pre_flight(121)
    assert binding["status"] == "armed"
    assert binding["lastlog_pre"] == 371
    assert binding["expected_number"] == 372


def test_binding_virgin_slot_only_when_truly_empty(monkeypatch) -> None:
    _pre(monkeypatch, lastlog=None, bins=0)
    binding = artifact.binding_pre_flight(121)
    assert binding["status"] == "armed"
    assert binding["expected_number"] == 1

    # LASTLOG absent but BINs present: no successor can be established
    # (R13) -- guessing here is exactly the mtime-discovery this design
    # replaced.
    _pre(monkeypatch, lastlog=None, bins=17)
    binding = artifact.binding_pre_flight(121)
    assert binding["status"] == "ambiguous"


def test_binding_refuses_the_rollover_window(monkeypatch) -> None:
    _pre(monkeypatch, lastlog=artifact.ROLLOVER_GUARD_MIN)
    binding = artifact.binding_pre_flight(121)
    assert binding["status"] == "rollover"


def _armed_binding(logs_dir="/home/u/ardupilot/121/logs") -> dict:
    return {
        "mode": "lastlog_successor", "status": "armed", "sysid": 121,
        "logs_dir": logs_dir, "lastlog_pre": 371, "expected_number": 372,
        "error": None, "recorded_wall_time_s": 0.0,
    }


def test_acquire_maps_unarmed_bindings_to_their_status(tmp_path) -> None:
    rollover = dict(_armed_binding(), status="rollover", error="wrap")
    result = artifact.acquire_bin(
        rollover, tmp_path, case_start_wall_s=0.0
    )
    assert result["status"] == "rollover_window"

    ambiguous = dict(_armed_binding(), status="ambiguous", error="?")
    result = artifact.acquire_bin(
        ambiguous, tmp_path, case_start_wall_s=0.0
    )
    assert result["status"] == "ambiguous"


def test_acquire_lastlog_unchanged_means_never_armed(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (371, ""))
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path, case_start_wall_s=0.0
    )
    assert result["status"] == "no_bin"
    assert "never advanced" in result["error"]


def test_acquire_rejects_any_number_but_the_successor(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (373, ""))
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path, case_start_wall_s=0.0
    )
    assert result["status"] == "ambiguous"
    assert "more than one logging session" in result["error"]


def test_acquire_rejects_a_stale_wrapped_file(tmp_path, monkeypatch) -> None:
    """Wrapped numbering keeps stale higher-numbered BINs on disk; the
    mtime-within-case check is load-bearing (slot 121 evidence, R13)."""
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (372, ""))
    monkeypatch.setattr(
        artifact, "stat_size_mtime", lambda p: ((1000, 100), "")
    )
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path,
        case_start_wall_s=100_000.0,  # case started long after mtime 100
    )
    assert result["status"] == "ambiguous"
    assert "stale" in result["error"]


def test_acquire_happy_path_copies_verifies_and_hashes(
    tmp_path, monkeypatch
) -> None:
    payload = b"DFBIN" * 100
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (372, ""))
    monkeypatch.setattr(
        artifact, "stat_size_mtime",
        lambda p: ((len(payload), 2_000_000_000), ""),
    )

    def _fake_copy(source, destination):
        destination.write_bytes(payload)
        return True, ""

    monkeypatch.setattr(artifact, "copy_slot_file", _fake_copy)
    monkeypatch.setattr(
        artifact.time, "time", lambda: 2_000_000_100.0
    )
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path, case_start_wall_s=1_999_999_990.0
    )
    assert result["status"] == "acquired"
    assert result["source_path"].endswith("00000372.BIN")
    assert result["size_bytes"] == len(payload)
    import hashlib

    assert result["sha256"] == hashlib.sha256(payload).hexdigest()
    # The copy staged and was renamed only after verification: no partial
    # file remains and the authoritative name holds the verified bytes.
    assert not (tmp_path / "sim_cpa.BIN.partial").exists()
    assert (tmp_path / "sim_cpa.BIN").read_bytes() == payload


def test_acquire_detects_a_short_copy(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (372, ""))
    monkeypatch.setattr(
        artifact, "stat_size_mtime", lambda p: ((500, 2_000_000_000), "")
    )

    def _fake_copy(source, destination):
        destination.write_bytes(b"short")
        return True, ""

    monkeypatch.setattr(artifact, "copy_slot_file", _fake_copy)
    monkeypatch.setattr(
        artifact.time, "time", lambda: 2_000_000_100.0
    )
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path, case_start_wall_s=1_999_999_990.0
    )
    assert result["status"] == "copy_failed"
    assert "not the bound artifact" in result["error"]
    # 90_review finding 5: a failed verification never leaves anything
    # under the authoritative name, and the staging file is cleaned up.
    assert not (tmp_path / "sim_cpa.BIN").exists()
    assert not (tmp_path / "sim_cpa.BIN.partial").exists()


def test_acquire_rejects_a_future_mtime(tmp_path, monkeypatch) -> None:
    """The staleness gate is two-sided: a file whose mtime is ahead of
    this host's clock cannot be certified as this case's evidence."""
    monkeypatch.setattr(artifact, "read_lastlog", lambda d: (372, ""))
    monkeypatch.setattr(
        artifact, "stat_size_mtime",
        lambda p: ((1000, 2_000_009_000), ""),
    )
    monkeypatch.setattr(artifact.time, "time", lambda: 2_000_000_000.0)
    result = artifact.acquire_bin(
        _armed_binding(), tmp_path, case_start_wall_s=1_999_999_990.0
    )
    assert result["status"] == "ambiguous"
    assert "future" in result["error"]
