"""SIM_CPA configuration policy: exact chunks, modes, override rejection."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts import eval_sim_cpa_config as config


def test_chunk_truncates_toward_zero_both_signs() -> None:
    assert config.chunk(430242544) == (43024, 2544)
    assert config.chunk(-430242544) == (-43024, -2544)
    assert config.chunk(9999) == (0, 9999)
    assert config.chunk(-9999) == (0, -9999)
    assert config.chunk(0) == (0, 0)


def test_chunks_reconstruct_and_survive_float32_transport() -> None:
    for value in (430242544, -430242544, 900000000, -1800000000, 12345):
        hi, lo = config.chunk(value)
        assert hi * 10000 + lo == value
        # The transport premise: both chunks are exactly representable in
        # REAL32, so a PARAM_SET round trip cannot alter them.
        assert int(float(hi)) == hi and int(float(lo)) == lo
        assert abs(lo) < 10000


def test_derive_params_matches_the_flight_validated_values() -> None:
    # The 2026-09-03 validation POI, chunk-for-chunk.
    assert config.derive_params(43.0242544, 34.0, 60.1) == [
        ("SIM_CPA_LAT_HI", 43024),
        ("SIM_CPA_LAT_LO", 2544),
        ("SIM_CPA_LNG_HI", 34000),
        ("SIM_CPA_LNG_LO", 0),
        ("SIM_CPA_ALT_CM", 6010),
        ("SIM_CPA_ENABLE", 1),
    ]


def test_derive_params_rejects_out_of_range_targets() -> None:
    with pytest.raises(ValueError):
        config.derive_params(91.0, 0.0, 100.0)
    with pytest.raises(ValueError):
        config.derive_params(0.0, 181.0, 100.0)
    with pytest.raises(ValueError):
        config.derive_params(0.0, 0.0, 10001.0)  # >1e6 cm


def test_resolve_mode_defaults_and_flag() -> None:
    assert config.resolve_mode(None, None) == "auto"
    assert config.resolve_mode(None, []) == "auto"
    assert config.resolve_mode("off", None) == "off"
    assert config.resolve_mode("auto", None) == "auto"


def test_resolve_mode_legacy_enable_zero_normalizes_to_off() -> None:
    assert config.resolve_mode(None, ["SIM_CPA_ENABLE=0"]) == "off"
    # But explicitly asking for auto AND for the module dark is a
    # contradiction, not a precedence question.
    with pytest.raises(ValueError):
        config.resolve_mode("auto", ["SIM_CPA_ENABLE=0"])


def test_resolve_mode_rejects_every_poi_override() -> None:
    for raw in (
        "SIM_CPA_LAT_HI=43024",
        "SIM_CPA_LAT_LO=1",
        "SIM_CPA_LNG_HI=0",
        "SIM_CPA_ALT_CM=6010",
        "SIM_CPA_ENABLE=1",
    ):
        with pytest.raises(ValueError):
            config.resolve_mode(None, [raw])


class _FakeMaster:
    """Records exact-push calls; a scripted probe and echo behavior."""

    def __init__(self, probe: float | None, fail_at: str | None = None):
        self.probe = probe
        self.fail_at = fail_at
        self.pushes: list[tuple[str, int]] = []


def _fake_read(master: _FakeMaster, name: str, **kwargs):
    assert name == "SIM_CPA_ENABLE"
    return master.probe


def _fake_set(master: _FakeMaster, name: str, value: int, **kwargs) -> bool:
    if master.fail_at == name:
        return False
    master.pushes.append((name, value))
    return True


def _configure(master: _FakeMaster, mode: str, case_dir: Path | None = None):
    return config.configure_sim_cpa(
        master,
        mode=mode,
        lat_deg=43.0242544,
        lon_deg=34.0,
        abs_alt_m=60.1,
        case_dir=case_dir,
        read_param=_fake_read,
        set_exact=_fake_set,
    )


def test_configure_unsupported_binary_records_and_pushes_nothing(
    tmp_path: Path,
) -> None:
    master = _FakeMaster(probe=None)
    record = _configure(master, "auto", tmp_path)
    assert record["status"] == "unsupported_binary"
    assert master.pushes == []
    # The record also lands as its own file so a later hard crash cannot
    # erase how far configuration got.
    on_disk = json.loads(
        (tmp_path / config.CONFIG_RECORD_NAME).read_text(encoding="utf-8")
    )
    assert on_disk["status"] == "unsupported_binary"


def test_configure_off_pushes_only_the_disable(tmp_path: Path) -> None:
    master = _FakeMaster(probe=1.0)
    record = _configure(master, "off", tmp_path)
    assert record["status"] == "disabled_by_operator"
    assert master.pushes == [("SIM_CPA_ENABLE", 0)]


def test_configure_auto_disarms_a_stale_enable_before_chunks() -> None:
    """The launcher syncs EEPROM from a mutable template, so the module can
    boot already armed on an old POI; chunks must never change under an
    active epoch."""
    master = _FakeMaster(probe=1.0)
    record = _configure(master, "auto")
    assert record["status"] == "configured"
    assert master.pushes[0] == ("SIM_CPA_ENABLE", 0)
    assert master.pushes[1:] == config.derive_params(43.0242544, 34.0, 60.1)
    assert master.pushes[-1] == ("SIM_CPA_ENABLE", 1)


def test_configure_auto_clean_boot_pushes_chunks_then_enable() -> None:
    master = _FakeMaster(probe=0.0)
    record = _configure(master, "auto")
    assert record["status"] == "configured"
    assert master.pushes == config.derive_params(43.0242544, 34.0, 60.1)
    assert record["acknowledged_params"] == [
        list(pair) for pair in master.pushes
    ]


def test_configure_stops_at_the_first_failed_push() -> None:
    master = _FakeMaster(probe=0.0, fail_at="SIM_CPA_LNG_HI")
    record = _configure(master, "auto")
    assert record["status"] == "push_failed"
    assert record["failed_param"] == "SIM_CPA_LNG_HI"
    # ENABLE was never attempted: a partially-configured module must not
    # be armed.
    assert all(name != "SIM_CPA_ENABLE" for name, _ in master.pushes)
    assert record["acknowledged_params"] == [
        ["SIM_CPA_LAT_HI", 43024], ["SIM_CPA_LAT_LO", 2544],
    ]


def test_configure_never_raises(tmp_path: Path) -> None:
    def _exploding_read(master, name, **kwargs):
        raise OSError("link dropped")

    record = config.configure_sim_cpa(
        object(),
        mode="auto",
        lat_deg=43.0,
        lon_deg=34.0,
        abs_alt_m=60.0,
        case_dir=tmp_path,
        read_param=_exploding_read,
        set_exact=_fake_set,
    )
    assert record["status"] == "push_failed"
    assert "link dropped" in record["error"]
