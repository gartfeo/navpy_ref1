"""Tests for the flight-log SYSID scanner (tools/tuning/log_scan.py)."""
import argparse
import math
from pathlib import Path

import pytest

from tools.tuning.log_scan import LogInfo, OnTimer, main, parse_range, scan_bin, select

FIXTURE = Path(__file__).resolve().parents[2] / "fixtures" / "sim_cpa" / "valA_370_prefix.BIN"


def test_parse_range_forms():
    assert parse_range("157") == (157, 157)
    assert parse_range("151-170") == (151, 170)
    assert parse_range("161-") == (161, 255)
    assert parse_range("-150") == (0, 150)
    with pytest.raises(argparse.ArgumentTypeError):
        parse_range("170-151")


def test_on_timer_counts_only_on_spans_and_skips_long_gaps():
    t = OnTimer()
    for ts, on in [(0, True), (1, True), (2, False), (3, True), (4, True), (20, True), (21, False)]:
        t.sample(ts, on)
    # 0-1, 1-2 on; 2-3 off; 3-4 on; 4-20 is a gap; 20-21 on
    assert t.total_s == pytest.approx(4.0)


def test_select_filters_sysid_range_and_minutes():
    def info(sysid, minutes):
        return LogInfo(Path(f"{sysid}.bin"), "bin", sysid, minutes, "flying", minutes)
    infos = [info(150, 20), info(151, 20), info(163, 5), info(170, 10), info(None, 30)]
    assert [i.sysid for i in select(infos, (151, 170), 10)] == [151, 170]


def test_scan_bin_reads_sysid_and_flying_time_from_a_real_log():
    info = scan_bin(FIXTURE, (0, 255))
    assert info.sysid == 121
    assert 0 < info.minutes <= info.span_minutes


def test_scan_bin_stops_early_outside_the_range():
    info = scan_bin(FIXTURE, (150, 170))
    assert info.sysid == 121 and info.time_basis == "skip" and math.isnan(info.minutes)


def test_main_lists_and_writes_csv(tmp_path, capsys):
    out = tmp_path / "sel.csv"
    assert main([str(FIXTURE.parent), "--sysid", "121", "--csv", str(out)]) == 0
    assert "1 of 1 logs" in capsys.readouterr().out
    assert out.read_text(encoding="utf-8").splitlines()[1].startswith("121,")
