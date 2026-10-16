"""Tests for the hand-back gate (tools/tuning/handback_check.py) on synthetic logs.

A pitch hold: FTUN at 200 Hz from 9 s (LEVEL until 10 s, then SETTLE and a check's RUN) to the hand-back at
12 s; TECS at 10 Hz except during the hold; CTUN at 25 Hz with the held throttle until TECS's first update.
"""
import json
import sys

import numpy as np
import pytest

from tools.tuning import handback_check
from tools.tuning.handback_check import check_log, hold_ends, slew_step_pct
from tools.tuning.logdata import FlightLog

PARAMS = {"THR_SLEWRATE": 100.0, "THR_MAX": 100.0, "THR_MIN": 0.0}      # 10 % per 0.1 s update
HAND_BACK_S = 12.0
FIRST_TECS_S = 12.05


RELEASE_PHASE = 6       # FTUN.Ph of the hand-back of the bank after a manoeuvre, on TECS's pitch and throttle


def _log(held: float, after: list[float], params: dict | None = None, axis: int = 1, hold_phase: int = 4,
         tecs_after: bool = True, holds: tuple[float, ...] = (HAND_BACK_S,), release_s: float = 0.0) -> FlightLog:
    """`after`: TECS's throttle (percent) at its updates from FIRST_TECS_S on, then held at the last value.
    release_s: FTUN records of the hand-back (Ph 6) for this long after each hold."""
    ftun_t, ftun_ax, ftun_ph = [], [], []
    for end in holds:
        t = np.arange(end - 3.0, end + release_s + 1e-9, 0.005)
        ftun_t.append(t)
        ftun_ax.append(np.full(len(t), axis))
        held_phases = np.where(t < end - 1.4, 2, hold_phase) if hold_phase != 1 else np.ones(len(t))
        ftun_ph.append(np.where(t < end - 2.0, 1, np.where(t > end + 1e-9, RELEASE_PHASE, held_phases)))
    tecs_t = np.round(np.arange(0.05, 20.0, 0.1), 3)           # 10 Hz, the first after the hand-back at 12.05 s
    in_hold = np.any([(tecs_t > end - 2.0) & (tecs_t < end) for end in holds], axis=0)
    if not tecs_after:
        in_hold |= tecs_t > holds[-1] - 2.0
    tecs_t = tecs_t[~in_hold]
    k = np.round((tecs_t - FIRST_TECS_S) / 0.1).astype(int)
    tecs_th = np.where(tecs_t < FIRST_TECS_S - 1e-6, held, np.array(after)[np.clip(k, 0, len(after) - 1)])
    ctun_t = np.round(np.arange(0.0, 20.0, 0.04), 3)
    latest = np.searchsorted(tecs_t, ctun_t, side="right") - 1
    thd = np.where(ctun_t < FIRST_TECS_S, held, tecs_th[latest])
    series = {"FTUN": {"TimeUS": np.concatenate(ftun_t) * 1e6, "Ax": np.concatenate(ftun_ax).astype(float),
                       "Ph": np.concatenate(ftun_ph).astype(float)},
              "TECS": {"TimeUS": tecs_t * 1e6, "th": tecs_th / 100.0},
              "CTUN": {"TimeUS": ctun_t * 1e6, "ThO": thd.copy(), "ThD": thd}}
    return FlightLog(path="synthetic", series=series, params=dict(PARAMS if params is None else params),
                     param_history=[])


def test_round2s_dip_fails_although_the_samples_before_tecss_first_update_show_no_step():
    # as round 2's first pitch hand-back: the throttle held, TECS's first output 0.0 %, then +10 % per update
    log = _log(26.4, [0.0, 10.0, 20.0])
    ctun_t, thd = log.get("CTUN", "TimeUS") / 1e6, log.get("CTUN", "ThD")
    assert thd[np.searchsorted(ctun_t, HAND_BACK_S, side="right")] == 26.4          # round 2's 0.0 % step
    result = check_log(log)
    assert not result["ok"] and len(result["handbacks"]) == 1
    hand_back = result["handbacks"][0]
    assert hand_back["held_pct"] == 26.4 and hand_back["tecs_pct"] == [0.0, 0.0]
    assert hand_back["deviation_pct"] == pytest.approx(26.4)
    assert hand_back["first_update_after_s"] == pytest.approx(FIRST_TECS_S - HAND_BACK_S, abs=0.006)
    assert "0.0 % against 26.4 % held" in result["problems"][0]


def test_a_hand_back_within_tecss_slew_passes_over_several_updates():
    result = check_log(_log(26.4, [31.0, 41.0, 50.0]), updates=3)
    assert result["ok"], result["problems"]
    assert result["step_pct"] == pytest.approx(10.0, rel=1e-3)
    assert result["handbacks"][0]["deviation_pct"] == pytest.approx(23.6)


def test_a_throttle_faster_than_tecss_slew_fails_in_the_update_where_it_happens():
    log = _log(26.4, [27.0, 49.0])                 # within one step at the first update, two steps at the second
    assert check_log(log, updates=1)["ok"]
    result = check_log(log, updates=2)
    assert not result["ok"] and "49.0 % against 26.4 % held" in result["problems"][0]


def test_the_step_comes_from_the_logs_throttle_parameters():
    assert slew_step_pct({"THR_SLEWRATE": 50, "THR_MAX": 80, "THR_MIN": 10}, 0.1) == pytest.approx(3.5)
    # AP_TECS clips a negative THR_MIN (reverse thrust) to 0 for the slew
    assert slew_step_pct({"THR_SLEWRATE": 50, "THR_MAX": 80, "THR_MIN": -20}, 0.1) == pytest.approx(4.0)
    assert slew_step_pct({"THR_SLEWRATE": 0, "THR_MAX": 80, "THR_MIN": 0}, 0.1) is None


def test_without_a_slew_limit_the_step_must_be_given():
    log = _log(26.4, [25.0], params={"THR_SLEWRATE": 0.0, "THR_MAX": 100.0, "THR_MIN": 0.0})
    result = check_log(log)
    assert not result["ok"] and "give --max-step-pct" in result["problems"][0]
    assert check_log(log, step_pct=5.0)["ok"]


def test_holds_are_pitch_past_levelling_only_and_each_one_is_checked():
    assert hold_ends(_log(26.4, [26.4], axis=0)) == []                 # roll manoeuvres hold nothing
    assert hold_ends(_log(26.4, [26.4], hold_phase=1)) == []           # levelling only: TECS still flies
    assert hold_ends(_log(26.4, [26.4], hold_phase=3)) == [pytest.approx(HAND_BACK_S * 1e6)]   # a test's RUN
    two = _log(26.4, [26.4], holds=(8.0, HAND_BACK_S))
    assert hold_ends(two) == [pytest.approx(8.0e6), pytest.approx(HAND_BACK_S * 1e6)]
    assert len(check_log(two)["handbacks"]) == 2


def test_the_hand_back_of_the_bank_after_a_hold_is_not_part_of_it():
    # the tuner flies on for a second after the manoeuvre (Ph 6) while TECS is already back: the hand-back of
    # pitch and throttle is still the end of the hold, and the gate checks TECS's first update after it
    log = _log(26.4, [0.0, 10.0], release_s=1.0)
    assert hold_ends(log) == [pytest.approx(HAND_BACK_S * 1e6)]
    result = check_log(log)
    assert not result["ok"] and "0.0 % against 26.4 % held" in result["problems"][0]
    assert result["handbacks"][0]["first_update_after_s"] == pytest.approx(FIRST_TECS_S - HAND_BACK_S, abs=0.006)


def test_a_log_with_nothing_to_check_fails_the_gate():
    no_hold = check_log(_log(26.4, [26.4], axis=0))
    assert not no_hold["ok"] and "no pitch hold" in no_hold["problems"][0]
    no_tecs = check_log(_log(26.4, [26.4], tecs_after=False))
    assert not no_tecs["ok"] and "no TECS update within" in no_tecs["problems"][0]
    with pytest.raises(ValueError):
        check_log(_log(26.4, [26.4]), updates=0)


def test_the_command_line_exits_1_on_a_failed_hand_back_and_writes_the_result(monkeypatch, tmp_path):
    logs = {"dip.BIN": _log(26.4, [0.0]), "good.BIN": _log(26.4, [25.0])}
    monkeypatch.setattr(handback_check, "read_log", lambda path, types: logs[path])
    for name, code in (("dip.BIN", 1), ("good.BIN", 0)):
        out = tmp_path / f"{name}.json"
        monkeypatch.setattr(sys, "argv", ["handback_check", "--log", name, "--out", str(out)])
        with pytest.raises(SystemExit) as stop:
            handback_check.main()
        assert stop.value.code == code and json.loads(out.read_text())[name]["ok"] == (code == 0)
