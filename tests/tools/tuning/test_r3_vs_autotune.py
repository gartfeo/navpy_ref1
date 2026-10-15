"""Tests for the vs-autotune procedure (tools/tuning/vs_autotune.py, sitl_driver vs-autotune) and the floor
on sitl_driver's other flights, on the fake aircraft (r3_fake_plane.py) instead of SITL.

Round 2 on the HITL chain: AUTOTUNE flew at 65 % throttle, sank in roll doublets and was stopped at
36 m before pitch; LOITER held 36 m, so the tuner (FTUN_ALT_MIN 40 + 10 m) never started; the cards compared
identical gains, card 13 struck the ground, and the run exited 0.
"""
import argparse
import json
import sys

import pytest

from tests.tools.tuning.r3_fake_plane import FakePlane
from tools.tuning import sitl_driver
from tools.tuning.flight_floor import MIN_HEIGHT_M
from tools.tuning.flight_steps import PILOT_THROTTLE_PWM, PITCH_TRIM, FlightCheckFailed
from tools.tuning.hitl_params import read_param_file
from tools.tuning.vs_autotune import AUTOTUNE_RECOVERY_M, CARD_MARKER


def _args(tmp_path, **over) -> argparse.Namespace:
    values = dict(workdir=str(tmp_path / "vsat"), level=0, axes=3, autotune_s=240, ftun_alt_min=40, timeout=1800,
                  firmware=True, min_height=MIN_HEIGHT_M)
    return argparse.Namespace(**{**values, **over})


def _fly(monkeypatch, tmp_path, plane: FakePlane, **over) -> tuple[int, dict]:
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: plane)
    code = sitl_driver.vs_autotune(_args(tmp_path, **over))
    return code, json.loads((tmp_path / "vsat.json").read_text())


def test_the_round2_case_now_gives_a_valid_comparison(monkeypatch, tmp_path):
    plane = FakePlane()
    code, record = _fly(monkeypatch, tmp_path, plane)
    assert code == 0 and record["height"]["breach"] is None
    # AUTOTUNE and the cards flew the cruise throttle LOITER held (30 %), not 65 %
    throttle = {k: record["pilot_throttle"][k] for k in ("cruise_pct", "stick_pwm", "fbwa_pct")}
    assert throttle == {"cruise_pct": 30, "stick_pwm": 1300, "fbwa_pct": 30}
    assert {pwm for ch, pwm in plane.sticks["AUTOTUNE"] if ch == 3} == {1300}
    assert PILOT_THROTTLE_PWM not in {pwm for ch, pwm in plane.sticks["FBWA"] if ch == 3}
    # AUTOTUNE was left for a commanded climb when it sank near the floor, and still finished both axes
    assert record["autotune_finished"] == {"Roll": True, "Pitch": True}
    assert record["autotune_flight"]["Roll"]["climbs"] >= 1
    assert record["height"]["lowest_m"] > MIN_HEIGHT_M + AUTOTUNE_RECOVERY_M - 10
    assert plane.commands and all(c == ("LOITER", 120.0, 3.0) for c in plane.commands)
    # the tuner started above FTUN_ALT_MIN + 10 m and changed the gains
    assert plane.tuner_started_at >= 50 and record["ftune_end"] == "FTUN: done" and record["gain_sets_differ"]
    # every card started back near TKOFF_ALT, and the marker went back to its own value after each
    assert [m for m, _ in plane.card_starts] == [11, 12, 13, 14]
    assert all(h >= 120 - 5 for _, h in plane.card_starts)
    assert plane.params[CARD_MARKER] == 4


def test_autotune_and_the_tuner_tune_and_the_cards_fly_only_the_chosen_axes(monkeypatch, tmp_path):
    plane = FakePlane()
    code, record = _fly(monkeypatch, tmp_path, plane, axes=2)
    assert code == 0 and record["autotune_finished"] == {"Pitch": True}
    assert plane.params["FTUN_AXES"] == 2
    assert read_param_file(tmp_path / "vsat_extra.parm")["AUTOTUNE_AXES"] == 2
    assert not any(ch == 1 for mode in ("AUTOTUNE", "FBWA") for ch, _ in plane.sticks[mode])


class _TrimWatch(FakePlane):
    """FakePlane noting PTCH_TRIM_DEG whenever AUTOTUNE is entered and a card starts."""

    def __init__(self, **kw: object) -> None:
        super().__init__(**kw)
        self.trim_at: list[tuple[str, float]] = []

    def set_mode(self, mode: str) -> None:
        if mode == "AUTOTUNE":
            self.trim_at.append(("AUTOTUNE", self.params[PITCH_TRIM]))
        super().set_mode(mode)

    def set_param(self, name: str, value: float) -> None:
        super().set_param(name, value)
        if name == CARD_MARKER and 11 <= value <= 14:
            self.trim_at.append((f"card {value:g}", self.params[PITCH_TRIM]))


def test_autotune_and_the_cards_fly_port_checks_cruise_trim_and_it_is_set_back(monkeypatch, tmp_path):
    # FBWA's stick centre flies 0.5 deg, the cruise 3 deg: port-check's trim is 2.5 deg (test_r3_flight_steps)
    plane = _TrimWatch()
    code, record = _fly(monkeypatch, tmp_path, plane)
    assert code == 0
    trim = record["pitch_trim"]
    assert trim["before_deg"] == 0.0 and trim["run_deg"] == pytest.approx(2.5) and trim["restored"] is True
    entries = [name for name, _ in plane.trim_at]
    assert entries.count("AUTOTUNE") >= 2 and [e for e in entries if e.startswith("card")] == [
        "card 11", "card 12", "card 13", "card 14"]
    assert all(value == pytest.approx(trim["run_deg"]) for _, value in plane.trim_at), plane.trim_at
    assert plane.params[PITCH_TRIM] == 0.0


def test_a_card_that_sinks_to_the_floor_fails_the_run_and_stops_the_cards(monkeypatch, tmp_path):
    plane = FakePlane(fbwa_sink=10)
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: plane)
    monkeypatch.setattr(sys, "argv", ["sitl_driver", "vs-autotune", "--workdir", str(tmp_path / "vsat"),
                                      "--firmware"])
    with pytest.raises(SystemExit) as stop:
        sitl_driver.main()
    assert stop.value.code == 1
    record = json.loads((tmp_path / "vsat.json").read_text())
    assert "below the run's floor of 30 m" in record["error"] and record["height"]["breach"]
    assert len(plane.card_starts) == 1 and plane.events == ["start", "stop"]


def test_a_tuner_that_does_not_finish_fails_the_run_before_the_cards(monkeypatch, tmp_path):
    plane = FakePlane(tuner_end="FTUN: stopped: model does not fit")
    with pytest.raises(FlightCheckFailed, match="the tuner did not finish .*model does not fit"):
        _fly(monkeypatch, tmp_path, plane)
    record = json.loads((tmp_path / "vsat.json").read_text())
    assert record["ftune_end"] == "FTUN: stopped: model does not fit" and not record["gain_sets_differ"]
    assert plane.card_starts == []


def test_port_check_fails_when_its_doublets_take_it_below_the_floor(monkeypatch, tmp_path):
    plane = FakePlane(fbwa_sink=8)
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: plane)
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        sitl_driver.port_check(argparse.Namespace(workdir=str(tmp_path / "sign"), min_height=MIN_HEIGHT_M))
    assert json.loads((tmp_path / "sign.json").read_text())["height"]["lowest_m"] < MIN_HEIGHT_M


def test_inflight_tune_fails_when_loiter_loses_its_height(monkeypatch, tmp_path):
    plane = FakePlane(loiter_follows_target=False, loiter_sink=5)
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: plane)
    args = argparse.Namespace(workdir=str(tmp_path / "tune"), axes=2, timeout=60, param=None, firmware=True,
                              min_height=MIN_HEIGHT_M, pattern="loiter")
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        sitl_driver.inflight_tune(args)
    assert json.loads((tmp_path / "tune.json").read_text())["height"]["breach"]
