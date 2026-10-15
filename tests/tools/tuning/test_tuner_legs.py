"""Where inflight-tune flies the tuner (tools/tuning/tuner_legs.py): straight CRUISE legs, one axis each, with the
turn back between them while the tuner is not testing; or the earlier LOITER. On the fake aircraft."""
import argparse
import json
import math

import pytest

from tests.tools.tuning import r3_fake_plane
from tests.tools.tuning.r3_fake_plane import FakePlane
from tools.tuning import sitl_driver, tuner_legs
from tools.tuning.flight_floor import MIN_HEIGHT_M
from tools.tuning.flight_steps import (PILOT_THROTTLE_PWM, FlightCheckFailed, cruise_airspeed_stick_pwm, nudged_airspeed,
                                       throttle_control_in)
from tools.tuning.tuner_legs import (COURSE_TOL_DEG, LEG_MODE, PATTERN_LOITER, PATTERN_STRAIGHT, TURN_MODE,
                                     axis_blocks, bearing_deg, course_error_deg, fly_tuner)

SESSION_S = 60.0          # a fake tuner session: a few kilometres of straight flight at the cruise


def test_each_axis_gets_its_own_leg_roll_first():
    assert axis_blocks(3) == [1, 2] and axis_blocks(1) == [1] and axis_blocks(2) == [2] and axis_blocks(0) == []


def test_bearings_and_course_errors():
    assert bearing_deg(0.0, 0.0, 0.01, 0.0) == pytest.approx(0.0)
    assert bearing_deg(0.0, 0.0, 0.0, 0.01) == pytest.approx(90.0)
    assert bearing_deg(0.0, 0.0, -0.01, 0.0) == pytest.approx(180.0)
    assert bearing_deg(60.0, 0.0, 60.0, -0.01) == pytest.approx(270.0)
    assert course_error_deg(350.0, 10.0) == pytest.approx(-20.0) and course_error_deg(10.0, 350.0) == pytest.approx(20.0)
    assert course_error_deg(180.0, 0.0) == pytest.approx(-180.0)


def _plane(**kwargs) -> FakePlane:
    plane = FakePlane(height=120, mode="FBWA", tuner_s=SESSION_S, course_deg=30.0, **kwargs)
    plane.armed = True
    plane.tx.set(3, 1650)                      # the pilot's throttle after the takeoff (takeoff_to_fbwa)
    return plane


def test_straight_legs_fly_one_session_per_axis_in_cruise_and_turn_only_between_them():
    plane = _plane()
    start = (plane.north, plane.east)
    sessions = fly_tuner(plane, 3, PATTERN_STRAIGHT, timeout_s=600)
    assert [(s["axes"], s["mode"], s["end"]) for s in sessions] == [(1, LEG_MODE, "FTUN: done"),
                                                                    (2, LEG_MODE, "FTUN: done")]
    flown = plane.sessions
    assert [(s["mode"], s["axes"], s.get("end_mode")) for s in flown] == [("CRUISE", 1, "CRUISE"), ("CRUISE", 2, "CRUISE")]
    # no turn while the tuner flies: the course a session started on is the one it ended on
    assert all(s["end_course"] == s["start_course"] for s in flown)
    # CRUISE at the airspeed LOITER flies at the pilot's stick: the throttle stick is set before the mode and the switch
    leg_speed = nudged_airspeed(plane, PILOT_THROTTLE_PWM)
    assert leg_speed == pytest.approx(23.0 + 8.0 * 0.26) and {s["airspeed_m_s"] for s in sessions} == {leg_speed}
    assert {s["throttle_pwm"] for s in flown} == {cruise_airspeed_stick_pwm(plane, leg_speed)}
    assert plane.airspeed() == pytest.approx(leg_speed, abs=0.13)        # whole percent of stick
    # the turn back happened between the sessions, in LOITER, and the second leg points at the first one's start
    loiter_at = [i for i, (mode, _) in enumerate(plane.modes) if mode == TURN_MODE]
    assert len(loiter_at) == 1 and sessions[1]["turn_s"] > 0 and sessions[0]["turn_s"] == 0
    second = flown[1]
    back = math.degrees(math.atan2(start[1] - second["start_pos"][1], start[0] - second["start_pos"][0])) % 360
    assert abs(course_error_deg(second["start_course"], back)) <= COURSE_TOL_DEG + r3_fake_plane.LOITER_TURN_DPS
    # out and back: the aircraft ends nearer its start than the first leg took it away
    leg = math.dist(start, flown[0]["start_pos"]) + plane.airspeed() * SESSION_S
    assert plane.distance_from(*start) < 0.6 * leg


def test_the_nudge_and_the_stick_for_an_airspeed_follow_the_firmwares_stick_map():
    plane = FakePlane()
    assert throttle_control_in(plane, 1650) == 63 and throttle_control_in(plane, 1030) == 0       # 30 us dead zone
    assert throttle_control_in(plane, 2100) == 100
    assert nudged_airspeed(plane, 1500) == 23.0                  # mid-stick: no nudge
    plane.params["THROTTLE_NUDGE"] = 0
    assert nudged_airspeed(plane, 1650) == 23.0
    plane.params["RC3_REVERSED"] = 1
    assert throttle_control_in(plane, 3000 - 1650) == 63
    plane.params.update({"RC3_REVERSED": 0, "FLIGHT_OPTIONS": 1 << 3})
    assert cruise_airspeed_stick_pwm(plane) == 1000
    with pytest.raises(FlightCheckFailed, match="CRUISE cannot fly 25 m/s"):
        cruise_airspeed_stick_pwm(plane, 25.0)


def test_a_session_that_does_not_finish_ends_the_legs_and_the_turn_is_not_flown():
    plane = _plane(tuner_end="FTUN: stopped: model does not fit")
    sessions = fly_tuner(plane, 3, PATTERN_STRAIGHT, timeout_s=600)
    assert [(s["axes"], s["end"]) for s in sessions] == [(1, "FTUN: stopped: model does not fit")]
    assert TURN_MODE not in [mode for mode, _ in plane.modes]


def test_the_loiter_pattern_flies_one_session_on_all_axes_as_before():
    plane = _plane()
    sessions = fly_tuner(plane, 3, PATTERN_LOITER, timeout_s=600)
    assert sessions == [{"axes": 3, "mode": TURN_MODE, "end": "FTUN: done"}]
    assert [(s["mode"], s["axes"]) for s in plane.sessions] == [("LOITER", 3)]
    with pytest.raises(ValueError, match="unknown pattern"):
        fly_tuner(plane, 3, "figure-eight", timeout_s=600)


def test_a_turn_that_never_points_back_fails_the_run(monkeypatch):
    monkeypatch.setattr(r3_fake_plane, "LOITER_TURN_DPS", 0.0)       # LOITER that flies straight on
    plane = _plane()
    with pytest.raises(FlightCheckFailed, match="did not turn towards the start"):
        fly_tuner(plane, 3, PATTERN_STRAIGHT, timeout_s=600)
    assert len(plane.sessions) == 1


def test_inflight_tune_records_the_pattern_and_each_session(monkeypatch, tmp_path):
    plane = FakePlane(tuner_s=SESSION_S, course_deg=200.0)
    monkeypatch.setattr(sitl_driver, "open_vehicle", lambda *args: plane)
    args = argparse.Namespace(workdir=str(tmp_path / "tune"), axes=3, timeout=600, param=None, firmware=True,
                              min_height=MIN_HEIGHT_M, pattern=PATTERN_STRAIGHT)
    assert sitl_driver.inflight_tune(args) == 0
    record = json.loads((tmp_path / "tune.json").read_text())
    assert record["pattern"] == "straight" and record["end"] == "FTUN: done"
    assert [(s["axes"], s["mode"], s["end"]) for s in record["sessions"]] == [(1, "CRUISE", "FTUN: done"),
                                                                             (2, "CRUISE", "FTUN: done")]
    assert record["after"]["PTCH_RATE_P"] != record["before"]["PTCH_RATE_P"]
    assert record["height"]["breach"] is None


def test_the_pattern_option_defaults_to_straight_legs(monkeypatch):
    seen = {}
    monkeypatch.setattr(sitl_driver, "inflight_tune", lambda args: seen.setdefault("args", args) and 0)
    monkeypatch.setattr("sys.argv", ["sitl_driver", "inflight-tune", "--workdir", "w", "--firmware"])
    with pytest.raises(SystemExit):
        sitl_driver.main()
    assert seen["args"].pattern == PATTERN_STRAIGHT
    assert tuner_legs.PATTERNS == (PATTERN_STRAIGHT, PATTERN_LOITER)
