"""Tests for the pilot's steps of the tuning flights (tools/tuning/flight_steps.py), without SITL."""
import pytest
from pymavlink import mavutil

from tests.tools.tuning.r3_fake_plane import FakePlane, Msg
from tools.tuning import flight_steps
from tools.tuning.flight_steps import (MIN_HEIGHT_M, PITCH_TRIM, CruisePitchTrim, FlightCheckFailed, HeightFloor,
                                       climb_to, cruise_state, pilot_throttle, throttle_stick_pwm)

M = mavutil.mavlink


def _firmware_stick_percent(pwm: int, rc_min: int, rc_max: int, reversed_: bool) -> int:
    """RC_Channel::pwm_to_range_dz(0) (RC_Channel.cpp:376-390), as FBWA and AUTOTUNE read the throttle stick."""
    r_in = max(rc_min, min(rc_max, pwm))
    if reversed_:
        r_in = rc_max - (r_in - rc_min)
    return (100 * (r_in - rc_min)) // (rc_max - rc_min) if r_in > rc_min else 0


@pytest.mark.parametrize("rc_min, rc_max, reversed_", [(1000, 2000, False), (1100, 1900, False), (1000, 2000, True)])
@pytest.mark.parametrize("percent", [0, 30, 30.5, 64, 100])
def test_the_stick_for_a_throttle_is_what_fbwa_flies_back(rc_min, rc_max, reversed_, percent):
    pwm = throttle_stick_pwm(percent, rc_min, rc_max, reversed_)
    assert abs(_firmware_stick_percent(pwm, rc_min, rc_max, reversed_) - percent) <= 1


def test_the_hitl_models_cruise_gets_its_own_stick_not_the_old_65_percent():
    # round 2: LOITER held the HITL model at its cruise throttle, and FBWA/AUTOTUNE flew the fixed 1650 us (65 %)
    assert throttle_stick_pwm(30, 1000, 2000, False) == 1300


def test_the_cruise_is_read_on_a_straight_cruise_leg_at_airspeed_cruise():
    # a straight leg, not a LOITER turn (which needs more throttle and lift), is the level cruise FBWA is matched to;
    # CRUISE's throttle stick is its airspeed: trim1 (2026-10-13) flew it at the low stick, AIRSPEED_MIN
    plane = FakePlane(height=118, mode="FBWA")
    plane.tx.set(3, 1650)
    assert cruise_state(plane) == {"throttle_pct": 30, "airspeed_m_s": 23.07, "climb_m_s": 0.0,
                                   "pitch_deg": pytest.approx(3.0)}
    assert plane.mode == "CRUISE" and plane.tx.get(3) == 1409


@pytest.mark.parametrize("options, rc_min, rc_max, reversed_, pwm", [
    (0, 1000, 2000, 0, 1409),      # 18 + 13 x 39 % through the 30 us dead zone: 23.07 m/s
    (0, 1100, 1900, 0, 1431),
    (0, 1000, 2000, 1, 1591),
    (1 << 1, 1100, 1900, 0, 1500),  # CRUISE_TRIM_THROTTLE: mid-stick is AIRSPEED_CRUISE
    (1 << 3, 1100, 1900, 0, 1000),  # CRUISE_TRIM_AIRSPEED: any stick, the low one is kept
])
def test_the_cruise_stick_asks_cruise_for_airspeed_cruise(options, rc_min, rc_max, reversed_, pwm):
    plane = FakePlane()
    plane.params.update({"FLIGHT_OPTIONS": options, "RC3_MIN": rc_min, "RC3_MAX": rc_max, "RC3_REVERSED": reversed_})
    assert flight_steps.cruise_airspeed_stick_pwm(plane) == pwm


def test_a_cruise_leg_that_is_not_the_level_cruise_fails_the_run():
    plane = FakePlane(height=118, mode="FBWA")
    plane.params["AIRSPEED_CRUISE"] = 30.0      # the fake's level cruise stays at 23 m/s
    plane.cruise_pct = 30

    def climbing(mtype: str, timeout: float = 10.0, cond: object = None) -> object:
        msg = FakePlane.wait_message(plane, mtype, timeout, cond)
        if mtype == "VFR_HUD":
            msg.climb = 0.4                     # trim1: still climbing back to the entry height
        return msg
    plane.wait_message = climbing
    with pytest.raises(FlightCheckFailed, match="CRUISE leg was not the level cruise: 30.1 m/s .* climbing 0.40"):
        cruise_state(plane)


def test_the_pilot_flies_fbwa_at_the_cruise_throttle():
    plane = FakePlane(height=118, mode="LOITER")
    flown = pilot_throttle(plane)
    assert {k: flown[k] for k in ("cruise_pct", "stick_pwm", "fbwa_pct")} == {"cruise_pct": 30, "stick_pwm": 1300,
                                                                            "fbwa_pct": 30}
    assert flown["cruise_pitch_deg"] == pytest.approx(3.0) and flown["fbwa_pitch_deg"] == pytest.approx(0.5)
    assert flown["cruise_airspeed_m_s"] == 23.07
    assert plane.mode == "FBWA" and plane.tx.get(3) == 1300


def test_untrimmed_fbwa_flies_faster_than_the_cruise_and_the_run_trim_brings_it_back():
    # demo2 (2026-10-13): FBWA held a pitch degrees below the level cruise's and flew several m/s too fast
    plane = FakePlane(height=118, mode="LOITER")
    pilot = pilot_throttle(plane)
    assert plane.airspeed() > 23.0 + 3.0
    record: dict = {}
    with CruisePitchTrim(plane, pilot, record):
        assert plane.params[PITCH_TRIM] == pytest.approx(2.5)
        assert plane.pitch_deg() == pytest.approx(3.0) and plane.airspeed() == pytest.approx(23.0)
        assert plane.mode == "FBWA"
    assert plane.params[PITCH_TRIM] == 0.0
    assert record["pitch_trim"] == {"name": PITCH_TRIM, "before_deg": 0.0, "run_deg": 2.5, "restored": True,
                                    "rounds": [{"trim_deg": 2.5, "pitch_deg": pytest.approx(3.0),
                                                "airspeed_m_s": pytest.approx(23.0)}]}


def test_the_trim_is_corrected_until_fbwa_flies_the_cruise_pitch():
    # FBWA's offset from its demand is not the one read before the trim (it moves with the speed)
    plane = FakePlane(height=118, mode="LOITER", trim_effect=0.9)
    pilot = pilot_throttle(plane)
    record: dict = {}
    with CruisePitchTrim(plane, pilot, record):
        assert abs(plane.pitch_deg() - 3.0) <= flight_steps.PITCH_MATCH_TOL_DEG
        assert abs(plane.airspeed() - 23.0) <= flight_steps.CRUISE_SPEED_TOL_M_S
    assert [r["trim_deg"] for r in record["pitch_trim"]["rounds"]] == [2.5, 2.75]
    assert record["pitch_trim"]["run_deg"] == 2.75 and plane.params[PITCH_TRIM] == 0.0


@pytest.mark.parametrize("options", [0, 1 << 8])
def test_the_run_trim_adds_to_a_trim_the_aircraft_already_has_and_sets_that_back(options):
    # ATTITUDE.pitch is sent less PTCH_TRIM_DEG unless FLIGHT_OPTIONS bit 8: the AHRS pitch is compared either way
    plane = FakePlane(height=118, mode="LOITER")
    plane.params.update({PITCH_TRIM: 1.0, "FLIGHT_OPTIONS": options})
    pilot = pilot_throttle(plane)
    assert pilot["cruise_pitch_deg"] == pytest.approx(3.0) and pilot["fbwa_pitch_deg"] == pytest.approx(1.5)
    with CruisePitchTrim(plane, pilot, {}):
        assert plane.params[PITCH_TRIM] == pytest.approx(2.5) and plane.airspeed() == pytest.approx(23.0)
    assert plane.params[PITCH_TRIM] == 1.0


def test_a_trim_that_fbwa_does_not_fly_fails_the_run_and_is_set_back():
    plane = FakePlane(height=118, mode="LOITER", trim_effect=0.0)
    pilot = pilot_throttle(plane)
    record: dict = {}
    with pytest.raises(FlightCheckFailed, match="did not fly the cruise pitch 3.00 deg in 3 trims"):
        with CruisePitchTrim(plane, pilot, record):
            pytest.fail("the doublets must not fly")
    assert plane.params[PITCH_TRIM] == 0.0 and record["pitch_trim"]["restored"] is True


def test_a_trimmed_fbwa_away_from_the_cruise_speed_fails_the_run():
    # the pitch matches, the speed does not (e.g. a cruise leg read in a gust)
    plane = FakePlane(height=118, mode="LOITER")
    pilot = {**pilot_throttle(plane), "cruise_airspeed_m_s": 21.0}
    with pytest.raises(FlightCheckFailed, match="trimmed to 2.5 deg flies 23.0 m/s, not the cruise 21.0 m/s"):
        with CruisePitchTrim(plane, pilot, {}):
            pytest.fail("the doublets must not fly")
    assert plane.params[PITCH_TRIM] == 0.0


def test_the_trim_is_set_back_after_a_failure_inside_and_a_failed_restore_does_not_hide_it():
    plane = FakePlane(height=118, mode="LOITER")
    pilot = pilot_throttle(plane)
    record: dict = {}
    with pytest.raises(FlightCheckFailed, match="below the floor"):
        with CruisePitchTrim(plane, pilot, record):
            def refuse(name: str, value: float) -> None:
                raise RuntimeError(f"could not set {name}={value}")
            plane.set_param = refuse
            raise FlightCheckFailed("below the floor")
    assert record["pitch_trim"]["restored"] is False
    assert record["pitch_trim"]["restore_error"] == "RuntimeError: could not set PTCH_TRIM_DEG=0.0"


def test_a_stick_that_fbwa_does_not_fly_as_the_cruise_fails_the_run():
    # e.g. FLIGHT_OPTIONS bit 10 maps the stick through a curve around TRIM_THROTTLE
    plane = FakePlane(height=118, mode="LOITER", fbwa_offset_pct=10)
    with pytest.raises(FlightCheckFailed, match="FBWA flies 40 % .* not the cruise 30 %"):
        pilot_throttle(plane)


def test_the_climb_enters_loiter_first_then_moves_its_height():
    # LOITER takes the height it is entered at as its target: a height change sent before entering it is lost
    plane = FakePlane(height=40, mode="AUTOTUNE")
    seconds = climb_to(plane, 120)
    assert plane.commands == [("LOITER", 120.0, M.MAV_FRAME_GLOBAL_RELATIVE_ALT)]
    assert plane.h >= 120 - flight_steps.CLIMB_MARGIN_M and 0 < seconds < 120


def test_loiter_alone_stays_at_the_height_it_was_entered_at():
    # round 2: "LOITER ... # climb back between cards" held 36 m, below the tuner's FTUN_ALT_MIN + 10 m
    plane = FakePlane(height=36, mode="FBWA")
    plane.set_mode("LOITER")
    plane.fly_for(30)
    assert plane.h == 36
    climb_to(plane, 120)
    assert plane.h >= 115


def test_a_climb_that_loiter_does_not_fly_fails_the_run():
    plane = FakePlane(height=40, mode="FBWA", loiter_follows_target=False)
    with pytest.raises(FlightCheckFailed, match="did not climb from 40.* to 120 m within 110 s"):
        climb_to(plane, 120)
    assert plane.t < 120 + 5


def test_a_refused_height_change_fails_the_run():
    plane = FakePlane(height=40, mode="FBWA", ack_result=M.MAV_RESULT_DENIED)
    with pytest.raises(FlightCheckFailed, match="not accepted"):
        climb_to(plane, 120)


def test_the_floor_records_the_lowest_height_and_lets_a_good_flight_pass():
    plane, record = FakePlane(height=100, mode="LOITER"), {}
    with HeightFloor(plane, MIN_HEIGHT_M, record):
        plane.fly_for(5)
    assert record["height"] == {"floor_m": MIN_HEIGHT_M, "lowest_m": 100.0, "breach": None}
    assert "recovery" not in record and plane.mode == "LOITER"
    assert plane.mav.message_hooks == []


def test_going_below_the_floor_fails_the_run_once_and_the_hook_is_gone_afterwards():
    plane, record = FakePlane(height=40, mode="FBWA", fbwa_sink=10), {}
    plane.tx.set(2, 1350)                          # a pitch-down card step: 1 m per 0.1 s
    with pytest.raises(FlightCheckFailed, match="height 29.0 m above home, below the run's floor of 30 m"):
        with HeightFloor(plane, 30, record):
            plane.fly_for(20)
    assert record["height"]["breach"].startswith("height 29.0 m")
    assert record["height"]["lowest_m"] == pytest.approx(29.0, abs=0.01)
    assert plane.mav.message_hooks == []
    plane.fly_for(5)                               # the vehicle's stop (disarm, log download) is not tripped again


def test_a_low_catapult_takeoff_climbs_to_the_test_height_before_the_pilots_steps(monkeypatch):
    from tools.tuning import sitl_driver
    plane = FakePlane(tkoff_alt=50)
    monkeypatch.setattr(sitl_driver, "takeoff", lambda v: (v.set_mode("TAKEOFF"), v.arm(), v.fly_for(5), 50.0)[-1])
    sitl_driver.takeoff_to_fbwa(plane)
    assert plane.mode == "FBWA" and plane.h >= flight_steps.TEST_HEIGHT_M - flight_steps.CLIMB_MARGIN_M
    assert [m for m, _ in plane.modes] == ["TAKEOFF", "LOITER", "FBWA"]


def test_after_a_floor_breach_the_aircraft_recovers_in_rtl_above_the_floor_before_the_run_ends():
    # sign_check2: the run failed at 17.7 m in an FBWA descent and the board was disarmed right there
    plane, record = FakePlane(height=40, mode="FBWA", fbwa_sink=10), {}
    plane.tx.set(2, 1350)
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        with HeightFloor(plane, 30, record):
            plane.fly_for(20)
    assert plane.t == pytest.approx(1.1 + record["recovery"]["seconds"], abs=1.0)
    assert plane.mode == flight_steps.RECOVERY_MODE == "RTL" and plane.h >= 30
    assert record["recovery"]["recovered"] is True and record["recovery"]["height_m"] >= 30
    assert record["recovery"]["seconds"] < 30


def test_a_recovery_that_does_not_climb_is_recorded_and_the_breach_is_still_the_failure():
    plane, record = FakePlane(height=40, mode="FBWA", fbwa_sink=10), {}
    plane.params["RTL_ALTITUDE"] = 10.0            # RTL holds a height below the floor
    plane.tx.set(2, 1350)
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        with HeightFloor(plane, 30, record):
            plane.fly_for(20)
    assert record["recovery"]["recovered"] is False and record["recovery"]["mode"] == "RTL"
    assert record["recovery"]["seconds"] >= flight_steps.CLIMB_SETTLE_S


def test_a_failing_recovery_does_not_hide_the_breach():
    plane, record = FakePlane(height=40, mode="FBWA", fbwa_sink=10), {}
    plane.tx.set(2, 1350)

    def refuse(mode):
        raise RuntimeError(f"mode {mode} not reached")
    plane.set_mode = refuse
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        with HeightFloor(plane, 30, record):
            plane.fly_for(20)
    assert record["recovery"] == {"mode": "RTL", "error": "RuntimeError: mode RTL not reached"}


def test_the_floor_trips_on_any_reading_of_the_link_but_only_once():
    plane, record = FakePlane(height=100), {}
    with HeightFloor(plane, 30, record) as floor:
        low = Msg("GLOBAL_POSITION_INT", relative_alt=12_000)
        with pytest.raises(FlightCheckFailed):
            floor._check(plane.mav, low)
        floor._check(plane.mav, low)               # tripped: later messages pass
        floor._check(plane.mav, Msg("VFR_HUD", throttle=20))
    assert record["height"]["lowest_m"] == 12.0
