"""Where inflight-tune flies the flight tuner: on straight legs (the default) or in LOITER.

After each manoeuvre the tuner hands the bank back to the mode (AP_FlightTune's hand-back, Ph 6). In LOITER
the manoeuvre's straight flight has left the circle by then, so LOITER asks for its full bank limit to get
back onto it, and the turn costs height and throttle that are not the tuner's (tune2, 2026-10-13: 1-3 m of
sink and about 10 % more throttle after every manoeuvre). On a straight CRUISE leg the mode holds the course
line it locked when it was entered, so after a manoeuvre it asks only for the small bank back onto that line.

A session of one axis takes a few kilometres of straight flight. To keep the aircraft near its start, each
axis is flown as its own tuner session on its own leg (FTUN_AXES one bit at a time), and between them, while
the tuner is not testing, LOITER turns the aircraft until its course points back at the first leg's start:
the second leg flies back towards it. A session that does not end with "FTUN: done" ends the legs; kept
gains of an earlier axis stay (FTUN_SAVE 0 keeps them in RAM, as within one session).
"""
from __future__ import annotations

import math

from tools.tuning.flight_steps import (FTUNE_SWITCH_CH, MESSAGE_TIMEOUT_S, PILOT_THROTTLE_PWM, THROTTLE_CH,
                                       TUNER_END_TEXTS, FlightCheckFailed, cruise_airspeed_stick_pwm, nudged_airspeed,
                                       wait_text)
from tools.tuning.vehicle import Vehicle

PATTERN_STRAIGHT = "straight"
PATTERN_LOITER = "loiter"
PATTERNS = (PATTERN_STRAIGHT, PATTERN_LOITER)
LEG_MODE = "CRUISE"
TURN_MODE = "LOITER"
TUNER_DONE = "FTUN: done"
AXIS_BITS = (1, 2)              # FTUN_AXES: roll, pitch
SWITCH_LOW_PWM, SWITCH_HIGH_PWM = 1000, 2000
# Straight flight on the leg before the tuning switch goes high. The mode change re-initialises TECS, and the
# tuner averages TECS's trim throttle for its pitch holds only from 20 s after it (AP_FlightTune TRIM_CONVERGE_S;
# a session waits at most 60 s for its first manoeuvre).
LEG_SETTLE_S = 20.0
# LOITER before the switch in the loiter pattern (the earlier procedure)
LOITER_SETTLE_S = 15.0
# After a session the switch goes low and the mode flies on this long before the next step
SESSION_END_S = 5.0
# The turn between legs ends once the ground course points this close to the first leg's start; CRUISE then
# locks the course it flies. A 150 m LOITER circle at the cruise turns about 9 deg/s, so readings
# TURN_CHECK_S apart cannot step across the window.
COURSE_TOL_DEG = 15.0
TURN_CHECK_S = 0.5
# A full LOITER circle at the cruise takes well under a minute; a turn that never lines up fails the run.
TURN_TIMEOUT_S = 120.0
M_PER_DEG_LAT = 111_320.0       # metres per degree of latitude (and of longitude at the equator)


def axis_blocks(axes: int) -> list[int]:
    """The FTUN_AXES value of each straight leg's session: one axis per leg, roll first."""
    return [bit for bit in AXIS_BITS if axes & bit]


def ground_track(sitl: Vehicle) -> tuple[float, float, float]:
    """(latitude, longitude, ground course in degrees from north) from GLOBAL_POSITION_INT."""
    msg = sitl.wait_message("GLOBAL_POSITION_INT", MESSAGE_TIMEOUT_S)
    if msg is None:
        raise FlightCheckFailed("no GLOBAL_POSITION_INT for the leg's course")
    return msg.lat * 1e-7, msg.lon * 1e-7, math.degrees(math.atan2(msg.vy, msg.vx)) % 360.0


def bearing_deg(lat: float, lon: float, to_lat: float, to_lon: float) -> float:
    """The bearing from one point to another, degrees from north, on a local flat earth (legs of kilometres)."""
    north = (to_lat - lat) * M_PER_DEG_LAT
    east = (to_lon - lon) * M_PER_DEG_LAT * math.cos(math.radians(lat))
    return math.degrees(math.atan2(east, north)) % 360.0


def course_error_deg(course: float, bearing: float) -> float:
    """course - bearing wrapped to [-180, 180)."""
    return (course - bearing + 180.0) % 360.0 - 180.0


def turn_towards(sitl: Vehicle, lat: float, lon: float) -> float:
    """LOITER until the ground course points at (lat, lon) within COURSE_TOL_DEG; returns the seconds flown."""
    sitl.set_mode(TURN_MODE)
    flown = 0.0
    while True:
        here_lat, here_lon, course = ground_track(sitl)
        if abs(course_error_deg(course, bearing_deg(here_lat, here_lon, lat, lon))) <= COURSE_TOL_DEG:
            return flown
        if flown >= TURN_TIMEOUT_S:
            raise FlightCheckFailed(f"{TURN_MODE} did not turn towards the start within {TURN_TIMEOUT_S:g} s")
        sitl.fly_for(TURN_CHECK_S)
        flown += TURN_CHECK_S


def leg_airspeed(sitl: Vehicle) -> float:
    """The airspeed of the legs: the one LOITER flies at the pilot's throttle stick, where the tuner has always been
    flown (nudged_airspeed). A roll test starts only above AIRSPEED_MIN x sqrt(1/cos 35 deg) + FTUN_SPD_MARGIN
    (AP_FlightTune wait_step), which can lie above AIRSPEED_CRUISE: a first leg at AIRSPEED_CRUISE (24.0 m/s) never
    started a roll test ("no safe conditions for 60 s"). The same speed also keeps the tune comparable with the
    LOITER runs."""
    return nudged_airspeed(sitl, PILOT_THROTTLE_PWM)


def enter_leg(sitl: Vehicle, airspeed: float) -> None:
    """A straight CRUISE leg at airspeed (the throttle stick is CRUISE's airspeed) on the course flown now, settled
    for LEG_SETTLE_S. The stick is set before the mode, and stays: the tuner stops on a stick that moves."""
    sitl.tx.set(THROTTLE_CH, cruise_airspeed_stick_pwm(sitl, airspeed))
    sitl.set_mode(LEG_MODE)
    sitl.fly_for(LEG_SETTLE_S)


def tuner_session(sitl: Vehicle, axes: int, timeout_s: float) -> str | None:
    """One tuner session on FTUN_AXES axes: switch high, wait for its end text, switch low. Returns the text."""
    sitl.set_param("FTUN_AXES", axes)
    print(f"[driver] tuning switch high (FTUN_AXES {axes})", flush=True)
    sitl.tx.set(FTUNE_SWITCH_CH, SWITCH_HIGH_PWM)
    end = wait_text(sitl, TUNER_END_TEXTS, timeout_s)
    sitl.tx.set(FTUNE_SWITCH_CH, SWITCH_LOW_PWM)
    sitl.fly_for(SESSION_END_S)
    return end


def fly_tuner(sitl: Vehicle, axes: int, pattern: str, timeout_s: float) -> list[dict]:
    """The tuner on axes in pattern; one record per session: its axes, mode and end text."""
    if pattern == PATTERN_LOITER:
        sitl.set_mode(TURN_MODE)
        sitl.fly_for(LOITER_SETTLE_S)
        return [{"axes": axes, "mode": TURN_MODE, "end": tuner_session(sitl, axes, timeout_s)}]
    if pattern != PATTERN_STRAIGHT:
        raise ValueError(f"unknown pattern {pattern!r}: {PATTERNS}")
    sessions: list[dict] = []
    start = None
    airspeed = leg_airspeed(sitl)
    for block in axis_blocks(axes):
        turn_s = 0.0
        if start is None:
            start = ground_track(sitl)[:2]
        else:
            turn_s = turn_towards(sitl, *start)
        enter_leg(sitl, airspeed)
        sessions.append({"axes": block, "mode": LEG_MODE, "airspeed_m_s": airspeed, "turn_s": turn_s,
                         "end": tuner_session(sitl, block, timeout_s)})
        if sessions[-1]["end"] is None or not sessions[-1]["end"].startswith(TUNER_DONE):
            break
    return sessions
