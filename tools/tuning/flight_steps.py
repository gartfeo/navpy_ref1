"""Pilot steps shared by the tuning flights (sitl_driver.py, vs_autotune.py, sitl_faults.py).

Stick inputs and STATUSTEXT waits, the run's height floor (HeightFloor: below it the run fails),
LOITER climbs that ArduPlane honours (climb_to), the pilot's throttle matched to the aircraft's level
cruise (pilot_throttle) and FBWA trimmed to its pitch for one run (CruisePitchTrim). Line numbers refer to the ArduPlane 4.6 fork (fw-worktrees/ardupilot-flight-tune).
"""
from __future__ import annotations

import argparse
import math
import statistics
import time
from types import TracebackType

from pymavlink import mavutil

from tools.tuning.vehicle import Vehicle

FTUNE_SWITCH_CH = 9
TUNER_END_TEXTS = ("FTUN: done", "FTUN: stopped", "internal error")
THROTTLE_CH = 3                # RCMAP_THROTTLE's default: the pilot's throttle stick
THROTTLE_LOW_PWM = 1000        # the transmitters' starting throttle (sitl_sim.Transmitter, vehicle.OverrideTransmitter)
# The pilot's throttle stick where a run does not match it to the cruise (takeoff_to_fbwa: port-check,
# inflight-tune, sitl_faults), and in LOITER while the tuner runs (vs-autotune flies its tuner the same way).
# FBWA flies it as 65 % throttle. Above mid-stick LOITER, CRUISE and FBWB raise their airspeed target
# (THROTTLE_NUDGE, radio.cpp:198-205, navigation.cpp:284): 1650 us is 63 % past the 30 us dead zone, so
# +0.26 x (AIRSPEED_MAX - AIRSPEED_CRUISE).
PILOT_THROTTLE_PWM = 1650
THROTTLE_NUDGE = "THROTTLE_NUDGE"
NUDGE_FROM_PCT = 50            # radio.cpp: the nudge starts above mid-stick ...
NUDGE_PER_PCT = 0.02           # ... and reaches the whole AIRSPEED_MAX - AIRSPEED_CRUISE gap at full stick
GAIN_NAMES = [f"{a}_RATE_{k}" for a in ("RLL", "PTCH") for k in ("FF", "P", "I", "D")] + \
             ["RLL2SRV_TCONST", "RLL2SRV_RMAX", "PTCH2SRV_TCONST", "PTCH2SRV_RMAX_UP", "PTCH2SRV_RMAX_DN"]

# After the takeoff has reached TKOFF_ALT the run fails below this height above home. Every step of these
# runs starts at TKOFF_ALT (120 m in the tuning runs) or above the tuner's FTUN_ALT_MIN (40-80 m); none is
# meant to come near the ground. In round 2 a vs-autotune card started at 36 m, struck the ground and the
# run still exited 0. 30 m leaves room for the pull-out of a manoeuvre that has gone wrong.
MIN_HEIGHT_M = 30.0
# The height the pilot's steps start at: peron_like.parm's TKOFF_ALT, which the tuning runs were sized on. An
# aircraft file may launch lower (a real catapult climb ends lower), and FBWA at the pilot's fixed throttle does
# not hold height: from such a takeoff the first HITL port-check sank through the 30 m floor during its
# doublets. The run climbs here in LOITER after such a takeoff (sitl_driver.takeoff_to_fbwa).
TEST_HEIGHT_M = 120.0
# A LOITER climb counts as done this close below its target: TECS closes the last metres slowly
# (TECS_TIME_CONST 5 s), and a few metres do not change any later step's start conditions.
CLIMB_MARGIN_M = 5.0
# The slowest climb accepted. TECS climbs at up to TECS_CLMB_MAX (5-10 m/s); an aircraft that cannot gain
# 1 m/s in LOITER cannot hold the test heights, and the run fails instead of flying on low.
CLIMB_RATE_MIN_MPS = 1.0
CLIMB_SETTLE_S = 30.0          # on top of the climb time: LOITER's entry turn and TECS's settling
CLIMB_CHECK_S = 1.0            # simulated seconds between height readings while climbing
MESSAGE_TIMEOUT_S = 5.0          # the longest wait for one MAVLink message or a command's answer
# After a floor breach the run has failed, but the aircraft is not left diving in the mode that failed until the
# vehicle's stop ends the flight (on a board: a disarm, which on a real aircraft would be a crash): RTL is ArduPlane's
# own recovery, it climbs to RTL_ALTITUDE and circles home. The stop comes once it is back above the floor, or after
# the time a LOITER climb of the same height is given (climb_to's limit).
RECOVERY_MODE = "RTL"
# The cruise: a straight CRUISE leg (wings level, TECS holding the entry height and the airspeed the throttle stick
# asks for) at AIRSPEED_CRUISE (cruise_airspeed_stick_pwm), then the medians over CRUISE_SAMPLE_S (TECS's throttle in
# LOITER spreads widely around its mean). Straight, not a LOITER turn: a turn needs more throttle and lift. Entered
# from a much faster FBWA, TECS took about five TECS_TIME_CONST of 5 s to settle level at the new speed (trim1,
# 2026-10-13).
CRUISE_SETTLE_S = 30.0
CRUISE_SAMPLE_S = 20.0
FBWA_SETTLE_S = 3.0            # FBWA's attitude after the mode change; the throttle slews 100 %/s
FBWA_SAMPLE_S = 5.0
THROTTLE_SAMPLE_S = 0.5        # one VFR_HUD and ATTITUDE reading per this much simulated flight
# FBWA must fly the cruise throttle this closely: both the stick map (RC_Channel.cpp:387) and VFR_HUD's
# throttle (system.cpp:473-475) truncate to whole percent.
THROTTLE_MATCH_TOL_PCT = 2.0
# FBWA's stick centre demands PTCH_TRIM_DEG (plus KFF_THR2PTCH x throttle, and STAB_PITCH_DOWN below
# TRIM_THROTTLE: Attitude.cpp:212, 617-627); a real aircraft is trimmed so that this is its cruise attitude
# (PTCH_TRIM_DEG, or AHRS_TRIM_Y when levelled). The HITL board frame is the model's body axis and the aircraft
# files have neither, so FBWA held a pitch degrees below the level cruise's, and the port-check doublets flew
# several m/s above the cruise (demo2, 2026-10-13). The run sets PTCH_TRIM_DEG for itself only (CruisePitchTrim).
PITCH_TRIM = "PTCH_TRIM_DEG"
FLIGHT_OPTIONS = "FLIGHT_OPTIONS"
# FLIGHT_OPTIONS bits (ArduPlane/defines.h): CRUISE's airspeed from the throttle stick through a curve around
# mid-stick (1), CRUISE at AIRSPEED_CRUISE whatever the stick (3), ATTITUDE.pitch sent without the trim correction (8)
CRUISE_TRIM_THROTTLE_BIT, CRUISE_TRIM_AIRSPEED_BIT, GCS_REMOVE_TRIM_PITCH_BIT = 1, 3, 8
# The CRUISE leg is the level cruise only when it flies level at AIRSPEED_CRUISE: trim1 (2026-10-13) sampled it
# still climbing back to its height at AIRSPEED_MIN, and FBWA matched to that pitch flew too slow. 0.25 m/s is
# 0.6 deg of flight path at 24 m/s.
LEVEL_CLIMB_TOL_M_S = 0.25
# The FBWA speed after a trim change converges with a time constant of 6-7 s (trim1); five.
TRIM_SETTLE_S = 36.0
# FBWA flies the cruise pitch this closely once trimmed; FBWA's offset from its demand changes with the speed, so the
# trim is corrected after each settle, at most PITCH_TRIM_ROUNDS times. 0.2 deg of pitch moves a cruising
# aircraft's speed by a fraction of a m/s.
PITCH_MATCH_TOL_DEG = 0.2
PITCH_TRIM_ROUNDS = 3
# The cruise leg must fly AIRSPEED_CRUISE, and the trimmed FBWA the cruise leg's airspeed, this closely: 4 % of a
# 24 m/s cruise; the untrimmed demo2 was several m/s off.
CRUISE_SPEED_TOL_M_S = 1.0


class FlightCheckFailed(RuntimeError):
    """A check of the run's procedure failed: the run is not valid, and the message says why."""


def add_floor_arg(p: argparse.ArgumentParser, default_m: float = MIN_HEIGHT_M) -> None:
    p.add_argument("--min-height", type=float, default=default_m, metavar="M",
                   help="m above home: once the takeoff has reached TKOFF_ALT the run fails below this "
                        "(default %(default)g)")


def stick_sequence(sitl: Vehicle, channel: int, steps: list[tuple[int, float]]) -> None:
    """Pilot stick inputs: list of (pwm, seconds)."""
    for pwm, seconds in steps:
        sitl.tx.set(channel, pwm)
        sitl.fly_for(seconds)
    sitl.tx.set(channel, 1500)


def read_gains(sitl: Vehicle) -> dict:
    return {n: sitl.get_param(n) for n in GAIN_NAMES}


def wait_text(sitl: Vehicle, texts: tuple[str, ...], seconds: float) -> str | None:
    """Fly until a STATUSTEXT containing one of texts arrives; returns it (None on timeout)."""
    seen = len(sitl.messages)
    end = time.time() + seconds / max(1, sitl.speedup)
    while time.time() < end:
        sitl.pump(0.2)
        for m in sitl.messages[seen:]:
            if any(t in m for t in texts):
                return m
        seen = len(sitl.messages)
    return None


class HeightFloor:
    """Inside the with-block the run fails (FlightCheckFailed) once the aircraft is below floor_m above home,
    and on leaving the block after such a breach the aircraft recovers in RECOVERY_MODE (recover_above).

    The check is a hook on the MAVLink connection (pymavlink mavfile.message_hooks, called for every
    received message in post_message), so every GLOBAL_POSITION_INT is seen whichever helper reads the
    link. It trips once and then lets messages pass, and leaving the block removes it: the vehicle's
    stop (a board's disarm and log download) must not trip it again. On leaving, record["height"] holds
    the floor, the lowest height seen and the breach (None when the floor held), and after a breach
    record["recovery"] what the recovery did (an error there is recorded; the breach is what is raised)."""

    def __init__(self, vehicle: Vehicle, floor_m: float, record: dict) -> None:
        self._vehicle = vehicle
        self._record = record
        self._conn = None
        self.floor_m = floor_m
        self.lowest_m: float | None = None
        self.breach: str | None = None

    def __enter__(self) -> HeightFloor:
        self._conn = self._vehicle.mav
        self._conn.message_hooks.append(self._check)
        return self

    def __exit__(self, kind: type[BaseException] | None, exc: BaseException | None,
                 trace: TracebackType | None) -> None:
        if self._check in self._conn.message_hooks:
            self._conn.message_hooks.remove(self._check)
        self._record["height"] = {"floor_m": self.floor_m, "lowest_m": self.lowest_m, "breach": self.breach}
        if self.breach is not None:
            try:
                self._record["recovery"] = recover_above(self._vehicle, self.floor_m)
            except Exception as exc:     # noqa: BLE001 - the breach stays the run's failure; the record keeps this
                self._record["recovery"] = {"mode": RECOVERY_MODE, "error": f"{type(exc).__name__}: {exc}"}

    def _check(self, conn: object, msg: object) -> None:
        if self.breach is not None or msg.get_type() != "GLOBAL_POSITION_INT":
            return
        height = msg.relative_alt / 1000.0
        self.lowest_m = height if self.lowest_m is None else min(self.lowest_m, height)
        if height < self.floor_m:
            self.breach = f"height {height:.1f} m above home, below the run's floor of {self.floor_m:g} m"
            raise FlightCheckFailed(self.breach)


def recover_above(vehicle: Vehicle, floor_m: float) -> dict:
    """RECOVERY_MODE until the aircraft is back at or above floor_m above home, or until the time a climb of that
    height is given (as climb_to). Returns the mode, whether it got there, the height and the simulated seconds."""
    vehicle.set_mode(RECOVERY_MODE)
    start = vehicle.state()["alt"]
    limit_s = max(0.0, floor_m - (start or 0.0)) / CLIMB_RATE_MIN_MPS + CLIMB_SETTLE_S
    flown = 0.0
    while True:
        height = vehicle.state()["alt"]
        recovered = height is not None and height >= floor_m
        if recovered or flown >= limit_s:
            print(f"[driver] {RECOVERY_MODE} after the floor breach: {height} m above home after {flown:g} s "
                  f"({'back above' if recovered else 'still below'} the floor of {floor_m:g} m)", flush=True)
            return {"mode": RECOVERY_MODE, "recovered": recovered, "height_m": height, "seconds": flown}
        vehicle.fly_for(CLIMB_CHECK_S)
        flown += CLIMB_CHECK_S


def change_altitude(sitl: Vehicle, height_m: float) -> None:
    """MAV_CMD_DO_CHANGE_ALTITUDE to height_m above home (GCS_Mavlink.cpp:890-905): it moves the height of the
    current navigation target, next_WP_loc (handle_change_alt_request, GCS_Mavlink.cpp:749-763)."""
    command = mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE
    conn = sitl.mav
    conn.mav.command_int_send(conn.target_system, conn.target_component,
                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command, 0, 0,
                              float(height_m), float(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT), 0, 0, 0, 0, 0)
    ack = sitl.wait_message("COMMAND_ACK", MESSAGE_TIMEOUT_S, lambda a: a.command == command)
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise FlightCheckFailed(f"height change to {height_m:g} m not accepted ({ack.result if ack else 'no answer'})")


def climb_to(sitl: Vehicle, height_m: float) -> float:
    """LOITER at height_m above home: returns the simulated seconds the climb took.

    Entering LOITER makes the current position and height its target (commands_logic.cpp:979
    do_loiter_at_location), so LOITER alone holds whatever height it was entered at. The height change
    is therefore sent after the mode change (a second LOITER request changes nothing, system.cpp:240-247),
    and LOITER's height target follows next_WP_loc (mode.cpp:208-226); it ignores it only with
    FLIGHT_OPTIONS bit 12 and stick mixing (mode_loiter.cpp:155-161), which then fails the climb."""
    sitl.set_mode("LOITER")
    change_altitude(sitl, height_m)
    start = sitl.state()["alt"]
    limit_s = max(0.0, height_m - (start or 0.0)) / CLIMB_RATE_MIN_MPS + CLIMB_SETTLE_S
    flown = 0.0
    while True:
        height = sitl.state()["alt"]
        if height is not None and height >= height_m - CLIMB_MARGIN_M:
            return flown
        if flown >= limit_s:
            raise FlightCheckFailed(f"LOITER did not climb from {start} m to {height_m:g} m within {limit_s:.0f} s "
                                    f"(at {height} m)")
        sitl.fly_for(CLIMB_CHECK_S)
        flown += CLIMB_CHECK_S


def gcs_pitch_trim_deg(sitl: Vehicle, trim_deg: float) -> float:
    """What ArduPlane takes off the AHRS pitch in ATTITUDE (GCS_Mavlink.cpp:137-139): PTCH_TRIM_DEG, unless
    FLIGHT_OPTIONS bit 8 says to send it as it is."""
    return 0.0 if int(sitl.get_param(FLIGHT_OPTIONS)) >> GCS_REMOVE_TRIM_PITCH_BIT & 1 else trim_deg


def median_flight(sitl: Vehicle, seconds: float) -> dict:
    """Medians over `seconds` of flight, one reading each per THROTTLE_SAMPLE_S: VFR_HUD.throttle (the throttle
    output in percent, system.cpp:466-478), .airspeed and .climb (m/s), and the AHRS pitch in degrees (ATTITUDE.pitch
    plus what ArduPlane took off it, gcs_pitch_trim_deg)."""
    sent_less = gcs_pitch_trim_deg(sitl, sitl.get_param(PITCH_TRIM))
    throttle, airspeed, climb, pitch = [], [], [], []
    for _ in range(max(1, round(seconds / THROTTLE_SAMPLE_S))):
        sitl.fly_for(THROTTLE_SAMPLE_S)
        hud = sitl.wait_message("VFR_HUD", MESSAGE_TIMEOUT_S)
        if hud is not None:
            throttle.append(hud.throttle)
            airspeed.append(hud.airspeed)
            climb.append(hud.climb)
        att = sitl.wait_message("ATTITUDE", MESSAGE_TIMEOUT_S)
        if att is not None:
            pitch.append(math.degrees(att.pitch) + sent_less)
    if not throttle or not pitch:
        raise FlightCheckFailed(f"no {'VFR_HUD' if not throttle else 'ATTITUDE'} reading")
    return {key: float(statistics.median(values)) for key, values in
            (("throttle_pct", throttle), ("airspeed_m_s", airspeed), ("climb_m_s", climb), ("pitch_deg", pitch))}


def cruise_airspeed_stick_pwm(sitl: Vehicle, airspeed: float | None = None) -> int:
    """The throttle stick (us) at which CRUISE flies `airspeed`, AIRSPEED_CRUISE by default (navigation.cpp:160-189):
    any stick with FLIGHT_OPTIONS bit 3 (the low stick is kept), mid-stick with bit 1, else the stick is the airspeed
    from AIRSPEED_MIN to AIRSPEED_MAX through RC_Channel::get_control_in (pwm_to_range_dz with RC3_DZ,
    RC_Channel.cpp:376-390; mirrored when RC3_REVERSED). With either bit CRUISE flies AIRSPEED_CRUISE only."""
    options = int(sitl.get_param(FLIGHT_OPTIONS))
    rc_min, rc_max = sitl.get_param(f"RC{THROTTLE_CH}_MIN"), sitl.get_param(f"RC{THROTTLE_CH}_MAX")
    cruise = sitl.get_param("AIRSPEED_CRUISE")
    want = cruise if airspeed is None else airspeed
    fixed = options >> CRUISE_TRIM_AIRSPEED_BIT & 1 or options >> CRUISE_TRIM_THROTTLE_BIT & 1
    if fixed and want != cruise:
        raise FlightCheckFailed(f"CRUISE cannot fly {want:g} m/s with FLIGHT_OPTIONS {options}: its stick is not "
                                f"its airspeed")
    if options >> CRUISE_TRIM_AIRSPEED_BIT & 1:
        return THROTTLE_LOW_PWM
    if options >> CRUISE_TRIM_THROTTLE_BIT & 1:
        return int(round((rc_min + rc_max) / 2))
    low, high = sitl.get_param("AIRSPEED_MIN"), sitl.get_param("AIRSPEED_MAX")
    fraction = (want - low) / (high - low)
    low_pwm = rc_min + sitl.get_param(f"RC{THROTTLE_CH}_DZ")
    percent = math.ceil(round(100.0 * fraction, 6))         # the whole control_in at or just above the cruise
    pwm = int(low_pwm + math.ceil(percent * (rc_max - low_pwm) / 100.0))
    return int(rc_min + rc_max - pwm) if sitl.get_param(f"RC{THROTTLE_CH}_REVERSED") else pwm


def throttle_control_in(sitl: Vehicle, pwm: int) -> int:
    """RC_Channel::get_control_in of the throttle stick at pwm, 0-100: pwm_to_range_dz with RC3_DZ above RC3_MIN
    (RC_Channel.cpp:376-390), mirrored when RC3_REVERSED."""
    rc_min, rc_max = sitl.get_param(f"RC{THROTTLE_CH}_MIN"), sitl.get_param(f"RC{THROTTLE_CH}_MAX")
    if sitl.get_param(f"RC{THROTTLE_CH}_REVERSED"):
        pwm = rc_min + rc_max - pwm
    low = rc_min + sitl.get_param(f"RC{THROTTLE_CH}_DZ")
    return max(0, min(100, int(100 * (min(pwm, rc_max) - low) / (rc_max - low)))) if pwm > low else 0


def nudged_airspeed(sitl: Vehicle, pwm: int) -> float:
    """The airspeed LOITER (and AUTO) fly with the throttle stick at pwm: AIRSPEED_CRUISE, raised above mid-stick by
    THROTTLE_NUDGE towards AIRSPEED_MAX, (control_in - 50) x 2 % of the gap (radio.cpp:198-205)."""
    cruise = sitl.get_param("AIRSPEED_CRUISE")
    control_in = throttle_control_in(sitl, pwm)
    if not sitl.get_param(THROTTLE_NUDGE) or control_in <= NUDGE_FROM_PCT:
        return cruise
    return cruise + (sitl.get_param("AIRSPEED_MAX") - cruise) * (control_in - NUDGE_FROM_PCT) * NUDGE_PER_PCT


def cruise_state(sitl: Vehicle) -> dict:
    """The aircraft's level cruise, read in this run (median_flight): a straight CRUISE leg at AIRSPEED_CRUISE
    (cruise_airspeed_stick_pwm), where TECS holds the entry height and the speed with the throttle and the pitch.
    FlightCheckFailed unless it flew level (LEVEL_CLIMB_TOL_M_S) at AIRSPEED_CRUISE (CRUISE_SPEED_TOL_M_S)."""
    sitl.tx.set(THROTTLE_CH, cruise_airspeed_stick_pwm(sitl))
    sitl.set_mode("CRUISE")
    sitl.fly_for(CRUISE_SETTLE_S)
    cruise = median_flight(sitl, CRUISE_SAMPLE_S)
    want = sitl.get_param("AIRSPEED_CRUISE")
    if abs(cruise["climb_m_s"]) > LEVEL_CLIMB_TOL_M_S or abs(cruise["airspeed_m_s"] - want) > CRUISE_SPEED_TOL_M_S:
        raise FlightCheckFailed(f"the CRUISE leg was not the level cruise: {cruise['airspeed_m_s']:.1f} m/s "
                                f"(AIRSPEED_CRUISE {want:g}), climbing {cruise['climb_m_s']:.2f} m/s")
    return cruise


def throttle_stick_pwm(percent: float, rc_min: float, rc_max: float, reversed_: bool) -> int:
    """The throttle stick (us) at which FBWA and AUTOTUNE fly `percent`. They pass the stick through
    (mode.cpp:300-310 output_pilot_throttle, reverse_thrust.cpp:126-143 with no dead zone):
    RC_Channel::pwm_to_range_dz(0) gives 100 x (pwm - RCn_MIN) / (RCn_MAX - RCn_MIN), mirrored when
    RCn_REVERSED (RC_Channel.cpp:376-390)."""
    pwm = round(rc_min + percent / 100.0 * (rc_max - rc_min))
    return int(rc_min + rc_max - pwm) if reversed_ else int(pwm)


def pilot_throttle(sitl: Vehicle) -> dict:
    """The pilot's throttle for FBWA and AUTOTUNE at the aircraft's cruise: cruise_state's throttle mapped to the
    stick with this vehicle's RC3_MIN/MAX/REVERSED, then flown in FBWA, where the median throttle must
    match the cruise (FLIGHT_OPTIONS bit 10 maps the stick through a curve, reverse_thrust.cpp:148-165;
    THR_MIN/THR_MAX clip it). Leaves the stick there, in FBWA. Returns the cruise's and FBWA's throttle, and the
    pitch and airspeed of both (CruisePitchTrim uses them)."""
    cruise = cruise_state(sitl)
    pwm = throttle_stick_pwm(cruise["throttle_pct"], sitl.get_param(f"RC{THROTTLE_CH}_MIN"),
                             sitl.get_param(f"RC{THROTTLE_CH}_MAX"), bool(sitl.get_param(f"RC{THROTTLE_CH}_REVERSED")))
    sitl.tx.set(THROTTLE_CH, pwm)
    sitl.set_mode("FBWA")
    sitl.fly_for(FBWA_SETTLE_S)
    fbwa = median_flight(sitl, FBWA_SAMPLE_S)
    want, flown = cruise["throttle_pct"], fbwa["throttle_pct"]
    print(f"[driver] cruise throttle {want:g} %, pitch {cruise['pitch_deg']:.2f} deg at {cruise['airspeed_m_s']:.1f} "
          f"m/s, climb {cruise['climb_m_s']:.2f} m/s: stick {pwm} us, FBWA flies {flown:g} % at pitch "
          f"{fbwa['pitch_deg']:.2f} deg", flush=True)
    if abs(flown - want) > THROTTLE_MATCH_TOL_PCT:
        raise FlightCheckFailed(f"FBWA flies {flown:g} % with the throttle stick at {pwm} us, not the cruise "
                                f"{want:g} % (check RC{THROTTLE_CH}_MIN/MAX, FLIGHT_OPTIONS bit 10, THR_MIN/THR_MAX)")
    return {"cruise_pct": want, "stick_pwm": pwm, "fbwa_pct": flown, "cruise_pitch_deg": cruise["pitch_deg"],
            "cruise_airspeed_m_s": cruise["airspeed_m_s"], "fbwa_pitch_deg": fbwa["pitch_deg"]}


class CruisePitchTrim:
    """Inside the with-block FBWA's stick centre flies the cruise attitude: PTCH_TRIM_DEG for this run only.

    The trim is moved by the cruise's AHRS pitch less FBWA's (pilot_throttle's readings), then, at most
    PITCH_TRIM_ROUNDS times, FBWA flies TRIM_SETTLE_S and the trim is corrected by what its pitch still misses; once
    within PITCH_MATCH_TOL_DEG, FBWA's median airspeed must be the cruise's within CRUISE_SPEED_TOL_M_S
    (FlightCheckFailed otherwise). On leaving, the old value is set back, also after a failure; a failed restore is
    recorded, not raised. record["pitch_trim"] holds the old and the run's trim, each round, and the restore."""

    def __init__(self, vehicle: Vehicle, pilot: dict, record: dict) -> None:
        self._vehicle = vehicle
        self._pilot = pilot
        self._record = record
        self.before: float | None = None

    def __enter__(self) -> CruisePitchTrim:
        v, pilot = self._vehicle, self._pilot
        self.before = v.get_param(PITCH_TRIM)
        want = pilot["cruise_pitch_deg"]
        trim = round(self.before + want - pilot["fbwa_pitch_deg"], 2)
        entry = self._record["pitch_trim"] = {"name": PITCH_TRIM, "before_deg": self.before, "rounds": []}
        try:
            for _ in range(PITCH_TRIM_ROUNDS):
                v.set_param(PITCH_TRIM, trim)
                v.fly_for(TRIM_SETTLE_S)
                fbwa = median_flight(v, FBWA_SAMPLE_S)
                entry["rounds"].append({"trim_deg": trim, "pitch_deg": fbwa["pitch_deg"],
                                        "airspeed_m_s": fbwa["airspeed_m_s"]})
                miss = want - fbwa["pitch_deg"]
                print(f"[driver] {PITCH_TRIM} {trim:g} deg: FBWA flies {fbwa['pitch_deg']:.2f} deg at "
                      f"{fbwa['airspeed_m_s']:.1f} m/s, the cruise {want:.2f} deg at "
                      f"{pilot['cruise_airspeed_m_s']:.1f} m/s", flush=True)
                if abs(miss) <= PITCH_MATCH_TOL_DEG:
                    break
                trim = round(trim + miss, 2)
            else:
                last = entry["rounds"][-1]
                raise FlightCheckFailed(f"FBWA did not fly the cruise pitch {want:.2f} deg in {PITCH_TRIM_ROUNDS} "
                                        f"trims (last {last['trim_deg']:g} deg: {last['pitch_deg']:.2f} deg)")
            entry["run_deg"] = trim
            if abs(fbwa["airspeed_m_s"] - pilot["cruise_airspeed_m_s"]) > CRUISE_SPEED_TOL_M_S:
                raise FlightCheckFailed(f"FBWA trimmed to {trim:g} deg flies {fbwa['airspeed_m_s']:.1f} m/s, not the "
                                        f"cruise {pilot['cruise_airspeed_m_s']:.1f} m/s")
        except BaseException:
            self._restore()
            raise
        return self

    def __exit__(self, kind: type[BaseException] | None, exc: BaseException | None,
                 trace: TracebackType | None) -> None:
        self._restore()

    def _restore(self) -> None:
        entry = self._record["pitch_trim"]
        if "restored" in entry:
            return
        try:
            self._vehicle.set_param(PITCH_TRIM, self.before)
            entry["restored"] = True
        except Exception as exc:     # noqa: BLE001 - the run's own failure, if any, stays the one raised
            entry["restored"] = False
            entry["restore_error"] = f"{type(exc).__name__}: {exc}"
