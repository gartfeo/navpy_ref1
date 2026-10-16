"""A fixed-wing stand-in for the driver tests (flight_steps, vs_autotune, sitl_driver, sitl_faults).

It keeps the parts of ArduPlane the procedures depend on, in 0.1 s steps of simulated time:
- LOITER takes the height it is entered at as its target (commands_logic.cpp:979); a second LOITER
  request changes nothing (system.cpp:240-247); MAV_CMD_DO_CHANGE_ALTITUDE moves the target
  (GCS_Mavlink.cpp:749-763, 890-905), and LOITER flies towards it unless told to ignore it
  (FLIGHT_OPTIONS bit 12 with stick mixing, mode_loiter.cpp:155-161).
- FBWA and AUTOTUNE fly the throttle stick 1:1 (RC_Channel.cpp:376-390); LOITER and CRUISE fly the cruise
  throttle, more with the stick above mid (THROTTLE_NUDGE raises its airspeed target).
- CRUISE flies level at the airspeed its throttle stick asks for, AIRSPEED_MIN..MAX through RC3_DZ
  (navigation.cpp:186-188), with the cruise pitch at AIRSPEED_CRUISE; FBWA's stick centre holds trim_effect x
  PTCH_TRIM_DEG plus its own level offset, and its airspeed grows by speed_per_deg for every degree below the cruise
  pitch. ATTITUDE.pitch is reported less PTCH_TRIM_DEG unless FLIGHT_OPTIONS bit 8 (GCS_Mavlink.cpp:137-139).
- AUTOTUNE sinks; a moving roll or pitch stick tunes that axis until "<Axis>: Finished".
- FBWA sinks while a roll or pitch stick is off centre (the cards' and port-check's doublets).
- RTL climbs (or descends) to RTL_ALTITUDE at the LOITER rate.
- The tuning switch moved high in LOITER, CRUISE or FBWB starts a tuner session (one per switch movement): it
  starts only at FTUN_ALT_MIN + 10 m or higher, lasts tuner_s and ends with tuner_end; a mode change while it runs
  stops it ("mode changed").
- It flies at its airspeed along its course: LOITER turns at LOITER_TURN_DPS, CRUISE holds the course it was entered
  with, the other modes fly straight. GLOBAL_POSITION_INT carries the position (from LAT0/LON0) and the velocity.
Every step sends GLOBAL_POSITION_INT to the connection's message hooks, as pymavlink does.
Its airspeeds, cruise throttle and attitudes are neutral numbers, not any real aircraft's.
"""
from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from typing import Callable

from pymavlink import mavutil

M = mavutil.mavlink
STEP_S = 0.1
TAKEOFF_CLIMB_MPS = 10.0
LEVEL_CRUISE_M_S = 23.0        # the fake's level cruise, whatever AIRSPEED_CRUISE says
LOITER_TURN_DPS = 9.0          # a 150 m circle at the cruise
TUNER_MODES = ("LOITER", "CRUISE", "FBWB")
LAT0, LON0 = -35.0, 149.0
M_PER_DEG = 111_320.0
GAINS = {f"{a}_RATE_{k}": v for a in ("RLL", "PTCH") for k, v in (("FF", 0.3), ("P", 0.1), ("I", 0.1), ("D", 0.003))}
GAINS.update({"RLL2SRV_TCONST": 0.2, "RLL2SRV_RMAX": 120, "PTCH2SRV_TCONST": 0.3, "PTCH2SRV_RMAX_UP": 120,
              "PTCH2SRV_RMAX_DN": 120})


class Msg:
    def __init__(self, mtype: str, **fields: object) -> None:
        self._type = mtype
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._type


class FakeTx:
    def __init__(self) -> None:
        self.pwm = {ch: 1500 for ch in range(1, 17)}
        self.pwm[3] = 1000

    def set(self, channel: int, pwm: int) -> None:
        self.pwm[channel] = int(pwm)

    def get(self, channel: int) -> int:
        return self.pwm[channel]


class FakePlane:
    def __init__(self, tkoff_alt: float = 120.0, cruise_pct: float = 30.0, autotune_sink: float = 1.5,
                 fbwa_sink: float = 3.0, loiter_rate: float = 5.0, loiter_follows_target: bool = True,
                 loiter_sink: float = 0.0, autotune_needs_s: dict | None = None,
                 tuner_end: str = "FTUN: done", fbwa_offset_pct: float = 0.0,
                 ack_result: int = M.MAV_RESULT_ACCEPTED, height: float = 0.0, mode: str = "MANUAL",
                 cruise_pitch_deg: float = 3.0, fbwa_level_deg: float = 0.5, speed_per_deg: float = 1.5,
                 trim_effect: float = 1.0, tuner_s: float = 0.0, course_deg: float = 0.0) -> None:
        self.speedup = 1
        self.messages: list[str] = []
        self.params = {"TKOFF_ALT": tkoff_alt, "RC3_MIN": 1000, "RC3_MAX": 2000, "RC3_REVERSED": 0,
                       "FTUN_REPS": 4, "FTUN_ALT_MIN": 80, "FTUN_AXES": 3, "RTL_ALTITUDE": 100.0,
                       "PTCH_TRIM_DEG": 0.0, "FLIGHT_OPTIONS": 0, "RC3_DZ": 30, "THROTTLE_NUDGE": 1, "AIRSPEED_MIN": 18.0,
                       "AIRSPEED_MAX": 31.0, "AIRSPEED_CRUISE": LEVEL_CRUISE_M_S, **GAINS}
        self.cruise_pitch_deg, self.fbwa_level_deg = cruise_pitch_deg, fbwa_level_deg
        self.speed_per_deg, self.trim_effect = speed_per_deg, trim_effect
        self.param_sets: list[tuple[str, float]] = []
        self.cruise_pct, self.autotune_sink, self.fbwa_sink = cruise_pct, autotune_sink, fbwa_sink
        self.loiter_rate, self.loiter_follows_target, self.loiter_sink = loiter_rate, loiter_follows_target, loiter_sink
        self.autotune_needs_s = autotune_needs_s or {"Roll": 60.0, "Pitch": 30.0}
        self.tuner_end, self.fbwa_offset_pct, self.ack_result = tuner_end, fbwa_offset_pct, ack_result
        self.mode, self.armed, self.h, self.t = mode, False, height, 0.0
        self.loiter_target: float | None = height if mode == "LOITER" else None
        self.tx = FakeTx()
        self.mav = SimpleNamespace(message_hooks=[], target_system=1, target_component=1,
                                   mav=SimpleNamespace(command_int_send=self._command_int))
        self.pending_ack: Msg | None = None
        self.commands: list[tuple[str, float, float]] = []      # (mode, param1, param2) per DO_CHANGE_ALTITUDE
        self.modes: list[tuple[str, float]] = []                # (mode, height) per mode change
        self.sticks: dict[str, set[tuple[int, int]]] = {}       # mode -> (channel, pwm) seen off centre
        self.tuned_s = {"Roll": 0.0, "Pitch": 0.0}
        self.tuner_started_at: float | None = None
        self.tuner_s = tuner_s
        self.sessions: list[dict] = []      # one per tuner start: mode, axes, times, positions and courses
        self.switch_high = False
        self.north = self.east = 0.0
        self.course = course_deg
        self.card_starts: list[tuple[float, float]] = []        # (marker, height) when a card marker is set
        self.events: list[str] = []

    # ---- what fly_and_record uses ----------------------------------------------------------------
    def start(self) -> None:
        self.events.append("start")

    def stop(self) -> None:
        self.events.append("stop")

    def logs(self) -> list[Path]:
        return []

    def report(self) -> dict:
        return {}

    # ---- Vehicle --------------------------------------------------------------------------------
    def get_param(self, name: str) -> float:
        return self.params[name]

    def set_param(self, name: str, value: float) -> None:
        self.param_sets.append((name, float(value)))
        self.params[name] = float(value)
        if name == "FTUN_REPS" and 11 <= value <= 14:
            self.card_starts.append((float(value), self.h))

    def wait_ready(self) -> None:
        pass

    def arm(self) -> None:
        self.armed = True

    def set_mode(self, mode: str) -> None:
        if mode == "LOITER" and self.mode != "LOITER":
            self.loiter_target = self.h
        if mode != self.mode:
            self.modes.append((mode, self.h))
            running = self._running_session()
            if running is not None:
                running["end_t"] = self.t
                self.messages.append("FTUN: stopped: mode changed")
        self.mode = mode

    def state(self) -> dict:
        self._step()
        return {"roll": 0.0, "pitch": 0.0, "airspeed": LEVEL_CRUISE_M_S, "alt": self.h}

    def fly_for(self, seconds: float) -> None:
        for _ in range(round(seconds / STEP_S)):
            self._step()

    def pump(self, timeout: float = 0.05) -> None:
        self.fly_for(max(timeout, STEP_S))

    def wait_message(self, mtype: str, timeout: float = 10.0, cond: Callable | None = None) -> Msg | None:
        if mtype == "COMMAND_ACK":
            ack, self.pending_ack = self.pending_ack, None
            return ack if ack is not None and (cond is None or cond(ack)) else None
        self._step()
        if mtype == "VFR_HUD":
            return Msg("VFR_HUD", throttle=int(self.throttle_pct()), airspeed=self.airspeed(), climb=0.0)
        if mtype == "ATTITUDE":
            sent_less = 0.0 if int(self.params["FLIGHT_OPTIONS"]) >> 8 & 1 else self.params["PTCH_TRIM_DEG"]
            return Msg("ATTITUDE", pitch=math.radians(self.pitch_deg() - sent_less))
        if mtype == "GLOBAL_POSITION_INT":
            return self._position_msg()
        return None

    # ---- the aircraft ---------------------------------------------------------------------------
    def throttle_pct(self) -> float:
        if self.mode in ("FBWA", "AUTOTUNE"):
            return max(0.0, min(100.0, (self.tx.pwm[3] - 1000) / 10.0 + self.fbwa_offset_pct))
        return self.cruise_pct + (5.0 if self.tx.pwm[3] > 1530 else 0.0)

    def pitch_deg(self) -> float:
        """The attitude's pitch (the AHRS's, before the GCS's trim correction)."""
        if self.mode == "FBWA":
            return self.trim_effect * self.params["PTCH_TRIM_DEG"] + self.fbwa_level_deg
        return self.cruise_pitch_deg

    def airspeed(self) -> float:
        if self.mode == "CRUISE":
            p = self.params
            low = p["RC3_MIN"] + p["RC3_DZ"]
            control_in = max(0, int(100 * (self.tx.pwm[3] - low) / (p["RC3_MAX"] - low)))
            return (int(p["AIRSPEED_MAX"] - p["AIRSPEED_MIN"]) * control_in + int(p["AIRSPEED_MIN"] * 100)) / 100.0
        return LEVEL_CRUISE_M_S + self.speed_per_deg * (self.cruise_pitch_deg - self.pitch_deg())

    def _command_int(self, target_system: int, target_component: int, frame: int, command: int, current: int,
                     autocontinue: int, p1: float, p2: float, *rest: float) -> None:
        if command == M.MAV_CMD_DO_CHANGE_ALTITUDE:
            self.commands.append((self.mode, p1, p2))
            if self.ack_result == M.MAV_RESULT_ACCEPTED and p2 == M.MAV_FRAME_GLOBAL_RELATIVE_ALT:
                self.loiter_target = p1
            self.pending_ack = Msg("COMMAND_ACK", command=command, result=self.ack_result)

    def _step(self) -> None:
        self.t += STEP_S
        off_centre = [(ch, self.tx.pwm[ch]) for ch in (1, 2) if self.tx.pwm[ch] != 1500]
        self.sticks.setdefault(self.mode, set()).update(off_centre + [(3, self.tx.pwm[3])])
        if self.mode == "TAKEOFF" and self.armed:
            self.h = min(self.h + TAKEOFF_CLIMB_MPS * STEP_S, self.params["TKOFF_ALT"] - 2)
        elif self.mode == "LOITER":
            self._loiter()
        elif self.mode == "RTL":
            gap = self.params["RTL_ALTITUDE"] - self.h
            self.h += max(-self.loiter_rate * STEP_S, min(self.loiter_rate * STEP_S, gap))
        elif self.mode == "FBWA" and off_centre:
            self.h -= self.fbwa_sink * STEP_S
        elif self.mode == "AUTOTUNE":
            self._autotune(off_centre)
        self._move()
        self._tuner()
        msg = self._position_msg()
        for hook in list(self.mav.message_hooks):
            hook(self.mav, msg)

    def _position_msg(self) -> Msg:
        lat = LAT0 + self.north / M_PER_DEG
        lon = LON0 + self.east / (M_PER_DEG * math.cos(math.radians(LAT0)))
        speed = self.airspeed() * 100.0
        return Msg("GLOBAL_POSITION_INT", relative_alt=int(self.h * 1000), lat=int(round(lat * 1e7)),
                   lon=int(round(lon * 1e7)), vx=int(round(speed * math.cos(math.radians(self.course)))),
                   vy=int(round(speed * math.sin(math.radians(self.course)))))

    def distance_from(self, north: float, east: float) -> float:
        return math.hypot(self.north - north, self.east - east)

    def _move(self) -> None:
        if not self.armed or self.mode == "MANUAL":
            return
        if self.mode == "LOITER":
            self.course = (self.course + LOITER_TURN_DPS * STEP_S) % 360.0
        step = self.airspeed() * STEP_S
        self.north += step * math.cos(math.radians(self.course))
        self.east += step * math.sin(math.radians(self.course))

    def _running_session(self) -> dict | None:
        last = self.sessions[-1] if self.sessions else None
        return last if last is not None and last.get("end_t") is None else None

    def _tuner(self) -> None:
        """A session per movement of the tuning switch to high in a tuner mode, ending tuner_s later."""
        high = self.tx.pwm[9] >= 1800
        rising, self.switch_high = high and not self.switch_high, high
        running = self._running_session()
        if running is not None and not high:
            running["end_t"] = self.t
            self.messages.append("FTUN: stopped: switch off")
        elif running is not None and self.t >= running["start_t"] + self.tuner_s:
            running.update(end_t=self.t, end_course=self.course, end_mode=self.mode)
            if self.tuner_end.startswith("FTUN: done"):
                self.params["PTCH_RATE_P"] *= 1.5
            self.messages.append(self.tuner_end)
        if rising and self.mode in TUNER_MODES:
            self._run_tuner()

    def _loiter(self) -> None:
        self.h -= self.loiter_sink * STEP_S
        if self.loiter_follows_target and self.loiter_target is not None:
            gap = self.loiter_target - self.h
            self.h += max(-self.loiter_rate * STEP_S, min(self.loiter_rate * STEP_S, gap))

    def _run_tuner(self) -> None:
        if self.tuner_started_at is None:
            self.tuner_started_at = self.h
        if self.h < self.params["FTUN_ALT_MIN"] + 10:
            self.messages.append("FTUN: stopped: no safe conditions for 60 s")
            return
        self.messages.append("FTUN: start")
        self.sessions.append({"mode": self.mode, "axes": int(self.params["FTUN_AXES"]), "start_t": self.t,
                              "start_course": self.course, "start_pos": (self.north, self.east),
                              "throttle_pwm": self.tx.pwm[3], "end_t": None})

    def _autotune(self, off_centre: list[tuple[int, int]]) -> None:
        self.h -= self.autotune_sink * STEP_S
        for name, channel in (("Roll", 1), ("Pitch", 2)):
            if any(ch == channel for ch, _ in off_centre):
                before = self.tuned_s[name]
                self.tuned_s[name] += STEP_S
                if before < self.autotune_needs_s[name] <= self.tuned_s[name]:
                    self.messages.append(f"{name}: Finished")
