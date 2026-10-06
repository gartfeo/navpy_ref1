"""SCRATCH: fly ONE aircraft at ONE configuration and write ONE result file.

One process per aircraft, its own connection, its own configuration, its own
output. Nothing here is shared with a sibling, which is the whole point:

* the earlier in-process version ran one thread per aircraft, and at N=8 the
  single Python process reached 0.973 CPU. It starved its own readers -- three
  aircraft returned a single ATTITUDE sample for an entire hold -- and reported
  that as an aircraft failing to track its commanded pitch.
* a thread cannot escape the GIL. A process can, so N aircraft now spread over
  N cores instead of queueing behind one.

Run the parent (`scratch_sitl_scale.py`) to launch SITL and fan these out; this
script only ever knows about the aircraft it was given.

Delete with the other scratch harnesses once the scale question is closed out.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from collections.abc import Callable
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE)]

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20.ardupilotmega import (  # noqa: E402
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE,
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
    MAVLink_message,
)

from scripts.eval_nav_origin import require_nav_solution  # noqa: E402

# 7 = BODY_ROLL/PITCH/YAW_RATE_IGNORE, i.e. USE the attitude quaternion.
ATTITUDE_HOLD_MASK = (
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)

# AIRCRAFT-time. ArduPlane reverts to TECS 3 AIRCRAFT-seconds after the last
# SET_ATTITUDE_TARGET, so 2 Hz leaves six commands of margin. Every Hz here
# costs `speedup` Hz of wall-clock sends, which is what made the threaded
# version saturate its core.
COMMAND_HZ = 2.0

# TAKEOFF levels off near 50 m, about ten aircraft-seconds of air under a
# -20 deg dive. Without a climb the hold ends on the ground and the ground's
# attitude -- ~0 deg, whatever was commanded -- is reported as the pitch held.
CLIMB_PITCH_DEG = 15.0
CLIMB_THROTTLE = 0.9
CLIMB_LIMIT_S = 240.0

SPEEDUP_PARAM = "SIM_SPEEDUP"

# Longest a child may sleep in the socket waiting for telemetry. Bounded so the
# command cadence cannot slip: at 10x the cadence period is 50 ms, so 10 ms
# leaves four wakeups of margin. A blocking read returns as soon as a message
# lands, so this is a CEILING on idle time, not an added delay.
POLL_BLOCK_MAX_S = 0.01

# A phase ends on the AIRCRAFT'S clock. These bound how long it may wait in
# wall time for that clock to advance at all, so a dead link ends the run
# instead of hanging it. A stop, not a duration.
WALL_GUARD_FACTOR = 5.0
WALL_GUARD_FLOOR_S = 60.0


# How much the settled-tail bearings must agree before their circular mean is
# reported as a course. This is the length of the mean unit vector: 1.0 is a
# perfectly steady track, 0.0 means the samples cancel and point nowhere. A
# settled hold sits near 1.0, so this only rejects a track with no direction to
# report rather than trimming a noisy but real one.
BEARING_SPREAD_MIN = 0.1


SAMPLES_FILE = "samples.json"


def _write_samples(
    directory: Path,
    samples: list[tuple[float, int, float, float, float | None, float | None,
                        float | None, float | None]],
) -> bool:
    """Write the hold's attitude series beside the result.

    A count and a mean say whether the aircraft ended up where it was told. They
    cannot show the path it took -- rise time, overshoot, whether it settled or
    oscillated -- which is the whole content of a plant response. Kept out of
    result.json because the parent folds every result into one summary.json.
    """
    try:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / SAMPLES_FILE).write_text(
            json.dumps([
                [round(wall_s, 4), boot_ms, round(pitch, 3), round(roll, 3)]
                + [None if v is None else round(v, 2) for v in rest]
                for wall_s, boot_ms, pitch, roll, *rest in samples
            ]),
            encoding="utf-8",
        )
        return True
    except OSError:
        # Reported, never swallowed. This harness exists to COLLECT the series;
        # a silent failure here yields a sweep that says 40/40 succeeded while
        # cells hold no data, and the loss surfaces during analysis, long after
        # the flights are gone.
        return False


def _mean_bearing(degrees: list[float]) -> float | None:
    """Mean of compass bearings, which is NOT their arithmetic mean.

    Bearings live on a circle, so 359 and 1 are two degrees apart while their
    arithmetic mean is 180 -- the exact opposite direction. Averaging them as
    plain numbers is only safe for a track that never crosses north, and the
    heading this harness launches on crosses it constantly.

    The circular mean sums each bearing as a unit vector and takes the angle of
    the resultant, which has no seam. When those vectors cancel -- a track that
    spent the window pointing every which way -- the resultant has no direction
    to report, and returning some arbitrary angle would read as a settled
    course, so that case is None.
    """
    if not degrees:
        return None
    radians = [math.radians(value) for value in degrees]
    east = statistics.fmean(math.sin(r) for r in radians)
    north = statistics.fmean(math.cos(r) for r in radians)
    if math.hypot(east, north) < BEARING_SPREAD_MIN:
        return None
    # Re-wrapped AFTER rounding. Due north lands a hair below zero, which the
    # modulo lifts to 359.999... and rounding then snaps to 360.0 -- outside
    # the range this claims to return, and a value no other bearing here can
    # take.
    return round(math.degrees(math.atan2(east, north)) % 360.0, 2) % 360.0


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """Same convention as `navpy.modules.vehicle.attitude_command`."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def _connect(device: str, sysid: int, timeout_s: float) -> mavutil.mavfile | None:
    """Wait for proof the aircraft's process EXISTS. Not that it can fly.

    ArduPilot heartbeats from very early in boot, so this returns while the
    firmware is still calibrating. `_wait_ready` is what makes it flyable.
    """
    link = mavutil.mavlink_connection(device)
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        beat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if beat is not None and beat.get_srcSystem() == sysid:
            return link
    link.close()
    return None


# MAVLink's own words for "still starting up". ArduPilot reports these while it
# calibrates and initialises, and refuses to arm with `Arm: System not
# initialised` the whole time.
BOOTING_STATES = (
    mavutil.mavlink.MAV_STATE_UNINIT,
    mavutil.mavlink.MAV_STATE_BOOT,
    mavutil.mavlink.MAV_STATE_CALIBRATING,
)


def _wait_ready(link: mavutil.mavfile, sysid: int, timeout_s: float,
                chatter: "Chatter | None" = None) -> tuple[bool, float]:
    """Block until the firmware says it finished booting. Returns (ok, waited_s).

    WHY THIS EXISTS. Without it the child commanded TAKEOFF and armed as soon as
    the first heartbeat landed. Under load the simulator boots slowly, the arm
    was refused with `Arm: System not initialised`, the retry landed ~10 s later,
    and the aircraft sat on the ground for the whole run. Measured over four
    levels: every one of the 5 grounded aircraft carried that message and a
    rejected first arm; all 143 that flew carried neither.

    A position fix is required too -- an aeroplane cannot take off without one.
    It is NOT an origin proof, though: `_arm` holds the ground for that.
    """
    started_s = time.monotonic()
    deadline_s = started_s + timeout_s
    positioned = False
    while time.monotonic() < deadline_s:
        message = link.recv_match(
            type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True, timeout=1.0,
        )
        if message is None or message.get_srcSystem() != sysid:
            continue
        if chatter is not None and chatter.note(message):
            continue
        if message.get_type() == "GLOBAL_POSITION_INT":
            positioned = positioned or bool(message.lat or message.lon)
            continue
        if message.system_status in BOOTING_STATES:
            continue
        if positioned:
            return True, time.monotonic() - started_s
    return False, time.monotonic() - started_s


class Chatter:
    """Keeps what the firmware SAID, not just what it did.

    ArduPlane states the reason it refuses to arm, and the reason a takeoff does
    not start, in STATUSTEXT. Nothing here read that channel, so an aircraft that
    armed and then sat on the ground recorded `never climbed` with the firmware's
    own explanation discarded -- leaving the harness to guess at a cause the
    simulator had already named.
    """

    LIMIT = 40

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.dropped = 0

    def note(self, message: MAVLink_message) -> bool:
        """Record a STATUSTEXT. Returns True if the message was one."""
        if message.get_type() != "STATUSTEXT":
            return False
        text = message.text.strip("\x00").strip()
        if not text:
            return True
        if len(self.lines) < self.LIMIT:
            self.lines.append(f"{int(message.severity)}:{text}")
        else:
            self.dropped += 1
        return True


def _mode(link: mavutil.mavfile, sysid: int, name: str, timeout_s: float = 10.0,
          chatter: "Chatter | None" = None) -> bool:
    """Command a mode and CONFIRM the aircraft actually entered it.

    Sending `set_mode` proves only that a message left.
    """
    mapping = link.mode_mapping() or {}
    if name not in mapping:
        return False
    wanted = mapping[name]
    link.set_mode(wanted)
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        beat = link.recv_match(
            type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1.0
        )
        if beat is None or beat.get_srcSystem() != sysid:
            continue
        if chatter is not None and chatter.note(beat):
            continue
        if beat.get_type() != "HEARTBEAT":
            continue
        if beat.custom_mode == wanted:
            return True
    return False


def _set_param(link: mavutil.mavfile, sysid: int, name: str, value: float,
               timeout_s: float = 10.0) -> bool:
    """Set one parameter and confirm THIS aircraft's own echo."""
    link.mav.param_set_send(
        sysid, 0, name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if message is None or message.get_srcSystem() != sysid:
            continue
        if message.param_id.strip("\x00") != name:
            continue
        return abs(float(message.param_value) - float(value)) < 1e-3
    return False


def _read_param(link: mavutil.mavfile, sysid: int, name: str,
                timeout_s: float = 10.0) -> float | None:
    link.mav.param_request_read_send(sysid, 0, name.encode(), -1)
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if message is None or message.get_srcSystem() != sysid:
            continue
        if message.param_id.strip("\x00") == name:
            return float(message.param_value)
    return None


def _arm(link: mavutil.mavfile, sysid: int, attempts: int, timeout_s: float,
         chatter: "Chatter | None" = None) -> tuple[bool, int, list[int]]:
    """Arm and confirm from this aircraft's own HEARTBEAT.

    Held on the ground until the EKF owns its NED origin. `_wait_ready`'s
    nonzero GLOBAL_POSITION_INT is not that proof -- AHRS falls back to
    DCM/GPS and publishes coordinates with no EKF origin behind them -- and
    both callers pin ARMING_CHECK to 0, so the firmware's own pre-arm gate is
    gone as well. An origin taken in the air is permanent
    (`NavEKF3_core::setOrigin` rejects a second one) and puts a fixed bias on
    every altitude reported for the rest of the flight: +1.66 m, measured on
    the 2026-09-05 fleet run. Raising rather than returning False is
    deliberate -- this is not an aircraft that refused to arm, it is one that
    must not be asked.
    """
    # STATED, not inherited. pymavlink locks target_system onto the first
    # VEHICLE heartbeat it decodes -- whichever aircraft that was
    # (`mavutil.post_message`) -- and never sets target_component, leaving it
    # at 0, MAV_COMP_ID_ALL. `_connect` filters heartbeats by sysid precisely
    # because it does not assume one aircraft per link; the gate addresses
    # with both fields and accepts reports on target_system.
    link.target_system = sysid
    link.target_component = 1
    require_nav_solution(link, sysid)
    acks: list[int] = []
    for attempt in range(1, attempts + 1):
        link.mav.command_long_send(
            sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        deadline_s = time.monotonic() + timeout_s
        while time.monotonic() < deadline_s:
            message = link.recv_match(
                type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                blocking=True, timeout=1.0,
            )
            if message is None or message.get_srcSystem() != sysid:
                continue
            if chatter is not None and chatter.note(message):
                continue
            if message.get_type() == "STATUSTEXT":
                # Requested for the record, never a HEARTBEAT: reading base_mode
                # off one would raise.
                continue
            if message.get_type() == "COMMAND_ACK":
                if message.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    acks.append(int(message.result))
                continue
            if message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return True, attempt, acks
    return False, attempts, acks


class Flight:
    """One aircraft's link, its telemetry, and the phases flown on it."""

    def __init__(self, link: mavutil.mavfile, sysid: int, speedup: float,
                 chatter: "Chatter | None" = None) -> None:
        self._link = link
        self._sysid = sysid
        self._speedup = speedup
        self._chatter = chatter
        self._started_s = time.monotonic()
        # wall_s, boot_ms, pitch_deg, roll_deg, airspeed_mps
        self.samples: list[tuple[float, int, float, float, float | None,
                                 float | None, float | None, float | None]] = []
        self.alt_now_m: float | None = None
        self.alt_max_m = 0.0
        # Airspeed arrives on VFR_HUD, on its own cadence, so it is carried
        # forward onto each ATTITUDE row rather than interpolated. A plant
        # response depends on it: the same commanded pitch at 18 m/s and at
        # 30 m/s is not the same manoeuvre, and without it a sweep across
        # throttles produces numbers nothing can be fitted to.
        self.airspeed_now: float | None = None
        self.ground_speed_now: float | None = None
        self.course_now_deg: float | None = None
        self.path_angle_now_deg: float | None = None

    def _absorb(self, message: MAVLink_message) -> None:
        # This link should carry only this aircraft, but the filter costs
        # nothing and a router left in the path would otherwise fold a
        # neighbour's attitude into these numbers.
        if message.get_srcSystem() != self._sysid:
            return
        if self._chatter is not None and self._chatter.note(message):
            return
        kind = message.get_type()
        if kind == "ATTITUDE":
            self.samples.append((
                time.monotonic(), int(message.time_boot_ms),
                math.degrees(message.pitch), math.degrees(message.roll),
                self.airspeed_now, self.ground_speed_now,
                self.course_now_deg, self.path_angle_now_deg,
            ))
        elif kind == "VFR_HUD":
            self.airspeed_now = float(message.airspeed)
        elif kind == "GLOBAL_POSITION_INT":
            # Named explicitly. This used to be an `else` that read relative_alt
            # off whatever arrived, so adding any new message type here would
            # have raised on the first one that lacked the field.
            self.alt_now_m = message.relative_alt / 1000.0
            self.alt_max_m = max(self.alt_max_m, self.alt_now_m)
            # GROUND-frame velocity, which is the only place wind shows up.
            # Commanded pitch and airspeed are both air-relative and identical
            # in still air and a gale -- a wind sweep measured on them reads as
            # "wind does nothing". What wind changes is where the aircraft
            # actually goes: its speed over the ground, its course, and the
            # angle of its path relative to the ground rather than the air.
            north, east = message.vx / 100.0, message.vy / 100.0
            down = message.vz / 100.0
            self.ground_speed_now = math.hypot(north, east)
            self.course_now_deg = math.degrees(math.atan2(east, north)) % 360.0
            self.path_angle_now_deg = math.degrees(
                math.atan2(-down, self.ground_speed_now)
            ) if (self.ground_speed_now or down) else None

    def pump(self, wait_s: float = 0.0) -> None:
        """Drain what is queued, optionally SLEEPING IN THE SOCKET first.

        `wait_s` is what keeps this process off the CPU. The previous version
        polled non-blocking and then slept 1 ms, so every aircraft woke its
        process ~1000 times a second no matter how little telemetry arrived. At
        N aircraft that is N thousand wakeups a second of pure overhead, and it
        lands on the SAME physical cores the WSL VM runs the simulator on -- so
        the probe taxed the machine whose capacity it was trying to measure.

        A blocking read returns the moment a message arrives, so this wakes at
        the telemetry rate (~40 Hz) instead of 1000 Hz, without adding latency.
        """
        types = ["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD", "STATUSTEXT"]
        message = (
            self._link.recv_match(type=types, blocking=True, timeout=wait_s)
            if wait_s > 0.0
            else self._link.recv_match(type=types, blocking=False)
        )
        while message is not None:
            self._absorb(message)
            message = self._link.recv_match(type=types, blocking=False)

    def phase(self, cmd_pitch: float | None, cmd_roll: float,
              cmd_throttle: float, aircraft_s: float,
              done: Callable[[], bool]) -> str:
        """Command one attitude for `aircraft_s` of the AIRCRAFT'S OWN clock.

        Timed on ATTITUDE.time_boot_ms, not on wall time divided by SIM_SPEEDUP:
        SIM_SPEEDUP is the rate the simulator was ASKED for, and a starved one
        does not deliver it. Dividing by the configured rate would shorten every
        phase exactly when the machine is loaded.

        `cmd_pitch` of None commands nothing and only watches, which is how the
        telemetry-only comparison runs.
        """
        quaternion = (
            None if cmd_pitch is None
            else _euler_to_quaternion(
                math.radians(cmd_roll), math.radians(cmd_pitch), 0.0
            )
        )
        mark = len(self.samples)
        wall_guard_s = time.monotonic() + max(
            WALL_GUARD_FLOOR_S, WALL_GUARD_FACTOR * aircraft_s / self._speedup
        )
        commanded_at_s = 0.0
        while True:
            now_s = time.monotonic()
            if quaternion is not None and now_s >= commanded_at_s:
                self._link.mav.set_attitude_target_send(
                    int((now_s - self._started_s) * 1000) & 0xFFFFFFFF,
                    self._sysid, 0,
                    ATTITUDE_HOLD_MASK, quaternion,
                    0.0, 0.0, 0.0, cmd_throttle,
                )
                # Cadence is aircraft-time too, so scale to wall rate. The
                # CONFIGURED speedup is safe here: commanding faster than needed
                # is free, commanding slower loses the aircraft to TECS.
                commanded_at_s = now_s + 1.0 / (COMMAND_HZ * self._speedup)
            # Block until a message arrives or the next command falls due,
            # whichever is sooner -- never longer, or the cadence would slip.
            #
            # With nothing commanded there is no cadence to protect, so wait the
            # full slice. Deriving the wait from `commanded_at_s` alone would
            # give 0 here (it stays at its initial 0.0), turning the settle phase
            # and telemetry-only mode into a pure busy-loop -- worse than the
            # 1 ms sleep this replaced, and in exactly the modes that command
            # nothing and should therefore cost nothing.
            self.pump(
                POLL_BLOCK_MAX_S if quaternion is None
                else min(
                    POLL_BLOCK_MAX_S,
                    max(0.0, commanded_at_s - time.monotonic()),
                )
            )
            if done():
                return "condition"
            flown = self.samples[mark:]
            if flown and (flown[-1][1] - flown[0][1]) / 1000.0 >= aircraft_s:
                return "limit"
            if time.monotonic() >= wall_guard_s:
                return "stalled"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", required=True)
    parser.add_argument("--sysid", required=True, type=int)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--pitch-deg", type=float, default=None,
                        help="omit to only watch telemetry, commanding nothing")
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--wind-speed", type=float, default=None)
    parser.add_argument("--wind-dir", type=float, default=0.0)
    parser.add_argument("--hold-s", type=float, default=30.0,
                        help="aircraft seconds to hold the commanded attitude")
    parser.add_argument("--settle-s", type=float, default=20.0,
                        help="aircraft seconds to climb out under TAKEOFF")
    parser.add_argument("--climb-to-m", type=float, default=400.0)
    parser.add_argument("--floor-m", type=float, default=80.0)
    parser.add_argument("--connect-timeout", type=float, default=180.0)
    parser.add_argument("--speedup", type=float, default=None,
                        help="SIM_SPEEDUP to ENFORCE on this aircraft after "
                             "connecting. The launcher's request loses to the "
                             "value persisted in the instance eeprom, so a "
                             "speed sweep must set it per aircraft.")
    parser.add_argument("--ready-timeout", type=float, default=180.0,
                        help="wall seconds to wait for the firmware to finish "
                             "booting before commanding anything")
    return parser


def run(options: argparse.Namespace) -> dict[str, object]:
    result: dict[str, object] = {
        "sysid": options.sysid,
        "connection": options.connection,
        "cmd_pitch_deg": options.pitch_deg,
        "cmd_roll_deg": options.roll_deg,
        "wind_speed_mps": options.wind_speed,
        "wind_dir_deg": options.wind_dir if options.wind_speed is not None else None,
        "errors": [],
    }
    link = _connect(options.connection, options.sysid, options.connect_timeout)
    if link is None:
        result["errors"].append("no heartbeat")
        return result
    try:
        chatter = Chatter()
        # Before ANYTHING is commanded. A parameter set, a mode change or an arm
        # aimed at a firmware that is still calibrating is not a command, it is a
        # message thrown away -- and the run that follows reports the aircraft's
        # failure to obey it as a result.
        ready, waited_s = _wait_ready(
            link, options.sysid, options.ready_timeout, chatter=chatter
        )
        result["ready_wait_s"] = round(waited_s, 1)
        if not ready:
            result["errors"].append(
                f"firmware never reported ready in {options.ready_timeout:.0f}s"
            )
            result["statustext"] = chatter.lines
            return result

        # Wind BEFORE takeoff, so the whole flight runs in the configuration
        # this case claims to be testing -- not just the hold.
        if options.wind_speed is not None:
            ok = _set_param(link, options.sysid, "SIM_WIND_SPD", options.wind_speed)
            ok = _set_param(
                link, options.sysid, "SIM_WIND_DIR", options.wind_dir
            ) and ok
            if not ok:
                result["errors"].append("wind not confirmed")
        # ENFORCED here, not merely requested at launch. `SPEEDUP=` reaches
        # run_swarm.sh correctly, but SIM_SPEEDUP is an AP_Float persisted in
        # each instance's eeprom, and swarm_run_wsl.launch_command's own
        # docstring says it is the SUPERVISOR that verifies and persists it
        # before relaunch -- a supervisor this harness does not use. So a stale
        # 10 from an earlier run outlived a `SPEEDUP=1` request and a whole
        # level ran at ten times the rate it reported. Each aircraft is its own
        # SITL process, so each child can set its own.
        if options.speedup is not None:
            if not _set_param(
                link, options.sysid, SPEEDUP_PARAM, options.speedup
            ):
                result["errors"].append(
                    f"{SPEEDUP_PARAM} would not take the requested "
                    f"{options.speedup:g}"
                )
        speedup = _read_param(link, options.sysid, SPEEDUP_PARAM)
        result["sim_speedup"] = speedup
        if not speedup:
            # Never borrowed from a sibling: an aircraft that did not answer for
            # itself must not inherit a healthy rate it never earned.
            result["errors"].append(f"{SPEEDUP_PARAM} unreadable")
            return result

        result["takeoff_mode"] = _mode(
            link, options.sysid, "TAKEOFF", chatter=chatter
        )
        # The result used to be discarded. When it is False the aircraft arms
        # under checks that were never actually turned off, which is one of the
        # few states that produces the observed `armed but never climbed`.
        result["arming_check_off"] = _set_param(
            link, options.sysid, "ARMING_CHECK", 0.0
        )
        armed, attempts, acks = _arm(
            link, options.sysid, attempts=3, timeout_s=10.0, chatter=chatter
        )
        result.update({"armed": armed, "arm_attempts": attempts, "arm_acks": acks})
        if not armed:
            result["errors"].append("never armed")
            result["statustext"] = chatter.lines
            return result

        flight = Flight(link, options.sysid, speedup, chatter=chatter)
        # Climb out under TAKEOFF first. Reading through it rather than sleeping
        # keeps the link drained, so the phases that follow time live flight
        # instead of a backlog.
        flight.phase(None, 0.0, 0.0, options.settle_s, lambda: False)
        # Settling is not the measurement either.
        del flight.samples[:]

        if options.pitch_deg is None:
            # Telemetry only, and that means telemetry only: no GUIDED, no
            # climb, no SET_ATTITUDE_TARGET at all. The climb below commands
            # CLIMB_PITCH_DEG, so running it here would make the run that exists
            # to command nothing command something -- and the comparison it
            # feeds would be against a flight that was being flown.
            climb_end = "skipped"
            # Absolute time, because the PARENT reads these to line its CPU
            # samples up with the phase the clock was measured over. Monotonic
            # clocks have no shared reference across processes.
            result["hold_from_epoch"] = time.time()
            hold_end = flight.phase(
                None, 0.0, 0.0, options.hold_s, lambda: False
            )
            result["hold_to_epoch"] = time.time()
        else:
            if not _mode(link, options.sysid, "GUIDED", chatter=chatter):
                # ArduPlane discards SET_ATTITUDE_TARGET outside GUIDED. Left in
                # TAKEOFF the aircraft holds its trim attitude whatever it is
                # told, which looks like a healthy run of a wrong number.
                result["errors"].append("GUIDED not entered")
                result["statustext"] = chatter.lines
                return result
            climb_end = flight.phase(
                CLIMB_PITCH_DEG, 0.0, CLIMB_THROTTLE, CLIMB_LIMIT_S,
                lambda: (flight.alt_now_m or 0.0) >= options.climb_to_m,
            )
            # The climb attitude is not the measurement, and averaging +15 deg
            # into a commanded dive would report a pitch no phase of the flight
            # ever held.
            del flight.samples[:]
            result["hold_from_epoch"] = time.time()
            hold_end = flight.phase(
                options.pitch_deg, options.roll_deg, options.throttle,
                options.hold_s,
                lambda: (flight.alt_now_m if flight.alt_now_m is not None
                         else float("inf")) <= options.floor_m,
            )
            result["hold_to_epoch"] = time.time()
        result.update({
            "climb_end": climb_end,
            "hold_end": hold_end,
            "max_rel_alt_m": round(flight.alt_max_m, 1),
            "end_rel_alt_m": (
                None if flight.alt_now_m is None else round(flight.alt_now_m, 1)
            ),
            "samples": len(flight.samples),
            # The SERIES, not just its length. Aggregates (mean held pitch, its
            # error, the rate) answer "did it track the command"; identifying a
            # plant needs the response itself -- how it got there, how fast, with
            # what overshoot. Cleared before the hold above, so this is the hold
            # and nothing else: the commanded step and what the aircraft did
            # with it. Written beside the result rather than inside it, because
            # the parent folds every result into one summary.json and bulk data
            # does not belong there.
            # `samples_file` is published only once the file EXISTS -- filled in
            # after the write below, not promised before it.
            "samples_columns": ["wall_s", "boot_ms", "pitch_deg", "roll_deg",
                                "airspeed_mps", "ground_speed_mps",
                                "course_deg", "path_angle_deg"],
            "statustext": chatter.lines,
            "statustext_dropped": chatter.dropped,
        })
        if len(flight.samples) < 2:
            # The hold's ONLY exit condition is the floor guard, and it is
            # checked before the first sample can land -- so an aircraft already
            # under the floor when the hold begins ends it with no samples.
            # Reporting that as `no ATTITUDE received` blamed the link for a
            # climb that fell short, with the telemetry fine throughout.
            #
            # This says how far it got, and deliberately does NOT rule on
            # whether it was ever airborne: the parent owns that threshold
            # (`--airborne-m`, scratch_sitl_scale.py:651). Calling anything
            # under the 80 m floor `never left the ground` here would have
            # libelled an aircraft that flew to 79 m.
            if hold_end == "condition":
                result["errors"].append(
                    f"hold aborted at once: {flight.alt_max_m:.1f} m peak never "
                    f"cleared the {options.floor_m:.0f} m floor "
                    f"(climb ended '{climb_end}')"
                )
            else:
                result["errors"].append("no ATTITUDE received during the hold")
            return result
        wall_s = flight.samples[-1][0] - flight.samples[0][0]
        sim_s = (flight.samples[-1][1] - flight.samples[0][1]) / 1000.0
        # Second half only. The first half is still rotating towards the
        # commanded attitude, and averaging the transient in would understate a
        # pitch the aircraft actually reached and held.
        tail = flight.samples[len(flight.samples) // 2:]
        pitch_held = statistics.fmean(row[2] for row in tail)
        measured = (sim_s / wall_s) if wall_s > 0 else None
        result.update({
            "held_pitch_deg": round(pitch_held, 2),
            "pitch_error_deg": (
                None if options.pitch_deg is None
                else round(pitch_held - options.pitch_deg, 2)
            ),
            "held_roll_deg": round(statistics.fmean(row[3] for row in tail), 2),
            # Airspeed over the same settled tail as the pitch, so the two
            # describe one flight condition. A cell of a sweep is (commanded
            # pitch, wind, THROTTLE) in, and (held pitch, achieved airspeed) out
            # -- reporting the input throttle without the speed it produced
            # would leave the output half of the plant unmeasured.
            "held_airspeed_mps": (
                round(statistics.fmean(speeds), 2)
                if (speeds := [row[4] for row in tail if row[4] is not None])
                else None
            ),
            # Only when it was actually COMMANDED. With no pitch the quaternion
            # is None and `phase` never sends SET_ATTITUDE_TARGET (see the guard
            # in phase), so the throttle in options was carried but never
            # applied -- and recording it would put a sweep input in the
            # artefact that the aircraft never received.
            "cmd_throttle": (
                None if options.pitch_deg is None else options.throttle
            ),
            # Ground-frame outputs over the same settled tail. These are what a
            # wind sweep actually varies; held pitch and airspeed do not move
            # with wind at all, so a grid scored on them alone reports that wind
            # has no effect.
            **{
                name: (round(statistics.fmean(vals), 2) if vals else None)
                for name, vals in (
                    ("held_ground_speed_mps",
                     [r[5] for r in tail if r[5] is not None]),
                    ("held_path_angle_deg",
                     [r[7] for r in tail if r[7] is not None]),
                )
            },
            # Course is a COMPASS BEARING, not a number: it wraps at north, so
            # the arithmetic mean of 359 and 1 is 180 -- due south for an
            # aircraft flying due north. An aircraft holding a northerly track
            # straddles the wrap on nearly every sample, which is exactly the
            # heading this harness launches on, so the plain mean would have
            # reported garbage for the whole wind sweep.
            "held_course_deg": _mean_bearing(
                [r[6] for r in tail if r[6] is not None]
            ),
            "attitude_hz": round(len(flight.samples) / wall_s, 1) if wall_s else None,
            "clock_fidelity": round(measured, 4) if measured else None,
            # Scored against the simulator's OWN rate, never against 1.0.
            "clock_ratio": round(measured / speedup, 4) if measured else None,
            "wall_s": round(wall_s, 2),
            "sim_s": round(sim_s, 2),
        })
        if _write_samples(options.result.parent, flight.samples):
            result["samples_file"] = SAMPLES_FILE
        else:
            result["errors"].append(
                f"attitude series could not be written to {SAMPLES_FILE} -- the "
                "flight is measured but its response data is lost"
            )
        return result
    finally:
        link.close()


def main() -> int:
    options = _parser().parse_args()
    try:
        result = run(options)
    except BaseException as error:  # noqa: BLE001 - the parent reads the file
        result = {
            "sysid": options.sysid,
            "errors": [f"{type(error).__name__}: {error}"],
        }
    result["passed"] = not result.get("errors")
    options.result.parent.mkdir(parents=True, exist_ok=True)
    options.result.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("UAV_RESULT " + json.dumps(result), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
