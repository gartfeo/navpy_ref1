"""The bridge's frame loop: one actuator frame in, one state reply and one CSV row out.

For each HIL_ACTUATOR_CONTROLS frame, in order (HIL link contract v2):
  1. board_clock.BoardClockStepper advances the plant to the last whole integration step at or
     before this frame's board time, each step with the outputs in effect at its start (the
     JSBSim program gets "iterate N"),
  2. the new outputs are latched (elevon unmix as ArduPilot's SIM_JSBSim); they act from the
     first step at or after this board time,
  3. HIL_STATE_QUATERNION is sent with this frame's time_usec (while the launcher holds the
     aircraft, the launcher's state: at rest, or moving along the catapult rail),
  4. one CSV row is written (frame index, board time, PC receive time, dt, event, flags,
     commands, key state).
Board time going back or jumping more than board_clock.MAX_FORWARD_JUMP_US is a board reboot:
the aircraft goes back to the origin (an ALERT when the previous frame was armed). A forward
jump longer than board_clock.GAP_FACTOR loop periods is stepped across with the held outputs and
reported as a GAP. The same frame again is answered with the same state (RETRANSMIT); other
content at the same board time is reported as an ERROR and answered with the state at that
time (CONFLICT). The PC clock only stamps the log (pc_rx_s, pc_tx_s).
"""
from __future__ import annotations

import csv
import math
import threading
from typing import Callable, TextIO

from pymavlink.dialects.v20 import common as mavlink2

from tools.tuning.hitl_bridge.link import ARMED_FLAG, LOCKSTEP_FLAG, MavlinkLink
from tools.tuning.hitl_bridge.board_clock import (CONFLICT, CRASH, GAP, REINIT, RETRANSMIT, BoardClockStepper,
                                                  StepResult)
from tools.tuning.hitl_bridge.state_fields import hil_state_fields

ActuatorMessage = mavlink2.MAVLink_hil_actuator_controls_message

LOG_COLUMNS = (
    "frame", "board_time_us", "board_dt_us", "pc_rx_s", "event", "flags", "substeps", "lost_frames", "armed",
    "on_launcher", "pwm1", "pwm2", "pwm3", "aileron", "elevator", "throttle", "catapult_n",
    "lat_deg", "lon_deg", "alt_msl_m", "roll_deg", "pitch_deg", "yaw_deg",
    "ias_m_s", "tas_m_s", "vn_m_s", "ve_m_s", "vd_m_s", "xacc_mg", "yacc_mg", "zacc_mg",
    "sent", "pc_tx_s",
)


class Bridge:
    """One actuator message in, one state message (and one log row) out."""

    def __init__(self, stepper: BoardClockStepper) -> None:
        self.stepper = stepper
        self.frames = 0                 # actuator frames handled: the log's frame index
        self.lockstep_seen = False

    def handle(self, msg: ActuatorMessage, pc_rx_s: float) -> tuple[dict, dict, StepResult]:
        controls = tuple(msg.controls)
        command = self.stepper.mixer(controls)
        armed = bool(msg.mode & ARMED_FLAG)
        result = self.stepper.on_actuators(msg.time_usec, controls, armed, msg.flags)
        self.frames += 1
        state = self.stepper.board_state(msg.time_usec)
        fields = hil_state_fields(state)
        row = {
            "frame": self.frames, "board_time_us": msg.time_usec, "board_dt_us": result.board_dt_us,
            "pc_rx_s": f"{pc_rx_s:.6f}", "event": result.event, "flags": msg.flags, "substeps": result.substeps,
            "lost_frames": result.lost_frames, "armed": int(armed), "on_launcher": int(result.on_launcher),
            "pwm1": controls[0], "pwm2": controls[1], "pwm3": controls[2],
            "aileron": f"{command.aileron:.4f}", "elevator": f"{command.elevator:.4f}",
            "throttle": f"{command.throttle:.4f}", "catapult_n": result.catapult_force_n,
            "lat_deg": f"{state.lat_deg:.7f}", "lon_deg": f"{state.lon_deg:.7f}",
            "alt_msl_m": f"{state.alt_msl_m:.3f}",
            "roll_deg": f"{math.degrees(state.roll_rad):.3f}", "pitch_deg": f"{math.degrees(state.pitch_rad):.3f}",
            "yaw_deg": f"{math.degrees(state.yaw_rad):.3f}",
            "ias_m_s": f"{state.ias_m_s:.3f}", "tas_m_s": f"{state.tas_m_s:.3f}",
            "vn_m_s": f"{state.vn_m_s:.3f}", "ve_m_s": f"{state.ve_m_s:.3f}", "vd_m_s": f"{state.vd_m_s:.3f}",
            "xacc_mg": fields["xacc"], "yacc_mg": fields["yacc"], "zacc_mg": fields["zacc"],
        }
        return fields, row, result

    def notices(self, msg: ActuatorMessage, result: StepResult) -> list[str]:
        """Console lines for this frame: its event (describe_event) and the first lockstep frame."""
        lines = [line for line in (describe_event(result, msg.time_usec, self.frames),) if line]
        if msg.flags & LOCKSTEP_FLAG and not self.lockstep_seen:
            self.lockstep_seen = True
            lines.append("bridge: the board runs in lockstep (flags bit 0): it waits for every reply; "
                         "lockstep is SITL only, a board never sets it")
        return lines


class LinkLog:
    """CSV link log, one row per actuator frame, LOG_COLUMNS in order (commands before the servo lag)."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream
        self._writer = csv.DictWriter(stream, fieldnames=LOG_COLUMNS)
        self._writer.writeheader()
        self.rows = 0

    def write(self, row: dict) -> None:
        self._writer.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self._stream.close()


def describe_event(result: StepResult, time_us: int, frame: int) -> str:
    """A console line for a REINIT, GAP, RETRANSMIT or CONFLICT frame, '' for the others."""
    if result.event == REINIT:
        previous = result.previous_time_us
        why = (f"board time went back from {previous} to {time_us} us" if time_us < previous else
               f"board time jumped {(time_us - previous) / 1e6:.3f} s forward")
        if result.was_armed:
            return (f"bridge: ALERT: {why} (frame {frame}) while the board was ARMED: board reboot or new link; "
                    "the aircraft was reset to the origin, this run is invalid")
        return f"bridge: {why} (frame {frame}): board reboot, aircraft reset to the origin"
    if result.event == GAP:
        return (f"bridge: GAP of {result.board_dt_us / 1000:.1f} ms board time before frame {frame} "
                f"(about {result.lost_frames} frames lost): stepped across with the held outputs")
    if result.event == RETRANSMIT:
        return (f"bridge: the frame at board time {time_us} us again (resent after a reconnect): "
                "answered with the same state, nothing stepped")
    if result.event == CRASH:
        return (f"bridge: CRASH at board time {time_us} us (frame {frame}): the plant left the envelope "
                f"({result.crash}); the aircraft stays at rest where it was last in it until the board reboots")
    if result.event == CONFLICT:
        return (f"bridge: ERROR: a second frame at board time {time_us} us with other outputs, mode or flags: "
                "the board broke the contract; ignored (answered with the state at that time)")
    return ""


def run_bridge(link: MavlinkLink, bridge: Bridge, log: LinkLog, clock: Callable[[], float],
               should_stop: Callable[[], bool], say: Callable[[str], None] = print) -> None:
    """Serve actuator frames until should_stop(); every frame gets its own reply, in order."""
    while not should_stop():
        frames = link.receive(should_stop)
        if not frames:
            continue
        pc_rx_s = clock()
        for msg in frames:
            fields, row, result = bridge.handle(msg, pc_rx_s)
            for line in bridge.notices(msg, result):
                say(line)
            sent = link.send_state(msg.time_usec, fields)
            row["sent"] = int(sent)
            row["pc_tx_s"] = f"{clock():.6f}" if sent else ""
            log.write(row)


class StopConditions:
    """When the run ends besides Ctrl+C/SIGTERM: no client for the grace time, stdin closed."""

    def __init__(self, link: MavlinkLink, client_grace_s: float) -> None:
        self._link = link
        self._grace_s = client_grace_s
        self.stdin_closed = threading.Event()
        self.reason = ""

    def watch_stdin(self, read: Callable[[], bytes]) -> None:
        def wait_for_eof() -> None:
            try:
                while read():
                    pass
            except OSError:
                pass
            self.stdin_closed.set()
        threading.Thread(target=wait_for_eof, name="stdin-eof", daemon=True).start()

    def __call__(self) -> bool:
        if self.stdin_closed.is_set():
            self.reason = "stdin closed"
            return True
        if self._grace_s > 0 and self._link.down_for() > self._grace_s:
            self.reason = f"no board link for more than {self._grace_s} s"
            return True
        return False
