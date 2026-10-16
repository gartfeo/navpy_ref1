"""What the JSBSim program says and is told: state output lines, input socket replies, command batches.

Checked against /usr/games/JSBSim (FlightGear 2020.3.18), probes in the session scratchpad
(hitl/proc_check, hitl/bridge_fix/servo_probe.py):
- The SOCKET output's first datagram is a label line,
    <LABELS> ,Time,sim-time-sec,frame,...,p-rad sec,...
  each column named by FGPropertyNode::GetPrintableName: the last part of the property
  name with every "_" replaced by a space ("velocities/p-rad_sec" -> "p-rad sec").
- Every following datagram is one line of 7-significant-digit values, Time first.
- The input socket answers "set successful" or "Iterations performed" per command line,
  each followed by the prompt "JSBSim> ", after a "Connected to JSBSim server" banner. A
  refused line ("Unknown property", "Not a leaf property") drops the rest of that batch.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

from tools.tuning.hitl_bridge.jsbsim_files import (DLAT_UDEG, DLON_UDEG, FRAME, FRAME_MOD, FRAME_MODULO,
                                                   MICRODEG_PER_DEG, SIM_TIME_S, STATE_PROPERTIES, JSBSimError)
from tools.tuning.hitl_bridge.environment import Wind
from tools.tuning.hitl_bridge.jsbsim_plant import (ALT_MSL_M, FORCE_LBS, FT_TO_M, IAS_FPS, MASS_SLUGS, N_PER_LBF,
                                                   P_RAD_S, PITCH_RAD, Q_RAD_S, R_RAD_S, ROLL_RAD, TAS_FPS, VD_FPS,
                                                   VE_FPS, VN_FPS, YAW_RAD)
from tools.tuning.hitl_bridge.physics import Origin, PlantState, SurfaceCommand

TIME_LABEL = "Time"                     # the SOCKET output's own first column (simulation time)
LABELS_PREFIX = b"<LABELS>"
SOCKET_SIGNIFICANT_DIGITS = 7
EXECUTION_BEGINNING = "JSBSim Execution beginning"
PROMPT = b"JSBSim> "
REPLY_BANNER = "Connected to JSBSim server"
REPLY_SET_OK = "set successful"
REPLY_ITERATE_OK = "Iterations performed"


# --- State output ---------------------------------------------------------------------
def printable_name(prop: str) -> str:
    """The column label JSBSim writes for a property (FGPropertyNode::GetPrintableName)."""
    return prop.rsplit("/", 1)[-1].replace("_", " ")


def expected_labels() -> list[str]:
    return [TIME_LABEL] + [printable_name(name) for name in STATE_PROPERTIES]


def parse_labels(datagram: bytes) -> list[str]:
    return [part.strip() for part in datagram[len(LABELS_PREFIX):].decode().strip().split(",") if part.strip()]


@dataclass(frozen=True)
class StateSample:
    sim_time_s: float
    frame: int
    frame_mod: int
    state: PlantState


def parse_state_line(datagram: bytes, origin: Origin) -> StateSample:
    """One SOCKET output line: Time, then STATE_PROPERTIES."""
    try:
        values = [float(part) for part in datagram.decode().strip().split(",")]
    except ValueError as exc:
        raise JSBSimError(f"unreadable state line {datagram[:120]!r}") from exc
    if len(values) != 1 + len(STATE_PROPERTIES):
        raise JSBSimError(f"state line has {len(values)} values, expected {1 + len(STATE_PROPERTIES)}")
    v = dict(zip(STATE_PROPERTIES, values[1:]))
    mass_slugs = v[MASS_SLUGS]
    fx, fy, fz = (v[name] / mass_slugs * FT_TO_M for name in FORCE_LBS)
    state = PlantState(
        lat_deg=origin.lat_deg + v[DLAT_UDEG] / MICRODEG_PER_DEG,
        lon_deg=origin.lon_deg + v[DLON_UDEG] / MICRODEG_PER_DEG,
        alt_msl_m=v[ALT_MSL_M],
        roll_rad=v[ROLL_RAD], pitch_rad=v[PITCH_RAD], yaw_rad=v[YAW_RAD],
        p_rad_s=v[P_RAD_S], q_rad_s=v[Q_RAD_S], r_rad_s=v[R_RAD_S],
        vn_m_s=v[VN_FPS] * FT_TO_M, ve_m_s=v[VE_FPS] * FT_TO_M, vd_m_s=v[VD_FPS] * FT_TO_M,
        ias_m_s=v[IAS_FPS] * FT_TO_M, tas_m_s=v[TAS_FPS] * FT_TO_M,
        fx_m_s2=fx, fy_m_s2=fy, fz_m_s2=fz)
    return StateSample(sim_time_s=v[SIM_TIME_S], frame=int(round(v[FRAME])),
                       frame_mod=int(round(v[FRAME_MOD])), state=state)


def printed_time_tolerance_s(time_s: float) -> float:
    """Half the last printed digit of a 7-significant-digit time (the output's rounding)."""
    if time_s <= 0.0:
        return 0.0
    return 0.5 * 10.0 ** (math.floor(math.log10(time_s)) - (SOCKET_SIGNIFICANT_DIGITS - 1))


BEHIND, TARGET, AHEAD = "behind", "target", "ahead"


def frame_relation(frame_mod: int, target_frame: int) -> str:
    """Where a packet's frame (modulo FRAME_MODULO) is relative to the target frame."""
    diff = (frame_mod - target_frame) % FRAME_MODULO
    if diff == 0:
        return TARGET
    return BEHIND if diff > FRAME_MODULO // 2 else AHEAD


# --- Input socket --------------------------------------------------------------------------
class ReplyReader:
    """Splits the input socket's reply stream into reply lines, without the prompts."""

    def __init__(self) -> None:
        self._buffer = b""

    def feed(self, data: bytes) -> list[str]:
        self._buffer += data
        *complete, self._buffer = self._buffer.split(b"\n")
        lines = []
        for raw in complete:
            while raw.startswith(PROMPT):
                raw = raw[len(PROMPT):]
            line = raw.decode(errors="replace").strip()
            if line:
                lines.append(line)
        return lines


def expect_replies(read_replies: Callable[[float], bytes], reader: ReplyReader, wanted: Sequence[str],
                   deadline: float, clock: Callable[[], float]) -> None:
    """Read until the wanted replies came, in order; JSBSimError on a refusal or at the deadline."""
    got: list[str] = []
    while len(got) < len(wanted):
        remaining = deadline - clock()
        if remaining <= 0:
            raise JSBSimError(f"JSBSim did not answer in time (got {got}, expected {list(wanted)})")
        for line in reader.feed(read_replies(remaining)):
            if line == REPLY_BANNER:
                continue
            if line not in (REPLY_SET_OK, REPLY_ITERATE_OK):
                raise JSBSimError(f"JSBSim refused a command: {line!r} (after {got}); "
                                  "the rest of that batch was dropped")
            got.append(line)
    if got != list(wanted):
        raise JSBSimError(f"unexpected JSBSim replies {got}, expected {list(wanted)}")


def _number(value: float) -> str:
    return f"{value:.9g}"


def iterate_command(properties: Sequence[str], command: SurfaceCommand, catapult_force_n: float, wind: Wind,
                    steps: int) -> bytes:
    """The lines of one batch (aileron, elevator, throttle, catapult in lbs, wind north and east in
    ft/s, then "iterate N"), sent in one write so they are read in one frame."""
    wind_north_fps, wind_east_fps = (v / FT_TO_M for v in wind.air_velocity_ne_m_s())
    values = (command.aileron, command.elevator, command.throttle, catapult_force_n / N_PER_LBF,
              wind_north_fps, wind_east_fps)
    lines = [f"set {name} {_number(value)}" for name, value in zip(properties, values)]
    lines.append(f"iterate {steps}")
    return ("\n".join(lines) + "\n").encode()


def batch_replies(properties: Sequence[str]) -> list[str]:
    return [REPLY_SET_OK] * len(properties) + [REPLY_ITERATE_OK]
