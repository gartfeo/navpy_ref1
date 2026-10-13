"""Parameter files, the full parameter list and the safety gates for a flight controller.

The gates refuse before anything is written: an armed vehicle, firmware that is not
ArduPilot simulation firmware (no SIM_OH_MASK, a SIM-group parameter that exists only
in SIM builds, SITL.cpp:1293-1296), or real servo outputs enabled (SIM_OH_MASK not 0;
ChibiOS RCOutput.cpp:722 drives a pin only if its bit is set).
"""
from __future__ import annotations

import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil

from tools.tuning.vehicle import Vehicle

PARAM_LIST_TIMEOUT_S = 180     # whole list: about 1000 parameters take seconds on USB, minutes on a radio
PARAM_QUIET_S = 2.0            # no new parameter for this long: ask again for the missing indices
PARAM_REREQUEST_BATCH = 30     # missing indices asked for per quiet period (the link's queue is short)
# Board values are float32. Two values match when they agree as float32 to within a few
# units in the last place; set_param's 1e-4 check is too loose for SIM_OPOS_LAT/LNG.
PARAM_MATCH_REL_TOL = 1e-6
PARAM_NAME = re.compile(r"[A-Z][A-Z0-9_]{0,15}")     # AP_Param names: 16 characters at most
# owned by the firmware, never written back: the storage layout version and the statistics
BOARD_OWNED_NAMES = frozenset({"FORMAT_VERSION"})
BOARD_OWNED_PREFIXES = ("STAT_",)
ORIGIN_NAMES = ("SIM_OPOS_LAT", "SIM_OPOS_LNG", "SIM_OPOS_ALT", "SIM_OPOS_HDG")


class Refused(RuntimeError):
    """The vehicle is not in a state this tool may change."""


@dataclass(frozen=True)
class ParamFile:
    path: Path
    required: bool      # every name must exist on the board (the run's settings), else optional


def parse_params(text: str, source: str = "<text>") -> list[tuple[str, float]]:
    """NAME VALUE or NAME,VALUE lines (spaces, tabs or a comma); '#' starts a comment."""
    out = []
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = re.split(r"[\s,]+", line)
        if len(fields) != 2 or not PARAM_NAME.fullmatch(fields[0]):
            raise ValueError(f"{source}:{number}: not a NAME VALUE line: {raw!r}")
        try:
            out.append((fields[0], float(fields[1])))
        except ValueError:
            raise ValueError(f"{source}:{number}: bad value: {raw!r}") from None
    return out


def read_param_file(path: Path) -> dict[str, float]:
    """A file's values; a later line for the same name wins, as when ArduPilot loads it."""
    return dict(parse_params(Path(path).read_text(encoding="utf-8"), str(path)))


def merge_param_files(files: list[ParamFile]) -> tuple[dict[str, float], set[str]]:
    """Values of the files in order (later files win) and the names the board must have."""
    values: dict[str, float] = {}
    required: set[str] = set()
    for f in files:
        file_values = read_param_file(f.path)
        values.update(file_values)
        if f.required:
            required.update(file_values)
    return values, required


def origin_params(origin: str) -> dict[str, float]:
    """SIM_OPOS_* from "lat,lon,alt,hdg" (degrees, metres above MSL, degrees), the bridge's origin."""
    parts = [float(p) for p in origin.split(",")]
    if len(parts) != len(ORIGIN_NAMES):
        raise ValueError(f"origin must be lat,lon,alt,hdg, not {origin!r}")
    lat, lon, _, hdg = parts
    if abs(lat) > 90 or abs(lon) > 180 or not 0 <= hdg < 360:
        raise ValueError(f"origin out of range: {origin!r}")
    return dict(zip(ORIGIN_NAMES, parts))


def _f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def same_value(a: float, b: float) -> bool:
    a32, b32 = _f32(a), _f32(b)
    return abs(a32 - b32) <= PARAM_MATCH_REL_TOL * max(1.0, abs(b32))


def changes(desired: dict[str, float], current: dict[str, float]) -> dict[str, float]:
    """The desired values the board has under that name but with another value."""
    return {n: v for n, v in desired.items() if n in current and not same_value(current[n], v)}


def board_owned(name: str) -> bool:
    return name in BOARD_OWNED_NAMES or name.startswith(BOARD_OWNED_PREFIXES)


def check_loaded(desired: dict[str, float], required: set[str], current: dict[str, float]) -> list[str]:
    """Refuse unless every required name exists and every present value matches.
    Returns the optional names the board does not have (e.g. EK2_ENABLE without EKF2)."""
    absent_required = sorted(n for n in required if n not in current)
    differing = sorted(changes(desired, current))
    if absent_required or differing:
        raise Refused(f"parameters not loaded: missing {absent_required}, differing "
                      + ", ".join(f"{n}={current[n]} (want {desired[n]})" for n in differing))
    return sorted(n for n in desired if n not in current)


def check_disarmed(heartbeat: mavutil.mavlink.MAVLink_heartbeat_message) -> None:
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise Refused("the vehicle is armed; disarm it first")


def check_sim_firmware(params: dict[str, float]) -> None:
    if "SIM_OH_MASK" not in params:
        raise Refused("no SIM_OH_MASK parameter: this is not ArduPilot simulation firmware")
    if params["SIM_OH_MASK"] != 0:
        raise Refused(f"SIM_OH_MASK is {params['SIM_OH_MASK']:g}: real servo outputs are enabled")


def check_sim_rate(params: dict[str, float]) -> None:
    """The onboard model steps once per main loop by 1/SIM_RATE_HZ (AP_Scheduler.cpp:423,
    SIM_Aircraft.cpp:856) while the board clock is real time: a mismatch runs it slow or fast."""
    if not same_value(params.get("SIM_RATE_HZ", -1), params.get("SCHED_LOOP_RATE", -2)):
        raise Refused(f"SIM_RATE_HZ {params.get('SIM_RATE_HZ')} differs from SCHED_LOOP_RATE "
                      f"{params.get('SCHED_LOOP_RATE')}: the onboard model would not run in real time")


def fetch_params(vehicle: Vehicle) -> dict[str, float]:
    """Every parameter, by index (PARAM_REQUEST_LIST, then missing indices one by one)."""
    conn = vehicle.mav
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)
    by_index: dict[int, tuple[str, float]] = {}
    count = None
    deadline = time.time() + PARAM_LIST_TIMEOUT_S
    while count is None or len(by_index) < count:
        if time.time() > deadline:
            raise RuntimeError(f"parameter list incomplete: {len(by_index)} of {count}")
        m = vehicle.wait_message("PARAM_VALUE", PARAM_QUIET_S)
        if m is None:
            if count is None:
                conn.mav.param_request_list_send(conn.target_system, conn.target_component)
            for i in [i for i in range(count or 0) if i not in by_index][:PARAM_REREQUEST_BATCH]:
                conn.mav.param_request_read_send(conn.target_system, conn.target_component, b"", i)
            continue
        if m.param_index >= m.param_count:     # 65535: the reply to a set or a read by name
            continue
        count = m.param_count
        by_index[m.param_index] = (m.param_id, m.param_value)
    return dict(by_index.values())


def write_backup(path: Path, params: dict[str, float], source: str) -> Path:
    """Write every parameter as NAME,VALUE to a new file and read it back.

    It counts as a verified backup only if it reads back to exactly the same names and
    values (repr round-trips a float); an existing file is never overwritten.
    """
    lines = [f"# parameters of {source}, {len(params)} in all, saved {time.strftime('%Y-%m-%d %H:%M:%S')}"]
    lines += [f"{name},{value!r}" for name, value in sorted(params.items())]
    with open(path, "x", encoding="ascii") as f:
        f.write("\n".join(lines) + "\n")
    if read_param_file(path) != params:
        raise Refused(f"{path} does not read back to the {len(params)} parameters fetched")
    return path
