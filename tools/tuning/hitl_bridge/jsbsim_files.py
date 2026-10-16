"""Run folder of the JSBSim program plant: model copy, generated system, reset, script, output.

The program (/usr/games/JSBSim, compiled from FlightGear 2020.3.18) is set up the way
ArduPilot's SITL JSBSim backend sets it up (fork libraries/SITL/SIM_JSBSim.cpp): a generated
start script (create_templates, :75-174) and the program started suspended with a simulation
rate and a nice sleep (start_JSBSim, :180-254). Differences, each from a check of this binary
(probe scripts in the session scratchpad: hitl/proc_check, hitl/bridge_fix/servo_probe.py):

- No ground Trim event (owner's decision): the initial conditions are the start pose
  (plant_options.StartPose): the rest attitude and height that the ground trim found for this
  model (the launch profile's ground_rest), the simple launcher's pitch and height, or the catapult rail's release
  (the bridge's default: the rail's end, moving at the release velocity).
- The SOCKET output writes 7 significant digits and ignores a precision attribute: too few
  for latitude/longitude in degrees (1e-5 deg, about 1 m). The generated system
  (navpy_hitl_output.xml) gives the position as offsets from the origin in micro-degrees and
  the frame number modulo 1e6 (exact). Systems run after the propagation in the same frame:
  the offsets matched the FLIGHTGEAR output's double latitude/longitude to 1e-10 deg.
- Servo lag (plant_options): the bridge sets raw commands (hitl/*-cmd-raw) and the generated
  system lags them into the model's inputs fcs/*-cmd-norm. A <system> loads before the
  model's <flight_control>, so it runs first in the same frame (probe: elevator-pos-rad was
  0.35 x the lagged command in the same packet). The FCS also runs in the stale frame an
  "iterate" starts with (jsbsim_process), at an unchanged simulation time, so each update is
  gated on the time having advanced: N updates per "iterate N" (probe: 7 FCS runs, 6
  updates; the lagged values equalled the Python recurrence to 7 digits).
- The wind (environment.py) is set with every batch, like the commands (the air velocity in
  atmosphere/wind-north-fps and wind-east-fps; probe: -16.4 north gave a 16.3 ft/s headwind
  on the resting aircraft's nose).
- The catapult force has no <function>, so its magnitude node has no value until set; the
  input socket refuses to set such a node ("Not a leaf property") and then drops the rest of
  the batch. The start script sets it to 0 first, and the raw commands are declared with a
  value for the same reason (probe: "set successful").
"""
from __future__ import annotations

import os
import shutil
from datetime import datetime
from pathlib import Path

from tools.tuning.hitl_bridge.jsbsim_plant import (AILERON_CMD, ALT_MSL_M, CATAPULT_MAGNITUDE_LBS, ELEVATOR_CMD,
                                                   ENGINE_RUNNING, FORCE_LBS, IAS_FPS, LAT_DEG, LON_DEG,
                                                   MASS_SLUGS, P_RAD_S, PITCH_RAD, Q_RAD_S, R_RAD_S, ROLL_RAD,
                                                   TAS_FPS, THROTTLE_CMD, VD_FPS, VE_FPS, VN_FPS, WIND_EAST_FPS,
                                                   WIND_NORTH_FPS, YAW_RAD)
from tools.tuning.hitl_bridge.physics import Origin
from tools.tuning.hitl_bridge.plant_options import FT_PER_M, GROUND_LEVEL, NO_LAG_ALPHA, StartPose, offset_lat_lon

LOCALHOST = "127.0.0.1"
NICE_SLEEP_S = 1e-8                     # --nice value, as SIM_JSBSim.cpp:211
RUN_END_TIME_S = 1e7                    # as SIM_JSBSim.cpp:108
ENGINE_START_TIME_S = 0.01              # as SIM_JSBSim.cpp:112
RESET_FILE = "reset"
SCRIPT_FILE = "start.xml"
OUTPUT_FILE = "output.xml"
SYSTEM_FILE = "navpy_hitl_output"
MODEL_END_TAG = "</fdm_config>"
RUN_DIR_ATTEMPTS = 1000                 # name suffixes tried when a run folder name is taken

# --- State output columns --------------------------------------------------------
SIM_TIME_S = "simulation/sim-time-sec"
FRAME = "simulation/frame"
FORCE_OUTPUT = "simulation/force-output"
FRAME_MOD = "hitl/frame-mod"            # generated system: frame modulo FRAME_MODULO
DLAT_UDEG = "hitl/dlat-udeg"            # generated system: (lat - origin lat) * 1e6
DLON_UDEG = "hitl/dlon-udeg"            # generated system: (lon - origin lon) * 1e6
FRAME_MODULO = 1_000_000                # fits 7 significant digits exactly
MICRODEG_PER_DEG = 1e6
STATE_PROPERTIES = (SIM_TIME_S, FRAME, FRAME_MOD, DLAT_UDEG, DLON_UDEG, ALT_MSL_M,
                    ROLL_RAD, PITCH_RAD, YAW_RAD, P_RAD_S, Q_RAD_S, R_RAD_S,
                    VN_FPS, VE_FPS, VD_FPS, IAS_FPS, TAS_FPS, *FORCE_LBS, MASS_SLUGS)

# --- Command inputs ------------------------------------------------------------------
# One batch sets aileron, elevator, throttle, catapult (lbs), wind north and east (ft/s).
# Without servo lag the surface commands go to the model's inputs; with it, to the generated
# system's raw inputs, each lagged into the model input of the same position.
DIRECT_COMMAND_PROPERTIES = (AILERON_CMD, ELEVATOR_CMD, THROTTLE_CMD, CATAPULT_MAGNITUDE_LBS,
                             WIND_NORTH_FPS, WIND_EAST_FPS)
LAGGED_COMMAND_PROPERTIES = ("hitl/aileron-cmd-raw", "hitl/elevator-cmd-raw", "hitl/throttle-cmd-raw",
                             CATAPULT_MAGNITUDE_LBS, WIND_NORTH_FPS, WIND_EAST_FPS)
SERVO_GATE = "hitl/servo-gate"          # 1 when the simulation time advanced since the last update
SERVO_LAST_TIME_S = "hitl/servo-last-time"


class JSBSimError(RuntimeError):
    """The JSBSim program cannot be started, refused a command or stopped answering."""


def has_servo_lag(lag_alpha: float) -> bool:
    return lag_alpha < NO_LAG_ALPHA


def command_properties(lag_alpha: float) -> tuple[str, ...]:
    """The properties one batch sets: aileron, elevator, throttle, catapult (lbs), wind north, east."""
    return LAGGED_COMMAND_PROPERTIES if has_servo_lag(lag_alpha) else DIRECT_COMMAND_PROPERTIES


def _fcs_function(name: str, body: str, output: str = "") -> str:
    out = f"\n      <output>{output}</output>" if output else ""
    return f"""    <fcs_function name="{name}">
      <function>
        {body}
      </function>{out}
    </fcs_function>
"""


def _offset(prop: str, reference_deg: float) -> str:
    return (f"<product> <difference> <property>{prop}</property> <value>{reference_deg!r}</value> </difference>"
            f" <value>{MICRODEG_PER_DEG:.0f}</value> </product>")


def _servo_channel(lag_alpha: float) -> str:
    """output += gate * alpha * (raw - output), written to the model input; then the time."""
    lags = ""
    for raw, model_input in zip(LAGGED_COMMAND_PROPERTIES[:3], DIRECT_COMMAND_PROPERTIES[:3]):
        lagged = raw.replace("-raw", "-lagged")
        lags += _fcs_function(lagged, f"<sum> <property>{lagged}</property> <product> <property>{SERVO_GATE}"
                                      f"</property> <value>{lag_alpha!r}</value> <difference> <property>{raw}"
                                      f"</property> <property>{lagged}</property> </difference> </product> </sum>",
                              output=model_input)
    return ('  <channel name="navpy HITL servo lag">\n'
            + _fcs_function(SERVO_GATE, f"<gt> <property>{SIM_TIME_S}</property> <property>{SERVO_LAST_TIME_S}"
                                        "</property> </gt>")
            + lags
            + _fcs_function(SERVO_LAST_TIME_S, f"<property>{SIM_TIME_S}</property>")
            + "  </channel>\n")


def system_xml(origin: Origin, lag_alpha: float) -> str:
    """The generated system: servo lag (if any), position offsets from the origin, frame modulo."""
    declarations = servo = ""
    if has_servo_lag(lag_alpha):
        declarations = "".join(f'  <property value="0"> {raw} </property>\n'
                               for raw in LAGGED_COMMAND_PROPERTIES[:3])
        servo = _servo_channel(lag_alpha)
    return ('<?xml version="1.0"?>\n'
            '<!-- Generated by navpy tools/tuning/hitl_bridge/jsbsim_files.py: servo lag as ArduPilot SITL, and\n'
            '     outputs the 7-digit SOCKET output of this JSBSim build can carry. -->\n'
            '<system name="navpy HITL">\n' + declarations + servo
            + '  <channel name="navpy HITL output">\n'
            + _fcs_function(DLAT_UDEG, _offset(LAT_DEG, origin.lat_deg))
            + _fcs_function(DLON_UDEG, _offset(LON_DEG, origin.lon_deg))
            + _fcs_function(FRAME_MOD, f"<mod> <property>{FRAME}</property> <value>{FRAME_MODULO}</value> </mod>")
            + "  </channel>\n</system>\n")


def add_system_to_model(model_xml: str) -> str:
    if model_xml.count(MODEL_END_TAG) != 1:
        raise JSBSimError(f"the model must contain {MODEL_END_TAG} exactly once")
    return model_xml.replace(MODEL_END_TAG, f' <system file="{SYSTEM_FILE}"/>\n{MODEL_END_TAG}')


def reset_xml(origin: Origin, start: StartPose = GROUND_LEVEL) -> str:
    """Initial conditions: the start pose at its offset from the origin (altitude is above ground),
    with its ground velocity. The velocity is given north/east/down, not as <vt>: the program applies
    <vt> after them and a 0 there stops the aircraft (probe: rail_launcher/ic_probe.py, with and
    without <vt>); at rest the three zeros give exactly the state <vt> 0 gave."""
    lat_deg, lon_deg = offset_lat_lon(origin.lat_deg, origin.lon_deg, start.north_m, start.east_m)
    vn_fps, ve_fps, vd_fps = (v * FT_PER_M for v in start.velocity_ned_m_s)
    return f"""<?xml version="1.0"?>
<initialize name="{RESET_FILE}">
  <latitude unit="DEG" type="geodetic"> {lat_deg!r} </latitude>
  <longitude unit="DEG"> {lon_deg!r} </longitude>
  <elevation unit="M"> {origin.ground_alt_msl_m!r} </elevation>
  <altitude unit="FT"> {start.height_agl_ft!r} </altitude>
  <vnorth unit="FT/SEC"> {vn_fps!r} </vnorth>
  <veast unit="FT/SEC"> {ve_fps!r} </veast>
  <vdown unit="FT/SEC"> {vd_fps!r} </vdown>
  <phi unit="DEG"> 0.0 </phi>
  <theta unit="DEG"> {start.pitch_deg!r} </theta>
  <psi unit="DEG"> {origin.heading_deg!r} </psi>
</initialize>
"""


def start_script_xml(aircraft: str, control_port: int, dt_s: float) -> str:
    """As SIM_JSBSim's create_templates, without the ground Trim event."""
    return f"""<?xml version="1.0" encoding="utf-8"?>
<runscript name="navpy HITL bridge: {aircraft}">
  <use aircraft="{aircraft}" initialize="{RESET_FILE}"/>
  <input port="{control_port}"/>
  <run start="0" end="{RUN_END_TIME_S:.0f}" dt="{dt_s!r}">
    <event name="start engine">
      <condition> {SIM_TIME_S} le {ENGINE_START_TIME_S} </condition>
      <set name="{ENGINE_RUNNING}" value="1"/>
      <!-- gives the catapult magnitude node a value: the input socket refuses to set it before -->
      <set name="{CATAPULT_MAGNITUDE_LBS}" value="0"/>
      <notify/>
    </event>
  </run>
</runscript>
"""


def output_xml(state_port: int, rate_hz: float) -> str:
    """A SOCKET output to us over UDP once per integration step, STATE_PROPERTIES in order."""
    props = "\n".join(f"  <property> {name} </property>" for name in STATE_PROPERTIES)
    return f"""<?xml version="1.0"?>
<output name="{LOCALHOST}" type="SOCKET" protocol="UDP" port="{state_port}" rate="{rate_hz:.9g}">
{props}
</output>
"""


def rate_hz_from_dt(dt_s: float) -> float:
    return 1.0 / dt_s


def prepare_run_folder(model_dir: Path, run_dir: Path, origin: Origin, ports: tuple[int, int], dt_s: float,
                       lag_alpha: float, start: StartPose = GROUND_LEVEL) -> None:
    """Fill a new run folder: model copy with the generated system and reset, script, output.

    ports: (the program's control port, our state port)."""
    name = model_dir.name
    aircraft_dir = run_dir / "aircraft" / name
    shutil.copytree(model_dir, aircraft_dir)
    model_file = aircraft_dir / f"{name}.xml"
    model_file.write_text(add_system_to_model(model_file.read_text(encoding="utf-8")), encoding="utf-8")
    (aircraft_dir / f"{SYSTEM_FILE}.xml").write_text(system_xml(origin, lag_alpha), encoding="utf-8")
    (aircraft_dir / f"{RESET_FILE}.xml").write_text(reset_xml(origin, start), encoding="utf-8")
    (run_dir / SCRIPT_FILE).write_text(start_script_xml(name, ports[0], dt_s), encoding="utf-8")
    (run_dir / OUTPUT_FILE).write_text(output_xml(ports[1], rate_hz_from_dt(dt_s)), encoding="utf-8")


def new_run_dir(run_root: Path, now: datetime) -> Path:
    run_root.mkdir(parents=True, exist_ok=True)
    base = f"{now.strftime('%Y%m%dT%H%M%SZ')}-{os.getpid()}"
    for index in range(RUN_DIR_ATTEMPTS):
        candidate = run_root / (base if index == 0 else f"{base}-{index}")
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        return candidate
    raise JSBSimError(f"no free run folder name under {run_root}")


def jsbsim_command(program: str, run_dir: Path, rate_hz: float) -> list[str]:
    return [program, "--suspend", f"--simulation-rate={rate_hz:.9g}", f"--nice={NICE_SLEEP_S:.8f}",
            f"--root={run_dir}", f"--logdirectivefile={run_dir / OUTPUT_FILE}",
            f"--script={run_dir / SCRIPT_FILE}", "--nohighlight"]
