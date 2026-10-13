"""JSBSim plant for the HITL bridge (bridge.py), through the jsbsim Python package (--plant package).

The default plant is the JSBSim program (jsbsim_process.py), which reuses the property
names and unit conversions defined here. The servo lag (plant_options.ServoLag) runs here in
Python, once per step; the wind is set with every step. The package is imported only when a JSBSimPlant
is built, so the bridge logic and its tests run without it. The model is a JSBSim aircraft folder <dir>/<Name>/<Name>.xml
with its Engines/ subfolder (the aircraft's models are private data, see tools/tuning/private_data.py),
and it needs an <external_reactions> force named "catapult".

Property choices, checked against the flightgear JSBSim binary (2020.3.18, WSL
/usr/games/JSBSim): the --catalog of the aircraft's model lists every name used here, and
two socket-free runs of that model showed which acceleration output is the
accelerometer reading:
- at rest on the ground forces/fbz-total-lbs / inertia/mass-slugs is minus the local gravity,
  equal to accelerations/a-pilot-z-ft_sec2, while accelerations/wdot-ft_sec2 = 0;
- dropped at zero airspeed forces/fbz-total-lbs = 0 (no specific force in free fall),
  while accelerations/wdot-ft_sec2 is plus the local gravity (kinematic, gravity included).
So the specific force at the CG is the total non-gravity body force over the mass.
accelerations/Nx|Ny|Nz carry the same direction but are divided by that build's own
gravity (slightly off the standard 32.174 ft/s^2); a-pilot-* is at the model's
EYEPOINT, away from the CG, and adds rotational terms.
"""
from __future__ import annotations

import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType
from typing import Optional

from tools.tuning.hitl_bridge.environment import CALM, Wind
from tools.tuning.hitl_bridge.physics import NEUTRAL_COMMAND, Origin, PlantState, SurfaceCommand
from tools.tuning.hitl_bridge.plant_options import PlantOptions, ServoLag, offset_lat_lon, servo_lag_alpha

FT_TO_M = 0.3048
RPM_TO_RAD_S = 2.0 * 3.141592653589793 / 60.0
N_PER_LBF = 4.4482216152605

# Inputs. The model's FCS reads these three (its flight_control: the pitch/roll trim summers
# and the throttle aerosurface_scale); SIM_JSBSim writes the same
# names (SIM_JSBSim.cpp:363-366).
AILERON_CMD = "fcs/aileron-cmd-norm"
ELEVATOR_CMD = "fcs/elevator-cmd-norm"
THROTTLE_CMD = "fcs/throttle-cmd-norm"
CATAPULT_MAGNITUDE_LBS = "external_reactions/catapult/magnitude"    # no <function>: set in lbs
# Velocity of the air mass (FGWinds vWindNED: the WSL probe set wind-north-fps -16.4 and the
# aircraft resting nose north read u-aero-fps +16.3, a headwind)
WIND_NORTH_FPS = "atmosphere/wind-north-fps"
WIND_EAST_FPS = "atmosphere/wind-east-fps"
# ArduPilot's JSBSim start script sets the engine running and then ground-trims
# (SIM_JSBSim.cpp:111-120); the WSL check used the same two steps.
ENGINE_RUNNING = "propulsion/engine[0]/set-running"
GROUND_TRIM = "simulation/do_simple_trim"
GROUND_TRIM_MODE = 2                        # FGTrim tGround: settles altitude, pitch, roll on the contacts
RESET_MODE = 0                              # reset_to_initial_conditions: keep the output files, run the ICs

# Initial guess handed to the ground trim, a few inches up and a little nose-up; the trim settles the
# model on its own contacts from there.
GROUND_TRIM_GUESS_AGL_FT = 0.4
GROUND_TRIM_GUESS_THETA_DEG = 3.0

# Outputs
LAT_DEG = "position/lat-geod-deg"           # geodetic, as GPS reports
LON_DEG = "position/long-gc-deg"            # longitude is the same geocentric or geodetic
ALT_MSL_M = "position/h-sl-meters"          # same reference as ic/terrain-elevation-ft
ROLL_RAD = "attitude/phi-rad"
PITCH_RAD = "attitude/theta-rad"
YAW_RAD = "attitude/psi-rad"
# Body rates relative to the Earth, like ArduPilot's own SITL gyro, which has no Earth
# rate (SIM_Aircraft.cpp:700-713 integrates the attitude from gyro directly).
P_RAD_S = "velocities/p-rad_sec"
Q_RAD_S = "velocities/q-rad_sec"
R_RAD_S = "velocities/r-rad_sec"
VN_FPS = "velocities/v-north-fps"
VE_FPS = "velocities/v-east-fps"
VD_FPS = "velocities/v-down-fps"
# Calibrated airspeed is what a pitot reads; SIM_JSBSim uses the same (fdm.vcas,
# SIM_JSBSim.cpp:442-443).
IAS_FPS = "velocities/vc-fps"
TAS_FPS = "velocities/vtrue-fps"
FORCE_LBS = ("forces/fbx-total-lbs", "forces/fby-total-lbs", "forces/fbz-total-lbs")
MASS_SLUGS = "inertia/mass-slugs"


# The JSBSim program the models were calibrated with (WSL /usr/games/JSBSim, FlightGear 2020.3.18) never uses a
# propeller ixx below this: it reports "IXX = 0.0010" for a file with less, and its spin-up from rest shows 0.00100.
# The jsbsim package (1.3.2 checked) takes the file's value. A model calibrated on the program can carry force
# couples that remove the gyroscopic moment of exactly the program's excess, so with the file's value the package
# would remove a moment it never added. The package plant therefore loads a copy of the model whose propeller ixx is raised to this floor: both
# plants then fly the same physics (the spin-up and the gyroscopic moment of the calibration).
#
# The two versions also differ in the roll reaction of the propeller. The package applies the whole engine torque
# to the airframe (propeller-torque-ftlb = moments/l-prop-lbsft = -engine power / omega), the program only the
# propeller's aerodynamic torque: a probe (full throttle from rest in flight, both at the floor ixx) gave the same
# rpm, power and steady moment, but during the spin-up a large roll moment in the package and almost none in the
# program; the difference is ixx times the propeller's angular acceleration. Physically the
# package is right; the calibration was made without that term. The model copy gets a roll couple
# (SPINUP_COUPLE_FORCES: two body-z forces SPINUP_COUPLE_ARM_IN either side of the CG) that the plant sets to
# + sense ixx d(omega)/dt each step, from the last step's rpm change: the airframe then feels the program's moment.
PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2 = 0.001
SPINUP_COUPLE_FORCES = ("navpy_spinup_right", "navpy_spinup_left")      # y = +arm, -arm
SPINUP_COUPLE_ARM_IN = 12.0
IN_PER_FT = 12.0
PROPELLER_RPM = "propulsion/engine[0]/propeller-rpm"
PROPELLER_SENSE = "propulsion/engine[0]/propeller-sense"
EXTERNAL_REACTIONS_END = "</external_reactions>"
MODEL_END = "</fdm_config>"
PROPELLER_ROOT_TAG = "propeller"
IXX_DEFAULT_UNIT = "SLUG*FT2"               # JSBSim's unit for a propeller's <ixx> without a unit attribute
IXX_ELEMENT = re.compile(r"(<ixx(?P<attrs>[^>]*)>)(?P<value>[^<]*)(</ixx>)")
MODEL_COPY_PREFIX = "navpy-hitl-jsbsim-package-"


def floor_propeller_ixx(text: str) -> tuple[str, list[float]]:
    """A propeller file with its <ixx> raised to PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2; returns the text and the
    values found. A unit other than slug ft2 is refused: the floor is in slug ft2."""
    found = []

    def raise_to_floor(m: re.Match) -> str:
        unit = re.search(r'unit\s*=\s*"([^"]*)"', m.group("attrs"))
        if unit is not None and unit.group(1).upper() != IXX_DEFAULT_UNIT:
            raise ValueError(f"propeller ixx in {unit.group(1)}: the program's floor is in {IXX_DEFAULT_UNIT}")
        value = float(m.group("value"))
        found.append(value)
        if value >= PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2:
            return m.group(0)
        return f"{m.group(1)} {PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2} {m.group(4)}"
    return IXX_ELEMENT.sub(raise_to_floor, text), found


def spinup_couple_xml(cg_x_in: float) -> str:
    """The two body-z forces of the spin-up roll couple, at the CG's x, SPINUP_COUPLE_ARM_IN either side."""
    forces = []
    for name, y_in in zip(SPINUP_COUPLE_FORCES, (SPINUP_COUPLE_ARM_IN, -SPINUP_COUPLE_ARM_IN)):
        forces.append(f'''   <force name="{name}" frame="BODY">
     <location unit="IN"> <x> {cg_x_in!r} </x> <y> {y_in!r} </y> <z> 0.0 </z> </location>
     <direction> <x> 0.0 </x> <y> 0.0 </y> <z> 1.0 </z> </direction>
   </force>
''')
    return "".join(forces)


def prepare_package_model(model_dir: Path, dest_root: Path) -> tuple[Path, float]:
    """A copy of the model folder under dest_root: every propeller's ixx raised to the program's floor and the spin-up
    roll couple added. Returns the copy and the propeller ixx it uses (0 without a propeller: no couple then).
    More than one propeller is refused (the couple follows engine 0)."""
    model_dir = Path(model_dir)
    copy = Path(dest_root) / model_dir.name
    shutil.copytree(model_dir, copy)
    ixx = []
    for path in sorted((copy / "Engines").glob("*.xml")):
        text = path.read_text(encoding="utf-8")
        if ET.fromstring(text).tag != PROPELLER_ROOT_TAG:
            continue
        floored, found = floor_propeller_ixx(text)
        path.write_text(floored, encoding="utf-8")
        ixx += [max(v, PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2) for v in found]
    if len(ixx) > 1:
        raise ValueError(f"{model_dir}: {len(ixx)} propeller ixx values; the spin-up couple handles one propeller")
    if not ixx:
        return copy, 0.0
    aircraft = copy / f"{model_dir.name}.xml"
    text = aircraft.read_text(encoding="utf-8")
    cg = ET.fromstring(text).find("mass_balance/location")
    cg_x_in = float(cg.find("x").text) if cg is not None else 0.0
    couple = spinup_couple_xml(cg_x_in)
    if EXTERNAL_REACTIONS_END in text:
        text = text.replace(EXTERNAL_REACTIONS_END, couple + " " + EXTERNAL_REACTIONS_END, 1)
    else:
        text = text.replace(MODEL_END, f" <external_reactions>\n{couple} </external_reactions>\n{MODEL_END}", 1)
    aircraft.write_text(text, encoding="utf-8")
    return copy, ixx[0]


def import_jsbsim() -> ModuleType:
    try:
        import jsbsim  # noqa: PLC0415 - optional dependency, needed only for a real run
    except ImportError as exc:
        raise RuntimeError("the jsbsim Python package is not installed; the HITL bridge "
                           "needs it for the physics (pip install jsbsim)") from exc
    return jsbsim


class JSBSimPlant:
    """physics.Plant on a JSBSim FGFDMExec with a fixed integration step."""

    def __init__(self, model_dir: Path, origin: Origin, dt_s: float, options: PlantOptions = PlantOptions(),
                 jsbsim_module: Optional[ModuleType] = None, work_root: Optional[Path] = None) -> None:
        model_dir = Path(model_dir)
        if not (model_dir / f"{model_dir.name}.xml").is_file():
            raise FileNotFoundError(f"no JSBSim model {model_dir / (model_dir.name + '.xml')}")
        jsbsim = jsbsim_module if jsbsim_module is not None else import_jsbsim()
        self._copy_root = Path(tempfile.mkdtemp(prefix=MODEL_COPY_PREFIX, dir=work_root))   # removed by close()
        model_dir, self.propeller_ixx = prepare_package_model(model_dir, self._copy_root)
        self.model_dir = model_dir
        self._last_rpm: Optional[float] = None
        self.dt_s = dt_s
        self._servos = ServoLag(servo_lag_alpha(dt_s, options.servo_speed_s))
        self._fdm = jsbsim.FGFDMExec(str(model_dir.parent))
        self._fdm.set_aircraft_path(str(model_dir.parent))
        self._fdm.set_engine_path(str(model_dir / "Engines"))
        if not self._fdm.load_model(model_dir.name):
            raise RuntimeError(f"JSBSim could not load {model_dir}")
        self._fdm.set_dt(dt_s)
        # On the ground the trim settles the pose from a guess; a launcher's pose is set as it is.
        start = options.start
        self._ground_trim = start.on_ground
        agl_ft, theta_deg = ((GROUND_TRIM_GUESS_AGL_FT, GROUND_TRIM_GUESS_THETA_DEG) if self._ground_trim
                             else (start.height_agl_ft, start.pitch_deg))
        lat_deg, lon_deg = offset_lat_lon(origin.lat_deg, origin.lon_deg, start.north_m, start.east_m)
        vn_fps, ve_fps, vd_fps = (v / FT_TO_M for v in start.velocity_ned_m_s)
        # Order matters: terrain before the height above it, attitude before the speeds (ic/vn-fps,
        # ve, vd: the ground velocity, names from the program's --catalog).
        self._ic = (
            ("ic/lat-geod-deg", lat_deg),
            ("ic/long-gc-deg", lon_deg),
            ("ic/terrain-elevation-ft", origin.ground_alt_msl_m / FT_TO_M),
            ("ic/h-agl-ft", agl_ft),
            ("ic/phi-deg", 0.0),
            ("ic/theta-deg", theta_deg),
            ("ic/psi-true-deg", origin.heading_deg),
            ("ic/vn-fps", vn_fps), ("ic/ve-fps", ve_fps), ("ic/vd-fps", vd_fps),
            ("ic/p-rad_sec", 0.0), ("ic/q-rad_sec", 0.0), ("ic/r-rad_sec", 0.0),
        )
        self._ran_ic = False

    def _apply(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        self._fdm[AILERON_CMD] = command.aileron
        self._fdm[ELEVATOR_CMD] = command.elevator
        self._fdm[THROTTLE_CMD] = command.throttle
        self._fdm[CATAPULT_MAGNITUDE_LBS] = catapult_force_n / N_PER_LBF
        self._fdm[WIND_NORTH_FPS], self._fdm[WIND_EAST_FPS] = (v / FT_TO_M for v in wind.air_velocity_ne_m_s())

    def reset(self) -> None:
        for name, value in self._ic:
            self._fdm[name] = value
        if self._ran_ic:
            self._fdm.reset_to_initial_conditions(RESET_MODE)
        elif not self._fdm.run_ic():
            raise RuntimeError("JSBSim run_ic failed")
        self._ran_ic = True
        self._last_rpm = None
        self._fdm[ENGINE_RUNNING] = 1
        self._servos.reset()
        self._apply(NEUTRAL_COMMAND, 0.0, CALM)
        if self._ground_trim:
            self._fdm[GROUND_TRIM] = GROUND_TRIM_MODE

    def _spinup_couple(self) -> None:
        """Remove the roll reaction to the propeller's spin-up that the program does not apply (see
        SPINUP_COUPLE_FORCES), using the angular acceleration of the last step."""
        if not self.propeller_ixx:
            return
        rpm = self._fdm[PROPELLER_RPM]
        omega_dot = 0.0 if self._last_rpm is None else (rpm - self._last_rpm) * RPM_TO_RAD_S / self.dt_s
        self._last_rpm = rpm
        moment_lbs_ft = self._fdm[PROPELLER_SENSE] * self.propeller_ixx * omega_dot
        # a force F along +z at y = +arm and -F at y = -arm make a roll moment of 2 F arm
        force_lbs = moment_lbs_ft / (2.0 * SPINUP_COUPLE_ARM_IN / IN_PER_FT)
        right, left = SPINUP_COUPLE_FORCES
        self._fdm[f"external_reactions/{right}/magnitude"] = force_lbs
        self._fdm[f"external_reactions/{left}/magnitude"] = -force_lbs

    def step(self, command: SurfaceCommand, catapult_force_n: float, wind: Wind) -> None:
        self._apply(self._servos.apply(command), catapult_force_n, wind)
        self._spinup_couple()
        if not self._fdm.run():
            raise RuntimeError("JSBSim run() returned false (simulation terminated)")

    def close(self) -> None:
        """The FDM lives in this process; only the model copy is removed."""
        shutil.rmtree(self._copy_root, ignore_errors=True)

    def state(self) -> PlantState:
        fdm = self._fdm
        mass_slugs = fdm[MASS_SLUGS]
        fx, fy, fz = (fdm[name] / mass_slugs * FT_TO_M for name in FORCE_LBS)
        return PlantState(
            lat_deg=fdm[LAT_DEG], lon_deg=fdm[LON_DEG], alt_msl_m=fdm[ALT_MSL_M],
            roll_rad=fdm[ROLL_RAD], pitch_rad=fdm[PITCH_RAD], yaw_rad=fdm[YAW_RAD],
            p_rad_s=fdm[P_RAD_S], q_rad_s=fdm[Q_RAD_S], r_rad_s=fdm[R_RAD_S],
            vn_m_s=fdm[VN_FPS] * FT_TO_M, ve_m_s=fdm[VE_FPS] * FT_TO_M, vd_m_s=fdm[VD_FPS] * FT_TO_M,
            ias_m_s=fdm[IAS_FPS] * FT_TO_M, tas_m_s=fdm[TAS_FPS] * FT_TO_M,
            fx_m_s2=fx, fy_m_s2=fy, fz_m_s2=fz,
        )
