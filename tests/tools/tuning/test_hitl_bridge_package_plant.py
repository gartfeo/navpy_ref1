"""The package plant (tools/tuning/hitl_bridge/jsbsim_plant.py) made to fly the JSBSim program's physics.

The models were calibrated on the WSL JSBSim program (FlightGear 2020.3.18); the bridge on Windows runs the jsbsim
package (1.3.2). Two differences were found and are handled in the model copy the package loads:
- the program never uses a propeller ixx below 0.001 slug ft2, the package takes the file's value;
- the package applies the whole engine torque to the airframe during a propeller spin-up, the program only the
  propeller's aerodynamic torque.
The unit tests use synthetic files. The integration tests load the owner's model and launch profile from the private
folder (tools/tuning/private_data.py, kept outside this public repo) and skip where they or the jsbsim package are
absent.
"""
import contextlib
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest


from tests.tools.tuning.hitl_data import REAL_PRIVATE_DIR
from tools.tuning.hitl_bridge.environment import CALM
from tools.tuning.hitl_bridge.jsbsim_plant import (PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2, SPINUP_COUPLE_ARM_IN,
                                                   SPINUP_COUPLE_FORCES, JSBSimPlant, floor_propeller_ixx,
                                                   prepare_package_model)
from tools.tuning.hitl_bridge.physics import Origin, SurfaceCommand
from tools.tuning.hitl_bridge.launch_profile import rail_spec, read_launch_data
from tools.tuning.hitl_bridge.plant_options import PlantOptions
from tools.tuning.hitl_bridge.rail import RailGeometry, RailLaunch, spec_stroke
from tools.tuning.private_data import HITL_DATA_DIR_NAME, LAUNCH_PROFILE_NAME

PROPELLER = '<?xml version="1.0"?>\n<!-- a comment -->\n<propeller name="P">\n  <ixx> {ixx} </ixx>\n</propeller>\n'
AIRCRAFT = ('<?xml version="1.0"?>\n<fdm_config name="X">\n <mass_balance>\n  <location name="CG" unit="IN">'
            ' <x> 20.0 </x> <y> 0 </y> <z> 0 </z> </location>\n </mass_balance>\n{reactions}</fdm_config>\n')
REACTIONS = ' <external_reactions>\n  <force name="catapult" frame="BODY"/>\n </external_reactions>\n'


def _model(root: Path, ixx: float = 0.0002, reactions: str = REACTIONS, engines: dict | None = None) -> Path:
    model = root / "src" / "M"
    (model / "Engines").mkdir(parents=True)
    (model / "M.xml").write_text(AIRCRAFT.format(reactions=reactions))
    for name, text in (engines or {"prop.xml": PROPELLER.format(ixx=ixx), "motor.xml": "<electric_engine/>"}).items():
        (model / "Engines" / name).write_text(text)
    return model


def test_a_propeller_ixx_below_the_programs_floor_is_raised_and_one_above_it_kept():
    text, found = floor_propeller_ixx(PROPELLER.format(ixx=0.0002))
    assert found == [0.0002] and f"<ixx> {PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2} </ixx>" in text
    text, found = floor_propeller_ixx(PROPELLER.format(ixx=0.0012))
    assert found == [0.0012] and text == PROPELLER.format(ixx=0.0012)
    assert floor_propeller_ixx('<ixx unit="SLUG*FT2"> 0.0005 </ixx>')[1] == [0.0005]
    with pytest.raises(ValueError, match="KG\\*M2"):
        floor_propeller_ixx('<ixx unit="KG*M2"> 0.0005 </ixx>')


def test_the_copy_has_the_floor_and_the_spin_up_couple_and_the_source_is_untouched(tmp_path):
    model = _model(tmp_path)
    copy, ixx = prepare_package_model(model, tmp_path / "copy")
    assert ixx == PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2
    assert "0.0002" in (model / "Engines" / "prop.xml").read_text()            # the source keeps its value
    assert (copy / "Engines" / "motor.xml").read_text() == "<electric_engine/>"
    forces = {f.get("name"): f for f in ET.parse(copy / "M.xml").getroot().iter("force")}
    assert set(forces) == {"catapult", *SPINUP_COUPLE_FORCES}
    right, left = (forces[name] for name in SPINUP_COUPLE_FORCES)
    assert [float(right.find(f"location/{a}").text) for a in "xyz"] == [20.0, SPINUP_COUPLE_ARM_IN, 0.0]
    assert [float(left.find(f"location/{a}").text) for a in "xyz"] == [20.0, -SPINUP_COUPLE_ARM_IN, 0.0]
    assert [float(right.find(f"direction/{a}").text) for a in "xyz"] == [0.0, 0.0, 1.0]


def test_a_model_without_external_reactions_gets_them_and_one_without_a_propeller_gets_no_couple(tmp_path):
    copy, _ = prepare_package_model(_model(tmp_path / "a", reactions=""), tmp_path / "a" / "copy")
    names = [f.get("name") for f in ET.parse(copy / "M.xml").getroot().iter("force")]
    assert names == list(SPINUP_COUPLE_FORCES)
    copy, ixx = prepare_package_model(_model(tmp_path / "b", engines={"m.xml": "<electric_engine/>"}),
                                      tmp_path / "b" / "copy")
    assert ixx == 0.0 and "navpy_spinup" not in (copy / "M.xml").read_text()
    two = {"a.xml": PROPELLER.format(ixx=0.001), "b.xml": PROPELLER.format(ixx=0.002)}
    with pytest.raises(ValueError, match="one propeller"):
        prepare_package_model(_model(tmp_path / "c", engines=two), tmp_path / "c" / "copy")


class _SpinFDM(dict):
    """Enough of FGFDMExec for the couple: the rpm follows a script, every set is kept."""

    def __init__(self, rpms: list[float], sense: float = 1.0) -> None:
        super().__init__({"propulsion/engine[0]/propeller-sense": sense})
        self.rpms = rpms

    def run(self) -> bool:
        self["propulsion/engine[0]/propeller-rpm"] = self.rpms.pop(0)
        return True


def test_the_couple_is_sense_ixx_times_the_last_steps_angular_acceleration(tmp_path):
    plant = JSBSimPlant.__new__(JSBSimPlant)
    plant.dt_s, plant.propeller_ixx, plant._last_rpm = 0.001, 0.001, None
    plant._servos = SimpleNamespace(apply=lambda c: c)
    plant._apply = lambda *args: None
    plant._fdm = _SpinFDM([600.0, 1200.0, 1800.0])
    plant._fdm["propulsion/engine[0]/propeller-rpm"] = 0.0
    right, left = (f"external_reactions/{n}/magnitude" for n in SPINUP_COUPLE_FORCES)
    plant.step(SurfaceCommand(0, 0, 1), 0.0, CALM)               # no previous rpm: no couple
    assert (plant._fdm[right], plant._fdm[left]) == (0.0, -0.0)
    plant.step(SurfaceCommand(0, 0, 1), 0.0, CALM)               # 0 -> 600 rpm in 1 ms
    moment = 0.001 * (600.0 * 2 * math.pi / 60) / 0.001          # ixx omega_dot, lbs ft
    assert plant._fdm[right] == pytest.approx(moment / 2) and plant._fdm[left] == pytest.approx(-moment / 2)
    plant._fdm["propulsion/engine[0]/propeller-sense"] = -1.0
    plant.step(SurfaceCommand(0, 0, 1), 0.0, CALM)
    assert plant._fdm[right] == pytest.approx(-moment / 2)


# ---- with the jsbsim package and the owner's model and launch profile ------------------------------------------
MODEL_DIR = Path(os.environ.get("NAVPY_HITL_MODEL_DIR", REAL_PRIVATE_DIR / "models" / "Peron15_hitl3")).expanduser()
LAUNCH_FILE = REAL_PRIVATE_DIR / HITL_DATA_DIR_NAME / LAUNCH_PROFILE_NAME
needs_model = pytest.mark.skipif(
    not (MODEL_DIR / f"{MODEL_DIR.name}.xml").is_file() or not LAUNCH_FILE.is_file(),
    reason=f"the owner's model {MODEL_DIR} or launch profile {LAUNCH_FILE} is not on this machine "
           "(set NAVPY_HITL_MODEL_DIR)")
ORIGIN = Origin(-35.363261, 149.16523, 584.0, 353.0)
DT = 1.0 / 1200.0
FULL = SurfaceCommand(aileron=0.0, elevator=0.2, throttle=1.0)


def _owners_rail(delay_s: float) -> RailLaunch:
    spec = rail_spec(read_launch_data(LAUNCH_FILE))
    geometry = RailGeometry(ORIGIN, spec.elevation_deg, spec.height_m, spec.carriage_pitch_deg)
    return RailLaunch(geometry, spec_stroke(spec), delay_s)


def _release_plant(tmp_path: Path) -> JSBSimPlant:
    pytest.importorskip("jsbsim")
    start = _owners_rail(3.0).release_pose()
    plant = JSBSimPlant(MODEL_DIR, ORIGIN, DT, PlantOptions(servo_speed_s=0.03, start=start), work_root=tmp_path)
    plant.reset()
    return plant


def _fly(plant: JSBSimPlant, seconds: float) -> list:
    states = []
    for k in range(round(seconds / DT)):
        plant.step(FULL, 0.0, CALM)
        if (k + 1) % 6 == 0:
            states.append(plant.state())
    return states


@needs_model
def test_the_owners_model_loads_with_the_floor_and_flies_bit_identically_twice(tmp_path):
    first, second = _release_plant(tmp_path), _release_plant(tmp_path)
    assert first.propeller_ixx == PROGRAM_PROPELLER_IXX_FLOOR_SLUGFT2
    assert _fly(first, 2.0) == _fly(second, 2.0)
    first.close()
    second.close()


@needs_model
def test_a_full_throttle_spin_up_does_not_roll_the_airframe_as_in_the_program(tmp_path, monkeypatch):
    # WSL program, same release and commands: almost no roll rate over the first 0.1 s of the spin-up.
    # Without the couple the package rolls the airframe fast in that time.
    plant = _release_plant(tmp_path)
    peak = max(abs(s.p_rad_s) for s in _fly(plant, 0.1))
    plant.close()
    assert peak < 0.05
    uncorrected = _release_plant(tmp_path)
    uncorrected.propeller_ixx = 0.0                     # the couple stays at 0
    assert max(abs(s.p_rad_s) for s in _fly(uncorrected, 0.1)) > 0.5
    uncorrected.close()


@needs_model
def test_board_frames_replayed_through_the_bridge_give_bit_identical_replies_without_the_pc_clock(tmp_path):
    from tests.tools.tuning.test_hitl_bridge_v2 import no_pc_clock, recorded_session, run
    frames = recorded_session()

    def replay(guard):
        rail = _owners_rail(0.5)
        plant = JSBSimPlant(MODEL_DIR, ORIGIN, DT, PlantOptions(servo_speed_s=0.03, start=rail.release_pose()),
                            work_root=tmp_path)
        try:
            return run(list(frames), plant, rail, CALM, guard)[0]
        finally:
            plant.close()
    first = replay(no_pc_clock)
    assert first == replay(no_pc_clock) == replay(contextlib.nullcontext)
