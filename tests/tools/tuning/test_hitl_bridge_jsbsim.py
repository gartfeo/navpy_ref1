"""JSBSim plant adapter (tools/tuning/hitl_bridge/jsbsim_plant.py) against a fake jsbsim module.

The real jsbsim package is not needed (and not installed on the development machine);
these tests pin the property names, the call order and the unit conversions.
"""
import sys
from types import SimpleNamespace

import pytest

from tools.tuning.hitl_bridge.jsbsim_plant import (CATAPULT_MAGNITUDE_LBS, ENGINE_RUNNING, FT_TO_M, GROUND_TRIM,
                                                   GROUND_TRIM_MODE, RESET_MODE, WIND_EAST_FPS, WIND_NORTH_FPS,
                                                   JSBSimPlant, import_jsbsim)
from tools.tuning.hitl_bridge.physics import Origin, SurfaceCommand
from tools.tuning.hitl_bridge.environment import CALM, Wind
from tools.tuning.hitl_bridge.plant_options import (PlantOptions, StartPose, launcher_pose, offset_lat_lon,
                                                    servo_lag_alpha)
from tools.tuning.hitl_bridge.state_fields import hil_state_fields

ORIGIN = Origin(lat_deg=40.2, lon_deg=44.4, ground_alt_msl_m=1300.0, heading_deg=270.0)


class FakeFDM:
    def __init__(self, root_dir: str) -> None:
        self.root_dir = root_dir
        self.props: dict = {}
        self.calls: list = []

    def set_aircraft_path(self, path: str) -> bool:
        self.calls.append(("aircraft_path", path))
        return True

    def set_engine_path(self, path: str) -> bool:
        self.calls.append(("engine_path", path))
        return True

    def load_model(self, name: str) -> bool:
        self.calls.append(("load_model", name))
        return True

    def set_dt(self, dt: float) -> None:
        self.calls.append(("set_dt", dt))

    def run_ic(self) -> bool:
        self.calls.append(("run_ic",))
        return True

    def reset_to_initial_conditions(self, mode: int) -> None:
        self.calls.append(("reset_to_initial_conditions", mode))

    def run(self) -> bool:
        self.calls.append(("run",))
        return True

    def __setitem__(self, name: str, value: float) -> None:
        self.props[name] = value
        self.calls.append(("set", name, value))

    def __getitem__(self, name: str) -> float:
        return self.props[name]


@pytest.fixture
def model_dir(tmp_path):
    d = tmp_path / "aircraft" / "Peron15"
    (d / "Engines").mkdir(parents=True)
    (d / "Peron15.xml").write_text("<fdm_config/>")
    return d


NO_LAG = PlantOptions(servo_speed_s=0.0)


def make_plant(model_dir, dt_s=1.0 / 1200.0, options=NO_LAG):
    return JSBSimPlant(model_dir, ORIGIN, dt_s, options, jsbsim_module=SimpleNamespace(FGFDMExec=FakeFDM),
                       work_root=model_dir.parent.parent)


def test_missing_model_is_reported_before_importing_jsbsim(tmp_path):
    with pytest.raises(FileNotFoundError):
        JSBSimPlant(tmp_path / "Nope", ORIGIN, 0.001, jsbsim_module=None)


def test_import_error_names_the_missing_package(monkeypatch):
    monkeypatch.setitem(sys.modules, "jsbsim", None)        # makes "import jsbsim" fail
    with pytest.raises(RuntimeError, match="jsbsim"):
        import_jsbsim()


def test_model_is_loaded_from_a_copy_of_its_folder_that_close_removes(model_dir):
    # Changed: the package loads a copy (prepare_package_model: the program's propeller ixx floor and the spin-up
    # couple), no longer the folder itself
    plant = make_plant(model_dir, dt_s=0.0025)
    fdm, copy = plant._fdm, plant.model_dir
    assert copy.name == "Peron15" and copy != model_dir and (copy / "Peron15.xml").is_file()
    assert ("aircraft_path", str(copy.parent)) in fdm.calls
    assert ("engine_path", str(copy / "Engines")) in fdm.calls
    assert ("load_model", "Peron15") in fdm.calls
    assert ("set_dt", 0.0025) in fdm.calls
    plant.close()
    assert not copy.exists() and (model_dir / "Peron15.xml").is_file()


def test_reset_puts_the_aircraft_on_the_ground_at_the_origin(model_dir):
    plant = make_plant(model_dir)
    plant.reset()
    fdm = plant._fdm
    assert fdm.props["ic/lat-geod-deg"] == 40.2
    assert fdm.props["ic/long-gc-deg"] == 44.4
    assert fdm.props["ic/terrain-elevation-ft"] == pytest.approx(1300.0 / FT_TO_M)
    assert fdm.props["ic/psi-true-deg"] == 270.0
    assert all(fdm.props[f"ic/{v}"] == 0.0 for v in ("vn-fps", "ve-fps", "vd-fps", "p-rad_sec", "q-rad_sec"))
    names = [c[1] if c[0] == "set" else c[0] for c in fdm.calls]
    # terrain before the height above it; ICs, then run_ic, engine on, then the ground trim last
    assert names.index("ic/terrain-elevation-ft") < names.index("ic/h-agl-ft") < names.index("run_ic")
    assert names.index("run_ic") < names.index(ENGINE_RUNNING) < names.index(GROUND_TRIM)
    assert fdm.props[ENGINE_RUNNING] == 1 and fdm.props[GROUND_TRIM] == GROUND_TRIM_MODE
    assert fdm.props[CATAPULT_MAGNITUDE_LBS] == 0.0
    assert (fdm.props[WIND_NORTH_FPS], fdm.props[WIND_EAST_FPS]) == (0.0, 0.0)     # calm by default
    plant.reset()                                           # a re-init resets every model
    assert ("reset_to_initial_conditions", RESET_MODE) in fdm.calls
    assert names.count("run_ic") == 1


def test_reset_puts_the_aircraft_on_the_launcher_as_given_without_a_ground_trim(model_dir):
    plant = make_plant(model_dir, options=PlantOptions(servo_speed_s=0.0, start=launcher_pose(10.0, 1.0)))
    plant.reset()
    fdm = plant._fdm
    assert fdm.props["ic/h-agl-ft"] == pytest.approx(1.0 / FT_TO_M) and fdm.props["ic/theta-deg"] == 10.0
    assert fdm.props["ic/phi-deg"] == 0.0 and fdm.props["ic/psi-true-deg"] == 270.0
    assert fdm.props[ENGINE_RUNNING] == 1 and GROUND_TRIM not in fdm.props     # held at rest, not settled


def test_reset_puts_the_aircraft_at_the_rails_release_moving_along_the_rail(model_dir):
    pose = StartPose(height_agl_ft=5.6, pitch_deg=13.4, on_ground=False, north_m=3.0, east_m=-0.4,
                     velocity_ned_m_s=(16.5, -2.0, -3.5))
    plant = make_plant(model_dir, options=PlantOptions(servo_speed_s=0.0, start=pose))
    plant.reset()
    fdm = plant._fdm
    assert (fdm.props["ic/lat-geod-deg"], fdm.props["ic/long-gc-deg"]) == offset_lat_lon(40.2, 44.4, 3.0, -0.4)
    assert (fdm.props["ic/vn-fps"], fdm.props["ic/ve-fps"], fdm.props["ic/vd-fps"]) == pytest.approx(
        (16.5 / FT_TO_M, -2.0 / FT_TO_M, -3.5 / FT_TO_M))
    names = [c[1] for c in fdm.calls if c[0] == "set"]
    assert names.index("ic/theta-deg") < names.index("ic/vn-fps")       # attitude before the speeds
    assert fdm.props["ic/h-agl-ft"] == 5.6 and GROUND_TRIM not in fdm.props


def test_step_writes_the_commands_then_runs_once(model_dir):
    plant = make_plant(model_dir)
    plant.step(SurfaceCommand(aileron=0.25, elevator=-0.5, throttle=0.75), catapult_force_n=350.0, wind=CALM)
    fdm = plant._fdm
    assert fdm.props["fcs/aileron-cmd-norm"] == 0.25
    assert fdm.props["fcs/elevator-cmd-norm"] == -0.5
    assert fdm.props["fcs/throttle-cmd-norm"] == 0.75
    assert fdm.props[CATAPULT_MAGNITUDE_LBS] == pytest.approx(78.68, abs=0.01)   # 350 N in lbf
    assert fdm.calls[-1] == ("run",) and fdm.calls.count(("run",)) == 1


def test_steps_go_through_the_servo_lag_which_a_reset_returns_to_neutral(model_dir):
    dt = 1.0 / 1200.0
    plant = make_plant(model_dir, options=PlantOptions(servo_speed_s=0.14))
    a = servo_lag_alpha(dt, 0.14)
    plant.reset()
    command = SurfaceCommand(aileron=0.25, elevator=-0.5, throttle=1.0)
    plant.step(command, 0.0, CALM)
    plant.step(command, 0.0, CALM)
    fdm = plant._fdm
    reached = 1 - (1 - a) ** 2
    assert (fdm.props["fcs/aileron-cmd-norm"], fdm.props["fcs/elevator-cmd-norm"],
            fdm.props["fcs/throttle-cmd-norm"]) == pytest.approx((0.25 * reached, -0.5 * reached, reached))
    plant.reset()
    plant.step(command, 0.0, CALM)
    assert fdm.props["fcs/throttle-cmd-norm"] == pytest.approx(a)


def test_each_step_sets_the_wind_as_the_velocity_of_the_air(model_dir):
    plant = make_plant(model_dir)
    plant.reset()
    plant.step(SurfaceCommand(0.0, 0.0, 0.0), 0.0, Wind(speed_m_s=3.048, from_deg=90.0))
    assert (plant._fdm.props[WIND_NORTH_FPS], plant._fdm.props[WIND_EAST_FPS]) == pytest.approx((0.0, -10.0))


def test_state_units_and_specific_force(model_dir):
    plant = make_plant(model_dir)
    # an aircraft sitting nose-down on the ground (synthetic: 0.4 slug, 0.05 rad, JSBSim's local g 32.01 ft/s^2)
    plant._fdm.props.update({
        "position/lat-geod-deg": -35.363261, "position/long-gc-deg": 149.16523, "position/h-sl-meters": 584.1,
        "attitude/phi-rad": 0.0, "attitude/theta-rad": -0.05, "attitude/psi-rad": 6.1,
        "velocities/p-rad_sec": 0.01, "velocities/q-rad_sec": -0.02, "velocities/r-rad_sec": 0.03,
        "velocities/v-north-fps": 100.0, "velocities/v-east-fps": -10.0, "velocities/v-down-fps": 1.0,
        "velocities/vc-fps": 72.1785, "velocities/vtrue-fps": 82.0,
        "forces/fbx-total-lbs": -0.6399, "forces/fby-total-lbs": 0.0, "forces/fbz-total-lbs": -12.788,
        "inertia/mass-slugs": 0.4,
    })
    s = plant.state()
    assert (s.vn_m_s, s.ve_m_s, s.vd_m_s) == pytest.approx((30.48, -3.048, 0.3048))
    assert s.ias_m_s == pytest.approx(22.0, abs=0.001)
    assert s.tas_m_s == pytest.approx(82.0 * 0.3048)
    assert s.fz_m_s2 == pytest.approx(-12.788 / 0.4 * 0.3048)                # -9.744 m/s^2
    assert s.fx_m_s2 == pytest.approx(-0.6399 / 0.4 * 0.3048)
    f = hil_state_fields(s)
    assert (f["xacc"], f["zacc"]) == (-50, -994)          # nose-down 2.9 deg at rest, local g < 1 g0
