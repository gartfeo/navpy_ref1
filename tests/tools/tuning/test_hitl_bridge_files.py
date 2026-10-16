"""Run folder of the JSBSim program plant (tools/tuning/hitl_bridge/jsbsim_files.py), without JSBSim."""
import re
from pathlib import Path

import pytest

from tools.tuning.hitl_bridge import jsbsim_files as jf
from tools.tuning.hitl_bridge.physics import Origin
from tools.tuning.hitl_bridge.plant_options import StartPose, launcher_pose, offset_lat_lon, servo_lag_alpha

ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)
DT_1200 = 1.0 / 1200.0
LAG_ALPHA = servo_lag_alpha(DT_1200, 0.14)
# A rest pose on the skids, as a launch profile's ground_rest gives it (synthetic: the real one is private)
ON_SKIDS = StartPose(height_agl_ft=0.5, pitch_deg=-2.0, on_ground=True)


def make_model(tmp_path: Path, name: str = "Peron15") -> Path:
    model = tmp_path / "models" / name
    (model / "Engines").mkdir(parents=True)
    (model / f"{name}.xml").write_text("<fdm_config name='x'>\n <aerodynamics/>\n</fdm_config>\n")
    (model / "Engines" / "motor.xml").write_text("<electric_engine/>")
    return model


def prepared(tmp_path: Path, lag_alpha: float = LAG_ALPHA, start: StartPose = ON_SKIDS) -> Path:
    model = make_model(tmp_path)
    run = tmp_path / "run"
    run.mkdir()
    jf.prepare_run_folder(model, run, ORIGIN, (9621, 9622), DT_1200, lag_alpha, start)
    return run


def test_run_folder_holds_the_model_copy_with_the_system_the_reset_script_and_output(tmp_path):
    run = prepared(tmp_path)
    aircraft = run / "aircraft" / "Peron15"
    model_xml = (aircraft / "Peron15.xml").read_text()
    assert model_xml.count('<system file="navpy_hitl_output"/>') == 1
    assert model_xml.index("navpy_hitl_output") < model_xml.index("</fdm_config>")
    assert (aircraft / "Engines" / "motor.xml").is_file()
    assert "navpy_hitl_output" not in (tmp_path / "models" / "Peron15" / "Peron15.xml").read_text()
    system = (aircraft / "navpy_hitl_output.xml").read_text()
    assert "<value>-35.363261</value>" in system and "<value>149.16523</value>" in system
    assert "<mod> <property>simulation/frame</property> <value>1000000</value> </mod>" in system
    reset = (aircraft / "reset.xml").read_text()
    assert "<altitude unit=\"FT\"> 0.5 </altitude>" in reset and "<theta unit=\"DEG\"> -2.0 </theta>" in reset
    assert "<psi unit=\"DEG\"> 353.0 </psi>" in reset and "<elevation unit=\"M\"> 584.0 </elevation>" in reset
    script = (run / "start.xml").read_text()
    assert '<use aircraft="Peron15" initialize="reset"/>' in script and '<input port="9621"/>' in script
    assert f'dt="{DT_1200!r}"' in script
    assert '<set name="propulsion/engine[0]/set-running" value="1"/>' in script
    assert '<set name="external_reactions/catapult/magnitude" value="0"/>' in script
    assert "do_simple_trim" not in script and "Trim" not in script            # no ground trim event
    output = (run / "output.xml").read_text()
    assert 'type="SOCKET" protocol="UDP" port="9622" rate="1200"' in output and 'name="127.0.0.1"' in output
    assert [line.split()[1] for line in output.splitlines() if "<property>" in line] == list(jf.STATE_PROPERTIES)


def test_on_a_launcher_the_aircraft_starts_at_the_launchers_pitch_and_height_at_rest(tmp_path):
    # Changed: the rest is written as a zero ground velocity (vnorth/veast/vdown) instead of <vt> 0,
    # which the program applies last and which would stop the rail's moving release; at rest both
    # gave the same state in the WSL probe (scratch rail_launcher/ic_probe.py).
    model = make_model(tmp_path)
    run = tmp_path / "run"
    run.mkdir()
    jf.prepare_run_folder(model, run, ORIGIN, (9621, 9622), DT_1200, LAG_ALPHA, launcher_pose(10.0, 1.0))
    reset = (run / "aircraft" / "Peron15" / "reset.xml").read_text()
    assert f'<altitude unit="FT"> {1.0 / 0.3048!r} </altitude>' in reset and '<theta unit="DEG"> 10.0 </theta>' in reset
    assert "<psi unit=\"DEG\"> 353.0 </psi>" in reset and "<vt" not in reset
    assert all(f'<{axis} unit="FT/SEC"> 0.0 </{axis}>' in reset for axis in ("vnorth", "veast", "vdown"))
    assert '<latitude unit="DEG" type="geodetic"> -35.363261 </latitude>' in reset


def test_the_rails_release_starts_the_program_at_the_rail_end_moving_along_it(tmp_path):
    model = make_model(tmp_path)
    run = tmp_path / "run"
    run.mkdir()
    pose = StartPose(height_agl_ft=5.6, pitch_deg=13.4, on_ground=False, north_m=3.0, east_m=-0.4,
                     velocity_ned_m_s=(16.5, -2.0, -3.5))
    jf.prepare_run_folder(model, run, ORIGIN, (9621, 9622), DT_1200, LAG_ALPHA, pose)
    reset = (run / "aircraft" / "Peron15" / "reset.xml").read_text()
    lat, lon = offset_lat_lon(ORIGIN.lat_deg, ORIGIN.lon_deg, 3.0, -0.4)
    assert f'<latitude unit="DEG" type="geodetic"> {lat!r} </latitude>' in reset
    assert f'<longitude unit="DEG"> {lon!r} </longitude>' in reset and lat > ORIGIN.lat_deg and lon < ORIGIN.lon_deg
    for axis, v in (("vnorth", 16.5), ("veast", -2.0), ("vdown", -3.5)):
        (written,) = re.findall(rf'<{axis} unit="FT/SEC"> (\S+) </{axis}>', reset)
        assert float(written) == pytest.approx(v / 0.3048, rel=1e-15)
    assert '<theta unit="DEG"> 13.4 </theta>' in reset and "<vt" not in reset


def test_the_servo_lag_system_lags_the_raw_inputs_into_the_model_inputs_gated_on_time(tmp_path):
    system = (prepared(tmp_path) / "aircraft" / "Peron15" / "navpy_hitl_output.xml").read_text()
    assert jf.LAGGED_COMMAND_PROPERTIES[3:] == jf.DIRECT_COMMAND_PROPERTIES[3:]     # catapult and wind as such
    for raw, model_input in zip(jf.LAGGED_COMMAND_PROPERTIES[:3], jf.DIRECT_COMMAND_PROPERTIES[:3]):
        lagged = raw.replace("-raw", "-lagged")
        assert f'<property value="0"> {raw} </property>' in system      # settable over the socket at once
        assert (f"<sum> <property>{lagged}</property> <product> <property>hitl/servo-gate</property> "
                f"<value>{LAG_ALPHA!r}</value> <difference> <property>{raw}</property> <property>{lagged}"
                f"</property> </difference> </product> </sum>") in system
        assert f"<output>{model_input}</output>" in system
    # the gate compares the time with the last update's; the last time is written after the lags
    assert ("<gt> <property>simulation/sim-time-sec</property> <property>hitl/servo-last-time</property> </gt>"
            in system)
    assert system.index("hitl/throttle-cmd-lagged") < system.index('name="hitl/servo-last-time"')
    assert system.index("navpy HITL servo lag") < system.index("navpy HITL output")
    assert jf.command_properties(LAG_ALPHA) == jf.LAGGED_COMMAND_PROPERTIES


def test_without_servo_lag_the_batch_sets_the_model_inputs_and_no_lag_is_generated(tmp_path):
    system = (prepared(tmp_path, lag_alpha=servo_lag_alpha(DT_1200, 0.0)) / "aircraft" / "Peron15"
              / "navpy_hitl_output.xml").read_text()
    assert jf.SERVO_GATE not in system and "cmd-raw" not in system and "<output>" not in system
    assert jf.command_properties(servo_lag_alpha(DT_1200, 0.0)) == jf.DIRECT_COMMAND_PROPERTIES


def test_a_model_without_one_closing_tag_is_refused():
    with pytest.raises(jf.JSBSimError):
        jf.add_system_to_model("<fdm_config/>")


def test_program_command_line_is_suspended_with_the_named_rate_and_nice_sleep(tmp_path):
    cmd = jf.jsbsim_command("/usr/games/JSBSim", tmp_path, 1200.0)
    assert cmd[:4] == ["/usr/games/JSBSim", "--suspend", "--simulation-rate=1200", "--nice=0.00000001"]
    assert f"--script={tmp_path / 'start.xml'}" in cmd and f"--logdirectivefile={tmp_path / 'output.xml'}" in cmd
    assert f"--root={tmp_path}" in cmd


def test_run_folders_get_fresh_names(tmp_path):
    from datetime import datetime
    now = datetime(2026, 10, 12, 8, 0, 0)
    first, second = jf.new_run_dir(tmp_path, now), jf.new_run_dir(tmp_path, now)
    assert first != second and first.is_dir() and second.is_dir()
