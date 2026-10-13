"""The owner's private HITL data (tools/tuning/private_data.py) against its own measurements.

The aircraft's launch profile and parameter files are kept out of this public repository. Where they are on the
machine (REAL_PRIVATE_DIR, read before the tests point the private folder at synthetic data), these tests check
them: the rail against the measured ranges stored next to it (the profile's "checks"), the launch detection, and
the parameter files' contents. Elsewhere they skip.
"""
from __future__ import annotations

import math

import pytest

from tests.tools.tuning.hitl_data import REAL_PRIVATE_DIR
from tools.tuning.hitl_bridge.environment import CALM, US_PER_S
from tools.tuning.hitl_bridge.launch_profile import ground_rest, rail_spec, read_launch_data
from tools.tuning.hitl_bridge.physics import Origin, PlantState, unmix_elevon
from tools.tuning.hitl_bridge.plant_options import FT_PER_M
from tools.tuning.hitl_bridge.rail import RailGeometry, RailLaunch, spec_stroke
from tools.tuning.hitl_bridge.state_fields import STANDARD_GRAVITY_M_S2
from tools.tuning.hitl_params import read_param_file
from tools.tuning.private_data import (AIRCRAFT_PARAMS_NAME, HITL_DATA_DIR_NAME, LAUNCH_PARAMS_NAME,
                                       LAUNCH_PROFILE_NAME)

HITL_DIR = REAL_PRIVATE_DIR / HITL_DATA_DIR_NAME
LAUNCH_FILE = HITL_DIR / LAUNCH_PROFILE_NAME
SERVO_OPTION = HITL_DIR / "hitl_board_servos.parm"
G = STANDARD_GRAVITY_M_S2
ORIGIN = Origin(lat_deg=-35.363261, lon_deg=149.16523, ground_alt_msl_m=584.0, heading_deg=353.0)
LOOP_US = 5000                      # 200 Hz board frames
CHECKS_KEY = "checks"

pytestmark = pytest.mark.skipif(not LAUNCH_FILE.is_file(), reason=f"no private HITL data in {HITL_DIR}")


def _profile():
    data = read_launch_data(LAUNCH_FILE)
    return data, rail_spec(data), data[CHECKS_KEY]


def test_the_owners_rail_releases_within_the_measured_stroke_time_mean_push_and_table_speed():
    _, spec, checks = _profile()
    stroke = spec_stroke(spec)
    assert stroke.at(stroke.duration_s) == (spec.travel_m, spec.release_speed_m_s, 0.0)
    assert checks["stroke_s"][0] <= stroke.duration_s <= checks["stroke_s"][1]
    mean_g = spec.release_speed_m_s / stroke.duration_s / G
    assert checks["mean_accel_g"][0] <= mean_g <= checks["mean_accel_g"][1]
    table_release_speed = spec.release_speed_m_s * stroke.bin_s / spec.accel_table_bin_s      # before the scaling
    assert abs(table_release_speed / spec.release_speed_m_s - 1.0) <= checks["table_speed_agreement"]


def test_the_owners_rail_releases_within_the_measured_rise_and_the_board_detects_it_in_the_measured_time():
    _, spec, checks = _profile()
    launch = RailLaunch(RailGeometry(ORIGIN, spec.elevation_deg, spec.height_m, spec.carriage_pitch_deg),
                        spec_stroke(spec), 0.0)
    pose = launch.release_pose()
    rise_m = pose.height_agl_ft / FT_PER_M - spec.height_m
    assert checks["rise_at_release_m"][0] <= rise_m <= checks["rise_at_release_m"][1]
    vn, ve, vd = pose.velocity_ned_m_s
    tas = math.sqrt(vn * vn + ve * ve + vd * vd)
    release = PlantState(lat_deg=ORIGIN.lat_deg, lon_deg=ORIGIN.lon_deg, alt_msl_m=ORIGIN.ground_alt_msl_m,
                         roll_rad=0.0, pitch_rad=math.radians(pose.pitch_deg), yaw_rad=math.radians(ORIGIN.heading_deg),
                         p_rad_s=0.0, q_rad_s=0.0, r_rad_s=0.0, vn_m_s=vn, ve_m_s=ve, vd_m_s=vd,
                         ias_m_s=tas * checks["probe_eas_per_tas"], tas_m_s=tas, fx_m_s2=0.0, fy_m_s2=0.0, fz_m_s2=0.0)
    launch.note(0, armed=True)
    times_s = [k * LOOP_US / US_PER_S for k in range(20)]
    vxdot = [s.fx_m_s2 - G * math.sin(s.pitch_rad)
             for s in (launch.held_state(release, t * US_PER_S, CALM) for t in times_s)]
    first = next(t for t, x in zip(times_s, vxdot) if x >= checks["tkoff_thr_minacc"])
    assert checks["detect_s"][0] <= first <= checks["detect_s"][1]
    launch_params = read_param_file(HITL_DIR / LAUNCH_PARAMS_NAME)
    assert launch_params["TKOFF_THR_MINACC"] == checks["tkoff_thr_minacc"]


def test_the_owners_ground_rest_is_on_the_skids():
    data, _, _ = _profile()
    rest = ground_rest(data)
    assert rest.on_ground and 0.0 < rest.height_agl_ft < 1.0          # centimetres above the ground


def test_the_owners_hitl_parameter_files_hold_the_airspeeds_takeoff_and_launch_in_one_place_each():
    aircraft = read_param_file(HITL_DIR / AIRCRAFT_PARAMS_NAME)
    launch = read_param_file(HITL_DIR / LAUNCH_PARAMS_NAME)
    assert {"AIRSPEED_MIN", "AIRSPEED_CRUISE", "AIRSPEED_MAX", "TKOFF_LVL_PITCH", "TKOFF_THR_MAX",
            "TKOFF_THR_SLEW"} == set(aircraft)
    assert aircraft["AIRSPEED_MIN"] < aircraft["AIRSPEED_CRUISE"] < aircraft["AIRSPEED_MAX"]
    assert set(launch) == {"TKOFF_THR_MINACC", "TKOFF_THR_DELAY"} and not set(launch) & set(aircraft)


@pytest.mark.skipif(not SERVO_OPTION.is_file(), reason="no private servo option file")
def test_the_aircraft_servo_option_documents_the_unmix_gap_of_its_trims():
    """hitl_board_servos.parm: the aircraft's own elevons (opt-in with --params-file). The bridge's unmix is per
    microsecond around 1500 with unreversed servos, so trims off 1500 read as a standing command on the model:
    the gap its comment names, with the numbers the unmix gives for its trims."""
    servos = read_param_file(SERVO_OPTION)
    assert (servos["SERVO1_FUNCTION"], servos["SERVO2_FUNCTION"]) == (77, 78)
    neutral = unmix_elevon([servos["SERVO1_TRIM"], servos["SERVO2_TRIM"], 1000] + [0] * 13)
    text = SERVO_OPTION.read_text(encoding="utf-8")
    assert "GAP" in text and f"{neutral.aileron:+.2f}" in text and f"({neutral.elevator:+.2f})" in text
