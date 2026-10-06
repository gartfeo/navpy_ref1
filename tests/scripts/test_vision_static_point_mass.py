import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

from navpy.modules.vision.visual_ray_projection import observation_body_ray


SCRIPT_PATH = Path("scripts/vision_static_point_mass.py").resolve()
RUN_PATH = Path("scripts/vision_static_point_mass_run.py").resolve()
spec = importlib.util.spec_from_file_location("vision_static_point_mass", SCRIPT_PATH)
static_pm = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = static_pm
spec.loader.exec_module(static_pm)


def test_static_detection_is_exact_frame_local_pixel_boundary():
    detection = static_pm.build_static_detection(
        position_ned_m=np.asarray([-100.0, 10.0, -20.0]),
        t_s=1.0,
        pitch_deg=-5.0,
        roll_deg=3.0,
        yaw_deg=17.0,
    )
    observation = detection.observation
    assert observation.source_timestamp_s == 1.0
    assert observation.source_name == "static-ideal"
    assert not hasattr(observation, "ownship_yaw_deg")
    assert not hasattr(observation, "air_speed_mps")


def test_static_point_mass_no_wind_is_strictly_well_conditioned():
    miss = static_pm.run_case(static_pm.StaticPointMassCase(
        name="no-wind", wind_speed_mps=0.0, wind_dir_from_deg=0.0,
    ))
    assert miss.passed_target
    assert not miss.timed_out
    assert miss.slant_m < 0.25


def test_crosswind_is_deterministic_bounded_challenge_not_hidden_input():
    case = static_pm.StaticPointMassCase(
        name="hard-crosswind",
        wind_speed_mps=8.0,
        wind_dir_from_deg=120.0,
        max_t_s=140.0,
    )
    first = static_pm.run_case(case)
    second = static_pm.run_case(case)
    assert first == second
    assert first.passed_target
    assert first.slant_m < 10.0


def test_bearing_noise_changes_body_ray_and_is_seed_deterministic():
    clean = static_pm.build_static_detection(
        position_ned_m=np.asarray([-100.0, 0.0, -20.0]),
        t_s=1.0,
        pitch_deg=-5.0,
        roll_deg=3.0,
        yaw_deg=17.0,
    )
    noisy = static_pm.build_static_detection(
        position_ned_m=np.asarray([-100.0, 0.0, -20.0]),
        t_s=1.0,
        pitch_deg=-5.0,
        roll_deg=3.0,
        yaw_deg=17.0,
        bearing_noise_rad=math.radians(0.2),
    )
    assert not np.allclose(
        observation_body_ray(noisy.observation),
        observation_body_ray(clean.observation),
    )

    def run(seed):
        return static_pm.run_case(static_pm.StaticPointMassCase(
            name="noise", wind_speed_mps=0.0, wind_dir_from_deg=0.0,
            bearing_noise_deg=0.13, noise_seed=seed,
        ))
    assert run(7) == run(7)
    assert run(11) != run(7)


def test_point_mass_never_holds_commands_near_closest_approach():
    source = RUN_PATH.read_text(encoding="utf-8")
    assert "norm(position_ned_m)) >=" not in source


def test_bench_law_ignores_the_real_gyro_delay_model(monkeypatch):
    # The bench sensor publishes the instantaneous truth turn rate, so the
    # bench law must always model zero gyro delay - whatever the process env
    # says the REAL filtered gyro's delay is. Same run either way = the
    # undelayed_truth_gyro pin works.
    case = static_pm.StaticPointMassCase(
        name="hard-crosswind",
        wind_speed_mps=8.0,
        wind_dir_from_deg=120.0,
        max_t_s=140.0,
    )
    monkeypatch.delenv("AAS_LAT_GYRO_DELAY_S", raising=False)
    default_env = static_pm.run_case(case)
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "0.03")
    overridden_env = static_pm.run_case(case)
    assert default_env == overridden_env
