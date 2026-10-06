"""Finite camera/gimbal/airframe certification through production pixels."""

from tests.modules.navigation.nav.vision_nav_camera_loop import (
    run_siyi_camera_loop,
)


def test_siyi_finite_fov_log_153922_closed_loop_hits_target() -> None:
    result = run_siyi_camera_loop()
    assert result.miss.passed_target
    assert not result.miss.timed_out
    assert result.miss.slant_m < 0.5
    assert result.first_fov_loss_m is None or result.first_fov_loss_m < 0.5
    assert result.observation_count > 0


def test_camera_loop_law_ignores_the_real_gyro_delay_model(monkeypatch) -> None:
    # The loop feeds the law the instantaneous coordinated-turn rate, so the
    # law inside it must model zero gyro delay no matter what the process env
    # says about the REAL filtered gyro. Same run either way = the pin works.
    monkeypatch.delenv("AAS_LAT_GYRO_DELAY_S", raising=False)
    default_env = run_siyi_camera_loop()
    monkeypatch.setenv("AAS_LAT_GYRO_DELAY_S", "0.03")
    overridden_env = run_siyi_camera_loop()
    assert default_env.miss == overridden_env.miss
