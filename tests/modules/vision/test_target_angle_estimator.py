import math

import pytest

from navpy.modules.vision.target_angle_estimator import (
    TargetAngleEstimator,
    TargetAngleEstimatorConfig,
    project_estimate,
)


def _commit_update(
    estimator: TargetAngleEstimator,
    yaw_rad: float,
    pitch_rad: float,
    timestamp_s: float,
    *,
    reset: bool = False,
):
    plan = estimator.preview_update(
        yaw_rad,
        pitch_rad,
        timestamp_s,
        reset=reset,
    )
    assert plan is not None
    estimator.commit(plan)
    return plan.estimate


def _settle(
    estimator: TargetAngleEstimator,
    yaw_rad: float,
    pitch_rad: float,
    start_s: float,
    count: int = 25,
    step_s: float = 0.04,
) -> float:
    timestamp_s = start_s
    for _ in range(count):
        _commit_update(estimator, yaw_rad, pitch_rad, timestamp_s)
        timestamp_s += step_s
    return timestamp_s


def test_preview_is_transactional_until_committed():
    estimator = TargetAngleEstimator()

    plan = estimator.preview_update(0.12, -0.08, 5.0)

    assert plan is not None
    assert estimator.snapshot is None
    estimator.commit(plan)
    assert estimator.snapshot == plan.estimate


def test_first_commit_initializes_zero_rate_state():
    estimator = TargetAngleEstimator()

    estimate = _commit_update(estimator, 0.12, -0.08, 5.0)

    assert estimate.yaw_rad == pytest.approx(0.12)
    assert estimate.pitch_rad == pytest.approx(-0.08)
    assert estimate.yaw_rate_rad_s == pytest.approx(0.0)
    assert estimate.pitch_rate_rad_s == pytest.approx(0.0)


def test_duplicate_and_regressed_samples_do_not_change_anchor():
    estimator = TargetAngleEstimator()
    committed = _commit_update(estimator, 0.1, -0.1, 5.0)

    assert estimator.preview_update(0.5, 0.5, 5.0) is None
    assert estimator.preview_update(0.5, 0.5, 4.9) is None

    assert estimator.snapshot == committed


def test_predict_preview_is_transactional_and_increases_uncertainty():
    estimator = TargetAngleEstimator()
    before = _commit_update(estimator, 0.0, 0.0, 0.0)

    plan = estimator.preview_predict(0.5)

    assert plan is not None
    assert estimator.snapshot == before
    assert plan.estimate.yaw_sigma_rad > before.yaw_sigma_rad
    assert plan.estimate.pitch_sigma_rad > before.pitch_sigma_rad


def test_predict_rejects_duplicate_or_regressed_time():
    estimator = TargetAngleEstimator()
    _commit_update(estimator, 0.0, 0.0, 1.0)

    assert estimator.preview_predict(1.0) is None
    assert estimator.preview_predict(0.0) is None


def test_linear_motion_estimates_velocity_and_projects_forward():
    estimator = TargetAngleEstimator(
        TargetAngleEstimatorConfig(measurement_sigma=0.01, accel_sigma=0.5)
    )
    _commit_update(estimator, 0.00, 0.00, 0.0)
    _commit_update(estimator, 0.05, -0.02, 0.1)
    estimate = _commit_update(estimator, 0.10, -0.04, 0.2)

    projected_yaw, projected_pitch = project_estimate(estimate, 0.1)

    assert estimate.yaw_rate_rad_s > 0.0
    assert estimate.pitch_rate_rad_s < 0.0
    assert projected_yaw > estimate.yaw_rad
    assert projected_pitch < estimate.pitch_rad


def test_negative_projection_lead_is_clamped_to_zero():
    estimator = TargetAngleEstimator()
    _commit_update(estimator, 0.0, 0.0, 0.0)
    estimate = _commit_update(estimator, 0.2, -0.1, 0.1)

    projected = project_estimate(estimate, -1.0)

    assert projected == pytest.approx((estimate.yaw_rad, estimate.pitch_rad))


def test_robust_knee_downweights_single_outlier():
    estimator = TargetAngleEstimator(TargetAngleEstimatorConfig())
    timestamp_s = _settle(estimator, 0.0, 0.0, 0.0)

    estimate = _commit_update(estimator, 1.0, -1.0, timestamp_s)

    assert abs(estimate.yaw_rad) < 0.1
    assert abs(estimate.pitch_rad) < 0.1
    assert abs(estimate.yaw_rate_rad_s) < 0.5
    assert abs(estimate.pitch_rate_rad_s) < 0.5


def test_disabling_robust_knee_accepts_same_outlier_at_full_gain():
    estimator = TargetAngleEstimator(
        TargetAngleEstimatorConfig(robust_nis_knee=0.0)
    )
    timestamp_s = _settle(estimator, 0.0, 0.0, 0.0)

    estimate = _commit_update(estimator, 1.0, -1.0, timestamp_s)

    assert abs(estimate.yaw_rad) > 0.3


def test_sustained_shift_converges_after_robust_downweighting():
    estimator = TargetAngleEstimator(TargetAngleEstimatorConfig())
    timestamp_s = _settle(estimator, 0.0, 0.0, 0.0)

    _settle(estimator, 0.30, -0.20, timestamp_s, count=30)
    estimate = estimator.snapshot

    assert estimate is not None
    assert estimate.yaw_rad == pytest.approx(0.30, abs=0.03)
    assert estimate.pitch_rad == pytest.approx(-0.20, abs=0.03)


def test_velocity_state_is_clamped_after_update():
    estimator = TargetAngleEstimator(
        TargetAngleEstimatorConfig(
            max_abs_rate=1.5,
            robust_nis_knee=0.0,
            initial_rate_sigma=100.0,
        )
    )
    _commit_update(estimator, 0.0, 0.0, 0.0)

    estimate = _commit_update(estimator, 10.0, -10.0, 0.01)

    assert abs(estimate.yaw_rate_rad_s) <= 1.5
    assert abs(estimate.pitch_rate_rad_s) <= 1.5


@pytest.mark.parametrize("value", [-1.0, math.inf, -math.inf, math.nan])
def test_invalid_max_abs_rate_is_rejected(value: float):
    with pytest.raises(ValueError, match="max_abs_rate"):
        TargetAngleEstimatorConfig(max_abs_rate=value)


@pytest.mark.parametrize("value", [0.0, -1.0, math.inf, math.nan])
def test_invalid_max_step_is_rejected(value: float):
    with pytest.raises(ValueError, match="max_step_dt"):
        TargetAngleEstimatorConfig(max_step_dt=value)


def test_reset_clears_committed_state():
    estimator = TargetAngleEstimator()
    _commit_update(estimator, 0.1, 0.2, 1.0)

    estimator.reset()

    assert estimator.snapshot is None
