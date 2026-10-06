import math
from dataclasses import dataclass, field, fields

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTracker
from navpy.modules.vision.gimbal_rate_types import (
    GimbalObservationDisposition,
    GimbalRateTrackerConfig,
    TrackingState,
)
from navpy.modules.vision.gimbal_tracking_sample import (
    GimbalAngularSample,
    GimbalTargetProjector,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.pixel_observation import PixelCalibration
from tests.detection_factory import make_detected_target
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.target_angle_estimator import (
    TargetAngleEstimatorConfig,
)


@dataclass
class _Actuator:
    commands: list[tuple[float, float]] = field(default_factory=list)
    fail_next: bool = False

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        if self.fail_next:
            self.fail_next = False
            raise OSError("actuator unavailable")
        self.commands.append((yaw_rate, pitch_rate))


@dataclass
class _Logger:
    debug_messages: list[str] = field(default_factory=list)
    info_messages: list[str] = field(default_factory=list)
    warning_messages: list[str] = field(default_factory=list)

    def debug(self, message: str) -> None:
        self.debug_messages.append(message)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


def _sample(
    yaw_rad: float = 0.0,
    pitch_rad: float = 0.0,
    timestamp_s: float = 10.0,
) -> GimbalAngularSample:
    return GimbalAngularSample(yaw_rad, pitch_rad, timestamp_s)


def _tracker(
    *,
    actuator: _Actuator | None = None,
    config: GimbalRateTrackerConfig | None = None,
) -> tuple[GimbalRateTracker, _Actuator, _Logger]:
    active_actuator = actuator or _Actuator()
    logger = _Logger()
    active_config = config or GimbalRateTrackerConfig(
        correction_bw=1.0,
        max_rate=100.0,
        max_slew_dps=90.0,
        command_lead_time=0.15,
        estimator=TargetAngleEstimatorConfig(
            measurement_sigma=0.02,
            accel_sigma=1.0,
            initial_angle_sigma=0.08,
            initial_rate_sigma=0.25,
            max_step_dt=0.25,
        ),
    )
    return (
        GimbalRateTracker(active_actuator, logger, active_config),
        active_actuator,
        logger,
    )


def test_centered_sample_tracks_with_zero_rate():
    tracker, actuator, _ = _tracker()

    update = tracker.update(_sample())

    assert update.disposition is GimbalObservationDisposition.ACCEPTED
    assert update.result.state is TrackingState.TRACKING
    assert update.result.yaw_rate == pytest.approx(0.0)
    assert update.result.pitch_rate == pytest.approx(0.0)
    assert actuator.commands == [(0.0, -0.0)]


def test_angular_error_signs_map_to_gimbal_rate_signs():
    tracker, _, _ = _tracker()

    update = tracker.update(_sample(0.2, 0.1))

    assert update.result.yaw_rate is not None
    assert update.result.pitch_rate is not None
    assert update.result.yaw_rate > 0.0
    assert update.result.pitch_rate < 0.0


def test_maturity_requires_two_accepted_samples():
    tracker, _, _ = _tracker()

    first = tracker.update(_sample(timestamp_s=10.0))
    second = tracker.update(_sample(timestamp_s=10.1))

    assert not first.result.mature
    assert second.result.mature


def test_constant_motion_builds_rate_and_lead_projection():
    tracker, _, _ = _tracker()
    tracker.update(_sample(timestamp_s=10.0))

    update = tracker.update(_sample(0.05, -0.02, 10.1))

    result = update.result
    assert result.yaw_rate_estimate is not None
    assert result.pitch_rate_estimate is not None
    assert result.yaw_error is not None
    assert result.pitch_error is not None
    assert result.yaw_rate_estimate > 0.0
    assert result.pitch_rate_estimate < 0.0
    assert result.yaw_error > 0.05
    assert result.pitch_error < -0.02


def test_rate_tracker_configuration_has_no_loss_policy_fields():
    names = {item.name for item in fields(GimbalRateTrackerConfig)}

    assert "coast_timeout" not in names
    assert "hold_resend_interval" not in names


def test_reset_clears_estimator_and_tracker_state():
    tracker, _, _ = _tracker()
    tracker.update(_sample(timestamp_s=10.0))

    tracker.reset()

    assert tracker.state is TrackingState.IDLE
    assert not tracker.is_tracking
    assert tracker.last_result.state is TrackingState.IDLE


@pytest.mark.parametrize("timestamp_s", [10.0, 9.9])
def test_duplicate_or_regressed_sample_does_not_rewind_or_actuate(
    timestamp_s: float,
):
    tracker, actuator, _ = _tracker()
    accepted = tracker.update(_sample(0.1, 0.0, 10.0))
    before = tracker.last_result

    rejected = tracker.update(_sample(0.9, 0.9, timestamp_s))

    assert accepted.disposition is GimbalObservationDisposition.ACCEPTED
    assert rejected.disposition is GimbalObservationDisposition.STALE_NOOP
    assert tracker.last_result == before
    assert len(actuator.commands) == 1


def test_rejected_duplicate_does_not_block_next_new_sample():
    tracker, actuator, _ = _tracker()
    tracker.update(_sample(0.1, 0.0, 10.0))
    tracker.update(_sample(0.9, 0.9, 10.0))

    update = tracker.update(_sample(0.2, 0.0, 10.1))

    assert update.disposition is GimbalObservationDisposition.ACCEPTED
    assert len(actuator.commands) == 2


def test_stale_sample_does_not_run_before_actuation_callback():
    tracker, _, _ = _tracker()
    tracker.update(_sample(0.1, 0.0, 10.0))
    callback_calls: list[bool] = []

    update = tracker.update(
        _sample(0.2, 0.0, 10.0),
        before_actuation=lambda: callback_calls.append(True) or True,
    )

    assert update.disposition is GimbalObservationDisposition.STALE_NOOP
    assert callback_calls == []


def test_actuator_failure_does_not_commit_sample_or_estimator():
    actuator = _Actuator(fail_next=True)
    tracker, _, _ = _tracker(actuator=actuator)
    sample = _sample(0.2, -0.1, 10.0)

    with pytest.raises(OSError, match="actuator unavailable"):
        tracker.update(sample)

    assert tracker.state is TrackingState.IDLE
    assert tracker.last_result.state is TrackingState.IDLE
    retry = tracker.update(sample)
    assert retry.disposition is GimbalObservationDisposition.ACCEPTED


def test_command_is_clamped_to_configured_rate_limit():
    tracker, _, _ = _tracker(
        config=GimbalRateTrackerConfig(
            correction_bw=100.0,
            max_rate=20.0,
            max_slew_dps=90.0,
        )
    )

    result = tracker.update(_sample(1.0, -1.0)).result

    assert result.yaw_rate == pytest.approx(20.0)
    assert result.pitch_rate == pytest.approx(20.0)


def test_nonsettled_subunit_command_is_raised_to_one_unit():
    tracker, _, _ = _tracker(
        config=GimbalRateTrackerConfig(
            correction_bw=0.01,
            max_slew_dps=90.0,
            settle_tolerance_deg=0.1,
        )
    )

    result = tracker.update(_sample(math.radians(1.0))).result

    assert result.yaw_rate == pytest.approx(1.0)


@pytest.mark.parametrize("speedup", [0.5, 1.0, 3.0, 10.0])
def test_rate_navigation_is_invariant_to_wall_clock_speedup(speedup: float):
    del speedup  # Wall cadence belongs to the scheduler, not this rate law.
    tracker, actuator, _ = _tracker()
    samples = [
        _sample(0.00, 0.00, 0.0),
        _sample(0.03, -0.01, 0.1),
        _sample(0.06, -0.02, 0.2),
        _sample(0.09, -0.03, 0.3),
    ]

    for sample in samples:
        tracker.update(sample)

    reference = _navigation_reference_commands()
    assert actuator.commands == pytest.approx(reference)


def _navigation_reference_commands() -> list[tuple[float, float]]:
    tracker, actuator, _ = _tracker()
    for sample in (
        _sample(0.00, 0.00, 0.0),
        _sample(0.03, -0.01, 0.1),
        _sample(0.06, -0.02, 0.2),
        _sample(0.09, -0.03, 0.3),
    ):
        tracker.update(sample)
    return actuator.commands


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("max_slew_dps", 0.0),
        ("max_slew_dps", math.inf),
        ("rate_clamp_slew_multiple", 0.0),
        ("correction_bw", -1.0),
        ("correction_bw", math.nan),
        ("max_rate", -1.0),
        ("max_rate", math.inf),
        ("settle_tolerance_deg", -1.0),
        ("settle_tolerance_deg", math.nan),
        ("command_lead_time", -1.0),
        ("command_lead_time", math.inf),
    ],
)
def test_invalid_rate_law_config_is_rejected(field_name: str, value: float):
    with pytest.raises(ValueError):
        GimbalRateTrackerConfig(**{field_name: value})


def _target(
    x_px: object,
    y_px: object,
    matrix: object,
) -> DetectedObject:
    return make_detected_target(
        obj_id=1,
        x_error=x_px,
        y_error=y_px,
        reference_height_m=2.0,
        k=matrix,
        g_data=GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        uas_att=Attitude(0.0, 0.0, 0.0),
    )


def test_projector_converts_rich_target_once_to_angular_sample():
    matrix = np.array(
        [[1000.0, 0.0, 960.0], [0.0, 500.0, 540.0], [0.0, 0.0, 1.0]]
    )

    sample = GimbalTargetProjector.project(
        _target(1460.0, 290.0, matrix),
        12.5,
    )

    assert sample == GimbalAngularSample(0.5, -0.5, 12.5)


def test_projector_can_use_explicit_principal_point_without_mutating_k():
    matrix = np.array(
        [[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]]
    )
    target = _target(600.0, 400.0, matrix)
    before = matrix.copy()

    sample = GimbalTargetProjector.project(target, 2.0, (500.0, 500.0))

    assert sample == GimbalAngularSample(0.1, -0.1, 2.0)
    np.testing.assert_array_equal(target.optics.camera_matrix(), before)


@pytest.mark.parametrize(
    "matrix",
    [
        None,
        np.zeros((3, 3)),
        np.array([[math.nan, 0.0, 1.0], [0.0, 1.0, 1.0]]),
        [[1.0]],
    ],
)
def test_pixel_calibration_rejects_invalid_camera_geometry(matrix: object):
    with pytest.raises((TypeError, ValueError)):
        PixelCalibration.from_matrix(matrix)


@pytest.mark.parametrize("timestamp_s", [math.nan, math.inf, -math.inf])
def test_projector_rejects_nonfinite_source_timestamp(timestamp_s: float):
    matrix = np.eye(3)
    assert GimbalTargetProjector.project(
        _target(1.0, 1.0, matrix),
        timestamp_s,
    ) is None
