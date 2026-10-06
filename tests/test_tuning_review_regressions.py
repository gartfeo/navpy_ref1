"""Independent-review regressions for the SIYI tuning tool."""

from __future__ import annotations

import ast
from types import SimpleNamespace

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.gimbal_rate_types import (
    GimbalObservationDisposition,
    GimbalRateTrackerConfig,
    GimbalRateUpdate,
    GimbalTrackResult,
    TrackingState,
    idle_result,
)
from tests.detection_factory import make_detected_target
from tools.cam import gimbal_controller
from tools.cam import gimbal_tuning_assembly as tuning_assembly
from tools.cam import gimbal_tuning_sample as tuning_sample
from tools.cam.gimbal_tuning_tracking import TuningTracker


class _SilentLog:
    def debug(self, _message: str, *args: object) -> None:
        return None

    def info(self, _message: str, *args: object) -> None:
        return None

    def warning(self, _message: str, *args: object) -> None:
        return None


class _TrackingPort:
    def __init__(self) -> None:
        self._result = idle_result()
        self.loss_count = 0
        self.stop_count = 0

    @property
    def last_result(self) -> GimbalTrackResult:
        return self._result

    def update(self, _sample: object) -> GimbalRateUpdate:
        self._result = GimbalTrackResult(
            TrackingState.TRACKING,
            True,
            yaw_rate=17.0,
            pitch_rate=-9.0,
        )
        return GimbalRateUpdate(
            self._result,
            GimbalObservationDisposition.ACCEPTED,
        )

    def lose_target(self) -> GimbalTrackResult:
        self.loss_count += 1
        self._result = idle_result()
        return self._result

    def stop(self) -> GimbalTrackResult:
        self.stop_count += 1
        self._result = idle_result()
        return self._result


def _tracking_command() -> tuning_sample.TrackingCommand:
    return tuning_sample.TrackingCommand(
        bbox_center=(350.0, 220.0),
        bbox_size=(40.0, 30.0),
        anchor_pixel=(320.0, 240.0),
        undistorted_bbox_center=(350.0, 220.0),
        undistorted_anchor=(320.0, 240.0),
        delta_yaw_deg=5.0,
        delta_pitch_deg=-3.0,
        tracking_k=np.eye(3),
    )


@pytest.mark.parametrize("timestamp", (None, float("nan")))
def test_tuning_loss_or_invalid_timestamp_zeros_and_returns_idle(
    timestamp: float | None,
) -> None:
    tracker = _TrackingPort()
    detected = make_detected_target(
        timestamp=1.0,
        tracking_bbox_cxcywh=(350.0, 220.0, 40.0, 30.0),
    )
    tracked = tuning_sample.tick_gimbal_tracker(
        tracker,
        _tracking_command(),
        detected,
    )
    assert tracked.state is TrackingState.TRACKING

    lost_target = (
        None
        if timestamp is None
        else make_detected_target(
            timestamp=timestamp,
            tracking_bbox_cxcywh=(350.0, 220.0, 40.0, 30.0),
        )
    )
    command = None if lost_target is None else _tracking_command()
    result = tuning_sample.tick_gimbal_tracker(tracker, command, lost_target)

    assert tracker.loss_count == 1
    assert tracker.stop_count == 0
    assert result == idle_result()


def test_repeated_tuning_loss_emits_one_zero_but_cleanup_forces_final_zero() -> None:
    commands: list[tuple[float, float]] = []

    class _Actuator:
        def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
            commands.append((yaw_rate, pitch_rate))

    tracker = TuningTracker(
        _Actuator(),
        _SilentLog(),
        GimbalRateTrackerConfig(),
    )
    detected = make_detected_target(timestamp=1.0)
    tuning_sample.tick_gimbal_tracker(tracker, _tracking_command(), detected)

    first_loss = tuning_sample.tick_gimbal_tracker(tracker, None, None)
    repeated_loss = tuning_sample.tick_gimbal_tracker(tracker, None, None)

    assert first_loss == idle_result()
    assert repeated_loss == idle_result()
    assert commands.count((0.0, 0.0)) == 1

    tracker.stop()
    assert commands.count((0.0, 0.0)) == 2


def test_tuning_sample_uses_current_grouped_target_schema() -> None:
    target = make_detected_target(
        obj_id=42,
        timestamp=8.5,
        tracking_bbox_cxcywh=(500.0, 400.0, 80.0, 60.0),
        bbox_cxcywh=(100.0, 100.0, 20.0, 20.0),
    )
    assert tuning_sample.tracking_bbox(target) == pytest.approx(
        (500.0, 400.0, 80.0, 60.0)
    )
    assert tuning_sample.source_timestamp(target) == pytest.approx(8.5)
    overlay = tuning_sample.build_overlay_target(target)
    assert overlay is not None
    assert overlay.obj_id == 42


def test_tuning_sample_propagates_programmer_schema_errors() -> None:
    with pytest.raises(AttributeError):
        tuning_sample.source_timestamp(SimpleNamespace(timestamp=2.0))
    with pytest.raises(AttributeError):
        tuning_sample.tracking_bbox(SimpleNamespace(bbox_cxcywh=(1, 2, 3, 4)))


def test_tuning_sample_has_narrow_static_dependencies() -> None:
    source = tuning_sample.__file__
    assert source is not None
    tree = ast.parse(open(source, encoding="utf-8").read())
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    calls = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "Any" not in names
    assert "getattr" not in calls
    assert "GimbalRateTracker" not in names


def test_center_gimbal_uses_public_centering_capability() -> None:
    events: list[tuple[str, object]] = []

    class _Gimbal:
        def set_att(self, attitude: Attitude) -> None:
            events.append(("att", attitude))

        def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
            events.append(("rate", (yaw_rate, pitch_rate)))

    tuning_assembly.center_gimbal(_Gimbal(), _SilentLog())

    assert events == [
        ("att", Attitude(0.0, 0.0, 0.0)),
        ("rate", (0.0, 0.0)),
    ]


def test_gimbal_controller_facade_has_no_star_or_private_test_exports() -> None:
    source = gimbal_controller.__file__
    assert source is not None
    tree = ast.parse(open(source, encoding="utf-8").read())
    imported = [
        alias
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    ]
    assert all(alias.name != "*" for alias in imported)
    assert all(
        alias.asname is None or not alias.asname.startswith("_")
        for alias in imported
    )
    assert not hasattr(gimbal_controller, "_tracking_bbox")
