"""Regression contract for the decomposed diagnostic runners and camera tool."""

from __future__ import annotations

import importlib
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from navpy.modules.vision.gimbal_rate_types import GimbalRateTrackerConfig
from navpy.modules.vision.gimbal_tracking_sample import GimbalAngularSample
from tests.detection_factory import make_detected_poi


REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_path(name: str, relative: str) -> ModuleType:
    path = REPO_ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


runner = _load_path(
    "solid_run_gimbal_tracker",
    "scripts/python/run_gimbal_tracker.py",
)
TOOLS_ROOT = REPO_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))
gimbal_controller = importlib.import_module("cam.gimbal_controller")
gimbal_sample = importlib.import_module("cam.gimbal_tuning_sample")
gimbal_tracking = importlib.import_module("cam.gimbal_tuning_tracking")


class _Actuator:
    def __init__(self) -> None:
        self.commands: list[tuple[float, float]] = []

    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None:
        self.commands.append((yaw_rate, pitch_rate))


class _Log:
    def debug(self, _message: str) -> None:
        return None

    def info(self, _message: str) -> None:
        return None

    def warning(self, _message: str) -> None:
        return None


def _intrinsics() -> object:
    return gimbal_controller.TrackingIntrinsics(
        zoom=1.0,
        fx=1000.0,
        fy=1000.0,
        cx=500.0,
        cy=500.0,
        k=np.asarray(
            [
                [1000.0, 0.0, 500.0],
                [0.0, 1000.0, 500.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        dist_coeffs=np.zeros(5, dtype=np.float32),
        source="calibrated",
        reference_zoom=1.0,
    )


@pytest.mark.parametrize("removed", ["--kp-yaw", "--kp-pitch"])
def test_gimbal_controller_rejects_removed_inert_gain_knobs(
    removed: str,
) -> None:
    with patch.object(sys, "argv", ["gimbal_controller.py"]):
        defaults = gimbal_controller.parse_args()
    assert not hasattr(defaults, "kp_yaw")
    assert not hasattr(defaults, "kp_pitch")
    assert not hasattr(gimbal_controller, "DEFAULT_TRACK_KP")

    with patch.object(sys, "argv", ["gimbal_controller.py", removed, "1.2"]):
        with pytest.raises(SystemExit):
            gimbal_controller.parse_args()


def test_gimbal_controller_ticks_current_tracker_with_exact_angular_sample() -> None:
    actuator = _Actuator()
    tracker = gimbal_tracking.TuningTracker(
        actuator,
        _Log(),
        GimbalRateTrackerConfig(),
    )
    idle = gimbal_sample.tick_gimbal_tracker(tracker, None, None)
    assert idle is tracker.last_result
    assert actuator.commands == []

    command = gimbal_controller.build_bbox_tracking_command(
        (600.0, 450.0, 80.0, 60.0),
        _intrinsics(),
        anchor_mode=5,
        image_width=1000,
        image_height=1000,
    )
    assert command is not None
    poi = make_detected_poi(timestamp=12.5)
    sample = gimbal_sample.build_tracker_poi(command, poi)
    assert isinstance(sample, GimbalAngularSample)
    assert sample.source_timestamp_s == pytest.approx(12.5)

    result = gimbal_sample.tick_gimbal_tracker(tracker, command, poi)
    assert result.has_poi
    assert len(actuator.commands) == 1


def test_run_gimbal_tracker_rejects_duplicate_performance_profile_knob() -> None:
    with patch.object(sys, "argv", ["run_gimbal_tracker.py"]):
        defaults = runner.parse_args()
    assert not hasattr(defaults, "perf_profile")
    assert not hasattr(runner, "PERF_PROFILES")
    assert not hasattr(runner, "DEFAULT_PERF_PROFILE")
    assert not hasattr(runner, "DEFAULT_MAX_RATE")
    assert not hasattr(runner, "DEFAULT_SIYI_STREAM")

    with patch.object(
        sys,
        "argv",
        ["run_gimbal_tracker.py", "--perf-profile", "native"],
    ):
        with pytest.raises(SystemExit):
            runner.parse_args()


def test_run_gimbal_tracker_uses_profile_values_unless_explicitly_overridden() -> None:
    base = {
        "detect_hz": 30.0,
        "track_hz": 60.0,
        "tracker": {"backend": "botsort", "frame_rate": 30},
        "deep_search": {"enabled": True, "hz": 2.0},
    }
    defaults = SimpleNamespace(
        detect_hz=None,
        track_hz=None,
        deep_search=None,
        deep_search_hz=None,
        deep_search_imgsz=None,
        deep_search_conf=None,
        tracker_backend=None,
    )
    assert runner.apply_detector_overrides(defaults, base) == base

    explicit = SimpleNamespace(
        detect_hz=12.0,
        track_hz=24.0,
        deep_search=True,
        deep_search_hz=3.0,
        deep_search_imgsz=1280,
        deep_search_conf=0.1,
        tracker_backend="strongsort",
    )
    resolved = runner.apply_detector_overrides(explicit, base)
    assert resolved["detect_hz"] == pytest.approx(12.0)
    assert resolved["track_hz"] == pytest.approx(24.0)
    assert resolved["tracker"] == {
        "backend": "strongsort",
        "frame_rate": 12,
    }
    assert resolved["deep_search"] == {
        "enabled": True,
        "hz": 3.0,
        "imgsz": 1280,
        "conf": 0.1,
    }
    assert base["detect_hz"] == pytest.approx(30.0)


def test_runner_and_tool_facades_are_thin_exact_reexports() -> None:
    expected_modules = (
        "scripts/python/run_gimbal_tracker_args.py",
        "scripts/python/run_gimbal_tracker_profile.py",
        "scripts/python/run_gimbal_tracker_assembly.py",
        "scripts/python/run_gimbal_tracker_commands.py",
        "scripts/python/run_gimbal_tracker_runtime.py",
        "tools/cam/gimbal_tuning_geometry.py",
        "tools/cam/gimbal_tuning_sample.py",
        "tools/cam/gimbal_tuning_overlay.py",
        "tools/cam/gimbal_tuning_runtime.py",
    )
    missing = [relative for relative in expected_modules if not (REPO_ROOT / relative).is_file()]
    assert not missing
    for relative in expected_modules:
        assert len((REPO_ROOT / relative).read_text(encoding="utf-8").splitlines()) <= 300
    assert len((REPO_ROOT / "scripts/python/run_gimbal_tracker.py").read_text(
        encoding="utf-8"
    ).splitlines()) <= 100
    assert len((REPO_ROOT / "tools/cam/gimbal_controller.py").read_text(
        encoding="utf-8"
    ).splitlines()) <= 100

    runner_commands = importlib.import_module(
        "scripts.python.run_gimbal_tracker_commands"
    )
    gimbal_geometry = importlib.import_module("cam.gimbal_tuning_geometry")
    assert runner.anchor_pixel is runner_commands.anchor_pixel
    assert runner.build_anchor_k is runner_commands.build_anchor_k
    assert gimbal_controller.TrackingIntrinsics is gimbal_geometry.TrackingIntrinsics
    assert (
        gimbal_controller.build_bbox_tracking_command
        is gimbal_geometry.build_bbox_tracking_command
    )
