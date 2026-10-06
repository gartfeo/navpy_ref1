"""Independent-review regressions for the SIYI runner."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from navpy.exception_groups import BaseExceptionGroup
from scripts.python import run_gimbal_tracker_runtime as runner_runtime
from tests.detection_factory import make_detected_poi


class _SilentLog:
    def info(self, _message: str, *args: object) -> None:
        return None

    def warning(self, _message: str, *args: object) -> None:
        return None

    def error(self, _message: str, *args: object) -> None:
        return None

    def debug(self, _message: str, *args: object) -> None:
        return None


class _NavigationProbe:
    def __init__(self) -> None:
        self.updates: list[tuple[object, float, tuple[float, float]]] = []

    def update(
        self,
        poi: object,
        *,
        now: float,
        principal_point: tuple[float, float],
    ) -> None:
        self.updates.append((poi, now, principal_point))


def test_feed_navigation_accepts_real_grouped_poi_pixel_calibration() -> None:
    poi = make_detected_poi(
        obj_id=7,
        task_id=8,
        timestamp=12.5,
        tracking_bbox_cxcywh=(330.0, 250.0, 40.0, 20.0),
        pose_is_frame_atomic=True,
    )
    detector = SimpleNamespace(
        get_detect_data=lambda _request: SimpleNamespace(
            detected_pois=[poi]
        ),
        get_raw_frame=lambda: np.zeros((480, 640, 3), dtype=np.uint8),
    )
    mount = SimpleNamespace(
        get_dist=lambda: np.zeros(5, dtype=float),
        image_width=640,
        image_height=480,
    )
    navigation = _NavigationProbe()
    state = runner_runtime.LoopState.from_anchor(5)

    runner_runtime.feed_navigation(state, detector, navigation, mount)

    assert navigation.updates == [(poi, 12.5, (320.0, 240.0))]


def _runner_args() -> SimpleNamespace:
    return SimpleNamespace(
        preset="vehicles",
        ip=None,
        port=None,
        stream=None,
        model=None,
        conf=None,
        device=None,
        detect_hz=None,
        track_hz=None,
        anchor=5,
        pitch=15.0,
        yaw=0.0,
        zoom=3.0,
        max_rate=None,
        deep_search=None,
        deep_search_hz=None,
        deep_search_imgsz=None,
        deep_search_conf=None,
        tracker_backend=None,
    )


class _CleanupResource:
    def __init__(
        self,
        name: str,
        events: list[str],
        failures: dict[str, BaseException],
    ) -> None:
        self.name = name
        self.events = events
        self.failures = failures

    def call(self, suffix: str) -> None:
        key = f"{self.name}.{suffix}"
        self.events.append(key)
        failure = self.failures.get(key)
        if failure is not None:
            raise failure


def _patch_runner(
    monkeypatch: pytest.MonkeyPatch,
    *,
    stage_failure: str | None = None,
    cleanup_failures: dict[str, BaseException] | None = None,
) -> list[str]:
    events: list[str] = []
    failures = cleanup_failures or {}
    mount_cleanup = _CleanupResource("mount", events, failures)
    navigation_cleanup = _CleanupResource("navigation", events, failures)
    detector_cleanup = _CleanupResource("detector", events, failures)
    window_cleanup = _CleanupResource("window", events, failures)

    mount = SimpleNamespace(
        stop=lambda: mount_cleanup.call("stop"),
        gimbal=SimpleNamespace(request_autofocus=lambda: None),
    )
    navigation = SimpleNamespace(
        stop_tracking=lambda: navigation_cleanup.call("stop"),
    )
    detector = SimpleNamespace(
        start=lambda: _raise_stage(stage_failure, "detector.start"),
        stop=lambda: detector_cleanup.call("stop"),
        stats=lambda: _cleanup_value(detector_cleanup, "stats", {}),
    )
    assembly = SimpleNamespace(
        mount=mount,
        detector_settings={},
        tracking_settings={},
        tracking_setup=object(),
        zoom_config=object(),
        default_stream="stream",
    )
    monkeypatch.setattr(runner_runtime, "parse_args", _runner_args)
    monkeypatch.setattr(runner_runtime, "ConsoleLogger", _SilentLog)
    monkeypatch.setattr(runner_runtime, "assemble_runner", lambda *_: assembly)
    monkeypatch.setattr(
        runner_runtime,
        "apply_detector_overrides",
        lambda _args, settings: settings,
    )
    monkeypatch.setattr(
        runner_runtime,
        "initialize_gimbal",
        lambda *_: _raise_stage(stage_failure, "initialize_gimbal"),
    )
    monkeypatch.setattr(
        runner_runtime,
        "build_navigation",
        lambda *_: _stage_value(stage_failure, "build_navigation", navigation),
    )
    monkeypatch.setattr(
        runner_runtime,
        "build_detector",
        lambda *_: _stage_value(stage_failure, "build_detector", detector),
    )
    monkeypatch.setattr(runner_runtime, "resolve_model_path", lambda *_: "model")
    monkeypatch.setattr(
        runner_runtime.cv2,
        "namedWindow",
        lambda *_: _raise_stage(stage_failure, "namedWindow"),
    )
    monkeypatch.setattr(
        runner_runtime.cv2,
        "destroyAllWindows",
        lambda: window_cleanup.call("destroy"),
    )
    monkeypatch.setattr(runner_runtime, "start_command_reader", lambda *_: None)
    monkeypatch.setattr(runner_runtime, "FocusMonitor", lambda *_args, **_kw: object())
    monkeypatch.setattr(runner_runtime, "run_loop", lambda *_: None)
    return events


def _raise_stage(active: str | None, stage: str) -> None:
    if active == stage:
        raise RuntimeError(stage)


def _stage_value(active: str | None, stage: str, value: object) -> object:
    _raise_stage(active, stage)
    return value


def _cleanup_value(
    resource: _CleanupResource,
    suffix: str,
    value: object,
) -> object:
    resource.call(suffix)
    return value


@pytest.mark.parametrize(
    ("stage", "expected"),
    (
        ("initialize_gimbal", ["mount.stop"]),
        ("build_navigation", ["mount.stop"]),
        ("build_detector", ["navigation.stop", "mount.stop"]),
        (
            "detector.start",
            ["detector.stats", "detector.stop", "navigation.stop", "mount.stop"],
        ),
        (
            "namedWindow",
            [
                "window.destroy",
                "detector.stats",
                "detector.stop",
                "navigation.stop",
                "mount.stop",
            ],
        ),
    ),
)
def test_runner_acquisition_failures_cleanup_in_reverse_order(
    monkeypatch: pytest.MonkeyPatch,
    stage: str,
    expected: list[str],
) -> None:
    events = _patch_runner(monkeypatch, stage_failure=stage)
    with pytest.raises(RuntimeError, match=stage):
        runner_runtime.main()
    assert events == expected


@pytest.mark.parametrize(
    "cleanup_name",
    (
        "window.destroy",
        "detector.stats",
        "detector.stop",
        "navigation.stop",
        "mount.stop",
    ),
)
def test_runner_single_cleanup_failure_is_preserved_and_all_cleanup_runs(
    monkeypatch: pytest.MonkeyPatch,
    cleanup_name: str,
) -> None:
    marker = RuntimeError(cleanup_name)
    events = _patch_runner(monkeypatch, cleanup_failures={cleanup_name: marker})
    with pytest.raises(RuntimeError) as captured:
        runner_runtime.main()
    assert captured.value is marker
    assert events == [
        "window.destroy",
        "detector.stats",
        "detector.stop",
        "navigation.stop",
        "mount.stop",
    ]


def test_runner_multiple_cleanup_failures_are_grouped(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    markers = {
        name: RuntimeError(name)
        for name in ("window.destroy", "detector.stop", "mount.stop")
    }
    events = _patch_runner(monkeypatch, cleanup_failures=markers)
    with pytest.raises(BaseExceptionGroup) as captured:
        runner_runtime.main()
    assert list(captured.value.exceptions) == list(markers.values())
    assert events == [
        "window.destroy",
        "detector.stats",
        "detector.stop",
        "navigation.stop",
        "mount.stop",
    ]
