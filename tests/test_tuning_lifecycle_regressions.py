"""Transactional lifecycle regressions for the SIYI tuning runtime."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from navpy.exception_groups import BaseExceptionGroup
from tools.cam import gimbal_tuning_runtime as tuning_runtime


class _SilentLog:
    def info(self, _message: str, *args: object) -> None:
        return None

    def warning(self, _message: str, *args: object) -> None:
        return None


def _args() -> SimpleNamespace:
    return SimpleNamespace(
        zoom=1.0,
        profile="siyi_zr10",
        ip=None,
        port=None,
        conf=None,
        device=None,
        max_rate=100.0,
        anchor=5,
        camera=None,
        model=None,
    )


class _Cleanup:
    def __init__(
        self,
        events: list[str],
        failures: dict[str, BaseException],
    ) -> None:
        self.events = events
        self.failures = failures

    def call(self, name: str) -> None:
        self.events.append(name)
        failure = self.failures.get(name)
        if failure is not None:
            raise failure


def _raise_stage(active: str | None, stage: str) -> None:
    if active == stage:
        raise RuntimeError(stage)


def _patch_runtime(
    monkeypatch: pytest.MonkeyPatch,
    *,
    stage_failure: str | None = None,
    cleanup_failures: dict[str, BaseException] | None = None,
) -> list[str]:
    events: list[str] = []
    cleanup = _Cleanup(events, cleanup_failures or {})

    gimbal = SimpleNamespace(
        is_connected=lambda: _connected(stage_failure),
    )
    mount = SimpleNamespace(
        gimbal=gimbal,
        start=lambda: _raise_stage(stage_failure, "mount.start"),
        stop=lambda: cleanup.call("mount.stop"),
    )
    tracker = SimpleNamespace(
        stop=lambda: cleanup.call("tracker.stop"),
        reset=lambda: cleanup.call("tracker.reset"),
    )
    detector = SimpleNamespace(
        start=lambda: _raise_stage(stage_failure, "detector.start"),
        stop=lambda: cleanup.call("detector.stop"),
    )

    monkeypatch.setattr(
        tuning_runtime,
        "build_mount",
        lambda *_: (mount, {}, "stream"),
    )
    monkeypatch.setattr(
        tuning_runtime,
        "capture_calibrated_intrinsics",
        lambda *_: _stage_value(stage_failure, "capture", {1.0: object()}),
    )
    monkeypatch.setattr(tuning_runtime, "zoom_sequence", lambda *_: [1.0])
    monkeypatch.setattr(
        tuning_runtime,
        "apply_detector_overrides",
        lambda *_: {},
    )

    def tracker_factory(*_args: object, **_kwargs: object) -> object:
        return _stage_value(stage_failure, "tracker.build", tracker)

    monkeypatch.setattr(
        tuning_runtime,
        "GimbalRateTracker",
        tracker_factory,
        raising=False,
    )
    monkeypatch.setattr(
        tuning_runtime,
        "build_tuning_tracker",
        tracker_factory,
        raising=False,
    )
    monkeypatch.setattr(
        tuning_runtime,
        "center_gimbal",
        lambda *_: _raise_stage(stage_failure, "center"),
    )
    # Replace the module-level ``time`` binding with a namespace so the fake
    # sleep is confined to tuning_runtime and never mutates the shared stdlib
    # ``time.sleep`` (which would busy-spin any leaked background sleeper).
    monkeypatch.setattr(
        tuning_runtime,
        "time",
        SimpleNamespace(sleep=lambda *_: None),
    )
    monkeypatch.setattr(
        tuning_runtime,
        "apply_zoom",
        lambda *_: _stage_value(stage_failure, "apply_zoom", object()),
    )
    monkeypatch.setattr(tuning_runtime, "parse_source", lambda value: value)
    monkeypatch.setattr(tuning_runtime, "default_model_path", lambda: "model")
    monkeypatch.setattr(
        tuning_runtime,
        "build_detector",
        lambda *_: _stage_value(stage_failure, "build_detector", detector),
    )
    monkeypatch.setattr(
        tuning_runtime.cv2,
        "namedWindow",
        lambda *_: _raise_stage(stage_failure, "namedWindow"),
    )
    monkeypatch.setattr(
        tuning_runtime.cv2,
        "destroyAllWindows",
        lambda: cleanup.call("window.destroy"),
    )
    monkeypatch.setattr(tuning_runtime, "_run_loop", lambda *_: None)
    return events


def _connected(active: str | None) -> bool:
    _raise_stage(active, "connected")
    return True


def _stage_value(active: str | None, stage: str, value: object) -> object:
    _raise_stage(active, stage)
    return value


@pytest.mark.parametrize(
    ("stage", "expected"),
    (
        ("capture", ["mount.stop"]),
        ("tracker.build", ["mount.stop"]),
        ("mount.start", ["tracker.stop", "mount.stop"]),
        ("connected", ["tracker.stop", "mount.stop"]),
        ("center", ["tracker.stop", "mount.stop"]),
        ("apply_zoom", ["tracker.stop", "mount.stop"]),
        ("build_detector", ["tracker.stop", "mount.stop"]),
        (
            "detector.start",
            ["detector.stop", "tracker.stop", "mount.stop"],
        ),
        (
            "namedWindow",
            [
                "window.destroy",
                "detector.stop",
                "tracker.stop",
                "mount.stop",
            ],
        ),
    ),
)
def test_tuning_acquisition_failures_cleanup_in_reverse_order(
    monkeypatch: pytest.MonkeyPatch,
    stage: str,
    expected: list[str],
) -> None:
    events = _patch_runtime(monkeypatch, stage_failure=stage)
    with pytest.raises(RuntimeError, match=stage):
        tuning_runtime.run(_args(), _SilentLog())
    assert events == expected


@pytest.mark.parametrize(
    "cleanup_name",
    ("window.destroy", "detector.stop", "tracker.stop", "mount.stop"),
)
def test_tuning_single_cleanup_error_keeps_identity_and_runs_all(
    monkeypatch: pytest.MonkeyPatch,
    cleanup_name: str,
) -> None:
    marker = RuntimeError(cleanup_name)
    events = _patch_runtime(
        monkeypatch,
        cleanup_failures={cleanup_name: marker},
    )
    with pytest.raises(RuntimeError) as captured:
        tuning_runtime.run(_args(), _SilentLog())
    assert captured.value is marker
    assert events == [
        "window.destroy",
        "detector.stop",
        "tracker.stop",
        "mount.stop",
    ]


def test_tuning_multiple_cleanup_errors_are_grouped(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    markers = {
        name: RuntimeError(name)
        for name in ("window.destroy", "tracker.stop", "mount.stop")
    }
    events = _patch_runtime(monkeypatch, cleanup_failures=markers)
    with pytest.raises(BaseExceptionGroup) as captured:
        tuning_runtime.run(_args(), _SilentLog())
    assert list(captured.value.exceptions) == list(markers.values())
    assert events == [
        "window.destroy",
        "detector.stop",
        "tracker.stop",
        "mount.stop",
    ]


def test_cleanup_stack_is_reverse_ordered_and_retry_safe() -> None:
    from navpy.modules.common.resource_cleanup import CleanupStack

    events: list[str] = []
    marker = RuntimeError("cleanup")
    stack = CleanupStack()
    stack.push(lambda: events.append("first"))
    attempts = 0

    def fail() -> None:
        nonlocal attempts
        events.append("second")
        attempts += 1
        if attempts == 1:
            raise marker

    stack.push(fail)
    with pytest.raises(RuntimeError) as captured:
        stack.close()
    assert captured.value is marker
    assert events == ["second", "first"]

    stack.close()
    assert events == ["second", "first", "second"]


def test_cleanup_stack_preserves_clean_primary_exception_identity() -> None:
    from navpy.modules.common.resource_cleanup import CleanupStack

    marker = RuntimeError("body")
    with pytest.raises(RuntimeError) as captured:
        with CleanupStack() as stack:
            stack.push(lambda: None)
            raise marker
    assert captured.value is marker


def test_cleanup_stack_groups_primary_and_cleanup_errors() -> None:
    from navpy.modules.common.resource_cleanup import CleanupStack

    primary = RuntimeError("body")
    cleanup = RuntimeError("cleanup")

    def fail() -> None:
        raise cleanup

    with pytest.raises(BaseExceptionGroup) as captured:
        with CleanupStack() as stack:
            stack.push(fail)
            raise primary
    assert list(captured.value.exceptions) == [primary, cleanup]
