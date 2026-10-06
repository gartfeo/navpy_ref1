"""Regression contract for the decomposed real-SIYI validation script."""

from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import ANY, Mock, patch

import pytest

from scripts.python import validate_tracking_config as config
from scripts.python import validate_tracking_metrics as metrics
from scripts.python import validate_tracking_orchestration as orchestration
from scripts.python import validate_tracking_runtime as runtime


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_FILES = (
    "scripts/python/validate_tracking.py",
    "scripts/python/validate_tracking_camera_motion.py",
    "scripts/python/validate_tracking_config.py",
    "scripts/python/validate_tracking_metrics.py",
    "scripts/python/validate_tracking_occlusion.py",
    "scripts/python/validate_tracking_orchestration.py",
    "scripts/python/validate_tracking_runtime.py",
)


def _load_facade(name: str, argv: list[str]) -> ModuleType:
    path = REPO_ROOT / "scripts/python/validate_tracking.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    with patch.object(sys, "argv", argv):
        spec.loader.exec_module(module)
    return module


def test_positional_record_seconds_and_public_surface_are_preserved() -> None:
    defaults = _load_facade(
        "validate_tracking_test_defaults",
        ["validate_tracking.py"],
    )
    explicit = _load_facade(
        "validate_tracking_test_explicit",
        ["validate_tracking.py", "12.5"],
    )

    assert defaults.REC_SECS == pytest.approx(90.0)
    assert explicit.REC_SECS == pytest.approx(12.5)
    assert (defaults.CAR, defaults.BUS, defaults.TRUCK) == (2, 5, 7)
    for name in (
        "annotate",
        "associate",
        "build",
        "camera_motion",
        "emit",
        "find_occ",
        "iou",
        "main",
        "record",
        "report_occ",
        "reset_all",
        "step",
    ):
        assert callable(getattr(defaults, name))


def test_invalid_record_seconds_still_raises_value_error() -> None:
    with pytest.raises(ValueError):
        config.parse_record_seconds(["validate_tracking.py", "not-a-number"])


def test_geometric_association_and_occlusion_metrics_are_unchanged() -> None:
    frames = [
        [
            (1, (10.0, 10.0, 4.0, 4.0), 2),
            (2, (100.0, 100.0, 8.0, 8.0), 5),
        ],
        [(1, (11.0, 10.0, 4.0, 4.0), 2)],
        [],
        [(9, (12.0, 10.0, 4.0, 4.0), 2)],
    ]

    tracklets = metrics.associate(frames, coast=2, thr=0.3)

    assert metrics.iou(
        (10.0, 10.0, 4.0, 4.0),
        (10.0, 10.0, 4.0, 4.0),
    ) == pytest.approx(1.0)
    assert [point[1] for point in tracklets[0]["pts"]] == [1, 1, 9]
    event = metrics.find_occ(
        [
            {
                "cls": 2,
                "pts": [
                    (0, 4, (10.0, 10.0, 8.0, 8.0)),
                    (5, 8, (11.0, 10.0, 8.0, 8.0)),
                ],
            }
        ],
        [
            [],
            [(10.0, 10.0, 8.0, 8.0)],
            [],
            [],
            [],
            [],
        ],
        fps=1.0,
    )
    assert event == {
        "fb": 0,
        "fa": 5,
        "gap": 5,
        "ib": 4,
        "ia": 8,
        "bb": (10.0, 10.0, 8.0, 8.0),
        "ba": (11.0, 10.0, 8.0, 8.0),
    }


def test_record_failure_keeps_exit_code_and_stops_provider() -> None:
    provider = Mock()
    provider.get_frame_state.return_value = (None, 0, 0, 0)
    emitted: list[str] = []
    with (
        patch.object(runtime, "FrameProvider", return_value=provider),
        patch.object(runtime.time, "time", side_effect=[0.0, 16.0]),
        pytest.raises(SystemExit) as raised,
    ):
        runtime.record_clip(
            "rtsp://camera",
            "clip.mp4",
            90.0,
            Mock(),
            emitted.append,
        )

    assert raised.value.code == 1
    assert emitted == ["RECORD FAILED: no frames"]
    provider.start.assert_called_once_with()
    provider.stop.assert_called_once_with()


def test_build_stack_keeps_production_backends_and_thresholds() -> None:
    logger = Mock()
    yolo = Mock(device="cuda:0")
    components = tuple(object() for _ in range(5))
    with (
        patch.object(runtime, "YoloDetector", return_value=yolo) as detector,
        patch.object(
            runtime,
            "create_tracker_backend",
            return_value=components[0],
        ) as tracker,
        patch.object(
            runtime,
            "TrackIdentityResolver",
            return_value=components[1],
        ) as identity,
        patch.object(
            runtime,
            "create_appearance_embedder",
            return_value=components[2],
        ) as embedder,
        patch.object(
            runtime,
            "TargetLock",
            return_value=components[3],
        ) as target_lock,
        patch.object(
            runtime,
            "LostTargetBridge",
            return_value=components[4],
        ),
    ):
        stack = runtime.build_stack("model.pt", logger)

    detector.assert_called_once_with(
        "model.pt",
        imgsz=640,
        conf=0.25,
        device="auto",
        classes=[2, 5, 7],
        logger=logger,
    )
    tracker.assert_called_once_with(
        {
            "backend": "botsort",
            "with_reid": False,
            "reid_device": "auto",
            "reid_weights": "clip_veri.pt",
            "cmc_method": "sof",
            "max_age": 90,
            "min_hits": 3,
            "conf": 0.25,
        },
        detector_device="cuda:0",
        detector_conf=0.25,
        logger=logger,
    )
    identity.assert_called_once_with()
    embedder.assert_called_once_with(
        {
            "enabled": True,
            "reid_device": "auto",
            "half": True,
            "weights": "clip_veri.pt",
        },
        detector_device="cuda:0",
        logger=logger,
    )
    target_lock.assert_called_once_with(
        max_lost_frames=120,
        auto_lock=False,
    )
    assert stack == (
        yolo,
        components[0],
        components[1],
        components[2],
        components[3],
        components[4],
    )


def test_orchestration_preserves_report_sections_and_fresh_camera_stack(
    tmp_path: Path,
) -> None:
    metrics_path = tmp_path / "metrics.txt"
    primary, camera = object(), object()
    frames = [[(1, (320.0, 240.0, 40.0, 20.0), 2)]]
    with (
        patch.object(
            orchestration,
            "record_clip",
            return_value=(640, 480),
        ),
        patch.object(
            orchestration,
            "build_stack",
            side_effect=[primary, camera],
        ) as build_stack,
        patch.object(
            orchestration,
            "replay_clip",
            return_value=(1.0, frames, [[]], [1]),
        ),
        patch.object(orchestration, "camera_motion") as camera_motion,
    ):
        orchestration.run_validation(
            3.0,
            str(metrics_path),
            "rtsp://camera",
            "model.pt",
            "clip.mp4",
            str(tmp_path),
            Mock(),
        )

    report = metrics_path.read_text(encoding="utf-8")
    assert "COMPREHENSIVE REAL-SIYI TRACKING VALIDATION" in report
    assert "\n[1] RECOGNITION\n" in report
    assert "\n[2] ID STABILITY " in report
    assert "\n[3] BUS / LARGE-VEHICLE OCCLUSION" in report
    assert "\n[4] CAMERA MOTION " in report
    assert report.endswith("evidence saved under validation_evidence/\n")
    assert build_stack.call_count == 2
    camera_motion.assert_called_once_with(
        camera,
        1.0,
        "clip.mp4",
        ANY,
    )


def test_validation_modules_and_functions_stay_bounded() -> None:
    violations: list[str] = []
    for relative in SCRIPT_FILES:
        path = REPO_ROOT / relative
        source = path.read_text(encoding="utf-8")
        if len(source.splitlines()) > 300:
            violations.append(f"{relative}: module exceeds 300 lines")
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            if node.end_lineno - node.lineno + 1 > 150:
                violations.append(
                    f"{relative}:{node.lineno} {node.name} exceeds 150 lines"
                )
    assert not violations
