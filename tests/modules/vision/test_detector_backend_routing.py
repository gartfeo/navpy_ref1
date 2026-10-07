"""Backend selection for the real detector: CLI -> factory -> frame detector."""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from navpy.modules.vision.charuco_board import CharucoBoardSpec
from navpy.modules.vision.charuco_detector import CharucoBoardDetector
from navpy.modules.vision.real_detector_config import (
    DetectorModelConfig,
    RealDetectorConfig,
)
from navpy.modules.vision.real_detector_factory import RealDetectorFactory
from navpy.modules.vision.real_detector_models import build_models
from navpy.modules.vision.real_detector_state import RuntimeMetrics
from navpy.modules.vision.vision_profiles import CameraMountSpec

_MODELS = "navpy.modules.vision.real_detector_models"


def _mount_spec() -> CameraMountSpec:
    mount = Mock()
    mount.name = "main"
    return CameraMountSpec(
        mount=mount, device={}, gimbal_device_id=1, profile_device_index=0,
    )


def _created_config(factory: RealDetectorFactory, settings: dict):
    detector_ctor = Mock(return_value=Mock())
    with patch("navpy.modules.vision.detector.Detector", detector_ctor):
        factory.create(_mount_spec(), settings, 0)
    _, config = detector_ctor.call_args.args
    return config


def test_charuco_factory_needs_no_model_and_reads_board_from_profile():
    factory = RealDetectorFactory(
        Mock(), Mock(), {}, "", False, backend="charuco",
    )

    config = _created_config(factory, {
        "conf": 0.2,
        "charuco": {"squares_x": 6, "squares_y": 8},
    })

    assert config.model.backend == "charuco"
    assert config.model.model_path == ""
    assert config.model.conf == pytest.approx(0.2)
    assert config.model.charuco_board == CharucoBoardSpec(
        squares_x=6, squares_y=8,
    )


def test_charuco_factory_defaults_to_bench_board_without_profile_block():
    factory = RealDetectorFactory(
        Mock(), Mock(), {}, "", False, backend="charuco",
    )

    assert _created_config(factory, {}).model.charuco_board == CharucoBoardSpec()


def test_yolo_factory_passes_resolved_model_path():
    factory = RealDetectorFactory(
        Mock(), Mock(), {}, "pyproject.toml", False, backend="yolo",
    )

    config = _created_config(factory, {})

    assert config.model.backend == "yolo"
    assert Path(config.model.model_path).name == "pyproject.toml"


def test_yolo_factory_requires_model_path():
    logger = Mock()
    with pytest.raises(ValueError, match="Model path is required"):
        RealDetectorFactory(Mock(), logger, {}, "", False, backend="yolo")
    assert "--detector-backend=yolo" in logger.error.call_args.args[0]


def test_factory_rejects_unknown_backend():
    with pytest.raises(ValueError, match="Unknown detector backend"):
        RealDetectorFactory(Mock(), Mock(), {}, "", False, backend="face")


def test_model_config_rejects_unknown_backend():
    with pytest.raises(ValueError, match="Unknown detector backend"):
        DetectorModelConfig("", backend="face")


def test_charuco_models_never_build_yolo_and_disable_deep_search():
    logger = Mock()
    tracker = Mock()
    config = RealDetectorConfig(model=DetectorModelConfig(
        "",
        conf=0.3,
        backend="charuco",
        deep_search={"enabled": True},
    ))

    with patch(
        f"{_MODELS}.YoloDetector",
        side_effect=AssertionError("YOLO must not be built"),
    ), patch(
        f"{_MODELS}.DeepSearchDetector",
        side_effect=AssertionError("deep search must not be built"),
    ), patch(
        f"{_MODELS}.create_tracker_backend", return_value=tracker,
    ) as create_tracker, patch(
        f"{_MODELS}.create_appearance_embedder", return_value=None,
    ):
        models = build_models(logger, config, RuntimeMetrics())

    assert isinstance(models.frame_detector, CharucoBoardDetector)
    assert models.frame_detector.min_confidence == pytest.approx(0.3)
    assert models.deep_search is None and models.deep_config is None
    assert create_tracker.call_args.kwargs["detector_device"] == "cpu"
    assert create_tracker.call_args.kwargs["detector_conf"] == pytest.approx(0.3)
    assert "deep_search is YOLO-only" in logger.warning.call_args.args[0]


def test_yolo_models_build_yolo_and_deep_search_unchanged():
    yolo = Mock(device="cpu")
    deep = Mock()
    config = RealDetectorConfig(model=DetectorModelConfig(
        "model.pt", deep_search={"enabled": True},
    ))

    with patch(f"{_MODELS}.YoloDetector", return_value=yolo) as yolo_ctor, patch(
        f"{_MODELS}.DeepSearchDetector", return_value=deep,
    ), patch(
        f"{_MODELS}.create_tracker_backend", return_value=Mock(),
    ), patch(
        f"{_MODELS}.create_appearance_embedder", return_value=None,
    ):
        models = build_models(Mock(), config, RuntimeMetrics())

    assert yolo_ctor.call_args.args == ("model.pt",)
    assert models.frame_detector is yolo
    assert models.deep_search is deep


def test_vision_composition_passes_cli_backend_to_real_factory():
    from navpy.modules.vision.vision_controller_composition import (
        build_vision_controller,
    )

    stop = RuntimeError("stop after factory construction")
    profile = Mock(mount_specs=(), detector_settings={}, profile={})
    vision_args = Mock(
        detector_type="real",
        detector_backend="charuco",
        model_path="",
        debug_show=False,
    )
    with patch(
        "navpy.modules.vision.vision_controller_composition.build_vision_profile",
        return_value=profile,
    ), patch(
        "navpy.modules.vision.real_detector_factory.RealDetectorFactory",
        side_effect=stop,
    ) as factory:
        with pytest.raises(RuntimeError, match="stop after factory"):
            build_vision_controller(Mock(), Mock(), vision_args, Mock(), None, None)

    assert factory.call_args.kwargs == {"backend": "charuco"}
    assert factory.call_args.args[3] == ""


def test_charuco_model_build_does_not_import_ultralytics():
    script = textwrap.dedent("""
        import sys
        from unittest.mock import Mock
        from navpy.modules.vision.real_detector_config import (
            DetectorModelConfig, RealDetectorConfig,
        )
        from navpy.modules.vision.real_detector_models import build_models
        from navpy.modules.vision.real_detector_state import RuntimeMetrics
        config = RealDetectorConfig(
            model=DetectorModelConfig("", backend="charuco"),
        )
        models = build_models(Mock(), config, RuntimeMetrics())
        models.frame_detector.close()
        models.tracker.close()
        assert "ultralytics" not in sys.modules, "ultralytics imported"
    """)
    src = Path(__file__).resolve().parents[3] / "src"
    result = subprocess.run(
        [sys.executable, "-c", script],
        env={**os.environ, "PYTHONPATH": str(src)},
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert result.returncode == 0, result.stderr
