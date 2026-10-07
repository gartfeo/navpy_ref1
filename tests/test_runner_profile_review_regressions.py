"""Profile ownership and façade regressions for the SIYI runner."""

from __future__ import annotations

import ast
from pathlib import Path
from typing import Any

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from scripts.python import run_gimbal_tracker as runner_facade
from scripts.python import run_gimbal_tracker_args as runner_args
from scripts.python import run_gimbal_tracker_profile as runner_profile


class _SilentLog:
    def info(self, _message: str, *args: object) -> None:
        return None

    def warning(self, _message: str, *args: object) -> None:
        return None


def _minimal_profile() -> dict[str, Any]:
    return {
        "detector": {
            "dock_presets": {
                "dock": {"min_pixel_size": 73.0},
            }
        },
        "devices": [
            {
                "name": "siyi_zr10",
                "gimbal": {
                    "type": "siyi",
                    "tracking": {"enabled": True},
                    "zoom": {"enabled": True},
                },
            }
        ],
    }


def _patch_profile_builders(
    monkeypatch: pytest.MonkeyPatch,
    profile: dict[str, Any],
) -> None:
    monkeypatch.setattr(
        runner_profile,
        "resolve_profile",
        lambda *_: ("siyi_zr10", profile, None),
    )
    monkeypatch.setattr(
        runner_profile,
        "build_camera_model",
        lambda *_a, **_k: object(),
    )
    monkeypatch.setattr(runner_profile, "build_zoom_calibration", lambda *_: None)
    monkeypatch.setattr(
        runner_profile,
        "build_gimbal_data",
        lambda *_: GimbalData(att=Attitude(0.0, 0.0, 0.0)),
    )


def test_runner_presets_own_only_model_and_classes() -> None:
    assert runner_args.DETECTOR_PRESETS["vehicles"] == ("yolov8s.pt", [2])


def test_runner_uses_canonical_profile_target_thresholds(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profile = _minimal_profile()
    _patch_profile_builders(monkeypatch, profile)
    loaded = runner_profile.load_siyi_profile(_SilentLog())
    assert loaded.zoom_config is not None
    assert loaded.zoom_config.target_pixels["0"] == pytest.approx(73.0)


@pytest.mark.parametrize(
    ("path", "bad_value", "message"),
    (
        (("devices", 0, "gimbal", "tracking", "enabled"), "false", "boolean"),
        (("devices", 0, "gimbal", "zoom", "enabled"), "false", "boolean"),
        (("detector", "deep_search"), "not-a-mapping", "mapping"),
    ),
)
def test_runner_profile_rejects_raw_bool_and_group_shapes(
    monkeypatch: pytest.MonkeyPatch,
    path: tuple[object, ...],
    bad_value: object,
    message: str,
) -> None:
    profile = _minimal_profile()
    cursor: Any = profile
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = bad_value
    _patch_profile_builders(monkeypatch, profile)
    with pytest.raises(ValueError, match=message):
        runner_profile.load_siyi_profile(_SilentLog())


def test_runner_facade_has_only_intentional_public_leaf_exports() -> None:
    source = runner_facade.__file__
    assert source is not None
    tree = ast.parse(Path(source).read_text(encoding="utf-8"))
    private_aliases = {
        alias.asname
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
        if alias.asname is not None and alias.asname.startswith("_")
    }
    assert not private_aliases
    assert not hasattr(runner_facade, "_Logger")
    assert not hasattr(runner_facade, "_feed_navigation")
