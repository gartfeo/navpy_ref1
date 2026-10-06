"""Review regressions for the point-mass visual sensor boundary."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.vision.models.pixel_observation import VisualDetection
from scripts import vision_static_point_mass as point_mass
from scripts import vision_static_point_mass_run as point_mass_run
from scripts import vision_static_point_mass_sensor as point_mass_sensor


def _case() -> point_mass.StaticPointMassCase:
    return point_mass.StaticPointMassCase(
        name="sensor-boundary",
        wind_speed_mps=0.0,
        wind_dir_from_deg=0.0,
        max_t_s=20.0,
    )


def test_point_mass_rejects_prebuilt_terminal_frame_factory() -> None:
    frame = TerminalVisionFrame(
        "injected",
        0,
        1,
        1,
        0.0,
        1.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
    )

    with pytest.raises(TypeError):
        point_mass_run.run_case_analysis(
            _case(),
            detection_factory=lambda **_kwargs: frame,
        )


def test_point_mass_runner_accepts_only_visual_detection_injection() -> None:
    publications: list[VisualDetection] = []

    def detection_factory(**kwargs: object) -> VisualDetection:
        detection = point_mass_sensor.build_static_detection(**kwargs)
        publications.append(detection)
        return detection

    analysis = point_mass_run.run_case_analysis(
        _case(),
        detection_factory=detection_factory,
    )

    assert publications
    assert analysis.command_timestamps_s[0] == pytest.approx(
        publications[0].observation.source_timestamp_s
    )


def test_point_mass_runner_owns_terminal_projection_boundary() -> None:
    run_source = Path(point_mass_run.__file__).read_text(encoding="utf-8")
    sensor_source = Path(point_mass_sensor.__file__).read_text(encoding="utf-8")
    run_tree = ast.parse(run_source)
    sensor_tree = ast.parse(sensor_source)
    run_names = {
        node.id for node in ast.walk(run_tree) if isinstance(node, ast.Name)
    }
    sensor_names = {
        node.id for node in ast.walk(sensor_tree) if isinstance(node, ast.Name)
    }

    assert "TerminalVisionFrame" not in run_names
    assert "TerminalFrameProjector" in run_names
    assert "TerminalFrameProjector" not in sensor_names
    assert "VisualDetection" in sensor_names
    assert hasattr(point_mass_sensor, "build_static_detection")
    assert not hasattr(point_mass_sensor, "build_static_observation")
