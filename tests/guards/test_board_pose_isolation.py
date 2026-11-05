"""Pure Vision guard for the own-detection orbit centre (HITL step 22).

The metric board pose and the capture-time camera location place the
waiting orbit before NAV. They must never reach NAV's command law: only the
named acquisition modules may name them, the command record has no field
for them, and poisoning them leaves the record unchanged.
"""

from __future__ import annotations

import ast
import math
from dataclasses import fields, replace
from pathlib import Path

from navpy.core.approach.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.core.kernel.attitude import Attitude
from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.core.ports.gimbal_abc import GimbalData
from tests.detection_factory import make_detected_poi

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src" / "navpy"

METRIC_NAMES = frozenset({
    "board_pose", "capture_camera_location", "capture_location",
    "centre_camera_m", "BoardPoseFit", "BoardPoseResult", "BoardPoseSolver",
    "CharucoBoardPoseSolver", "TrackBoardPose", "board_orbit_centre",
    "BoardOrbitCentre", "read_capture_location",
})
METRIC_MODULES = ("navpy.core.ports.board_pose", "navpy.plugins.detection.charuco.board_pose",
                  "navpy.core.perception.pipeline.capture_position",
                  "navpy.core.perception.pipeline.real_board_pose",
                  "navpy.core.skills.geo_approach.board_orbit_centre")
# The acquisition chain: producer, carrier and the one consumer before NAV.
OWNERS = frozenset({
    "core/ports/board_pose.py",
    "core/ports/detection_components.py",
    "plugins/detection/charuco/board_pose.py",
    "core/perception/pipeline/capture_position.py",
    "core/perception/pipeline/real_board_pose.py",
    "core/perception/pipeline/real_frame_association.py",
    "core/perception/pipeline/real_detected_object_builder.py",
    "core/perception/pipeline/real_detection_mapper.py",
    "core/perception/pipeline/real_detector_models.py",
    "core/perception/pipeline/real_detector_runtime_composition.py",
    "core/skills/geo_approach/board_orbit_centre.py",
    "core/skills/geo_approach/self_detected_approach.py",
    "app/flow/nav_task_composition.py",
    "app/perception/real_detector_model_builder.py",
})


def _uses(tree: ast.AST) -> list[tuple[int, str]]:
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr in METRIC_NAMES:
            found.append((node.lineno, node.attr))
        elif isinstance(node, ast.Name) and node.id in METRIC_NAMES:
            found.append((node.lineno, node.id))
        elif isinstance(node, ast.keyword) and node.arg in METRIC_NAMES:
            found.append((node.value.lineno, node.arg))
        elif isinstance(node, ast.ImportFrom) and node.module in METRIC_MODULES:
            found.append((node.lineno, node.module))
        elif isinstance(node, ast.Import):
            found.extend((node.lineno, a.name) for a in node.names if a.name in METRIC_MODULES)
        elif isinstance(node, ast.Constant) and node.value in METRIC_NAMES:
            found.append((node.lineno, node.value))
    return found


def test_only_the_acquisition_chain_names_the_metric_pose():
    offenders = []
    for path in SRC.rglob("*.py"):
        relative = path.relative_to(SRC).as_posix()
        if relative in OWNERS:
            continue
        for line, name in _uses(ast.parse(path.read_text(encoding="utf-8"))):
            offenders.append(f"{relative}:{line} names {name}")
    assert offenders == []


def test_no_nav_law_module_is_an_owner():
    law_homes = ("core/approach/", "plugins/laws/", "core/command/")
    assert [owner for owner in OWNERS if owner.startswith(law_homes)] == []


def test_the_command_record_has_no_metric_field():
    names = {field.name for field in fields(FinalApproachVisionFrame)}
    assert not names & METRIC_NAMES
    assert not {name for name in names if "location" in name or "range" in name or "pose" in name}


class _Poison:
    """Any read of a poisoned value fails the test."""

    def __getattr__(self, name):
        raise AssertionError(f"command path read .{name} of a metric value")

    def __bool__(self):
        raise AssertionError("command path tested a metric value")


def _frame(poi) -> FinalApproachVisionFrame:
    projector = FinalApproachFrameProjector(FinalApproachProjectionConfig("ZYX", True))
    return projector.project(poi.visual_detection(), source_generation=1)


def test_poisoned_board_pose_and_capture_location_leave_the_record_unchanged():
    poi = make_detected_poi(
        uas_att=Attitude(pitch=4.0, yaw=0.0, roll=-6.0),
        g_data=GimbalData(att=Attitude(pitch=-30.0, yaw=8.0, roll=0.0), name="zr10"),
        pose_is_frame_atomic=True, timestamp=12.5, uas_body_rates_rad_s=(0.0, 0.01, 0.02),
    )
    clean = _frame(poi)
    assert clean is not None and math.isfinite(clean.body_x)
    poi.geo = replace(poi.geo, board_pose=_Poison(), capture_camera_location=_Poison())
    assert _frame(poi) == clean
