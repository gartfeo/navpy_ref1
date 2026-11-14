"""Pure Vision guard for the whole-dock detector (HITL step 23b; hitl_dock_detector.md section 4.2).

The NAV command and completion paths read only the box centre as a frame-local ray, with gimbal readback as camera
state (and the owner's allowed gyro turn rate). Box size, the orbit centre, aircraft heading, position and altitude
never reach them: changing or poisoning each leaves the command frame, the law's command, the visual-pass
(completion) plan and the confirmability unchanged. Box size is used only by the acquisition gate, before a POI
exists; the new detector and gate modules are never named by a NAV module.
"""
from __future__ import annotations

import ast
import threading
from dataclasses import replace
from pathlib import Path

import pytest

from navpy.core.approach.vision_nav.confirmation import (
    GuidanceConfirmation,
    GuidanceConfirmationPorts,
)
from navpy.core.approach.vision_nav.frame_projection import (
    GuidanceFrameProjector,
    GuidanceProjectionConfig,
)
from navpy.core.approach.vision_nav.source_epoch import SourceEpochLedger
from navpy.core.approach.vision_nav.visual_pass import VisualPassDetector
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.tracks import TrackedObject
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from tests.detection_factory import make_detected_poi
from tests.modules.nav.test_approach_acquisition_review import (
    GuidanceLawConfig,
    FixedGuidanceLawConfigProvider,
)

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src" / "navpy"
# The NAV command and completion homes.
NAV_HOMES = ("core/approach/", "plugins/laws/")
# What only the detector, the acquisition gate and the runner's floor release may name.
DETECTOR_MODULES = ("navpy.plugins.detection.rfdetr", "navpy.core.perception.tracking.dock_acquisition_gate",
                    "navpy.app.flow.confirm_record", "navpy.app.perception.rfdetr_profile")
DETECTOR_NAMES = frozenset({"DockAcquisitionGate", "DockAcquisitionSettings", "acquired_id", "min_box_px",
                            "recall_min_box_px", "input_scale", "RfDetrDetector", "print_confirm_record"})


class _Poison:
    """Any read of a poisoned value fails the test."""

    def __getattr__(self, name):
        raise AssertionError(f"NAV path read .{name} of a value it must not use")

    def __bool__(self):
        raise AssertionError("NAV path tested a value it must not use")

    def __iter__(self):
        raise AssertionError("NAV path iterated a value it must not use")

    def __float__(self):
        raise AssertionError("NAV path read a value it must not use")


def _offenders(tree: ast.AST) -> list[str]:
    found = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [node.module or ""] if isinstance(node, ast.ImportFrom) else [a.name for a in node.names]
            found += [n for n in names if n.startswith(DETECTOR_MODULES)]
        elif isinstance(node, ast.Name) and node.id in DETECTOR_NAMES:
            found.append(node.id)
        elif isinstance(node, ast.Attribute) and node.attr in DETECTOR_NAMES:
            found.append(node.attr)
    return found


def test_no_nav_module_names_the_detector_the_gate_or_the_floor_release():
    offenders = []
    for path in SRC.rglob("*.py"):
        relative = path.relative_to(SRC).as_posix()
        if relative.startswith(NAV_HOMES):
            offenders += [f"{relative}: {name}" for name in _offenders(ast.parse(path.read_text(encoding="utf-8")))]
    assert offenders == []


def _confirmation() -> GuidanceConfirmation:
    return GuidanceConfirmation(GuidanceConfirmationPorts(
        lock=threading.RLock(),
        projector=GuidanceFrameProjector(GuidanceProjectionConfig("ZYX", True)),
        epochs=SourceEpochLedger(),
        law=VisionNavLaw(FixedGuidanceLawConfigProvider(GuidanceLawConfig(-55.0, 20.0, 45.0, None, 0.0))),
    ))


def _outcome(poi):
    """The command frame, the law's command, the visual pass's plan and the confirmability of one detection."""
    confirmation = _confirmation()
    frame = confirmation._project(poi)
    assert frame is not None
    plan = confirmation._ports.law.preview(frame)
    passing = VisualPassDetector().plan(frame)
    return (frame, None if plan is None else (plan.command, plan.within_limits), passing,
            confirmation.can_confirm_detection(poi))


def _poi(**overrides):
    values = dict(
        x_error=1010.0, y_error=560.0, uas_att=Attitude(3.0, 0.0, -8.0),
        g_data=GimbalData(att=Attitude(pitch=-12.0, yaw=6.0, roll=0.0), name="zr10"),
        pose_is_frame_atomic=True, timestamp=40.0, uas_body_rates_rad_s=(0.0, 0.01, 0.03),
        source_air_speed_mps=24.0, bbox_cxcywh=(1010.0, 560.0, 33.0, 14.0),
        tracking_bbox_cxcywh=(1010.0, 560.0, 33.0, 14.0), reference_height_m=0.0,
        c_g_loc=Location(40.31, 44.45, 80.0), p_t_g_l=Location(40.312, 44.452, 0.75),
    )
    values.update(overrides)
    return make_detected_poi(**values)


@pytest.mark.parametrize("change", [
    {"bbox_cxcywh": (1010.0, 560.0, 300.0, 140.0), "tracking_bbox_cxcywh": (1010.0, 560.0, 4.0, 400.0)},  # box size
    {"p_t_g_l": Location(40.4, 44.6, 0.75)},                       # the orbit centre (own geolocation)
    {"uas_att": Attitude(3.0, 137.0, -8.0)},                       # aircraft compass heading
    {"c_g_loc": Location(41.0, 45.0, 80.0)},                       # aircraft position
    {"c_g_loc": Location(40.31, 44.45, 400.0), "reference_height_m": 37.0},  # altitude, dock height
])
def test_box_size_orbit_centre_heading_position_and_altitude_change_nothing(change):
    assert _outcome(_poi(**change)) == _outcome(_poi())


def test_poisoned_box_orbit_heading_position_and_altitude_are_never_read():
    clean = _outcome(_poi())
    poisoned = _poi()
    poisoned.capture_confirmation(_Poison())
    poisoned.replace_tracking(_Poison())
    poisoned.pose = _Poison()
    poisoned.geo = _Poison()
    assert _outcome(poisoned) == clean


def test_box_size_through_the_real_mapper_leaves_the_command_frame_unchanged():
    """Upstream of the law: two tracks with one centre and very different boxes give one command frame."""
    from navpy.core.perception.pipeline.real_detection_mapper import DetectedObjectMapper, DetectionMappingConfig
    from navpy.core.perception.pipeline.real_detector_state import ConfirmationFrameStore
    from tests.modules.vision.test_capture_time_association_e2e import BASE_S, _associate, _calibration

    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=_calibration())

    def outcome(w: float, h: float):
        track = TrackedObject(id=7, cx=320.0, cy=240.0, w=w, h=h, confidence=0.9, class_id=0, age=1, hits=1,
                              missed=0, is_confirmed=True, timestamp=exposure, vx=0.0, vy=0.0)
        mapper = DetectedObjectMapper(DetectionMappingConfig(reference_height_m=0.0, output_mode="locked"),
                                      ConfirmationFrameStore(maximum_frames=4))
        poi = mapper.convert([track], track, association, ())[0]
        return poi.visual_detection(), _outcome(poi)

    assert outcome(33.0, 14.0) == outcome(4.0, 400.0) == outcome(300.0, 2.0)


def test_poisoned_capture_location_upstream_leaves_the_command_frame_unchanged():
    """Position and altitude at capture (the ground ray's inputs) never bend the pixel observation."""
    from navpy.core.perception.pipeline.real_detection_mapper import DetectedObjectMapper, DetectionMappingConfig
    from navpy.core.perception.pipeline.real_detector_state import ConfirmationFrameStore
    from tests.modules.vision.test_capture_time_association_e2e import BASE_S, _associate, _calibration

    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=_calibration())
    track = TrackedObject(id=7, cx=320.0, cy=240.0, w=33.0, h=14.0, confidence=0.9, class_id=0, age=1, hits=1,
                          missed=0, is_confirmed=True, timestamp=exposure, vx=0.0, vy=0.0)

    def visual(frame_association):
        mapper = DetectedObjectMapper(DetectionMappingConfig(reference_height_m=0.0, output_mode="locked"),
                                      ConfirmationFrameStore(maximum_frames=4))
        return mapper.convert([track], track, frame_association, ())[0].visual_detection()

    assert visual(replace(association, capture_location=_Poison())) == visual(
        replace(association, capture_location=None))
