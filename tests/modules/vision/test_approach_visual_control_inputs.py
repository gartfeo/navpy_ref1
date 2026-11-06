"""Input boundary of the gimbal feedforward (HITL step 24): no heading, no truth."""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import numpy as np
import pytest

from navpy.app.perception.vision_tracking_profile import build_tracking_profile
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.perception.pointing import (
    camera_motion_state,
    compensated_los_rate,
    gimbal_rate_command,
    gimbal_rate_tracker,
    gimbal_tracking_sample,
)
from navpy.core.perception.pointing.compensated_los_rate import (
    CameraFeedforwardConfig,
)
from navpy.core.perception.pointing.gimbal_tracking_sample import GimbalPoiProjector
from navpy.core.perception.profile import vision_profile_loader
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.pixel_observation import PixelProjectionKind
from tests.detection_factory import make_detected_poi

_FORBIDDEN = {
    # compass heading and anything derived from it
    "heading", "yaw", "aircraft_attitude", "uas_att", "reference_aircraft_attitude",
    # body rates are read only as the frame's converted gyro turn rate
    "body_rates_rad_s", "uas_body_rates_rad_s",
    # ground speed, position, altitude, range, truth
    "velocity", "ground_speed", "ground_speed_ned", "location", "altitude",
    "relative_altitude", "reference_height_m", "geo", "camera_location",
    "projected_poi_location", "truth_poi_location", "t_g_loc_debug", "c_g_loc",
    "is_simulation", "bbox_cxcywh", "tracking_bbox_cxcywh", "pose", "gimbal_data",
}


def _names(module) -> set[str]:
    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            found.add(node.id)
        elif isinstance(node, ast.Attribute):
            found.add(node.attr)
        elif isinstance(node, ast.keyword) and node.arg is not None:
            if not _is_zero_yaw(node):  # the level frame writes yaw=0.0
                found.add(node.arg)
        elif isinstance(node, ast.Call) and len(node.args) >= 2:
            key = node.args[1]
            if isinstance(key, ast.Constant) and isinstance(key.value, str):
                found.add(key.value)
    return found


def _is_zero_yaw(node: ast.keyword) -> bool:
    value = node.value
    return node.arg == "yaw" and isinstance(value, ast.Constant) and value.value == 0.0


@pytest.mark.parametrize(
    "module",
    [
        camera_motion_state,
        compensated_los_rate,
        gimbal_rate_command,
        gimbal_rate_tracker,
        gimbal_tracking_sample,
    ],
)
def test_feedforward_modules_read_no_heading_ground_speed_altitude_or_truth(module):
    assert not (_FORBIDDEN & _names(module)), module.__name__


def test_level_frame_uses_zero_yaw():
    source = inspect.getsource(compensated_los_rate.level_angles)
    assert "yaw=0.0" in source


def _poi(**kwargs):
    defaults = dict(
        x_error=700.0,
        y_error=300.0,
        k=np.array([[1400.0, 0.0, 640.0], [0.0, 1400.0, 360.0], [0.0, 0.0, 1.0]]),
        g_data=GimbalData(att=Attitude(-12.0, 25.0, 0.0), name="zr10"),
        uas_att=Attitude(3.0, 40.0, 18.0),
        pose_is_frame_atomic=True,
        uas_body_rates_rad_s=(0.01, 0.02, 0.12),
    )
    defaults.update(kwargs)
    return make_detected_poi(**defaults)


def test_projector_carries_the_frame_camera_state():
    sample = GimbalPoiProjector.project(_poi(), 5.0)

    assert sample.camera is not None
    assert sample.camera.aircraft_pitch_deg == 3.0
    assert sample.camera.aircraft_roll_deg == 18.0
    assert sample.camera.aircraft_turn_rate_rad_s == pytest.approx(
        (0.02 * np.sin(np.radians(18.0)) + 0.12 * np.cos(np.radians(18.0)))
        / np.cos(np.radians(3.0))
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"pose_is_frame_atomic": None},
        {"pose_is_frame_atomic": False},
        {"projection": PixelProjectionKind.SPHERICAL_EQUIANGULAR},
    ],
)
def test_projector_gives_no_camera_state_without_an_associated_pinhole_frame(kwargs):
    assert GimbalPoiProjector.project(_poi(**kwargs), 5.0).camera is None


def test_projector_ignores_heading_and_truth():
    """Compass yaw, geo and sim truth poisoned: the same sample."""
    plain = GimbalPoiProjector.project(_poi(), 5.0)
    poisoned = GimbalPoiProjector.project(
        _poi(
            uas_att=Attitude(3.0, -170.0, 18.0),
            c_g_loc=Location(1.0, 2.0, 300.0),
            t_g_loc_debug=Location(3.0, 4.0, 5.0),
            reference_height_m=99.0,
            is_simulation=True,
        ),
        5.0,
    )

    assert poisoned == plain


def _device(profile: str) -> dict:
    class _Log:
        def __getattr__(self, name):
            return lambda *args, **kwargs: None

    resolved = vision_profile_loader.resolve_profile(profile, _Log())[1]
    return next(d for d in resolved["devices"] if d.get("name") == "siyi_zr10")


def test_only_the_hitl_profile_turns_the_feedforward_on():
    hitl = build_tracking_profile(_device("siyi_zr10_hitl_encoded"), sim=False)
    base = build_tracking_profile(_device("siyi_zr10"), sim=False)

    assert hitl.rate.camera_feedforward == CameraFeedforwardConfig()
    assert base.rate.camera_feedforward is None


def test_profile_refuses_a_non_boolean_feedforward():
    device = {"gimbal": {"tracking": {"enabled": True, "camera_feedforward": "yes"}}}
    with pytest.raises(ValueError):
        build_tracking_profile(device, sim=False)
