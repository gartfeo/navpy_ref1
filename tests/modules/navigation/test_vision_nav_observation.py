import math
from dataclasses import FrozenInstanceError, fields, replace

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.modules.vision.models.pixel_observation import PixelProjectionKind
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.visual_ray_projection import body_ray_to_pixel
from tests.detection_factory import make_detected_poi


def _poi(**changes):
    body_ray = changes.pop("body_ray", None)
    values = dict(
        obj_id=2,
        task_id=1,
        x_error=320.0,
        y_error=240.0,
        reference_height_m=999.0,
        k=np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]),
        g_data=GimbalData(att=Attitude(0.0, 0.0, 0.0), name="cam"),
        uas_att=Attitude(-5.0, 77.0, 3.0),
        timestamp=4.0,
        pose_is_frame_atomic=True,
    )
    values.update(changes)
    poi = make_detected_poi(**values)
    if body_ray is not None:
        u_px, v_px = body_ray_to_pixel(
            body_ray,
            poi.pixel.calibration,
            poi.pixel.camera_to_body,
            PixelProjectionKind.SPHERICAL_EQUIANGULAR,
        )
        poi.replace_pixel(replace(
            poi.pixel,
            u_px=u_px,
            v_px=v_px,
            projection=PixelProjectionKind.SPHERICAL_EQUIANGULAR,
        ))
    return poi.visual_detection()


def _projector():
    return FinalApproachFrameProjector(FinalApproachProjectionConfig("ZYX", True))


def test_final_approach_frame_has_exact_primitive_schema_and_is_frozen_slotted():
    frame = _projector().project(_poi(), 3)
    assert frame is not None
    assert [field.name for field in fields(frame)] == [
        "source_name", "source_generation", "task_id", "obj_id",
        "source_timestamp_s", "body_x", "body_y", "body_z",
        "control_x", "control_y", "control_z", "aircraft_roll_deg",
        "air_speed_mps", "aircraft_yaw_rate_rad_s", "aircraft_pitch_deg",
    ]
    assert not hasattr(frame, "__dict__")
    with pytest.raises(FrozenInstanceError):
        frame.body_x = 2.0


@pytest.mark.parametrize("value", [True, 1.0, "1"])
def test_final_approach_frame_rejects_non_exact_integer_identity(value):
    with pytest.raises(ValueError):
        FinalApproachVisionFrame(
            "cam", value, 1, 2, 1.0,
            1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        )


def test_projector_rejects_any_pose_not_explicitly_atomic():
    assert _projector().project(_poi(pose_is_frame_atomic=None), 0) is None
    assert _projector().project(_poi(pose_is_frame_atomic=False), 0) is None


def test_spherical_pixels_accept_negative_forward_and_ignore_compass_yaw():
    class PoisonAttitude:
        pitch = -5.0
        roll = 2.0

        @property
        def yaw(self):
            raise AssertionError("aircraft compass yaw was read")

    frame = _projector().project(_poi(
        uas_att=PoisonAttitude(),
        body_ray=np.array([-1.0, 0.0, 0.0]),
        uas_body_rates_rad_s=(0.0, 0.0, 0.0),
    ), 0)

    assert frame is not None
    assert frame.body_x == -1.0
    assert math.isclose(sum(value * value for value in frame.control_ray), 1.0)


def test_pixel_projection_accepts_arbitrary_unbounded_pixels():
    frame = _projector().project(_poi(
        x_error=1.0e12,
        y_error=-1.0e12,
    ), 0)

    assert frame is not None
    assert math.isclose(sum(value * value for value in frame.body_ray), 1.0)


def test_missing_source_or_pitch_roll_fails_closed():
    missing_source = _poi()
    missing_source = replace(
        missing_source,
        observation=replace(missing_source.observation, source_name=""),
    )
    bad_attitude = _poi()
    bad_attitude = replace(
        bad_attitude,
        observation=replace(bad_attitude.observation, aircraft_pitch_deg=math.nan),
    )
    assert _projector().project(missing_source, 0) is None
    assert _projector().project(bad_attitude, 0) is None


def test_projection_signs_positive_body_y_and_z_for_level_aircraft():
    ray = np.array([1.0, 0.2, 0.3])
    frame = _projector().project(_poi(
        uas_att=Attitude(0.0, 999.0, 0.0),
        body_ray=ray,
    ), 0)
    assert frame is not None
    assert frame.control_y > 0.0
    assert frame.control_z > 0.0


def test_nonzero_pitch_roll_transform_matches_yaw_zero_rotation():
    body = np.array([0.9, 0.2, 0.3])
    frame = _projector().project(_poi(
        uas_att=Attitude(-12.0, -177.0, 8.0),
        body_ray=body,
    ), 0)
    assert frame is not None
    expected = _projector().project(_poi(
        uas_att=Attitude(-12.0, 45.0, 8.0),
        body_ray=body,
    ), 0)
    assert expected is not None
    assert frame.control_ray == pytest.approx(expected.control_ray)
