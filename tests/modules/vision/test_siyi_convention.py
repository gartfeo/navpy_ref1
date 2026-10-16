"""The SIYI gimbal's angle convention end to end: wire report or sim plant -> GimbalData -> camera ray in the body.

The real gimbal is yaw-outer (camera-to-body = Rz(yaw) Ry(pitch) Rx(roll)) and its wire yaw is left-positive.
At yaw 90 / pitch -45 the optical axis points right and 45 deg down; a roll-outer reading would make it horizontal.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from navpy.app.perception.vision_gimbal_profile import build_gimbal_data, resolve_gimbal_seq
from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.visual_ray_projection import camera_to_body_from_gimbal
from navpy.core.ports.gimbal_abc import GimbalData, GimbalMountSetup
from navpy.plugins.gimbal.siyi.hardware.state import SiyiReadbackStore
from navpy.plugins.gimbal.siyi.siyi_convention import SIYI_GIMBAL_SEQ, yaw_from_wire, yaw_to_wire
from navpy.sim.gimbal.siyi.gimbal_angles import MODE_FOLLOW, GimbalAngularPlant
from navpy.sim.gimbal.siyi.gimbal_frame_transform import body_readback, matrix_to_zyx, zyx_to_matrix

_SETUP = GimbalMountSetup(att=Attitude(90.0, 0.0, 90.0), seq="XYZ")    # the siyi_zr10 profile's camera mount


def _optical_axis(att: Attitude) -> np.ndarray:
    """The camera's optical axis in the aircraft body frame (FRD) for a SIYI gimbal at ``att``."""
    data = GimbalData(att=att, setup=_SETUP, g_seq=SIYI_GIMBAL_SEQ)
    return camera_to_body_from_gimbal(data).matrix() @ np.array([0.0, 0.0, 1.0])


def test_level_gimbal_looks_forward():
    assert np.allclose(_optical_axis(Attitude(0.0, 0.0, 0.0)), [1.0, 0.0, 0.0], atol=1e-9)


def test_yaw_90_pitch_down_45_looks_right_and_45_degrees_down():
    axis = _optical_axis(Attitude(-45.0, 90.0, 0.0))
    s = math.sqrt(0.5)
    assert np.allclose(axis, [0.0, s, s], atol=1e-9)        # right (+y) and down (+z) in FRD


def test_wire_report_reaches_the_ray_with_its_yaw_negated():
    store = SiyiReadbackStore(GimbalData(att=Attitude(0.0, 0.0, 0.0), setup=_SETUP, g_seq=SIYI_GIMBAL_SEQ))
    assert store.accept_attitude((1, 10.0, -90.0, -45.0, 0.0))     # wire: yaw 90 right reads -90, pitch 45 down
    data = store.snapshot().data
    assert data.att == Attitude(-45.0, 90.0, 0.0)
    s = math.sqrt(0.5)
    assert np.allclose(camera_to_body_from_gimbal(data).matrix() @ [0.0, 0.0, 1.0], [0.0, s, s], atol=1e-9)


def test_wire_yaw_is_left_positive_both_ways():
    assert yaw_to_wire(30.0) == -30.0
    assert yaw_from_wire(-30.0) == 30.0


@pytest.mark.parametrize("vehicle", [Attitude(0.0, 0.0, 0.0), Attitude(8.0, 40.0, 25.0), Attitude(-12.0, -150.0, -35.0)])
def test_sim_lock_readback_composes_with_the_aircraft_to_the_held_world_pose(vehicle):
    plant = GimbalAngularPlant(-45.0, 90.0)
    plant.set_vehicle_attitude(vehicle)
    body = body_readback(plant.snapshot())
    world = zyx_to_matrix(vehicle.yaw, vehicle.pitch, vehicle.roll) @ zyx_to_matrix(body.yaw, body.pitch, body.roll)
    assert np.allclose(world, zyx_to_matrix(90.0, -45.0, 0.0), atol=1e-9)


def test_sim_follow_readback_level_aircraft_is_the_plant_angles():
    plant = GimbalAngularPlant(-45.0, 0.0)
    plant.set_motion_mode(MODE_FOLLOW)
    plant.set_target_angles(90.0, -45.0)
    plant.advance(2.0)
    body = body_readback(plant.snapshot())
    assert (body.yaw, body.pitch, body.roll) == pytest.approx((90.0, -45.0, 0.0), abs=1e-9)


def test_matrix_to_zyx_inverts_zyx_to_matrix():
    for angles in [(90.0, -45.0, 0.0), (-120.0, 20.0, 15.0), (10.0, -80.0, -30.0)]:
        assert matrix_to_zyx(zyx_to_matrix(*angles)) == pytest.approx(angles, abs=1e-9)


def test_siyi_profile_gimbal_defaults_to_and_requires_the_yaw_outer_sequence():
    assert resolve_gimbal_seq({"type": "siyi"}) == "ZYX"
    assert resolve_gimbal_seq({"type": "siyi", "seq": "ZYX"}) == "ZYX"
    for wrong in ("XYZ", "zyx"):
        with pytest.raises(ValueError, match="siyi"):
            resolve_gimbal_seq({"type": "siyi", "seq": wrong})
    assert resolve_gimbal_seq({"seq": "XYZ"}) == "XYZ"      # other gimbals keep their profile's sequence
    assert build_gimbal_data({"gimbal": {"type": "siyi"}}, 0).g_seq == "ZYX"
