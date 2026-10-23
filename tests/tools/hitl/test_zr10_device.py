"""The emulated ZR10 without sockets: wire layouts, signs, conventions against navpy's sim plant, effects."""
from __future__ import annotations

import math
import struct

import numpy as np
import pytest

from navpy.core.kernel.attitude import Attitude
from navpy.sim.gimbal.siyi.gimbal_angles import MODE_FOLLOW, MODE_LOCK, GimbalAngularLimits, GimbalAngularPlant
from navpy.sim.gimbal.siyi.gimbal_frame_transform import zyx_to_matrix
from tests.tools.hitl.zr10_frames import crc16_xmodem, frame, parse
from tools.hitl.zr10_config import STEP_S, EmulatorConfig, load_zoom_table, zoom_table_from_profile
from tools.hitl.zr10_device import FIRMWARE_HEX, Zr10Device

SOURCE = ("127.0.0.1", 50000)
LEVEL = Attitude(0.0, 0.0, 0.0)


@pytest.fixture(scope="module")
def zoom_table():
    return load_zoom_table("siyi_zr10")


def make(zoom_table, attitude=LEVEL, **config):
    device = Zr10Device(EmulatorConfig(**config), zoom_table)
    if attitude is not None:
        device.set_vehicle_attitude(attitude)
    return device


def ask(device, cmd, data=b""):
    return [parse(raw) for raw in device.handle_datagram(frame(cmd, data), SOURCE)]


def run(device, seconds, attitude_at=None):
    for _ in range(round(seconds / STEP_S)):
        if attitude_at is not None:
            device.set_vehicle_attitude(attitude_at(device.now_s))
        device.step()


def wire_attitude(device):
    (reply,) = ask(device, 0x0D)
    return [v / 10.0 for v in struct.unpack("<6h", reply.data)]


def set_angles(device, yaw, pitch):
    return ask(device, 0x0E, struct.pack("<2h", round(yaw * 10), round(pitch * 10)))


def zoom_reading(device):
    (reply,) = ask(device, 0x18)
    return reply.data[0] + reply.data[1] / 10.0


# -- framing ---------------------------------------------------------------------------------------------------
def test_replies_are_device_frames_with_rising_sequence(zoom_table):
    device = make(zoom_table)
    first, = ask(device, 0x01)
    second, = ask(device, 0x01)
    assert (first.ctrl, first.cmd, first.data.hex()) == (0x02, 0x01, FIRMWARE_HEX)
    assert (first.seq, second.seq) == (1, 2)
    assert crc16_xmodem(b"123456789") == 0x31C3     # the CRC used by the test helper is CRC-16/XMODEM


def test_hardware_id_starts_with_zr10_code(zoom_table):
    (reply,) = ask(make(zoom_table), 0x02)
    assert reply.data == b"6B02234248\x00\x00"


def test_nothing_is_answered_before_the_first_aircraft_attitude(zoom_table):
    device = make(zoom_table, attitude=None)
    assert device.handle_datagram(frame(0x01), SOURCE) == []
    device.step()
    assert device.true_state() is None and device.counters.before_attitude == 1
    device.set_vehicle_attitude(LEVEL)
    assert len(ask(device, 0x01)) == 1


def test_bad_crc_unknown_command_and_two_frames_in_one_datagram(zoom_table):
    device = make(zoom_table)
    broken = bytearray(frame(0x01)); broken[-1] ^= 0xFF
    assert device.handle_datagram(bytes(broken), SOURCE) == []
    assert device.handle_datagram(frame(0x7E), SOURCE) == []
    replies = device.handle_datagram(b"\x00\x01" + frame(0x01) + frame(0x18), SOURCE)
    assert [parse(r).cmd for r in replies] == [0x01, 0x18]
    assert device.counters.bad_frames == 2 and device.counters.unknown_commands == 1


def test_attitude_and_set_angles_reply_layouts(zoom_table):
    device = make(zoom_table)
    (reply,) = set_angles(device, 10.0, -5.0)
    assert len(reply.data) == 6 and struct.unpack("<3h", reply.data) == (0, 0, 0)    # the angles before the move
    run(device, 0.5)
    yaw, pitch, roll, *_rates = wire_attitude(device)
    assert (yaw, pitch, roll) == (10.0, -5.0, 0.0)


# -- signs and conventions ------------------------------------------------------------------------------------
def test_wire_yaw_is_left_positive_and_unreal_yaw_right_positive(zoom_table):
    device = make(zoom_table)
    set_angles(device, 30.0, -20.0)
    run(device, 1.0)
    state = device.true_state()
    assert (state.yaw, state.pitch) == pytest.approx((-30.0, -20.0))
    assert wire_attitude(device)[:2] == [30.0, -20.0]


def test_positive_rate_turns_right_lowers_the_yaw_report_and_raises_pitch(zoom_table):
    device = make(zoom_table)
    (ack,) = ask(device, 0x07, struct.pack("<2b", 50, 50))
    assert ack.data == b"\x01"
    run(device, 0.2)
    state = device.true_state()
    assert state.yaw == pytest.approx(9.0) and state.pitch == pytest.approx(9.0)    # 45 deg/s for 0.2 s
    yaw, pitch, *_ = wire_attitude(device)
    assert yaw == pytest.approx(-9.0) and pitch == pytest.approx(9.0)
    assert wire_attitude(device)[3] == pytest.approx(-45.0)        # the reported yaw rate falls too


def _vehicle(roll, pitch, yaw):
    return zyx_to_matrix(yaw, pitch, roll)


@pytest.mark.parametrize("name, attitude_at", [
    ("level", lambda t: Attitude(0.0, 0.0, 0.0)),
    ("banked", lambda t: Attitude(5.0, 40.0, 30.0)),
    ("turning", lambda t: Attitude(3.0, 10.0 + 15.0 * t, 25.0)),
])
def test_rendered_camera_matches_navpy_sim_plant_world_rotation(zoom_table, name, attitude_at):
    """Unreal composes vehicle x FRotator(body angles): that must give navpy's sim plant's world camera rotation."""
    device = make(zoom_table, attitude=attitude_at(0.0))
    config = EmulatorConfig()
    plant = GimbalAngularPlant(attitude_at(0.0).pitch, attitude_at(0.0).yaw, GimbalAngularLimits(
        config.slew_rate_deg_s, config.yaw_min_deg, config.yaw_max_deg, config.pitch_min_deg,
        config.pitch_max_deg))
    plant.set_vehicle_attitude(attitude_at(0.0))
    set_angles(device, -50.0, -30.0)
    plant.set_target_angles(50.0, -30.0)          # the plant takes navpy's (right-positive) yaw
    for _ in range(100):
        attitude = attitude_at(device.now_s)
        device.set_vehicle_attitude(attitude)
        plant.set_vehicle_attitude(attitude)
        device.step()
        plant.advance(STEP_S)
        snapshot = plant.snapshot()
        state = device.true_state()
        rendered = _vehicle(attitude.roll, attitude.pitch, attitude.yaw) @ zyx_to_matrix(state.yaw, state.pitch,
                                                                                         state.roll)
        world = zyx_to_matrix(snapshot.yaw, snapshot.pitch, 0.0)
        np.testing.assert_allclose(rendered, world, atol=1e-9, err_msg=name)
        yaw, pitch, roll = wire_attitude(device)[:3]
        assert (yaw, pitch, roll) == pytest.approx((-state.yaw, state.pitch, state.roll), abs=0.051)
    assert plant.snapshot().yaw == pytest.approx(50.0)        # lock mode holds the world yaw through the turn


def test_yaw_outer_decomposition_points_right_and_down(zoom_table):
    """Body yaw 90 then pitch -45: the optical axis looks right and 45 deg down (the rig's yaw-outer chain)."""
    device = make(zoom_table)
    set_angles(device, -90.0, -45.0)
    run(device, 2.0)
    state = device.true_state()
    axis = zyx_to_matrix(state.yaw, state.pitch, state.roll) @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(axis, [0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-9)
    assert wire_attitude(device)[:3] == [-90.0, -45.0, 0.0]     # right is negative on the wire


def test_center_returns_to_the_body_centre_in_lock_mode(zoom_table):
    device = make(zoom_table, attitude=Attitude(0.0, 90.0, 0.0))
    set_angles(device, -100.0, -40.0)
    run(device, 2.0)
    (reply,) = ask(device, 0x08, b"\x01")
    assert reply.data == b"\x01"
    run(device, 2.0)
    state = device.true_state()
    assert (state.yaw, state.pitch) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_body_limits_are_the_rig_sweep(zoom_table):
    device = make(zoom_table)
    ask(device, 0x07, struct.pack("<2b", 100, 100))
    run(device, 3.0)
    assert (device.true_state().yaw, device.true_state().pitch) == pytest.approx((132.8, 23.9))
    ask(device, 0x07, struct.pack("<2b", -100, -100))
    run(device, 4.0)
    assert (device.true_state().yaw, device.true_state().pitch) == pytest.approx((-135.0, -88.9))
    assert device.counters.stop_steps == 0 and device.invalid_reasons() == []


def test_a_banked_camera_stops_at_the_body_pitch_stop_and_returns_when_level(zoom_table):
    # Banked 45 deg right, lock mode holding the camera level and to the right: in the body frame that is pitch
    # +45 deg, past the +23.9 deg stop. navpy's plant clamps world-angle differences and asks for it anyway; the
    # camera stops at the stop (rendered and reported), and the run stays valid.
    device = make(zoom_table, attitude=Attitude(0.0, 0.0, 45.0))
    set_angles(device, -90.0, 0.0)                  # wire yaw left-positive: -90 is 90 deg right
    run(device, 1.5)
    state = device.true_state()
    assert (state.yaw, state.pitch) == pytest.approx((90.0, 23.9), abs=0.01)
    assert wire_attitude(device)[:2] == pytest.approx([-90.0, 23.9], abs=0.051)
    assert device.counters.stop_steps > 0 and device.invalid_reasons() == []
    stopped = device.counters.stop_steps
    device.set_vehicle_attitude(LEVEL)              # the aircraft levels: the lock target was kept
    run(device, 0.2)
    state = device.true_state()
    assert (state.yaw, state.pitch, state.roll) == pytest.approx((90.0, 0.0, 0.0), abs=0.01)
    assert device.counters.stop_steps == stopped


def test_a_banked_camera_at_the_yaw_stop_is_held_there(zoom_table):
    # Heading 0, banked 30 deg left; lock target 130 deg right, 10 deg down. The world yaw difference (130) is
    # inside the limits, so navpy's plant allows it, but the body yaw it needs is 138.2 deg, past the +132.8 stop.
    device = make(zoom_table, attitude=Attitude(0.0, 0.0, -30.0))
    set_angles(device, -130.0, -10.0)
    run(device, 2.0)
    assert device.gimbal.at_stop() and device.counters.stop_steps > 0
    state = device.true_state()
    assert state.yaw == pytest.approx(132.8)
    assert wire_attitude(device)[0] == pytest.approx(-132.8, abs=0.051)


def test_motion_mode_function_and_config_reply(zoom_table):
    device = make(zoom_table)
    assert ask(device, 0x0C, b"\x04") == []
    (info,) = ask(device, 0x0A)
    assert info.data == bytes((0, 0, 0, 2, MODE_FOLLOW, 1)) and device.true_state().motion_mode == MODE_FOLLOW
    assert ask(device, 0x0C, b"\x00") == [] and device.counters.rejected_commands == 1
    assert device.invalid_reasons() == ["1 rejected SIYI command(s)"]


def test_fpv_is_refused_and_follow_does_not_jump_the_camera(zoom_table):
    # navpy's plant keeps the world pitch on entering FPV: pitched up 20 deg, the camera would jump 20 deg.
    device = make(zoom_table, attitude=Attitude(20.0, 30.0, 0.0))
    set_angles(device, -60.0, -10.0)
    run(device, 1.0)
    before = device.true_state()
    assert ask(device, 0x0C, b"\x05") == [] and device.counters.rejected_commands == 1
    assert device.true_state() == before and before.motion_mode == MODE_LOCK
    ask(device, 0x0C, b"\x04")
    after = device.true_state()
    assert after.motion_mode == MODE_FOLLOW
    assert (after.yaw, after.pitch, after.roll) == pytest.approx((before.yaw, before.pitch, before.roll), abs=1e-6)


# -- ordering and effects --------------------------------------------------------------------------------------
def test_a_command_never_acts_on_time_before_it_arrived(zoom_table):
    device = make(zoom_table)
    device.step()
    device.advance_to(device.now_s + 0.01)
    ask(device, 0x07, struct.pack("<2b", 100, 0))
    device.step()
    assert device.true_state().yaw == pytest.approx(0.9)       # 90 deg/s for the 10 ms left in the step


def test_rate_dead_time_delays_moves_and_stops_and_angle_commands_cancel(zoom_table):
    device = make(zoom_table, rate_dead_time_s=0.6)
    ask(device, 0x07, struct.pack("<2b", 100, 0))
    run(device, 0.6)
    assert device.true_state().yaw == pytest.approx(0.0)
    run(device, 0.2)
    assert device.true_state().yaw == pytest.approx(18.0)
    ask(device, 0x07, struct.pack("<2b", 0, 0))
    run(device, 0.6)
    assert device.true_state().yaw == pytest.approx(72.0)     # the stop waits its dead time too
    run(device, 0.2)
    assert device.true_state().yaw == pytest.approx(72.0)
    ask(device, 0x07, struct.pack("<2b", -100, 0))
    set_angles(device, -72.0, 0.0)          # the plant is already there: the waiting rate must not restart it
    run(device, 1.0)
    assert device.true_state().yaw == pytest.approx(72.0)


def test_rate_deadband(zoom_table):
    device = make(zoom_table, rate_deadband=4)
    ask(device, 0x07, struct.pack("<2b", 4, -4))
    run(device, 0.5)
    assert (device.true_state().yaw, device.true_state().pitch) == (0.0, 0.0)
    ask(device, 0x07, struct.pack("<2b", 5, 0))
    run(device, 1.0)
    assert device.true_state().yaw == pytest.approx(4.5)


def test_attitude_and_zoom_report_lags_are_separate(zoom_table):
    device = make(zoom_table, attitude_report_lag_s=0.1, zoom_report_lag_s=0.2)
    ask(device, 0x07, struct.pack("<2b", 100, 0))
    run(device, 0.5)
    assert device.true_state().yaw == pytest.approx(45.0)
    assert wire_attitude(device)[0] == pytest.approx(-36.0)        # the angle 0.1 s ago
    ask(device, 0x0F, bytes((5, 5)))
    device.step()
    assert device.true_state().zoom == 5.5
    run(device, 0.12)
    assert zoom_reading(device) == 1.1                              # the jump was 0.12 s ago: not reported yet
    run(device, 0.1)
    assert zoom_reading(device) == 5.5


def test_report_scale(zoom_table):
    device = make(zoom_table, yaw_report_scale=0.93, pitch_report_scale=0.995)
    set_angles(device, -40.0, -60.0)
    run(device, 1.0)
    yaw, pitch, *_ = wire_attitude(device)
    assert (yaw, pitch) == pytest.approx((0.93 * -40.0, 0.995 * -60.0), abs=0.051)
    assert (device.true_state().yaw, device.true_state().pitch) == pytest.approx((40.0, -60.0))


# -- zoom ------------------------------------------------------------------------------------------------------
def test_absolute_zoom_follows_the_profile_table(zoom_table):
    device = make(zoom_table)
    assert zoom_reading(device) == 1.1                  # command 1.0 reads 1.1 (navpy's start-up zoom)
    for command, reading in ((20, 1.8), (22, 2.2), (23, 2.1), (100, 10.0)):
        (ack,) = ask(device, 0x0F, bytes(divmod(command, 10)))
        assert ack.data == b"\x01"
        device.step()
        assert zoom_reading(device) == reading == device.true_state().zoom


def test_absolute_zoom_outside_the_table_is_refused(zoom_table):
    device = make(zoom_table)
    (ack,) = ask(device, 0x0F, bytes((15, 0)))
    assert ack.data == b"\x00"
    assert ask(device, 0x0F, bytes((2, 12))) == [] and device.counters.rejected_commands == 1
    (center,) = ask(device, 0x08, b"\x00")
    assert center.data == b"\x00"
    assert device.invalid_reasons() == ["1 rejected SIYI command(s)", "2 refused SIYI command(s)"]
    run(device, 0.5)
    assert zoom_reading(device) == 1.1


def test_lens_travel_moves_in_tenth_steps_that_are_rendered_as_reported(zoom_table):
    device = make(zoom_table, absolute_zoom_speed_per_s=2.0)
    ask(device, 0x0F, bytes((5, 5)))
    seen = []
    for _ in range(150):
        device.step()
        reading = zoom_reading(device)
        assert reading == device.true_state().zoom and round(reading * 10) == reading * 10
        seen.append(reading)
    assert seen[0] < 1.3 and seen[-1] == 5.5 and seen == sorted(seen)


def test_manual_zoom_speed_stop_and_range(zoom_table):
    device = make(zoom_table)
    (reply,) = ask(device, 0x05, b"\x01")
    assert struct.unpack("<H", reply.data) == (11,)
    run(device, 1.0)
    assert zoom_reading(device) == 1.6                  # 0.5 x/s
    ask(device, 0x05, b"\x00")
    run(device, 1.0)
    assert zoom_reading(device) == 1.6
    ask(device, 0x05, b"\xff")
    run(device, 3.0)
    assert zoom_reading(device) == 1.1                  # the table's lowest reading


def lens(device):
    state = device.true_state()
    return state.zoom, state.lens_command, state.lens_settled


def test_lens_setting_starts_at_the_start_up_command_and_follows_absolute_zoom(zoom_table):
    device = make(zoom_table)
    assert lens(device) == (1.1, 1.0, True)
    ask(device, 0x0F, bytes((2, 3)))
    device.step()
    assert lens(device) == (2.1, 2.3, True)            # command 2.3 settles at reading 2.1
    ask(device, 0x0F, bytes((15, 0)))                  # refused: the lens and its setting stay
    run(device, 0.2)
    assert lens(device) == (2.1, 2.3, True)


def test_lens_is_unsettled_while_it_travels_even_within_one_reading(zoom_table):
    device = make(zoom_table, absolute_zoom_speed_per_s=2.0)
    ask(device, 0x0F, bytes((1, 2)))                   # 1.2 reads 1.1, as the start: the reading cannot show it
    assert lens(device) == (1.1, 1.2, False)           # unsettled until the plant has stepped to the target
    device.step()
    assert lens(device) == (1.1, 1.2, True)
    ask(device, 0x0F, bytes((2, 0)))
    device.step()
    assert lens(device)[1:] == (2.0, False)
    seen = []
    for _ in range(100):
        device.step()
        seen.append(lens(device))
    settled_at = next(i for i, (_reading, _command, settled) in enumerate(seen) if settled)
    assert all(not s for _r, _c, s in seen[:settled_at]) and all(s for _r, _c, s in seen[settled_at:])
    assert seen[settled_at] == (1.8, 2.0, True) and seen[settled_at - 1][0] == 1.8   # last 0.1 of travel at 1.8


def test_manual_zoom_leaves_no_known_lens_setting(zoom_table):
    device = make(zoom_table)
    ask(device, 0x05, b"\x01")
    device.step()
    assert lens(device)[1:] == (None, False)
    ask(device, 0x05, b"\x00")
    device.step()
    assert lens(device)[1:] == (None, True)
    ask(device, 0x0F, bytes((2, 0)))
    device.step()
    assert lens(device) == (1.8, 2.0, True)


def test_zoom_table_parsing_refuses_made_up_rows():
    profile = {"devices": [{"camera": {"zoom_calibration": {"1.0": 1.1, "2.0": 1.8}}}]}
    assert zoom_table_from_profile(profile) == {10: 1.1, 20: 1.8}
    for bad in ({"1.05": 1.1, "2.0": 1.8}, {"1.0": 1.15, "2.0": 1.8}, {"1.0": 1.1}):
        with pytest.raises(ValueError):
            zoom_table_from_profile({"devices": [{"camera": {"zoom_calibration": bad}}]})
    with pytest.raises(ValueError):
        zoom_table_from_profile({"devices": []})


# -- 0x25 stream -----------------------------------------------------------------------------------------------
def test_attitude_stream_at_the_asked_rate_and_stop(zoom_table):
    device = make(zoom_table)
    (reply,) = ask(device, 0x25, bytes((1, 4)))      # attitude at 10 Hz
    assert reply.data == b"\x01"
    pushes = []
    for _ in range(50):
        pushes += device.step()
    assert len(pushes) == 10 and all(address == SOURCE and parse(raw).cmd == 0x0D for raw, address in pushes)
    ask(device, 0x25, bytes((1, 0)))
    assert sum(len(device.step()) for _ in range(50)) == 0
    assert ask(device, 0x25, bytes((2, 4))) == []    # laser: not on a ZR10
    assert ask(device, 0x25, bytes((1, 7))) == []    # 100 Hz: faster than the 20 ms step, refused
    assert device.counters.rejected_commands == 2


# -- 0x0D push (the real unit: one request starts a ~50 Hz push) ------------------------------------------------
def test_one_attitude_request_starts_a_50_hz_push_to_the_requester(zoom_table):
    device = make(zoom_table)
    assert sum(len(device.step()) for _ in range(10)) == 0          # silent until addressed
    (reply,) = ask(device, 0x0D)                                     # the request still gets its reply
    assert reply.cmd == 0x0D and len(reply.data) == 12
    pushes = []
    for _ in range(50):
        pushes += device.step()
    assert len(pushes) == 50 and all(address == SOURCE and parse(raw).cmd == 0x0D for raw, address in pushes)
    ask(device, 0x0D)                                                # a further request keeps the same push
    assert sum(len(device.step()) for _ in range(50)) == 50
    other = ("127.0.0.1", 50001)
    device.handle_datagram(frame(0x0D), other)
    assert {address for _raw, address in device.step()} == {SOURCE, other}


def test_a_pushed_frame_carries_the_attitude_reported_at_that_step(zoom_table):
    device = make(zoom_table)
    ask(device, 0x0D)
    ask(device, 0x07, struct.pack("<2b", 100, 0))
    ((raw, _address),) = device.step()
    yaw = struct.unpack("<6h", parse(raw).data)[0] / 10.0
    assert yaw == pytest.approx(-device.true_state().yaw, abs=0.051)    # wire yaw left-positive


def test_request_reply_only_mode_pushes_nothing(zoom_table):
    device = make(zoom_table, attitude_push_hz=0.0)
    ask(device, 0x0D)
    assert sum(len(device.step()) for _ in range(50)) == 0
    assert EmulatorConfig(attitude_push_hz=0.0).non_ideal() == {"attitude_push_hz": 0.0}


def test_push_and_0x25_stream_send_one_frame_per_step(zoom_table):
    device = make(zoom_table)
    ask(device, 0x0D)
    ask(device, 0x25, bytes((1, 6)))                                 # 50 Hz as well
    assert all(len(device.step()) == 1 for _ in range(20))


def test_config_validation():
    for push_hz in (-1.0, 51.0):
        with pytest.raises(ValueError):
            EmulatorConfig(attitude_push_hz=push_hz)
    with pytest.raises(ValueError):
        EmulatorConfig(rate_deadband=2.5)
    with pytest.raises(ValueError):
        EmulatorConfig(yaw_report_scale=0.0)
    assert EmulatorConfig().non_ideal() == {}
    assert EmulatorConfig(rate_dead_time_s=0.6).non_ideal() == {"rate_dead_time_s": 0.6}
