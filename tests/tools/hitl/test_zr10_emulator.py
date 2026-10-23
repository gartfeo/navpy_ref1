"""The emulator process loop: vehicle_state in, gimbal_state out, and navpy's real SIYI client (GimbalSiyi) on UDP."""
from __future__ import annotations

import json
import math
import socket
import threading
import time

import pytest

from navpy.core.kernel.attitude import Attitude
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.plugins.gimbal.siyi.gimbal_siyi import GimbalSiyi
from navpy.plugins.gimbal.siyi.siyi_sdk import SIYISDK
from tests.tools.hitl.zr10_frames import frame, parse
from tests.tools.tuning.test_hitl_bridge_physics import rest_state
from tools.hitl.zr10_config import EmulatorConfig, load_zoom_table
from tools.hitl.zr10_device import Zr10Device
from tools.hitl.zr10_emulator import GIMBAL_STATE_SCHEMA, Zr10Emulator, build_parser, open_log
from tools.hitl.zr10_vehicle_input import VEHICLE_STATE_TIMEOUT_S, VehicleInput, decode_vehicle_state
from tools.tuning.hitl_bridge.unreal_pose import UnrealPoseSender

LOOPBACK = ("127.0.0.1", 0)


def vehicle_state(sequence, *, yaw=0.0, pitch=0.0, roll=0.0, session="s1", sys_id=1, board_time_us=None):
    return json.dumps({
        "schema": "navpy.unreal.vehicle_state.v1", "sys_id": sys_id, "session_id": session, "sequence": sequence,
        "board_time_us": sequence * 20_000 if board_time_us is None else board_time_us,
        "position_lla": {"lat_deg": 40.0, "lon_deg": 44.0, "alt_m": 1000.0},
        "attitude_deg": {"roll": roll, "pitch": pitch, "yaw": yaw},
    }).encode()


class _Logger:
    def __init__(self):
        self.lines = []

    def __getattr__(self, level):
        return lambda message, *args, **kwargs: self.lines.append((level, message))


class Running:
    """An emulator serving on loopback in a thread, a 50 Hz vehicle_state feed and an Unreal listener."""

    def __init__(self, config=None):
        self.unreal = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.unreal.bind(LOOPBACK)
        self.unreal.settimeout(1.0)
        device = Zr10Device(config or EmulatorConfig(), load_zoom_table("siyi_zr10"))
        self.emulator = Zr10Emulator(device, LOOPBACK, LOOPBACK, self.unreal.getsockname())
        self.stop = threading.Event()
        self.feeding = threading.Event()
        self._sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._threads = [threading.Thread(target=self.emulator.serve, args=(self.stop,), daemon=True),
                         threading.Thread(target=self._feed, daemon=True)]
        for thread in self._threads:
            thread.start()

    def _feed(self):
        sequence = 0
        next_s = time.perf_counter()
        while not self.stop.is_set():
            if self.feeding.is_set():
                sequence += 1
                self._sender.sendto(vehicle_state(sequence), self.emulator.vehicle_state_address)
            next_s += 0.02
            self.stop.wait(max(0.0, next_s - time.perf_counter()))

    def gimbal_states(self, seconds):
        """The gimbal_state datagrams Unreal got in the next ``seconds``."""
        states, end_s = [], time.monotonic() + seconds
        while time.monotonic() < end_s:
            self.unreal.settimeout(max(0.01, end_s - time.monotonic()))
            try:
                states.append(json.loads(self.unreal.recvfrom(4096)[0]))
            except socket.timeout:
                break
        return states

    def drain(self):
        """Drop the gimbal_state datagrams already queued."""
        self.unreal.setblocking(False)
        try:
            while True:
                self.unreal.recvfrom(4096)
        except BlockingIOError:
            pass
        finally:
            self.unreal.setblocking(True)

    def close(self):
        self.stop.set()
        for thread in self._threads:
            thread.join(2.0)
        self.emulator.close()
        self.unreal.close()
        self._sender.close()


@pytest.fixture
def running():
    emulator = Running()
    yield emulator
    emulator.close()


def wait_for(predicate, timeout_s=3.0):
    end_s = time.monotonic() + timeout_s
    while time.monotonic() < end_s:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    return predicate()


def test_navpy_siyi_client_drives_the_emulator_end_to_end(running):
    running.feeding.set()       # level, heading 0: lock-mode 0x0E angles are then body angles too
    assert wait_for(lambda: running.emulator.device.has_attitude)
    logger = _Logger()
    host, port = running.emulator.siyi_address
    gimbal = GimbalSiyi(GimbalData(att=Attitude(0.0, 0.0, 0.0)), host, port, logger)
    try:
        gimbal.start()
        assert gimbal.is_connected()
        assert ("info", "SIYI gimbal connected, Lock Mode enabled") in logger.lines
        # Absolute angles: navpy's right-positive yaw goes on the wire negated (left-positive) and comes back
        # right-positive; Unreal renders the same right turn.
        gimbal.set_att(Attitude(-20.0, 30.0, 0.0))
        assert wait_for(lambda: abs(gimbal.get_data().att.yaw - 30.0) < 0.05
                        and abs(gimbal.get_data().att.pitch + 20.0) < 0.05)
        state = running.gimbal_states(0.1)[-1]
        assert state["schema"] == GIMBAL_STATE_SCHEMA and state["sys_id"] == 1 and state["device_id"] == 1
        assert state["attitude_deg"]["yaw"] == pytest.approx(30.0, abs=0.05)
        assert state["attitude_deg"]["pitch"] == pytest.approx(-20.0, abs=0.05)
        assert state["motion_mode"] == "lock" and state["roll_stabilize"] is True
        # Rate: a positive yaw rate turns the camera right, so navpy's right-positive yaw readback rises.
        gimbal.set_rate(50.0, 0.0)
        assert wait_for(lambda: gimbal.get_data().att.yaw > 40.0)
        gimbal.set_rate(0.0, 0.0)
        # Zoom: command 2.0 reads 1.8 (profile table); manual zoom moves and holds.
        assert gimbal.get_zoom_level() == 1.1
        assert gimbal.set_zoom(2.0)
        assert wait_for(lambda: gimbal.get_zoom_level() == 1.8)
        state = running.gimbal_states(0.1)[-1]
        assert state["zoom"] == 1.8
        assert state["lens"] == {"reading": 1.8, "set_by": {"absolute_zoom": 2.0}, "settled": True}
        assert gimbal.zoom_in()
        assert wait_for(lambda: gimbal.get_zoom_level() >= 2.0)
        assert gimbal.zoom_hold()
        assert wait_for(lambda: running.gimbal_states(0.05)[-1]["lens"]["set_by"] is None)    # no row after manual
        held = running.emulator.device.true_state().zoom
        time.sleep(0.3)
        assert running.emulator.device.true_state().zoom == held
        gimbal.set_motion_mode(1)      # follow, as navpy's neutral return asks for
        assert wait_for(lambda: running.emulator.device.true_state().motion_mode == 1)
        assert wait_for(lambda: running.gimbal_states(0.05)[-1]["motion_mode"] == "follow")
    finally:
        assert gimbal.stop()
    counters = running.emulator.device.counters
    assert counters.bad_frames == 0 and counters.unknown_commands == 0 and counters.rejected_commands == 0
    for command in ("01", "02", "04", "05", "07", "08", "0a", "0c", "0d", "0e", "0f", "18"):
        assert counters.by_command.get(command, 0) > 0, command


def test_sdk_reads_the_config_reply_and_the_attitude_stream(running):
    running.feeding.set()
    assert wait_for(lambda: running.emulator.device.has_attitude)
    host, port = running.emulator.siyi_address
    sdk = SIYISDK(server_ip=host, port=port)
    try:
        assert sdk.connect(maxWaitTime=2.0)
        assert sdk.getCameraTypeString() == "ZR10"
        assert wait_for(lambda: sdk.getMotionMode() == 0 and sdk.getMountingDirection() == 1)
    finally:
        sdk.disconnect()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.5)
    try:
        client.sendto(frame(0x25, bytes((1, 5))), running.emulator.siyi_address)     # attitude at 20 Hz
        frames, end_s = [], time.monotonic() + 0.5
        while time.monotonic() < end_s:
            frames.append(parse(client.recvfrom(256)[0]))
        assert frames[0].cmd == 0x25 and frames[0].data == b"\x01"
        assert 7 <= sum(f.cmd == 0x0D for f in frames[1:]) <= 12
        client.sendto(frame(0x25, bytes((1, 0))), running.emulator.siyi_address)
    finally:
        client.close()


def test_nothing_answers_and_nothing_renders_before_the_aircraft_attitude(running):
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.3)
    try:
        client.sendto(frame(0x01), running.emulator.siyi_address)
        with pytest.raises(socket.timeout):
            client.recvfrom(256)
    finally:
        client.close()
    assert running.gimbal_states(0.2) == []
    assert running.emulator.summary()["valid"] is False


def test_a_vehicle_state_gap_stops_rendering_and_invalidates_the_run(running):
    running.feeding.set()
    assert len(running.gimbal_states(0.3)) >= 10
    assert running.emulator.summary()["valid"] is True
    running.feeding.clear()
    time.sleep(VEHICLE_STATE_TIMEOUT_S + 0.05)
    running.drain()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(0.2)
    try:
        client.sendto(frame(0x0D), running.emulator.siyi_address)     # stale attitude: no answer
        with pytest.raises(socket.timeout):
            client.recvfrom(256)
    finally:
        client.close()
    assert running.gimbal_states(0.1) == []
    assert running.emulator.stats.siyi_dropped_stale >= 1
    running.feeding.set()
    assert running.gimbal_states(0.3)
    summary = running.emulator.summary()
    assert summary["valid"] is False and summary["vehicle_state"]["gaps"] == 1


def test_catch_up_steps_carry_their_own_step_time():
    unreal = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    unreal.bind(LOOPBACK)
    unreal.settimeout(1.0)
    emulator = Zr10Emulator(Zr10Device(EmulatorConfig(), load_zoom_table("siyi_zr10")), LOOPBACK, LOOPBACK,
                            unreal.getsockname())
    try:
        state = emulator.vehicle.accept(vehicle_state(1), 10.0)
        emulator.device.set_vehicle_attitude(state.attitude)
        emulator._step(10.05, 10.02)     # a stalled loop runs two overdue steps at one instant
        emulator._step(10.05, 10.04)
        times = [json.loads(unreal.recvfrom(4096)[0])["sender_time_s"] for _ in range(2)]
        assert times == [10.02, 10.04]
    finally:
        emulator.close()
        unreal.close()


def test_vehicle_state_decoding_and_ordering():
    state = decode_vehicle_state(vehicle_state(3, yaw=10.0, pitch=2.0, roll=-5.0))
    assert (state.attitude.yaw, state.attitude.pitch, state.attitude.roll) == (10.0, 2.0, -5.0)
    for bad in (b"{}", b"not json", vehicle_state(1).replace(b'"yaw": 0.0', b'"yaw": NaN'),
                vehicle_state(1).replace(b'"sequence": 1', b'"sequence": -1')):
        with pytest.raises(ValueError):
            decode_vehicle_state(bad)
    vehicle = VehicleInput()
    assert vehicle.accept(vehicle_state(5), 0.0) is not None
    assert vehicle.accept(vehicle_state(5), 0.01) is None and vehicle.accept(vehicle_state(4), 0.01) is None
    assert vehicle.accept(vehicle_state(6, sys_id=2), 0.01) is None
    assert vehicle.invalid_reasons() == ["1 vehicle_state(s) from another system"]
    vehicle = VehicleInput()
    vehicle.accept(vehicle_state(5), 0.0)
    assert vehicle.accept(vehicle_state(6, board_time_us=0), 0.02) is not None     # a reboot: kept, but counted
    vehicle.accept(vehicle_state(1, session="s2"), 0.04)
    assert vehicle.invalid_reasons() == ["2 bridge sessions", "1 board time step(s) back"]
    assert vehicle.fresh(0.04 + VEHICLE_STATE_TIMEOUT_S) and not vehicle.fresh(0.05 + VEHICLE_STATE_TIMEOUT_S)
    assert not vehicle.fresh(1.0) and vehicle.stats.gaps == 1


def test_cli_defaults_are_the_ideal_config():
    args = build_parser().parse_args(["--vehicle-state", "127.0.0.1:0"])
    config = EmulatorConfig(**{name: getattr(args, name) for name in EmulatorConfig.__dataclass_fields__})
    assert config == EmulatorConfig() and config.non_ideal() == {}
    args = build_parser().parse_args(["--vehicle-state", "127.0.0.1:0", "--rate-dead-time-s", "0.6",
                                      "--rate-deadband", "4"])
    assert (args.rate_dead_time_s, args.rate_deadband) == (0.6, 4)


def test_the_bridges_vehicle_state_copy_is_the_emulators_attitude(running):
    """The bridge sends Unreal's vehicle_state datagram to the emulator too (--vehicle-state-copy): its IMU."""
    sender = UnrealPoseSender(running.unreal.getsockname(), alt_offset_m=0.0,
                              copies=(running.emulator.vehicle_state_address,))
    try:
        for slot in range(1, 6):
            sender(slot * 20_000, rest_state(roll_rad=math.radians(10.0), yaw_rad=math.radians(30.0)))
        assert wait_for(lambda: running.emulator.vehicle.stats.accepted == 5)
        latest = running.emulator.vehicle.latest
        assert (latest.attitude.roll, latest.attitude.yaw) == pytest.approx((10.0, 30.0))
        assert running.emulator.device.has_attitude and sender.stats.copies_sent == 5
    finally:
        sender.close()


def test_log_records_reach_the_file_before_close(tmp_path):
    path = tmp_path / "emulator.jsonl"
    log = open_log(str(path))
    try:
        log.write('{"event":"step"}\n')
        assert path.read_text(encoding="utf-8") == '{"event":"step"}\n'     # a killed process keeps it
    finally:
        log.close()
