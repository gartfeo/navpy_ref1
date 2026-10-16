"""The Mission Planner mirror of a board run (tools/tuning/mp_mirror.py, board.Board, hitl --mp-mirror):
real UDP on loopback, fake board connections and a fake Mission Planner socket."""
from __future__ import annotations

import socket
import threading
import time
from pathlib import Path
from typing import Callable

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tests.tools.tuning.fake_board import FakeConn, Msg
from tests.tools.tuning.test_board import ORIGIN, _runner, _sim_board, files  # noqa: F401 - files is a fixture
from tests.tools.tuning.test_hitl_args import _args
from tools.tuning.board import SOURCE_SYSTEM, Board
from tools.tuning.hitl import open_vehicle, vehicle_arg_errors
from tools.tuning.hitl_params import read_param_file
from tools.tuning.mp_mirror import (BLOCKED_MESSAGES, DEFAULT_PORT, LOOPBACK, RESERVED_PORT_BANDS, MavlinkMirror,
                                    reserved_band)
from tools.tuning.sitl_sim import DEFAULT_BINARY, HITL_PARAMS

MISSION_PLANNER_SYSID = 255            # Mission Planner's own MAVLink system id (its default)
WAIT_S = 2.0                           # loopback UDP arrives in microseconds; this only bounds a failing test
POLL_S = 0.01                          # how often wait_until looks (a FakeConn has no event to wait on)
M = mavlink2


def wait_until(condition: Callable[[], object]) -> bool:
    deadline = time.monotonic() + WAIT_S
    while not condition():
        if time.monotonic() > deadline:
            return False
        time.sleep(POLL_S)
    return True


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind((LOOPBACK, 0))
        return probe.getsockname()[1]


class MissionPlanner:
    """Mission Planner's UDP connection: it listens on the port and answers whoever sent to it."""

    def __init__(self, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOOPBACK, port))
        self.sock.settimeout(WAIT_S)
        self.mav = M.MAVLink(None, srcSystem=MISSION_PLANNER_SYSID, srcComponent=190)
        self.peer = None

    def receive(self) -> bytes:
        data, self.peer = self.sock.recvfrom(65535)
        return data

    def send(self, *messages: M.MAVLink_message) -> None:
        self.sock.sendto(b"".join(m.pack(self.mav) for m in messages), self.peer)

    def close(self) -> None:
        self.sock.close()


class Conn:
    """A board connection as the mirror uses it: hooks called with every message read, raw writes."""

    def __init__(self) -> None:
        self.message_hooks: list = []
        self.written: list[bytes] = []
        self._arrived = threading.Condition()

    def write(self, buf: bytes) -> int:
        with self._arrived:
            self.written.append(bytes(buf))
            self._arrived.notify_all()
        return len(buf)

    def read(self, msg: M.MAVLink_message) -> None:
        """What pymavlink's post_message does with every message the runner reads from the board."""
        for hook in self.message_hooks:
            hook(self, msg)

    def wait_written(self, count: int) -> list[bytes]:
        with self._arrived:
            assert self._arrived.wait_for(lambda: len(self.written) >= count, WAIT_S), self.written
            return list(self.written)


def board_packet(seq: int = 0) -> M.MAVLink_message:
    board = M.MAVLink(None, srcSystem=1, srcComponent=1)
    board.seq = seq
    msg = board.heartbeat_encode(M.MAV_TYPE_FIXED_WING, M.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
    msg.pack(board)                      # as received: the message keeps its bytes (get_msgbuf)
    return msg


def marker() -> M.MAVLink_message:
    """A packet the mirror passes: once the board has it, every earlier datagram was handled."""
    return M.MAVLink_heartbeat_message(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)


@pytest.fixture
def linked():
    """A mirror on a free port, Mission Planner listening there, one board connection attached."""
    port = free_udp_port()
    planner = MissionPlanner(port)
    mirror = MavlinkMirror(port)
    conn = Conn()
    mirror.attach(conn)
    conn.read(board_packet())                       # Mission Planner learns the mirror's address
    planner.receive()
    yield mirror, planner, conn
    mirror.close()
    planner.close()


def test_every_board_packet_goes_to_mission_planner_and_its_answers_go_to_the_board(linked):
    mirror, planner, conn = linked
    sent = [board_packet(seq) for seq in range(1, 4)]
    for msg in sent:
        conn.read(msg)
    assert [planner.receive() for _ in sent] == [bytes(m.get_msgbuf()) for m in sent]
    request = M.MAVLink_param_request_list_message(1, 1)
    command = M.MAVLink_command_long_message(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    planner.send(request, command)                  # two packets in one datagram
    written = conn.wait_written(2)
    parsed = M.MAVLink(None).parse_buffer(b"".join(written))
    assert [(m.get_type(), m.get_srcSystem()) for m in parsed] == [("PARAM_REQUEST_LIST", MISSION_PLANNER_SYSID),
                                                                   ("COMMAND_LONG", MISSION_PLANNER_SYSID)]
    assert mirror.stats["to_mission_planner"] == 4 and mirror.stats["from_mission_planner"] == 2


def test_mission_planner_cannot_fly_the_aircraft_take_its_gcs_id_or_change_the_runners_rates(linked):
    """The board takes RC overrides only from SYSID_MYGCS (GCS_Common.cpp:3990), which the runner's own
    hitl.parm sets to its system id: Mission Planner's 255 is not it, and the mirror keeps it so."""
    assert read_param_file(HITL_PARAMS)["SYSID_MYGCS"] == SOURCE_SYSTEM == 250 != MISSION_PLANNER_SYSID
    mirror, planner, conn = linked
    planner.send(M.MAVLink_rc_channels_override_message(1, 1, *[1900] * 8),
                 M.MAVLink_manual_control_message(1, 0, 0, 500, 0, 0),
                 M.MAVLink_param_set_message(1, 1, b"SYSID_MYGCS", 255.0, M.MAV_PARAM_TYPE_REAL32),
                 M.MAVLink_request_data_stream_message(1, 1, M.MAV_DATA_STREAM_ALL, 4, 1),
                 M.MAVLink_command_long_message(1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 30, 1e6, 0, 0, 0, 0, 0))
    planner.send(M.MAVLink_param_set_message(1, 1, b"RLL_RATE_P", 0.1, M.MAV_PARAM_TYPE_REAL32),
                 M.MAVLink_command_long_message(1, 1, M.MAV_CMD_REQUEST_MESSAGE, 0, 1, 0, 0, 0, 0, 0, 0), marker())
    written = M.MAVLink(None).parse_buffer(b"".join(conn.wait_written(3)))
    assert [m.get_type() for m in written] == ["PARAM_SET", "COMMAND_LONG", "HEARTBEAT"]
    assert written[0].param_id == "RLL_RATE_P" and written[1].command == M.MAV_CMD_REQUEST_MESSAGE
    assert {k: v for k, v in mirror.stats.items() if k.startswith("blocked_")} == {
        "blocked_RC_CHANNELS_OVERRIDE": 1, "blocked_MANUAL_CONTROL": 1, "blocked_PARAM_SET": 1,
        "blocked_REQUEST_DATA_STREAM": 1, "blocked_COMMAND_LONG": 1}
    assert BLOCKED_MESSAGES == {"RC_CHANNELS_OVERRIDE", "MANUAL_CONTROL", "REQUEST_DATA_STREAM"}


def test_while_the_board_reboots_mission_planners_packets_are_dropped_then_reach_the_new_link(linked):
    mirror, planner, old = linked
    mirror.detach()                                 # the runner closes the link for the reboot
    planner.send(M.MAVLink_param_request_list_message(1, 1))
    assert wait_until(lambda: mirror.stats["dropped_link_down"] == 1)
    new = Conn()
    mirror.attach(new)                              # the new boot's link
    new.read(board_packet(9))
    assert planner.receive() == bytes(board_packet(9).get_msgbuf())
    planner.send(marker())
    assert len(new.wait_written(1)) == 1 and old.written == []
    assert mirror.stats["dropped_link_down"] == 1


def test_only_mission_planners_port_is_heard(linked):
    mirror, planner, conn = linked
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
        other.sendto(M.MAVLink_param_request_list_message(1, 1).pack(M.MAVLink(None)), planner.peer)
    planner.send(marker())
    assert len(conn.wait_written(1)) == 1 and mirror.stats["foreign"] == 1


def test_close_ends_the_receiver_and_returns_the_counts():
    mirror = MavlinkMirror(free_udp_port())
    conn = Conn()
    mirror.attach(conn)
    conn.read(board_packet())                       # nobody listens: sent all the same
    record = mirror.close()
    assert record["port"] == mirror.port and record["to_mission_planner"] == 1
    assert not mirror._thread.is_alive()


def test_a_board_run_mirrors_every_link_through_the_reboots_and_records_the_counts(tmp_path, files):
    board = _sim_board()
    conns: list[FakeConn] = []

    def connect(device: str, **kwargs: object) -> FakeConn:
        conns.append(board.connection(device, **kwargs))
        return conns[-1]
    planner = MissionPlanner(free_udp_port())
    runner = Board("COM15", tmp_path / "run", files, origin=ORIGIN, erase=True, connect=connect,
                   mirror=MavlinkMirror(planner.sock.getsockname()[1]))
    try:
        runner.start()
        assert board.boots >= 2 and len(conns) == board.boots + 1      # the first link and one per boot
        assert all(c.message_hooks for c in conns)                    # every new link is mirrored
        assert all(c.closed for c in conns[:-1]) and not conns[-1].closed
        first = planner.receive()
        assert b"HEARTBEAT" in first                                   # the fake's packets are type and fields
        planner.send(M.MAVLink_command_long_message(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0))
        assert wait_until(lambda: conns[-1].written)                   # to the board's current link only
        assert [c.written for c in conns[:-1]] == [[]] * (len(conns) - 1)
    finally:
        runner.stop()
        planner.close()
    record = runner.record["mp_mirror"]
    assert record["port"] == runner.mirror.port and record["to_mission_planner"] > 0
    assert record["from_mission_planner"] == 1 and not runner.mirror._thread.is_alive()


def test_acks_to_mission_planners_commands_are_not_taken_as_the_runners(tmp_path, files):
    board = _sim_board()
    runner = _runner(board, tmp_path, files)
    runner.connect()
    reboot = M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    board.queue += [Msg("COMMAND_ACK", command=reboot, result=M.MAV_RESULT_FAILED, target_system=MISSION_PLANNER_SYSID),
                    Msg("COMMAND_ACK", command=reboot, result=M.MAV_RESULT_ACCEPTED, target_system=SOURCE_SYSTEM)]
    assert runner._ack(reboot).result == M.MAV_RESULT_ACCEPTED
    board.queue.append(Msg("COMMAND_ACK", command=reboot, result=M.MAV_RESULT_DENIED))   # MAVLink 1: no target
    assert runner._ack(reboot).result == M.MAV_RESULT_DENIED
    runner.close_link()


def test_the_mirror_needs_a_board_and_a_port_outside_the_ports_in_use(tmp_path):
    assert reserved_band(DEFAULT_PORT) is None
    assert all(reserved_band(low) and reserved_band(high) for _, low, high in RESERVED_PORT_BANDS)
    assert _args("--board", "COM15", "--mp-mirror").mp_mirror == DEFAULT_PORT == 9640
    assert vehicle_arg_errors(_args("--mp-mirror"), []) == ["--mp-mirror needs --board"]
    assert vehicle_arg_errors(_args("--board", "COM15", "--mp-mirror"), []) == []
    (error,) = vehicle_arg_errors(_args("--board", "COM15", "--mp-mirror", "14550"), [])
    assert "Mission Planner and GCS monitors" in error
    (error,) = vehicle_arg_errors(_args("--board", "COM15", "--mp-mirror", "70000"), [])
    assert "out of range" in error
    port = free_udp_port()
    board = open_vehicle(_args("--workdir", str(tmp_path / "b"), "--board", "COM15", "--mp-mirror", str(port)),
                         [], [], DEFAULT_BINARY)
    try:
        assert board.mirror.port == port
    finally:
        board.mirror.close()
    assert open_vehicle(_args("--workdir", str(Path(tmp_path) / "c"), "--board", "COM15"), [], [],
                        DEFAULT_BINARY).mirror is None
