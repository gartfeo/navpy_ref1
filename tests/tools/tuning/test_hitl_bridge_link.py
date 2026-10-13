"""MAVLink link of the HITL JSBSim bridge (tools/tuning/hitl_bridge/link.py) with a fake transport."""
import socket
import threading
from collections import deque

import pytest
from pymavlink.dialects.v20 import common as mavlink2

from tools.tuning.hitl_bridge.link import BRIDGE_COMPID, BRIDGE_SYSID, MavlinkLink
from tools.tuning.hitl_bridge.transport import SerialTransport, TcpTransport, make_transport

BOARD = mavlink2.MAVLink(None, srcSystem=1, srcComponent=mavlink2.MAV_COMP_ID_AUTOPILOT1)


def actuator_frame(time_usec: int, armed: bool = False, pwm=(1500, 1500, 1000), flags: int = 0,
                   sender: mavlink2.MAVLink = BOARD, mavlink1: bool = False) -> bytes:
    """HIL_ACTUATOR_CONTROLS as the board's HIL backend sends it (contract v2: MAVLink 2, CRC,
    flags 0 on a board, 1 = lockstep only from SITL's hil:lockstep)."""
    controls = list(pwm) + [0.0] * (16 - len(pwm))
    mode = mavlink2.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
    return bytes(sender.hil_actuator_controls_encode(time_usec, controls, mode, flags).pack(sender,
                                                                                            force_mavlink1=mavlink1))


def heartbeat_frame() -> bytes:
    return bytes(BOARD.heartbeat_encode(mavlink2.MAV_TYPE_FIXED_WING, mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                        0, 0, 0).pack(BOARD))


STATE_FIELDS = dict(attitude_quaternion=[1.0, 0.0, 0.0, 0.0], rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
                    lat=-353632610, lon=1491652300, alt=584000, vx=0, vy=0, vz=0, ind_airspeed=0,
                    true_airspeed=0, xacc=0, yacc=0, zacc=-1000)


class FakeTransport:
    """Plays a script: open() results and read() chunks; an exception in either is raised."""

    def __init__(self, opens=(), reads=()) -> None:
        self.opens = deque(opens)
        self.reads = deque(reads)
        self.written = bytearray()
        self.fail_writes = False
        self.open_calls = 0
        self.close_calls = 0

    def open(self) -> None:
        self.open_calls += 1
        result = self.opens.popleft() if self.opens else None
        if isinstance(result, Exception):
            raise result

    def read(self) -> bytes:
        item = self.reads.popleft() if self.reads else b""
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data: bytes) -> None:
        if self.fail_writes:
            raise OSError("write failed: device gone")
        self.written += data

    def close(self) -> None:
        self.close_calls += 1

    def shutdown(self) -> None:
        self.close()


def make_link(transport: FakeTransport, sleeps: list) -> MavlinkLink:
    return MavlinkLink(transport, reconnect_interval_s=1.5, sleep=sleeps.append, log=lambda _: None)


def never() -> bool:
    return False


def decode_all(data: bytes) -> list:
    return mavlink2.MAVLink(None).parse_buffer(data) or []


def test_frames_split_across_reads_and_other_messages_ignored():
    frame = actuator_frame(5000, flags=1)
    link = make_link(FakeTransport(reads=[frame[:10], frame[10:] + heartbeat_frame()]), [])
    assert link.receive(never) == []
    msgs = link.receive(never)
    assert [(m.time_usec, m.flags) for m in msgs] == [(5000, 1)]
    assert (link.stats.rx_actuators, link.stats.rx_other) == (1, 1)


def test_crc_error_is_counted_and_the_next_frame_still_parses():
    bad = bytearray(actuator_frame(5000))
    bad[-1] ^= 0xFF                                         # break the CRC
    link = make_link(FakeTransport(reads=[bytes(bad) + actuator_frame(10000)]), [])
    assert [m.time_usec for m in link.receive(never)] == [10000]
    assert link.stats.rx_bad >= 1


def test_state_is_sent_as_mavlink2_from_the_bridge_identity():
    transport = FakeTransport()
    link = make_link(transport, [])
    assert link.connect(never)
    assert link.send_state(123456789, STATE_FIELDS)
    assert transport.written[0] == mavlink2.PROTOCOL_MARKER_V2
    (msg,) = decode_all(bytes(transport.written))
    assert msg.get_type() == "HIL_STATE_QUATERNION"
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (BRIDGE_SYSID, BRIDGE_COMPID)
    assert (BRIDGE_SYSID, BRIDGE_COMPID) == (251, mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER)
    assert (msg.time_usec, msg.lat, msg.zacc) == (123456789, -353632610, -1000)


def test_open_is_retried_at_the_reconnect_interval():
    transport = FakeTransport(opens=[OSError("COM11 not found"), OSError("COM11 not found"), None],
                              reads=[actuator_frame(5000)])
    sleeps: list = []
    link = make_link(transport, sleeps)
    assert [m.time_usec for m in link.receive(never)] == [5000]
    assert sleeps == [1.5, 1.5]
    assert (link.stats.open_failures, link.stats.connects, transport.open_calls) == (2, 1, 3)


def test_lost_device_reconnects_with_a_fresh_parser():
    first = actuator_frame(5000)
    second = actuator_frame(9_000_000)
    transport = FakeTransport(reads=[first[:15], OSError("ClearCommError failed"), second])
    link = make_link(transport, [])
    assert link.receive(never) == []                        # half a frame
    assert link.receive(never) == [] and not link.connected
    assert transport.close_calls == 1
    assert [m.time_usec for m in link.receive(never)] == [9_000_000]    # reopened: the half frame is gone
    assert link.stats.rx_bad == 0
    assert (link.stats.connects, link.stats.disconnects) == (2, 1)


def test_failed_write_drops_the_link_until_the_next_receive():
    transport = FakeTransport(reads=[actuator_frame(5000)])
    link = make_link(transport, [])
    link.connect(never)
    transport.fail_writes = True
    assert link.send_state(5000, STATE_FIELDS) is False
    assert not link.connected
    assert link.send_state(5000, STATE_FIELDS) is False     # still down: nothing attempted
    transport.fail_writes = False
    assert [m.time_usec for m in link.receive(never)] == [5000]
    assert link.send_state(5000, STATE_FIELDS) is True


def test_stop_request_ends_the_reconnect_loop():
    sleeps: list = []
    link = make_link(FakeTransport(opens=[OSError("absent")] * 5), sleeps)
    answers = iter([False, False, True])
    assert link.receive(lambda: next(answers)) == []
    assert len(sleeps) == 2 and not link.connected


def test_device_strings():
    tcp = make_transport("tcp:127.0.0.1:5763")
    assert isinstance(tcp, TcpTransport) and (tcp.host, tcp.port) == ("127.0.0.1", 5763)
    port = make_transport("COM11", 921600)
    assert isinstance(port, SerialTransport) and (port.port, port.baud) == ("COM11", 921600)
    with pytest.raises(ValueError):
        make_transport("tcp:5763")


def test_tcp_transport_timeout_data_and_peer_close():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    accepted = {}
    thread = threading.Thread(target=lambda: accepted.setdefault("conn", server.accept()[0]))
    thread.start()
    transport = TcpTransport("127.0.0.1", server.getsockname()[1])
    try:
        transport.open()
        thread.join(timeout=5)
        conn = accepted["conn"]
        assert transport.read() == b""                       # nothing within the read timeout
        conn.sendall(b"\xfd\x01")
        assert transport.read() == b"\xfd\x01"
        transport.write(b"abc")
        assert conn.recv(3) == b"abc"
        conn.close()
        with pytest.raises(OSError):                         # peer closed: the link must reconnect
            transport.read()
    finally:
        transport.close()
        server.close()


def test_the_down_time_counts_from_the_link_start_until_the_first_connection():
    now = [100.0]
    link = MavlinkLink(FakeTransport(), reconnect_interval_s=1.0, sleep=lambda _: None, log=lambda _: None,
                       clock=lambda: now[0])
    now[0] = 112.5
    assert link.down_for() == 12.5                          # no client yet: the grace time runs
    assert link.connect(never) and link.down_for() == 0.0
