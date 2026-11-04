"""AIRSPEED (295) decoding on the shared bus (plugins/vehicle/ardupilot/airspeed_decode.py, mav_bus_reader.py).

hitl_final_approach.md section 9, step 19. The end-to-end tests send raw packet bytes from a fake board through
the PC router (tools/tuning/mav_router_core.py) into navpy's real MavBus on a UDP port, with one receiver per
fleet system, and check what each receiver gets.
"""
from __future__ import annotations

import math
import socket
import struct
import threading

import pytest
from pymavlink.dialects.v20 import all as A
from pymavlink.dialects.v20 import ardupilotmega as M

from navpy.plugins.vehicle.ardupilot.airspeed_decode import AIRSPEED_MSG_ID, AirspeedDecoder, unknown_msg_id
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from tools.tuning.hitl_bridge.transport import TcpTransport
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mp_mirror import LOOPBACK

OWN, PEER = 1, 2                    # the two fleet systems with a receiver on the bus
FOREIGN = 3                         # a system with no receiver on the bus
AUTOPILOT = M.MAV_COMP_ID_AUTOPILOT1
UNHEALTHY, USING = A.AIRSPEED_SENSOR_UNHEALTHY, A.AIRSPEED_SENSOR_USING
WAIT_S = 5.0                        # loopback delivers in milliseconds; this only bounds a failing test


def airspeed(system: int = OWN, component: int = AUTOPILOT, seq: int = 0, sensor: int = 0, cas: float = 22.5,
             temperature: int = 2500, raw_press: float = 3.1, flags: int = USING) -> bytes:
    """One AIRSPEED packet as ArduPilot's send_airspeed packs it (MAVLink 2, trailing zeros trimmed)."""
    mav = A.MAVLink(None, srcSystem=system, srcComponent=component)
    mav.seq = seq
    return bytes(mav.airspeed_encode(sensor, cas, temperature, raw_press, flags).pack(mav))


def ardupilotmega(msg: M.MAVLink_message, system: int = OWN, seq: int = 0) -> bytes:
    mav = M.MAVLink(None, srcSystem=system, srcComponent=AUTOPILOT)
    mav.seq = seq
    return bytes(msg.pack(mav))


def attitude(system: int = OWN, seq: int = 0, t_ms: int = 1) -> bytes:
    return ardupilotmega(M.MAVLink_attitude_message(t_ms, 0.1, -0.2, 0.3, 0.0, 0.0, 0.0), system, seq)


def with_crc_extra(raw: bytes, crc_extra: int) -> bytes:
    """``raw`` re-signed with another CRC extra (a sender with a different AIRSPEED definition)."""
    crc = A.x25crc(bytearray(raw[1:-2]) + bytearray([crc_extra])).crc
    return raw[:-2] + struct.pack("<H", crc)


def flip(raw: bytes, index: int) -> bytes:
    out = bytearray(raw)
    out[index] ^= 0x10
    return bytes(out)


# Fixtures: every flag combination, both sensors, both fleet systems, a trimmed all-zero tail, unknown
# temperature and NaN pressure (bytes, not floats, are compared), and wrapped sequence numbers.
VALID = [
    airspeed(seq=0, flags=0),
    airspeed(seq=1, flags=UNHEALTHY),
    airspeed(seq=2, flags=USING),
    airspeed(seq=3, flags=UNHEALTHY | USING),
    airspeed(seq=4, sensor=1, flags=0, cas=0.0),
    airspeed(seq=5, sensor=0, flags=0, cas=31.25, temperature=0, raw_press=0.0),   # tail trimmed to the floats
    airspeed(seq=255, temperature=32767, raw_press=float("nan")),
    airspeed(system=PEER, seq=7, sensor=1, flags=UNHEALTHY, cas=18.0),
    airspeed(system=OWN, component=M.MAV_COMP_ID_ONBOARD_COMPUTER, seq=8, flags=0),
]

_GOOD = airspeed(seq=9)
CORRUPT = [
    flip(_GOOD, 12),                                    # a payload bit (airspeed)
    flip(_GOOD, len(_GOOD) - 4),                        # the flags/id tail
    flip(_GOOD, len(_GOOD) - 1),                        # the CRC itself
    flip(_GOOD, 5),                                     # the source system in the header
    with_crc_extra(_GOOD, (A.MAVLink_airspeed_message.crc_extra + 1) % 256),
]


def fields(raw: bytes) -> dict:
    """Header and payload of a packet as the ``all`` dialect reads it."""
    m = A.MAVLink(None).decode(bytearray(raw))
    return {"src": m.get_srcSystem(), "comp": m.get_srcComponent(), "seq": m.get_seq(), "id": m.id,
            "flags": m.flags, "temperature": m.temperature,
            "floats": struct.pack("<ff", m.airspeed, m.raw_press)}


def unknown(raw: bytes) -> M.MAVLink_message:
    """What the shared dialect makes of ``raw`` (as the bus's mavutil connection does)."""
    return M.MAVLink(None).parse_buffer(bytearray(raw))[0]


# ---- the decoder alone ------------------------------------------------------------------

def test_every_valid_fixture_keeps_its_bytes_source_and_flags():
    decoder = AirspeedDecoder()
    for raw in VALID:
        shared = unknown(raw)
        assert isinstance(shared, M.MAVLink_unknown) and shared.get_srcSystem() == 0  # the identity it lacks
        assert unknown_msg_id(shared) == AIRSPEED_MSG_ID
        decoded = decoder.admit(shared)
        assert decoded is not None and decoded.get_type() == "AIRSPEED"
        assert bytes(decoded.get_msgbuf()) == raw
        assert {"src": decoded.get_srcSystem(), "comp": decoded.get_srcComponent(), "seq": decoded.get_seq(),
                "id": decoded.id, "flags": decoded.flags, "temperature": decoded.temperature,
                "floats": struct.pack("<ff", decoded.airspeed, decoded.raw_press)} == fields(raw)
    assert (decoder.decoded, decoder.refused) == (len(VALID), 0)


def test_trimmed_tail_decodes_as_zeros():
    decoded = AirspeedDecoder().admit(unknown(airspeed(sensor=0, flags=0, temperature=0, raw_press=0.0)))
    assert decoded.get_header().mlen < A.MAVLink_airspeed_message.unpacker.size
    assert (decoded.id, decoded.flags, decoded.temperature, decoded.raw_press) == (0, 0, 0, 0.0)


def test_nan_pressure_stays_nan():
    decoded = AirspeedDecoder().admit(unknown(airspeed(raw_press=float("nan"))))
    assert math.isnan(decoded.raw_press)


@pytest.mark.parametrize("raw", CORRUPT, ids=["payload", "tail", "crc", "header", "crc_extra"])
def test_a_corrupt_airspeed_is_refused(raw):
    decoder = AirspeedDecoder()
    shared = unknown(raw)
    assert unknown_msg_id(shared) == AIRSPEED_MSG_ID         # the shared dialect cannot tell: it passes it on
    assert decoder.admit(shared) is None
    assert (decoder.decoded, decoder.refused) == (0, 1)


def test_a_packet_relabelled_as_airspeed_is_refused():
    """Another message's bytes with its id changed to 295 (and the length kept) fail AIRSPEED's CRC."""
    other = bytearray(attitude())
    other[7], other[8], other[9] = AIRSPEED_MSG_ID & 0xFF, AIRSPEED_MSG_ID >> 8, 0
    assert AirspeedDecoder().admit(unknown(bytes(other))) is None


def test_other_packets_keep_the_shared_dialects_result():
    decoder = AirspeedDecoder()
    known = unknown(attitude())
    assert decoder.admit(known) is known
    # RADIO_RC_CHANNELS (420): the ``all`` dialect knows it, but only 295 is decoded with it.
    mav = A.MAVLink(None, srcSystem=OWN, srcComponent=AUTOPILOT)
    rc = unknown(bytes(mav.radio_rc_channels_encode(1, 1, 5, 0, 2, [1500, 1600] + [0] * 30).pack(mav)))
    assert isinstance(rc, M.MAVLink_unknown) and unknown_msg_id(rc) == 420
    assert decoder.admit(rc) is rc
    assert (decoder.decoded, decoder.refused) == (0, 0)


def test_mavlink1_unknowns_are_not_airspeed():
    assert unknown_msg_id(M.MAVLink_unknown(39, bytearray(b"\xfe\x00\x00\x01\x01\x27\x00\x00"))) is None


# ---- through the PC router into navpy's bus -------------------------------------------------

class Recorder:
    def __init__(self) -> None:
        self.messages: list[M.MAVLink_message] = []
        self.changed = threading.Condition()

    def feed_message(self, message: M.MAVLink_message) -> None:
        with self.changed:
            self.messages.append(message)
            self.changed.notify_all()

    def wait_for_attitude(self, t_ms: int) -> list[M.MAVLink_message]:
        def arrived() -> bool:
            return any(m.get_type() == "ATTITUDE" and m.time_boot_ms == t_ms for m in self.messages)
        with self.changed:
            assert self.changed.wait_for(arrived, WAIT_S), [m.get_type() for m in self.messages]
            return list(self.messages)


class Board:
    """The board's side: a TCP listener (as a SITL serial port) that sends raw bytes to the router."""

    def __init__(self) -> None:
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind((LOOPBACK, 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.conn: socket.socket | None = None

    def accept(self) -> None:
        self.listener.settimeout(WAIT_S)
        self.conn, _ = self.listener.accept()

    def send(self, raw: bytes) -> None:
        self.conn.sendall(raw)

    def close(self) -> None:
        if self.conn is not None:
            self.conn.close()
        self.listener.close()


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((LOOPBACK, 0))
        return s.getsockname()[1]


@pytest.fixture
def chain():
    board = Board()
    port = free_udp_port()
    bus = MavBus.get_or_create(f"udpin:{LOOPBACK}:{port}", source_system=OWN,
                               source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER)
    own, peer = Recorder(), Recorder()
    bus.attach(OWN, own)
    bus.attach(PEER, peer)
    router = MavRouter(TcpTransport(LOOPBACK, board.port), [Client("navpy", port, False)],
                       announce=lambda line: None).start()
    board.accept()
    yield board, bus, own, peer, router
    router.close()
    bus.close()
    board.close()


def send_all(board: Board, packets: list[bytes], end_ms: int) -> None:
    """Each packet in its own write, then an ATTITUDE marker to each system (stream order is kept)."""
    for raw in packets:
        board.send(raw)
    board.send(attitude(OWN, t_ms=end_ms))
    board.send(attitude(PEER, t_ms=end_ms))


def airspeeds(messages: list[M.MAVLink_message]) -> list[M.MAVLink_message]:
    return [m for m in messages if m.get_msgId() == AIRSPEED_MSG_ID or "295" in m.get_type()]


def test_valid_airspeed_reaches_only_its_own_system_unchanged(chain):
    board, bus, own, peer, router = chain
    send_all(board, VALID, end_ms=100)
    got_own, got_peer = airspeeds(own.wait_for_attitude(100)), airspeeds(peer.wait_for_attitude(100))
    expected_own = [raw for raw in VALID if fields(raw)["src"] == OWN]
    expected_peer = [raw for raw in VALID if fields(raw)["src"] == PEER]
    assert [bytes(m.get_msgbuf()) for m in got_own] == expected_own
    assert [bytes(m.get_msgbuf()) for m in got_peer] == expected_peer
    for m in got_own + got_peer:
        assert m.get_type() == "AIRSPEED"
        raw = bytes(m.get_msgbuf())
        assert (m.get_srcSystem(), m.get_srcComponent(), m.get_seq(), m.id, m.flags) == tuple(
            fields(raw)[k] for k in ("src", "comp", "seq", "id", "flags"))
    assert (bus.airspeed.decoded, bus.airspeed.refused) == (len(VALID), 0)
    assert router.stats["to_navpy"] == len(VALID) + 2


def test_corrupt_airspeed_reaches_no_receiver(chain):
    board, bus, own, peer, router = chain
    send_all(board, CORRUPT + [VALID[2]], end_ms=200)
    got_own, got_peer = own.wait_for_attitude(200), peer.wait_for_attitude(200)
    assert [bytes(m.get_msgbuf()) for m in airspeeds(got_own)] == [VALID[2]]
    assert airspeeds(got_peer) == []
    assert not any(isinstance(m, M.MAVLink_unknown) for m in got_own + got_peer)
    assert router.stats["to_navpy"] == len(CORRUPT) + 3          # the router forwards them: only navpy refuses
    assert (bus.airspeed.decoded, bus.airspeed.refused) == (1, len(CORRUPT))


def test_ardupilotmega_and_navlink_traffic_is_unchanged(chain):
    board, bus, own, peer, router = chain
    ack = M.MAVLink_swarm_ack_message(7, 8, 9, 1000, 0, 11, 12, 13, 1)        # navlink 25110, broadcast
    packets = [attitude(OWN, seq=1, t_ms=5), ardupilotmega(ack, OWN, seq=2), airspeed(seq=3),
               ardupilotmega(M.MAVLink_vfr_hud_message(21.0, 20.0, 90, 50, 100.0, 0.5), OWN, seq=4)]
    send_all(board, packets, end_ms=300)
    got = own.wait_for_attitude(300)
    assert [m.get_type() for m in got] == ["ATTITUDE", "SWARM_ACK", "AIRSPEED", "VFR_HUD", "ATTITUDE"]
    assert [bytes(m.get_msgbuf()) for m in got[:4]] == packets
    assert isinstance(got[1], M.MAVLink_swarm_ack_message) and got[1].ref_msg_type == 13
    assert all(m.get_srcSystem() == OWN for m in got)
    assert airspeeds(peer.wait_for_attitude(300)) == []


def test_airspeed_from_an_unattached_system_reaches_no_receiver(chain):
    """Other traffic from an unattached source still goes to every receiver (the bus's fallback); AIRSPEED never."""
    board, bus, own, peer, router = chain
    send_all(board, [airspeed(system=FOREIGN, seq=1), attitude(FOREIGN, seq=2, t_ms=7)], end_ms=400)
    for recorder in (own, peer):
        got = recorder.wait_for_attitude(400)
        assert airspeeds(got) == []
        assert [m.get_srcSystem() for m in got if m.get_type() == "ATTITUDE"] == [FOREIGN, own is recorder and OWN or PEER]
    assert (bus.airspeed.decoded, bus.airspeed.refused) == (1, 0)
