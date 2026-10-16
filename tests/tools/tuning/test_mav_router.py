"""The board link router (tools/tuning/mav_router.py, mav_router_policy.py): routing and stream-rate claims as pure
rules, then the router itself between a fake board on loopback TCP and real UDP clients."""
from __future__ import annotations

import io
import json
import socket
import threading
import time
from typing import Optional

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tools.tuning.mav_router import parse_endpoint
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mav_router_policy import (INTERVAL_DEFAULT_US, ROUTER_COMPONENT, ROUTER_SYSTEM, Identity,
                                            StreamClaims, destinations, interval_request, is_router_target)
from tools.tuning.hitl_bridge.transport import TcpTransport
from tools.tuning.mp_mirror import LOOPBACK

M = mavlink2
BOARD = Identity(1, M.MAV_COMP_ID_AUTOPILOT1)
RUNNER = Identity(250, 0)                       # board.SOURCE_SYSTEM, pymavlink's default component
NAVPY = Identity(1, M.MAV_COMP_ID_ONBOARD_COMPUTER)
ATTITUDE = M.MAVLINK_MSG_ID_ATTITUDE
POSITION = M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT
WRITE_HOLD_S = 0.002                            # how long GatedBoard holds one write
WAIT_S = 2.0                                    # loopback arrives in microseconds; this only bounds a failing test


def enc(identity: Identity) -> M.MAVLink:
    return M.MAVLink(None, srcSystem=identity.system, srcComponent=identity.component)


def decode(identity: Identity, msg: M.MAVLink_message) -> M.MAVLink_message:
    """msg as a receiver parses it (source ids and fields set)."""
    return M.MAVLink(None).parse_buffer(msg.pack(enc(identity)))[0]


def ack(command: int, to: Identity, result: int = M.MAV_RESULT_ACCEPTED) -> M.MAVLink_message:
    return M.MAVLink_command_ack_message(command, result, 0, 0, to.system, to.component)


def set_interval(message_id: int, interval_us: int, param3: float = 0) -> M.MAVLink_message:
    return M.MAVLink_command_long_message(1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id, interval_us,
                                          param3, 0, 0, 0, 0)


def all_streams(rate_hz: int) -> M.MAVLink_message:
    return M.MAVLink_request_data_stream_message(1, 1, M.MAV_DATA_STREAM_ALL, rate_hz, 1)


def heartbeat() -> M.MAVLink_message:
    return M.MAVLink_heartbeat_message(M.MAV_TYPE_FIXED_WING, M.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0, 3)


# ---- rules ---------------------------------------------------------------------------------------
CLIENTS = {"runner": {RUNNER}, "navpy": {NAVPY}}


def test_untargeted_board_packets_go_to_every_client():
    assert destinations(decode(BOARD, heartbeat()), CLIENTS) == ["runner", "navpy"]


def test_acks_go_only_to_their_sender():
    assert destinations(decode(BOARD, ack(1, RUNNER)), CLIENTS) == ["runner"]
    assert destinations(decode(BOARD, ack(1, NAVPY)), CLIENTS) == ["navpy"]


def test_a_component_zero_client_stands_for_its_system_and_a_zero_target_for_every_component():
    assert destinations(decode(BOARD, ack(1, Identity(250, 7))), CLIENTS) == ["runner"]
    assert destinations(decode(BOARD, ack(1, Identity(1, 0))), CLIENTS) == ["navpy"]


def test_an_unknown_target_reaches_no_client():
    assert destinations(decode(BOARD, ack(1, Identity(42, 1))), CLIENTS) == []
    assert destinations(decode(BOARD, ack(1, Identity(1, 99))), CLIENTS) == []


def test_router_replies_are_recognised():
    assert is_router_target(decode(BOARD, ack(1, Identity(ROUTER_SYSTEM, ROUTER_COMPONENT))))
    assert not is_router_target(decode(BOARD, ack(1, RUNNER)))


def test_interval_request_only_for_plain_set_message_interval():
    assert interval_request(decode(NAVPY, set_interval(ATTITUDE, 20000))) == (ATTITUDE, 20000)
    assert interval_request(decode(NAVPY, set_interval(ATTITUDE, 20000, param3=1))) is None
    assert interval_request(decode(RUNNER, all_streams(10))) is None


def test_fastest_positive_claim_wins_and_a_withdrawal_never_slows_another_client():
    claims = StreamClaims()
    assert claims.claim("navpy", ATTITUDE, 20000) == 20000
    assert claims.claim("mp", ATTITUDE, 50000) == 20000
    assert claims.claim("mp", ATTITUDE, -1) == 20000
    assert claims.claim("navpy", ATTITUDE, 0) == INTERVAL_DEFAULT_US
    assert claims.claimed() == {}


def test_all_streams_rate_is_a_floor_for_claims():
    claims = StreamClaims()
    claims.claim("navpy", POSITION, 1_000_000)          # 1 Hz claim
    claims.stream_rate(10)                               # the runner's ALL at 10 Hz
    assert claims.effective(POSITION) == 100_000
    claims.claim("navpy", ATTITUDE, 20000)
    assert claims.claimed() == {ATTITUDE: 20000, POSITION: 100_000}


@pytest.mark.parametrize("text, ok", [("navpy=9642", True), ("navpy", False), ("=9642", False),
                                      ("navpy=x", False), ("navpy=8000", False),
                                      ("router=9642", False)])
def test_endpoint_argument(text, ok):
    if ok:
        assert parse_endpoint(text) == ("navpy", 9642)
    else:
        with pytest.raises(Exception):
            parse_endpoint(text)


# ---- the router on loopback ----------------------------------------------------------------------
class FakeBoard:
    """The board's side: a TCP listener (as a SITL serial port) that records every packet the router writes."""

    def __init__(self) -> None:
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind((LOOPBACK, 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.conn: Optional[socket.socket] = None
        self.parser = M.MAVLink(None)
        self.received: list[M.MAVLink_message] = []
        self.arrived = threading.Condition()
        self.mav = enc(BOARD)

    def accept(self) -> None:
        self.listener.settimeout(WAIT_S)
        self.conn, _ = self.listener.accept()
        self.parser = M.MAVLink(None)
        threading.Thread(target=self._read, args=(self.conn,), daemon=True).start()

    def _read(self, conn: socket.socket) -> None:
        while True:
            try:
                data = conn.recv(4096)
            except OSError:
                return
            if not data:
                return
            with self.arrived:
                self.received += self.parser.parse_buffer(data) or []
                self.arrived.notify_all()

    def wait_for(self, count: int) -> list[M.MAVLink_message]:
        with self.arrived:
            assert self.arrived.wait_for(lambda: len(self.received) >= count, WAIT_S), self.received
            return list(self.received)

    def send(self, msg: M.MAVLink_message) -> None:
        self.conn.sendall(msg.pack(self.mav))
        self.mav.seq = (self.mav.seq + 1) % 256

    def drop(self) -> None:
        self.conn.close()

    def close(self) -> None:
        if self.conn is not None:
            self.conn.close()
        self.listener.close()


class UdpClient:
    """A client as pymavlink's udpin: listens on its port and answers whoever sent to it."""

    def __init__(self, identity: Identity) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOOPBACK, 0))
        self.sock.settimeout(WAIT_S)
        self.port = self.sock.getsockname()[1]
        self.mav = enc(identity)
        self.peer = None

    def receive(self) -> M.MAVLink_message:
        data, self.peer = self.sock.recvfrom(65535)
        return M.MAVLink(None).parse_buffer(data)[0]

    def nothing_within(self, seconds: float) -> bool:
        self.sock.settimeout(seconds)
        try:
            self.sock.recvfrom(65535)
            return False
        except socket.timeout:
            return True
        finally:
            self.sock.settimeout(WAIT_S)

    def send(self, msg: M.MAVLink_message) -> None:
        self.sock.sendto(msg.pack(self.mav), self.peer)
        self.mav.seq = (self.mav.seq + 1) % 256


@pytest.fixture
def rig():
    board = FakeBoard()
    runner, navpy, mp = UdpClient(RUNNER), UdpClient(NAVPY), UdpClient(Identity(255, 190))
    router = MavRouter(TcpTransport(LOOPBACK, board.port),
                       [Client("runner", runner.port, False), Client("navpy", navpy.port, False),
                        Client("mp", mp.port, True)], announce=lambda line: None).start()
    board.accept()
    board.send(heartbeat())                    # every client learns the router's address from it
    for client in (runner, navpy, mp):
        assert client.receive().get_type() == "HEARTBEAT"
    yield board, runner, navpy, mp, router
    router.close()
    board.close()
    for client in (runner, navpy, mp):
        client.sock.close()


def intervals(packets: list[M.MAVLink_message]) -> list[tuple[int, int, int, int]]:
    """(source system, source component, message id, interval) of every SET_MESSAGE_INTERVAL the board got."""
    return [(m.get_srcSystem(), m.get_srcComponent(), int(m.param1), int(m.param2)) for m in packets
            if m.get_type() == "COMMAND_LONG" and m.command == M.MAV_CMD_SET_MESSAGE_INTERVAL]


def test_runner_stream_request_after_navpy_claim_is_followed_by_the_claim(rig):
    board, runner, navpy, _, router = rig
    navpy.send(set_interval(ATTITUDE, 20000))
    board.wait_for(1)
    runner.send(all_streams(10))
    got = board.wait_for(3)
    assert [m.get_type() for m in got] == ["COMMAND_LONG", "REQUEST_DATA_STREAM", "COMMAND_LONG"]
    assert intervals(got) == [(1, 191, ATTITUDE, 20000), (ROUTER_SYSTEM, ROUTER_COMPONENT, ATTITUDE, 20000)]
    assert (got[2].target_system, got[2].target_component) == (1, 1)    # addressed as the runner's request


def test_navpy_claim_after_runner_stream_request_is_not_slowed(rig):
    board, runner, navpy, _, _ = rig
    runner.send(all_streams(10))
    board.wait_for(1)
    navpy.send(set_interval(POSITION, 1_000_000))          # slower than the runner's stream
    got = board.wait_for(2)
    assert intervals(got) == [(1, 191, POSITION, 100_000)]  # repacked in navpy's name at the stream rate


def test_acks_reach_only_their_client_and_router_acks_none(rig):
    board, runner, navpy, mp, router = rig
    board.send(ack(M.MAV_CMD_SET_MESSAGE_INTERVAL, NAVPY))      # before navpy has sent: no client known
    assert navpy.nothing_within(0.2)
    navpy.send(heartbeat())
    runner.send(heartbeat())
    board.wait_for(2)
    board.send(ack(M.MAV_CMD_SET_MESSAGE_INTERVAL, NAVPY))
    assert navpy.receive().target_component == NAVPY.component
    board.send(ack(M.MAV_CMD_SET_MESSAGE_INTERVAL, Identity(ROUTER_SYSTEM, ROUTER_COMPONENT)))
    board.send(ack(M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, RUNNER))
    assert runner.receive().command == M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    assert navpy.nothing_within(0.2) and mp.nothing_within(0.05)
    stats = router.report()["stats"]
    assert stats["router_replies"] == 1 and stats["board_unrouted"] == 1


def test_observer_cannot_fly_or_change_rates(rig):
    board, runner, _, mp, router = rig
    mp.send(M.MAVLink_rc_channels_override_message(1, 1, *([1500] * 8)))
    mp.send(all_streams(4))
    mp.send(M.MAVLink_param_request_list_message(1, 1))
    got = board.wait_for(1)
    assert [m.get_type() for m in got] == ["PARAM_REQUEST_LIST"]
    stats = router.report()["stats"]
    assert stats["blocked_mp_RC_CHANNELS_OVERRIDE"] == 1 and stats["blocked_mp_REQUEST_DATA_STREAM"] == 1


def test_claims_are_resent_after_the_board_comes_back(rig):
    board, _, navpy, _, router = rig
    navpy.send(set_interval(ATTITUDE, 20000))
    board.wait_for(1)
    board.drop()                                 # a reboot: the port goes away
    board.accept()                               # the router reopens it
    board.send(heartbeat())
    got = board.wait_for(2)
    assert intervals(got)[-1] == (ROUTER_SYSTEM, ROUTER_COMPONENT, ATTITUDE, 20000)
    stats = router.report()["stats"]
    assert stats["board_lost"] == 1 and stats["board_opened"] == 2


def test_board_sequence_gaps_are_counted(rig):
    board, runner, _, _, router = rig
    board.mav.seq = (board.mav.seq + 3) % 256    # three packets lost on the way
    board.send(heartbeat())
    runner.receive()
    assert router.report()["board_seq_gaps"] == {"1/1": 3}


def test_board_packets_keep_flowing_while_both_clients_write(rig):
    """No starvation: a burst from the board reaches navpy whole while the runner and navpy flood the board."""
    board, runner, navpy, _, router = rig
    burst = 500
    navpy.send(heartbeat())
    runner.send(heartbeat())
    board.wait_for(2)
    writers = [threading.Thread(target=lambda c=c: [c.send(heartbeat()) for _ in range(burst)]) for c in (runner, navpy)]
    for w in writers:
        w.start()
    for i in range(burst):
        board.send(M.MAVLink_attitude_message(i, 0, 0, 0, 0, 0, 0))
    seen = [navpy.receive() for _ in range(burst)]
    for w in writers:
        w.join()
    assert [m.time_boot_ms for m in seen] == list(range(burst))
    assert len(board.wait_for(2 + 2 * burst)) == 2 + 2 * burst
    assert router.report()["board_seq_gaps"] == {}


# ---- board handle ownership and the endpoint barrier ---------------------------------------------
class GatedBoard:
    """An in-process board transport whose write can be held, to put a write in progress or a queue behind it."""

    def __init__(self) -> None:
        self.gate = threading.Event()
        self.gate.set()
        self.writing = threading.Event()
        self.written: list[bytes] = []
        self.open_count = 0
        self.closed_mid_write = 0
        self._handle: Optional[object] = None
        self._lost = threading.Event()

    def open(self) -> None:
        self._handle = object()
        self._lost.clear()
        self.open_count += 1

    def read(self) -> bytes:
        if self._lost.wait(0.01):
            raise OSError("board rebooted")
        return b""

    def write(self, data: bytes) -> None:
        handle = self._handle
        assert handle is not None, "write on a closed handle"          # SerialTransport.write's own assertion
        self.writing.set()
        self.gate.wait(WAIT_S)
        time.sleep(WRITE_HOLD_S)                 # a USB write takes time: a close must not land inside it
        if self._handle is not handle:           # closed (and maybe reopened) while writing
            self.closed_mid_write += 1
        self.written.append(data)
        self.writing.clear()

    def close(self) -> None:
        self._handle = None

    def shutdown(self) -> None:
        self.close()

    def reboot(self) -> None:
        self._lost.set()


@pytest.fixture
def gated():
    board = GatedBoard()
    navpy, runner = UdpClient(NAVPY), UdpClient(RUNNER)
    router = MavRouter(board, [Client("navpy", navpy.port, False), Client("runner", runner.port, False)],
                       announce=lambda line: None, control_port=0).start()
    router_ports = {name: c.sock.getsockname() for name, c in router.clients.items()}
    navpy.peer, runner.peer = router_ports["navpy"], router_ports["runner"]
    yield board, navpy, runner, router
    board.gate.set()
    router.close()
    navpy.sock.close()
    runner.sock.close()


def send_across_transition(client: UdpClient, router: MavRouter, name: str, landed) -> None:
    """One heartbeat just after a transition (barrier, reopen, reboot). If the receiver read it before it took the
    transition's wake-up, it is dropped as stale (fail closed) and counted; the receiver has then taken the
    transition, so exactly one more is sent. ``landed`` is the outcome the test expects for a packet after it."""
    def stale() -> int:
        return router.report()["stats"].get(f"dropped_stale_{name}", 0)

    before = stale()
    client.send(heartbeat())
    wait_until(lambda: landed() or stale() > before)
    if not landed():
        assert stale() == before + 1
        client.send(heartbeat())
        wait_until(landed)


def wait_until(predicate, seconds: float = WAIT_S) -> None:
    deadline = time.monotonic() + seconds
    while not predicate():
        assert time.monotonic() < deadline, "condition not reached"
        time.sleep(0.005)


def test_a_reboot_under_write_load_never_writes_on_a_closed_handle(gated):
    board, navpy, _, router = gated
    stop = threading.Event()

    def flood() -> None:
        while not stop.is_set():
            navpy.send(heartbeat())

    flooder = threading.Thread(target=flood)
    flooder.start()
    for _ in range(20):
        board.reboot()
        wait_until(lambda: board._handle is not None and not board._lost.is_set())
    stop.set()
    flooder.join()
    wait_until(lambda: router.report()["stats"].get("board_lost", 0) >= 20)
    writer = [t for t in router._threads if t.name == "router-board-writer"][0]
    assert writer.is_alive()                     # an assertion inside write would have killed it
    assert board.closed_mid_write == 0
    count = len(board.written)

    def link_works() -> bool:                    # the flood's backlog is dropped after the last reboot: send until one
        navpy.send(heartbeat())                  # packet sent after it is written
        return len(board.written) > count

    wait_until(link_works)


def test_barrier_drops_queued_packets_and_waits_for_the_write_in_progress(gated):
    board, navpy, runner, router = gated
    board.gate.clear()                           # the first write hangs: a write in progress, the rest queue
    for _ in range(5):
        navpy.send(heartbeat())
    assert board.writing.wait(WAIT_S)
    wait_until(lambda: router.clients["navpy"].queued == 4)
    reply: dict = {}
    closer = threading.Thread(target=lambda: reply.update(router.close_endpoint("navpy")))
    closer.start()
    closer.join(0.2)
    assert closer.is_alive()                     # the barrier waits for the write in progress
    board.gate.set()
    closer.join(WAIT_S)
    assert reply["dropped_queued"] == 4 and not reply["already_closed"]
    assert reply["last_write_mono_s"] is not None and reply["last_write_mono_s"] <= reply["barrier_mono_s"]
    send_across_transition(navpy, router, "navpy",    # after the barrier: refused
                           lambda: router.report()["stats"].get("refused_closed_navpy") == 1)
    runner.send(heartbeat())                     # other clients unaffected
    wait_until(lambda: len(board.written) == 2)
    stats = router.report()["stats"]
    assert stats["dropped_closed_navpy"] == 4


def test_barrier_over_the_control_port_and_reopen(gated):
    board, navpy, _, router = gated
    navpy.send(heartbeat())
    wait_until(lambda: len(board.written) == 1)
    control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control.settimeout(WAIT_S)
    address = router.control.sock.getsockname()
    control.sendto(json.dumps({"op": "close", "client": "navpy", "id": 7}).encode(), address)
    reply = json.loads(control.recvfrom(4096)[0])
    assert reply["op"] == "closed" and reply["id"] == 7 and reply["dropped_queued"] == 0
    control.sendto(json.dumps({"op": "close", "client": "navpy", "id": 8}).encode(), address)
    assert json.loads(control.recvfrom(4096)[0])["already_closed"] is True    # idempotent (runner after navpy)
    control.sendto(json.dumps({"op": "open", "client": "navpy", "id": 9}).encode(), address)
    assert json.loads(control.recvfrom(4096)[0])["op"] == "opened"
    send_across_transition(navpy, router, "navpy", lambda: len(board.written) == 2)
    control.sendto(b"not json", address)
    assert json.loads(control.recvfrom(4096)[0])["op"] == "error"
    control.sendto(json.dumps({"op": "close", "client": "nobody"}).encode(), address)
    assert json.loads(control.recvfrom(4096)[0])["op"] == "error"
    control.close()



def test_a_packet_received_before_a_reboot_never_reaches_the_new_boot(gated):
    board, navpy, _, router = gated
    navpy.send(heartbeat())
    wait_until(lambda: len(board.written) == 1)
    old_gen = router._link_gen
    board.reboot()
    wait_until(lambda: router._link_gen == old_gen + 1 and router._link_up.is_set())
    client = router.clients["navpy"]
    router._from_client(client, heartbeat(), old_gen, client.epoch)    # read before the reboot, queued after it
    send_across_transition(navpy, router, "navpy", lambda: len(board.written) == 2)    # under the new link: goes out
    assert router.report()["stats"]["dropped_link_down"] == 1


def test_a_packet_received_while_closed_never_goes_out_after_the_reopen(gated):
    board, _, _, router = gated
    client = router.clients["navpy"]
    router.close_endpoint("navpy")
    closed_epoch = client.epoch
    router.open_endpoint("navpy")
    router._from_client(client, heartbeat(), router._link_gen, closed_epoch)
    assert router.report()["stats"]["refused_closed_navpy"] == 1
    assert client.queued == 0 and board.written == []


def test_a_partial_packet_is_dropped_never_joined_to_the_next_datagram(gated):
    board, navpy, _, router = gated
    raw = heartbeat().pack(navpy.mav)
    navpy.sock.sendto(raw[:5], navpy.peer)      # e.g. the last bytes of a process that stopped mid-send
    navpy.sock.sendto(raw[5:], navpy.peer)
    navpy.send(heartbeat())
    wait_until(lambda: len(board.written) == 1)
    assert navpy.nothing_within(0.1) and len(board.written) == 1
    assert router.report()["stats"]["partial_navpy"] >= 1


def test_a_repeated_barrier_does_not_count_the_same_packets_twice(gated):
    board, navpy, _, router = gated
    board.gate.clear()
    for _ in range(5):
        navpy.send(heartbeat())
    assert board.writing.wait(WAIT_S)
    wait_until(lambda: router.clients["navpy"].queued == 4)
    board.gate.set()
    first = router.close_endpoint("navpy")
    second = router.close_endpoint("navpy")
    assert (first["dropped_queued"], second["dropped_queued"]) == (4, 0) and second["already_closed"]
    wait_until(lambda: router.report()["stats"].get("dropped_closed_navpy") == 4)
    assert router.clients["navpy"].queued == 0



class ObservedLock:
    """The router lock, held by a test so a receiver stops between its recvfrom and its stamp."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.waiting = threading.Event()

    def __enter__(self) -> "ObservedLock":
        if not self._lock.acquire(blocking=False):
            self.waiting.set()
            self._lock.acquire()
        return self

    def __exit__(self, *exc) -> None:
        self._lock.release()


def observed_rig():
    board, navpy = GatedBoard(), UdpClient(NAVPY)
    router = MavRouter(board, [Client("navpy", navpy.port, False)], announce=lambda line: None)
    router._lock = ObservedLock()
    router.start()
    navpy.peer = router.clients["navpy"].sock.getsockname()
    wait_until(lambda: router._link_up.is_set())
    navpy.send(heartbeat())
    wait_until(lambda: len(board.written) == 1)
    return board, navpy, router


BACKLOG = 3                                     # datagrams waiting when a transition happens


def straddle(router: MavRouter, navpy: UdpClient, transition) -> None:
    """navpy's first datagram is read and the rest wait in the socket, then the transition runs before the receiver
    can stamp the first."""
    lock = router._lock
    with lock:
        lock.waiting.clear()
        for _ in range(BACKLOG):
            navpy.send(heartbeat())
        assert lock.waiting.wait(WAIT_S)
        transition()


@pytest.mark.parametrize("transition", ["barrier_and_reopen", "reboot"])
def test_a_datagram_read_across_a_transition_is_dropped(transition):
    board, navpy, router = observed_rig()
    client = router.clients["navpy"]
    try:
        if transition == "reboot":       # the reopen's state change, made while the receiver waits for the lock
            straddle(router, navpy, lambda: setattr(router, "_link_gen", router._link_gen + 1))
        else:
            straddle(router, navpy, lambda: (client.close_barrier(), client.reopen()))
        wait_until(lambda: router.report()["stats"].get("dropped_stale_navpy") == BACKLOG)    # the whole backlog
        navpy.send(heartbeat())          # sent after the transition: goes out
        wait_until(lambda: len(board.written) == 2)
        assert navpy.nothing_within(0.1) and len(board.written) == 2
    finally:
        router.close()
        navpy.sock.close()


def test_a_stale_stream_request_changes_no_claim_and_queues_nothing(gated):
    board, _, _, router = gated
    client = router.clients["navpy"]
    router.claims.claim("navpy", M.MAVLINK_MSG_ID_ATTITUDE, 20_000)
    before = router.claims.claimed()
    stale = M.MAVLink_request_data_stream_message(1, 1, M.MAV_DATA_STREAM_ALL, 2, 1)
    with router._lock:
        stale_gen = router._link_gen - 1
    router._from_client(client, stale, stale_gen, client.epoch)
    stats = router.report()["stats"]
    assert router.claims.claimed() == before and "router_intervals_sent" not in stats
    assert stats["dropped_link_down"] == 1 and board.written == []



def test_a_claim_sent_while_the_link_is_down_changes_nothing(gated):
    board, _, _, router = gated
    client = router.clients["navpy"]
    request = M.MAVLink_command_long_message(1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                             M.MAVLINK_MSG_ID_ATTITUDE, 20_000, 0, 0, 0, 0, 0)
    with router._lock:
        router._link_up.clear()
    router._from_client(client, request, -1, client.epoch)     # stamped while down
    with router._lock:
        router._link_up.set()
    assert router.claims.claimed() == {} and board.written == []


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_an_interval_that_is_no_number_is_not_a_claim(value):
    request = M.MAVLink_command_long_message(1, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL, 0, value, 20_000, 0, 0, 0, 0, 0)
    assert interval_request(request) is None
    request.param1, request.param2 = M.MAVLINK_MSG_ID_ATTITUDE, value
    assert interval_request(request) is None

def test_opening_an_open_endpoint_keeps_its_queue(gated):
    board, navpy, _, router = gated
    board.gate.clear()
    for _ in range(5):
        navpy.send(heartbeat())
    assert board.writing.wait(WAIT_S)
    wait_until(lambda: router.clients["navpy"].queued == 4)
    assert router.open_endpoint("navpy")["already_open"] is True
    board.gate.set()
    wait_until(lambda: len(board.written) == 5)
    assert router.clients["navpy"].queued == 0 and router.close_endpoint("navpy")["dropped_queued"] == 0

class FailingOpenBoard(GatedBoard):
    """Its first open takes the handle and then fails (pyserial's reset_input_buffer after the port opened)."""

    def __init__(self) -> None:
        super().__init__()
        self.opened_while_held = 0

    def open(self) -> None:
        if self._handle is not None:
            self.opened_while_held += 1          # a real port would refuse: it is still open
        super().open()
        if self.open_count == 1:
            raise OSError("reset_input_buffer failed")


def test_a_failed_open_releases_the_handle_before_the_retry():
    board = FailingOpenBoard()
    navpy = UdpClient(NAVPY)
    router = MavRouter(board, [Client("navpy", navpy.port, False)], announce=lambda line: None).start()
    navpy.peer = router.clients["navpy"].sock.getsockname()
    try:
        wait_until(lambda: router._link_up.is_set())
        navpy.send(heartbeat())
        wait_until(lambda: len(board.written) == 1)
    finally:
        router.close()
        navpy.sock.close()
    assert board.open_count == 2 and board.opened_while_held == 0

def test_write_log_names_the_source_of_every_board_packet():
    board = GatedBoard()
    navpy = UdpClient(NAVPY)
    log = io.StringIO()
    router = MavRouter(board, [Client("navpy", navpy.port, False)], announce=lambda line: None,
                       write_log=log).start()
    navpy.peer = router.clients["navpy"].sock.getsockname()
    try:
        navpy.send(heartbeat())
        wait_until(lambda: len(board.written) == 1)
    finally:
        router.close()
        navpy.sock.close()
    stamp, source, message = log.getvalue().strip().split(",")
    assert source == "navpy" and int(message) == M.MAVLINK_MSG_ID_HEARTBEAT and float(stamp) > 0
