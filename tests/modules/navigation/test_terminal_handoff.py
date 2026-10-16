"""The HITL terminal handoff (navpy.app.terminal_handoff, navpy.app.router_barrier; design hitl_final_approach.md
sections 2-3): the episode latch reports its first outcome, the fence drops every later MAVLink write, the barrier
client waits bounded for the router's ack, and the record comes only after the fence and the barrier."""
from __future__ import annotations

import io
import json
import socket
import threading
import time
from contextlib import redirect_stdout

import pytest
from pymavlink import mavutil

from navpy.app.navpy_argparse import make_parser
from navpy.app.router_barrier import BarrierResult, RouterBarrier, parse_address
from navpy.app.terminal_handoff import (NAVPY_TERMINAL_MARKER, LinkFence, TerminalHandoff, build_terminal_handoff,
                                        fenced_opener, parse_terminal_record, print_terminal_record,
                                        siyi_fence, terminal_observer)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachEpisodeLatch
from navpy.core.approach.vision_nav.episode_latch import FinalApproachTerminal as Terminal
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.plugins.gimbal.siyi.hardware.command_fence import SiyiCommandFence
from navpy.plugins.gimbal.siyi.siyi_message import SIYIMESSAGE

WAIT_S = 2.0                     # loopback answers in microseconds; this only bounds a failing test
IDENTITY = DockContactIdentity(vehicle=1, task_id=7, dock_uid=42, episode=3)
LOOPBACK = "127.0.0.1"


def contact(identity: DockContactIdentity = IDENTITY) -> DockContactEvent:
    return DockContactEvent(identity, aircraft_time_s=12.5, closest_3d_m=0.3, closest_h_m=0.2, closest_v_m=0.1)


# ---- the episode latch's observer ----------------------------------------------------------------
def observed_latch() -> tuple[FinalApproachEpisodeLatch, list]:
    seen: list = []
    return FinalApproachEpisodeLatch(lambda outcome, identity: seen.append((outcome, identity))), seen


def test_the_first_latch_is_reported_once_with_the_armed_identity():
    latch, seen = observed_latch()
    latch.begin(IDENTITY)
    assert latch.latch_docked(contact())
    assert not latch.latch_missed() and latch.latch_failure() is False
    assert seen == [(Terminal.DOCKED, IDENTITY)]


def test_a_miss_then_its_nav_fail_is_one_outcome():
    latch, seen = observed_latch()
    latch.begin(IDENTITY)
    assert latch.latch_missed()
    assert latch.latch_failure()                 # the NAV_FAIL report of the miss is admitted, not a new outcome
    assert seen == [(Terminal.MISSED, IDENTITY)]


def test_a_nav_fail_with_no_armed_episode_is_reported_without_identity():
    latch, seen = observed_latch()
    assert latch.latch_failure()
    assert not latch.latch_docked(contact())
    assert seen == [(Terminal.FAILED, None)]


def test_a_contact_of_another_episode_is_not_an_outcome():
    latch, seen = observed_latch()
    latch.begin(IDENTITY)
    other = DockContactIdentity(vehicle=1, task_id=7, dock_uid=42, episode=2)
    assert not latch.latch_docked(contact(other))
    assert seen == []


# ---- the MAVLink fence ---------------------------------------------------------------------------
@pytest.fixture
def udp_link():
    """A pymavlink connection writing to a loopback socket the test reads."""
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((LOOPBACK, 0))
    connection = mavutil.mavlink_connection(f"udpout:{LOOPBACK}:{receiver.getsockname()[1]}", source_system=1,
                                            source_component=191)
    yield connection, receiver
    connection.close()
    receiver.close()


def arrived(receiver: socket.socket, seconds: float) -> bool:
    receiver.settimeout(seconds)
    try:
        receiver.recvfrom(65535)
        return True
    except socket.timeout:
        return False


def test_the_fence_drops_every_write_once_closed(udp_link):
    connection, receiver = udp_link
    fence = LinkFence()
    fence.install(connection, threading.RLock())
    connection.mav.heartbeat_send(0, 0, 0, 0, 0)
    assert arrived(receiver, WAIT_S)
    fence.close()
    connection.mav.heartbeat_send(0, 0, 0, 0, 0)
    connection.mav.set_mode_send(1, 1, 10)
    assert not arrived(receiver, 0.2)
    assert fence.closed and fence.dropped == 2


class SlowLink:
    """A connection whose write is held: a write in progress when the fence closes."""

    def __init__(self) -> None:
        self.entered = threading.Event()
        self.release = threading.Event()
        self.written: list[bytes] = []

    def write(self, buf: bytes) -> None:
        self.entered.set()
        self.release.wait(WAIT_S)
        self.written.append(buf)


def test_wait_idle_returns_only_after_the_write_in_progress():
    link, lock = SlowLink(), threading.RLock()
    fence = LinkFence()
    fence.install(link, lock)

    def send() -> None:
        with lock:                               # every navpy write runs under the bus send lock (MavTransport)
            link.write(b"before")

    sender = threading.Thread(target=send)
    sender.start()
    assert link.entered.wait(WAIT_S)
    fence.close()
    waiter = threading.Thread(target=fence.wait_idle)
    waiter.start()
    waiter.join(0.2)
    assert waiter.is_alive()
    link.release.set()
    waiter.join(WAIT_S)
    sender.join(WAIT_S)
    assert not waiter.is_alive()
    with lock:
        link.write(b"after")
    assert link.written == [b"before"] and fence.dropped == 1


def test_the_fence_reads_the_aircraft_time_of_the_last_message():
    link = SlowLink()
    fence = LinkFence()
    assert fence.autopilot_uptime_s() is None
    fence.install(link, threading.RLock())
    link.uptime = 81.25                          # pymavlink's post_message: time_boot_ms of the last message
    assert fence.autopilot_uptime_s() == 81.25


def test_the_opener_installs_the_fence_only_with_a_handoff():
    class Bus:
        conn, send_lock = SlowLink(), threading.RLock()

    def opener(*args: object, **kwargs: object) -> Bus:
        return Bus()

    assert fenced_opener(opener, None) is opener
    handoff = TerminalHandoff(LinkFence(), RouterBarrier((LOOPBACK, 9), "navpy"))
    bus = fenced_opener(opener, handoff)("udpin:x", 115200)
    handoff.fence.close()
    with bus.send_lock:
        bus.conn.write(b"x")
    assert bus.conn.written == [] and handoff.fence.dropped == 1
    assert terminal_observer(None) is None and terminal_observer(handoff) == handoff.on_terminal


# ---- the barrier client --------------------------------------------------------------------------
class FakeControl:
    """A router control port that answers after skipping `lose` requests, with `reply` (None: never)."""

    def __init__(self, reply=lambda request: {"op": "closed", "client": request["client"]}, lose: int = 0) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOOPBACK, 0))
        self.address = self.sock.getsockname()
        self.requests: list[dict] = []
        self._reply, self._lose = reply, lose
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self) -> None:
        while True:
            try:
                data, peer = self.sock.recvfrom(65535)
            except OSError:
                return
            request = json.loads(data)
            self.requests.append(request)
            if len(self.requests) <= self._lose:
                continue
            reply = self._reply(request)
            if reply is not None:
                self.sock.sendto(json.dumps({"id": request["id"], **reply}).encode(), peer)

    def close(self) -> None:
        self.sock.close()


def test_the_barrier_returns_the_router_ack():
    control = FakeControl()
    try:
        result = RouterBarrier(control.address, "navpy").close_endpoint()
    finally:
        control.close()
    assert result.ok and result.ack["op"] == "closed"
    assert control.requests[0]["op"] == "close" and control.requests[0]["client"] == "navpy"


def test_a_lost_request_is_sent_again_within_the_bound():
    control = FakeControl(lose=2)
    try:
        result = RouterBarrier(control.address, "navpy").close_endpoint()
    finally:
        control.close()
    assert result.ok and len(control.requests) == 3
    assert len({r["id"] for r in control.requests}) == 1      # one request, repeated: the router's close is idempotent


def test_no_ack_within_the_bound_is_a_failed_barrier():
    control = FakeControl(reply=lambda request: None)
    start = time.monotonic()
    try:
        result = RouterBarrier(control.address, "navpy", timeout_s=0.5).close_endpoint()
    finally:
        control.close()
    assert not result.ok and "no ack within 0.5 s" in result.error
    assert 0.5 <= time.monotonic() - start < WAIT_S


def test_a_refusal_is_a_failed_barrier_at_once():
    control = FakeControl(reply=lambda request: {"op": "error", "error": "unknown client 'navpy'"})
    try:
        result = RouterBarrier(control.address, "navpy").close_endpoint()
    finally:
        control.close()
    assert not result.ok and "unknown client" in result.error and len(control.requests) == 1


def test_a_reply_to_another_request_is_not_the_ack():
    control = FakeControl(reply=lambda request: {"op": "closed", "client": "navpy", "id": "another request"})
    try:
        result = RouterBarrier(control.address, "navpy", timeout_s=0.5).close_endpoint()
    finally:
        control.close()
    assert not result.ok and "no ack" in result.error


@pytest.mark.parametrize("text", ["127.0.0.1", ":9643", "host:0", "host:x", "host:70000"])
def test_a_bad_control_address_is_refused(text):
    with pytest.raises(ValueError):
        parse_address(text)


# ---- the handoff ---------------------------------------------------------------------------------
class ScriptedBarrier:
    """Records the order of events; close_endpoint blocks until released, then answers `result`."""

    def __init__(self, events: list, fence: LinkFence, result: BarrierResult) -> None:
        self.events, self.fence, self.result = events, fence, result
        self.release = threading.Event()
        self.calls = 0

    def close_endpoint(self) -> BarrierResult:
        self.calls += 1
        self.events.append(("barrier", self.fence.closed))
        assert self.release.wait(WAIT_S)
        return self.result


def scripted(result: BarrierResult) -> tuple[TerminalHandoff, ScriptedBarrier, list]:
    events: list = []
    fence = LinkFence()
    barrier = ScriptedBarrier(events, fence, result)
    return TerminalHandoff(fence, barrier, publish=lambda record: events.append(("record", record))), barrier, events


def test_the_record_follows_the_fence_and_the_barrier_ack():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy", "dropped_queued": 2}))
    lock = threading.Lock()
    with lock:                                   # the latch calls under the final-approach lock: it must not block
        handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    assert handoff.fence.closed
    assert not handoff.join(0.2)                 # the barrier is waiting: no record yet
    assert events == [("barrier", True)]
    barrier.release.set()
    assert handoff.join(WAIT_S)
    record = events[1][1]
    assert events[1][0] == "record" and record["barrier"] == "acked" and record["router_ack"]["dropped_queued"] == 2
    assert (record["outcome"], record["task"], record["dock"], record["episode"]) == ("docked", 7, 42, 3)


def test_no_ack_writes_a_barrier_failed_record():
    handoff, barrier, events = scripted(BarrierResult(None, "no ack within 2 s"))
    barrier.release.set()
    handoff.on_terminal(Terminal.FAILED, None)
    assert handoff.join(WAIT_S)
    record = events[-1][1]
    assert record["outcome"] == "failed" and record["barrier"] == "failed"
    assert record["barrier_error"] == "no ack within 2 s" and record["task"] is None


def test_only_the_first_outcome_hands_off():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    barrier.release.set()
    handoff.on_terminal(Terminal.MISSED, IDENTITY)
    handoff.on_terminal(Terminal.FAILED, IDENTITY)
    assert handoff.join(WAIT_S)
    assert barrier.calls == 1 and [e[0] for e in events] == ["barrier", "record"]
    assert events[1][1]["outcome"] == "missed"


def test_the_barrier_starts_only_after_the_write_in_progress_returns():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    barrier.release.set()
    link, lock = SlowLink(), threading.RLock()
    handoff.fence.install(link, lock)

    def send() -> None:
        with lock:
            link.write(b"attitude target")

    sender = threading.Thread(target=send)
    sender.start()
    assert link.entered.wait(WAIT_S)
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    assert not handoff.join(0.2) and events == []   # the write passed the fence before it closed: wait for it
    link.release.set()
    assert handoff.join(WAIT_S)
    sender.join(WAIT_S)
    assert [e[0] for e in events] == ["barrier", "record"] and link.written == [b"attitude target"]


def test_the_record_counts_writes_dropped_by_the_fence():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    link, lock = SlowLink(), threading.RLock()
    link.release.set()
    handoff.fence.install(link, lock)
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    with lock:
        link.write(b"held refresh")
    barrier.release.set()
    assert handoff.join(WAIT_S)
    assert events[-1][1]["fenced_writes"] == 1 and link.written == []


def test_the_record_is_one_flushed_line_the_runner_parses():
    out = io.StringIO()
    with redirect_stdout(out):
        print_terminal_record({"outcome": "docked", "barrier": "acked"})
    line = out.getvalue().strip()
    assert line.startswith(NAVPY_TERMINAL_MARKER + " ")
    assert parse_terminal_record(line) == {"outcome": "docked", "barrier": "acked"}
    assert parse_terminal_record("NAVPY_TERMINAL_V1 not json") is None
    assert parse_terminal_record("NAVPY_RUNTIME_READY_V1 {}") is None


def test_the_handoff_exists_only_with_a_router_control_address():
    parser = make_parser()
    base = ["-c", "udpin:127.0.0.1:14550", "-ss", "1"]
    assert build_terminal_handoff(parser.parse_args(base)) is None
    handoff = build_terminal_handoff(parser.parse_args(base + ["--router-control", "127.0.0.1:9643"]))
    assert handoff is not None


# ---- the gimbal command fence in the handoff ----------------------------------------------------
class RecordingGimbal:
    """A SIYI SDK stand-in: records the command id of every frame that passes the fence."""

    def __init__(self, events: list) -> None:
        self.events = events

    def sendMsg(self, msg: str) -> bool:
        self.events.append(("gimbal", bytes.fromhex(msg)[7]))
        return True

    def requestGimbalSpeed(self, yaw: int, pitch: int) -> bool:
        return self.sendMsg(SIYIMESSAGE().gimbalSpeedMsg(yaw, pitch))

    def requestZoomHold(self) -> bool:
        return self.sendMsg(SIYIMESSAGE().stopZoomMsg())


def test_the_gimbal_is_fenced_at_once_and_stopped_before_the_barrier():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    gimbal = handoff.gimbal_fence.wrap(lambda **_: RecordingGimbal(events))(server_ip=LOOPBACK, port=1)
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    assert handoff.gimbal_fence.closed           # closed on the latch's thread, before the handoff thread runs
    gimbal.requestGimbalSpeed(40, 0)             # a tracker command after the outcome
    barrier.release.set()
    assert handoff.join(WAIT_S)
    assert [e for e in events if e[0] != "record"] == [("gimbal", 0x07), ("gimbal", 0x05), ("barrier", True)]
    record = events[-1][1]
    assert record["gimbal_stop"] == "sent" and record["fenced_gimbal_commands"] == 1


def test_a_tracker_command_after_the_outcome_never_reaches_the_gimbal():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    gimbal = handoff.gimbal_fence.wrap(lambda **_: RecordingGimbal(events))(server_ip=LOOPBACK, port=1)
    handoff.on_terminal(Terminal.MISSED, IDENTITY)
    barrier.release.set()
    assert handoff.join(WAIT_S)
    gimbal.requestGimbalSpeed(40, 0)
    gimbal.sendMsg(SIYIMESSAGE().centerMsg())
    assert [e for e in events if e[0] == "gimbal"] == [("gimbal", 0x07), ("gimbal", 0x05)]
    assert handoff.gimbal_fence.dropped == 2


def test_without_a_siyi_gimbal_the_record_says_so():
    handoff, barrier, events = scripted(BarrierResult({"op": "closed", "client": "navpy"}))
    barrier.release.set()
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    assert handoff.join(WAIT_S)
    assert events[-1][1]["gimbal_stop"] == "no_gimbal" and events[-1][1]["fenced_gimbal_commands"] == 0


def test_the_vision_path_gets_the_handoff_gimbal_fence():
    assert siyi_fence(None) is None
    handoff = TerminalHandoff(LinkFence(), RouterBarrier((LOOPBACK, 9), "navpy"))
    assert siyi_fence(handoff) is handoff.gimbal_fence


class StuckGimbal(RecordingGimbal):
    """A gimbal whose send never returns: the record must still go out."""

    def __init__(self, events: list) -> None:
        super().__init__(events)
        self.release = threading.Event()

    def sendMsg(self, msg: str) -> bool:
        self.release.wait(WAIT_S)
        return super().sendMsg(msg)


def test_a_stuck_gimbal_send_still_gives_the_record_with_a_timeout():
    events: list = []
    fence = LinkFence()
    barrier = ScriptedBarrier(events, fence, BarrierResult({"op": "closed", "client": "navpy"}))
    handoff = TerminalHandoff(fence, barrier, publish=lambda record: events.append(("record", record)),
                              gimbal_fence=SiyiCommandFence(stop_timeout_s=0.3))
    gimbal = handoff.gimbal_fence.wrap(lambda **_: StuckGimbal(events))(server_ip=LOOPBACK, port=1)
    threading.Thread(target=gimbal.sendMsg, args=(SIYIMESSAGE().gimbalAttMsg(),), daemon=True).start()
    time.sleep(0.05)                             # the poll's send is in progress, holding the fence
    barrier.release.set()
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)
    assert handoff.join(WAIT_S)
    record = [e for e in events if e[0] == "record"][0][1]
    assert record["gimbal_stop"] == "timeout" and record["barrier"] == "acked"
    gimbal.release.set()
