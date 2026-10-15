"""The terminal barrier at the board boundary (design hitl_final_approach.md section 2): navpy's fence and router
barrier against the real router, then the runner's handoff (navpy_handoff.py) before its RTL, and the floor's
recovery through it. A packet's time at the board is the router's write log (time.monotonic, as here)."""
from __future__ import annotations

import io
import socket
import subprocess
import sys
import threading
import time

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as M

from navpy.app.router_barrier import BarrierResult, RouterBarrier
from navpy.app.terminal_handoff import LinkFence, TerminalHandoff
from navpy.core.approach.vision_nav.episode_latch import FinalApproachTerminal
from tests.tools.tuning import test_mav_router as rig
from tests.tools.tuning.r3_fake_plane import FakePlane
from tools.tuning.flight_floor import HeightFloor
from tools.tuning.flight_steps import FlightCheckFailed
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mp_mirror import LOOPBACK
from tools.tuning.navpy_handoff import NavpyHandoff, direct_rtl, stop_navpy
from tools.tuning.vehicle import MODE

WAIT_S = rig.WAIT_S
PROBE_S = 0.2                    # a probe reaches the board in microseconds once the link is up
QUEUED = 4                       # navpy packets queued behind the write in progress at the terminal outcome


@pytest.fixture
def board_link():
    """The router on a held board, navpy as pymavlink's udpin on its client port (as launched in HITL), the runner
    as a plain UDP client, and the router's write log."""
    board, log = rig.GatedBoard(), io.StringIO()
    navpy = mavutil.mavlink_connection(f"udpin:{LOOPBACK}:0", source_system=1, source_component=191)
    navpy_port = navpy.port.getsockname()[1]
    runner = rig.UdpClient(rig.RUNNER)
    router = MavRouter(board, [Client("navpy", navpy_port, False), Client("runner", runner.port, False)],
                       announce=lambda line: None, control_port=0, write_log=log).start()
    # navpy learns the router's address from the board's first packet (udpin answers whoever sent to it)
    router.clients["navpy"].sock.sendto(rig.heartbeat().pack(rig.enc(rig.BOARD)), (LOOPBACK, navpy_port))
    assert navpy.recv_match(type="HEARTBEAT", blocking=True, timeout=WAIT_S) is not None
    runner.peer = router.clients["runner"].sock.getsockname()

    def link_up() -> bool:                       # packets before the board opens, or read across it, are dropped:
        navpy.mav.heartbeat_send(0, 0, 0, 0, 0)  # one probe at a time, so none is still in flight afterwards
        deadline = time.monotonic() + PROBE_S
        while time.monotonic() < deadline:
            if board.written:
                return True
            time.sleep(0.005)
        return False

    rig.wait_until(link_up)
    board.written.clear()
    log.seek(0)
    log.truncate()
    yield board, navpy, runner, router, log
    board.gate.set()
    router.close()
    navpy.close()
    runner.sock.close()


def control_address(router: MavRouter) -> tuple[str, int]:
    return router.control.sock.getsockname()


def writes(log: io.StringIO) -> list[tuple[float, str]]:
    return [(float(t), source) for t, source, _ in (line.split(",") for line in log.getvalue().splitlines())]


def hold_navpy_writes(board: rig.GatedBoard, router: MavRouter, send) -> None:
    """One navpy write in progress at the board and QUEUED more behind it."""
    board.gate.clear()
    for _ in range(1 + QUEUED):
        send()
    assert board.writing.wait(WAIT_S)
    rig.wait_until(lambda: router.clients["navpy"].queued == QUEUED)


def test_navpy_writes_its_record_only_after_the_router_barrier_and_nothing_of_it_follows(board_link):
    board, navpy, _, router, log = board_link
    hold_navpy_writes(board, router, lambda: navpy.mav.heartbeat_send(0, 0, 0, 0, 0))
    fence, records = LinkFence(), []
    fence.install(navpy, threading.RLock())
    handoff = TerminalHandoff(fence, RouterBarrier(control_address(router), "navpy"),
                              publish=lambda record: records.append((time.monotonic(), record)))
    handoff.on_terminal(FinalApproachTerminal.DOCKED, None)
    navpy.mav.set_mode_send(1, 1, MODE["AUTO"])  # RESET's AUTO restart after the outcome: fenced
    assert not handoff.join(0.2) and records == []   # the router waits for the write in progress
    board.gate.set()
    assert handoff.join(WAIT_S)
    at, record = records[0]
    assert record["barrier"] == "acked" and record["router_ack"]["dropped_queued"] == QUEUED
    assert record["fenced_writes"] == 1
    navpy_writes = [t for t, source in writes(log) if source == "navpy"]
    assert len(navpy_writes) == 1 and navpy_writes[0] <= record["router_ack"]["barrier_mono_s"] <= at


def test_the_runner_closes_navpy_after_navpy_closed_itself():
    board = rig.GatedBoard()
    router = MavRouter(board, [Client("navpy", 0, False)], announce=lambda line: None, control_port=0).start()
    try:
        barrier = RouterBarrier(control_address(router), "navpy")
        assert barrier.close_endpoint().ack["already_closed"] is False
        assert barrier.close_endpoint().ack["already_closed"] is True
    finally:
        router.close()


def test_buffered_navpy_packets_never_follow_the_runners_rtl_after_a_forced_stop(board_link):
    """navpy stopped without its own barrier (forced stop or crash): its queue in the router still drops first."""
    board, navpy, runner, router, log = board_link
    hold_navpy_writes(board, router, lambda: navpy.mav.heartbeat_send(0, 0, 0, 0, 0))
    handoff = NavpyHandoff(stop=lambda: {"stop": "timed out", "ended": "terminated"},
                           barrier=RouterBarrier(control_address(router), "navpy"),
                           stop_router=lambda: pytest.fail("the router answered"),
                           rtl_direct=lambda: pytest.fail("the router answered"))
    result: dict = {}
    closer = threading.Thread(target=lambda: result.update(handoff.before_rtl()))
    closer.start()
    closer.join(0.2)
    assert closer.is_alive()
    board.gate.set()
    closer.join(WAIT_S)
    assert result["barrier"]["ok"] and result["barrier"]["ack"]["dropped_queued"] == QUEUED and "failed" not in result
    runner.send(M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, MODE["RTL"], 0, 0, 0, 0, 0))
    navpy.mav.heartbeat_send(0, 0, 0, 0, 0)      # a late datagram from the stopped navpy
    rig.wait_until(lambda: [s for _, s in writes(log)].count("runner") == 1)
    rig.wait_until(lambda: router.report()["stats"].get("refused_closed_navpy") == 1)
    sources = [source for _, source in writes(log)]
    assert sources == ["navpy", "runner"]


def test_no_ack_ends_the_router_and_sends_rtl_on_the_boards_own_port():
    calls: list[str] = []
    dead = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dead.bind((LOOPBACK, 0))                     # a control port nobody answers
    try:
        handoff = NavpyHandoff(stop=lambda: calls.append("stop") or {"stop": "requested"},
                               barrier=RouterBarrier(dead.getsockname(), "navpy", timeout_s=0.3),
                               stop_router=lambda: calls.append("router") or {"ended": "terminated"},
                               rtl_direct=lambda: calls.append("rtl") or {"rtl": "direct", "confirmed": True})
        record = handoff.before_rtl()
    finally:
        dead.close()
    assert calls == ["stop", "router", "rtl"]
    assert record["rtl"] == "direct" and "no ack within 0.3 s" in record["failed"]
    assert handoff.before_rtl() is record and calls == ["stop", "router", "rtl"]     # once


def test_a_failing_direct_rtl_is_recorded():
    def refuse() -> dict:
        raise OSError("COM15 busy")
    dead = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dead.bind((LOOPBACK, 0))
    try:
        record = NavpyHandoff(lambda: {}, RouterBarrier(dead.getsockname(), "navpy", timeout_s=0.2),
                              lambda: {}, refuse).before_rtl()
    finally:
        dead.close()
    assert record["rtl"] == "direct" and record["confirmed"] is False and "COM15 busy" in record["error"]


def test_a_failing_router_stop_still_sends_the_direct_rtl_and_stays_failed():
    calls: list[str] = []

    def stuck_router() -> dict:
        calls.append("router")
        raise OSError("access denied")

    class NoAck:
        def close_endpoint(self):
            return BarrierResult(None, "no ack within 2 s")

    handoff = NavpyHandoff(lambda: {}, NoAck(), stuck_router,
                           lambda: calls.append("rtl") or {"rtl": "direct", "confirmed": True})
    first = handoff.before_rtl()
    second = handoff.before_rtl()                # e.g. the floor after the approach's own end
    assert calls == ["router", "rtl"] and second is first
    assert first["rtl"] == "direct" and "no ack" in first["failed"] and "access denied" in first["router"]["error"]


# ---- the floor's recovery goes through the handoff -----------------------------------------------
def breach(hook) -> tuple[FakePlane, dict]:
    """A dive through the floor; hook(plane) is the floor's before_recovery."""
    plane, record = FakePlane(height=40, mode="FBWA", fbwa_sink=10), {}
    plane.tx.set(2, 1350)
    with pytest.raises(FlightCheckFailed, match="below the run's floor"):
        with HeightFloor(plane, 30, record, before_recovery=hook(plane)):
            plane.fly_for(20)
    return plane, record


def test_a_floor_breach_stops_navpy_and_closes_its_endpoint_before_the_rtl():
    modes_at_handoff: list[str] = []
    plane, record = breach(lambda plane: lambda: modes_at_handoff.append(plane.mode) or {"barrier": {"ok": True}})
    assert modes_at_handoff == ["FBWA"]          # the handoff ran before the RTL
    assert record["navpy_handoff"] == {"barrier": {"ok": True}}
    assert record["recovery"]["mode"] == "RTL" and plane.mode == "RTL"


def test_a_floor_breach_with_a_direct_rtl_does_not_recover_through_the_router():
    plane, record = breach(lambda plane: lambda: {"rtl": "direct", "confirmed": True, "failed": "no ack"})
    assert record["recovery"] == {"mode": "RTL", "sent_by": "navpy handoff"}
    assert plane.mode == "FBWA"                  # nothing went through the runner's (router) link


def test_a_raising_handoff_sends_no_rtl_behind_navpys_open_endpoint():
    def broken() -> dict:
        raise RuntimeError("handoff broke")
    plane, record = breach(lambda plane: broken)
    assert record["navpy_handoff"] == {"error": "RuntimeError: handoff broke"} and plane.mode == "FBWA"
    assert record["recovery"] == {"mode": "RTL", "error": "navpy handoff failed: no RTL sent"}


@pytest.mark.parametrize("step", ["stop", "barrier"])
def test_an_error_before_the_confirmed_close_takes_the_no_ack_path(step):
    calls: list[str] = []

    def broken() -> dict:
        raise OSError(f"{step} broke")

    class BrokenBarrier:
        def close_endpoint(self):
            raise OSError("barrier broke")

    class AckingBarrier:
        def close_endpoint(self):
            return BarrierResult({"op": "closed", "client": "navpy"})

    handoff = NavpyHandoff(stop=broken if step == "stop" else (lambda: {}),
                           barrier=BrokenBarrier() if step == "barrier" else AckingBarrier(),
                           stop_router=lambda: calls.append("router") or {},
                           rtl_direct=lambda: calls.append("rtl") or {"rtl": "direct", "confirmed": True})
    record = handoff.before_rtl()
    if step == "stop":                           # the router confirmed the close: RTL through it is safe
        assert calls == [] and record["navpy"]["error"] == "OSError: stop broke" and "failed" not in record
    else:
        assert calls == ["router", "rtl"] and "barrier broke" in record["failed"]


# ---- navpy's process and the direct RTL ------------------------------------------------------------
STOPS_ON_REQUEST = "import sys\nfor line in sys.stdin:\n    if line.strip() == 'NAVPY_STOP_V1':\n        sys.exit(0)\n"
IGNORES_REQUEST = "import time\ntime.sleep(60)\n"


def test_navpy_stops_on_its_stdin_request():
    process = subprocess.Popen([sys.executable, "-c", STOPS_ON_REQUEST], stdin=subprocess.PIPE, text=True)
    assert stop_navpy(process, wait_s=WAIT_S) == {"stop": "requested", "returncode": 0}


def test_a_navpy_that_does_not_stop_is_terminated():
    process = subprocess.Popen([sys.executable, "-c", IGNORES_REQUEST], stdin=subprocess.PIPE)
    record = stop_navpy(process, wait_s=0.3)
    assert record["stop"] == "timed out" and record["ended"] in ("terminated", "killed")
    assert process.poll() is not None


class FakeAutopilot:
    """The board on its own port: heartbeats, and RTL once a DO_SET_MODE asks for it."""

    def __init__(self, port: int, lose_first: bool) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.05)
        self.target = (LOOPBACK, port)
        self.mav = M.MAVLink(None, srcSystem=1, srcComponent=1)
        self.mode, self.commands, self._lose = MODE["FBWA"], 0, lose_first
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        parser = M.MAVLink(None)
        while not self.stop.is_set():
            hb = M.MAVLink_heartbeat_message(M.MAV_TYPE_FIXED_WING, M.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, self.mode, 0, 3)
            self.sock.sendto(hb.pack(self.mav), self.target)
            try:
                data = self.sock.recv(4096)
            except (socket.timeout, ConnectionResetError):
                continue
            for msg in parser.parse_buffer(data) or []:
                if msg.get_type() == "COMMAND_LONG" and msg.command == M.MAV_CMD_DO_SET_MODE:
                    self.commands += 1
                    if self._lose and self.commands == 1:
                        continue
                    self.mode = int(msg.param2)


@pytest.mark.parametrize("lose_first", [False, True])
def test_direct_rtl_sends_until_the_heartbeat_shows_rtl(lose_first):
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind((LOOPBACK, 0))
    port = probe.getsockname()[1]
    probe.close()
    autopilot = FakeAutopilot(port, lose_first)
    try:
        record = direct_rtl(f"udpin:{LOOPBACK}:{port}", 115200, wait_s=5.0)
    finally:
        autopilot.stop.set()
        autopilot.thread.join(WAIT_S)
        autopilot.sock.close()
    assert record["confirmed"] is True and autopilot.mode == MODE["RTL"]
    assert record["commands_sent"] == (2 if lose_first else 1)


def test_direct_rtl_without_a_heartbeat_is_unconfirmed():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind((LOOPBACK, 0))
    port = probe.getsockname()[1]
    probe.close()
    record = direct_rtl(f"udpin:{LOOPBACK}:{port}", 115200, wait_s=0.3)
    assert record == {"rtl": "direct", "confirmed": False, "commands_sent": 0, "error": "no heartbeat"}
