"""The bridge's optional Unreal pose sender (tools/tuning/hitl_bridge/unreal_pose.py): wire format, board-time
slots, a dead or full receiver never blocking or failing the board link, and its place after the reply."""
import io
import itertools
import json
import math
import socket

import pytest

from tests.tools.tuning.test_hitl_bridge_link import FakeTransport, actuator_frame, make_link
from tests.tools.tuning.test_hitl_bridge_physics import FakePlant, rest_state
from tools.tuning.hitl_bridge.board_clock import BoardClockStepper
from tools.tuning.hitl_bridge.bridge import EXIT_BAD_SETUP, build_parser
from tools.tuning.hitl_bridge.catapult import Catapult
from tools.tuning.hitl_bridge.frame_loop import Bridge, LinkLog, run_bridge
from tools.tuning.hitl_bridge.unreal_pose import (NO_POSE_OUT, POSE_PERIOD_US, VEHICLE_STATE_SCHEMA, UnrealPoseSender,
                                                  encode_vehicle_state, parse_endpoint, unreal_pose_option)


class RecordingSocket:
    """A UDP socket stand-in: records datagrams, or raises the given error on every send."""

    def __init__(self, error=None):
        self.sent = []
        self.error = error
        self.blocking = True

    def setblocking(self, flag):
        self.blocking = flag

    def sendto(self, data, endpoint):
        if self.error is not None:
            raise self.error
        self.sent.append((json.loads(data), endpoint))
        return len(data)

    def close(self):
        pass


def sender(sock=None, period_us=POSE_PERIOD_US):
    return UnrealPoseSender(("127.0.0.1", 17110), alt_offset_m=23.5, period_us=period_us,
                            sock=sock or RecordingSocket())


def test_datagram_is_the_receivers_schema_with_board_time_sequence_and_the_named_altitude_offset():
    state = rest_state(lat_deg=40.3117412, lon_deg=44.4552112, alt_msl_m=1293.0, roll_rad=math.radians(5),
                       pitch_rad=math.radians(-2), yaw_rad=math.radians(90))
    payload = json.loads(encode_vehicle_state(state, 123_456_789, 7, "s1", 23.5, 1))
    assert payload["schema"] == VEHICLE_STATE_SCHEMA == "navpy.unreal.vehicle_state.v1"
    assert (payload["board_time_us"], payload["sequence"], payload["session_id"]) == (123_456_789, 7, "s1")
    assert payload["position_lla"] == {"lat_deg": 40.3117412, "lon_deg": 44.4552112, "alt_m": 1316.5}
    assert payload["attitude_deg"] == pytest.approx({"roll": 5.0, "pitch": -2.0, "yaw": 90.0})


@pytest.mark.parametrize("change, board_time_us", [
    ({"alt_msl_m": math.nan}, 0), ({"lat_deg": 91.0}, 0), ({"yaw_rad": math.inf}, 0), ({}, 2 ** 53), ({}, -1)])
def test_values_the_receiver_cannot_take_are_refused_before_sending(change, board_time_us):
    with pytest.raises(ValueError):
        encode_vehicle_state(rest_state(**change), board_time_us, 0, "s", 0.0, 1)
    out = sender()
    out(board_time_us, rest_state(**change))
    assert out.stats.refused == 1 and out.stats.sent == 0


def test_one_pose_per_board_time_slot_at_any_loop_rate():
    # Every slot that holds a frame gets exactly one pose, its first frame's.
    for loop_us, frames, at_least in ((5_000, 40, 10),          # 200 Hz board: every 4th frame, 50 Hz
                                      (19_992, 50, 49)):        # SITL's 19.992 ms loop: not halved to 25 Hz
        sock = RecordingSocket()
        out = sender(sock)
        times = [1_000_000 + k * loop_us for k in range(frames)]
        for t in times:
            out(t, rest_state())
        first_in_slot = {}
        for t in times:
            first_in_slot.setdefault(t // POSE_PERIOD_US, t)
        assert [p["board_time_us"] for p, _ in sock.sent] == sorted(first_in_slot.values())
        assert len(sock.sent) >= at_least, loop_us
        assert [p["sequence"] for p, _ in sock.sent] == list(range(len(sock.sent)))


def test_a_board_reboot_sends_at_once_and_the_sequence_keeps_rising():
    sock = RecordingSocket()
    out = sender(sock)
    out(5_000_000, rest_state())
    out(5_005_000, rest_state())        # same slot: not sent
    out(1_000, rest_state())            # board time went back: a new boot
    assert [(p["board_time_us"], p["sequence"]) for p, _ in sock.sent] == [(5_000_000, 0), (1_000, 1)]
    sock.sent.clear()
    out(19_000, rest_state())           # same slot as 1_000: not sent
    out(500, rest_state())              # back again, inside the same slot number: still a new boot
    assert [p["board_time_us"] for p, _ in sock.sent] == [500]


@pytest.mark.parametrize("error", [BlockingIOError(), ConnectionResetError(), OSError("unreachable")])
def test_a_full_or_dead_receiver_only_counts_a_dropped_pose(error):
    out = sender(RecordingSocket(error))
    for k in range(5):
        out(k * POSE_PERIOD_US, rest_state())
    assert (out.stats.sent, out.stats.dropped) == (0, 5)


def test_the_real_socket_is_non_blocking_and_a_closed_port_never_raises():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]              # closed again when the block ends: nobody listens
    out = UnrealPoseSender(("localhost", port), alt_offset_m=0.0)
    try:
        assert out._sock.getblocking() is False
        assert out.endpoint == ("127.0.0.1", port)              # resolved once, not per send
        for k in range(20):
            out(k * POSE_PERIOD_US, rest_state())
        assert out.stats.sent + out.stats.dropped == 20
    finally:
        out.close()


def test_the_pose_goes_out_after_the_boards_reply_with_the_answered_state_and_its_board_time():
    transport = FakeTransport(reads=[actuator_frame(1_000_000), actuator_frame(1_005_000)])
    link = make_link(transport, [])
    calls = []

    def after_reply(board_time_us, state):
        calls.append((board_time_us, state, len(transport.written)))

    stepper = BoardClockStepper(FakePlant(), Catapult(0.0, 0.0, 0.0))
    ticks = itertools.count()
    run_bridge(link, Bridge(stepper), LinkLog(io.StringIO()), lambda: next(ticks) * 0.001,
               should_stop=lambda: not transport.reads, say=lambda _: None, after_reply=after_reply)
    assert [c[0] for c in calls] == [1_000_000, 1_005_000]
    assert all(written > 0 for _, _, written in calls)          # the reply was already written
    assert calls[1][2] > calls[0][2]
    assert calls[-1][1] == stepper.board_state(1_005_000)


def test_cli_takes_the_endpoint_and_the_altitude_offset_together_with_no_default():
    parse = build_parser().parse_args
    assert unreal_pose_option(parse(["--device", "COM11", "--model-dir", "P"])) is NO_POSE_OUT
    for bad in (["--unreal-pose", "127.0.0.1:17110"], ["--unreal-pose", "127.0.0.1:17110", "nan"],
                ["--unreal-pose", "nohost", "23.5"], ["--unreal-pose", "127.0.0.1:17110", "x"]):
        with pytest.raises(SystemExit) as exit_info:
            parse(["--device", "COM11", "--model-dir", "P", *bad])
        assert exit_info.value.code == EXIT_BAD_SETUP
    out = unreal_pose_option(parse(["--device", "COM11", "--model-dir", "P", "--unreal-pose", "localhost:17110",
                                    "23.5"]))
    try:
        assert (out.endpoint, out.alt_offset_m) == (("127.0.0.1", 17110), 23.5)
    finally:
        out.close()


@pytest.mark.parametrize("text", ["17110", ":17110", "host:", "host:0", "host:70000", "host:x", "no.such.host.invalid:1"])
def test_endpoint_must_be_host_and_port(text):
    with pytest.raises(ValueError):
        parse_endpoint(text)


def test_the_vehicle_state_copy_is_the_same_datagram_to_the_zr10_emulator():
    sock, copy_sock = RecordingSocket(), RecordingSocket()
    out = UnrealPoseSender(("127.0.0.1", 17110), alt_offset_m=23.5, sock=sock, copies=(("127.0.0.1", 17999),),
                           copy_socks=(copy_sock,))
    out(40_000, rest_state())
    [(copy, copy_to)], [(pose, pose_to)] = copy_sock.sent, sock.sent
    assert (copy_to, pose_to) == (("127.0.0.1", 17999), ("127.0.0.1", 17110))
    assert copy == pose             # same session, sequence, board time and attitude
    assert (out.stats.sent, out.stats.copies_sent) == (1, 1)
    assert "copies 1 sent" in out.summary()


def test_a_dead_copy_receiver_only_counts_dropped_copies_never_a_dropped_pose():
    class ResetOnNextSend(RecordingSocket):
        """Windows: an ICMP port unreachable from one send surfaces as a reset on the socket's next send."""

        def __init__(self, dead_port):
            super().__init__()
            self.dead_port, self.pending = dead_port, False

        def sendto(self, data, endpoint):
            if self.pending:
                self.pending = False
                raise ConnectionResetError
            self.pending = endpoint[1] == self.dead_port
            return super().sendto(data, endpoint)

    sock, copy_sock = ResetOnNextSend(17999), ResetOnNextSend(17999)
    out = UnrealPoseSender(("127.0.0.1", 17110), alt_offset_m=23.5, sock=sock, copies=(("127.0.0.1", 17999),),
                           copy_socks=(copy_sock,))
    for k in range(4):
        out(k * POSE_PERIOD_US, rest_state())
    assert (out.stats.sent, out.stats.dropped) == (4, 0)
    assert (out.stats.copies_sent, out.stats.copies_dropped) == (2, 2)


def test_each_copy_has_its_own_non_blocking_socket():
    out = UnrealPoseSender(("127.0.0.1", 17110), alt_offset_m=0.0, copies=(("127.0.0.1", 17999),))
    try:
        [copy_sock] = out._copy_socks
        assert copy_sock is not out._sock and copy_sock.getblocking() is False
    finally:
        out.close()
    assert copy_sock.fileno() == -1                 # closed with the sender
    with pytest.raises(ValueError):
        UnrealPoseSender(("127.0.0.1", 17110), alt_offset_m=0.0, sock=RecordingSocket(), copies=(("127.0.0.1", 1),),
                         copy_socks=())


def test_cli_vehicle_state_copy_with_and_without_the_unreal_pose():
    parse = build_parser().parse_args
    base = ["--device", "COM11", "--model-dir", "P"]
    with_unreal = unreal_pose_option(parse([*base, "--unreal-pose", "127.0.0.1:17110", "23.5",
                                            "--vehicle-state-copy", "localhost:17999"]))
    alone = unreal_pose_option(parse([*base, "--vehicle-state-copy", "127.0.0.1:17999"]))
    try:
        assert (with_unreal.endpoint, with_unreal.copies) == (("127.0.0.1", 17110), (("127.0.0.1", 17999),))
        assert (alone.endpoint, alone.copies, alone.alt_offset_m) == (("127.0.0.1", 17999), (), 0.0)
        assert alone.summary().startswith(", ZR10 vehicle_state")
    finally:
        with_unreal.close()
        alone.close()
    with pytest.raises(SystemExit) as exit_info:
        parse([*base, "--vehicle-state-copy", "nohost"])
    assert exit_info.value.code == EXIT_BAD_SETUP
