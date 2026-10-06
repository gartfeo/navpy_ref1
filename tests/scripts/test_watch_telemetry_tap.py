"""The tap must be one-way, and must be able to say what it stopped.

Both properties are load-bearing. If anything the viewer sends reaches the
vehicle, the tap has not solved the problem it exists for -- a single
`REQUEST_DATA_STREAM` at 2 Hz is enough to drop the evaluator's 30 Hz position
stream and fail the run. And if the block count is not attributable per
message, a future "the viewer is harmless now" claim cannot be checked.
"""

from __future__ import annotations

import argparse
import errno
import socket
import threading
import time
from contextlib import ExitStack

import pytest

from scripts.watch_telemetry_tap import (
    TapCounters,
    message_id,
    message_label,
    run,
)

REQUEST_DATA_STREAM = 66
SET_MESSAGE_INTERVAL = 511


def _v1(msgid: int) -> bytes:
    """A MAVLink v1 frame header: magic, len, seq, sysid, compid, msgid."""
    return bytes([0xFE, 2, 7, 255, 190, msgid]) + b"\x00\x00"


def _v2(msgid: int) -> bytes:
    """A MAVLink v2 frame header, whose message id is a 24-bit field."""
    return (
        bytes([0xFD, 2, 0, 0, 7, 255, 190])
        + int(msgid).to_bytes(3, "little")
        + b"\x00\x00"
    )


def _free_ports(count: int) -> tuple[int, ...]:
    with ExitStack() as stack:
        probes = [
            stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            for _ in range(count)
        ]
        for probe in probes:
            probe.bind(("127.0.0.1", 0))
        return tuple(probe.getsockname()[1] for probe in probes)


class _SocketProxy:
    """Inject a specific socket fault while retaining real binds and cleanup."""

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return self.sock.__exit__(*args)

    def __getattr__(self, name):
        return getattr(self.sock, name)


def test_a_v1_frame_reports_its_message_id() -> None:
    assert message_id(_v1(REQUEST_DATA_STREAM)) == REQUEST_DATA_STREAM


def test_a_v2_frame_reports_its_three_byte_message_id() -> None:
    """511 does not fit in one byte, and it is the id that matters most here."""
    assert message_id(_v2(SET_MESSAGE_INTERVAL)) == SET_MESSAGE_INTERVAL


@pytest.mark.parametrize("frame", [b"", b"\xfe", b"not mavlink", b"\xfd\x01"])
def test_a_frame_that_is_not_mavlink_is_not_guessed_at(frame: bytes) -> None:
    """A wrong id in the evidence is worse than an honest `unparsed`."""
    assert message_id(frame) is None
    assert message_label(message_id(frame)) == "unparsed"


def test_the_two_stream_seizing_messages_are_named_not_numbered() -> None:
    """These are the ones a reader has to recognise without a lookup table."""
    assert message_label(REQUEST_DATA_STREAM) == "REQUEST_DATA_STREAM"
    assert message_label(SET_MESSAGE_INTERVAL) == "SET_MESSAGE_INTERVAL"


def test_blocked_frames_are_counted_per_message() -> None:
    counters = TapCounters()

    counters.record_block(_v1(REQUEST_DATA_STREAM))
    counters.record_block(_v1(REQUEST_DATA_STREAM))
    counters.record_block(_v2(SET_MESSAGE_INTERVAL))

    assert counters.blocked == 3
    assert counters.blocked_by_message == {
        "REQUEST_DATA_STREAM": 2,
        "SET_MESSAGE_INTERVAL": 1,
    }
    assert "REQUEST_DATA_STREAM" in counters.summary()


def test_the_summary_says_so_when_the_viewer_stayed_quiet() -> None:
    """`blocked 0 from the viewer:` with an empty list reads as truncated."""
    assert "blocked 0 from the viewer: none" in TapCounters().summary()


def test_the_vehicle_reaches_the_viewer_and_the_viewer_reaches_nothing() -> None:
    """The whole contract, over real sockets, with both directions LIVE at once.

    A mocked socket would pass whether or not the tap declines to send back.
    The two directions also have to OVERLAP: a leak only shows while the
    vehicle is transmitting too, which is the real condition -- telemetry
    streaming while the ground station talks back. An earlier version of this
    test drove the directions in sequence and stayed green against a
    deliberately leaky tap, so it proved nothing.

    The vehicle socket is BOUND, so a leak is RECEIVED here, not inferred.
    """
    listen_port, viewer_port, vehicle_port = _free_ports(3)
    options = argparse.Namespace(
        listen=listen_port,
        viewer=viewer_port,
        viewer_host="127.0.0.1",
        seconds=2.5,
    )
    counters: list[TapCounters] = []
    tap = threading.Thread(target=lambda: counters.append(run(options)))
    tap.start()

    stop = threading.Event()

    def stream_telemetry(sock: socket.socket) -> None:
        while not stop.is_set():
            sock.sendto(_v1(0), ("127.0.0.1", listen_port))
            time.sleep(0.02)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as viewer, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vehicle:
            vehicle.bind(("127.0.0.1", vehicle_port))
            vehicle.settimeout(0.75)
            viewer.bind(("127.0.0.1", viewer_port))
            viewer.settimeout(0.1)

            # Wait for a forwarded probe before the vehicle socket sends.
            # That socket must not inherit an ICMP reset from a startup race.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                for _ in range(30):
                    probe.sendto(_v1(0), ("127.0.0.1", listen_port))
                    try:
                        _, tap_address = viewer.recvfrom(65535)
                        break
                    except socket.timeout:
                        continue
                else:
                    pytest.fail("tap did not forward the startup probe")
            viewer.settimeout(3.0)

            streamer = threading.Thread(target=stream_telemetry, args=(vehicle,))
            streamer.start()
            try:
                # Talk back WHILE telemetry is still flowing.
                for _ in range(10):
                    viewer.sendto(_v1(REQUEST_DATA_STREAM), tap_address)
                    time.sleep(0.02)
                with pytest.raises(socket.timeout):
                    vehicle.recvfrom(65535)
            finally:
                stop.set()
                streamer.join(timeout=5.0)
    finally:
        tap.join(timeout=15.0)

    assert counters, "tap thread did not finish"
    result = counters[0]
    assert result.forwarded > 1
    assert result.blocked >= 1
    assert "REQUEST_DATA_STREAM" in result.blocked_by_message


# --- the viewer leaving must not take the tap down with it ------------------
#
# Forwarding is only half of what the tap does.  The other half is HOLDING the
# run's Mission Planner port so no ground station can reach the vehicle
# directly; a direct attach overwrites the evaluator's 30 Hz request with
# Mission Planner's 2 Hz default and was measured at FAIL 3.459 m against
# PASS 0.201 m.  So a tap that exits when its viewer quits converts "operator
# closed a window" into "the score is silently wrong" -- the failure it exists
# to prevent.
#
# These run on real sockets. Connected UDP reports a missing viewer through
# ICMP when the OS supplies it; the signal is not a delivery receipt.


def test_the_tap_outlives_its_viewer_and_keeps_the_port() -> None:
    """No viewer at all: the tap counts it, says so, and holds the port."""
    listen_port, vehicle_port, viewer_port = _free_ports(3)
    options = argparse.Namespace(
        listen=listen_port,
        viewer=viewer_port,  # nothing is ever bound here
        viewer_host="127.0.0.1",
        seconds=1.5,
    )
    stop = threading.Event()

    def stream_telemetry() -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vehicle:
            vehicle.bind(("127.0.0.1", vehicle_port))
            while not stop.is_set():
                vehicle.sendto(_v1(0), ("127.0.0.1", listen_port))
                time.sleep(0.02)

    results: list[TapCounters] = []
    tap = threading.Thread(target=lambda: results.append(run(options)))
    streamer = threading.Thread(target=stream_telemetry)
    streamer.start()
    tap.start()
    try:
        time.sleep(0.6)
        # Try the competing binds that defeated SO_REUSEADDR on Windows.
        for host in ("0.0.0.0", "127.0.0.1"):
            for reuse in (False, True):
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rival:
                    if reuse:
                        rival.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    with pytest.raises(OSError):
                        rival.bind((host, listen_port))
    finally:
        tap.join(timeout=10.0)
        stop.set()
        streamer.join(timeout=5.0)

    assert not tap.is_alive(), "the tap did not reach its own deadline"
    assert len(results) == 1, "tap thread did not return a result"
    result = results[0]
    # The OS reported the absent viewer while the tap stayed active.
    assert result.viewer_gone > 1, "OS supplied no loopback ICMP for viewer loss"
    assert f"viewer unreachable reports x{result.viewer_gone}" in result.summary()
    # `forwarded` counts frames
    # handed to the OS, and a UDP send to nobody succeeds, so this climbing is
    # exactly why the counter above is needed to tell the two apart.
    assert result.forwarded > 1


def test_a_viewer_that_comes_back_finds_the_stream_still_running() -> None:
    """The real sequence: watch, close the window, reopen it."""
    listen_port, vehicle_port, viewer_port = _free_ports(3)
    options = argparse.Namespace(
        listen=listen_port,
        viewer=viewer_port,
        viewer_host="127.0.0.1",
        seconds=4.0,
    )
    stop = threading.Event()

    def stream_telemetry() -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vehicle:
            vehicle.bind(("127.0.0.1", vehicle_port))
            while not stop.is_set():
                vehicle.sendto(_v1(0), ("127.0.0.1", listen_port))
                time.sleep(0.02)

    def open_viewer() -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", viewer_port))
        sock.settimeout(3.0)
        return sock

    results: list[TapCounters] = []
    tap = threading.Thread(target=lambda: results.append(run(options)))
    streamer = threading.Thread(target=stream_telemetry)
    # The viewer is bound BEFORE anything forwards.  Starting the tap first
    # would leave a window where frames go to an unbound port, and the ICMP
    # errors queued in that window would satisfy the assertion below on their
    # own -- letting a startup race pass as "the close was detected".
    viewer = open_viewer()
    tap.start()
    streamer.start()
    try:
        viewer.recvfrom(65535)      # watching, on a viewer that was never absent
        viewer.close()              # operator closes the window
        time.sleep(0.8)             # the tap keeps forwarding into the void
        reopened = open_viewer()    # operator reopens it
        try:
            reopened.recvfrom(65535)  # still live: raises on timeout
            # Let the tap reach its deadline WHILE this viewer is still bound.
            # Closing first would leave a second dead window running to the
            # deadline, and the errors from THAT would satisfy the assertion
            # below whether or not the deliberate gap was ever noticed.
            tap.join(timeout=10.0)
        finally:
            reopened.close()
    finally:
        tap.join(timeout=10.0)
        stop.set()
        streamer.join(timeout=5.0)

    assert not tap.is_alive()
    # So every counted error comes from the deliberate gap, not from startup.
    assert len(results) == 1, "tap thread did not return a result"
    assert results[0].viewer_gone > 0, "OS supplied no loopback ICMP for the gap"


def test_a_healthy_run_says_nothing_about_a_missing_viewer() -> None:
    """The counter must not add noise to the normal summary line."""
    counters = TapCounters()
    counters.record_forward(24)

    assert "unreachable" not in counters.summary()


def test_a_connected_viewer_has_no_false_absence_report() -> None:
    """Keep the viewer bound for the entire real-socket run."""
    listen_port, viewer_port = _free_ports(2)
    options = argparse.Namespace(
        listen=listen_port,
        viewer=viewer_port,
        viewer_host="127.0.0.1",
        seconds=1.0,
    )
    results: list[TapCounters] = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as viewer, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as vehicle:
        viewer.bind(("127.0.0.1", viewer_port))
        viewer.settimeout(2.0)
        tap = threading.Thread(target=lambda: results.append(run(options)))
        tap.start()
        try:
            for _ in range(20):
                vehicle.sendto(_v1(0), ("127.0.0.1", listen_port))
                time.sleep(0.025)
            viewer.recvfrom(65535)
            tap.join(timeout=5.0)
        finally:
            tap.join(timeout=5.0)

    assert not tap.is_alive()
    assert len(results) == 1
    assert results[0].forwarded > 0
    assert results[0].viewer_gone == 0


def test_busy_listen_port_reports_why_the_tap_cannot_protect_the_run() -> None:
    listen_port, viewer_port = _free_ports(2)
    options = argparse.Namespace(
        listen=listen_port, viewer=viewer_port, viewer_host="127.0.0.1", seconds=0.1,
    )
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as owner:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            owner.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        owner.bind(("0.0.0.0", listen_port))
        with pytest.raises(RuntimeError, match="cannot hold listen port"):
            run(options)


@pytest.mark.parametrize("viewer_host", ("127.0.0.1", "192.0.2.1"))
def test_viewer_cannot_use_the_tap_listen_port(viewer_host: str) -> None:
    (port,) = _free_ports(1)
    options = argparse.Namespace(
        listen=port, viewer=port, viewer_host=viewer_host, seconds=0.1,
    )
    with pytest.raises(ValueError, match="must differ"):
        run(options)


def test_unexpected_inbound_error_cannot_report_success(monkeypatch) -> None:
    import scripts.watch_telemetry_tap as tap_module

    listen_port, viewer_port = _free_ports(2)
    real_socket = socket.socket
    created = 0

    class BrokenInbound(_SocketProxy):
        def recvfrom(self, _size):
            raise OSError(errno.EBADF, "injected receive failure")

    def socket_factory(*args, **kwargs):
        nonlocal created
        created += 1
        sock = real_socket(*args, **kwargs)
        return BrokenInbound(sock) if created == 1 else sock

    monkeypatch.setattr(tap_module.socket, "socket", socket_factory)
    options = argparse.Namespace(
        listen=listen_port, viewer=viewer_port, viewer_host="127.0.0.1", seconds=0.1,
    )
    with pytest.raises(RuntimeError, match="inbound receive failed"):
        tap_module.run(options)


@pytest.mark.parametrize(
    ("fault_side", "error", "counter_name"),
    [
        ("send", OSError(errno.ENETUNREACH, "no route"), "viewer_gone"),
        ("send", BlockingIOError(errno.EWOULDBLOCK, "send busy"), "send_blocked"),
        ("recv", OSError(errno.ENETUNREACH, "no route"), "viewer_gone"),
        ("recv", OSError(errno.EHOSTUNREACH, "no host"), "viewer_gone"),
    ],
)
def test_transient_outbound_errors_keep_the_tap_bound(
    monkeypatch, fault_side: str, error: OSError, counter_name: str,
) -> None:
    import scripts.watch_telemetry_tap as tap_module

    listen_port, viewer_port = _free_ports(2)
    real_socket = socket.socket
    created = 0

    class FaultyOutbound(_SocketProxy):
        def send(self, _frame):
            if fault_side == "send":
                raise error
            return len(_frame)

        def recv(self, _size):
            if fault_side == "recv":
                raise error
            raise BlockingIOError()

    def socket_factory(*args, **kwargs):
        nonlocal created
        created += 1
        sock = real_socket(*args, **kwargs)
        return FaultyOutbound(sock) if created == 2 else sock

    monkeypatch.setattr(tap_module.socket, "socket", socket_factory)
    options = argparse.Namespace(
        listen=listen_port, viewer=viewer_port, viewer_host="127.0.0.1", seconds=0.3,
    )
    stop = threading.Event()

    def feed() -> None:
        with real_socket(socket.AF_INET, socket.SOCK_DGRAM) as vehicle:
            while not stop.is_set():
                vehicle.sendto(_v1(0), ("127.0.0.1", listen_port))
                time.sleep(0.01)

    streamer = threading.Thread(target=feed)
    streamer.start()
    try:
        result = tap_module.run(options)
    finally:
        stop.set()
        streamer.join(timeout=2.0)

    assert getattr(result, counter_name) > 0
    assert not streamer.is_alive()
