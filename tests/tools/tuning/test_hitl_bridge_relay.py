"""Windows relay of the HITL link (tools/tuning/hitl_bridge/relay.py, relay_endpoint.py) with scripted transports.

The case from the review: after the WSL bridge is gone, WSL's localhost forwarding keeps
accepting on Windows and closes every connection at once. Such a connection must not count
as the bridge being back (no "open" line, the gone timer keeps running), the relay must not
spin on it, and bytes written into it are lost, not forwarded.
"""
import time
from collections import deque

from tools.tuning.hitl_bridge.relay import Relay
from tools.tuning.hitl_bridge.relay_endpoint import CONFIRM_AFTER_S, DirectionStats, Endpoint

READ_PAUSE_S = 0.002                    # a scripted read blocks this long, like a transport's read timeout
RETRY_S = 0.05
GONE_S = 0.3
CONFIRM_S = 0.05


class ScriptedTransport:
    """Each open() starts the next scripted connection (a list of read results); once the script
    is used up every connection is a phantom: open succeeds, the first read finds it closed."""

    def __init__(self, connections=(), idle=False) -> None:
        self.connections = deque(deque(c) for c in connections)
        self.idle = idle                # a device that is always there and quiet
        self.current: deque = deque()
        self.open_times: list[float] = []
        self.written = bytearray()

    def open(self) -> None:
        self.open_times.append(time.monotonic())
        if self.connections:
            self.current = self.connections.popleft()
        elif not self.idle:
            self.current = deque([ConnectionResetError("TCP peer closed the connection")])

    def read(self) -> bytes:
        time.sleep(READ_PAUSE_S)
        item = self.current.popleft() if self.current else b""
        if isinstance(item, Exception):
            raise item
        return item

    def write(self, data: bytes) -> None:
        self.written += data

    def close(self) -> None:
        pass

    def shutdown(self) -> None:
        pass


def endpoint(transport, log, clock=time.monotonic, name="bridge 127.0.0.1:9620") -> Endpoint:
    return Endpoint(name, transport, RETRY_S, log, clock=clock, confirm_after_s=CONFIRM_S)


def test_a_connection_counts_after_its_first_byte_or_the_confirm_time_not_before():
    now = [0.0]
    said = []
    side = endpoint(ScriptedTransport(), said.append, clock=lambda: now[0])
    assert side.try_open() is None and side.up and not side.confirmed
    side.note_read(False)                               # nothing yet, too early
    assert not side.confirmed and said == []
    now[0] = CONFIRM_S
    side.note_read(False)                               # quiet, but alive long enough
    assert side.confirmed and side.opens == 1 and said == ["relay: bridge 127.0.0.1:9620 open (#1)"]
    side.drop("peer closed")
    assert side.losses == 1 and side.down_since == CONFIRM_S and not side.closed_without_data
    assert side.try_open() is None
    side.note_read(True)                                # the first byte confirms at once
    assert side.opens == 2 and side.down_since is None
    assert CONFIRM_AFTER_S == 1.0


def test_a_connection_that_closes_before_it_counts_is_not_an_open_and_its_bytes_are_lost():
    now = [10.0]
    said = []
    side = endpoint(ScriptedTransport(), said.append, clock=lambda: now[0])
    side.down_since = 5.0                               # the bridge was lost at t = 5 s
    assert side.try_open() is None
    assert side.write(b"\xfd" * 40)                     # device bytes go into the tentative connection
    side.drop("TCP peer closed the connection")
    assert said == [] and side.opens == 0 and side.losses == 0
    assert (side.tentative_closes, side.lost_bytes, side.closed_without_data) == (1, 40, True)
    assert side.down_since == 5.0                       # the outage continues


def test_the_summary_counts_bytes_into_unconfirmed_connections_as_dropped():
    relay = Relay(endpoint(ScriptedTransport(), print, name="COM11"), endpoint(ScriptedTransport(), print),
                  bridge_gone_s=GONE_S)
    relay.to_bridge = DirectionStats(bytes=1000, chunks=10, dropped_bytes=5)
    relay.bridge.lost_bytes = 40
    text = relay.summary()
    assert "device->bridge 960 B forwarded in 10 writes, 45 B dropped" in text
    assert "0 closed without data" in text


def test_after_the_bridge_is_gone_phantom_connections_neither_spin_nor_delay_the_exit():
    said = []
    bridge_transport = ScriptedTransport(connections=[[b"\x01", ConnectionResetError("bridge ended")]])
    bridge = endpoint(bridge_transport, said.append)
    device = endpoint(ScriptedTransport(idle=True), said.append, name="COM11")
    relay = Relay(device, bridge, bridge_gone_s=GONE_S, log=said.append)
    relay.start()
    try:
        assert relay.stop.wait(5.0), "the relay did not end"
        ended = time.monotonic()
    finally:
        relay.close()
    assert "unreachable" in relay.reason
    lost_at = bridge.down_since
    phantom_opens = bridge_transport.open_times[1:]     # after the one real connection
    # the exit comes GONE_S after the loss, however many phantoms were accepted meanwhile
    assert len(phantom_opens) >= 2 and GONE_S <= ended - lost_at < GONE_S + 1.0
    # each phantom is followed by the retry wait instead of an immediate reconnect (a spinning
    # relay reconnects after one read, READ_PAUSE_S; half the wait allows Windows' 15.6 ms clock ticks)
    gaps = [b - a for a, b in zip(phantom_opens, phantom_opens[1:])]
    assert min(gaps) >= RETRY_S / 2
    assert [line for line in said if "bridge" in line and " open " in line] == [
        "relay: bridge 127.0.0.1:9620 open (#1)"]
    assert bridge.opens == 1 and bridge.losses == 1 and bridge.tentative_closes == len(phantom_opens)


def test_with_a_timer_the_relay_stamps_each_frame_without_changing_what_it_forwards():
    from tests.tools.tuning.test_hitl_bridge_relay_timing import actuator_frame, state_frame
    from tools.tuning.hitl_bridge.relay_timing import FrameTimer
    frame_up, frame_down = actuator_frame(5_000), state_frame(5_000)
    device_transport = ScriptedTransport(connections=[[frame_up]], idle=True)
    bridge_transport = ScriptedTransport(connections=[[b""] * 25 + [frame_down]], idle=True)   # answers after it
    timer = FrameTimer()
    bridge = endpoint(bridge_transport, print)
    device_read = device_transport.read

    def read_once_the_bridge_is_up() -> bytes:  # a frame read while the bridge is down is dropped, by design
        deadline = time.monotonic() + 5.0
        while not bridge.up and time.monotonic() < deadline:
            time.sleep(READ_PAUSE_S)
        return device_read()

    device_transport.read = read_once_the_bridge_is_up
    relay = Relay(endpoint(device_transport, print, name="COM11"), bridge,
                  bridge_gone_s=GONE_S, log=print, timer=timer)
    relay.start()
    deadline = time.monotonic() + 5.0
    while not timer.stats.pc_round_trip_ms and time.monotonic() < deadline:
        time.sleep(READ_PAUSE_S)
    relay.close()
    assert bytes(bridge_transport.written) == frame_up and bytes(device_transport.written) == frame_down
    assert len(timer.stats.pc_round_trip_ms) == 1 and "1 frames answered" in relay.summary()
