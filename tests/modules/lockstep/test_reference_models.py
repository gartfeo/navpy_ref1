"""The UART and router contracts, pinned on their reference models."""

from __future__ import annotations

from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from navpy.sim.lockstep.router_model import ReferenceRouter
from navpy.sim.lockstep.uart_model import StepUart


def _messages(count: int, src=(255, 190)) -> bytes:
    link, sink = encoder(*src)
    for seq in range(count):
        link.mission_request_int_send(1, 1, seq, 0)
    return sink.drain()


def test_an_oversized_burst_arrives_whole_and_in_order_over_io_ticks():
    uart = StepUart(read_buffer_bytes=64)
    burst = _messages(20)
    assert len(burst) > 64
    uart.enqueue(burst)
    received, io_ticks = b"", 0
    while uart.header().fifo_bytes or uart.header().read_buffer_bytes:
        uart.io_tick()
        io_ticks += 1
        received += uart.read(32)  # the parser reads less than the buffer holds
    assert received == burst
    assert io_ticks > 1


def test_a_message_split_across_ticks_parses_once_complete():
    data = _messages(1)
    decoder = StreamDecoder()
    assert decoder.feed(data[:7]) == []
    (message,) = decoder.feed(data[7:])
    assert message.seq == 0


def test_a_short_write_is_counted_as_an_egress_drop():
    uart = StepUart(write_buffer_bytes=10)
    data = _messages(1)
    taken = uart.write(data)
    uart.record_drop(len(data) - taken)
    assert taken == 10 and uart.header().egress_dropped_bytes == len(data) - 10
    assert uart.drain() == data[:10] and uart.txspace() == 10


def _message(kind: str, src: tuple[int, int], **target):
    link, sink = encoder(*src)
    if kind == "heartbeat":
        link.heartbeat_send(1, 3, 0, 0, 0)
    else:
        link.param_request_read_send(target["system"], target["component"], b"X", -1)
    (message,) = StreamDecoder().feed(sink.drain())
    return message


def test_the_router_learns_endpoints_and_forwards_targeted_and_broadcast():
    router = ReferenceRouter(["sitl1", "sitl2", "backend"])
    router.route("sitl1", _message("heartbeat", (1, 1)))
    router.route("sitl2", _message("heartbeat", (2, 1)))
    # A broadcast never goes back to where it came from.
    assert router.route("backend", _message("heartbeat", (255, 190))) == ["sitl1", "sitl2"]
    # Targeted: only the endpoint that has seen the target.
    assert router.route("backend", _message("param", (255, 190), system=2, component=1)) == ["sitl2"]
    assert router.route("backend", _message("param", (255, 190), system=2, component=0)) == ["sitl2"]
    # Unknown target: dropped and counted, not broadcast.
    assert router.route("backend", _message("param", (255, 190), system=9, component=1)) == []
    assert router.dropped_unknown_target == 1
    # The GCS endpoint has seen sysid 255, so as the sniffer it gets traffic
    # targeted between vehicles too.
    assert router.route("sitl1", _message("param", (1, 1), system=2, component=1)) == [
        "sitl2", "backend",
    ]


def test_a_forwarded_echo_is_not_sent_to_an_endpoint_that_saw_its_source():
    # AP 1 forwards its companion's message (src 1/191) out of serial1; the
    # router must not send it back to an endpoint that already carried 1/191.
    router = ReferenceRouter(["sitl1", "sitl2", "backend"])
    router.route("sitl1", _message("heartbeat", (1, 191)))
    assert router.route("sitl2", _message("heartbeat", (1, 191))) == ["backend"]


def test_admission_is_capped_per_io_tick_and_by_usable_space():
    from navpy.sim.lockstep.uart_model import MAX_ADMIT_PER_IO_TICK, READ_BUFFER_BYTES

    uart = StepUart()
    uart.enqueue(bytes(READ_BUFFER_BYTES + 5))
    assert uart.io_tick() == MAX_ADMIT_PER_IO_TICK
    assert uart.io_tick() == READ_BUFFER_BYTES - MAX_ADMIT_PER_IO_TICK
    assert uart.io_tick() == 0  # full: one byte stays free in the ring buffer
    assert uart.header().fifo_bytes == 5


def test_the_decoder_skips_boot_text_like_a_real_link():
    decoder = StreamDecoder()
    (message,) = decoder.feed(b"0 0 0 " + _messages(1))
    assert message.seq == 0
    assert decoder.skipped_bytes == 6
