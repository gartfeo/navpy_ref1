"""The UART and router contracts, pinned on their reference models."""

from __future__ import annotations

import dataclasses

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from navpy.sim.lockstep.router_entries import load_entry_table
from navpy.sim.lockstep.router_framer import NO_TARGET, RouterFramer, RouterPacket
from navpy.sim.lockstep.router_model import SITL_FILTER, EndpointFilter, ReferenceRouter, Route
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
    elif kind == "image":
        link.encapsulated_data_send(0, bytes(253))
    else:
        link.param_request_read_send(target["system"], target["component"], b"X", -1)
    (segment,) = RouterFramer(load_entry_table()).feed(sink.drain())
    return segment.packet


def test_the_router_learns_endpoints_and_forwards_targeted_and_broadcast():
    router = ReferenceRouter(["sitl1", "sitl2", "backend"])
    router.route("sitl1", _message("heartbeat", (1, 1)))
    router.route("sitl2", _message("heartbeat", (2, 1)))
    # A broadcast never goes back to where it came from.
    assert router.route("backend", _message("heartbeat", (255, 190))).destinations == ("sitl1", "sitl2")
    # Targeted: only the endpoint that has seen the target.
    assert router.route("backend", _message("param", (255, 190), system=2, component=1)).destinations == ("sitl2",)
    assert router.route("backend", _message("param", (255, 190), system=2, component=0)).destinations == ("sitl2",)
    # Unknown target: dropped and counted, not broadcast.
    assert router.route("backend", _message("param", (255, 190), system=9, component=1)).destinations == ()
    assert router.dropped_unknown_target == 1
    # The GCS endpoint has seen sysid 255, so as the sniffer it gets traffic
    # targeted between vehicles too.
    assert router.route("sitl1", _message("param", (1, 1), system=2, component=1)).destinations == (
        "sitl2", "backend",
    )


def test_a_forwarded_echo_is_not_sent_to_an_endpoint_that_saw_its_source():
    # AP 1 forwards its companion's message (src 1/191) out of serial1; the
    # router must not send it back to an endpoint that already carried 1/191.
    router = ReferenceRouter(["sitl1", "sitl2", "backend"])
    router.route("sitl1", _message("heartbeat", (1, 191)))
    assert router.route("sitl2", _message("heartbeat", (1, 191))).destinations == ("backend",)


def _filtered_router() -> ReferenceRouter:
    router = ReferenceRouter(["sitl1", "sitl2", "backend"], filters={
        "sitl1": SITL_FILTER, "sitl2": SITL_FILTER})
    for endpoint, source in (("sitl1", (1, 1)), ("sitl2", (2, 1)), ("backend", (255, 190))):
        router.route(endpoint, _message("heartbeat", source))
    return router


def test_a_sitl_endpoint_gets_no_peer_autopilot_or_camera_traffic_and_the_gcs_gets_all():
    """radio_budget.md, item 2: BlockSrcCompOut 1,100 and BlockMsgIdOut 130,131 on each SITL
    endpoint, none on the backend."""
    router = _filtered_router()
    assert router.route("sitl1", _message("heartbeat", (1, 1))) == Route(("backend",), True)
    assert router.route("sitl1", _message("heartbeat", (1, 100))) == Route(("backend",), True)
    # The companion (NAVLINK, 191) and the GCS still reach every peer.
    assert router.route("sitl1", _message("heartbeat", (1, 191))).destinations == ("sitl2", "backend")
    assert router.route("backend", _message("heartbeat", (255, 0))).destinations == ("sitl1", "sitl2")
    # A targeted message from a blocked component is filtered too: the filter comes before the
    # target and sniffer rules.
    assert router.route("sitl1", _message("param", (1, 1), system=2, component=1)) == Route(
        ("backend",), True)
    # The companion's confirmation image goes to the ground station only.
    assert router.route("sitl1", _message("image", (1, 191))) == Route(("backend",), True)
    assert router.dropped_unknown_target == 0


def test_a_sitl_endpoint_drops_an_unknown_id_before_learning_its_source():
    """radio_budget.md, todo 96: SITL byte loss makes garbage frames with random headers; one
    from the GCS's 255/0 with an id outside the entry table (no CRC check) must not teach the
    endpoint that source, or the loop check drops every later GCS packet to that UAV."""
    router = _filtered_router()
    router.route("backend", _message("heartbeat", (255, 0)))
    garbage = RouterPacket(b"", 207, (255, 0), (NO_TARGET, NO_TARGET), known=False)
    assert garbage.msgid not in load_entry_table().entries
    assert router.route("sitl2", garbage) == Route((), True)
    assert router.dropped_unknown_target == 0
    assert router.route("backend", _message("param", (255, 0), system=2, component=1)).destinations == (
        "sitl2",)
    # A known id passes and learns as before; the backend has no incoming filter.
    assert router.route("sitl2", _message("heartbeat", (7, 191))).destinations == ("sitl1", "backend")
    assert router.route("backend", _message("param", (255, 0), system=7, component=0)).destinations == (
        "sitl2",)
    assert router.route("backend", dataclasses.replace(garbage, source=(255, 9))).destinations == (
        "sitl1", "sitl2")


def test_a_blocked_id_is_filtered_before_the_target_rules():
    # BlockMsgIdOut on a targeted id (not the SITL filter's ids, which have no target field).
    blocked = EndpointFilter(block_msg_id_out=(mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ,))
    router = ReferenceRouter(["sitl1", "sitl2", "backend"], filters={"sitl2": blocked})
    router.route("sitl2", _message("heartbeat", (2, 1)))
    assert router.route("backend", _message("param", (255, 190), system=2, component=1)) == Route((), True)
    assert router.dropped_unknown_target == 0


def test_a_packet_every_endpoint_filters_or_rejects_is_filtered_not_unknown():
    """With no destination, a packet some endpoint filtered is not ``msg_to_unknown``; the
    loop check comes before the filter."""
    router = ReferenceRouter(["sitl1", "sitl2", "backend"], filters={"sitl2": EndpointFilter(
        block_src_comp_out=(1,))})
    router.route("sitl1", _message("heartbeat", (1, 1)))
    router.route("backend", _message("heartbeat", (1, 1)))  # backend has now seen 1/1: loop-rejected
    assert router.route("sitl1", _message("heartbeat", (1, 1))) == Route((), True)
    assert router.dropped_unknown_target == 0
    # sitl2 has not seen sysid 9, the backend is no sniffer here: rejected everywhere.
    assert router.route("sitl1", _message("param", (1, 191), system=9, component=1)) == Route((), False)
    assert router.dropped_unknown_target == 1


def test_filters_must_name_router_endpoints():
    import pytest

    with pytest.raises(ValueError, match="unknown endpoints"):
        ReferenceRouter(["sitl1", "backend"], filters={"sitl9": EndpointFilter(block_src_comp_out=(1,))})


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


def test_the_forwarding_autopilot_passes_a_peers_targeted_navlink_to_its_companion():
    """A task request for sysid 2 arriving on SITL 2's serial1 reaches the
    NavPy that shares sysid 2 on serial0 (real ArduPilot forwards it: the
    10f journals); a parameter read for SITL 2's autopilot stays local."""
    from pymavlink.dialects.v20 import ardupilotmega as mavlink

    from tests.modules.lockstep.lockstep_autopilot import ReferenceAutopilot

    autopilot = ReferenceAutopilot(sys_id=2, forward=True, heartbeats=frozenset())
    peer, wire = encoder(1, 191)
    peer.send(mavlink.MAVLink_task_assign_request_message(7, 3, 0, 2000, 2, 1, 0, 0, 0, 0, 0.0))
    peer.param_request_read_send(2, mavlink.MAV_COMP_ID_AUTOPILOT1, b"AAS_NAV_CGT", -1)
    serial0, serial1 = autopilot.step(0, [b"", wire.drain()])
    forwarded = [(m.get_type(), m.get_srcSystem()) for m in StreamDecoder().feed(serial0)]
    assert forwarded == [("TASK_ASSIGN_REQUEST", 1)]
    assert "PARAM_VALUE" in {m.get_type() for m in StreamDecoder().feed(serial1)}  # answered, not forwarded
