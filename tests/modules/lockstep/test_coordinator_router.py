"""A routed fleet's ports: router in the tick path, queues and packet records (step 10b).

``lockstep_fleet.md``, Router and delivery and Evidence (Tests (10b)):
the tick order, overload, a packet split across replies, no overtaking,
pending bytes at parking, latency with source-side carry-over, and no
delivery to an unadmitted or retired receiver (the gate rejects a fixture
that claims one, of any generation, and one that defines an id twice).
Point-to-point bytes move by the byte rules across admission, retirement
and DONE boundaries; the records only observe them. Driven without processes (``router_doubles``).
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib

import pytest

from navpy.sim.lockstep.coordinator_mavlink import ROUTER_LINK
from navpy.sim.lockstep.coordinator_packets import TERMINALS
from navpy.sim.lockstep.coordinator_queues import FIFO_CAP, MAX_BLOCK
from navpy.sim.lockstep.coordinator_types import CoordinatorFault
from navpy.sim.lockstep.packet_journal import PacketJournal
from tests.modules.lockstep.router_doubles import (
    BACKEND,
    ROUTER,
    PortsHarness,
    Sources,
    routed_manifest,
    serial0,
    serial1,
)


@pytest.fixture
def run():
    return PortsHarness()


def _learned(run, src: Sources, backend) -> None:
    """Tick 1: every SITL's AP heartbeat on serial1, and the backend's."""
    run.tick(1, egress={serial1(i): src.heartbeat(i, 1) for i in (1, 2, 3)}, admitted=[backend],
             done=[(backend, ROUTER, src.heartbeat(255, 190))])


def test_sitl_serial1_routes_first_then_the_backend_and_nothing_overtakes(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    hb = {i: src.heartbeat(i, 1) for i in (1, 2, 3)}
    gcs, param = src.heartbeat(255, 190), src.param((255, 190), 2, 1)
    delivered, ingress = run.tick(1, egress={serial1(i): hb[i] for i in (1, 2, 3)}, admitted=[backend],
                                  done=[(backend, ROUTER, gcs + param)])
    # Step 1, endpoint order: the backend's TICK 1 carries every SITL's heartbeat.
    assert delivered == {ROUTER: hb[1] + hb[2] + hb[3]}
    # Step 4: the backend's bytes join each serial1 queue behind step 1's. The
    # targeted PARAM_REQUEST_READ reaches only SITL 2, which has seen 2/1.
    assert ingress[serial1(1)] == hb[2] + hb[3] + gcs
    assert ingress[serial1(2)] == hb[1] + hb[3] + gcs + param
    assert ingress[serial1(3)] == hb[1] + hb[2] + gcs
    assert all(ingress[serial0(i)] == b"" for i in (1, 2, 3))
    journal = run.journal()
    assert journal.problems() == []
    routed = [event for _k, event in run.events("routed")]
    assert [event[2] for event in routed] == [3, 3, 3, 3, 1]  # copies per source, in routing order


def test_serial0_is_point_to_point_through_its_navpys_tick_and_queue(run):
    src = Sources()
    navpys = {i: run.roster.add(i, serial0(i)) for i in (1, 2, 3)}
    status = {i: src.heartbeat(i, 1) for i in (1, 2, 3)}
    companion = {i: src.heartbeat(i, 191) for i in (1, 2, 3)}
    delivered, ingress = run.tick(1, egress={serial0(i): status[i] for i in (1, 2, 3)},
                                  admitted=list(navpys.values()),
                                  done=[(navpys[i], serial0(i), companion[i]) for i in (1, 2, 3)])
    assert delivered == {serial0(i): status[i] for i in (1, 2, 3)}
    assert {d: data for d, data in ingress.items() if data} == {serial0(i): companion[i] for i in (1, 2, 3)}
    receivers = [event[2] for _k, event in run.events("delivered")]
    # TICK 1 first (step 2), then reply 1 (step 5).
    assert receivers == [[i, 1] for i in (1, 2, 3)] + [serial0(i) for i in (1, 2, 3)]
    assert run.journal().problems() == []


def test_sustained_overload_carries_over_in_order_bounded_by_the_block_and_never_drops(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    sent, received, pending = b"", b"", []
    for k in range(2, 12):  # ~80 KB a tick for 5 ticks: more than one block each
        load = b"".join(src.blob((255, 190), 300 * k + n) for n in range(300)) if k < 7 else b""
        sent += load
        _, ingress = run.tick(k, admitted=[backend], done=[(backend, ROUTER, load)])
        assert len(ingress[serial1(2)]) <= MAX_BLOCK
        received += ingress[serial1(2)]
        pending.append(run.records[-1]["pending"][serial1(2)])
    assert pending[3] > pending[0] > 0  # a backlog builds while the load lasts
    assert pending[-1] == 0 and received == sent  # then everything arrives, in order
    # Admission respects the firmware FIFO: a nearly full FIFO takes only its space.
    last = src.blob((255, 190), 2)
    _, ingress = run.tick(12, admitted=[backend], done=[(backend, ROUTER, last)],
                          fifo={serial1(2): FIFO_CAP - 100})
    assert len(ingress[serial1(2)]) == 100
    run.park(13)  # the rest is undelivered, with its progress; nothing was dropped
    assert [e[2:] for _k, e in run.events("undelivered")] == [[100, len(last) - 100]]
    assert run.events("discarded") == [] and run.events("router_dropped") == []
    assert run.journal().problems() == []


def test_a_packet_split_across_replies_is_delivered_at_its_last_fragment(run):
    src = Sources()
    navpy = run.roster.add(2, serial0(2))
    packet = src.assign((2, 191), 1)
    for k, fifo in ((1, FIFO_CAP - 10), (2, FIFO_CAP - 10), (3, 0)):
        run.tick(k, admitted=[navpy], done=[(navpy, serial0(2), packet)] if k == 1 else [],
                 fifo={serial0(2): fifo})
    (emitted,) = [e for _k, e in run.events("emitted") if e[1][0] == 2]
    packet_id = emitted[1]
    admitted = [(k, e[2]) for k, e in run.events("admitted") if e[1] == packet_id]
    assert admitted == [(1, 10), (2, 10), (3, len(packet) - 20)]
    assert [(k, e[2]) for k, e in run.events("delivered") if e[1] == packet_id] == [(3, serial0(2))]
    assert emitted[2] == [0, len(packet)] and emitted[7]["uid"] == [2, 7, 1]
    assert run.journal().problems() == [] and run.journal().latency(packet_id) == 2


def test_a_pending_fragment_at_parking_is_undelivered_with_its_progress(run):
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    first, second = src.heartbeat(1, 191), src.heartbeat(1, 191)
    run.tick(1, admitted=[navpy], done=[(navpy, serial0(1), first + second)],
             fifo={serial0(1): FIFO_CAP - 30})
    parked = run.park(2)
    rest = (first + second)[30:]
    assert parked["undelivered"] == {serial0(1): {"bytes": len(rest),
                                                 "sha256": hashlib.sha256(rest).hexdigest()}}
    undelivered = [e for _k, e in run.events("undelivered")]
    assert [e[2:] for e in undelivered] == [[30 - len(first), len(first) * 2 - 30]]
    assert run.journal().problems() == []
    assert run.ports.closing()["mavlink_counts"][serial0(1)]["to_sitl"]["messages"] == {"0:1.191>-": 1}


def test_latency_counts_from_emission_through_source_side_carry_over(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    narrow = {serial1(2): FIFO_CAP - 1000}  # 1000 bytes a reply: a slow port
    for k in range(2, 10):
        run.tick(k, admitted=[backend], fifo=narrow)
    backlog = b"".join(src.heartbeat(255, 190) for _ in range(238))  # 4998 bytes ahead
    target = src.assign((255, 190), 2, task_id=9)
    run.tick(10, admitted=[backend], done=[(backend, ROUTER, backlog + target)], fifo=narrow)
    for k in range(11, 16):
        run.tick(k, admitted=[backend], fifo=narrow)
    journal = run.journal()
    (source,) = [e[1] for k, e in run.events("emitted") if k == 10 and e[4] == 25106]
    (copy_id,) = [e[1] for _k, e in run.events("copy") if e[2] == source and e[1][2] == serial1(2)]
    fragments = [(k, e[2]) for k, e in run.events("admitted") if e[1] == copy_id]
    assert fragments == [(14, 2), (15, len(target) - 2)]  # behind the backlog, split over two replies
    assert journal.latency(copy_id) == 5  # counted from DONE 10, not from reaching the queue head
    assert journal.problems() == []


def test_bytes_for_an_unadmitted_or_retired_receiver_are_discarded_never_delivered(run):
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    backend = run.roster.add(BACKEND, ROUTER)
    status = [src.heartbeat(1, 1) for _ in range(3)]
    delivered, _ = run.tick(1, egress={serial0(1): status[0], serial1(1): src.heartbeat(1, 1)})
    assert delivered == {}
    assert run.records[-1]["discarded"] == {serial0(1): len(status[0]), ROUTER: len(src.heartbeat(1, 1))}
    delivered, _ = run.tick(2, egress={serial0(1): status[1]}, admitted=[navpy, backend])
    assert delivered == {serial0(1): status[1]}
    navpy.retired_at = 2
    run.tick(3, egress={serial0(1): status[2]}, admitted=[backend])
    reasons = [(k, e[2]) for k, e in run.events("discarded")]
    assert reasons == [(1, "unadmitted"), (1, "unadmitted"), (3, "retired")]
    journal = run.journal()
    assert journal.problems() == []
    # A fixture that claims a delivery to the unadmitted NavPy (tick 1) or the
    # retired one (tick 3) fails the gate.
    for tick in (1, 3):
        forged = copy.deepcopy(run.records)
        record = forged[tick - 1]
        event = next(e for e in record["packets"] if e[0] == "discarded" and e[1][2] == serial0(1))
        event[:] = ["delivered", event[1], [1, 1]]
        assert any("did not go to" in problem for problem in PacketJournal(forged).problems())


def test_drain_egress_is_discarded_and_tallied_never_delivered(run):
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    backend = run.roster.add(BACKEND, ROUTER)
    run.tick(1, admitted=[navpy, backend])
    record = run.drain(2, [(navpy, serial0(1), src.heartbeat(1, 191)),
                           (backend, ROUTER, src.heartbeat(255, 190))])
    assert [e[0] for e in record["packets"]] == ["emitted", "discarded"] * 2
    assert {e[2] for e in record["packets"] if e[0] == "discarded"} == {"drain"}
    assert record["drain_egress"][0][:3] == [1, serial0(1), len(src.heartbeat(1, 191))]
    counts = run.ports.closing()["mavlink_counts"]
    assert counts[serial0(1)]["drain_to_sitl"]["messages"] == {"0:1.191>-": 1}
    assert counts[ROUTER_LINK]["drain_to_sitl"]["messages"] == {"0:255.190>-": 1}


def test_the_tally_derives_from_the_records(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    run.tick(2, egress={serial1(1): b"boot text\r\n" + src.param((1, 1), 9, 1)}, admitted=[backend],
             done=[(backend, ROUTER, src.param((255, 190), 9, 1))])
    counts = run.ports.closing()["mavlink_counts"]
    assert counts[serial1(1)]["router_skipped_bytes"]["skipped_bytes"] == len(b"boot text\r\n")
    # 1/1 -> 9/1 reaches only the sniffer (the backend); 255/190 -> 9/1 nobody.
    assert counts[ROUTER_LINK]["routed_out"]["messages"]["20:1.1>9"] == 1
    assert counts[ROUTER_LINK]["router_dropped"]["messages"] == {"20:255.190>9": 1}
    assert counts[serial1(2)]["to_sitl"]["messages"] == {"0:1.1>-": 1, "0:3.1>-": 1, "0:255.190>-": 1}


def test_every_packet_must_end_and_an_incomplete_frame_is_journaled(run):
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    whole = src.heartbeat(1, 191)
    run.tick(1, admitted=[navpy], done=[(navpy, serial0(1), whole + whole[:5])], fifo={serial0(1): FIFO_CAP})
    with pytest.raises(CoordinatorFault, match="never ended"):
        run.ports.closing()
    run.park(2)
    assert run.ports.closing()["held"] == {f"1/1/{serial0(1)}": {
        "bytes": 5, "sha256": hashlib.sha256(whole[:5]).hexdigest()}}


def test_a_routed_manifest_keeps_serial1_with_the_coordinator():
    plan = routed_manifest()
    with pytest.raises(ValueError, match="owns"):
        dataclasses.replace(plan, roles={**plan.roles, "rogue": dataclasses.replace(
            plan.roles["backend"], devices=(serial1(1),))})
    with pytest.raises(ValueError, match="SITL port"):
        dataclasses.replace(plan, router=serial1(2))


# --- point-to-point bytes move by the byte rules; records only observe them ---


def test_a_prefix_before_admission_is_discarded_and_only_the_suffix_is_delivered(run):
    src = Sources()
    navpy = run.roster.add(1, serial0(1))
    packet = src.heartbeat(1, 1)
    delivered, _ = run.tick(1, egress={serial0(1): packet[:7]})  # NavPy 1 not admitted yet
    assert delivered == {} and run.records[-1]["discarded"] == {serial0(1): 7}
    delivered, _ = run.tick(2, egress={serial0(1): packet[7:]}, admitted=[navpy])
    assert delivered == {serial0(1): packet[7:]}  # never the discarded prefix
    assert [e[2] for _k, e in run.events("discarded")] == ["unadmitted"]
    assert run.events("delivered") == [] and run.journal().problems() == []


def test_a_packet_across_a_retirement_or_a_new_generation_is_never_delivered(run):
    src = Sources()
    first = run.roster.add(1, serial0(1))
    packet, other = src.heartbeat(1, 1), src.heartbeat(1, 1)
    delivered, _ = run.tick(1, egress={serial0(1): packet[:7]}, admitted=[first])
    assert delivered == {serial0(1): packet[:7]}  # the prefix goes out in its own tick
    first.retired_at = 1
    delivered, _ = run.tick(2, egress={serial0(1): packet[7:] + other[:5]})
    assert delivered == {}
    second = run.roster.add(1, serial0(1), generation=2)
    delivered, _ = run.tick(3, egress={serial0(1): other[5:]}, admitted=[second])
    assert delivered == {serial0(1): other[5:]}
    assert [e[2] for _k, e in run.events("discarded")] == ["retired", "retired"]
    assert run.events("delivered") == [] and run.journal().problems() == []


def test_a_prefix_written_in_done_10_goes_out_in_reply_10(run):
    src = Sources()
    navpy = run.roster.add(2, serial0(2))
    packet = src.assign((2, 191), 1)
    for k in range(1, 10):
        run.tick(k, admitted=[navpy])
    _, ingress = run.tick(10, admitted=[navpy], done=[(navpy, serial0(2), packet[:9])])
    assert ingress[serial0(2)] == packet[:9]  # an empty queue: the bytes go now
    assert run.events("emitted") == []  # not a packet yet
    _, ingress = run.tick(11, admitted=[navpy], done=[(navpy, serial0(2), packet[9:])])
    assert ingress[serial0(2)] == packet[9:]
    (emitted,) = [e for k, e in run.events("emitted") if k == 11]
    admitted = [(k, e[2:]) for k, e in run.events("admitted") if e[1] == emitted[1]]
    assert admitted == [(11, [9, 10]), (11, [len(packet) - 9])]  # reply 10's share, then reply 11's
    assert [(k, e[2]) for k, e in run.events("delivered")] == [(11, serial0(2))]
    assert run.journal().problems() == []
    to_sitl = run.ports.closing()["mavlink_counts"][serial0(2)]["to_sitl"]
    assert to_sitl == {"messages": {"25106:2.191>1": 1}, "skipped_bytes": 0}  # no byte is garbage


def test_only_garbage_counts_as_skipped_around_a_split_packet(run):
    src = Sources()
    navpy = run.roster.add(2, serial0(2))
    garbage, packet = b"boot text\r\n", src.assign((2, 191), 1)
    _, ingress = run.tick(1, admitted=[navpy], done=[(navpy, serial0(2), garbage + packet[:9])])
    assert ingress[serial0(2)] == garbage + packet[:9]
    _, ingress = run.tick(2, admitted=[navpy], done=[(navpy, serial0(2), packet[9:] + b"tail")],
                          fifo={serial0(2): FIFO_CAP - 3})  # reply 2 takes 3 bytes, so the rest waits
    run.tick(3, admitted=[navpy])
    to_sitl = run.ports.closing()["mavlink_counts"][serial0(2)]["to_sitl"]
    assert to_sitl == {"messages": {"25106:2.191>1": 1}, "skipped_bytes": len(garbage) + len(b"tail")}
    assert run.journal().problems() == []


def test_a_retired_generations_prefix_moves_and_the_next_generations_packets_stay_exact(run):
    src = Sources()
    first = run.roster.add(1, serial0(1))
    run.tick(1, admitted=[first], done=[(first, serial0(1), src.heartbeat(1, 191)[:6])])
    first.retired_at = 1
    second = run.roster.add(1, serial0(1), generation=2)
    whole = src.heartbeat(1, 191)
    _, ingress = run.tick(2, admitted=[second], done=[(second, serial0(1), whole)])
    assert ingress[serial0(1)] == whole
    assert [(e[1][:2], e[2]) for _k, e in run.events("delivered")] == [([1, 2], serial0(1))]
    run.park(3)
    assert set(run.ports.closing()["held"]) == {f"1/1/{serial0(1)}"}
    assert run.journal().problems() == []


# --- the gate: full receiver identity, ids defined once ---


def _generations_run(run):
    src = Sources()
    first = run.roster.add(1, serial0(1))
    run.tick(1, egress={serial0(1): src.heartbeat(1, 1)}, admitted=[first])
    first.retired_at = 1
    second = run.roster.add(1, serial0(1), generation=2)
    run.tick(2, egress={serial0(1): src.heartbeat(1, 1)}, admitted=[second])
    assert run.journal().problems() == []
    assert run.records[-1]["members"][0]["generation"] == 2
    return next(e for e in run.records[-1]["packets"] if e[0] == "delivered")


@pytest.mark.parametrize("generation", [1, 3], ids=["retired", "unadmitted"])
def test_a_forged_delivery_to_another_generation_of_a_ticked_ordinal_fails(run, generation):
    delivered = _generations_run(run)
    assert delivered[2] == [1, 2]
    delivered[2] = [1, generation]  # the records are the journal's: forge in place
    assert any("did not go to" in problem for problem in run.journal().problems())


def test_duplicate_emission_and_copy_ids_fail_the_gate(run):
    src = Sources()
    backend = run.roster.add(BACKEND, ROUTER)
    _learned(run, src, backend)
    emitted = next(e for e in run.records[0]["packets"] if e[0] == "emitted")
    copied = next(e for e in run.records[0]["packets"] if e[0] == "copy")
    for event in (emitted, copied):
        forged = copy.deepcopy(run.records)
        terminal = next(e for e in forged[0]["packets"] if e[0] in TERMINALS and e[1] == event[1])
        forged[0]["packets"] += [copy.deepcopy(event), copy.deepcopy(terminal)]  # A defined twice
        assert any("already defined" in problem for problem in PacketJournal(forged).problems())


def test_a_split_frame_that_fails_its_crc_counts_as_skipped_once_framing_decides(run):
    src = Sources()
    navpy = run.roster.add(2, serial0(2))
    corrupt = bytearray(src.heartbeat(2, 191))
    corrupt[-1] ^= 0xFF
    run.tick(1, admitted=[navpy], done=[(navpy, serial0(2), bytes(corrupt[:9]))])  # admitted, unclassified
    counts = run.ports.tally.totals()[serial0(2)]["to_sitl"]
    assert counts["skipped_bytes"] == 0
    run.tick(2, admitted=[navpy], done=[(navpy, serial0(2), bytes(corrupt[9:]))])
    counts = run.ports.closing()["mavlink_counts"][serial0(2)]["to_sitl"]
    assert counts == {"messages": {}, "skipped_bytes": len(corrupt)}
    assert run.events("emitted") == []


# --- framing that resolves in drain: last reply -> parking -> drain ---


def _prefix_in_the_last_reply(run, frame: bytes):
    """A serial0 owner still live into drain sends 9 bytes reply k_F takes; then parking."""
    navpy = run.roster.add(2, serial0(2))
    _, ingress = run.tick(1, admitted=[navpy], done=[(navpy, serial0(2), frame[:9])])
    assert ingress[serial0(2)] == frame[:9]
    parked = run.park(2)
    assert parked["undelivered"] == {} and parked["packets"] == []
    run.drain(3, [(navpy, serial0(2), frame[9:])])
    return run.ports.closing()["mavlink_counts"][serial0(2)]


def test_a_valid_packet_resolved_in_drain_keeps_the_admission_of_its_prefix(run):
    packet = Sources().assign((2, 191), 1)
    counts = _prefix_in_the_last_reply(run, packet)
    (emitted,) = [e for _k, e in run.events("emitted")]
    assert [(k, e[2:]) for k, e in run.events("admitted")] == [(3, [9, 1])]  # reply 1's share
    assert [(k, e[2]) for k, e in run.events("discarded") if e[1] == emitted[1]] == [(3, "drain")]
    assert counts["to_sitl"] == {"messages": {}, "skipped_bytes": 0}
    assert counts["drain_to_sitl"] == {"messages": {"25106:2.191>1": 1}, "skipped_bytes": 0}
    assert run.journal().problems() == []


def test_a_bad_crc_frame_resolved_in_drain_splits_its_bytes_by_where_they_went(run):
    corrupt = bytearray(Sources().heartbeat(2, 191))
    corrupt[-1] ^= 0xFF
    counts = _prefix_in_the_last_reply(run, bytes(corrupt))
    assert run.events("emitted") == []
    assert counts["to_sitl"]["skipped_bytes"] == 9  # admitted before parking
    assert counts["drain_to_sitl"]["skipped_bytes"] == len(corrupt) - 9  # written in drain


def test_a_bad_crc_frame_resolved_over_two_drain_ticks_counts_each_write_in_its_phase(run):
    corrupt = bytearray(Sources().heartbeat(2, 191))
    corrupt[-1] ^= 0xFF
    assert len(corrupt) == 21
    navpy = run.roster.add(2, serial0(2))
    run.tick(1, admitted=[navpy], done=[(navpy, serial0(2), bytes(corrupt[:9]))])
    run.park(2)
    run.drain(3, [(navpy, serial0(2), bytes(corrupt[9:14]))])  # framing cannot resolve yet
    run.drain(4, [(navpy, serial0(2), bytes(corrupt[14:]))])
    closing = run.ports.closing()
    counts = closing["mavlink_counts"][serial0(2)]
    assert counts["to_sitl"]["skipped_bytes"] == 9
    assert counts["drain_to_sitl"]["skipped_bytes"] == 12
    assert closing["stream_bytes"][serial0(2)] == {"admitted": {"skipped": 9}, "drain": {"skipped": 12}}
