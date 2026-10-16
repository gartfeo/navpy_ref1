"""NAVLINK traced hop by hop through packet records (``navlink_trace``, step 10f).

The records are the shapes ``coordinator_packets`` writes: NavPy 1 sends one
NAVLINK packet; ArduPilot 1 re-emits it on its serial1, the router copies it
to SITLs 2 and 3, and each ArduPilot re-emits it on its serial0 to its NavPy.
"""

from __future__ import annotations

from navpy.sim.lockstep.navlink_trace import HOPS, trace_navlink

UID = [1, 77, 5]
NAVLINK = 25004
FLEET = (1, 2, 3)
SENT = [1, 1, "lockstep:serial0.1", 0]
UP = ["sitl1", 1, "sitl1.serial1", 4]
COPY = {2: ["router", 1, "sitl2.serial1", 0], 3: ["router", 1, "sitl3.serial1", 1]}
DOWN = {2: ["sitl2", 1, "lockstep:serial0.2", 9], 3: ["sitl3", 1, "lockstep:serial0.3", 9]}


def emitted(packet_id, sha="a", msgid=NAVLINK, uid=UID, target=-1):
    fields = {} if uid is None else {"uid": uid}
    return ["emitted", packet_id, [0, 40], sha, msgid, [1, 191], [target, -1], fields]


def records(*, down_sha="a", deliver_to_2=True):
    return [
        {"k": 10, "packets": [emitted(SENT)]},
        {"k": 11, "packets": [["delivered", SENT, "lockstep:serial0.1"]]},
        {"k": 13, "packets": [emitted(UP), ["copy", COPY[2], UP], ["copy", COPY[3], UP], ["routed", UP, 2]]},
        {"k": 14, "packets": [["delivered", COPY[2], "sitl2.serial1"], ["delivered", COPY[3], "sitl3.serial1"]]},
        {"k": 16, "packets": [emitted(DOWN[2]), emitted(DOWN[3], sha=down_sha)]},
        {"k": 17, "packets": [["delivered", DOWN[2], [2, 1]]] if deliver_to_2 else []},
        {"parked": {"tick": 18, "packets": [["delivered", DOWN[3], [3, 1]]]}},
    ]


def test_each_receiver_gets_a_trace_through_every_hop_with_its_latency():
    to_2, to_3 = trace_navlink(records(), FLEET)
    assert (to_2.uid, to_2.msgid, to_2.sender, to_2.receiver) == (tuple(UID), NAVLINK, 1, 2)
    assert to_2.hops == tuple(zip(HOPS, (10, 11, 13, 14, 16, 17)))
    assert to_2.complete and to_2.latency_ticks == 7
    assert to_3.receiver == 3 and to_3.hops[-1] == ("navpy_delivered", 18)  # read from the parked record
    assert to_3.latency_ticks == 8


def test_a_forward_that_changed_the_bytes_stops_the_trace_short():
    _to_2, to_3 = trace_navlink(records(down_sha="b"), FLEET)
    assert to_3.hops[-1] == ("router_copy", 14)


def test_a_re_emission_in_the_tick_of_its_delivery_is_not_its_answer():
    same_tick = records()
    same_tick[1]["packets"].append(same_tick[2]["packets"].pop(0))  # UP emitted at tick 11
    to_2, _to_3 = trace_navlink(same_tick, FLEET)
    assert to_2.hops[-1] == ("sitl_serial0", 11)


def test_a_trace_that_stops_short_names_the_last_hop_it_reached():
    to_2, to_3 = trace_navlink(records(deliver_to_2=False), FLEET)
    assert to_2.hops[-1] == ("ap_serial0", 16)
    assert not to_2.complete and to_2.latency_ticks is None
    assert to_3.complete


def test_only_a_fleet_navpys_navlink_on_its_own_serial0_is_traced():
    other = [
        {"k": 20, "packets": [
            emitted([1, 1, "lockstep:serial0.1", 1], msgid=0, uid=None),  # its heartbeat
            emitted([4, 1, "lockstep:router", 0], uid=[4, 9, 1]),  # the backend's NAVLINK
            emitted(["sitl2", 1, "sitl2.serial1", 30], uid=[2, 8, 1]),  # an autopilot's copy
        ]},
    ]
    assert len(trace_navlink(records() + other, FLEET)) == 2
    assert trace_navlink(records(), (1,)) == []  # no peer to reach


def test_a_targeted_packet_is_traced_only_to_its_target():
    targeted = records()
    targeted[0]["packets"][0] = emitted(SENT, target=3)
    (to_3,) = trace_navlink(targeted, FLEET)
    assert to_3.receiver == 3 and to_3.complete
    for target in (1, 255):  # the sender itself, a non-NavPy system
        targeted[0]["packets"][0] = emitted(SENT, target=target)
        assert trace_navlink(targeted, FLEET) == []
    targeted[0]["packets"][0] = emitted(SENT, target=0)  # broadcast
    assert len(trace_navlink(targeted, FLEET)) == 2


def test_a_resend_under_the_same_uid_pairs_with_the_next_re_emission_in_order():
    """NavPy's ack resend keeps the UID but has its own bytes (its MAVLink
    sequence); each send is matched to the re-emission of its own bytes."""
    resent = [1, 1, "lockstep:serial0.1", 1]
    up2, down2 = ["sitl1", 1, "sitl1.serial1", 6], ["sitl2", 1, "lockstep:serial0.2", 11]
    copy2 = ["router", 1, "sitl2.serial1", 2]
    later = [
        {"k": 30, "packets": [emitted(resent, sha="c")]},
        {"k": 31, "packets": [["delivered", resent, "lockstep:serial0.1"]]},
        {"k": 33, "packets": [emitted(up2, sha="c"), ["copy", copy2, up2], ["routed", up2, 1]]},
        {"k": 34, "packets": [["delivered", copy2, "sitl2.serial1"]]},
        {"k": 36, "packets": [emitted(down2, sha="c")]},
        {"k": 37, "packets": [["delivered", down2, [2, 1]]]},
    ]
    first_2, first_3, second_2, second_3 = trace_navlink(records() + later, FLEET)
    assert first_2.hops == tuple(zip(HOPS, (10, 11, 13, 14, 16, 17)))
    assert second_2.hops == tuple(zip(HOPS, (30, 31, 33, 34, 36, 37)))
    assert first_3.complete and second_3.hops[-1] == ("ap_serial1", 33)  # no copy to SITL 3


def test_a_send_the_autopilot_dropped_does_not_borrow_a_later_resends_path():
    """ArduPilot forwards only the second of two sends under one UID: the
    first stops at SITL 1, the second keeps its own path."""
    resent, up2 = [1, 1, "lockstep:serial0.1", 1], ["sitl1", 1, "sitl1.serial1", 6]
    dropped = [
        {"k": 10, "packets": [emitted(SENT)]},
        {"k": 11, "packets": [["delivered", SENT, "lockstep:serial0.1"]]},
        {"k": 30, "packets": [emitted(resent, sha="c")]},
        {"k": 31, "packets": [["delivered", resent, "lockstep:serial0.1"]]},
        {"k": 33, "packets": [emitted(up2, sha="c")]},
    ]
    first, _first_3, second, _second_3 = trace_navlink(dropped, FLEET)
    assert first.hops[-1] == ("sitl_serial0", 11)
    assert second.hops[-1] == ("ap_serial1", 33)


def test_byte_identical_sends_of_one_uid_are_ambiguous():
    """NavPy 1's send at tick 10 reaches NavPy 2 but not 3; an unchanged
    retry (same bytes) at tick 30 reaches 3. No pairing can tell which send
    3 got, so both sends' traces are marked ambiguous."""
    retry, up2 = [1, 1, "lockstep:serial0.1", 1], ["sitl1", 1, "sitl1.serial1", 6]
    copy3, down3 = ["router", 1, "sitl3.serial1", 2], ["sitl3", 1, "lockstep:serial0.3", 11]
    lost_to_3 = records()
    lost_to_3[4]["packets"].pop()  # AP 3 dropped its forward of the first send
    lost_to_3[6]["parked"]["packets"].clear()
    lost_to_3 += [
        {"k": 30, "packets": [emitted(retry)]},
        {"k": 31, "packets": [["delivered", retry, "lockstep:serial0.1"]]},
        {"k": 33, "packets": [emitted(up2), ["copy", copy3, up2], ["routed", up2, 1]]},
        {"k": 34, "packets": [["delivered", copy3, "sitl3.serial1"]]},
        {"k": 36, "packets": [emitted(down3)]},
        {"k": 37, "packets": [["delivered", down3, [3, 1]]]},
    ]
    traces = trace_navlink(lost_to_3, FLEET)
    assert len(traces) == 4 and all(t.ambiguous for t in traces)
    assert not any(t.ambiguous for t in trace_navlink(records(), FLEET))
