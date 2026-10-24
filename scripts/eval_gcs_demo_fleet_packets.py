"""A certified lockstep run's packet gates (``lockstep_fleet.md``, Journal gates).

They read the journal's packet records (``coordinator_packets``) and the
NAVLINK traces built from them (``navpy.sim.lockstep.navlink_trace``):

| Gate | Pass |
|---|---|
| Run (packets) | every packet id is defined once and has exactly one terminal disposition (``PacketJournal``); no NAVLINK packet is ``undelivered`` |
| Assignment and forwarding | every peer holds exactly one task by a complete chain (below); adverts sent before stop-all reach every peer they are for (an advert may stop only at ``radio_broken``: the radio broke its sender SITL's serial1 write, ``navlink_trace``); every packet's source is a fleet sysid or the ground station, except an emission the radio journal records ``false`` (below, counted in ``report.json``); no NAVLINK packet is ``router_dropped`` or ``router_filtered`` |

The chain of a peer task, correlated by UID and ``task_id`` (auction task ids
are the detector's namespace, so the plan's POIs are joined by the backend
log's coordinates, not here): the owner's ``TASK_ASSIGN_REQUEST`` to helper h,
h's RECEIVED ``SWARM_ACK`` of that request and, after the first such ACK (the
owner's fence floor, same boot and a later ``msg_seq``), its
``TASK_ASSIGN_RESPONSE`` for the task to the owner, and the owner's APPLIED
``SWARM_ACK`` of that response to h; an ACK's ref names the acked UID and its
``MsgType`` (``task_ack_routing``). Each is a complete, unambiguous trace: NavPy i's serial0, SITL i's
serial1, the router's copy to SITL j (the bytes kept, so the sha256), SITL
j's serial0, NavPy j. The peers' tasks are distinct, one per peer.

A false emission (:func:`unverified_sources`): with radio byte loss
(``radio_link``) the router's framer can parse a frame out of a broken one
(an embedded or stitched frame, whose CRC it never checks for an id outside
its table). The coordinator does the deletion, so it records every emission
that is no clean packet as ``false``; only those are exempt from the source
check, and every other emission stays checked.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping, Sequence
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.link.swarm_ack_msg import ACK_STATUS_APPLIED, ACK_STATUS_RECEIVED
from navpy.core.link.types import MsgType
from navpy.sim.lockstep.coordinator_mavlink import NAVLINK_IDS
from navpy.sim.lockstep.coordinator_types import serial0_device
from navpy.sim.lockstep.navlink_trace import RADIO_BROKEN, NavlinkTrace, trace_navlink
from navpy.sim.lockstep.packet_journal import PacketJournal
from navpy.sim.lockstep.radio_link import radio_events
from scripts.eval_gcs_demo_slice_gates import GCS_SYSTEM
from scripts.lockstep_navlink_gate import first_stop_tick

REQUEST = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST
RESPONSE = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE
ACK = mavlink.MAVLINK_MSG_ID_SWARM_ACK
#: A free peer's availability advert and the owner's offer.
ADVERT_IDS = (mavlink.MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST, mavlink.MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE)


def packet_events(records: Iterable[Mapping[str, Any]]) -> Iterator[tuple[int, list[Any]]]:
    """Every packet event with its tick, the parked record's last."""
    for record in records:
        if "packets" in record:
            yield from ((record["k"], event) for event in record["packets"])
        elif "parked" in record and "packets" in record["parked"]:
            yield from ((record["parked"]["tick"], event) for event in record["parked"]["packets"])


def _roots(records: Sequence[Mapping[str, Any]]) -> dict[tuple, list[Any]]:
    """Packet id -> its root emission event (a router copy's is its source's)."""
    roots: dict[tuple, list[Any]] = {}
    for _k, event in packet_events(records):
        if event[0] == "emitted":
            roots[tuple(event[1])] = event
        elif event[0] == "copy" and tuple(event[2]) in roots:
            roots[tuple(event[1])] = roots[tuple(event[2])]
    return roots


def packet_run_errors(records: Sequence[Mapping[str, Any]]) -> list[str]:
    errors = [f"packets: {problem}" for problem in PacketJournal(records).problems()]
    roots = _roots(records)
    for k, event in packet_events(records):
        root = roots.get(tuple(event[1]))
        if event[0] == "undelivered" and root is not None and root[4] in NAVLINK_IDS:
            errors.append(f"packets: NAVLINK {event[1]} (msg {root[4]}) undelivered at parking (tick {k})")
    return errors


def _navpy_emissions(records: Sequence[Mapping[str, Any]]) -> Iterator[list[Any]]:
    """Each packet with a UID a NavPy emitted on its own serial0 (not a SITL's forwarding)."""
    for _k, event in packet_events(records):
        if (event[0] == "emitted" and len(event) > 7 and "uid" in event[7]
                and event[1][2] == serial0_device(event[1][0])):
            yield event


def _fields(records: Sequence[Mapping[str, Any]]) -> dict[tuple, dict]:
    """NavPy UID -> its NAVLINK fields (``task_id``, ``ack``), from the NavPys' emissions."""
    return {tuple(event[7]["uid"]): event[7] for event in _navpy_emissions(records)}


class _Chains:
    """The complete, unambiguous traces and their fields, for the chain lookups."""

    def __init__(self, traces: Sequence[NavlinkTrace], fields: Mapping[tuple, dict]) -> None:
        self._traces = [t for t in traces if t.complete and not t.ambiguous]
        self._fields = fields

    def find(self, msgid: int, sender: int, receiver: int, **match: Any) -> list[NavlinkTrace]:
        found = []
        for trace in self._traces:
            if (trace.msgid, trace.sender, trace.receiver) != (msgid, sender, receiver):
                continue
            fields = self._fields.get(trace.uid, {})
            if all(_matches(fields, key, value) for key, value in match.items()):
                found.append(trace)
        return found

    def requests(self, owner: int) -> list[NavlinkTrace]:
        return [t for t in self._traces if t.msgid == REQUEST and t.sender == owner]

    def task(self, trace: NavlinkTrace) -> Any:
        return self._fields.get(trace.uid, {}).get("task_id")


def _matches(fields: Mapping[str, Any], key: str, value: Any) -> bool:
    ack = fields.get("ack", {})
    if key == "acks":  # (UID, MsgType) an ACK refers to: ref [boot_id, msg_seq, type]
        uid, msg_type = value
        return ack.get("ref") == [uid[1], uid[2], msg_type.value]
    if key == "status":
        return ack.get("status") == value
    return fields.get(key) == value


def _applied(chains: _Chains, owner: int, request: NavlinkTrace) -> set[tuple]:
    """The request's chains: its helper acked it and answered for its task after its
    first ACK, and the owner applied the answer (``on_request_ack``, ``answer_response``).
    Each as ``(helper, task, response (boot, seq), APPLIED (boot, seq))``."""
    helper, task = request.receiver, chains.task(request)
    acks = chains.find(ACK, helper, owner, acks=(request.uid, MsgType.TASK_ASSIGN_REQUEST),
                       status=ACK_STATUS_RECEIVED)
    if not acks:
        return set()
    floor = min(acks, key=lambda ack: (ack.hops[-1][1], ack.uid[2])).uid
    return {(helper, task, response.uid[1:], applied.uid[1:])
            for response in chains.find(RESPONSE, helper, owner, task_id=task)
            if response.uid[1] == floor[1] and response.uid[2] > floor[2]
            for applied in chains.find(ACK, owner, helper, status=ACK_STATUS_APPLIED,
                                       acks=(response.uid, MsgType.TASK_ASSIGN_RESPONSE))}


def applied_chains(records: Sequence[Mapping[str, Any]], owner: int,
                   sys_ids: Sequence[int]) -> set[tuple]:
    """Every complete chain, ``(helper, task, response (boot, seq), APPLIED (boot, seq))``."""
    chains = _Chains(trace_navlink(records, sys_ids), _fields(records))
    return {chain for request in chains.requests(owner) for chain in _applied(chains, owner, request)}


def applied_emissions(records: Sequence[Mapping[str, Any]]) -> set[tuple]:
    """Every APPLIED ``SWARM_ACK`` a NavPy emitted on its serial0 of a ``TASK_ASSIGN_RESPONSE``
    its target emitted there, delivered or not,
    ``(owner, helper, task, response (boot, seq), APPLIED (boot, seq))``."""
    emitted = list(_navpy_emissions(records))
    responses = {tuple(e[7]["uid"]): e[7].get("task_id") for e in emitted if e[4] == RESPONSE}
    found = set()
    for event in emitted:
        ack, uid, helper = event[7].get("ack", {}), event[7]["uid"], event[6][0]
        if (event[4] != ACK or ack.get("status") != ACK_STATUS_APPLIED
                or ack["ref"][2] != MsgType.TASK_ASSIGN_RESPONSE.value):
            continue
        task = responses.get((helper, *ack["ref"][:2]))
        if task is not None:
            found.add((uid[0], helper, task, tuple(ack["ref"][:2]), tuple(uid[1:])))
    return found


def held_tasks(records: Sequence[Mapping[str, Any]], owner: int,
               sys_ids: Sequence[int]) -> dict[int, set]:
    """Helper -> the tasks it holds by a complete chain."""
    held: dict[int, set] = {}
    for helper, task, _response, _applied_uid in applied_chains(records, owner, sys_ids):
        held.setdefault(helper, set()).add(task)
    return held


def _assignment_errors(records: Sequence[Mapping[str, Any]], owner: int, peers: Sequence[int],
                       sys_ids: Sequence[int]) -> list[str]:
    held = held_tasks(records, owner, sys_ids)
    errors = [f"forwarding: peer {peer} holds tasks {sorted(held.get(peer, ()))} by a complete "
              f"chain, not one" for peer in peers if len(held.get(peer, ())) != 1]
    errors += [f"forwarding: system {sys_id} holds tasks {sorted(tasks)}, not a peer"
               for sys_id, tasks in sorted(held.items()) if sys_id not in peers]
    tasks = [task for peer in peers for task in held.get(peer, ())]
    if not errors and len(set(tasks)) != len(tasks):
        errors.append(f"forwarding: peers {list(peers)} hold one task {tasks}")
    return errors


def _advert_errors(records: Sequence[Mapping[str, Any]], sys_ids: Sequence[int]) -> list[str]:
    stop = first_stop_tick(list(records))
    errors = []
    for trace in trace_navlink(records, sys_ids):
        if trace.msgid not in ADVERT_IDS or (stop is not None and trace.hops[0][1] >= stop):
            continue
        if not trace.ambiguous and trace.hops[-1][0] == RADIO_BROKEN:
            continue
        if not trace.complete or trace.ambiguous:
            errors.append(f"forwarding: advert {trace.msgid} {list(trace.uid)} from {trace.sender} "
                          f"to {trace.receiver} stopped at {trace.hops[-1][0]}"
                          + (" (ambiguous)" if trace.ambiguous else ""))
    return errors


def unverified_sources(records: Sequence[Mapping[str, Any]], sys_ids: Sequence[int]) -> list[list[Any]]:
    """The emissions the source check exempts: false emissions from outside the fleet (module docstring)."""
    allowed = {*sys_ids, GCS_SYSTEM}
    false = {tuple(event[2]) for event in radio_events(records) if event[0] == "false"}
    return [event for _k, event in packet_events(records)
            if event[0] == "emitted" and event[5][0] not in allowed and tuple(event[1]) in false]


def _source_errors(records: Sequence[Mapping[str, Any]], sys_ids: Sequence[int]) -> list[str]:
    allowed = {*sys_ids, GCS_SYSTEM}
    exempt = {tuple(event[1]) for event in unverified_sources(records, sys_ids)}
    roots = _roots(records)
    errors = []
    for k, event in packet_events(records):
        if event[0] == "emitted" and event[5][0] not in allowed and tuple(event[1]) not in exempt:
            errors.append(f"forwarding: {event[1]} from system {event[5][0]} at tick {k}")
        elif (event[0] in ("router_dropped", "router_filtered")
              and roots.get(tuple(event[1]), [None] * 5)[4] in NAVLINK_IDS):
            errors.append(f"forwarding: NAVLINK {event[1]} {event[0]} at tick {k}")
    return errors


def forwarding_errors(records: Sequence[Mapping[str, Any]], owner: int, peers: Sequence[int],
                      sys_ids: Sequence[int]) -> list[str]:
    """The radio's own journal (``radio_link``) says what its byte loss broke: an advert may
    stop only at ``radio_broken``, and only a ``false`` emission may come from any source."""
    return (_assignment_errors(records, owner, peers, sys_ids) + _advert_errors(records, sys_ids)
            + _source_errors(records, sys_ids))


__all__ = ["ADVERT_IDS", "applied_chains", "applied_emissions", "forwarding_errors", "held_tasks", "packet_events", "packet_run_errors",
           "unverified_sources"]
