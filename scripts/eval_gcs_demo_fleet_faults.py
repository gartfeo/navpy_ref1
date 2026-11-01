"""Did each scheduled fault bite, and was the mission back by its deadline? (step 7)

``deterministic_dev_sim.md``, section 4: a scheduled fault is expected, and the
run fails only if the mission does not recover by the fault's journaled deadline.
A run with no plan has nothing to check. These gates add to the run's others, which
still pass in full (only what a cut packet can excuse is excused, in
``eval_gcs_demo_fleet_packets``): that is the mission finishing.

Window: the ticks after the start up to the heal, ``s+1..h``, as a change at the end
of tick k takes effect from k+1. "By the deadline": at a tick no later than the heal's
time plus the plan's ``recovery_s``. "Sent after the heal": a packet its first sender
emitted after tick h, followed across hops by its NAVLINK UID or else its bytes, so
bytes queued before a cut and an old generation's are not recovery.

| Fault | It bit | Back by the deadline |
|---|---|---|
| Link drop (a SITL's or the backend's router link) | packets ended ``radio_fault`` in the window and only there (a window the fence cut short may have ended none); nothing crossed the cut | sent after the heal: the autopilot's heartbeat at the backend, its NavPy's too if that was reaching the backend before, and the GCS heartbeat at the SITL and the NavPy; the backend's link state is connected, and was lost if the heartbeats stopped long enough; its companion status is ok if it was at the start; peers |
| Partition (a UAV and its peers) | as above, on the peer paths; its link to the backend still carried | peers |
| Late heartbeat (a NavPy frozen) | a NavPy that had spoken says nothing and logs nothing in the window; the backend's companion status of it was ok at the start, and left ok in the window unless the fence cut it short | that status is ok again, and a companion heartbeat at the backend was sent after the thaw (a stale one can set the status); peers |
| NavPy restart | its generation ended flushed on a stop naming the fault, and the next one, with a new boot id if both spoke NAVLINK, was spawned in the heal's tick | the new generation ready; its companion heartbeat at the backend, sent after the heal; its companion status is ok if it was at the start; the flow line of the phase the last generation had reached, or a later one; and if it checked in by the deadline, it and each peer present then hear each other, unless the last generation had never checked in and this one did so inside a heartbeat TTL of the deadline |

Peers: ``eval_gcs_demo_fault_peers``. A link drop leaves a NavPy's own serial link
alone, and a reader stalled behind it would show in no line of its: that is left to
the later mission gates, not this deadline.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.swarm.swarm_heartbeat_runtime import HEARTBEAT_INTERVAL_S
from navpy.sim.lockstep.coordinator_mavlink import NAVLINK_IDS, RADIO_FAULT
from navpy.sim.lockstep.coordinator_router import BACKEND_ENDPOINT, router_endpoints
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE, serial1_device
from navpy.sim.lockstep.fault_schedule import (
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PEER_RECOVERY_S,
    PRESENCE_MARKER,
    RADIO_PARTITION,
)
from navpy.sim.lockstep.part_wire import FREEZE
from scripts.eval_gcs_demo_fault_evidence import FaultRun, timeline_events
from scripts.eval_gcs_demo_fault_peers import both_ways, heard_errors, peer_errors, silence_over_errors
from scripts.eval_gcs_demo_fault_schedule import FaultSpan, Lines, fault_schedule_errors, fault_spans, planned_faults
from scripts.eval_gcs_demo_fleet_packets import packet_events
from scripts.eval_gcs_demo_slice_gates import GCS_SYSTEM

HEARTBEAT = mavlink.MAVLINK_MSG_ID_HEARTBEAT
AUTOPILOT = mavlink.MAV_COMP_ID_AUTOPILOT1
COMPANION = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
#: The backend's link timeout, which ``VehicleManager`` passes to each ``VehicleMav``.
BACKEND_LINK_TIMEOUT_S = 5.0
#: A gap this long passes the backend's link timeout for certain: the timeout, and twice a heartbeat's period.
LINK_LOST_PROOF_S = BACKEND_LINK_TIMEOUT_S + 2 * HEARTBEAT_INTERVAL_S
#: The first line a NavPy's flow logs, once it is in service, before its launch.
IN_SERVICE_MARKER = "ONHOLD"
#: The backend's companion status when the newest heartbeat is fresh (``VehicleManager.companion_status``).
COMPANION_OK = "ok"


def _cut_errors(run: FaultRun, span: FaultSpan, sources: set[str], paths: set[tuple[str, str]]) -> list[str]:
    """Packets ended ``radio_fault`` in the window and only there, and nothing crossed the cut."""
    start, heal = span.start_k, span.heal_k
    ended: set[tuple] = set()
    errors = []
    events = list(packet_events(run.records))
    for k, event in events:
        if event[0] == RADIO_FAULT and event[2] == span.id:
            ended.add(tuple(event[1]))
            if not start < k <= heal:
                errors.append(f"fault {span.id}: packet {event[1]} ended {RADIO_FAULT} at tick {k}, "
                              f"outside the window {start + 1}..{heal}")
    if not ended and not span.forced:
        errors.append(f"fault {span.id}: cut nothing in its window {start + 1}..{heal}")
    for k, event in events:
        if not start < k <= heal:
            continue
        if event[0] == "routed" and event[1][2] in sources:
            errors.append(f"fault {span.id}: packet {event[1]} crossed the cut at tick {k}")
        elif event[0] == "copy" and (event[2][2], event[1][2]) in paths and tuple(event[1]) not in ended:
            errors.append(f"fault {span.id}: copy {event[1]} crossed the cut at tick {k}")
    return errors


def _link_state_errors(run: FaultRun, span: FaultSpan, sys_id: int) -> list[str]:
    """The backend's link to ``sys_id`` is up at the deadline, and was lost if its heartbeats stopped long enough."""
    events = run.links.get(sys_id, [])
    at_deadline = [outcome for t_us, outcome in events if t_us <= span.deadline_us]
    errors = []
    if not at_deadline or at_deadline[-1] != "connected":
        errors.append(f"fault {span.id}: the backend's link to {sys_id} is {at_deadline[-1:]} at the deadline")
    ticks = [d.k for d in run.received(run.backend, sys_id, (HEARTBEAT,), AUTOPILOT)]
    before, after, gap = run.gap_s(ticks, span.heal_k)
    if gap > LINK_LOST_PROOF_S and before is not None and after is not None:
        lost = [t_us for t_us, outcome in events if outcome == "lost" and run.times[before] < t_us]
        if not any(t_us <= run.times[after] for t_us in lost):
            errors.append(f"fault {span.id}: the backend never lost {sys_id} in its {gap:.1f} s gap "
                          f"(ticks {before}..{after})")
    return errors


def _companion_back_errors(run: FaultRun, span: FaultSpan, sys_id: int) -> list[str]:
    """The backend's companion status of ``sys_id``, ok when the fault began, is ok at the deadline."""
    status = run.companion_at(sys_id, span.deadline_us)
    if run.companion_at(sys_id, run.times[span.start_k]) != COMPANION_OK or status == COMPANION_OK:
        return []
    return [f"fault {span.id}: the backend's companion status of {sys_id} is {status!r} at the deadline"]


def _endpoint_errors(run: FaultRun, span: FaultSpan, sys_id: int) -> list[str]:
    """One UAV's heartbeats with the backend both ways, sent after the heal."""
    by = run.by_tick(span)
    heal = span.heal_k
    expected = [
        (run.heard(run.backend, sys_id, (HEARTBEAT,), heal, by, AUTOPILOT), f"the backend from {sys_id}'s autopilot"),
        (run.heard(serial1_device(sys_id), GCS_SYSTEM, (HEARTBEAT,), heal, by), f"SITL {sys_id} from the GCS"),
        (run.heard(sys_id, GCS_SYSTEM, (HEARTBEAT,), heal, by), f"NavPy {sys_id} from the GCS"),
    ]
    if any(d.k <= span.start_k for d in run.received(run.backend, sys_id, (HEARTBEAT,), COMPANION)):
        expected.append((run.heard(run.backend, sys_id, (HEARTBEAT,), heal, by, COMPANION),
                         f"the backend from {sys_id}'s NavPy"))
    errors = [f"fault {span.id}: nothing reached {what}, sent after the heal, by tick {by}"
              for tick, what in expected if tick is None]
    return errors + _link_state_errors(run, span, sys_id) + _companion_back_errors(run, span, sys_id)


def _link_drop_errors(run: FaultRun, span: FaultSpan) -> list[str]:
    target = span.plan["target"]
    endpoints = router_endpoints(len(run.sys_ids))
    device = {BACKEND_ENDPOINT: ROUTER_DEVICE, **{f"sitl{i}": serial1_device(i) for i in run.sys_ids}}
    paths = {(device[other], device[target]) for other in endpoints if other != target}
    errors = _cut_errors(run, span, {device[target]}, paths)
    if target == BACKEND_ENDPOINT:
        return errors + [e for sys_id in run.sys_ids for e in _endpoint_errors(run, span, sys_id)]
    sys_id = int(target.removeprefix("sitl"))
    pairs = both_ways(sys_id, (other for other in run.sys_ids if other != sys_id))
    return errors + _endpoint_errors(run, span, sys_id) + peer_errors(run, span, pairs)


def _partition_errors(run: FaultRun, span: FaultSpan) -> list[str]:
    uav = int(span.plan["target"].removeprefix("uav"))
    others = [other for other in run.sys_ids if other != uav]
    paths = {pair for other in others for pair in ((serial1_device(uav), serial1_device(other)),
                                                   (serial1_device(other), serial1_device(uav)))}
    errors = _cut_errors(run, span, set(), paths)
    if run.heard(run.backend, uav, None, span.start_k, span.heal_k) is None and not span.forced:
        errors.append(f"fault {span.id}: its link to the backend carried nothing in the window")
    return errors + peer_errors(run, span, both_ways(uav, others))


def _late_heartbeat_errors(run: FaultRun, span: FaultSpan) -> list[str]:
    uav = int(span.plan["target"].removeprefix("uav"))
    start, heal = span.start_k, span.heal_k
    payloads = [(k, entry["payload"]) for k, entry in run.lifecycle
                if entry["kind"] == FREEZE and entry["ordinal"] == uav and entry["payload"]["fault"] == span.id]
    errors = []
    if payloads != [(start, {"frozen": True, "fault": span.id}), (heal, {"frozen": False, "fault": span.id})]:
        errors.append(f"fault {span.id}: freeze records {payloads}, not one at its start and one at its heal")
    spoke = [k for k, event in packet_events(run.records) if event[0] == "emitted" and event[1][0] == uav]
    if not any(k <= start for k in spoke):
        errors.append(f"fault {span.id}: NavPy {uav} had said nothing before its freeze")
    spoke_frozen = [k for k in spoke if start < k <= heal]
    logged_frozen = [k for k, _line in run.lines[uav] if start < k <= heal]
    if spoke_frozen or logged_frozen:
        errors.append(f"fault {span.id}: NavPy {uav} frozen {start + 1}..{heal} spoke at {spoke_frozen[:3]} "
                      f"and logged at {logged_frozen[:3]}")
    by = run.by_tick(span)
    if run.heard(run.backend, uav, (HEARTBEAT,), heal, by, COMPANION) is None:
        errors.append(f"fault {span.id}: no companion heartbeat of {uav} at the backend, sent after the thaw, by "
                      f"tick {by}")
    start_us, heal_us = run.times[start], run.times[heal]
    status = run.companion_at(uav, start_us)
    if status != COMPANION_OK:
        errors.append(f"fault {span.id}: the backend's companion status of {uav} was {status!r} at its start")
    elif not span.forced and all(state == COMPANION_OK for at, state in run.companions.get(uav, ())
                                 if start_us < at <= heal_us):
        errors.append(f"fault {span.id}: the backend's companion status of {uav} stayed ok through the freeze")
    errors += _companion_back_errors(run, span, uav)
    return errors + peer_errors(run, span, both_ways(uav, (other for other in run.sys_ids if other != uav)))


def _boots(run: FaultRun, uav: int, generation: int) -> set[int]:
    return {event[7]["uid"][1] for _k, event in packet_events(run.records)
            if event[0] == "emitted" and event[1][0] == uav and event[1][1] == generation
            and event[4] in NAVLINK_IDS and len(event) > 7 and "uid" in event[7]}


def _service_markers(run: FaultRun, uav: int, heal_k: int) -> tuple[str, ...]:
    """A flow line of the phase the last generation had reached, or of a later one (``navigation_decision``)."""
    said = [line for k, line in run.lines[uav] if k <= heal_k]
    if any(PRESENCE_MARKER in line for line in said):
        return (PRESENCE_MARKER,)
    if any(LAUNCH_MARKER in line for line in said):
        return (LAUNCH_MARKER, PRESENCE_MARKER)
    return (IN_SERVICE_MARKER, PRESENCE_MARKER)


def _reintegration_errors(run: FaultRun, span: FaultSpan, uav: int) -> list[str]:
    """A new generation that checked in and its present peers hear each other; a first check-in inside
    a heartbeat TTL of the deadline owes no meeting yet, but a generation that had checked in owes it."""
    checked_in = run.line_at(uav, PRESENCE_MARKER, span.heal_k, run.by_tick(span))
    if checked_in is None:
        return []
    if not run.present(uav, span.start_k) and run.times[checked_in] + round(PEER_RECOVERY_S * 1e6) > span.deadline_us:
        return []
    pairs = both_ways(uav, (other for other in run.sys_ids if other != uav and run.present(other, checked_in)))
    errors = heard_errors(run, span, pairs)
    for receiver, sender in pairs:
        errors += silence_over_errors(run, span, receiver, sender)
    return errors


def _restart_errors(run: FaultRun, span: FaultSpan) -> list[str]:
    uav = int(span.plan["target"].removeprefix("uav"))
    start, heal = span.start_k, span.heal_k
    fault_payload = {"fault": span.id}
    respawn = next((entry for k, entry in run.lifecycle if k == heal and entry["kind"] == "spawn"
                    and entry["payload"] == fault_payload and entry["result"] == "ok"), None)
    if respawn is None:
        return [f"fault {span.id}: no respawn in the heal's tick {heal}"]
    generation = respawn["generation"]
    errors = []
    if not any(k == start and entry["kind"] == "stop" and entry["payload"] == fault_payload
               and (entry["ordinal"], entry["generation"]) == (uav, generation - 1) for k, entry in run.lifecycle):
        errors.append(f"fault {span.id}: no stop of generation {generation - 1} naming it at its start {start}")
    if not any(entry["kind"] == "exited" and (entry["ordinal"], entry["generation"]) == (uav, generation - 1)
               and entry["payload"] == {"failed": False} for _k, entry in run.lifecycle):
        errors.append(f"fault {span.id}: generation {generation - 1} did not end flushed")
    if _boots(run, uav, generation) & _boots(run, uav, generation - 1):
        errors.append(f"fault {span.id}: generation {generation} kept its boot id")
    by = run.by_tick(span)
    if not any(entry["kind"] == "ready" and (entry["ordinal"], entry["generation"]) == (uav, generation) and k <= by
               for k, entry in run.lifecycle):
        errors.append(f"fault {span.id}: generation {generation} was not ready by tick {by}")
    if run.heard(run.backend, uav, (HEARTBEAT,), heal, by, COMPANION) is None:
        errors.append(f"fault {span.id}: no companion heartbeat of {uav}, sent after the heal, at the backend by "
                      f"tick {by}")
    errors += _companion_back_errors(run, span, uav)
    markers = _service_markers(run, uav, heal)
    if all(run.line_at(uav, marker, heal, by) is None for marker in markers):
        errors.append(f"fault {span.id}: generation {generation} logged none of {markers} by tick {by}")
    return errors + _reintegration_errors(run, span, uav)


_GATES: dict[str, Callable[[FaultRun, FaultSpan], list[str]]] = {
    LINK_DROP: _link_drop_errors,
    RADIO_PARTITION: _partition_errors,
    LATE_HEARTBEAT: _late_heartbeat_errors,
    NAVPY_RESTART: _restart_errors,
}


def fault_recovery_errors(log_dir: Path, records: Sequence[Mapping[str, Any]], lines: Lines) -> list[str]:
    """The schedule, then each fault that ran; a run with no plan owes no recovery, and no fault event."""
    errors = fault_schedule_errors(records, lines)
    if not planned_faults(records):
        return errors
    run = FaultRun(records, lines, timeline_events(log_dir, "vehicle_link_state"),
                   timeline_events(log_dir, "vehicle_companion_state"))
    for span in fault_spans(records):
        errors += _GATES[span.plan["kind"]](run, span)
    return errors


__all__ = ["BACKEND_LINK_TIMEOUT_S", "IN_SERVICE_MARKER", "LINK_LOST_PROOF_S", "fault_recovery_errors"]
