"""Did each scheduled fault bite, and was the mission back by its deadline? (step 7)

Each test builds a run that passes (``fault_journal_evidence``) and breaks one thing;
the gate must then say exactly that.
"""

from __future__ import annotations

import pytest

from navpy.core.swarm.task_peer_roster import PEER_ADMITTED_LOG, PEER_HEARD_AGAIN_LOG, PEER_SILENT_LOG
from navpy.sim.lockstep.coordinator_types import serial0_device
from navpy.sim.lockstep.fault_schedule import (
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PEER_RECOVERY_S,
    PRESENCE_MARKER,
    RADIO_PARTITION,
    RESTART_RECOVERY_S,
    FaultPlan,
)
from navpy.sim.lockstep.part_wire import FREEZE
from scripts.eval_gcs_demo_fleet_faults import IN_SERVICE_MARKER, fault_recovery_errors
from tests.scripts.fault_journal_evidence import (
    BACKEND,
    COMPANION,
    FENCE_K,
    FLEET,
    GCS,
    LAUNCH_K,
    PRESENCE_K,
    S,
    SWARM_HEARTBEAT,
    FaultJournal,
    window,
    write_links,
)

START = 10
HEAL = 20
DEADLINE_K = HEAL + round(PEER_RECOVERY_S)
#: The backend's link to each vehicle, always up.
ALWAYS_UP = {i: [(1, "connected")] for i in FLEET}
#: ... and its link to system 1 lost in the gap, back after the heal.
ONE_LOST = {**ALWAYS_UP, 1: [(1, "connected"), (16, "lost"), (HEAL + 1, "connected")]}


def _errors(tmp_path, journal, links=ALWAYS_UP, companions=None):
    records = journal.build()
    write_links(tmp_path, links, journal.companions() if companions is None else companions)
    return fault_recovery_errors(tmp_path, records, journal.lines)


def _peer_lines(journal, pairs, *, silent=HEAL - 3, heard=HEAL + 1):
    for receiver, sender in pairs:
        journal.lines[receiver] += [(silent, PEER_SILENT_LOG.format(sender)), (heard, PEER_HEARD_AGAIN_LOG.format(sender))]


def _both_ways(i):
    others = [j for j in FLEET if j != i]
    return [*((j, i) for j in others), *((i, j) for j in others)]


def _scheduled(plan, start=START, heal=HEAL, deadline=None, forced=False):
    journal = FaultJournal((plan,))
    journal.fault(start, "start", plan.id)
    event = ["heal", plan.id, deadline if deadline is not None else heal * S + round(plan.recovery_s * S)]
    journal.fault(heal, *event, *(["forced"] if forced else []))
    return journal


def _link_drop(target="sitl1", start=START, heal=HEAL, lines=True):
    plan = FaultPlan(LINK_DROP, target, start - LAUNCH_K, heal - start)
    journal = _scheduled(plan, start, heal)
    ticks = window(start, heal)
    journal.cut_sources[target] = (ticks, plan.id)
    endpoints = [*(f"sitl{i}" for i in FLEET), "backend"]
    for other in endpoints:
        if other != target:
            journal.cut_paths[(other, target)] = (ticks, plan.id)
    if lines and target != "backend":
        _peer_lines(journal, _both_ways(int(target[4:])), silent=heal - 3, heard=heal + 1)
    return journal


def _partition(uav=3, start=START, heal=HEAL, lines=True):
    plan = FaultPlan(RADIO_PARTITION, f"uav{uav}", start - PRESENCE_K, heal - start)
    journal = _scheduled(plan, start, heal)
    ticks = window(start, heal)
    for other in FLEET:
        if other != uav:
            journal.cut_paths[(f"sitl{uav}", f"sitl{other}")] = (ticks, plan.id)
            journal.cut_paths[(f"sitl{other}", f"sitl{uav}")] = (ticks, plan.id)
    if lines:
        _peer_lines(journal, _both_ways(uav), silent=heal - 3, heard=heal + 1)
    return journal


def _late_heartbeat(uav=2, start=START, heal=HEAL, duration=None, forced=False, lines=True):
    plan = FaultPlan(LATE_HEARTBEAT, f"uav{uav}", start - LAUNCH_K, (heal - start) if duration is None else duration)
    journal = _scheduled(plan, start, heal, forced=forced)
    journal.silent_navpy[uav] = window(start, heal)
    journal.lifecycle(start, FREEZE, uav, 1, {"frozen": True, "fault": plan.id})
    journal.lifecycle(heal, FREEZE, uav, 1, {"frozen": False, "fault": plan.id})
    if lines:
        _peer_lines(journal, [(other, uav) for other in FLEET if other != uav], silent=heal - 3, heard=heal + 1)
    return journal


READY_K = START + 5


def _restart(uav=2, start=START, heal=START + 2, ready=READY_K):
    plan = FaultPlan(NAVPY_RESTART, f"uav{uav}", start - LAUNCH_K, 0.0)
    journal = _scheduled(plan, start, heal)
    payload = {"fault": plan.id}
    journal.lifecycle(start, "stop", uav, 1, payload)
    journal.lifecycle(heal, "exited", uav, 1, {"failed": False})
    journal.lifecycle(heal, "spawn", uav, 2, payload, "ok")
    if ready is not None:
        journal.lifecycle(ready, "ready", uav, 2, {"generation": 2})
    journal.silent_navpy[uav] = range(start + 1, ready or heal + 1)
    journal.respawn[uav] = heal + 1
    journal.lines[uav].append((READY_K + 1, PRESENCE_MARKER))  # it had checked in, so it does again
    journal.lines[3] = [(start + 1, PRESENCE_MARKER)]  # a restart starts only before the last check-in
    return journal


def test_a_run_with_no_plan_has_nothing_to_check(tmp_path):
    journal = FaultJournal(())
    assert _errors(tmp_path, journal) == []


def test_a_run_with_no_plan_and_a_fault_event_fails(tmp_path):
    journal = FaultJournal(())
    journal.fault(START, "start", "link_drop:sitl1")
    assert _errors(tmp_path, journal) == ["fault schedule: events for link_drop:sitl1, which the grid does not plan"]


@pytest.mark.parametrize("build,links", [
    (_link_drop, ONE_LOST),
    (lambda: _link_drop("backend"), {i: [(1, "connected"), (16, "lost"), (HEAL + 1, "connected")] for i in FLEET}),
    (_partition, ALWAYS_UP),
    (_late_heartbeat, ALWAYS_UP),
    (_restart, ALWAYS_UP),
])
def test_a_fault_that_bit_and_recovered_passes(tmp_path, build, links):
    assert _errors(tmp_path, build(), links) == []


def test_the_schedule_is_part_of_the_gate(tmp_path):
    plan = FaultPlan(LINK_DROP, "sitl1", START - LAUNCH_K, HEAL - START)
    journal = _link_drop()
    journal.records[HEAL]["faults"] = [["heal", plan.id, (HEAL + 1) * S]]
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert any(e.startswith(f"fault schedule: {plan.id}'s deadline") for e in errors)


# -- Link drop ---------------------------------------------------------------------------------------------------------

DROP = "link_drop:sitl1"


def test_a_link_drop_that_cut_nothing_fails(tmp_path):
    journal = _link_drop()
    journal.cut_sources.clear()
    journal.cut_paths.clear()
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert f"fault {DROP}: cut nothing in its window {START + 1}..{HEAL}" in errors
    assert any(e.startswith(f"fault {DROP}: packet ") and "crossed the cut" in e for e in errors)
    assert any(e.startswith(f"fault {DROP}: copy ") and "crossed the cut" in e for e in errors)


def test_a_cut_that_ended_a_packet_outside_its_window_fails(tmp_path):
    journal = _link_drop()
    journal.cut_sources["sitl1"] = (range(START + 1, HEAL + 2), DROP)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert any(e.endswith(f"ended radio_fault at tick {HEAL + 1}, outside the window {START + 1}..{HEAL}")
               for e in errors)


def test_a_source_the_cut_let_through_for_part_of_its_window_fails(tmp_path):
    journal = _link_drop()
    journal.cut_sources["sitl1"] = (range(START + 1, HEAL), DROP)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert any(e.startswith(f"fault {DROP}: packet ") and e.endswith(f"crossed the cut at tick {HEAL}") for e in errors)


def test_a_path_the_cut_let_through_for_part_of_its_window_fails(tmp_path):
    journal = _link_drop()
    journal.cut_paths[("sitl2", "sitl1")] = (range(START + 1, HEAL), DROP)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert any(e.startswith(f"fault {DROP}: copy ") and e.endswith(f"crossed the cut at tick {HEAL}") for e in errors)


def test_a_sitl_silent_past_the_deadline_is_not_back(tmp_path):
    journal = _link_drop(lines=False)
    journal.silent_sources["sitl1"] = range(HEAL + 1, DEADLINE_K + 1)
    journal.silent_navpy[1] = range(HEAL + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal, ONE_LOST)
    message = f"fault {DROP}: nothing reached the backend from {{}}, sent after the heal, by tick {DEADLINE_K}"
    assert message.format("1's autopilot") in errors
    assert message.format("1's NavPy") in errors


def test_a_navpys_heartbeat_does_not_stand_in_for_the_autopilots(tmp_path):
    journal = _link_drop()
    journal.silent_sources["sitl1"] = range(HEAL + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert errors == [f"fault {DROP}: nothing reached the backend from 1's autopilot, sent after the heal, "
                      f"by tick {DEADLINE_K}"]


def test_a_navpy_whose_heartbeat_stopped_with_the_link_is_not_back(tmp_path):
    journal = _link_drop(lines=False)
    journal.silent_navpy[1] = range(START + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert f"fault {DROP}: nothing reached the backend from 1's NavPy, sent after the heal, by tick {DEADLINE_K}" in errors


def test_a_navpy_that_had_not_reached_the_backend_before_the_drop_owes_it_nothing(tmp_path):
    journal = _link_drop(lines=False, heal=START + 4)
    journal.lines[1] = [row for row in journal.lines[1] if LAUNCH_MARKER in row[1]]  # no check-in: no peers owed
    journal.silent_navpy[1] = range(1, DEADLINE_K + 1)
    assert _errors(tmp_path, journal) == []


def test_a_gcs_heartbeat_queued_before_the_cut_is_not_the_link_back(tmp_path):
    journal = _link_drop()
    journal.silent_sources["backend"] = range(HEAL + 1, DEADLINE_K + 1)
    records = journal.build()
    journal.forward_stale(HEAL + 2, BACKEND, START + 1, GCS, serial0_device(1), [1, 1])
    write_links(tmp_path, ONE_LOST, journal.companions())
    errors = fault_recovery_errors(tmp_path, records, journal.lines)
    assert f"fault {DROP}: nothing reached NavPy 1 from the GCS, sent after the heal, by tick {DEADLINE_K}" in errors


def test_a_gcs_heartbeat_that_never_came_back_fails_both_hops(tmp_path):
    journal = _link_drop("backend", lines=False)
    journal.silent_sources["backend"] = range(HEAL + 1, DEADLINE_K + 1)
    lost = {i: [(1, "connected"), (16, "lost"), (HEAL + 1, "connected")] for i in FLEET}
    errors = _errors(tmp_path, journal, lost)
    message = f"fault link_drop:backend: nothing reached {{}}, sent after the heal, by tick {DEADLINE_K}"
    assert message.format("SITL 2 from the GCS") in errors
    assert message.format("NavPy 2 from the GCS") in errors


def test_the_backends_link_still_lost_at_the_deadline_fails(tmp_path):
    links = {**ALWAYS_UP, 1: [(1, "connected"), (16, "lost"), (DEADLINE_K + 1, "connected")]}
    errors = _errors(tmp_path, _link_drop(), links)
    assert errors == [f"fault {DROP}: the backend's link to 1 is ['lost'] at the deadline"]


def test_a_companion_status_not_back_at_the_deadline_fails(tmp_path):
    journal = _link_drop()
    late = {1: [(1, "ok"), (16, "checking"), (DEADLINE_K + 1, "ok")]}
    assert _errors(tmp_path, journal, ONE_LOST, late) == [
        f"fault {DROP}: the backend's companion status of 1 is 'checking' at the deadline"]


def test_a_companion_status_that_was_not_ok_at_the_start_owes_nothing_at_the_deadline(tmp_path):
    journal = _link_drop()
    late = {1: [(1, "down")]}
    assert _errors(tmp_path, journal, ONE_LOST, late) == []


def test_a_long_gap_the_backend_never_noticed_fails(tmp_path):
    errors = _errors(tmp_path, _link_drop(), ALWAYS_UP)
    assert errors == [f"fault {DROP}: the backend never lost 1 in its 11.0 s gap (ticks {START}..{HEAL + 1})"]


@pytest.mark.parametrize("lost_k", [START - 5, DEADLINE_K + 1], ids=["before", "after"])
def test_a_loss_outside_the_gap_is_not_the_gaps(tmp_path, lost_k):
    links = {**ALWAYS_UP, 1: [(1, "connected"), (lost_k, "lost"), (lost_k + 1, "connected")]}
    assert _errors(tmp_path, _link_drop(), links) == [
        f"fault {DROP}: the backend never lost 1 in its 11.0 s gap (ticks {START}..{HEAL + 1})"]


def test_a_dropped_sitls_uav_must_hear_its_peers_again(tmp_path):
    journal = _link_drop()
    journal.silent_navpy[2] = range(HEAL + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal, ONE_LOST)
    assert f"fault {DROP}: 1 did not hear peer 2 by tick {DEADLINE_K}" in errors


def test_a_short_gap_owes_no_lost_link_and_no_silence_lines(tmp_path):
    assert _errors(tmp_path, _link_drop(heal=START + 4, lines=False), ALWAYS_UP) == []


@pytest.mark.parametrize(("kind", "target", "anchor_k"), [(LINK_DROP, "sitl1", LAUNCH_K),
                                                          (RADIO_PARTITION, "uav3", PRESENCE_K)])
def test_a_window_the_fence_cut_short_may_have_carried_nothing(tmp_path, kind, target, anchor_k):
    start = FENCE_K - 1
    plan = FaultPlan(kind, target, start - anchor_k, 9.0)
    journal = _scheduled(plan, start, FENCE_K, forced=True)
    ticks = window(start, FENCE_K)
    journal.silent_navpy = dict.fromkeys(FLEET, ticks)
    journal.silent_sources = dict.fromkeys(("sitl1", "sitl2", "sitl3", "backend"), ticks)
    assert _errors(tmp_path, journal) == []


def _long_forced_drop():
    start = FENCE_K - 10
    plan = FaultPlan(LINK_DROP, "sitl1", start - LAUNCH_K, 99.0)
    journal = _scheduled(plan, start, FENCE_K, forced=True)
    journal.cut_sources["sitl1"] = (window(start, FENCE_K), plan.id)
    for other in ("sitl2", "sitl3", "backend"):
        journal.cut_paths[(other, "sitl1")] = (window(start, FENCE_K), plan.id)
    _peer_lines(journal, _both_ways(1), silent=FENCE_K - 3, heard=FENCE_K + 1)
    return journal


def test_a_long_gap_the_fence_ended_still_owes_the_lost_link(tmp_path):
    lost = {**ALWAYS_UP, 1: [(1, "connected"), (FENCE_K - 5, "lost"), (FENCE_K + 1, "connected")]}
    assert _errors(tmp_path, _long_forced_drop(), lost) == []
    assert _errors(tmp_path, _long_forced_drop(), ALWAYS_UP) == [
        f"fault {DROP}: the backend never lost 1 in its 11.0 s gap (ticks {FENCE_K - 10}..{FENCE_K + 1})"]


# -- Peers -------------------------------------------------------------------------------------------------------------

PARTITION = "radio_partition:uav3"


def test_a_peer_not_heard_again_by_the_deadline_fails(tmp_path):
    journal = _partition()
    journal.cut_paths[("sitl3", "sitl1")] = (range(START + 1, HEAL + 1), PARTITION)
    journal.silent_navpy[3] = range(HEAL + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal)
    assert f"fault {PARTITION}: 1 did not hear peer 3 by tick {DEADLINE_K}" in errors
    assert f"fault {PARTITION}: 2 did not hear peer 3 by tick {DEADLINE_K}" in errors


def test_a_peer_heard_only_from_bytes_queued_before_the_heal_is_not_back(tmp_path):
    journal = _partition()
    journal.silent_navpy[3] = range(HEAL + 1, DEADLINE_K + 1)
    records = journal.build()
    # SITL 1 re-emits, after the heal, a UID its sender first emitted before the cut.
    queued = ["sitl1", 1, "lockstep:serial0.1", 999_999]
    uid = [3, journal.boot(3, START), START]
    records[HEAL + 2]["packets"] += [
        ["emitted", queued, [0, 1], "s" * 8, SWARM_HEARTBEAT, [3, COMPANION], [0, 0], {"uid": uid}],
        ["delivered", queued, [1, 1]],
    ]
    write_links(tmp_path, ALWAYS_UP, journal.companions())
    errors = fault_recovery_errors(tmp_path, records, journal.lines)
    assert f"fault {PARTITION}: 1 did not hear peer 3 by tick {DEADLINE_K}" in errors


def test_a_long_gap_with_no_silence_line_fails(tmp_path):
    journal = _partition()
    journal.lines[1] = [row for row in journal.lines[1] if row[1] != PEER_SILENT_LOG.format(3)]
    assert _errors(tmp_path, journal) == [
        f"fault {PARTITION}: 1 logged no silence of peer 3 in its 11.0 s gap (ticks {START}..{HEAL + 1})"]


def test_a_long_gap_with_no_heard_again_line_by_the_deadline_fails(tmp_path):
    journal = _partition()
    journal.lines[1] = [row for row in journal.lines[1] if row[1] != PEER_HEARD_AGAIN_LOG.format(3)]
    journal.lines[1].append((DEADLINE_K + 1, PEER_HEARD_AGAIN_LOG.format(3)))
    assert _errors(tmp_path, journal) == [
        f"fault {PARTITION}: 1 did not log peer 3 heard again by tick {DEADLINE_K}"]


def test_a_silence_from_before_the_pair_last_met_owes_nothing(tmp_path):
    journal = _partition(heal=START + 4, lines=False)
    journal.lines[1].append((START - 2, PEER_SILENT_LOG.format(3)))  # an earlier generation's, say: it never logged its end
    assert _errors(tmp_path, journal) == []


def test_a_silence_logged_before_the_gap_does_not_answer_for_it(tmp_path):
    journal = _partition()
    silent = PEER_SILENT_LOG.format(3)
    journal.lines[1] = [(START - 1 if line == silent else k, line) for k, line in journal.lines[1]]
    assert _errors(tmp_path, journal) == [
        f"fault {PARTITION}: 1 logged no silence of peer 3 in its 11.0 s gap (ticks {START}..{HEAL + 1})"]


def test_a_silence_logged_in_a_short_gap_is_over_by_the_deadline(tmp_path):
    heal = START + 4
    by = heal + round(PEER_RECOVERY_S)
    journal = _partition(heal=heal, lines=False)  # a 5 s gap: its silence is not owed, but this one was logged
    journal.lines[1].append((heal - 1, PEER_SILENT_LOG.format(3)))
    assert _errors(tmp_path, journal) == [f"fault {PARTITION}: 1 did not log peer 3 heard again by tick {by}"]
    journal = _partition(heal=heal, lines=False)
    journal.lines[1] += [(heal - 1, PEER_SILENT_LOG.format(3)), (by + 1, PEER_HEARD_AGAIN_LOG.format(3))]
    assert _errors(tmp_path, journal) == [f"fault {PARTITION}: 1 did not log peer 3 heard again by tick {by}"]
    journal = _partition(heal=heal, lines=False)
    journal.lines[1] += [(heal - 1, PEER_SILENT_LOG.format(3)), (by, PEER_HEARD_AGAIN_LOG.format(3))]
    assert _errors(tmp_path, journal) == []


def test_a_heard_again_line_in_the_silences_own_tick_ends_it(tmp_path):
    journal = _partition(heal=START + 4, lines=False)
    journal.lines[1] += [(START + 6, PEER_SILENT_LOG.format(3)), (START + 6, PEER_HEARD_AGAIN_LOG.format(3))]
    assert _errors(tmp_path, journal) == []


def test_a_heard_again_line_before_a_new_silence_does_not_end_it(tmp_path):
    heal = START + 4
    journal = _partition(heal=heal, lines=False)
    journal.lines[1] += [(heal - 2, PEER_SILENT_LOG.format(3)), (heal - 1, PEER_HEARD_AGAIN_LOG.format(3)),
                         (heal, PEER_SILENT_LOG.format(3))]
    assert _errors(tmp_path, journal) == [
        f"fault {PARTITION}: 1 did not log peer 3 heard again by tick {heal + round(PEER_RECOVERY_S)}"]


def test_a_pair_that_first_met_across_the_partition_must_still_meet(tmp_path):
    journal = _partition(lines=False)
    # 3 checked in during the partition, spoke only then, and is silent after the heal.
    journal.lines[3] = [(START + 2, PRESENCE_MARKER)]
    journal.silent_navpy[3] = [*range(1, START + 1), *range(HEAL + 1, DEADLINE_K + 1)]
    errors = _errors(tmp_path, journal)
    assert f"fault {PARTITION}: 1 did not hear peer 3 by tick {DEADLINE_K}" in errors
    assert f"fault {PARTITION}: 2 did not hear peer 3 by tick {DEADLINE_K}" in errors


def _first_met_after_the_partition():
    """3 checked in during the partition, so neither side had taken the other before the heal."""
    journal = _partition(lines=False)
    journal.lines[3] = [(START + 2, PRESENCE_MARKER)]
    journal.silent_navpy[3] = range(1, START + 1)
    return journal


def test_a_first_meeting_is_recovered_by_the_receivers_admission(tmp_path):
    assert _errors(tmp_path, _first_met_after_the_partition()) == []


def test_a_first_meeting_delivered_and_never_taken_by_the_roster_fails(tmp_path):
    """A delivery is the participant's input: a stalled reader leaves no silence to end, as no roster has the sender."""
    journal = _first_met_after_the_partition()
    journal.admitted.add((1, 1, 3))
    assert _errors(tmp_path, journal) == [f"fault {PARTITION}: 1 did not admit peer 3 by tick {DEADLINE_K}"]


def test_a_first_meeting_taken_by_the_roster_after_the_deadline_fails(tmp_path):
    journal = _first_met_after_the_partition()
    journal.admitted.add((1, 1, 3))
    journal.lines[1].append((DEADLINE_K + 1, PEER_ADMITTED_LOG.format(3)))
    assert _errors(tmp_path, journal) == [f"fault {PARTITION}: 1 did not admit peer 3 by tick {DEADLINE_K}"]


def test_a_first_meeting_taken_by_the_roster_at_the_deadline_passes(tmp_path):
    journal = _first_met_after_the_partition()
    journal.admitted.add((1, 1, 3))
    journal.lines[1].append((DEADLINE_K, PEER_ADMITTED_LOG.format(3)))
    assert _errors(tmp_path, journal) == []


def test_a_receiver_whose_presence_had_not_started_owes_nothing(tmp_path):
    journal = _partition()
    journal.lines[1] = [row for row in journal.lines[1] if LAUNCH_MARKER in row[1]]  # no check-in, no peer lines
    assert _errors(tmp_path, journal) == []


def test_a_receiver_not_yet_present_at_the_heal_need_not_have_heard_anyone(tmp_path):
    journal = _partition()
    journal.lines[1] = [row for row in journal.lines[1] if LAUNCH_MARKER in row[1]]
    records = journal.build()
    for record in records[HEAL + 1:]:
        record["packets"] = [event for event in record["packets"] if event[:1] != ["delivered"] or event[2] != [1, 1]]
    write_links(tmp_path, ALWAYS_UP, journal.companions())
    assert fault_recovery_errors(tmp_path, records, journal.lines) == []


def test_a_sender_the_receiver_had_not_heard_before_the_fault_owes_no_silence_line(tmp_path):
    journal = _partition()
    journal.silent_navpy[3] = range(1, START + 1)  # 3 first spoke after the partition began: nothing was working
    journal.lines[1] = [row for row in journal.lines[1] if "Peer" not in row[1]]
    journal.lines[2] = [row for row in journal.lines[2] if "Peer" not in row[1]]
    journal.lines[3] = [row for row in journal.lines[3] if "Peer" not in row[1]]
    errors = _errors(tmp_path, journal)
    assert not any(e.startswith((f"fault {PARTITION}: 1 ", f"fault {PARTITION}: 2 ")) for e in errors)
    assert f"fault {PARTITION}: 3 logged no silence of peer 1 in its 11.0 s gap (ticks {START}..{HEAL + 1})" in errors


def test_a_partition_that_also_cut_the_backend_link_fails(tmp_path):
    journal = _partition()
    journal.cut_paths[("sitl3", "backend")] = (window(START, HEAL), PARTITION)
    assert _errors(tmp_path, journal) == [f"fault {PARTITION}: its link to the backend carried nothing in the window"]


def test_a_partition_after_a_restart_owes_no_silence_of_the_new_generation(tmp_path):
    journal = _partition(uav=3, lines=False)
    journal.lifecycle(START - 2, "spawn", 1, 2, {"fault": "navpy_restart:uav1"}, "ok")
    journal.respawn[1] = START - 1
    journal.lines[1].append((START + 2, PRESENCE_MARKER))  # it checked in inside the partition
    _peer_lines(journal, [(2, 3), (3, 2)])
    assert _errors(tmp_path, journal) == []


def test_a_sender_not_yet_present_at_the_heal_owes_nothing(tmp_path):
    journal = _partition(lines=False)
    journal.lines[3] = [(HEAL + 1, PRESENCE_MARKER)]
    journal.silent_navpy[3] = range(HEAL + 1, DEADLINE_K + 1)
    assert not any("hear peer" in e for e in _errors(tmp_path, journal))


# -- Late heartbeat ----------------------------------------------------------------------------------------------------

FREEZE_ID = "late_heartbeat:uav2"


def test_a_freeze_with_no_records_fails(tmp_path):
    journal = _late_heartbeat()
    for k in (START, HEAL):
        del journal.records[k]["lifecycle"]
    assert _errors(tmp_path, journal) == [
        f"fault {FREEZE_ID}: freeze records [], not one at its start and one at its heal"]


def test_a_navpy_that_had_not_spoken_before_its_freeze_fails(tmp_path):
    journal = _late_heartbeat()
    journal.silent_navpy[2] = range(1, HEAL + 1)
    errors = _errors(tmp_path, journal)
    assert f"fault {FREEZE_ID}: NavPy 2 had said nothing before its freeze" in errors


def test_a_frozen_navpy_that_spoke_fails(tmp_path):
    journal = _late_heartbeat()
    journal.silent_navpy[2] = range(START + 2, HEAL + 1)
    errors = _errors(tmp_path, journal)
    assert any(e.startswith(f"fault {FREEZE_ID}: NavPy 2 frozen {START + 1}..{HEAL} spoke at [{START + 1}")
               and e.endswith("and logged at []") for e in errors)


def test_a_frozen_navpy_that_logged_fails(tmp_path):
    journal = _late_heartbeat()
    journal.lines[2].append((START + 3, "a line"))
    assert _errors(tmp_path, journal) == [
        f"fault {FREEZE_ID}: NavPy 2 frozen {START + 1}..{HEAL} spoke at [] and logged at [{START + 3}]"]


def test_the_autopilots_heartbeat_does_not_stand_in_for_the_companions(tmp_path):
    journal = _late_heartbeat()
    journal.silent_navpy[2] = range(START + 1, DEADLINE_K + 1)  # the autopilot's heartbeat keeps coming
    errors = _errors(tmp_path, journal)
    assert (f"fault {FREEZE_ID}: no companion heartbeat of 2 at the backend, sent after the thaw, "
            f"by tick {DEADLINE_K}") in errors
    assert f"fault {FREEZE_ID}: 1 did not hear peer 2 by tick {DEADLINE_K}" in errors


def test_a_frozen_navpy_must_hear_its_peers_after_the_thaw(tmp_path):
    journal = _late_heartbeat(uav=3)
    journal.silent_navpy[1] = range(HEAL + 1, DEADLINE_K + 1)
    errors = _errors(tmp_path, journal)
    assert f"fault late_heartbeat:uav3: 3 did not hear peer 1 by tick {DEADLINE_K}" in errors


def test_a_heartbeat_queued_before_the_freeze_is_not_the_companion_back(tmp_path):
    journal = _late_heartbeat()
    journal.silent_navpy[2] = range(START + 1, DEADLINE_K + 1)
    records = journal.build()
    journal.forward_stale(HEAL + 2, 2, START + 1, (2, COMPANION), "lockstep:router", [BACKEND, 1])
    write_links(tmp_path, ALWAYS_UP, journal.companions())
    errors = fault_recovery_errors(tmp_path, records, journal.lines)
    assert (f"fault {FREEZE_ID}: no companion heartbeat of 2 at the backend, sent after the thaw, "
            f"by tick {DEADLINE_K}") in errors


def test_a_heartbeat_a_queue_delayed_into_the_freeze_does_not_reject_it(tmp_path):
    """The status left ok and came back; only the gap in deliveries looks short."""
    journal = _late_heartbeat()
    records = journal.build()
    journal.forward_stale(HEAL - 1, 2, START + 1, (2, COMPANION), "lockstep:router", [BACKEND, 1])
    write_links(tmp_path, ALWAYS_UP, journal.companions())
    assert fault_recovery_errors(tmp_path, records, journal.lines) == []


def test_a_freeze_too_short_to_leave_the_companion_status_ok_fails(tmp_path):
    journal = _late_heartbeat(heal=START + 3, lines=False)
    assert _errors(tmp_path, journal) == [
        f"fault {FREEZE_ID}: the backend's companion status of 2 stayed ok through the freeze"]


@pytest.mark.parametrize("rows", [[(1, "checking"), (START - 2, "ok")],  # it left ok before the freeze
                                  [(1, "ok"), (HEAL + 1, "checking"), (HEAL + 2, "ok")]])  # and after the thaw
def test_a_status_that_left_ok_outside_the_freeze_shows_nothing_of_it(tmp_path, rows):
    assert _errors(tmp_path, _late_heartbeat(), companions={2: rows}) == [
        f"fault {FREEZE_ID}: the backend's companion status of 2 stayed ok through the freeze"]


def test_a_freeze_of_a_navpy_the_backend_never_had_as_ok_fails(tmp_path):
    journal = _late_heartbeat()
    never = {2: [(1, "down"), (HEAL + 1, "ok")]}
    assert _errors(tmp_path, journal, companions=never) == [
        f"fault {FREEZE_ID}: the backend's companion status of 2 was 'down' at its start"]


def test_a_thawed_navpy_whose_status_is_not_ok_at_the_deadline_fails(tmp_path):
    journal = _late_heartbeat()
    stuck = {2: [(1, "ok"), (16, "checking")]}
    assert _errors(tmp_path, journal, companions=stuck) == [
        f"fault {FREEZE_ID}: the backend's companion status of 2 is 'checking' at the deadline"]


def test_a_freeze_the_fence_cut_short_owes_no_window(tmp_path):
    start = FENCE_K - 3
    journal = _late_heartbeat(uav=3, start=start, heal=FENCE_K, duration=8, forced=True, lines=False)
    assert _errors(tmp_path, journal) == []


# -- NavPy restart -----------------------------------------------------------------------------------------------------

RESTART = "navpy_restart:uav2"
RESTART_HEAL = START + 2
RESTART_BY = RESTART_HEAL + round(RESTART_RECOVERY_S)


def _without_lifecycle(journal, kind, generation):
    for record in journal.records:
        if "lifecycle" in record:
            record["lifecycle"] = [e for e in record["lifecycle"]
                                   if not (e["kind"] == kind and e["generation"] == generation)]


def test_a_restart_with_no_respawn_in_its_heal_tick_fails(tmp_path):
    journal = _restart()
    _without_lifecycle(journal, "spawn", 2)
    errors = _errors(tmp_path, journal)
    assert f"fault {RESTART}: no respawn in the heal's tick {RESTART_HEAL}" in errors


def test_a_restart_with_no_stop_naming_it_fails(tmp_path):
    journal = _restart()
    _without_lifecycle(journal, "stop", 1)
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: no stop of generation 1 naming it at its start {START}"]


def test_a_restart_whose_old_generation_did_not_flush_fails(tmp_path):
    journal = _restart()
    next(e for r in journal.records for e in r.get("lifecycle", ()) if e["kind"] == "exited")["payload"] = {"failed": True}
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: generation 1 did not end flushed"]


def test_a_restart_that_kept_its_boot_id_fails(tmp_path):
    journal = _restart()
    journal.same_boot = True
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: generation 2 kept its boot id"]


def test_a_new_generation_not_ready_by_the_deadline_fails(tmp_path):
    journal = _restart()
    _without_lifecycle(journal, "ready", 2)
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: generation 2 was not ready by tick {RESTART_BY}"]


NO_COMPANION = f"fault {RESTART}: no companion heartbeat of 2, sent after the heal, at the backend by tick {RESTART_BY}"


def test_a_new_generation_with_no_companion_heartbeat_by_the_deadline_fails(tmp_path):
    journal = _restart()
    journal.silent_navpy[2] = range(START + 1, RESTART_BY + 1)
    assert NO_COMPANION in _errors(tmp_path, journal)


def test_the_old_generations_heartbeat_forwarded_after_the_respawn_is_not_the_new_ones(tmp_path):
    journal = _restart()
    journal.silent_navpy[2] = range(START + 1, RESTART_BY + 1)
    records = journal.build()
    journal.forward_stale(RESTART_HEAL + 2, 2, START + 1, (2, COMPANION), "lockstep:router", [BACKEND, 1])
    write_links(tmp_path, ALWAYS_UP, journal.companions())
    assert NO_COMPANION in fault_recovery_errors(tmp_path, records, journal.lines)


def test_a_new_generation_whose_status_is_not_ok_at_the_deadline_fails(tmp_path):
    journal = _restart()
    stuck = {2: [(1, "ok"), (START + 3, "checking")]}
    assert _errors(tmp_path, journal, companions=stuck) == [
        f"fault {RESTART}: the backend's companion status of 2 is 'checking' at the deadline"]


def test_a_silence_the_last_generation_logged_owes_the_new_one_nothing(tmp_path):
    journal = _restart()
    journal.silent_navpy[3] = range(START - 5, RESTART_HEAL + 2)
    journal.lines[2].append((START - 1, PEER_SILENT_LOG.format(3)))
    assert _errors(tmp_path, journal) == []


def _said_before(journal, *rows):
    """What the last generation of UAV 2 had logged, and the new one nothing yet."""
    journal.lines[2] = [*rows, *((k, line) for k, line in journal.lines[2]
                                 if k > RESTART_HEAL and PRESENCE_MARKER not in line)]


NO_LAUNCH_YET = (1, f"x {IN_SERVICE_MARKER}: DISARMED")
LAUNCHED = (2, f"x {LAUNCH_MARKER} 1")
CHECKED_IN = (PRESENCE_K, PRESENCE_MARKER)


@pytest.mark.parametrize(("before", "line", "ok"), [
    # The phase the last generation had reached, or a later one.
    (NO_LAUNCH_YET, f"x {IN_SERVICE_MARKER}: DISARMED", True),
    (NO_LAUNCH_YET, f"x {IN_SERVICE_MARKER}: MODE MANUAL", True),
    (NO_LAUNCH_YET, f"x {LAUNCH_MARKER} 1", True),
    (NO_LAUNCH_YET, PRESENCE_MARKER, True),
    (NO_LAUNCH_YET, "a line of no phase", False),
    (LAUNCHED, f"x {LAUNCH_MARKER} 1", True),
    (LAUNCHED, PRESENCE_MARKER, True),
    (LAUNCHED, f"x {IN_SERVICE_MARKER}: DISARMED", False),
    (CHECKED_IN, PRESENCE_MARKER, True),
    (CHECKED_IN, f"x {LAUNCH_MARKER} 1", False),
])
def test_a_new_generation_is_back_at_the_phase_its_last_one_had_reached(tmp_path, before, line, ok):
    journal = _restart()
    _said_before(journal, before)
    journal.lines[2].append((READY_K + 1, line))
    errors = _errors(tmp_path, journal)
    assert ok == (not any("logged none of" in e for e in errors))


def test_a_new_generation_with_no_flow_line_by_the_deadline_fails(tmp_path):
    journal = _restart()
    _said_before(journal, CHECKED_IN)
    journal.lines[2].append((RESTART_BY + 1, PRESENCE_MARKER))
    assert _errors(tmp_path, journal) == [
        f"fault {RESTART}: generation 2 logged none of ('{PRESENCE_MARKER}',) by tick {RESTART_BY}"]


AFTER_THE_HEAL = range(RESTART_HEAL + 1, RESTART_BY + 1)


def test_a_new_generation_and_its_present_peers_must_hear_each_other(tmp_path):
    journal = _restart()
    journal.cut_paths[("sitl2", "sitl1")] = (AFTER_THE_HEAL, "elsewhere")
    journal.cut_paths[("sitl3", "sitl2")] = (AFTER_THE_HEAL, "elsewhere")
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: 1 did not hear peer 2 by tick {RESTART_BY}",
                                          f"fault {RESTART}: 2 did not hear peer 3 by tick {RESTART_BY}"]


def test_a_new_generation_owes_its_own_admission_of_each_peer(tmp_path):
    """The last generation's line was for a roster that went with it."""
    journal = _restart()
    journal.admitted.add((2, 2, 1))
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: 2 did not admit peer 1 by tick {RESTART_BY}"]


def test_a_peer_not_present_at_the_new_generations_check_in_owes_nothing(tmp_path):
    journal = _restart()
    journal.lines[3] = [(RESTART_BY, PRESENCE_MARKER)]
    journal.cut_paths[("sitl2", "sitl3")] = (AFTER_THE_HEAL, "elsewhere")
    journal.cut_paths[("sitl3", "sitl2")] = (AFTER_THE_HEAL, "elsewhere")
    assert _errors(tmp_path, journal) == []


@pytest.mark.parametrize(("check_in", "owed"), [(RESTART_BY - round(PEER_RECOVERY_S), True),
                                                (RESTART_BY - round(PEER_RECOVERY_S) + 1, False)])
def test_a_first_check_in_inside_the_last_heartbeat_ttl_owes_no_meeting_yet(tmp_path, check_in, owed):
    journal = _restart()
    journal.lines[2] = [row for row in journal.lines[2] if PRESENCE_MARKER not in row[1]]  # the last one never did
    journal.lines[2].append((check_in, PRESENCE_MARKER))
    journal.cut_paths[("sitl2", "sitl1")] = (AFTER_THE_HEAL, "elsewhere")
    assert _errors(tmp_path, journal) == ([f"fault {RESTART}: 1 did not hear peer 2 by tick {RESTART_BY}"] if owed else [])


def test_a_late_check_in_of_a_generation_that_had_checked_in_still_owes_the_meeting(tmp_path):
    journal = _restart()
    journal.lines[2] = [row for row in journal.lines[2] if row[0] != READY_K + 1]
    journal.lines[2].append((RESTART_BY - round(PEER_RECOVERY_S) + 1, PRESENCE_MARKER))
    journal.cut_paths[("sitl2", "sitl1")] = (AFTER_THE_HEAL, "elsewhere")
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: 1 did not hear peer 2 by tick {RESTART_BY}"]


def test_a_silence_of_the_restarted_peer_is_over_by_the_deadline(tmp_path):
    journal = _restart()
    journal.lines[1].append((READY_K - 1, PEER_SILENT_LOG.format(2)))
    assert _errors(tmp_path, journal) == [f"fault {RESTART}: 1 did not log peer 2 heard again by tick {RESTART_BY}"]
    journal = _restart()
    journal.lines[1] += [(READY_K - 1, PEER_SILENT_LOG.format(2)), (READY_K + 2, PEER_HEARD_AGAIN_LOG.format(2))]
    assert _errors(tmp_path, journal) == []
