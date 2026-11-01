"""The scheduled faults' journal against the grid, tick times and lines (step 7)."""

from __future__ import annotations

import dataclasses

import pytest

from navpy.sim.lockstep.fault_schedule import (
    FENCE_MARKER,
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PEER_RECOVERY_S,
    PRESENCE_MARKER,
    RESTART_RECOVERY_S,
    FaultPlan,
)
from scripts.eval_gcs_demo_fault_schedule import NAV_MARKER, fault_schedule_errors, fault_spans

S = 1_000_000
ANCHOR_K = 10
FENCE_K = 40
#: Due at tick 12, healed at tick 15 (3 s), deadline 5 s later.
DROP = FaultPlan(LINK_DROP, "sitl1", 2.0, 3.0)
DROP_ID = DROP.id
DROP_DEADLINE = 15 * S + round(PEER_RECOVERY_S * S)


def _records(plans: tuple[FaultPlan, ...], faults: dict[int, list], lifecycle: dict[int, list] | None = None) -> list:
    records: list[dict] = [{"grid": {"faults": [plan.to_json() for plan in plans]}}]
    for k in range(1, 61):
        record: dict = {"k": k, "t_us": k * S}
        if k in faults:
            record["faults"] = faults[k]
        if lifecycle and k in lifecycle:
            record["lifecycle"] = lifecycle[k]
        records.append(record)
    return records


def _lines(fence: int | None = FENCE_K, nav: int | None = 55) -> dict:
    rows = [(ANCHOR_K, f"x {LAUNCH_MARKER} 1")]
    if fence is not None:
        rows.append((fence, f"x {FENCE_MARKER}1"))
    if nav is not None:
        rows.append((nav, NAV_MARKER))
    return {1: [], 2: rows}


GOOD = {12: [["start", DROP_ID]], 15: [["heal", DROP_ID, DROP_DEADLINE]]}


def test_a_fault_free_journal_has_nothing_to_check():
    assert fault_schedule_errors(_records((), {}), _lines()) == []
    assert fault_schedule_errors([{"grid": {}}, {"k": 1, "t_us": S}], {1: []}) == []


def test_a_fault_on_time_passes_and_gives_its_span():
    records = _records((DROP,), GOOD)
    assert fault_schedule_errors(records, _lines()) == []
    (span,) = fault_spans(records)
    assert (span.id, span.start_k, span.heal_k, span.deadline_us, span.forced) == (DROP_ID, 12, 15, DROP_DEADLINE, False)


@pytest.mark.parametrize("start", [11, 13])
def test_a_start_off_its_due_tick_fails(start):
    faults = {start: [["start", DROP_ID]], start + 3: [["heal", DROP_ID, (start + 3) * S + 5 * S]]}
    assert fault_schedule_errors(_records((DROP,), faults), _lines()) == [
        f"fault schedule: {DROP_ID} started at tick {start}; it was due and clear at tick 12"]


def test_a_start_with_no_anchor_line_fails():
    lines = {1: [(FENCE_K, FENCE_MARKER)], 2: []}
    assert fault_schedule_errors(_records((DROP,), GOOD), lines) == [
        f"fault schedule: {DROP_ID} started at tick 12 with no {LAUNCH_MARKER!r} line"]


@pytest.mark.parametrize("heal", [14, 16])
def test_a_heal_off_its_tick_fails(heal):
    faults = {12: [["start", DROP_ID]], heal: [["heal", DROP_ID, heal * S + 5 * S]]}
    assert fault_schedule_errors(_records((DROP,), faults), _lines()) == [
        f"fault schedule: {DROP_ID} healed at tick {heal} (forced False); expected tick 15 (forced False)"]


def test_a_wrong_deadline_fails():
    faults = {12: [["start", DROP_ID]], 15: [["heal", DROP_ID, DROP_DEADLINE + 1]]}
    assert fault_schedule_errors(_records((DROP,), faults), _lines()) == [
        f"fault schedule: {DROP_ID}'s deadline is {DROP_DEADLINE + 1}, not its heal's time plus its "
        f"recovery {DROP_DEADLINE}"]


def test_the_fence_forces_a_heal_at_its_tick():
    long = dataclasses.replace(DROP, duration_s=30.0)
    deadline = 20 * S + 5 * S
    faults = {12: [["start", long.id]], 20: [["heal", long.id, deadline, "forced"]]}
    assert fault_schedule_errors(_records((long,), faults), _lines(fence=20)) == []
    # The fence came and nothing ended the cut.
    faults = {12: [["start", long.id]], 42: [["heal", long.id, 47 * S]]}
    assert fault_schedule_errors(_records((long,), faults), _lines(fence=20)) == [
        f"fault schedule: {long.id} healed at tick 42 (forced False); expected tick 20 (forced True)"]
    # It claims the fence with no fence.
    faults = {12: [["start", long.id]], 20: [["heal", long.id, deadline, "forced"]]}
    assert fault_schedule_errors(_records((long,), faults), _lines(fence=None)) == [
        f"fault schedule: {long.id} healed at tick 20 (forced True); expected tick 42 (forced False)"]


def test_a_second_fault_waits_for_the_first_deadline():
    second = FaultPlan(LATE_HEARTBEAT, "uav2", 3.0, 2.0)  # due at tick 13, the first's deadline is at tick 20
    faults = {**GOOD, 20: [["start", second.id]], 22: [["heal", second.id, 22 * S + round(PEER_RECOVERY_S * S)]]}
    assert fault_schedule_errors(_records((DROP, second), faults), _lines()) == []
    faults = {**GOOD, 19: [["start", second.id]], 21: [["heal", second.id, 21 * S + round(PEER_RECOVERY_S * S)]]}
    assert fault_schedule_errors(_records((DROP, second), faults), _lines()) == [
        f"fault schedule: {second.id} started at tick 19; it was due and clear at tick 20"]


RESTART = FaultPlan(NAVPY_RESTART, "uav2", 2.0, 0.0)
EXITED = {"kind": "exited", "ordinal": 2, "generation": 1, "payload": {"failed": False}}
SPAWNED = {"kind": "spawn", "ordinal": 2, "generation": 2, "payload": {"fault": RESTART.id}}


def test_a_restart_heals_in_the_tick_its_generation_ended():
    deadline = 14 * S + round(RESTART_RECOVERY_S * S)
    faults = {12: [["start", RESTART.id]], 14: [["heal", RESTART.id, deadline]]}
    lifecycle = {14: [EXITED, SPAWNED]}
    assert fault_schedule_errors(_records((RESTART,), faults, lifecycle), _lines(nav=None)) == []
    assert fault_schedule_errors(_records((RESTART,), faults, {15: [EXITED], 14: [SPAWNED]}), _lines(nav=None)) == [
        f"fault schedule: {RESTART.id} healed at tick 14 (forced False); expected tick 15 (forced False)"]


def test_a_fault_due_and_clear_before_the_fence_may_not_be_skipped_there():
    early = dataclasses.replace(DROP, start_s=5.0)  # due at tick 15
    assert fault_schedule_errors(_records((early,), {40: [["skip", early.id, "fence"]]}), _lines()) == [
        f"fault schedule: {early.id} was due and clear at tick 15, yet skipped at the fence 40"]


def test_a_fault_due_at_the_fence_itself_is_skipped_there():
    due = dataclasses.replace(DROP, start_s=float(FENCE_K - ANCHOR_K))
    assert fault_schedule_errors(_records((due,), {FENCE_K: [["skip", due.id, "fence"]]}), _lines()) == []


def test_a_fault_with_no_anchor_line_may_be_skipped_at_the_fence():
    lines = {1: [(FENCE_K, FENCE_MARKER)], 2: []}
    assert fault_schedule_errors(_records((DROP,), {FENCE_K: [["skip", DROP_ID, "fence"]]}), lines) == []


def test_a_fault_another_held_back_past_the_fence_may_be_skipped_there():
    second = FaultPlan(LATE_HEARTBEAT, "uav2", 3.0, 2.0)  # due at 13, while the first's deadline is at 20
    faults = {**GOOD, 18: [["skip", second.id, "fence"]]}
    assert fault_schedule_errors(_records((DROP, second), faults), _lines(fence=18)) == []
    # With the fence after that deadline it should have run.
    faults = {**GOOD, 25: [["skip", second.id, "fence"]]}
    assert fault_schedule_errors(_records((DROP, second), faults), _lines(fence=25)) == [
        f"fault schedule: {second.id} was due and clear at tick 20, yet skipped at the fence 25"]


def test_a_fault_that_started_after_another_was_due_did_not_hold_it_back():
    later = dataclasses.replace(DROP, start_s=6.0)  # due at 16, healed at 19, deadline at 24
    first = FaultPlan(LATE_HEARTBEAT, "uav2", 3.0, 2.0)  # due at 13, with nothing running
    faults = {16: [["start", later.id]], 19: [["heal", later.id, 24 * S]], 20: [["skip", first.id, "fence"]]}
    assert fault_schedule_errors(_records((first, later), faults), _lines(fence=20)) == [
        f"fault schedule: {first.id} was due and clear at tick 13, yet skipped at the fence 20"]


def test_a_skip_is_only_at_the_fence():
    late = dataclasses.replace(DROP, start_s=45.0)
    assert fault_schedule_errors(_records((late,), {40: [["skip", late.id, "fence"]]}), _lines()) == []
    assert fault_spans(_records((late,), {40: [["skip", late.id, "fence"]]})) == []
    assert fault_schedule_errors(_records((late,), {41: [["skip", late.id, "fence"]]}), _lines()) == [
        f"fault schedule: {late.id} skipped at tick 41, the fence came at 40"]


PRESENCE_K = 30
#: Heals in tick 14, its generation ended.
RESTART_FAULTS = {12: [["start", RESTART.id]], 14: [["heal", RESTART.id, 14 * S + round(RESTART_RECOVERY_S * S)]]}
RESTART_LIFECYCLE = {14: [EXITED, SPAWNED]}
#: Due in the tick the last UAV first checks in.
DUE_AT_PRESENCE = dataclasses.replace(RESTART, start_s=float(PRESENCE_K - ANCHOR_K))


def _present(fence: int | None = FENCE_K) -> dict:
    """Every UAV has checked in by ``PRESENCE_K``, the last one then."""
    lines = _lines(fence=fence, nav=None)
    lines[1] = [(20, PRESENCE_MARKER), (29, PRESENCE_MARKER)]
    lines[2] = [*lines[2][:1], (PRESENCE_K, PRESENCE_MARKER), *lines[2][1:]]
    return lines


def test_a_restart_may_start_only_until_every_uav_has_checked_in():
    assert fault_schedule_errors(_records((RESTART,), RESTART_FAULTS, RESTART_LIFECYCLE), _present()) == []
    k, heal = PRESENCE_K, PRESENCE_K + 2
    late = DUE_AT_PRESENCE
    faults = {k: [["start", late.id]], heal: [["heal", late.id, heal * S + round(RESTART_RECOVERY_S * S)]]}
    assert fault_schedule_errors(_records((late,), faults, {heal: [EXITED, SPAWNED]}), _present(fence=None)) == [
        f"fault schedule: {late.id} started at tick {k}, once every UAV had checked in at {k}"]


def test_a_later_check_in_does_not_move_the_presence():
    lines = _present(fence=None)
    lines[2].append((35, PRESENCE_MARKER))
    k, heal = PRESENCE_K, PRESENCE_K + 2
    late = DUE_AT_PRESENCE
    faults = {k: [["start", late.id]], heal: [["heal", late.id, heal * S + round(RESTART_RECOVERY_S * S)]]}
    assert fault_schedule_errors(_records((late,), faults, {heal: [EXITED, SPAWNED]}), lines) == [
        f"fault schedule: {late.id} started at tick {k}, once every UAV had checked in at {k}"]


def test_a_restart_due_once_every_uav_has_checked_in_is_skipped_then():
    late = DUE_AT_PRESENCE
    skip = {PRESENCE_K: [["skip", late.id, "presence"]]}
    assert fault_schedule_errors(_records((late,), skip), _present()) == []
    assert fault_spans(_records((late,), skip)) == []
    assert fault_schedule_errors(_records((late,), {31: [["skip", late.id, "presence"]]}), _present()) == [
        f"fault schedule: {late.id} skipped at tick 31, the presence came at {PRESENCE_K}"]


def test_a_restart_due_and_clear_before_the_last_check_in_may_not_be_skipped_then():
    skip = {PRESENCE_K: [["skip", RESTART.id, "presence"]]}
    assert fault_schedule_errors(_records((RESTART,), skip), _present()) == [
        f"fault schedule: {RESTART.id} was due and clear at tick 12, yet skipped at the presence {PRESENCE_K}"]


def test_a_presence_skip_needs_every_uav_checked_in():
    lines = _present()
    lines[1] = []
    late = DUE_AT_PRESENCE
    assert fault_schedule_errors(_records((late,), {PRESENCE_K: [["skip", late.id, "presence"]]}), lines) == [
        f"fault schedule: {late.id} skipped at tick {PRESENCE_K}, the presence came at None"]


def test_only_a_restart_is_skipped_at_the_presence():
    late = dataclasses.replace(DROP, start_s=float(PRESENCE_K - ANCHOR_K))
    assert fault_schedule_errors(_records((late,), {PRESENCE_K: [["skip", late.id, "presence"]]}), _present()) == [
        f"fault schedule: {late.id} skipped at the presence, which only closes a restart"]


def test_a_skip_for_no_known_reason_fails():
    assert fault_schedule_errors(_records((DROP,), {40: [["skip", DROP_ID, "whim"]]}), _lines()) == [
        f"fault schedule: {DROP_ID} skipped at tick 40, the whim came at None"]


def test_a_restarts_deadline_falls_at_the_fence_at_the_latest():
    records = _records((RESTART,), RESTART_FAULTS, RESTART_LIFECYCLE)
    deadline_k = 14 + round(RESTART_RECOVERY_S)
    assert fault_schedule_errors(records, _lines(fence=deadline_k, nav=None)) == []
    assert fault_schedule_errors(records, _lines(fence=deadline_k - 1, nav=None)) == [
        f"fault schedule: {RESTART.id}'s deadline {deadline_k * S} is after the fence at tick {deadline_k - 1}"]


@pytest.mark.parametrize("faults", [{}, {12: [["start", DROP_ID]]}, {15: [["heal", DROP_ID, DROP_DEADLINE]]}])
def test_a_planned_fault_that_never_ran_its_course_fails(faults):
    errors = fault_schedule_errors(_records((DROP,), faults), _lines())
    assert errors == [f"fault schedule: {DROP_ID} has events {[e[0] for es in faults.values() for e in es]}, "
                      f"not a start and a heal or a skip"]


def test_events_for_an_unplanned_fault_fail():
    assert fault_schedule_errors(_records((), {3: [["start", "ghost"]]}), _lines()) == [
        "fault schedule: events for ghost, which the grid does not plan"]


def test_a_deadline_after_the_first_approach_fails():
    assert fault_schedule_errors(_records((DROP,), GOOD), _lines(nav=19)) == [
        f"fault schedule: {DROP_ID}'s deadline {DROP_DEADLINE} is after the first approach at tick 19"]
    assert fault_schedule_errors(_records((DROP,), GOOD), _lines(nav=20)) == []
