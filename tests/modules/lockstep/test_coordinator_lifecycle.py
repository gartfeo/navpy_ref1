"""The lifecycle book in isolation: stops of ended generations and unflushed ends."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Member
from navpy.sim.lockstep.coordinator_spawn import SpawnRecord
from navpy.sim.lockstep.coordinator_types import STOP_BUDGET_US, HostFault, ParticipantFault
from navpy.sim.lockstep.part_wire import StopRequest
from navpy.sim.lockstep.participant_hooks import RequestIds

ROVER, HUB = 1, 2


class _Roster:
    def __init__(self, *members: Member) -> None:
        self.members = list(members)

    def spawn(self, role: str, ordinal: int, spawn_id, requester: int) -> Member:
        new = _member(ordinal, requester)
        new.spawn = SpawnRecord(spawn_id, role, ordinal, self.generation(ordinal) + 1, "cap", requester)
        self.members.append(new)
        return new

    def current(self, ordinal: int):
        found = [m for m in self.members if m.ordinal == ordinal]
        return found[-1] if found else None

    def generation(self, ordinal: int):
        member = self.current(ordinal)
        return None if member is None else member.spawn.generation


def _member(ordinal: int, requester: int = HUB) -> Member:
    spawn = SpawnRecord((requester, ordinal), "rover", ordinal, 1, "cap", requester)
    return Member(spawn, child=None, link=None, devices=(), admitted_at=1, ready=True)


def _book(*members: Member) -> tuple[LifecycleBook, TickRecord]:
    book = LifecycleBook(_Roster(*members), RequestIds(0))
    record = TickRecord(9, "finalization", 180_000)
    book.begin_tick(record)
    return book, record


def _results(record: TickRecord) -> list:
    return [(e["kind"], e["result"], e["payload"]) for e in record.data["lifecycle"]]


def test_a_fresh_stop_of_a_generation_that_ended_on_its_own_is_an_error():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.retire(rover, 8, flushed=True)
    book.stop(StopRequest((HUB, 5), ROVER, 1), HUB, 180_000, draining=False)
    assert _results(record)[-1] == ("stop_result", "error", "not running")
    assert not book.stopped(ROVER, 1)  # never counts as a participant-stop retirement
    assert [f.kind for f in book.faults] == ["unscheduled_exit"]  # nobody asked for that end


def test_a_duplicate_of_a_completed_stop_keeps_its_answer():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.stop(StopRequest((HUB, 5), ROVER, 1), HUB, 180_000, draining=False)
    book.retire(rover, 9, flushed=True)
    book.stop(StopRequest((HUB, 6), ROVER, 1), HUB, 180_000, draining=False)
    assert [r for r in _results(record) if r[0] == "stop_result"] == [
        ("stop_result", "acked", None), ("stop_result", "acked", None)]
    assert book.stopped(ROVER, 1)


@pytest.mark.parametrize("flushed", [True, False])
def test_an_end_nobody_scheduled_is_a_product_failure_flushed_or_not(flushed):
    rover = _member(ROVER)
    book, record = _book(rover)
    book.retire(rover, 9, flushed=flushed)
    assert [(type(f), f.kind, f.tick, f.ordinal) for f in book.faults] == [
        (ParticipantFault, "unscheduled_exit", 9, ROVER)]
    assert _results(record) == [("exited", None, {"failed": not flushed})]


def test_a_flushed_end_the_drain_asked_for_is_no_fault():
    rover = _member(ROVER)
    rover.end_sent = True
    book, _record = _book(rover)
    book.retire(rover, 9, flushed=True)
    assert book.faults == []


def test_an_unflushed_end_the_drain_asked_for_is_a_host_fault():
    rover = _member(ROVER)
    rover.end_sent = True
    book, record = _book(rover)
    book.retire(rover, 9, flushed=False)
    assert [(type(f), f.kind, f.tick, f.ordinal) for f in book.faults] == [(HostFault, "unclean_end", 9, ROVER)]
    assert _results(record) == [("exited", None, {"failed": True})]


def test_an_unflushed_end_under_a_pending_stop_is_missed_and_a_fault():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.stop(StopRequest((HUB, 5), ROVER, 1), HUB, 180_000, draining=False)
    book.retire(rover, 9, flushed=False)
    assert ("stop_result", "missed", None) in _results(record)
    assert [f.kind for f in book.faults] == ["unclean_end"]


def test_a_stop_for_a_generation_replaced_by_a_respawn_is_stale_not_its_old_ack():
    first = _member(ROVER)
    book, record = _book(first)
    book.stop(StopRequest((HUB, 5), ROVER, 1), HUB, 180_000, draining=False)
    book.retire(first, 9, flushed=True)
    assert book.stopped(ROVER, 1)
    second = _member(ROVER)
    second.spawn = SpawnRecord((HUB, 7), "rover", ROVER, 2, "cap", HUB)
    book._roster.members.append(second)
    book.stop(StopRequest((HUB, 8), ROVER, 1), HUB, 200_000, draining=False)
    assert _results(record)[-1] == ("stop_result", "error", "stale generation")
    book.stop(StopRequest((HUB, 9), ROVER, 2), HUB, 200_000, draining=False)
    assert _results(record)[-1] == ("stop", None, None)  # the current generation's stop goes out


# --- a scheduled fault's stop and respawn ---


def test_a_scheduled_stop_goes_to_its_target_naming_the_fault_and_nobody_gets_an_answer():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.stop_scheduled(rover, 180_000, "navpy_restart:uav1")
    assert _results(record) == [("stop", None, {"fault": "navpy_restart:uav1"})]
    assert record.data["lifecycle"][0]["to"] == ROVER
    assert [e.kind for e in rover.inputs.lifecycle] == ["stop"]
    book.retire(rover, 9, flushed=True)
    assert book.stopped(ROVER, 1) and book.faults == []
    assert [r for r in _results(record) if r[0] == "stop_result"] == []  # no asker


def test_a_scheduled_stop_ended_unflushed_is_a_host_fault():
    rover = _member(ROVER)
    book, _record = _book(rover)
    book.stop_scheduled(rover, 180_000, "navpy_restart:uav1")
    book.retire(rover, 9, flushed=False)
    assert not book.stopped(ROVER, 1)
    assert [f.kind for f in book.faults] == ["unclean_end"]


def test_a_scheduled_stop_has_the_stop_budget():
    rover = _member(ROVER)
    book, _record = _book(rover)
    book.stop_scheduled(rover, 180_000, "navpy_restart:uav1")
    book.check_budgets(180_000 + STOP_BUDGET_US - 1)
    assert book.faults == []
    book.check_budgets(180_000 + STOP_BUDGET_US)
    assert [f.kind for f in book.faults] == ["stop_budget"]


def test_a_respawn_is_the_same_spawn_id_and_requesters_next_generation():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.stop_scheduled(rover, 180_000, "navpy_restart:uav1")
    book.retire(rover, 9, flushed=True)
    new = book.respawn(rover, "navpy_restart:uav1")
    assert (new.spawn.spawn_id, new.spawn.requester, new.spawn.generation) == (rover.spawn.spawn_id, HUB, 2)
    assert record.data["lifecycle"][-1] == {
        "kind": "spawn", "id": record.data["lifecycle"][-1]["id"], "request": list(rover.spawn.spawn_id),
        "ordinal": ROVER, "generation": 2, "result": "ok", "payload": {"fault": "navpy_restart:uav1"}, "to": HUB}
    # It is a stop target like any generation, and an old generation's stop is stale.
    book.stop(StopRequest((HUB, 8), ROVER, 1), HUB, 200_000, draining=False)
    assert _results(record)[-1] == ("stop_result", "error", "stale generation")
