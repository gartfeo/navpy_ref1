"""The lifecycle book in isolation: stops of ended generations and unflushed ends."""

from __future__ import annotations

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_lifecycle import LifecycleBook
from navpy.sim.lockstep.coordinator_members import Member
from navpy.sim.lockstep.coordinator_spawn import SpawnRecord
from navpy.sim.lockstep.part_wire import StopRequest
from navpy.sim.lockstep.participant_hooks import RequestIds

ROVER, HUB = 1, 2


class _Roster:
    def __init__(self, *members: Member) -> None:
        self.members = list(members)

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
    assert book.faults == []


def test_a_duplicate_of_a_completed_stop_keeps_its_answer():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.stop(StopRequest((HUB, 5), ROVER, 1), HUB, 180_000, draining=False)
    book.retire(rover, 9, flushed=True)
    book.stop(StopRequest((HUB, 6), ROVER, 1), HUB, 180_000, draining=False)
    assert [r for r in _results(record) if r[0] == "stop_result"] == [
        ("stop_result", "acked", None), ("stop_result", "acked", None)]
    assert book.stopped(ROVER, 1)


def test_every_unflushed_end_is_a_host_fault_even_with_no_stop_pending():
    rover = _member(ROVER)
    book, record = _book(rover)
    book.retire(rover, 9, flushed=False)
    assert [(f.kind, f.tick, f.ordinal) for f in book.faults] == [("unclean_end", 9, ROVER)]
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
