"""Roster teardown: one failing step never stops the others."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.coordinator_members import Member, Roster
from navpy.sim.lockstep.coordinator_spawn import SpawnRecord
from navpy.sim.lockstep.coordinator_types import HostFault, WallBudgets
from tests.modules.lockstep.coordinator_doubles import AllowAll, manifest


class _Child:
    def __init__(self, pid: int, *, fails: bool = False) -> None:
        self.pid = pid
        self.fails = fails
        self.killed = False

    def kill(self) -> None:
        if self.fails:
            raise PermissionError("access denied")
        self.killed = True

    def wait(self, timeout_s: float):
        return 0 if self.killed else None

    def go(self) -> None:
        raise RuntimeError("go failed")


class _Link:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _Spawner:
    def __init__(self, child) -> None:
        self.child = child

    def spawn(self, *_args):
        return self.child


def _member(ordinal: int, child: _Child) -> Member:
    return Member(SpawnRecord((0, ordinal), "hub", ordinal, 1, "c", 0), child, _Link(), ())


def test_one_failing_kill_still_stops_every_later_child_and_the_listener(tmp_path):
    roster = Roster(manifest(), _Spawner(None), AllowAll(), None, WallBudgets())
    first, failing, last = _Child(1), _Child(2, fails=True), _Child(3)
    roster.members = [_member(1, first), _member(2, failing), _member(3, last)]
    with pytest.raises(HostFault, match="access denied") as caught:
        roster.stop_all()
    assert caught.value.kind == "teardown_failed"
    assert first.killed and last.killed
    assert all(member.link.closed for member in roster.members)
    with pytest.raises(OSError):  # the listener was closed too
        roster._listener.accept(0.01)


class _ForgetFails(AllowAll):
    def forget(self, child, spawn) -> None:
        raise OSError("registry unwritable")


def test_a_spawn_unwind_runs_every_step_and_keeps_the_original_error(tmp_path):
    child = _Child(9, fails=True)  # both unwind steps fail
    roster = Roster(manifest(), _Spawner(child), _ForgetFails(), None, WallBudgets())
    with pytest.raises(RuntimeError, match="go failed"):
        roster.spawn("hub", 2, (0, 1), 0)
    assert roster.members == []


class _Recorder(AllowAll):
    def __init__(self) -> None:
        super().__init__()
        self.forgotten: list = []

    def forget(self, child, spawn) -> None:
        self.forgotten.append(child.pid)


def test_a_child_that_survives_its_unwind_kill_stays_recorded_and_stop_all_retries_it():
    child = _Child(9, fails=True)
    recorder = _Recorder()
    roster = Roster(manifest(), _Spawner(child), recorder, None, WallBudgets(exit_s=0.1))
    with pytest.raises(RuntimeError, match="go failed"):  # the launch fails after recording
        roster.spawn("hub", 2, (0, 1), 0)
    assert recorder.forgotten == [] and [pid for pid, _ in recorder.children] == [9]
    assert [c.pid for c, _ in roster.unadmitted] == [9]
    child.fails = False  # the next teardown can kill it
    roster.stop_all()
    assert child.killed and roster.unadmitted == [] and recorder.forgotten == [9]


def test_a_confirmed_unwind_releases_the_record():
    child = _Child(9)
    recorder = _Recorder()
    roster = Roster(manifest(), _Spawner(child), recorder, None, WallBudgets(exit_s=0.1))
    with pytest.raises(RuntimeError, match="go failed"):
        roster.spawn("hub", 2, (0, 1), 0)
    assert recorder.forgotten == [9] and roster.unadmitted == []


def test_a_member_killed_but_not_confirmed_gone_is_a_teardown_failure():
    class Lingering(_Child):
        def wait(self, timeout_s):
            return None

    roster = Roster(manifest(), _Spawner(None), AllowAll(), None, WallBudgets(exit_s=0.1))
    roster.members = [_member(1, Lingering(1))]
    with pytest.raises(HostFault, match="not confirmed gone"):
        roster.stop_all()
    assert not roster.members[0].exited
