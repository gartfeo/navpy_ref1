"""The CPU share record (bounded_cpu_share.md): claims, owners, expiry and its mutex."""

from __future__ import annotations

import json
import os
import sys
import threading

import pytest

from gcs.backend import cpu_share
from gcs.backend.cpu_share import MAX_SHARES, SHARE_CPUS, Share, ShareRecord, owner_alive, pick_cpus

#: This PC's hybrid CPU: 8 performance cores (class 1) and 12 efficiency cores.
P_CORES = (0, 1, 6, 7, 8, 9, 18, 19)
HYBRID = {cpu: int(cpu in P_CORES) for cpu in range(20)}
START = 1234


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture(autouse=True)
def own_start(monkeypatch):
    monkeypatch.setattr(cpu_share.runtime, "pid_start_time", lambda _pid: START)


def _record(tmp_path, alive=lambda _pid, _start: True, clock=None) -> ShareRecord:
    return ShareRecord(tmp_path / "cpu_shares.json", alive=alive, clock=clock or Clock())


def _write(path, *shares: dict) -> None:
    path.write_text(json.dumps({"shares": list(shares)}), encoding="utf-8")


def test_pick_cpus_takes_the_fastest_class_outside_the_taken_cpus():
    assert pick_cpus(HYBRID, set()) == [0, 1, 6, 7]
    assert pick_cpus(HYBRID, {0, 1, 6, 7}) == [8, 9, 18, 19]
    assert pick_cpus(HYBRID, set(P_CORES)) == []  # never an efficiency core
    assert pick_cpus({cpu: 0 for cpu in range(6)}, {0}) == [1, 2, 3, 4]  # not hybrid
    assert pick_cpus({}, set()) == []


def test_claim_writes_a_share_on_free_performance_cores(tmp_path):
    record = _record(tmp_path)
    share = record.claim("eval", 60.0, HYBRID)
    assert isinstance(share, Share)
    assert (share.pid, share.start, share.cpus, share.expires_at) == (
        os.getpid(), START, (0, 1, 6, 7), 1060.0)
    assert [Share.from_json(raw) for raw in json.loads(record.path.read_text())["shares"]] == [share]


def test_shares_are_disjoint_and_bounded(tmp_path):
    record = _record(tmp_path)
    first, second = record.claim("a", 60.0, HYBRID), record.claim("b", 60.0, HYBRID)
    assert set(first.cpus).isdisjoint(second.cpus) and len(second.cpus) == SHARE_CPUS
    assert record.claim("c", 60.0, HYBRID) == f"{MAX_SHARES} shares are held (at most {MAX_SHARES})"


def test_a_claim_without_enough_free_performance_cores_is_refused_at_once(tmp_path):
    record = _record(tmp_path)
    five = {cpu: int(cpu < 5) for cpu in range(10)}
    assert isinstance(record.claim("a", 60.0, five), Share)
    assert record.claim("b", 60.0, five) == "no free performance cores"


def test_refusals_without_cpu_information_or_an_own_start_time(tmp_path, monkeypatch):
    record = _record(tmp_path)
    assert record.claim("a", 60.0, {}) == "no CPU information on this platform"
    monkeypatch.setattr(cpu_share.runtime, "pid_start_time", lambda _pid: None)
    assert record.claim("a", 60.0, HYBRID) == "this process's start time is unreadable"
    assert not record.path.exists()


def test_a_dead_owners_entry_is_dropped_by_the_next_claim(tmp_path):
    record = _record(tmp_path, alive=lambda pid, _start: pid != 999)
    _write(record.path, {"id": "old", "pid": 999, "start": 1, "label": "gone",
                         "cpus": [0, 1, 6, 7], "expires_at": 5000.0})
    assert record.avoided() == (set(), None)
    share = record.claim("eval", 60.0, HYBRID)
    assert share.cpus == (0, 1, 6, 7)
    assert [raw["id"] for raw in json.loads(record.path.read_text())["shares"]] == [share.id]


def test_owner_identity_is_strict(monkeypatch):
    monkeypatch.setattr(cpu_share.runtime, "pid_alive", lambda _pid: True)
    monkeypatch.setattr(cpu_share.runtime, "pid_start_time", lambda _pid: 5)
    assert owner_alive(42, 5)
    assert not owner_alive(42, 6)  # a reused pid
    assert not owner_alive(42, None)
    monkeypatch.setattr(cpu_share.runtime, "pid_alive", lambda _pid: False)
    assert not owner_alive(42, 5)


def test_an_expired_share_stays_held_but_background_no_longer_avoids_it(tmp_path):
    clock = Clock()
    record = _record(tmp_path, clock=clock)
    four = {cpu: int(cpu < 4) for cpu in range(8)}
    share = record.claim("eval", 60.0, four)
    assert record.avoided() == ({0, 1, 2, 3}, 1060.0)
    clock.now = 1060.0
    assert record.avoided() == (set(), None)
    assert record.listing() == [(share, "expired, held until released")]
    assert record.claim("next", 60.0, four) == "no free performance cores"
    assert record.release(share)
    assert isinstance(record.claim("next", 60.0, four), Share)


def test_avoided_names_every_valid_share_and_the_earliest_expiry(tmp_path):
    record = _record(tmp_path)
    first, second = record.claim("a", 60.0, HYBRID), record.claim("b", 30.0, HYBRID)
    assert record.avoided() == (set(first.cpus) | set(second.cpus), 1030.0)


def test_release_removes_only_that_share(tmp_path):
    record = _record(tmp_path)
    first, second = record.claim("a", 60.0, HYBRID), record.claim("b", 60.0, HYBRID)
    assert record.release(first)
    assert [share for share, _state in record.listing()] == [second]


def test_listing_states_each_entry_and_changes_nothing(tmp_path):
    record = _record(tmp_path, alive=lambda pid, _start: pid != 999)
    share = record.claim("eval", 90.0, HYBRID)
    raw = json.loads(record.path.read_text())["shares"]
    _write(record.path, *raw, {"id": "old", "pid": 999, "start": 1, "label": "gone",
                               "cpus": [8], "expires_at": 0.0})
    before = record.path.read_text()
    assert [state for _share, state in record.listing()] == ["held, 90 s left", "owner gone"]
    assert record.listing()[0][0] == share
    assert record.path.read_text() == before


def test_an_unreadable_record_fails_closed(tmp_path):
    record = _record(tmp_path)
    share = record.claim("eval", 60.0, HYBRID)
    record.path.write_text("{not json", encoding="utf-8")
    assert record.claim("next", 60.0, HYBRID).startswith("the share record is unusable")
    assert not record.release(share)  # its CPUs stay held until the owner exits
    assert record.avoided() == (set(), None)  # priority still ranks the share first
    with pytest.raises(ValueError):
        record.listing()


def test_an_unwritable_record_refuses_the_claim_and_keeps_a_release_held(tmp_path, monkeypatch):
    record = _record(tmp_path)
    share = record.claim("eval", 60.0, HYBRID)

    def denied(_shares):
        raise PermissionError("in use")

    monkeypatch.setattr(record, "_save", denied)
    assert record.claim("next", 60.0, HYBRID).startswith("the share record is unusable")
    assert not record.release(share)


windows = pytest.mark.skipif(sys.platform != "win32", reason="named mutex: Windows only")


def _hold_mutex(path, done: threading.Event, release: bool) -> threading.Thread:
    """Another transaction: it holds the record's mutex until *done*."""
    taken = threading.Event()

    def hold():
        other = ShareRecord(path)
        assert other._mutex.acquire(0.0)
        taken.set()
        done.wait()
        if release:
            other._mutex.release()

    thread = threading.Thread(target=hold)
    thread.start()
    assert taken.wait(10.0)
    return thread


@windows
def test_a_busy_record_refuses_after_the_bounded_wait(tmp_path, monkeypatch):
    monkeypatch.setattr(cpu_share, "LOCK_TIMEOUT_S", 0.05)
    record = _record(tmp_path)
    share = record.claim("eval", 60.0, HYBRID)
    done = threading.Event()
    thread = _hold_mutex(record.path, done, release=True)
    try:
        assert record.claim("next", 60.0, HYBRID) == "the share record stayed busy"
        assert not record.release(share)
        assert record.avoided() == (set(), None)
    finally:
        done.set()
        thread.join()
    assert record.release(share)


@windows
def test_a_dead_holders_mutex_is_handed_over_at_once(tmp_path):
    record = _record(tmp_path)
    done = threading.Event()
    done.set()
    _hold_mutex(record.path, done, release=False).join()  # ends holding it: abandoned
    assert isinstance(record.claim("eval", 60.0, HYBRID), Share)
