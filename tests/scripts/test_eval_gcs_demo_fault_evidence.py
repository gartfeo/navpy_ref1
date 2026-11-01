"""What a fault run's journal says was sent when, and who was present (step 7)."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.fault_schedule import PRESENCE_MARKER
from scripts.eval_gcs_demo_fault_evidence import FaultRun, deliveries, timeline_events
from tests.scripts.fault_journal_evidence import write_links

SOURCE = [2, 191]
UP = "lockstep:serial0.2"  # how a NavPy's packet reaches its SITL


def _emitted(origin, seq, sha, uid=None):
    event = ["emitted", [origin, 1, "d", seq], [0, 1], sha, 0, SOURCE, [0, 0]]
    return [*event, {"uid": uid}] if uid else event


def _delivered(origin, seq, to=7):
    return ["delivered", [origin, 1, "d", seq], to]


def _sent(ticks):
    records = [{"k": k, "t_us": k, "packets": packets} for k, packets in ticks.items()]
    return [(d.k, d.sent_k) for d in deliveries(records)[7]]


@pytest.mark.parametrize("device", ["lockstep:serial0.2", "sitl2.serial1"])
def test_a_forwarded_packet_was_sent_when_its_first_sender_emitted_those_bytes(device):
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, device)],
                  5: [_emitted("sitl2", 2, "aa"), _delivered("sitl2", 2)]}) == [(5, 1)]


def test_the_same_sender_emitting_the_same_bytes_again_is_a_new_send():
    """A heartbeat's bytes repeat once its sequence byte wraps, and each hop is first in, first out."""
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, UP)],
                  5: [_emitted(2, 2, "aa"), _delivered(2, 2, UP)],
                  7: [_emitted("sitl2", 3, "aa"), _delivered("sitl2", 3)],
                  8: [_emitted("sitl2", 4, "aa"), _delivered("sitl2", 4)]}) == [(7, 1), (8, 5)]


def test_a_copy_queued_before_its_bytes_were_sent_again_keeps_its_own_send():
    """The queued copy of a send from before a heal comes first, however late, and is no recovery."""
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, UP)],
                  5: [_emitted(2, 2, "aa"), _delivered(2, 2, UP)],
                  6: [_emitted("sitl2", 3, "aa"), _delivered("sitl2", 3)]}) == [(6, 1)]


def test_a_send_lost_on_the_way_does_not_date_a_later_send_of_the_same_bytes():
    """hw seed 21: a heartbeat emitted the tick it arrived, after the drop ate one with the same bytes."""
    assert _sent({1: [_emitted("sitl2", 1, "aa")],
                  5: [_emitted("sitl2", 2, "aa"), _delivered("sitl2", 2)]}) == [(5, 5)]


def test_a_copy_passed_on_into_a_cut_does_not_date_a_later_one_of_the_same_bytes():
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, UP), _emitted("sitl2", 2, "aa")],
                  5: [_emitted(2, 3, "aa"), _delivered(2, 3, UP),
                      _emitted("sitl2", 4, "aa"), _delivered("sitl2", 4)]}) == [(5, 5)]


def test_a_sitl_with_nothing_to_pass_on_sends_its_own():
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, UP)],
                  2: [_emitted("sitl2", 2, "aa"), _delivered("sitl2", 2)],
                  3: [_emitted("sitl2", 3, "aa"), _delivered("sitl2", 3)]}) == [(2, 1), (3, 3)]


def test_each_sitl_passes_on_its_own_copy():
    assert _sent({1: [_emitted("sitl1", 1, "aa"), _delivered("sitl1", 1, "sitl2.serial1"),
                      _delivered("sitl1", 1, "sitl3.serial1")],
                  3: [_emitted("sitl2", 2, "aa"), _delivered("sitl2", 2)],
                  4: [_emitted("sitl3", 3, "aa"), _delivered("sitl3", 3)]}) == [(3, 1), (4, 1)]


def test_another_hop_emitting_the_same_bytes_is_no_new_send():
    assert _sent({1: [_emitted(2, 1, "aa"), _delivered(2, 1, UP)],
                  3: [_emitted("sitl2", 2, "aa"), _delivered("sitl2", 2, "sitl1.serial1")],
                  5: [_emitted("sitl1", 3, "aa"), _delivered("sitl1", 3)]}) == [(5, 1)]


def test_a_copy_was_sent_with_its_parent():
    assert _sent({1: [_emitted("sitl2", 2, "aa")],
                  4: [["copy", ["router", 1, "d", 3], ["sitl2", 1, "d", 2]], ["delivered", ["router", 1, "d", 3], 7]]}
                 ) == [(4, 1)]


def test_a_navlink_uid_is_one_send_however_often_it_is_resent():
    uid = [2, 9, 1]
    assert _sent({1: [_emitted(2, 1, "aa", uid)],
                  5: [_emitted(2, 2, "bb", uid), _delivered(2, 2)]}) == [(5, 1)]


def _lifecycle_run(lines, lifecycle=()):
    records = [{"k": k, "t_us": k, **({"lifecycle": [e for at, e in lifecycle if at == k]} if lifecycle else {})}
               for k in range(1, 31)]
    return FaultRun(records, lines, {}, {})


def test_a_uav_is_present_once_its_generation_has_checked_in():
    run = _lifecycle_run({1: [(5, PRESENCE_MARKER)], 2: []})
    assert not run.present(1, 4)
    assert run.present(1, 5) and run.present(1, 30)
    assert not run.present(2, 30)


def test_a_respawn_ends_a_uavs_presence_until_it_checks_in_again():
    spawn = {"kind": "spawn", "ordinal": 1, "generation": 2, "payload": None, "result": "ok"}
    other = {"kind": "spawn", "ordinal": 2, "generation": 2, "payload": None, "result": "ok"}
    refused = {"kind": "spawn", "ordinal": 1, "generation": 2, "payload": None, "result": "error"}
    run = _lifecycle_run({1: [(5, PRESENCE_MARKER), (20, PRESENCE_MARKER)], 2: []},
                         [(10, spawn), (12, other), (25, refused)])
    assert run.present(1, 9)
    assert not run.present(1, 10) and not run.present(1, 19)
    assert run.present(1, 20) and run.present(1, 30)
    assert (run.spawned_at(1, 9), run.spawned_at(1, 10), run.spawned_at(1, 30)) == (0, 10, 10)  # the refused one is none
    assert run.spawned_at(2, 30) == 12


def test_the_companion_status_at_a_time_is_its_last_change_by_then():
    run = FaultRun([], {1: []}, {}, {1: [(10, "ok"), (20, "checking")]})
    assert [run.companion_at(1, t) for t in (9, 10, 19, 20, 99)] == [None, "ok", "ok", "checking", "checking"]
    assert run.companion_at(2, 99) is None


def test_the_timeline_keeps_each_events_changes_apart(tmp_path):
    write_links(tmp_path, {1: [(5, "connected")]}, {1: [(6, "ok")], 2: [(7, "down")]})
    assert timeline_events(tmp_path, "vehicle_link_state") == {1: [(5_000_000, "connected")]}
    assert timeline_events(tmp_path, "vehicle_companion_state") == {1: [(6_000_000, "ok")], 2: [(7_000_000, "down")]}
