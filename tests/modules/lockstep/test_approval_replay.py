"""The approval gate's polling-schedule replay (``lockstep_fleet.md``, "Approval timing").

Passing journals come from the real confirmation workflow on the operator's
baton with its evidence on (``approval_runs``); failing ones are those
journals with one record moved or changed. Grids: the recorded serial-step
frames and uniform 10 ms and 20 ms grids.
"""

from __future__ import annotations

import copy
from functools import lru_cache
from types import SimpleNamespace

import pytest

from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts import eval_gcs_demo_mission as mission
from scripts.eval_gcs_demo_approval_replay import approval_replay_errors
from scripts.eval_gcs_demo_confirmation import CONFIRMATION_POLL_S, ConfirmationRequest
from scripts.eval_gcs_demo_confirmation_timing import APPROVAL_TARGET, tick_times
from scripts.eval_gcs_demo_scenario import load_scenario
from scripts.lockstep_fleet_actor import stack_context
from scripts.lockstep_slice_actor import SLICE
from tests.modules.lockstep.approval_runs import TELEMETRY, UID, request, run_workflow, uniform
from tests.modules.lockstep.test_grid_check_recording import recorded_frames
from scripts.eval_gcs_demo_constants import ROOT

GRID_US = 20_000
#: Vehicles 2 and 3 are consumed a pass or two after vehicle 1, so both
#: become due while vehicle 1's approval POST blocks (0.5 s of resends).
FLEET = {100: [request(1)], 103: [request(2)], 106: [request(3)]}


def _entries(records, key):
    return [(r["k"], entry) for r in records for entry in r.get("operator", {}).get(key, ())]


def _receives(records):
    """Each receive's ``[start, return, ended by]`` (its subscription dropped)."""
    return [(k, e[:3]) for k, e in _entries(records, "receive")]


def _posts(records):
    return [(k, e) for k, e in _entries(records, "sent") if e[2] == "POST" and e[3] == APPROVAL_TARGET]


def _answer_tick(records, post):
    return next(k for k, e in _entries(records, "answered") if e[:2] == post[:2])


def _replace(records, key, old_k, old, new_k, new):
    """The journal with entry ``old`` of tick ``old_k``'s ``key`` moved to ``new_k`` as ``new``."""
    changed = copy.deepcopy(records)
    by_k = {r["k"]: r for r in changed if "k" in r}
    entries = by_k[old_k]["operator"][key]
    entries.remove(old)
    if not entries:
        del by_k[old_k]["operator"][key]
    by_k[new_k].setdefault("operator", {}).setdefault(key, []).append(new)
    return changed


@lru_cache
def _single(grid: str):
    frames = recorded_frames() if grid == "recorded" else uniform(10_000, 2_000)
    return run_workflow(frames, {100: [TELEMETRY, request(1)]}, {1}, max_gap_us=20_825)


@lru_cache
def _fleet(extra: tuple = ()):
    deliveries = {k: list(v) for k, v in FLEET.items()}
    for k, data in extra:
        deliveries.setdefault(k, []).append(data)
    return run_workflow(uniform(GRID_US, 2_000), deliveries, {1, 2, 3}, max_gap_us=GRID_US)


@pytest.mark.parametrize("grid", ["recorded", "uniform"])
def test_the_real_workflow_passes_the_replay(grid):
    records, approvals = _single(grid)
    assert approval_replay_errors(records, approvals) == []


def test_a_request_due_during_a_blocking_post_is_sent_in_the_next_pass():
    records, approvals = _fleet()
    assert approval_replay_errors(records, approvals) == []
    first, second, third = _posts(records)
    t = tick_times(records)
    k3 = {tuple(e[2:5]): k for k, e in _entries(records, "received")}
    answered = _answer_tick(records, first[1])
    # Vehicle 2 became due during vehicle 1's POST, and waited for the next pass.
    assert t[k3[(2, 1, UID)]] + 15_000_000 <= t[answered] < t[second[0]]
    checks = {k: due for k, (_k, _now, due) in _entries(records, "due_check")}
    assert checks[second[0]] == [[2, 1, UID], [3, 1, UID]]
    # Vehicle 3, due in the same pass, goes out at vehicle 2's answer tick.
    assert third[0] == _answer_tick(records, second[1])
    assert [row["sys_id"] for row in approvals] == [1, 2, 3]


def _sleep_with_room(records):
    return next((start, wake) for _k, (start, wake) in _entries(records, "sleep") if wake - start >= 2)


def test_an_event_queued_during_a_sleep_is_consumed_by_the_next_receive():
    start, wake = _sleep_with_room(_fleet()[0])
    records, approvals = _fleet(((start + 1, TELEMETRY),))
    assert approval_replay_errors(records, approvals) == []
    assert [wake, wake, "event"] in [e for _k, e in _receives(records)]
    assert [start + 1, "telemetry", 1, None, None] in [e[:5] for _k, e in _entries(records, "received")]


def test_an_event_queued_during_another_approvals_post_is_consumed_by_the_next_receive():
    base = _fleet()[0]
    post = _posts(base)[0]
    queued = post[0] + 5
    assert queued < _answer_tick(base, post[1])
    records, approvals = _fleet(((queued, TELEMETRY),))
    assert approval_replay_errors(records, approvals) == []
    (consumed,) = [k for k, e in _entries(records, "received") if e[:2] == [queued, "telemetry"]]
    assert [consumed, consumed, "event"] in [e for _k, e in _receives(records)]


def test_a_delivery_on_a_receives_timeout_tick_wins():
    base = _fleet()[0]
    _k, (start, timeout, _ended) = next((k, e) for k, e in _receives(base)
                                        if e[2] == "timeout" and e[0] > 200)
    records, approvals = _fleet(((timeout, TELEMETRY),))
    assert approval_replay_errors(records, approvals) == []
    assert [start, timeout, "event"] in [e for _k, e in _receives(records)]


def test_an_unexplained_late_pass_fails():
    records, approvals = _fleet()
    k, sleep = _entries(records, "sleep")[3]
    late = _replace(records, "sleep", k, sleep, k + 1, [sleep[0], sleep[1] + 1])
    (error,) = approval_replay_errors(late, approvals)
    assert f"sleep [{sleep[0]}, {sleep[1] + 1}], expected [{sleep[0]}, {sleep[1]}]" in error
    k, receive = next((k, e) for k, e in _entries(records, "receive") if e[2] == "timeout")
    slow = _replace(records, "receive", k, receive, k + 1, [receive[0], k + 1, "timeout", *receive[3:]])
    (error,) = approval_replay_errors(slow, approvals)
    assert f"returned at {k + 1} (timeout), expected {k} (timeout)" in error


def test_an_unexplained_late_send_fails():
    records, approvals = _fleet()
    k, third = _posts(records)[2]
    late = _replace(records, "sent", k, third, k + 1, third)
    (error,) = approval_replay_errors(late, approvals)
    assert f"approval POST for [3, 1, '{UID}'] at tick {k + 1}, expected [3, 1, '{UID}'] at tick {k}" in error


def test_an_unexplained_delayed_consumption_fails():
    start, wake = _sleep_with_room(_fleet()[0])
    records, approvals = _fleet(((start + 1, TELEMETRY),))
    receive = next(e for k, e in _entries(records, "receive") if e[:3] == [wake, wake, "event"])
    delayed = _replace(records, "receive", wake, receive, wake + 1, [wake, wake + 1, "event", *receive[3:]])
    (error,) = approval_replay_errors(delayed, approvals)
    assert f"receive from tick {wake} returned at {wake + 1} (event), expected {wake} (event)" in error


def test_a_consumption_out_of_delivery_order_fails():
    records, approvals = _single("recorded")
    (k, telemetry), (k2, confirm) = _entries(records, "received")
    # Both queued at one tick; the request is claimed to be consumed first.
    swapped = _replace(records, "received", k, telemetry, k, confirm)
    swapped = _replace(swapped, "received", k2, confirm, k2, telemetry)
    (error,) = approval_replay_errors(swapped, approvals)
    assert "not the oldest queued" in error


def test_a_due_list_that_is_not_the_replays_fails():
    records, approvals = _single("uniform")
    due = [(k, e) for k, e in _entries(records, "due_check") if e[2]]
    (k4, check) = due[0]
    previous_k, previous = max((k, e) for k, e in _entries(records, "due_check") if k < k4)
    early = _replace(records, "due_check", previous_k, previous, previous_k, [*previous[:2], check[2]])
    (error,) = approval_replay_errors(early, approvals)
    assert f"due list {check[2]} at tick {previous_k}, expected []" in error


def test_the_audit_and_the_evidence_must_match_the_journal():
    records, approvals = _fleet()
    hasty = [dict(approvals[0], actual_delay_s=approvals[0]["actual_delay_s"] - 0.02), *approvals[1:]]
    assert "audited actual_delay_s" in approval_replay_errors(records, hasty)[0]
    assert "are not the journal's POSTs" in approval_replay_errors(records, approvals[:2])[0]
    plain, plain_approvals = run_workflow(uniform(GRID_US, 2_000), FLEET, {1, 2, 3},
                                          max_gap_us=GRID_US, evidence=False)
    assert approval_replay_errors(plain, plain_approvals) == [
        "approval replay: the journal holds no due checks (operator evidence off)"]


def test_the_lockstep_stack_context_journals_due_checks_only_with_evidence():
    scenario = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")
    for evidence in (False, True):
        operator = OperatorBaton(RequestIds(0), lambda _operator: None, evidence=evidence)
        context = stack_context(operator, scenario, SLICE)
        if not evidence:
            assert context.on_due_check is None and context.on_polling_start is None
            continue
        context.on_polling_start(61.25)
        context.on_due_check(1.5, [ConfirmationRequest(1, 1, UID)])
        assert operator.journal.tick == {"polling": [[0, 61.25]],
                                         "due_check": [[0, 1_500_000, [[1, 1, UID]]]]}


@pytest.mark.parametrize("hook", [None, print])
def test_the_mission_hands_the_contexts_due_check_hook_to_the_loop(monkeypatch, hook):
    """Wall mode's context has none, so its loop runs as before; lockstep's journals."""
    seen = {}

    def workflow(**kwargs):
        seen.update(kwargs)
        return [{"sys_id": 1, "task_id": 1}]

    monkeypatch.setattr(mission, "run_confirmation_workflow", workflow)
    scenario = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")
    context = SimpleNamespace(sys_ids=(1,), scenario=scenario, clock=None, on_due_check=hook,
                              on_polling_start=None if hook is None else id,
                              api=SimpleNamespace(websocket_url="ws://unused"),
                              event_stream_factory=lambda _url: None)
    assert mission.approve_requests_until_snap(context, None, None, 1.0, 0.0) == [
        {"sys_id": 1, "task_id": 1}]
    assert seen["on_due_check"] is hook
    assert seen["on_polling_start"] is context.on_polling_start


def test_the_lockstep_eval_runs_its_operator_with_evidence(tmp_path):
    from scripts.eval_gcs_demo_lockstep import LockstepOptions, lockstep_operator

    scenario = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")
    options = LockstepOptions(seed=1, timeout_s=600.0, confirmation_delay_s=15.0)
    assert lockstep_operator(RequestIds(0), scenario, tmp_path, options).journal.evidence


def test_a_wait_the_deadline_shortened_is_explained():
    """Codex 10g-2 round 1 P2-1: polling for SNAP after the approval, the
    loop's last receive ends at its deadline, short of the full poll, and the
    run still succeeds (its stop holds at that due check)."""
    records, approvals = run_workflow(uniform(10_000, 2_300), {100: [request(1)]}, {1},
                                      max_gap_us=10_000, timeout_s=20.03, stop_at_s=20.03)
    t = tick_times(records)
    start, end, ended = _receives(records)[-1][1]
    assert ended == "timeout" and t[end] - t[start] < CONFIRMATION_POLL_S * 1e6
    assert approval_replay_errors(records, approvals) == []
    # A pass the loop could not have started: the deadline before its start.
    ((k, polling),) = _entries(records, "polling")
    early = _replace(records, "polling", k, polling, k, [polling[0], t[start] / 1e6])
    assert approval_replay_errors(early, approvals) == [
        f"approval replay: pass {len(_receives(records)) - 1}: starts at tick {start}, "
        f"at or after the deadline {t[start] / 1e6}"]


def test_missing_initial_passes_fail():
    """Codex 10g-2 round 1 P2-2: the first receive starts at the polling start."""
    records, approvals = _fleet()
    changed = copy.deepcopy(records)
    ((_k, polling),) = _entries(records, "polling")
    first_check_k = _entries(records, "due_check")[0][0]
    for record in changed:
        operator = record.get("operator", {})
        for key in ("receive", "due_check", "sleep"):
            operator[key] = [e for e in operator.get(key, ()) if not (
                (key == "sleep" and e[0] == first_check_k) or (key != "sleep" and record["k"] == first_check_k))]
    (error,) = approval_replay_errors(changed, approvals)
    assert "pass 0: receive starts at tick" in error and f"not {polling[0]}" in error


def test_receives_on_another_stream_are_not_the_loops():
    records, approvals = _fleet()
    k, receive = _entries(records, "receive")[5]
    other = copy.deepcopy(records)
    other_stream = [receive[0], receive[1], "timeout", 9, 9]
    next(r for r in other if r.get("k") == k)["operator"]["receive"].append(other_stream)
    assert approval_replay_errors(other, approvals) == []
    same = copy.deepcopy(records)
    next(r for r in same if r.get("k") == k)["operator"]["receive"].append(
        [receive[0], receive[1], "timeout", *receive[3:]])
    (error,) = approval_replay_errors(same, approvals)
    assert "receives on the loop's stream for" in error


def test_same_type_requests_consumed_out_of_delivery_order_fail():
    """Codex 10g-2 round 1 P2-3: deliveries carry the confirmation identity."""
    records, approvals = run_workflow(uniform(GRID_US, 2_000), {100: [request(1), request(2)]}, {1, 2},
                                      max_gap_us=GRID_US)
    assert approval_replay_errors(records, approvals) == []
    (k1, first), (k2, second) = [(k, e) for k, e in _entries(records, "received") if e[1] == "task_confirm_request"]
    swapped = _replace(records, "received", k1, first, k1, second)
    swapped = _replace(swapped, "received", k2, second, k2, first)
    (error,) = approval_replay_errors(swapped, approvals)
    assert "not the oldest queued" in error


def test_an_unreplayed_consumption_or_post_fails():
    """Codex 10g-2 round 1 P2-4: every consumption on the loop's stream and
    every approval POST belongs to a replayed pass."""
    records, approvals = _fleet()
    k, received = _entries(records, "received")[-1]
    extra = copy.deepcopy(records)
    next(r for r in extra if r.get("k") == k)["operator"]["received"].append(received)
    assert approval_replay_errors(extra, approvals) == [
        "approval replay: 1 consumptions outside the replayed passes"]
    k, post = _posts(records)[-1]
    extra = copy.deepcopy(records)
    next(r for r in extra if r.get("k") == k)["operator"]["sent"].append([9, 9, *post[2:]])
    assert approval_replay_errors(extra, approvals) == [
        "approval replay: 1 approval POSTs outside the replayed passes"]


def test_a_last_receive_that_returns_an_event_needs_no_timeout_tick():
    """Codex 10g-2 round 2 P2-1: the loop stops on an event delivered before
    the receive's timeout; the journal ends there, before that timeout tick."""
    base, _approvals = run_workflow(uniform(GRID_US, 2_000), FLEET, {1, 2, 3}, max_gap_us=GRID_US,
                                    stop_at_s=20.0)
    t = tick_times(base)
    start = next(e[0] for _k, e in _receives(base) if t[e[0]] >= 19_000_000)
    stop_at = t[start + 1] / 1e6
    records, approvals = run_workflow(uniform(GRID_US, 2_000), {**FLEET, start + 1: [TELEMETRY]}, {1, 2, 3},
                                      max_gap_us=GRID_US, stop_at_s=stop_at)
    assert _receives(records)[-1][1] == [start, start + 1, "event"]
    assert max(t for t in tick_times(records)) == start + 1  # the journal ends at the return
    assert approval_replay_errors(records, approvals) == []


def test_missing_trailing_passes_fail():
    """Codex 10g-2 round 2 P2-2: the loop ends where its stream is unsubscribed;
    passes polled after the approvals (for SNAP) cannot be dropped."""
    records, approvals = run_workflow(uniform(GRID_US, 2_000), FLEET, {1, 2, 3}, max_gap_us=GRID_US,
                                      stop_at_s=20.0)
    assert approval_replay_errors(records, approvals) == []
    last_post = _answer_tick(records, _posts(records)[-1][1])
    cut = [k for k, _e in _entries(records, "due_check") if k > last_post][-3]
    # Drop the timeout-only passes after the cut (their receives, due checks
    # and sleeps); the unsubscribe and the tick grid stay.
    changed = copy.deepcopy(records)
    for record in changed:
        operator = record.get("operator", {})
        for key, keep in (("receive", lambda e: e[0] < cut), ("due_check", lambda e: e[0] <= cut),
                          ("sleep", lambda e: e[0] < cut)):
            if key in operator:
                operator[key] = [e for e in operator[key] if keep(e)]
    (error,) = approval_replay_errors(changed, approvals)
    assert f"last replayed pass ends at tick {cut}, but its stream is unsubscribed" in error
