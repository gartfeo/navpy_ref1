"""Step 12: replay a run from its action file, the script off, and stop at tick N.

The run is step 11's relay run (``test_app_relay_run``): the real backend,
the scripted operator and relay traffic in one tick, overlapping reads,
cancels before dispatch, during MAVLink work and after the response, and a
refused request. Its replay feeds only the action file.
"""

from __future__ import annotations

import copy
import json

import pytest

from gcs.backend.lockstep.app_refusals import refusal
from navpy.sim.lockstep.coordinator_admission import RELAY, SCRIPT, RelayCancel, RelayRequest
from navpy.sim.lockstep.coordinator_stop import STOPPED
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, OperatorInbox, OperatorRequest
from navpy.sim.lockstep.part_wire import Response
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.replay import (
    ActionFile,
    ActionReplay,
    ReplayOperator,
    ReplayRelay,
    end_action,
    first_difference,
    load_actions,
    replay_digest,
    same_actions,
)
from scripts.eval_gcs_demo_lockstep_app import write_actions
from scripts.lockstep_slice_actor import BACKEND as SLICE_BACKEND
from tests.modules.lockstep.backend_doubles import build
from tests.modules.lockstep.test_app_relay_run import RelayDouble, _script

BACKEND = 4


def _request(k: int, seq: int, source: str = SCRIPT, target: str = "/a") -> dict:
    return {"k": k, "source": source, "kind": "request", "target": BACKEND, "id": [0, seq],
            "payload": {"kind": "http", "method": "GET", "target": target}}


def _cancel(k: int, seq: int, source: str = RELAY) -> dict:
    return {"k": k, "source": source, "kind": "cancel", "target": BACKEND, "id": [0, seq]}


def _inbox(k: int, **kwargs) -> OperatorInbox:
    return OperatorInbox(k, k * 20_000, **kwargs)


# -- the replay's parts --------------------------------------------------------------

def test_the_action_file_round_trips_with_its_end(tmp_path):
    actions = [_request(2, 3), _request(2, 4, RELAY), _cancel(5, 4)]
    write_actions(tmp_path / "a.jsonl", actions, 9)
    file = load_actions(tmp_path / "a.jsonl")
    assert file.end == 9 and same_actions(file, actions)
    write_actions(tmp_path / "b.jsonl", actions, None)
    assert load_actions(tmp_path / "b.jsonl").end is None


@pytest.mark.parametrize("lines", [
    [_request(3, 1), _request(2, 2)],  # out of tick order
    [end_action(4), _request(5, 1)],  # after the end
    [{**_request(1, 1), "source": "browser"}],
    [{**_cancel(1, 1), "payload": {}}],
    [{**_request(1, 1), "id": [0]}],
    [{**_request(1, 1), "k": 0}],
])
def test_a_line_that_is_not_an_admitted_item_is_refused(tmp_path, lines):
    path = tmp_path / "a.jsonl"
    path.write_text("".join(json.dumps(line) + "\n" for line in lines), encoding="utf-8")
    with pytest.raises(ValueError):
        load_actions(path)


def test_the_operator_sends_the_scripts_items_with_the_shared_counters_ids():
    ids = RequestIds(0)
    ids.next()  # the initial spawn's lifecycle id
    replay = ActionReplay(ActionFile((_request(2, 3), _request(2, 4, RELAY), _request(3, 5)), 6))
    operator = ReplayOperator(ids, replay)
    assert operator.run_tick(_inbox(1)) == ()
    # Id 2 was drawn by the script and never sent: skipped, as the script left it.
    assert operator.run_tick(_inbox(2)) == (OperatorRequest(BACKEND, (0, 3), _request(2, 3)["payload"]),)
    assert ids.next() == (0, 4)  # the relay's request takes the next id in the admission
    assert [item.id for item in operator.run_tick(_inbox(3))] == [(0, 5)]
    assert operator.open == {(0, 3): _request(2, 3)["payload"], (0, 5): _request(3, 5)["payload"]}
    operator.run_tick(_inbox(4, responses=((BACKEND, Response((0, 3), {"status": 200})),),
                             log_lines=((1, "line"),)))
    assert list(operator.open) == [(0, 5)] and operator.journal.log_lines == {1: ["line"]}
    assert not operator.done and operator.observations() == {}
    operator.run_tick(_inbox(6))
    assert operator.done


def test_an_id_the_counter_already_passed_is_a_divergence():
    ids = RequestIds(0)
    for _ in range(3):
        ids.next()
    operator = ReplayOperator(ids, ActionReplay(ActionFile((_request(1, 2),), None)))
    with pytest.raises(CoordinatorFault, match="shared counter"):
        operator.run_tick(_inbox(1))


def test_an_item_whose_tick_passed_untaken_is_a_divergence():
    replay = ActionReplay(ActionFile((_request(2, 1, RELAY),), None))
    relay = ReplayRelay(SLICE_BACKEND, replay)
    replay.k = 3  # the backend was not ready at tick 2
    with pytest.raises(CoordinatorFault, match="tick 2"):
        relay.take()
    assert replay.left() == {SCRIPT: 0, RELAY: 1}


def test_the_relay_gives_cancels_their_requests_ticket_and_keeps_answers_by_id():
    replay = ActionReplay(ActionFile((_request(1, 1, RELAY), _request(1, 2, RELAY), _cancel(1, 1),
                                      _cancel(2, 9)), None))
    broadcasts = []
    relay = ReplayRelay(BACKEND, replay, broadcasts.append)
    replay.k = 1
    assert relay.take() == [RelayRequest(0, _request(1, 1)["payload"]),
                            RelayRequest(1, _request(1, 2)["payload"]), RelayCancel(0)]
    relay.answer(0, {"cancelled": True})
    relay.broadcast("t")
    assert relay.answers == {(0, 1): {"cancelled": True}} and relay.open() == [(0, 2)]
    assert broadcasts == ["t"]
    replay.k = 2
    with pytest.raises(CoordinatorFault, match="never sent"):
        relay.take()


def test_the_replay_digest_leaves_out_only_the_scripts_operator_notes():
    records = [{"grid": 1}, {"k": 1, "phase": "mission", "operator": {"sent": [1]}}]
    assert replay_digest(records) == replay_digest([{"grid": 1}, {"k": 1, "phase": "mission"}])
    assert replay_digest(records) != replay_digest([{"grid": 1}, {"k": 1, "phase": "boot"}])
    assert first_difference(records, [records[0], {"k": 1, "phase": "mission"}]) is None
    assert first_difference(records, [records[0], {"k": 1, "phase": "boot"}]) == 1
    assert first_difference(records, records[:1]) == 1


# -- a run and its replays, with the real backend --------------------------------------

def _replay(tmp_path, name: str, file: ActionFile, stop_at: int | None = None):
    ids = RequestIds(0)
    replay = ActionReplay(file)
    relay = ReplayRelay(SLICE_BACKEND, replay)
    run = build(tmp_path / name, relay=relay, refuse=refusal, operator=ReplayOperator(ids, replay),
                ids=ids, stop_at=stop_at)
    return run, run.run(), replay, relay


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("replay")
    relay = RelayDouble()
    run = build(tmp_path / "run", script=_script(relay), relay=relay, refuse=refusal)
    result = run.run()
    assert run.admission.returned_at == result.final_tick  # the end line, kept even if later work faults
    write_actions(tmp_path / "actions.jsonl", run.admission.actions, run.admission.returned_at)
    return tmp_path, run, result, load_actions(tmp_path / "actions.jsonl"), relay


def test_a_replay_with_the_script_off_gives_the_runs_digest(recorded):
    tmp_path, run, result, file, live_relay = recorded
    assert {a["source"] for a in file.actions} == {SCRIPT, RELAY} and file.end == result.final_tick
    replay_run, replay_result, replay, relay = _replay(tmp_path, "replay", file)
    assert replay_result == result
    journal = run.coordinator.journal.records
    assert any("operator" in record for record in journal)  # the script's notes: left out
    assert not any("operator" in record for record in replay_run.coordinator.journal.records)
    assert first_difference(journal, replay_run.coordinator.journal.records) is None
    assert replay_digest(replay_run.coordinator.journal.records) == replay_digest(journal)
    assert same_actions(file, replay_run.admission.actions)
    assert replay.left() == {SCRIPT: 0, RELAY: 0}
    # Every relay request got the run's answer again: cancelled, refused, 200 and 502 alike.
    relay_ids = [tuple(a["id"]) for a in file.actions if a["source"] == RELAY and a["kind"] == "request"]
    assert [relay.answers[i] for i in relay_ids] == [live_relay.answers[t] for t in sorted(live_relay.answers)]


def test_a_request_taken_in_tick_k_runs_in_tick_k_plus_1_in_admission_order(recorded):
    _tmp_path, run, _result, file, _relay = recorded
    notes = {record["k"]: record["admission"] for record in run.coordinator.journal.records
             if "admission" in record}
    assert notes  # the relay traffic took admission steps
    expected: dict[int, list[tuple[int, int]]] = {}
    for k in sorted({a["k"] for a in file.actions}):
        if k in notes:  # the step's whole order: script, then relay. A refused request
            # (its answer) or one cancelled in this step (the cancel's answer) never goes out.
            step = notes[k]["script"] + notes[k]["relay"]
            suppressed = {tuple(e["id"]) for e in step if e["kind"] == "cancel" and e.get("answer")}
            entries = [e for e in step if e["kind"] == "request" and "answer" not in e
                       and tuple(e["id"]) not in suppressed]
        else:
            entries = [a for a in file.actions if a["k"] == k and a["kind"] == "request"]
        expected[k + 1] = [tuple(e["id"]) for e in entries]
    sent = {tick.k: [request.id for request in tick.requests] for tick in run.ticks}
    assert {k: sent.get(k, []) for k in expected} == expected
    # No request reached the backend in any other tick.
    assert all(k in expected for k, ids in sent.items() if ids)
    # The order is admission order, so one tick carried both sides: the script's first.
    mixed = [k for k, note in notes.items() if note["script"] and note["relay"]]
    assert mixed and expected[mixed[0] + 1][0] == tuple(notes[mixed[0]]["script"][0]["id"])


def test_a_changed_action_changes_the_digest(recorded):
    tmp_path, run, _result, file, _relay = recorded
    actions = [copy.deepcopy(a) for a in file.actions]
    index = next(i for i, a in enumerate(actions) if a["source"] == RELAY and a["kind"] == "request"
                 and a["payload"]["target"] == "/api/settings")
    actions[index]["payload"]["target"] = "/api/vehicles"
    replay_run, _result, _replay_state, _relay = _replay(tmp_path, "changed", ActionFile(tuple(actions), file.end))
    records = replay_run.coordinator.journal.records
    assert replay_digest(records) != replay_digest(run.coordinator.journal.records)
    changed = first_difference(run.coordinator.journal.records, records)
    assert records[changed]["k"] == actions[index]["k"]  # its admission note holds the payload's hash


def test_a_replay_stops_at_tick_n_with_its_journal_and_a_snapshot(recorded):
    tmp_path, run, result, file, _relay = recorded
    stop_at = (file.actions[0]["k"] + result.final_tick) // 2
    replay_run, stopped, replay, _relay = _replay(tmp_path, "stopped", file, stop_at=stop_at)
    assert (stopped.outcome, stopped.final_tick) == (STOPPED, stop_at)
    records = replay_run.coordinator.journal.records
    last = records[-1]
    assert last["stopped_at"] == stop_at and last["phase"] in ("mission", "finalization")
    snapshot = last["snapshot"]
    assert [m["ordinal"] for m in snapshot["members"]] == [SLICE_BACKEND] and snapshot["members"][0]["live"]
    assert len(snapshot["sitls"]) == 1 and snapshot["sitls"][0]["source_us"] == records[-2]["t_us"]
    full = run.coordinator.journal.records
    tick_n = max(i for i, r in enumerate(full) if r.get("k") == stop_at and "phase" in r)
    assert first_difference(full[:tick_n + 1], records[:-1]) is None
    assert replay_digest(full[:tick_n + 1]) == replay_digest(records[:-1])
    assert same_actions(ActionFile(tuple(a for a in file.actions if a["k"] <= stop_at), None),
                        replay_run.admission.actions)
    assert replay.left()[SCRIPT] + replay.left()[RELAY] == sum(a["k"] > stop_at for a in file.actions)
