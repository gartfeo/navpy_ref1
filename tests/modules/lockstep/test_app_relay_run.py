"""Step 11: relay requests enter a run with the real backend, through the coordinator's admission.

The backend runs as a participant against the reference autopilot (the 9e
double). The scripted operator connects the vehicle, then makes its own
requests while it feeds a relay double, so both kinds of traffic meet in one
tick. A name the autopilot does not have keeps a fresh read waiting on
MAVLink until its ``timeout_s``: that is the slow read the overlap, the
out-of-order completion and the cancel during MAVLink work use.
"""

from __future__ import annotations

import json

from gcs.backend.lockstep.app_refusals import refusal
from navpy.sim.lockstep.baton_sync import sleep
from navpy.sim.lockstep.coordinator_admission import (
    CANCELLED,
    RELAY,
    SCRIPT,
    RelayCancel,
    RelayInbox,
    RelayRequest,
)
from scripts.eval_gcs_demo_lifecycle import connect_vehicles
from scripts.eval_gcs_demo_stack_ops import wait_for_vehicles
from scripts.lockstep_fleet_actor import stack_context
from scripts.lockstep_slice_actor import BACKEND, SLICE
from tests.modules.lockstep.backend_doubles import build

WAIT_S = 20.0


class RelayDouble:
    """A ``RelayPort`` the script feeds; it records what comes back, in order."""

    target = BACKEND

    def __init__(self) -> None:
        self.inbox = RelayInbox()
        self.answers: dict[int, object] = {}
        self.order: list[int] = []
        self.broadcasts: list[str] = []

    def take(self):
        return self.inbox.take()

    def answer(self, ticket: int, answer: object) -> None:
        assert ticket not in self.answers, f"ticket {ticket} answered twice"
        self.answers[ticket] = answer
        self.order.append(ticket)

    def broadcast(self, data: str) -> None:
        self.broadcasts.append(data)

    def request(self, ticket: int, method: str, target: str, **extra) -> None:
        self.inbox.submit(RelayRequest(ticket, {"kind": "http", "method": method, "target": target,
                                                "deadline_s": 30.0, **extra}))

    def cancel(self, ticket: int) -> None:
        self.inbox.submit(RelayCancel(ticket))


def _named(name: str, timeout_s: float) -> str:
    return f"/api/vehicles/1/params/named?names={name}&timeout_s={timeout_s}"


def _script(relay: RelayDouble):
    def make(scenario):
        def script(operator) -> dict:
            def wait(*tickets: int) -> None:
                for _ in range(int(WAIT_S / 0.05)):
                    if all(ticket in relay.answers for ticket in tickets):
                        return
                    sleep(operator.baton, 0.05)
                raise AssertionError(f"no answer for {tickets}")

            operator.wait_ready(BACKEND)
            context = stack_context(operator, scenario, SLICE)
            connect_vehicles(context)
            wait_for_vehicles(context, WAIT_S)
            ticks: dict[str, int] = {}
            # Scripted and relay traffic in the same tick: the script's request goes first.
            relay.request(1, "GET", "/api/vehicles")
            ticks["same"] = operator.tick
            scripted = operator.call(BACKEND, {"kind": "http", "method": "GET", "target": "/api/vehicles",
                                               "deadline_s": 30.0})
            wait(1)
            # Two slow MAVLink reads overlap; a fast local read sent later finishes first.
            relay.request(2, "GET", _named("NOPE_A", 1.0))
            relay.request(3, "GET", _named("NOPE_B", 0.5))
            sleep(operator.baton, 0.2)
            relay.request(4, "GET", "/api/settings")
            wait(2, 3, 4)
            # A cancel taken in the step that took its request: never dispatched.
            relay.request(5, "GET", _named("NOPE_C", 1.0))
            relay.cancel(5)
            wait(5)
            # A cancel during MAVLink work.
            relay.request(6, "GET", _named("NOPE_D", 5.0))
            sleep(operator.baton, 0.5)
            ticks["cancel_during"] = operator.tick
            relay.cancel(6)
            wait(6)
            ticks["answered_6"] = operator.tick
            # A cancel after the response: the response stands.
            relay.request(7, "GET", "/api/settings")
            wait(7)
            relay.cancel(7)
            # A request the run does not support: refused, never dispatched.
            relay.request(8, "GET", "/api/diagnostics/export")
            wait(8)
            sleep(operator.baton, 0.5)
            return {"scripted": scripted, "ticks": ticks}

        return script

    return make


def _admissions(coordinator) -> list[tuple[int, dict]]:
    return [(r["k"], r["admission"]) for r in coordinator.journal.records if "admission" in r]


def test_relay_requests_share_the_run_with_the_script(tmp_path):
    relay = RelayDouble()
    run = build(tmp_path / "run", script=_script(relay), relay=relay, refuse=refusal)
    result = run.run()
    assert result.outcome == "timeout"  # no NavPy: the mission budget ends the mission phase
    operator, coordinator = run.operator, run.coordinator
    outcome = operator.result
    assert outcome["scripted"]["status"] == 200

    admissions = _admissions(coordinator)
    requests = [tuple(e["id"]) for _k, note in admissions for e in note["relay"] if e["kind"] == "request"]
    relay_ids = dict(enumerate(requests, start=1))  # the script sends its tickets in order
    # Same tick: one admission holds the script's request first, then the relay's,
    # with ids from the one shared counter.
    same = next(note for _k, note in admissions if note["relay"][0]["id"] == list(relay_ids[1]))
    assert [e["kind"] for e in same["script"]] == ["request"]
    assert tuple(same["script"][0]["id"]) < relay_ids[1]
    assert relay_ids[1][0] == 0 and tuple(same["script"][0]["id"])[0] == 0
    assert relay.answers[1]["status"] == 200

    # Overlap and out-of-order completion: 4 was sent last and answered first.
    assert relay.order.index(4) < relay.order.index(3) < relay.order.index(2)
    assert relay.answers[4]["status"] == 200
    for ticket in (2, 3):
        assert relay.answers[ticket]["status"] == 502  # the unanswered name, after MAVLink waits
        assert "unanswered" in json.loads(relay.answers[ticket]["body"])["detail"]

    delivered = {request.id for tick in run.ticks for request in tick.requests}
    cancels = [(tick.k, cancel) for tick in run.ticks for cancel in tick.cancels]
    # Cancel before dispatch: the coordinator answers; the backend never got it.
    assert relay.answers[5] == CANCELLED
    assert relay_ids[5] not in delivered
    assert relay_ids[5] not in {cancel for _k, cancel in cancels}
    suppressed = next(note for _k, note in admissions
                      if any(e["kind"] == "cancel" and e["id"] == list(relay_ids[5]) for e in note["relay"]))
    assert [e for e in suppressed["relay"] if e["kind"] == "cancel"][0]["answer"] == CANCELLED
    # Cancel during MAVLink work: delivered, the backend's request answers cancelled
    # well before its 5 s timeout.
    assert relay_ids[6] in delivered
    assert [k for k, cancel in cancels if cancel == relay_ids[6]]
    assert relay.answers[6] == CANCELLED
    assert outcome["ticks"]["answered_6"] - outcome["ticks"]["cancel_during"] < 50
    # Cancel after the response: journaled, not delivered, the answer stands.
    assert relay.answers[7]["status"] == 200
    assert relay_ids[7] not in {cancel for _k, cancel in cancels}
    late = [e for _k, note in admissions for e in note["relay"]
            if e["kind"] == "cancel" and e["id"] == list(relay_ids[7])]
    assert late == [{"kind": "cancel", "id": list(relay_ids[7]), "target": BACKEND, "answer": None}]
    # Refused: answered 501 by the admission, never dispatched.
    assert relay.answers[8]["status"] == 501 and relay_ids[8] not in delivered
    # Each relay answer came once, and none reached the scripted operator.
    assert sorted(relay.answers) == list(range(1, 9))
    assert not operator.journal.unclaimed
    # The backend's broadcasts reached the relay.
    assert any(json.loads(data)["type"] == "telemetry" for data in relay.broadcasts)

    # The action file: every admitted item, scripted and relay alike, with tick and id.
    actions = run.admission.actions
    assert {a["source"] for a in actions} == {SCRIPT, RELAY}
    relay_actions = [a for a in actions if a["source"] == RELAY]
    assert [a["kind"] for a in relay_actions].count("request") == 8
    assert [a["kind"] for a in relay_actions].count("cancel") == 3
    assert all(isinstance(a["k"], int) and len(a["id"]) == 2 for a in actions)


def test_a_run_without_relay_traffic_journals_no_admission(tmp_path):
    relay = RelayDouble()
    run = build(tmp_path / "run", relay=relay, refuse=refusal)
    assert run.run().outcome == "timeout"
    assert not _admissions(run.coordinator)
    assert run.admission.actions
    assert {a["source"] for a in run.admission.actions} == {SCRIPT}
