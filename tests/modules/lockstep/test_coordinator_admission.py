"""Step 11: the coordinator's admission step, the 1x pacer and the app's refusals."""

from __future__ import annotations

import pytest

from gcs.backend.lockstep import app_refusals
from navpy.sim.lockstep.coordinator_admission import (
    CANCELLED,
    Admission,
    RelayCancel,
    RelayInbox,
    RelayRequest,
    payload_hash,
)
from navpy.sim.lockstep.coordinator_pacing import RealTimePacer
from navpy.sim.lockstep.coordinator_types import OperatorCancel, OperatorRequest
from navpy.sim.lockstep.part_wire import Event, Response
from navpy.sim.lockstep.participant_hooks import RequestIds

BACKEND = 4


class Relay:
    target = BACKEND

    def __init__(self) -> None:
        self.inbox = RelayInbox()
        self.answers: list[tuple[int, object]] = []
        self.broadcasts: list[str] = []

    def take(self):
        return self.inbox.take()

    def answer(self, ticket, answer):
        self.answers.append((ticket, answer))

    def broadcast(self, data):
        self.broadcasts.append(data)


def _http(target: str, method: str = "GET") -> dict:
    return {"kind": "http", "method": method, "target": target}


def _admission(**kwargs):
    relay, ids = Relay(), RequestIds(0)
    return Admission(ids, relay, **kwargs), relay, ids


def test_scripted_items_go_first_then_relay_items_in_arrival_order_with_shared_ids():
    admission, relay, ids = _admission()
    scripted = OperatorRequest(BACKEND, ids.next(), _http("/a"))
    relay.inbox.submit(RelayRequest(7, _http("/b")))
    relay.inbox.submit(RelayRequest(3, _http("/c")))
    deliveries, note = admission.admit(5, [scripted], lambda ordinal: True)
    assert deliveries == [scripted, OperatorRequest(BACKEND, (0, 2), _http("/b")),
                          OperatorRequest(BACKEND, (0, 3), _http("/c"))]
    assert note == {"script": [{"kind": "request", "id": [0, 1], "target": BACKEND,
                                "payload": payload_hash(_http("/a"))}],
                    "relay": [{"kind": "request", "id": [0, 2], "target": BACKEND,
                               "payload": payload_hash(_http("/b"))},
                              {"kind": "request", "id": [0, 3], "target": BACKEND,
                               "payload": payload_hash(_http("/c"))}]}
    assert [(a["k"], a["source"], a["id"]) for a in admission.actions] == [
        (5, "script", [0, 1]), (5, "relay", [0, 2]), (5, "relay", [0, 3])]


def test_responses_go_back_by_id_in_any_order_and_the_operator_keeps_its_own():
    admission, relay, ids = _admission()
    relay.inbox.submit(RelayRequest(1, _http("/slow")))
    relay.inbox.submit(RelayRequest(2, _http("/fast")))
    admission.admit(1, [], lambda ordinal: True)
    mine = Response((0, 99), {"status": 200, "body": "mine"})
    kept = admission.route([(BACKEND, Response((0, 2), {"status": 200, "body": "fast"})), (BACKEND, mine)],
                           [(BACKEND, Event("broadcast", {"seq": 1, "data": "t"})),
                            (BACKEND, Event("ws", {"subscription": [0, 9], "data": "x"})),
                            (1, Event("broadcast", {"seq": 1, "data": "not the backend"}))])
    assert kept == ((BACKEND, mine),)
    assert admission.route([(BACKEND, Response((0, 1), {"status": 502, "body": "slow"}))], []) == ()
    assert relay.answers == [(2, {"status": 200, "body": "fast"}), (1, {"status": 502, "body": "slow"})]
    assert relay.broadcasts == ["t"]


def test_a_cancel_in_the_step_that_took_its_request_is_resolved_by_the_coordinator():
    admission, relay, _ids = _admission()
    relay.inbox.submit(RelayRequest(1, _http("/x")))
    relay.inbox.submit(RelayRequest(2, _http("/y")))
    relay.inbox.submit(RelayCancel(1))
    deliveries, note = admission.admit(3, [], lambda ordinal: True)
    assert deliveries == [OperatorRequest(BACKEND, (0, 2), _http("/y"))]  # never delivered
    assert relay.answers == [(1, CANCELLED)]
    assert note["relay"][2] == {"kind": "cancel", "id": [0, 1], "target": BACKEND, "answer": CANCELLED}
    # Its suppressed payload is in the journal, so changing it changes the digest.
    assert note["relay"][0]["payload"] == payload_hash(_http("/x"))


def test_a_later_cancel_is_delivered_and_a_cancel_after_the_response_does_nothing():
    admission, relay, _ids = _admission()
    relay.inbox.submit(RelayRequest(1, _http("/x")))
    relay.inbox.submit(RelayRequest(2, _http("/y")))
    admission.admit(1, [], lambda ordinal: True)
    relay.inbox.submit(RelayCancel(1))
    deliveries, note = admission.admit(2, [], lambda ordinal: True)
    assert deliveries == [OperatorCancel(BACKEND, (0, 1))]
    assert note["relay"] == [{"kind": "cancel", "id": [0, 1], "target": BACKEND}]
    admission.route([(BACKEND, Response((0, 2), {"status": 200, "body": "{}"}))], [])
    relay.inbox.submit(RelayCancel(2))
    deliveries, note = admission.admit(3, [], lambda ordinal: True)
    assert deliveries == []
    assert note["relay"] == [{"kind": "cancel", "id": [0, 2], "target": BACKEND, "answer": None}]
    assert relay.answers == [(2, {"status": 200, "body": "{}"})]


def test_relay_items_wait_until_their_target_is_ready():
    admission, relay, _ids = _admission()
    relay.inbox.submit(RelayRequest(1, _http("/x")))
    assert admission.admit(1, [], lambda ordinal: False) == ([], None)
    deliveries, _note = admission.admit(2, [], lambda ordinal: ordinal == BACKEND)
    assert deliveries == [OperatorRequest(BACKEND, (0, 1), _http("/x"))]
    assert admission.actions[0]["k"] == 2


def test_a_refused_request_is_answered_journaled_and_never_delivered():
    admission, relay, _ids = _admission(refuse=app_refusals.refusal)
    relay.inbox.submit(RelayRequest(1, _http("/api/vehicles/2/parameters?refresh=1")))
    relay.inbox.submit(RelayCancel(1))
    deliveries, note = admission.admit(1, [], lambda ordinal: True)
    assert deliveries == []
    assert relay.answers[0][1]["status"] == app_refusals.STATUS
    assert note["relay"][0]["answer"]["status"] == 501
    assert note["relay"][1]["answer"] is None


def test_no_relay_traffic_adds_no_journal_note():
    admission = Admission(RequestIds(0))
    scripted = [OperatorRequest(BACKEND, (0, 1), _http("/a"))]
    assert admission.admit(1, scripted, lambda ordinal: True) == (scripted, None)
    assert admission.route([(BACKEND, Response((0, 1), {}))], []) == ((BACKEND, Response((0, 1), {})),)


@pytest.mark.parametrize(("method", "target", "refused"), [
    ("GET", "/api/vehicles/1/parameters", True),
    ("GET", "/api/vehicles/1/parameters?force=1", True),
    ("PUT", "/api/vehicles/1/parameters", False),
    ("POST", "/api/vehicles/1/parameters/arm-token", False),
    ("GET", "/api/vehicles/1/params", False),
    ("GET", "/api/diagnostics/export", True),
    ("GET", "/api/logs/download/3", True),
    ("POST", "/api/control/navpy-sim/start", True),
    ("POST", "/api/control/navpy-sim/stop/2", True),
    ("POST", "/api/control/navpy-sim/restart-ready", False),
    ("GET", "/api/settings", False),
])
def test_app_refusals(method, target, refused):
    assert (app_refusals.refusal(_http(target, method)) is not None) is refused


def test_a_payload_that_is_not_http_is_refused():
    assert app_refusals.refusal({"kind": "subscribe"})["status"] == 501
    assert app_refusals.refusal({"kind": "http", "method": "GET"})["status"] == 501


def test_the_pacer_sleeps_until_wall_start_plus_t_k_and_never_when_behind():
    now, slept = [100.0], []
    pacer = RealTimePacer(clock=lambda: now[0], sleep=slept.append)
    pacer(1_000_000)  # anchors: wall 100 is t = 1 s
    now[0] = 100.01
    pacer(1_020_000)
    assert slept == [pytest.approx(0.01)]
    now[0] = 100.5  # slower than 1x: no sleep
    pacer(1_040_000)
    assert len(slept) == 1


def test_a_script_cancel_in_the_step_that_sent_its_request_is_resolved_by_the_coordinator():
    admission, relay, ids = _admission()
    request = OperatorRequest(BACKEND, ids.next(), _http("/x"))
    kept = OperatorRequest(BACKEND, ids.next(), _http("/y"))
    deliveries, note = admission.admit(1, [request, kept, OperatorCancel(BACKEND, request.id)],
                                       lambda ordinal: True)
    assert deliveries == [kept]  # the handler never starts
    assert note == {"script": [{"kind": "request", "id": [0, 1], "target": BACKEND,
                                "payload": payload_hash(_http("/x"))},
                               {"kind": "request", "id": [0, 2], "target": BACKEND,
                                "payload": payload_hash(_http("/y"))},
                               {"kind": "cancel", "id": [0, 1], "target": BACKEND, "answer": CANCELLED}],
                    "relay": []}
    # The script gets its answer in its next inbox, once.
    assert admission.route([], []) == ((BACKEND, Response((0, 1), CANCELLED)),)
    assert admission.route([], []) == ()


def test_a_script_cancel_mixed_with_relay_traffic_and_a_later_script_cancel_is_delivered():
    admission, relay, ids = _admission()
    request = OperatorRequest(BACKEND, ids.next(), _http("/x"))
    relay.inbox.submit(RelayRequest(1, _http("/r")))
    deliveries, note = admission.admit(1, [request, OperatorCancel(BACKEND, request.id)],
                                       lambda ordinal: True)
    assert deliveries == [OperatorRequest(BACKEND, (0, 2), _http("/r"))]
    assert note["script"][1]["answer"] == CANCELLED and note["relay"][0]["id"] == [0, 2]
    assert relay.answers == []  # the script's answer never goes to the relay
    later = OperatorRequest(BACKEND, ids.next(), _http("/z"))
    admission.admit(2, [later], lambda ordinal: True)
    deliveries, note = admission.admit(3, [OperatorCancel(BACKEND, later.id)], lambda ordinal: True)
    assert deliveries == [OperatorCancel(BACKEND, later.id)] and note is None
