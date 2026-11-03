"""Step 11: the app relay forwards only (``scripts/lockstep_app_relay.py``).

The relay serves a real socket; the test plays the coordinator: it takes
what the relay submitted and answers by ticket, in any order.
"""

from __future__ import annotations

import http.client
import json
import socket
import time

import pytest
from websockets.sync.client import connect

from navpy.sim.lockstep.coordinator_admission import RelayCancel, RelayRequest
from scripts.lockstep_app_relay import AppRelay, TelemetryState, http_reply

WAIT_S = 10.0


@pytest.fixture
def relay():
    app = AppRelay(target=4)
    app.start(0)
    yield app
    app.stop()


def _taken(relay: AppRelay, count: int) -> list:
    items, end = [], time.monotonic() + WAIT_S
    while len(items) < count and time.monotonic() < end:
        items.extend(relay.take())
        time.sleep(0.01)
    assert len(items) == count, items
    return items


def _until(condition) -> None:
    end = time.monotonic() + WAIT_S
    while not condition():
        assert time.monotonic() < end
        time.sleep(0.01)


def _send(relay: AppRelay, method: str, path: str, body: bytes | None = None,
          headers: dict | None = None) -> http.client.HTTPConnection:
    connection = http.client.HTTPConnection("127.0.0.1", relay.port, timeout=WAIT_S)
    connection.request(method, path, body=body, headers=headers or {})
    return connection


def test_requests_carry_method_target_json_and_deadline_and_answer_out_of_order(relay):
    first = _send(relay, "PUT", "/api/vehicles/1/params?x=1", json.dumps({"a": 1}).encode(),
                  {"Content-Type": "application/json", "X-NavPy-Deadline-S": "20",
                   "X-GCS-Client-ID": "page"})
    second = _send(relay, "GET", "/api/settings")
    one, two = _taken(relay, 2)
    assert one == RelayRequest(one.ticket, {"kind": "http", "method": "PUT",
                                            "target": "/api/vehicles/1/params?x=1",
                                            "json": {"a": 1}, "deadline_s": 20.0})
    assert two.payload == {"kind": "http", "method": "GET", "target": "/api/settings"}
    relay.answer(two.ticket, {"status": 200, "body": '{"b":2}'})  # the later one finishes first
    reply = second.getresponse()
    assert (reply.status, json.loads(reply.read())) == (200, {"b": 2})
    relay.answer(one.ticket, {"deadline": True})
    reply = first.getresponse()
    assert reply.status == 504


def test_the_target_is_forwarded_as_sent_for_the_backend_to_decode_once(relay):
    pending = _send(relay, "GET", "/api/vehicles/1/%2570arameters?a=%20")
    (item,) = _taken(relay, 1)
    assert item.payload["target"] == "/api/vehicles/1/%2570arameters?a=%20"
    relay.answer(item.ticket, {"status": 404, "body": "{}"})
    assert pending.getresponse().status == 404


def test_a_closed_page_sends_no_cancel_and_its_late_answer_is_logged(relay):
    connection = _send(relay, "GET", "/api/vehicles/1/mission")
    (item,) = _taken(relay, 1)
    connection.sock.shutdown(socket.SHUT_RDWR)
    connection.close()
    _until(lambda: ("disconnected", item.ticket) in relay.notes)
    relay.answer(item.ticket, {"status": 200, "body": "{}"})
    _until(lambda: ("unreceived", item.ticket) in relay.notes)
    assert relay.take() == ()  # no cancel: a disconnect is not an operator action


def test_only_an_explicit_cancel_of_a_pending_request_is_a_cancel(relay):
    pending = _send(relay, "POST", "/api/control/command", b"{}", {"X-GCS-Request-ID": "r-1"})
    (item,) = _taken(relay, 1)
    cancel = _send(relay, "POST", "/__relay/cancel/r-1")
    assert cancel.getresponse().status == 202
    assert _taken(relay, 1) == [RelayCancel(item.ticket)]
    relay.answer(item.ticket, {"cancelled": True})
    assert pending.getresponse().status == 499
    unknown = _send(relay, "POST", "/__relay/cancel/r-1")  # answered: nothing to cancel
    assert unknown.getresponse().status == 404
    assert relay.take() == ()


def test_a_body_that_is_not_json_never_enters_the_run(relay):
    reply = _send(relay, "POST", "/api/settings", b"not json").getresponse()
    assert reply.status == 400
    reply = _send(relay, "GET", "/api/settings", headers={"X-NavPy-Deadline-S": "nan"}).getresponse()
    assert reply.status == 400
    for body in (b'{"x": 1e400}', b'{"a": [NaN]}', b'{"a": {"b": -Infinity}}'):  # the journal has no such number
        assert _send(relay, "POST", "/api/settings", body).getresponse().status == 400
    assert relay.take() == ()


def test_stop_answers_open_requests_and_refuses_new_ones():
    app = AppRelay(target=4)
    app.start(0)
    pending = _send(app, "GET", "/api/settings")
    _taken(app, 1)
    app.stop()
    assert pending.getresponse().status == 503


def test_stop_ends_a_request_whose_body_never_came():
    app = AppRelay(target=4)
    app.start(0)
    stalled = socket.create_connection(("127.0.0.1", app.port), timeout=WAIT_S)
    stalled.sendall(b"POST /api/settings HTTP/1.1\r\nHost: x\r\nContent-Length: 100\r\n\r\n{")
    time.sleep(0.2)  # let the relay start reading the body
    started = time.monotonic()
    app.stop()  # raises if its thread outlives the grace
    assert time.monotonic() - started < WAIT_S
    assert app.take() == ()
    stalled.close()


def test_telemetry_goes_to_every_page_and_a_new_page_gets_the_merged_state(relay):
    relay.broadcast(json.dumps({"type": "telemetry", "vehicles": [{"sys_id": 1, "alt": 5, "mode": "A"}],
                                "active_ids": [1, 2], "ts": 1.0}))
    relay.broadcast(json.dumps({"type": "telemetry", "vehicles": [{"sys_id": 2, "alt": 7}],
                                "active_ids": [1, 2], "ts": 2.0}))
    _until(lambda: relay._telemetry.snapshot() is not None and '"ts": 2.0' in relay._telemetry.snapshot())
    with connect(f"ws://127.0.0.1:{relay.port}/ws/telemetry", open_timeout=WAIT_S) as page:
        first = json.loads(page.recv(timeout=WAIT_S))
        assert first["ts"] == 2.0
        assert sorted(v["sys_id"] for v in first["vehicles"]) == [1, 2]
        page.send("ping")
        assert page.recv(timeout=WAIT_S) == "pong"
        page.send(json.dumps({"type": "manual_control", "sys_id": 1}))
        relay.broadcast(json.dumps({"type": "task_event", "x": 1}))
        assert json.loads(page.recv(timeout=WAIT_S)) == {"type": "task_event", "x": 1}
    _until(lambda: ("ws_dropped", "manual_control") in relay.notes)
    assert relay.take() == ()  # watching and stick input never enter the run


def test_telemetry_state_merges_like_the_page():
    state = TelemetryState()
    assert state.snapshot() is None
    state.observe({"type": "telemetry", "vehicles": [{"sys_id": 1, "a": 1, "b": 1}], "ts": 1})
    state.observe({"type": "telemetry", "vehicles": [{"sys_id": 1, "b": 2}], "active_ids": [1], "ts": 2})
    state.observe({"type": "task_event"})
    assert json.loads(state.snapshot())["vehicles"] == [{"sys_id": 1, "a": 1, "b": 2}]
    state.observe({"type": "telemetry", "vehicles": [], "active_ids": [], "ts": 3})
    assert json.loads(state.snapshot())["vehicles"] == []


@pytest.mark.parametrize(("answer", "status"), [
    ({"status": 404, "body": "{}"}, 404), ({"cancelled": True}, 499), ({"deadline": True}, 504),
    ({"subscribed": [0, 1]}, 502),
])
def test_http_reply(answer, status):
    assert http_reply(answer)[0] == status
