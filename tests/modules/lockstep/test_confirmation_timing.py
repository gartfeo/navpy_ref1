"""The confirmation timing from the journal (``lockstep_slice.md``, Confirmation timing).

The real confirmation workflow (``run_confirmation_workflow``) runs on the
operator's baton through the lockstep ports, against a scripted backend
that delivers events at chosen ticks and answers the approval after the
route's resend schedule on the tick grid. Its journal records are built as
the coordinator builds them (the operator's observations, the response
sends' NAVLINK counts), and the gate reads them.

Grids: the recorded serial-step frames (``test_grid_check_recording``), and
a uniform 10 ms grid on which the 15 s deadline falls exactly on a tick.
"""

from __future__ import annotations

import json

import pytest

from navpy.sim.lockstep.coordinator_types import OperatorInbox, OperatorRequest
from navpy.sim.lockstep.operator_baton import DELIVERY_EVENT, OperatorBaton
from navpy.sim.lockstep.operator_ports import BatonClock, LockstepApi, LockstepEventStream
from navpy.sim.lockstep.part_wire import Event, Response
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts.eval_gcs_demo_confirmation import run_confirmation_workflow
from scripts.eval_gcs_demo_confirmation_timing import (
    RESPONSE_DIRECTION,
    RESPONSE_ID,
    confirmation_ticks,
    confirmation_timing_errors,
)
from scripts.lockstep_wsl_sitl import MAX_FRAME_GAP_US
from tests.modules.lockstep.test_grid_check_recording import recorded_frames

BACKEND = 2
UID = "1167813307:17"
DELAY_S = 15.0
REQUEST = json.dumps({"type": "task_confirm_request", "sys_id": 1, "task_id": 1, "round_uid": UID})
TELEMETRY = json.dumps({"type": "telemetry", "sys_id": 1})
#: The backend's response to vehicle 1, keyed as the journal keys it.
RESPONSE_KEY = f"{RESPONSE_ID}:255.0>1"
#: Vehicle 1's SITL ports as the journal's grid names them; the response
#: sends are the router's copies admitted to its serial1.
SITL_PORTS = [["lockstep:serial0.1", "sitl1.serial1"]]
RESPONSE_LINK = SITL_PORTS[0][1]


class ScriptedBackend:
    """Subscribe acks next tick; scripted deliveries; the approval answered after
    three sends 0.25 s apart, each at the first tick at or after its deadline."""

    def __init__(self, times: dict[int, int], deliveries: dict[int, list[str]]) -> None:
        self._times = times
        self._deliveries = deliveries
        self._responses: dict[int, list[Response]] = {}
        self.sends: dict[int, int] = {}
        self._subscription = None

    def _due_after(self, k: int, seconds: float) -> int:
        deadline = self._times[k] / 1e6 + seconds
        return next(j for j in sorted(self._times) if j > k and self._times[j] / 1e6 >= deadline)

    def inbox(self, k: int) -> OperatorInbox:
        events = ()
        if self._subscription is not None:
            events = tuple((BACKEND, Event(DELIVERY_EVENT, {"subscription": list(self._subscription),
                                                            "data": data}))
                           for data in self._deliveries.get(k, ()))
        responses = tuple((BACKEND, r) for r in self._responses.pop(k, ()))
        return OperatorInbox(k, self._times[k], responses, events)

    def handle(self, k: int, requests: tuple[OperatorRequest, ...]) -> None:
        for request in requests:
            payload = request.payload
            if payload.get("kind") == "subscribe":
                self._subscription = request.id
                self._answer(k + 1, request.id, {"subscribed": list(request.id)})
            elif payload.get("target") == "/api/control/task_confirm":
                first = k + 1
                second = self._due_after(first, 0.25)
                third = self._due_after(second, 0.25)
                for send in (first, second, third):
                    self.sends[send] = self.sends.get(send, 0) + 1
                self._answer(third, request.id, {"status": 200, "body": json.dumps({"status": "approved"})})
            else:
                self._answer(k + 1, request.id, {})

    def _answer(self, k: int, request_id, payload) -> None:
        self._responses.setdefault(k, []).append(Response(request_id, payload))


def run(frames: list[tuple[int, int]], deliveries: dict[int, list[str]], max_gap_us: int):
    times = dict(frames)
    backend = ScriptedBackend(times, deliveries)
    approvals: list[dict] = []

    def script(operator: OperatorBaton) -> None:
        approvals.extend(run_confirmation_workflow(
            event_stream=LockstepEventStream(operator, BACKEND), api=LockstepApi(operator, BACKEND),
            expected_sys_ids={1}, trigger=lambda: None, stop_when=lambda: bool(approvals),
            requested_delay_s=DELAY_S, timeout_s=60.0, clock=BatonClock(operator),
            on_approval_accepted=approvals.append))

    operator = OperatorBaton(RequestIds(0), script)
    records: list[dict] = [{"grid": {"max_frame_gap_us": max_gap_us, "sitl_ports": SITL_PORTS}}]
    for k, t_us in frames:
        backend.handle(k, operator.run_tick(backend.inbox(k)))
        record: dict = {"k": k, "t_us": t_us}
        observed = operator.observations()
        if observed:
            record["operator"] = observed
        if k in backend.sends:
            record["navlink"] = {RESPONSE_LINK: {RESPONSE_DIRECTION: {RESPONSE_KEY: backend.sends[k]}}}
        records.append(record)
        if operator.done:
            break
    assert operator.done, "the workflow never approved"
    return records, approvals[0]


def uniform_grid(count: int, period_us: int = 10_000) -> list[tuple[int, int]]:
    return [(k, 1_000_000 + (k - 1) * period_us) for k in range(1, count + 1)]


def _timing(records, approval, k_request):
    lines = [(k_request, "CONFIRMING: P1")]
    return confirmation_ticks(records, lines, 1, 1), confirmation_timing_errors(records, lines, approval, 1, 1)


def test_the_relations_hold_on_recorded_firmware_frames():
    frames = recorded_frames()
    records, approval = run(frames, {100: [REQUEST]}, MAX_FRAME_GAP_US)
    ticks, errors = _timing(records, approval, 100)
    assert errors == []
    t = dict(frames)
    k = ticks.k
    assert k["k2"] == 100 and k["k5"] == k["k4"] + 1 and len(ticks.sends) == 3
    assert 15_000_000 <= t[k["k4"]] - t[k["k3"]] <= 15_100_000 + 2 * MAX_FRAME_GAP_US
    assert approval["actual_delay_s"] == pytest.approx((t[k["k6"]] - t[k["k3"]]) / 1e6, abs=1e-9)


def test_a_deadline_exactly_on_a_tick():
    frames = uniform_grid(1800)
    records, approval = run(frames, {50: [REQUEST]}, 10_000)
    ticks, errors = _timing(records, approval, 50)
    assert errors == []
    t = dict(frames)
    waited = t[ticks.k["k4"]] - t[ticks.k["k3"]]
    # t(k3) + 15 s is a tick; the approval is never before it, and within the bound.
    assert (t[ticks.k["k3"]] + 15_000_000) in t.values()
    assert 15_000_000 <= waited <= 15_100_000 + 2 * 10_000


def test_delivery_while_the_loop_sleeps_is_consumed_after_the_sleep():
    frames = recorded_frames()
    outcomes = {}
    for delivery in range(100, 106):  # one loop cycle: a receive wait and a sleep
        records, approval = run(frames, {delivery: [REQUEST]}, MAX_FRAME_GAP_US)
        ticks, errors = _timing(records, approval, delivery)
        assert errors == [], (delivery, errors)
        outcomes[delivery] = ticks.k["k3"] - ticks.k["k2"]
    assert 0 in outcomes.values()  # delivered while the receive waits: consumed at once
    assert any(lag > 0 for lag in outcomes.values())  # delivered during the sleep: after it


def test_delivery_behind_queued_telemetry_is_consumed_after_them():
    frames = recorded_frames()
    records, approval = run(frames, {100: [TELEMETRY] * 4 + [REQUEST]}, MAX_FRAME_GAP_US)
    ticks, errors = _timing(records, approval, 100)
    assert errors == []
    received = [kind for r in records for _k, kind in r.get("operator", {}).get("received", ())]
    assert received[:5] == ["telemetry"] * 4 + ["task_confirm_request"]
    assert ticks.k["k2"] == 100 and ticks.k["k3"] > ticks.k["k2"]  # one loop pass per queued event


def _shift(records, key: str, k_from: int, k_to: int) -> list[dict]:
    """Move one operator observation (or the NAVLINK sends) from tick k_from to k_to."""
    out = json.loads(json.dumps(records))
    by_k = {r.get("k"): r for r in out}
    if key == "navlink":
        by_k[k_to]["navlink"] = by_k[k_from].pop("navlink")
    else:
        by_k.setdefault(k_to, {}).setdefault("operator", {})[key] = by_k[k_from]["operator"].pop(key)
    return out


def test_failing_evidence():
    frames = recorded_frames()
    records, approval = run(frames, {100: [REQUEST]}, MAX_FRAME_GAP_US)
    ticks, _ = _timing(records, approval, 100)
    k = ticks.k
    lines = [(100, "CONFIRMING: P1")]
    early = _shift(records, "sent", k["k4"], k["k4"] - 2)  # approved before 15 s
    assert any("t(k4) - t(k3)" in e for e in confirmation_timing_errors(early, lines, approval, 1, 1))
    late = _shift(records, "navlink", k["k5"], k["k5"] + 1)  # first send not at k4 + 1
    assert any("not k4 + 1" in e for e in confirmation_timing_errors(late, lines, approval, 1, 1))
    audited = {**approval, "actual_delay_s": approval["actual_delay_s"] + 0.02}
    assert any("actual_delay_s" in e for e in confirmation_timing_errors(records, lines, audited, 1, 1))
    after = [(k["k3"] + 1, "CONFIRMING: P1")]  # the companion state after the operator saw it
    assert any("k1 <= k2 <= k3" in e for e in confirmation_timing_errors(records, after, approval, 1, 1))
    assert confirmation_timing_errors(records, [], approval, 1, 1) == [
        "confirmation timing: the journal holds no k1"]


def test_the_bound_is_inclusive_at_exactly_the_delay_and_exclusive_past_the_upper_bound():
    frames = uniform_grid(1800)
    records, approval = run(frames, {50: [REQUEST]}, 10_000)
    ticks, _ = _timing(records, approval, 50)
    k3 = ticks.k["k3"]
    lines = [(50, "CONFIRMING: P1")]
    exact = 1500  # 15.00 s after k3 on the 10 ms grid
    upper = 1500 + 10 + 2  # 15.10 s + 2 * 10 ms
    for offset, ok in ((exact, True), (exact - 1, False), (upper, True), (upper + 1, False)):
        moved = _shift(records, "sent", ticks.k["k4"], k3 + offset)
        errors = [e for e in confirmation_timing_errors(moved, lines, approval, 1, 1) if "t(k4)" in e]
        assert (errors == []) is ok, (offset, errors)


def _with_sends(records, sends: list[int]) -> list[dict]:
    out = json.loads(json.dumps(records))
    for record in out:
        record.pop("navlink", None)
    by_k = {r.get("k"): r for r in out}
    for k in sends:
        counts = by_k[k].setdefault("navlink", {RESPONSE_LINK: {RESPONSE_DIRECTION: {RESPONSE_KEY: 0}}})
        counts[RESPONSE_LINK][RESPONSE_DIRECTION][RESPONSE_KEY] += 1
    return out


def test_each_resend_interval_is_checked_not_only_the_span():
    """Codex 9g P2-4: sends [k5, k5, k6] fit the first-to-last span (2 gaps) but
    two leave in one tick; each consecutive interval must be its own gap."""
    frames = recorded_frames()
    records, approval = run(frames, {100: [REQUEST]}, MAX_FRAME_GAP_US)
    ticks, errors = _timing(records, approval, 100)
    assert errors == []
    k5, k6 = ticks.k["k5"], ticks.k["k6"]
    t = dict(frames)
    assert 500_000 <= t[k6] - t[k5] <= 2 * (250_000 + MAX_FRAME_GAP_US)  # the span alone passes
    lines = [(100, "CONFIRMING: P1")]
    doubled = confirmation_timing_errors(_with_sends(records, [k5, k5, k6]), lines, approval, 1, 1)
    assert doubled == [f"confirmation timing: resend interval t({k5}) - t({k5}) = 0 us outside "
                       f"[250000, {250_000 + MAX_FRAME_GAP_US}]",
                       f"confirmation timing: resend interval t({k6}) - t({k5}) = {t[k6] - t[k5]} us "
                       f"outside [250000, {250_000 + MAX_FRAME_GAP_US}]"]
    # The second send two ticks late: its interval is too long and the next too short,
    # though the span is unchanged.
    middle = ticks.sends[1] + 2
    errors = confirmation_timing_errors(_with_sends(records, [k5, middle, k6]), lines, approval, 1, 1)
    assert [e.split(" = ")[0] for e in errors] == [
        f"confirmation timing: resend interval t({middle}) - t({k5})",
        f"confirmation timing: resend interval t({k6}) - t({middle})"]
