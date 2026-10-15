"""The operator evidence the fleet's approval replay reads (``lockstep_fleet.md``,
"Operator approvals" and "Operator polling schedule").

The real confirmation workflow runs on the operator's baton through the
lockstep ports against the scripted backend of ``test_confirmation_timing``
(deliveries at chosen ticks, the approval answered after the route's resend
schedule), on the recorded serial-step frames. With ``evidence`` the
operator's journal carries each pass's receive, due check and sleep, every
delivery apart from its consumption, and the confirmation identity on
``sent`` and ``received``; without it, the slice's records keep their shape.
"""

from __future__ import annotations

from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.operator_journal import confirmation_identity
from navpy.sim.lockstep.operator_ports import (
    BatonClock,
    LockstepApi,
    LockstepEventStream,
    due_check_hook,
    polling_start_hook,
)
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts.eval_gcs_demo_confirmation import CONFIRMATION_POLL_S, run_confirmation_workflow
from tests.modules.lockstep.test_confirmation_timing import (
    BACKEND,
    DELAY_S,
    REQUEST,
    TELEMETRY,
    UID,
    ScriptedBackend,
)
from tests.modules.lockstep.test_grid_check_recording import recorded_frames

CONFIRM_ROUTE = "/api/control/task_confirm"
EVIDENCE_KEYS = {"delivered", "receive", "polling", "due_check", "sleep"}


def _run(evidence: bool, deliveries: dict[int, list[str]]):
    frames = recorded_frames()
    backend = ScriptedBackend(dict(frames), deliveries)
    approvals: list[dict] = []

    def script(operator: OperatorBaton) -> None:
        approvals.extend(run_confirmation_workflow(
            event_stream=LockstepEventStream(operator, BACKEND), api=LockstepApi(operator, BACKEND),
            expected_sys_ids={1}, trigger=lambda: None, stop_when=lambda: bool(approvals),
            requested_delay_s=DELAY_S, timeout_s=60.0, clock=BatonClock(operator),
            on_approval_accepted=approvals.append, on_polling_start=polling_start_hook(operator),
            on_due_check=due_check_hook(operator)))

    operator = OperatorBaton(RequestIds(0), script, evidence=evidence)
    observed: dict[int, dict] = {}
    for k, _t_us in frames:
        backend.handle(k, operator.run_tick(backend.inbox(k)))
        tick = operator.observations()
        if tick:
            observed[k] = tick
        if operator.done:
            break
    assert operator.done, "the workflow never approved"
    return observed, approvals[0], dict(frames)


def _all(observed: dict[int, dict], key: str) -> list[tuple[int, list]]:
    return [(k, entry) for k, tick in sorted(observed.items()) for entry in tick.get(key, ())]


def test_each_pass_journals_its_receive_due_check_and_sleep():
    observed, approval, t = _run(True, {100: [TELEMETRY, REQUEST]})
    subscription = _all(observed, "receipts")[0][1]
    # Delivery and consumption are journaled apart: both queued at 100, each
    # consumed by its own pass's receive.
    assert _all(observed, "delivered") == [(100, [*subscription, "telemetry", 1, None, None]),
                                           (100, [*subscription, "task_confirm_request", 1, 1, UID])]
    received = _all(observed, "received")
    assert [entry for _k, entry in received] == [
        [100, "telemetry", 1, None, None, *subscription],
        [100, "task_confirm_request", 1, 1, UID, *subscription]]
    k3 = received[1][0]
    assert k3 > received[0][0] >= 100
    # Every receive returns at its record's tick: with an event, or at its timeout.
    receives = _all(observed, "receive")
    assert all(entry[1] == k and entry[3:] == subscription for k, entry in receives)
    receives = [(k, entry[:3]) for k, entry in receives]
    # Polling starts once, at the first receive's start, with the deadline the
    # loop set on entry (its subscribe's tick) plus its 60 s timeout.
    (polling,) = _all(observed, "polling")
    entered = next(k for k, entry in _all(observed, "sent") if entry[2] == "subscribe")
    assert polling == (receives[0][1][0], [receives[0][1][0], t[entered] / 1e6 + 60.0])
    first, second = [entry for _k, entry in receives if entry[2] == "event"]
    assert first[0] < first[1] == 100  # a waiting receive returns at the delivery tick
    assert second == [k3, k3, "event"]  # one that starts with an event queued returns at once
    assert all(t[entry[1]] >= t[entry[0]] + CONFIRMATION_POLL_S * 1e6
               for _k, entry in receives if entry[2] == "timeout")
    # The sleep wakes at the first tick at or after its 0.05 s deadline.
    sleeps = _all(observed, "sleep")
    for k, (start, wake) in sleeps:
        assert wake == k and t[wake] >= t[start] + CONFIRMATION_POLL_S * 1e6 > t[wake - 1]
    # The approval goes out in the first pass whose due check is 15 s past k3.
    checks = _all(observed, "due_check")
    assert all(entry[0] == k and entry[1] == t[k] for k, entry in checks)
    due = [(k, entry) for k, entry in checks if entry[2]]
    k4 = due[0][0]
    assert due[0][1][2] == [[1, 1, UID]]
    assert t[k4] - t[k3] >= DELAY_S * 1e6
    previous = max(k for k, _entry in checks if k < k4)
    assert t[previous] - t[k3] < DELAY_S * 1e6
    (post,) = [(k, entry) for k, entry in _all(observed, "sent") if entry[3] == CONFIRM_ROUTE]
    assert post[0] == k4 and post[1][2:] == ["POST", CONFIRM_ROUTE, 1, 1, UID]
    (answer,) = [k for k, entry in _all(observed, "answered") if entry[:2] == post[1][:2]]
    assert answer > k4 and approval["task_id"] == 1


def test_without_evidence_the_slice_records_keep_their_shape():
    observed, _approval, _t = _run(False, {100: [TELEMETRY, REQUEST]})
    assert not EVIDENCE_KEYS & {key for tick in observed.values() for key in tick}
    assert {len(entry) for _k, entry in _all(observed, "received")} == {2}
    assert {len(entry) for _k, entry in _all(observed, "sent")} == {4}


def test_the_confirmation_identity_reads_approvals_and_events():
    body = {"sys_id": 2, "task_id": 7, "round_uid": UID, "is_confirmed": True}
    assert confirmation_identity(body) == [2, 7, UID]
    assert confirmation_identity(REQUEST) == [1, 1, UID]
    assert confirmation_identity("not json") == [None, None, None]
    assert confirmation_identity(None) == [None, None, None]
