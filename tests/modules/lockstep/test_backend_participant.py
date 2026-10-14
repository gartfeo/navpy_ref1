"""The backend participant's contracts, each through a real participant run.

``lockstep_slice.md``, "Backend participant": subscription timing and the
tick input order, cancels of one request among two, and the strict shutdown
in drain while work is pending (a parameter read, a mission upload whose
replies are withheld, a job waiting for the mission lock behind a probe in
flight). Each shutdown case asserts that nothing is sent or broadcast for
the cancelled work after its cancel, and that the backend still ends
flushed.
"""

from __future__ import annotations

import json

from gcs.backend.lockstep.launcher_port import generation_pid
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.operator_ports import BatonClock, LockstepEventStream
from scripts.eval_gcs_demo_lifecycle import (
    companion_ids,
    connect_vehicles,
    start_companions,
    stop_companions,
)
from scripts.lockstep_fleet_actor import stack_context as fleet_context
from scripts.lockstep_slice_actor import BACKEND, NAVPY, SLICE
from tests.modules.lockstep.backend_doubles import BOUND_VALUES, READY_PAYLOAD, BackendRun, build
from tests.modules.lockstep.lockstep_autopilot import SYS_ID, ReferenceAutopilot

TICK_S = 0.02
EMPTY_UPLOAD = {"assignments": [{"sys_id": SYS_ID, "zone_index": 0, "waypoints": [],
                                 "altitude_m": 100.0}]}


def _http(method: str, target: str, body: dict | None = None) -> dict:
    payload = {"kind": "http", "method": method, "target": target, "deadline_s": 30.0}
    if body is not None:
        payload["json"] = body
    return payload


def _connected(operator: OperatorBaton, scenario) -> None:
    operator.wait_ready(BACKEND)
    connect_vehicles(fleet_context(operator, scenario, SLICE))


def _run(tmp_path, script, **options) -> BackendRun:
    run = build(tmp_path, script=lambda scenario: lambda operator: script(operator, scenario),
                **options)
    run.run()
    return run


def _ended_flushed(run: BackendRun) -> None:
    last = run.dones[-1]
    assert last.ended is True and last.faults == ()
    assert all(done.faults == () for done in run.dones)


def _upload_events(events: list[tuple[int, dict]]) -> list[tuple[int, dict]]:
    return [(k, json.loads(e["data"])) for k, e in events
            if json.loads(e["data"])["type"] == "upload_progress"]


def test_delivery_starts_exactly_at_the_subscription_and_a_same_tick_trigger_is_seen(tmp_path):
    ids = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        clock = BatonClock(operator)
        before = operator.expect(ids.setdefault("before", operator.next_id()))
        operator.send(BACKEND, _http("POST", "/api/vehicles/upload", EMPTY_UPLOAD), ids["before"])
        clock.sleep(TICK_S)  # the subscribe goes one tick later
        stream = LockstepEventStream(operator, BACKEND)
        stream.start()
        ids["subscription"] = stream.subscription
        trigger = operator.expect(ids.setdefault("trigger", operator.next_id()))  # after it, same tick
        operator.send(BACKEND, _http("POST", "/api/vehicles/upload", EMPTY_UPLOAD), ids["trigger"])
        stream.wait_subscribed()
        for pending in (before, trigger):
            assert operator.await_answer(pending)["status"] == 200
        stream.__exit__(None, None, None)

    run = _run(tmp_path, script)
    subscribed_at, _answer = run.response(ids["subscription"])
    journal = _upload_events(run.events("broadcast"))
    delivered = _upload_events(run.events("ws"))
    starts = [k for k, data in journal if data.get("progress") == 0.1]
    assert len(starts) == 2 and starts[0] < subscribed_at == starts[1]
    # The earlier request's start broadcast ran before the installation: journaled, not delivered.
    assert [k for k, data in delivered if data.get("progress") == 0.1] == [subscribed_at]
    # Both completions came after the installation: both delivered.
    completes = [k for k, data in journal if data.get("stage") == "complete"]
    assert len(completes) == 2 and [k for k, d in delivered if d.get("stage") == "complete"] == completes
    first = run.events("ws")[0]
    assert first[0] == subscribed_at and json.loads(first[1]["data"])["type"] == "telemetry"  # snapshot first
    _ended_flushed(run)


def test_one_of_two_simultaneous_requests_is_cancelled_and_the_other_stands(tmp_path):
    ids = {}

    def script(operator: OperatorBaton, scenario) -> dict:
        _connected(operator, scenario)
        slow = operator.expect(ids.setdefault("slow", operator.next_id()))
        operator.send(BACKEND, _http("GET", f"/api/vehicles/{SYS_ID}/params/named?names=NOPE_A,NOPE_B"),
                      ids["slow"])
        fast = operator.expect(ids.setdefault("fast", operator.next_id()))
        operator.send(BACKEND, _http("GET", f"/api/vehicles/{SYS_ID}/params/named?names=TRIM_THROTTLE"),
                      ids["fast"])
        BatonClock(operator).sleep(0.1)  # NOPE_A's read is parked by now
        ids["cancel_tick"] = operator.tick
        operator.cancel(BACKEND, ids["slow"])
        return {"slow": operator.await_answer(slow), "fast": operator.await_answer(fast)}

    run = _run(tmp_path, script)
    answers = run.operator.result
    assert answers["slow"] == {"cancelled": True}
    assert answers["fast"]["status"] == 200
    assert json.loads(answers["fast"]["body"])["params"] == [
        {"name": "TRIM_THROTTLE", "value": BOUND_VALUES["TRIM_THROTTLE"]}]
    reads = [m.param_id for _k, kind, m in run.sent() if kind == "PARAM_REQUEST_READ"]
    assert reads.count("NOPE_A") == 1 and "NOPE_B" not in reads  # nothing after the cancel
    _ended_flushed(run)


def _shutdown_with_pending(tmp_path, request: dict, settle_s: float, **options):
    ids = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        ids["pending"] = operator.send(BACKEND, request)
        BatonClock(operator).sleep(settle_s)  # the job is parked; then the operator returns

    run = _run(tmp_path, script, **options)
    drain = next(done.k for done in run.dones if done.k > run.coordinator.journal.records[-1]["final_tick"])
    cancelled_at, answer = run.response(ids["pending"])
    assert answer == {"cancelled": True} and cancelled_at == drain  # the first drain tick
    _ended_flushed(run)
    return run, drain


def test_shutdown_cancels_a_pending_parameter_read(tmp_path):
    run, drain = _shutdown_with_pending(
        tmp_path, _http("GET", f"/api/vehicles/{SYS_ID}/params/named?names=NOPE_A,NOPE_B"), 0.1)
    reads = [m.param_id for _k, kind, m in run.sent() if kind == "PARAM_REQUEST_READ"]
    assert reads.count("NOPE_A") == 1 and "NOPE_B" not in reads


def test_shutdown_cancels_a_mission_upload_whose_replies_are_withheld(tmp_path):
    """The cancel lands just after the job parks for its next MISSION_REQUEST."""
    scenario_upload = {"assignments": [{
        "sys_id": SYS_ID, "zone_index": 0, "altitude_m": 100.0,
        "waypoints": [{"lat": 40.31, "lon": 44.45}, {"lat": 40.30, "lon": 44.45}],
    }]}
    run, drain = _shutdown_with_pending(
        tmp_path, _http("POST", "/api/vehicles/upload", scenario_upload), 0.3,
        autopilot=ReferenceAutopilot(parameters=BOUND_VALUES, answer_limits={"MISSION_ITEM_INT": 2}))
    items = [(k, m.seq) for k, kind, m in run.sent() if kind == "MISSION_ITEM_INT"]
    # Two items answered; the third's request is withheld, so the job parks.
    assert [seq for _k, seq in items] == [0, 1, 2] and all(k < drain for k, _seq in items)
    assert not [k for k, kind, _m in run.sent() if kind.startswith("MISSION") and k >= drain]
    # No progress submission after the cancel: the last upload broadcast precedes the drain.
    progress = _upload_events(run.events("broadcast"))
    assert progress and all(k < drain for k, _data in progress)
    assert not any(data.get("stage") in ("complete", "failed") for _k, data in progress)


def test_shutdown_cancels_a_job_waiting_for_the_mission_lock_behind_a_probe_in_flight(tmp_path):
    """The connect-time probe holds the mission lock waiting for MISSION_COUNT
    (withheld); a mission download waits for the lock. The shutdown cancels both."""
    run, drain = _shutdown_with_pending(
        tmp_path, _http("GET", f"/api/vehicles/{SYS_ID}/mission"), 0.3,
        autopilot=ReferenceAutopilot(parameters=BOUND_VALUES, answer_limits={"MISSION_REQUEST_LIST": 0}))
    lists = [k for k, kind, _m in run.sent() if kind == "MISSION_REQUEST_LIST"]
    assert lists and all(k < drain for k in lists)  # only the probe's; nothing after the cancel
    assert len(lists) == 1  # the download never got the lock: no request of its own


def test_a_full_lifespan_with_a_companion_started_and_stopped_through_the_launcher_port(tmp_path):
    """Start-all spawns the companion through a spawn request; status reads the
    lifecycle records (ordinal, generation); stop-all is acked by the
    participant stop; the backend's own shutdown then ends it flushed."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        context = fleet_context(operator, scenario, SLICE)
        start_companions(context, 10.0)  # waits for status ``running``
        seen["running"] = companion_ids(context)
        seen["status"] = context.api.get_json("/api/control/navpy-sim/status")
        stop_companions(context)  # every outcome must be STOP_ACKED
        seen["after"] = context.api.get_json("/api/control/navpy-sim/status")

    run = _run(tmp_path, script, companions=True)
    assert seen["running"] == {SYS_ID}
    [status] = seen["status"]["instances"]
    assert (status["pid"], status["ordinal"], status["generation"]) == (generation_pid(NAVPY, 1), NAVPY, 1)
    assert status["connection"] == "lockstep:serial0.1" and status["running"] is True
    assert seen["after"] == {"instances": []}
    spawns = [s for done in run.dones for s in done.spawn_requests]
    stops = [s for done in run.dones for s in done.stop_requests]
    assert [(s.role, s.ordinal) for s in spawns] == [("navpy", NAVPY)]
    assert [(s.ordinal, s.generation) for s in stops] == [(NAVPY, 1)]
    _ended_flushed(run)


def test_the_esp32_simulator_runs_in_process_through_its_port(tmp_path):
    """Start, health and status answer from the simulator's own state with no
    socket and no port; stop ends it; the shutdown stops a running one."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        api = fleet_context(operator, scenario, SLICE).api
        seen["before"] = api.get_json("/api/control/launch/esp32-sim/status")
        seen["start"] = api.post_json("/api/control/launch/esp32-sim/start", {})
        seen["probe"] = api.get_json("/api/control/launch/esp32-status")
        seen["stop"] = api.post_json("/api/control/launch/esp32-sim/stop", {})
        seen["after"] = api.get_json("/api/control/launch/esp32-status")
        api.post_json("/api/control/launch/esp32-sim/start", {})  # left for the shutdown

    run = _run(tmp_path, script)
    assert seen["before"] == {"running": False}
    assert seen["start"] == (200, {"status": "started", "port": None})
    probe = seen["probe"]
    assert probe["reachable"] is True and probe["simulator_running"] is True
    assert probe["status"]["channels"] == [1, 2, 3, 4, 5, 6]
    assert seen["stop"] == (200, {"status": "stopped"})
    assert seen["after"]["simulator_running"] is False
    from gcs.backend.routes import control
    assert control._esp32_sim is None
    _ended_flushed(run)


def test_cancelling_the_scan_that_opened_the_bus_leaves_its_reader_and_the_vehicle_working(tmp_path):
    """The bus reader belongs to the link, not to the request that first opened it."""
    ids, seen = {}, {}

    def script(operator: OperatorBaton, scenario) -> None:
        operator.wait_ready(BACKEND)
        scan = operator.expect(ids.setdefault("scan", operator.next_id()))
        operator.send(BACKEND, _http("POST", "/api/vehicles/scan?device=lockstep%3Arouter&timeout=5"),
                      ids["scan"])
        BatonClock(operator).sleep(0.1)  # the scan opened the bus and polls it
        context = fleet_context(operator, scenario, SLICE)
        connect_vehicles(context)  # through the scan's bus
        operator.cancel(BACKEND, ids["scan"])
        seen["scan"] = operator.await_answer(scan)
        BatonClock(operator).sleep(0.5)
        seen["read"] = context.api.get_json(f"/api/vehicles/{SYS_ID}/params/named?names=TRIM_THROTTLE")

    run = _run(tmp_path, script)
    assert seen["scan"] == {"cancelled": True}
    assert seen["read"]["params"] == [{"name": "TRIM_THROTTLE", "value": BOUND_VALUES["TRIM_THROTTLE"]}]
    _ended_flushed(run)


def test_the_shutdown_closes_a_bus_a_scan_opened_with_no_vehicle(tmp_path):
    """The scan finds the autopilot and keeps its bus open; no vehicle holds it."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        operator.wait_ready(BACKEND)
        api = fleet_context(operator, scenario, SLICE).api
        seen["scan"] = api.post_json("/api/vehicles/scan?device=lockstep%3Arouter&timeout=2.5")

    run = _run(tmp_path, script)
    status, body = seen["scan"]
    assert status == 200 and body["found"] == 1
    _ended_flushed(run)  # the reader stopped: no managed thread outlived the shutdown


def test_unsubscribe_drops_queued_and_late_deliveries_and_the_run_journal_records_them(tmp_path):
    """Deliveries queued at ``__exit__`` and those the backend sent before it
    applied the unsubscribe are dropped and counted; receipt ticks and drops
    are in the coordinator's run journal, so in its digest."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        stream = LockstepEventStream(operator, BACKEND).__enter__()
        stream.receive(1.0)  # the initial snapshot: one receipt
        BatonClock(operator).sleep(0.5)  # telemetry queues, unreceived
        stream.__exit__(None, None, None)
        seen["exit_tick"], seen["queued"] = operator.tick, stream.queue.qsize()
        hasty = LockstepEventStream(operator, BACKEND)
        hasty.start()  # subscribe, then unsubscribe in the same tick: the backend
        hasty.__exit__(None, None, None)  # sends the snapshot before it applies the unsubscribe
        BatonClock(operator).sleep(0.1)

    run = _run(tmp_path, script)
    journal = run.operator.journal
    observed = [(r["k"], r["operator"]) for r in run.coordinator.journal.records if "operator" in r]
    receipts = [(k, subscription) for k, o in observed for subscription in o.get("receipts", [])]
    drops = [(k, count) for k, o in observed for count in o.get("dropped", {}).values()]
    assert seen["queued"] == 0
    assert receipts == journal.receipts and len(receipts) == 1
    assert sum(count for _k, count in drops) == journal.dropped
    assert drops[0][0] == seen["exit_tick"] and drops[0][1] >= 1  # the queued ones, at the exit
    assert drops[1:] == [(seen["exit_tick"] + 1, 1)]  # the hasty stream's snapshot, late
    _ended_flushed(run)


def test_cancelling_an_upload_after_its_reconnect_leaves_the_new_vehicle_heartbeat_running(tmp_path):
    """The link drops mid-upload; the upload reconnects the vehicle (a new
    vehicle, its heartbeat started inside the upload's executor job), then
    the upload is cancelled. The heartbeat is the vehicle's, not the
    request's: it keeps beating, and no thread leaks into the request scope."""
    autopilot = ReferenceAutopilot(parameters=BOUND_VALUES)
    ids, seen = {}, {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        clock = BatonClock(operator)
        clock.sleep(3.0)  # the connect-time mission probe has finished
        with LockstepEventStream(operator, BACKEND) as stream:
            autopilot.silent = True  # the link is lost
            upload = operator.expect(ids.setdefault("upload", operator.next_id()))
            operator.send(BACKEND, _http("POST", "/api/vehicles/upload",
                                         {**EMPTY_UPLOAD, "assignments": [{
                                             **EMPTY_UPLOAD["assignments"][0],
                                             "waypoints": [{"lat": 40.31, "lon": 44.45}]}]}),
                          ids["upload"])
            stages = seen.setdefault("stages", [])
            while not stages or stages[-1][1] != "reconnecting":
                stages.append((operator.tick, json.loads(stream.receive(25.0)).get("stage")))
            autopilot.silent = False  # back: the reconnect finds the vehicle
            while stages[-1][1] != "clearing":  # the retry after the reconnect has begun
                stages.append((operator.tick, json.loads(stream.receive(25.0)).get("stage")))
            seen["cancel_tick"] = operator.tick
            operator.cancel(BACKEND, ids["upload"])
            seen["upload"] = operator.await_answer(upload)
        clock.sleep(5.0)

    run = _run(tmp_path, script, autopilot=autopilot)
    assert seen["upload"] == {"cancelled": True}
    log = run.operator.journal.log_lines[BACKEND]
    connected = [line for line in log if line.endswith("Vehicle 1 (s1-u1) connected")]
    assert len(connected) == 2  # the connect, then the upload's reconnect
    beats = [k for k, kind, m in run.sent() if kind == "HEARTBEAT" and m.get_srcSystem() == 255
             and k > seen["cancel_tick"] + 1]
    assert len(beats) >= 4  # about 1 Hz for the 5 s after the cancel
    _ended_flushed(run)  # no ScopeLeak fault


def test_a_configured_restart_starts_a_new_generation_with_its_own_pid(tmp_path):
    """``/navpy-sim/restart-ready`` stops generation 1, starts generation 2 and
    validates it: the replacement's pid differs from its predecessor's."""
    seen = {}
    expected = {"targ_wps": READY_PAYLOAD["targ_wps"], "nav_last_wp": READY_PAYLOAD["nav_last_wp"]}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        context = fleet_context(operator, scenario, SLICE)
        start_companions(context, 10.0)
        seen["restart"] = context.api.post_json("/api/control/navpy-sim/restart-ready", {
            "instances": [{"sys_id": SYS_ID, "expected_params": expected}]})
        seen["status"] = context.api.get_json("/api/control/navpy-sim/status")
        stop_companions(context)

    run = _run(tmp_path, script, companions=True)
    status, body = seen["restart"]
    assert status == 200, body
    [instance] = body["instances"]
    assert (instance["old_pid"], instance["pid"]) == (generation_pid(NAVPY, 1), generation_pid(NAVPY, 2))
    [current] = seen["status"]["instances"]
    assert (current["pid"], current["generation"]) == (generation_pid(NAVPY, 2), 2)
    stops = [(s.ordinal, s.generation) for done in run.dones for s in done.stop_requests]
    assert stops == [(NAVPY, 1), (NAVPY, 2)]
    _ended_flushed(run)


def test_a_stream_closed_as_the_script_returns_still_counts_its_late_drops_in_drain(tmp_path):
    """The script subscribes and unsubscribes in its last tick and returns: the
    snapshot the backend sent before applying the unsubscribe arrives in a
    drain tick, is counted as a late drop, and is in the run journal."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        hasty = LockstepEventStream(operator, BACKEND)
        hasty.start()
        hasty.__exit__(None, None, None)
        seen["return_tick"] = operator.tick  # no trailing sleep

    run = _run(tmp_path, script)
    records = run.coordinator.journal.records
    drops = [(r["k"], r["phase"], r["operator"]["dropped"]) for r in records
             if "dropped" in r.get("operator", {})]
    assert drops and all(phase == "drain" for _k, phase, _d in drops)
    assert sum(sum(d.values()) for _k, _p, d in drops) == run.operator.journal.dropped == 1
    _ended_flushed(run)


def test_with_no_companions_the_launcher_port_refuses_a_start_and_spawns_nothing(tmp_path):
    """The lockstep launcher is installed even with no configured companions:
    a start is refused (409), never handed to the subprocess manager."""
    seen = {}

    def script(operator: OperatorBaton, scenario) -> None:
        _connected(operator, scenario)
        api = fleet_context(operator, scenario, SLICE).api
        seen["start"] = api.post_json("/api/control/navpy-sim/start", {"sys_id": SYS_ID}, expected=(409,))
        seen["status"] = api.get_json("/api/control/navpy-sim/status")

    run = _run(tmp_path, script)
    status, body = seen["start"]
    assert status == 409 and "no lockstep companion role" in body["detail"]
    assert seen["status"] == {"instances": []}
    assert not [s for done in run.dones for s in done.spawn_requests]
    _ended_flushed(run)
