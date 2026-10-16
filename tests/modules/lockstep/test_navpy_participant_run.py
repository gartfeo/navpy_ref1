"""9f acceptance: NavPy spawned through the launcher port, booted, stopped through the API.

The real backend and the real NavPy run as participant processes under the
9d coordinator, against the reference autopilot's byte stream (serial0 for
NavPy, serial1 for the backend; the autopilot carries the SITL's AAS Lua
table). A scripted API actor makes 9e's setup calls (connect, the command
bounds by name, the AAS writes, the slice mission upload), then POST
start-all: the backend's launcher port puts a spawn request in its ``DONE``,
the coordinator spawns NavPy (``-c lockstep:serial0.1 --stop-on-lifecycle``)
and admits it at the next tick. NavPy boots on ticks (heartbeat, mission
download, parameter reads, terrain from the DEM after the 1 s report wait);
its ``runtime_ready`` event becomes the ``ready`` lifecycle record that the
status route reports. Then stop-all: the stop request becomes NavPy's
lifecycle ``stop``, its run loop ends, cleanup joins its producers and
drains the log worker, the ack is a ``stopped`` event, and its ending
``DONE`` (flushed) answers ``STOP_ACKED``. Drain then ends the backend.

Two runs under two claimed slots, the second with host delays, give the
same journal and digest, the same serial bytes both ways, the same NavPy
log lines (``DONE.log_lines``) and the same NavPy log files. Apart, NavPy
as an initial participant that no stop reaches ends on the drain ``end``.
The scripted actor is ``scripts.lockstep_slice_actor.companion_script``.
"""

from __future__ import annotations

import json

import pytest

from gcs.backend import instance_registry_lockstep as lockstep
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_tick import READY_EVENT
from navpy.sim.lockstep.coordinator_types import Manifest, RoleSpec, WallBudgets
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.participant_hooks import DEVICE_KEY, RequestIds
from scripts.eval_gcs_demo_config import build_settings
from scripts.eval_gcs_demo_scenario import load_scenario
from scripts.lockstep_slice_actor import (
    NAVPY,
    ROUTER,
    SERIAL0,
    SITL_PORTS,
    slice_argv,
    companion_script,
)
from scripts.lockstep_fleet_actor import NAVPY_HOOKS
from tests.modules.lockstep.backend_doubles import BOUND_VALUES, SLICE_SCENARIO, AutopilotSitl, build
from tests.modules.lockstep.coordinator_doubles import PYTHON_PATH, AllowAll
from tests.modules.lockstep.lockstep_autopilot import ReferenceAutopilot, navpy_parameters


def _claim() -> tuple[int, dict]:
    return lockstep.claim(run_token="seed-1", clone="clone", owner="dir:test", port_check=lambda n: True)


def _run(tmp_path, monkeypatch, name: str, slot: int, delay_seed: int = 0):
    monkeypatch.setenv("GCS_CHAT_INDEX", str(slot))  # host side only
    run = build(tmp_path / name, delay_seed=delay_seed, script=companion_script, navpy=True,
                autopilot=ReferenceAutopilot(parameters={**BOUND_VALUES, **navpy_parameters()}))
    return run, run.run()


def _navpy_files(run_dir) -> dict[str, bytes]:
    logs = run_dir / "host"  # NavPy logs into its working directory, as in wall mode
    return {path.name: path.read_bytes() for path in sorted(logs.glob("uav_*")) if path.is_file()}


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("navpy_runs")
    with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setenv("GCS_INSTANCE_REGISTRY", str(tmp_path / "instances.json"))
        (first, first_owner), (second, second_owner) = _claim(), _claim()  # both held: two slots
        assert first != second
        try:
            plain = _run(tmp_path, monkeypatch, "first", first)
            delayed = _run(tmp_path, monkeypatch, "second", second, delay_seed=7)
        finally:
            assert lockstep.release_if_owner(first, first_owner)
            assert lockstep.release_if_owner(second, second_owner)
    return tmp_path, plain, delayed


def test_navpy_boots_and_stops_through_the_api_and_the_run_repeats(runs):
    tmp_path, (plain, plain_result), (delayed, delayed_result) = runs
    assert plain_result == delayed_result
    assert plain.coordinator.journal.records == delayed.coordinator.journal.records
    assert plain.coordinator.journal.digest == delayed.coordinator.journal.digest
    assert plain.sitl.serial1 == delayed.sitl.serial1
    assert plain.sitl.snapshot_rows == delayed.sitl.snapshot_rows
    assert plain.operator.result == delayed.operator.result
    log = plain.operator.journal.log_lines[NAVPY]
    assert log == delayed.operator.journal.log_lines[NAVPY]
    assert _navpy_files(tmp_path / "first") == _navpy_files(tmp_path / "second")

    result = plain.operator.result
    (ready,) = result["ready"]  # one status per companion
    assert ready["running"] and ready["connection"] == SERIAL0
    assert ready["runtime"]["ordinal"] == NAVPY and ready["runtime"]["process_pid"] == NAVPY
    assert ready["runtime"]["mission_items"] > 6 and ready["runtime"]["role"] == "navpy"
    assert result["stopped"]["instances"] == []  # the stopped generation is untracked

    dones = [done for done in plain.dones if any(device == SERIAL0 for device, _ in done.ports)]
    kinds = [event.kind for done in dones for event in done.events]
    assert kinds == [READY_EVENT, "stopped"]
    ending = dones[-1]
    assert ending.ended is True and [e.kind for e in ending.events] == ["stopped"]
    host = json.loads((tmp_path / "first" / "lockstep_participants_host.json").read_text())
    assert sorted((row["role"], row["exit_code"]) for row in host) == [("backend", 0), ("navpy1", 0)]


def test_navpy_log_lines_are_its_file_log_and_name_no_host_path(runs):
    tmp_path, (plain, _result), _delayed = runs
    log = plain.operator.journal.log_lines[NAVPY]
    assert any("Args (non-default): connection='lockstep:serial0.1'" in line
               and "stop_on_lifecycle=True" in line for line in log)
    assert any(line.endswith("Mission download successful.") for line in log)
    assert not [line for line in log if str(tmp_path) in line or tmp_path.as_posix() in line]
    # The file log is the same lines, and names no host path either (the
    # repository's files read ``<abs>``; the host log keeps the real ones).
    navigation_log = (tmp_path / "first" / "host" / "uav_1_navigation.log")
    assert navigation_log.read_text(encoding="utf-8").splitlines() == log
    assert any("Using vision profile 'siyi_zr10' from <abs>" in line for line in log)


def test_nothing_writes_sim_speedup_and_the_stop_is_acked(runs):
    _tmp_path, (plain, _result), _delayed = runs
    autopilot = plain.sitl.autopilot
    assert "SIM_SPEEDUP" not in autopilot.parameters
    navpy_sets = [kind for port, kind, _source, _component in autopilot.received
                  if port == 0 and kind == "PARAM_SET"]
    assert navpy_sets == []
    lifecycle = [entry for record in plain.coordinator.journal.records
                 for entry in record.get("lifecycle", ())]
    results = [entry["result"] for entry in lifecycle if entry["kind"] == "stop_result"]
    assert results == ["acked"]


def test_the_terrain_comes_from_the_dem_after_an_unanswered_report_wait(runs):
    """``TERRAIN_ENABLE 0``: NavPy asks, the autopilot never reports, and the
    POI is placed 1 s of sim time later from the DEM."""
    _tmp_path, (plain, _result), _delayed = runs
    received = plain.sitl.autopilot.received
    assert any(port == 0 and kind == "TERRAIN_CHECK" for port, kind, _s, _c in received)
    log = plain.operator.journal.log_lines[NAVPY]
    home = next(i for i, line in enumerate(log) if "Using home from mission" in line)
    poi = next(i for i, line in enumerate(log) if "P1: wp:" in line)
    stamp = lambda line: line.split(" ")[1]  # noqa: E731  (HH:MM:SS,mmm)
    assert poi > home and (stamp(log[home]), stamp(log[poi])) == ("08:53:24,660", "08:53:25,660")


def _drain_script(operator) -> str:
    operator.wait_ready(NAVPY)
    return "ready"


def test_navpy_not_stopped_by_the_api_ends_on_the_drain_end(tmp_path):
    """NavPy as an initial participant: the operator returns once it is ready,
    and the first drain ``TICK`` (``end``) stops its run loop; it ends
    flushed, with no ``stopped`` event (there was no participant stop)."""
    argv = slice_argv(build_settings(load_scenario(SLICE_SCENARIO)))
    plan = Manifest(seed=1, epoch_us=1_760_000_000_000_000, max_frame_gap_us=25_000,
                    drain_period_us=20_000, mission_budget_us=0,
                    roles={"navpy": RoleSpec(NAVPY_HOOKS, (SERIAL0,), {DEVICE_KEY: SERIAL0, "argv": argv})},
                    sitl_ports=SITL_PORTS, router=ROUTER, initial=(("navpy", NAVPY),))
    sitl = AutopilotSitl(ReferenceAutopilot(parameters={**BOUND_VALUES, **navpy_parameters()}))
    roster = Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH), AllowAll(),
                    HostSidecar(tmp_path / PROCESSES_HOST_FILE),
                    WallBudgets(register_s=120, done_s=120, exit_s=60))
    ids = RequestIds(0)
    operator = OperatorBaton(ids, _drain_script)
    coordinator = LockstepCoordinator(plan, (sitl,), operator, roster, ids, tmp_path / "host")
    dones = []
    receive = roster.receive
    roster.receive = lambda member, k: dones.append(receive(member, k)) or dones[-1]
    result = coordinator.run()

    assert result.outcome == "timeout" and result.parked_tick == result.final_tick + 1
    ending = dones[-1][0]
    assert ending.ended is True and ending.k > result.final_tick  # a drain tick
    assert [event.kind for done, _raw in dones for event in done.events] == [READY_EVENT]
    host = json.loads((tmp_path / PROCESSES_HOST_FILE).read_text())
    assert [row["exit_code"] for row in host] == [0]
    assert operator.journal.log_lines[NAVPY][-1].endswith("Logger closed")
