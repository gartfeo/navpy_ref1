"""The 10a coordinator for N SITLs: a scripted 3-link toy on recorded frames.

Three scripted SITLs share the recorded serial-step clock; a hub (backend,
ordinal N+1) and three rovers (NavPy-like, ordinals 1-3, spawned one at a
time through the hub) run as real processes over NVPART01
(``fleet_doubles``). ``lockstep_fleet.md``: Fleet barrier, Participants,
Terminal tick.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from gcs.backend.instance_registry_runtime import pid_alive
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault, Manifest, backend_ordinal
from navpy.sim.lockstep.step_wire import unpack_reply
from scripts.lockstep_slice_actor import BACKEND as SLICE_BACKEND
from tests.modules.lockstep.coordinator_doubles import ScriptedSitl
from tests.modules.lockstep.fleet_doubles import (
    HUB,
    ROVERS,
    build_fleet,
    fleet_manifest,
    fleet_sitls,
    recorded_source_us,
    serial0,
    serial1,
)
from tests.modules.lockstep.test_grid_check_recording import recorded_frames


def _run(tmp_path, name, **kwargs):
    coordinator, sitls, operator, roster = build_fleet(tmp_path / name, **kwargs)
    result = coordinator.run()
    return coordinator, sitls, operator, roster, result


def _ticks(coordinator) -> list[dict]:
    return [r for r in coordinator.journal.records if "members" in r]


def _first_member_tick(coordinator, ordinal: int) -> int:
    return next(r["k"] for r in _ticks(coordinator) if any(m["ordinal"] == ordinal for m in r["members"]))


def _end(coordinator) -> dict:
    return coordinator.journal.records[-1]


def test_the_three_link_toy_is_deterministic_under_host_delays(tmp_path):
    plain = _run(tmp_path, "a")
    again = _run(tmp_path, "b")
    delayed = _run(tmp_path, "c", delay_seed=7, plan=fleet_manifest(
        hub={"exit_delay_s": 0.2}, rovers={i: {"exit_delay_s": 0.2} for i in ROVERS}))
    assert plain[0].journal.digest == again[0].journal.digest == delayed[0].journal.digest
    assert plain[0].journal.records == delayed[0].journal.records
    assert [s.snapshot_rows for s in plain[1]] == [s.snapshot_rows for s in delayed[1]]
    coordinator, sitls, operator, roster, result = plain
    assert result.outcome == "ok"
    assert all(m.flushed and m.exited for m in roster.members)
    assert all(sitl.stopped for sitl in sitls)
    # Every tick reads and replies to every SITL; the hub (N+1) ticks last.
    mission = [r for r in _ticks(coordinator) if r["phase"] != "drain"]
    assert all(len(r["frame"]) == 3 and len(r["reply"]) == 3 for r in mission)
    full = [r for r in mission if len(r["members"]) == 4]
    assert full and all([m["ordinal"] for m in r["members"]] == [1, 2, 3, HUB] for r in full)


def test_the_backend_is_ordinal_n_plus_one():
    assert backend_ordinal(3) == HUB == 4
    assert backend_ordinal(1) == SLICE_BACKEND == 2


def test_spawns_are_sequential_and_each_sitl_serial0_reaches_its_own_rover(tmp_path):
    coordinator, _sitls, operator, _roster, _result = _run(tmp_path, "run")
    admitted = [_first_member_tick(coordinator, rover) for rover in ROVERS]
    assert admitted == sorted(admitted) and len(set(admitted)) == 3
    lifecycle = [(r["k"], e["kind"], e["ordinal"]) for r in _ticks(coordinator) for e in r["lifecycle"]]
    for previous, rover in zip(ROVERS, ROVERS[1:]):
        ready = next(k for k, kind, ordinal in lifecycle if kind == "ready" and ordinal == previous)
        spawned = next(k for k, kind, ordinal in lifecycle if kind == "spawn" and ordinal == rover)
        assert spawned > ready  # the next rover starts after the previous one's readiness
    # Before its rover is admitted, SITL i's serial0 egress has no owner and is
    # discarded; the other SITLs' serial0 links are unaffected.
    for rover, first in zip(ROVERS, admitted):
        before = [r for r in _ticks(coordinator) if r["phase"] == "mission" and r["k"] < first]
        assert before and all(r["discarded"].get(serial0(rover)) for r in before)
        after = [r for r in _ticks(coordinator) if r["phase"] == "mission" and r["k"] >= first]
        assert all(serial0(rover) not in r["discarded"] for r in after)


def _capture(sitls, roster):
    """Record every TICK's ports by ordinal and every reply's ingress by SITL."""
    ticks: dict[tuple[int, int], tuple] = {}
    replies: dict[tuple[int, int], tuple[bytes, ...]] = {}
    send = roster.send

    def capture_tick(member, tick):
        ticks[(member.ordinal, tick.k)] = tick.ports
        return send(member, tick)

    roster.send = capture_tick
    for index, sitl in enumerate(sitls, start=1):
        def capture_reply(reply, timeout_s=math.inf, sitl=sitl, index=index, original=sitl.send_reply):
            replies[(index, sitl.k)] = unpack_reply(reply, expected_tick=sitl.k)
            original(reply, timeout_s)

        sitl.send_reply = capture_reply
    return ticks, replies


def test_each_rover_gets_its_own_sitls_serial0_bytes_and_answers_into_its_reply(tmp_path):
    coordinator, sitls, _operator, roster = build_fleet(tmp_path)
    ticks, replies = _capture(sitls, roster)
    coordinator.run()
    checked = 0
    for (ordinal, k), ports in sorted(ticks.items()):
        if ordinal == HUB:  # the hub owns every SITL's serial1, each with its own SITL's bytes
            if ports:
                assert [(device, data[:3]) for device, data in ports] == [
                    (serial1(i), f"S{i}:".encode()) for i in ROVERS]
                for i, (_device, data) in zip(ROVERS, ports):
                    assert data.startswith(f"S{i}:s1k{k}|".encode())
                    assert replies[(i, k)][1] == b"H" + data[::-1]  # its echo, on SITL i's serial1
                checked += 1
            continue
        if not ports:  # a drain TICK carries no port bytes
            continue
        ((device, data),) = ports
        assert device == serial0(ordinal)
        assert data.startswith(f"S{ordinal}:s0k{k}|".encode())  # SITL i's frame k, serial0
        # The rover echoes it reversed behind its tag in DONE k: reply k to
        # SITL i (and to no other SITL) carries it on serial0.
        assert replies[(ordinal, k)][0] == b"R" + data[::-1]
        assert all(b"R" + data[::-1] not in replies[(other, k)][0] for other in ROVERS if other != ordinal)
        checked += 1
    assert {ordinal for ordinal, _k in ticks if ordinal != HUB} == set(ROVERS) and checked > 60


def test_k_t_is_the_latest_first_snap_of_the_expected_fleet(tmp_path):
    coordinator, _sitls, operator, _roster, result = _run(tmp_path, "run")
    assert result.outcome == "ok"
    first = _end(coordinator)["first_snaps"]
    assert [ordinal for ordinal, _generation, _k in first] == list(ROVERS)
    assert {ordinal: k for ordinal, _generation, k in first} == operator.snaps
    assert result.terminal_tick == max(k for *_rest, k in first) == operator.snaps[3]
    assert operator.snaps[1] < operator.snaps[2] < operator.snaps[3]
    # A SNAP of rover 1 alone ended nothing: the mission ran on to the last one.
    phases = {r["k"]: r["phase"] for r in _ticks(coordinator)}
    assert phases[operator.snaps[1]] == phases[operator.snaps[2]] == "mission"
    assert phases[result.terminal_tick] == "mission" and phases[result.terminal_tick + 1] == "finalization"


def test_completion_wins_over_a_budget_expiring_in_the_same_tick(tmp_path):
    reference = _run(tmp_path, "reference")
    k_t = reference[4].terminal_tick
    start = next(r["k"] for r in _ticks(reference[0]) if r["phase"] == "mission")
    times = dict(recorded_frames())
    budget = times[k_t] - times[start]  # the budget runs out exactly at k_T
    coordinator, *_rest, result = _run(tmp_path, "shared", plan=fleet_manifest(mission_budget_us=budget))
    assert (result.outcome, result.terminal_tick) == ("ok", k_t)
    assert not any("timeout" in r for r in coordinator.journal.records)
    early_budget = times[k_t - 1] - times[start]  # the budget runs out one tick before k_T
    early = _run(tmp_path, "early", plan=fleet_manifest(mission_budget_us=early_budget))[4]
    assert (early.outcome, early.terminal_tick) == ("timeout", k_t - 1)


def test_an_unspawned_expected_companion_never_completes_the_mission(tmp_path):
    coordinator, _sitls, operator, roster, result = _run(tmp_path, "run", spawn=(1, 2))
    assert operator.snaps.keys() == {1, 2}
    assert result.outcome == "operator_returned"
    assert [o for o, _g, _k in _end(coordinator)["first_snaps"]] == [1, 2]
    assert [m.ordinal for m in roster.members] == [HUB, 1, 2]


def test_a_companion_retired_without_snap_never_completes_the_mission(tmp_path):
    plan = fleet_manifest(rovers={2: {"snap_at_s": 1e9}})
    coordinator, _sitls, operator, roster, result = _run(tmp_path, "run", plan=plan, stop_before_snap=(2,))
    assert operator.snaps.keys() == {1, 3}
    rover2 = next(m for m in roster.members if m.ordinal == 2)
    assert rover2.retired_at < result.final_tick and rover2.flushed
    assert result.outcome == "operator_returned"
    assert [o for o, _g, _k in _end(coordinator)["first_snaps"]] == [1, 3]


def test_a_mission_budget_ends_an_incomplete_fleet_as_a_timeout(tmp_path):
    plan = fleet_manifest(rovers={3: {"snap_at_s": 1e9}}, mission_budget_us=1_500_000)
    coordinator, *_rest, result = _run(tmp_path, "run", plan=plan, give_up_at_us=3_600_000)
    assert result.outcome == "timeout"
    assert {"timeout": "mission sim-time budget", "k": result.terminal_tick} in coordinator.journal.records


def test_a_late_latch_holds_admission(tmp_path):
    sitls = fleet_sitls(quiet_from=(3, 3, 8))
    plan = fleet_manifest(boot_budget_us=200_000)
    coordinator, *_rest, result = _run(tmp_path, "run", plan=plan, sitls=sitls, spawn=())
    assert result.outcome == "operator_returned"  # the hub only: nothing to wait for
    boot = [r for r in _ticks(coordinator) if r["phase"] == "boot"]
    assert [r["k"] for r in boot] == list(range(1, 9))
    assert all(r["members"] == [] for r in boot)
    assert [(r["k"], r["latched"]) for r in boot if "latched" in r] == [(3, [1, 2]), (8, [3])]
    assert _first_member_tick(coordinator, HUB) == 9


def test_the_boot_barrier_faults_when_a_latch_never_comes_within_the_budget(tmp_path):
    sitls = fleet_sitls(quiet_from=(3, 10**9, 3))
    plan = fleet_manifest(boot_budget_us=100_000)
    coordinator, _sitls, _operator, roster = build_fleet(tmp_path, plan=plan, sitls=sitls)
    with pytest.raises(CoordinatorFault, match="not quiet"):
        coordinator.run()
    times = dict(recorded_frames())
    expected = next(k for k in sorted(times) if times[k] - times[1] >= 100_000)
    assert _end(coordinator) == {"fault": "CoordinatorFault", "kind": "boot_barrier", "tick": expected,
                                 "ordinal": None, "context": {"waiting": [2]}}
    assert all(r["members"] == [] for r in _ticks(coordinator))
    assert all(sitl.stopped for sitl in sitls)
    assert not any(pid_alive(m.child.pid) for m in roster.members)  # the hub registered before launch


def _clock_fault(tmp_path, sources) -> dict:
    sitls = fleet_sitls(source_us=sources)
    coordinator, *_ = build_fleet(tmp_path, plan=fleet_manifest(initial=()), sitls=sitls)
    with pytest.raises(CoordinatorFault):
        coordinator.run()
    assert all(sitl.stopped for sitl in sitls)
    return _end(coordinator)


def test_the_fleet_clock_faults_on_unequal_source_us_inside_each_grid(tmp_path):
    recorded = recorded_source_us()
    # SITL 3 runs 833 us (one physics step) behind from tick 10: its own frame
    # gaps stay on its grid, so only the fleet clock can see it.
    offset = {3: lambda k: recorded(k) - (833 if k >= 10 else 0)}
    fault = _clock_fault(tmp_path, offset)
    assert fault == {"fault": "CoordinatorFault", "kind": "fleet_clock", "tick": 10, "ordinal": None,
                     "context": {"source_us": [recorded(10), recorded(10), recorded(10) - 833]}}


def test_a_per_sitl_grid_fault_takes_precedence_over_the_fleet_clock(tmp_path):
    recorded = recorded_source_us()
    # SITL 2 jumps past Δ_max at tick 10 and SITL 3 is off the fleet clock in the
    # same tick: the per-SITL checks run first, in SITL order.
    sources = {2: lambda k: recorded(k) + (30_000 if k >= 10 else 0),
               3: lambda k: recorded(k) - (833 if k >= 10 else 0)}
    fault = _clock_fault(tmp_path, sources)
    assert (fault["kind"], fault["tick"]) == ("grid", 10)


class _Ordered(ScriptedSitl):
    """A SITL that logs each frame read into a shared list."""

    def __init__(self, index: int, log: list, **kwargs) -> None:
        super().__init__(tag=f"S{index}:".encode(), source_us=recorded_source_us(), **kwargs)
        self._index = index
        self._log = log

    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        raw = super().recv_frame(timeout_s)
        self._log.append((self._index, self.k))
        return raw


def test_parking_reads_and_journals_each_sitl_parked_frame_in_order(tmp_path):
    log: list = []
    sitls = [_Ordered(i, log) for i in ROVERS]
    coordinator, _sitls, _operator, _roster, result = _run(tmp_path, "run", sitls=sitls)
    parked = result.parked_tick
    assert log[-3:] == [(1, parked), (2, parked), (3, parked)]
    (record,) = [r["parked"] for r in coordinator.journal.records if "parked" in r]
    assert record["tick"] == parked
    assert [frame["sha256"] for frame in record["frames"]] == [s.rows[-1][2] for s in sitls]
    for sitl in sitls:  # never replied to: each is still blocked on reply k_F+1
        assert sitl.snapshot_rows[-1][0] == parked and sitl.stopped


def test_a_parked_frame_of_a_later_sitl_gets_every_check(tmp_path):
    parked = _run(tmp_path, "plain")[4].parked_tick
    sitls = fleet_sitls()
    sitls[1] = ScriptedSitl(tag=b"S2:", source_us=recorded_source_us(),
                            advances=lambda k: 1 if k == parked else 0)
    coordinator, *_ = build_fleet(tmp_path / "advance", sitls=sitls)
    with pytest.raises(HostFault, match="out-of-loop"):
        coordinator.run()
    assert _end(coordinator) == {"fault": "HostFault", "kind": "out_of_loop_advance", "tick": parked,
                                 "ordinal": None}


def test_the_sitl_ports_are_data_and_must_name_distinct_devices():
    assert fleet_manifest().sitl_devices == (
        "lockstep:serial0.1", "sitl1.serial1", "lockstep:serial0.2", "sitl2.serial1",
        "lockstep:serial0.3", "sitl3.serial1")
    with pytest.raises(ValueError, match="distinct devices"):
        dataclasses.replace(fleet_manifest(), sitl_ports=(("a", "b"), ("b", "c")))
    slice_plan = Manifest(seed=1, epoch_us=0, max_frame_gap_us=1, drain_period_us=1,
                          mission_budget_us=1, roles={})
    assert slice_plan.sitl_devices == ("lockstep:serial0", "lockstep:serial1")
    assert not slice_plan.is_fleet and fleet_manifest().is_fleet
