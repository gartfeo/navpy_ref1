"""Step 10d's fleet check loop (``ScriptedFleetRun``) on three scripted SITLs.

The real run flies step 7's scripted operator on three serial-step SITLs
(``scripts/eval_lockstep_fleet_auto.py``); here the SITLs are the 10a
recorded-clock doubles and the operator a stand-in, so the loop's contract
is pinned without WSL: boot ticks reply empty, the operator is admitted at
the tick after the last latch, its bytes go to each SITL's own serial1, the
journal repeats under host delays, and every fault stops every SITL.
"""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
from navpy.sim.lockstep.coordinator_fleet import SitlFleet
from navpy.sim.lockstep.baton import LockstepFault
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault
from navpy.sim.lockstep.scripted_fleet import ScriptedFleetRun
from scripts.eval_lockstep_fleet_auto import fleet_manifest
from tests.modules.lockstep.coordinator_doubles import HostDelays
from tests.modules.lockstep.fleet_doubles import fleet_sitls, recorded_source_us
from tests.modules.lockstep.test_grid_check_recording import recorded_frames


class StandInOperator:
    """Writes ``to<i>@<k>`` to SITL i each tick; done after ``ticks`` ticks."""

    def __init__(self, ticks: int = 20) -> None:
        self._ticks = ticks
        self.seen: list[tuple[int, list[bytes]]] = []
        self.tick = 0

    @property
    def done(self) -> bool:
        return len(self.seen) >= self._ticks

    def on_tick(self, k, now_us, egress, deadline=None):
        self.tick = k
        self.seen.append((k, list(egress)))
        return [f"to{i}@{k}|".encode() for i in (1, 2, 3)]


def _run(*, sitls=None, budget=None, delay_seed=0, max_ticks=200, operator=None):
    delays = HostDelays(delay_seed)
    sitls = sitls or fleet_sitls(delays=delays)
    plan = fleet_manifest(1, budget)
    deadline = ExchangeDeadline()
    operator = operator or StandInOperator()
    run = ScriptedFleetRun(plan, SitlFleet(plan, sitls, deadline), deadline, operator,
                           max_ticks=max_ticks, stop_when=lambda: operator.done, delay=delays)
    return run, sitls, operator


def test_boot_replies_empty_and_the_operator_joins_after_the_last_latch():
    run, sitls, operator = _run(sitls=fleet_sitls(quiet_from=(3, 3, 8)), budget=200_000)
    result = run.run()
    ticks = [r for r in run.journal.records if "k" in r]
    assert [r["k"] for r in ticks if r["phase"] == "boot"] == list(range(1, 9))
    assert [(r["k"], r["latched"]) for r in ticks if "latched" in r] == [(3, [1, 2]), (8, [3])]
    assert (result.barrier_tick, result.latches) == (8, (3, 3, 8))
    assert operator.seen[0][0] == 9
    assert result.final_tick == 28 and result.parked_tick == 29
    assert all(sitl.stopped and sitl.snapshot_rows[-1][0] == 29 for sitl in sitls)


def test_each_sitls_serial1_talks_to_its_own_vehicle():
    run, _sitls, operator = _run()
    run.run()
    (k, first), (_k2, second) = operator.seen[:2]
    for i, data in enumerate(second, start=1):
        assert data.startswith(f"S{i}:s1k{k + 1}|".encode())  # SITL i's serial1, tagged
        assert f"to{i}@{k}|".encode() in data  # what the operator sent SITL i last tick
        assert all(f"to{j}@".encode() not in data for j in (1, 2, 3) if j != i)


def test_the_journal_repeats_under_host_delays():
    plain, *_ = _run()
    plain.run()
    delayed, *_ = _run(delay_seed=7)
    delayed.run()
    assert plain.journal.records == delayed.journal.records
    assert plain.journal.digest == delayed.journal.digest


def test_unequal_source_us_is_the_fleet_clock_fault_and_stops_every_sitl():
    recorded = recorded_source_us()
    sitls = fleet_sitls(source_us={2: lambda k: recorded(k) - (833 if k >= 12 else 0)})
    run, *_ = _run(sitls=sitls)
    with pytest.raises(CoordinatorFault, match="unequal source_us"):
        run.run()
    end = run.journal.records[-1]
    assert (end["kind"], end["tick"]) == ("fleet_clock", 12)
    assert end["context"]["source_us"][1] == end["context"]["source_us"][0] - 833
    assert all(sitl.stopped for sitl in sitls)


def test_a_barrier_past_its_budget_and_a_run_past_its_ticks_stop_every_sitl():
    sitls = fleet_sitls(quiet_from=(3, 10**9, 3))
    run, *_ = _run(sitls=sitls, budget=100_000)
    with pytest.raises(CoordinatorFault, match="not quiet"):
        run.run()
    times = dict(recorded_frames())
    assert run.journal.records[-1]["tick"] == next(k for k in sorted(times) if times[k] - times[1] >= 100_000)
    assert all(sitl.stopped for sitl in sitls)
    long_run, sitls, _operator = _run(operator=StandInOperator(ticks=10**6), max_ticks=30)
    with pytest.raises(CoordinatorFault, match="within 30 ticks"):
        long_run.run()
    assert long_run.journal.records[-1]["kind"] == "tick_budget"
    assert all(sitl.stopped for sitl in sitls)


def _deadline_run(operator, *, step_s: float = 1.0, margin_s: float = 0.5):
    """A real-clock exchange deadline: each tick's exchange gets ``step - margin``."""
    sitls = fleet_sitls()
    plan = fleet_manifest(1, None)
    deadline = ExchangeDeadline(step_s, margin_s)
    run = ScriptedFleetRun(plan, SitlFleet(plan, sitls, deadline), deadline, operator,
                           max_ticks=50, stop_when=lambda: operator.done)
    return run, sitls


def test_the_scripted_operators_startup_runs_on_the_exchange_deadline(monkeypatch):
    """Codex 10d round 1 P2-3: the operator's first baton round (its threads'
    start) draws on the tick's deadline, not the baton's own 30 s; a handoff
    cut by it is ``exchange_budget`` in the operator wait, and every SITL stops."""
    import time

    from navpy.sim.lockstep.scripted_operator import ScriptedFleetOperator
    from navpy.sim.lockstep.scripted_vehicle import ScriptedVehicle

    BLOCK_S = 2.0
    def blocked_start(self, runtime):  # a thread that blocks on a host wait at startup
        runtime.spawn(lambda: time.sleep(BLOCK_S), name=f"operator-blocked-{self.sysid}")

    monkeypatch.setattr(ScriptedVehicle, "start", blocked_start)
    operator = ScriptedFleetOperator(((1, "40,44,0,0"), (2, "40,44,0,0"), (3, "40,44,0,0")))
    run, sitls = _deadline_run(operator)
    started = time.monotonic()
    try:
        with pytest.raises(HostFault, match="operator wait") as raised:
            run.run()
        elapsed = time.monotonic() - started
    finally:
        operator.close()  # the poisoned baton refuses an abort; close still restores the modules
    assert elapsed < BLOCK_S  # cut at the 0.5 s exchange, not the 30 s baton budget
    assert raised.value.kind == "exchange_budget"
    assert run.journal.records[-1] == {"fault": "HostFault", "kind": "exchange_budget", "tick": 4,
                                       "ordinal": None, "context": {"wait": "operator"}}
    assert all(sitl.stopped for sitl in sitls)


class _FailingOperator(StandInOperator):
    """Its baton fails at tick 6, after (or before) the tick's wall deadline."""

    def __init__(self, *, past_deadline: bool) -> None:
        super().__init__()
        self._past = past_deadline

    def on_tick(self, k, now_us, egress, deadline=None):
        if k == 6:
            if self._past:
                import time

                time.sleep(max(0.0, deadline - time.monotonic()) + 0.05)
            raise LockstepFault("baton handoff past its budget")
        return super().on_tick(k, now_us, egress, deadline)


def test_a_baton_fault_past_the_deadline_is_exchange_budget_and_before_it_stays_its_own():
    late, sitls = _deadline_run(_FailingOperator(past_deadline=True))
    with pytest.raises(HostFault, match="operator wait"):
        late.run()
    assert late.journal.records[-1]["kind"] == "exchange_budget" and all(s.stopped for s in sitls)
    early, sitls = _deadline_run(_FailingOperator(past_deadline=False))
    with pytest.raises(LockstepFault, match="past its budget") as raised:
        early.run()
    assert not isinstance(raised.value, HostFault)
    assert early.journal.records[-1] == {"fault": "unexpected", "kind": "LockstepFault", "tick": 6,
                                         "ordinal": None}
    assert all(s.stopped for s in sitls)
