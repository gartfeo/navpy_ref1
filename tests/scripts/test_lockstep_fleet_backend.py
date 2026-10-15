"""The 10e/10f harness's verdict and its fixture pin (``scripts/eval_lockstep_fleet_backend.py``).

A run passes only with the intended outcome (``operator_returned``) and no
fault and, with NavPys (10f), a complete NAVLINK trace for every ordered
pair and none lost before stop-all; acceptance also needs two or more runs with equal artifacts, and a
single run makes no repeat claim. A run whose children are not confirmed
stopped keeps its slot; every other slot is released. The frozen fixture is
refused unless its canonical sha256 is the pinned one, which names the
certified file's source revision.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

from navpy.sim.lockstep.navlink_trace import HOPS, NavlinkTrace
from scripts import eval_lockstep_fleet_backend as harness
from scripts import lockstep_navlink_gate as gate

IDS = harness.FLEET.sys_ids

ROOT = Path(__file__).resolve().parents[2]
SAME = {"lockstep_journal.jsonl": True, "fleet_actor.json": True, "backend_log_lines.txt": True, "gcs": True}


def _run(outcome: str = "operator_returned", *, fault: str | None = None, fault_records: int = 0,
         navlink: dict | None = None) -> dict:
    return {"result": {"outcome": outcome}, "fault": fault, "fault_records": fault_records, "navlink": navlink}


def _trace(sender: int, receiver: int, *, delivered: int | None = 17, sent: int = 10,
           ambiguous: bool = False) -> NavlinkTrace:
    ticks = (sent, 11, 13, 14, 16, delivered)
    hops = tuple(zip(HOPS, ticks if delivered is not None else ticks[:5]))
    return NavlinkTrace((sender, 77, 5), 25004, sender, receiver, hops, ambiguous)


def _every_pair(**kwargs) -> list[NavlinkTrace]:
    return [_trace(i, j, **kwargs) for i in harness.FLEET.sys_ids for j in harness.FLEET.sys_ids if i != j]


def test_two_equal_runs_with_the_intended_outcome_are_accepted():
    assert harness.verdict([_run(), _run()], SAME) == {
        "accepted": True, "repeats": True, "outcomes_ok": True, "passed": True}


@pytest.mark.parametrize("bad", [_run("timeout"), _run(fault="HostFault: x"), _run(fault_records=1),
                                 {"result": None, "fault": "RuntimeError: y", "fault_records": 1}])
def test_equal_artifacts_do_not_pass_without_the_intended_outcome(bad):
    for runs in ([bad, bad], [_run(), bad]):
        verdict = harness.verdict(runs, SAME)
        assert not verdict["passed"] and not verdict["accepted"] and not verdict["outcomes_ok"]


def test_with_navpys_a_run_passes_only_if_every_ordered_pair_has_a_complete_trace():
    complete = gate.trace_summary(_every_pair(), IDS, 100)
    assert complete["pairs_complete"] and harness.run_ok(_run(navlink=complete))
    missing = gate.trace_summary(_every_pair()[1:], IDS, 100)  # no complete 1>2
    assert missing["pairs"]["1>2"] == {"traces": 0, "complete": 0, "latency_ticks": None}
    assert not missing["pairs_complete"] and not harness.run_ok(_run(navlink=missing))
    short = gate.trace_summary([*_every_pair()[1:], _trace(1, 2, delivered=None)], IDS, 100)
    assert short["pairs"]["1>2"] == {"traces": 1, "complete": 0, "latency_ticks": None}
    assert not short["pairs_complete"]


def test_a_packet_lost_before_stop_all_fails_the_run_and_one_in_flight_at_the_stop_does_not():
    in_flight = gate.trace_summary([*_every_pair(), _trace(1, 2, delivered=None, sent=100)], IDS, 100)
    assert in_flight["lost_before_stop"] == 0 and harness.run_ok(_run(navlink=in_flight))
    lost = gate.trace_summary([*_every_pair(), _trace(1, 2, delivered=None, sent=99)], IDS, 100)
    assert lost["pairs_complete"] and lost["lost_before_stop"] == 1
    assert not harness.run_ok(_run(navlink=lost))
    assert gate.trace_summary([_trace(1, 2, delivered=None, sent=500)], IDS, None)["lost_before_stop"] == 1
    records = [{"k": 5, "lifecycle": [{"kind": "spawn"}]}, {"k": 7, "lifecycle": [{"kind": "stop"}]},
               {"k": 9, "lifecycle": [{"kind": "stop"}]}]
    assert gate.first_stop_tick(records) == 7 and gate.first_stop_tick(records[:1]) is None


def test_an_ambiguous_trace_fails_the_run_and_never_counts_as_delivered():
    traces = [*_every_pair()[1:], _trace(1, 2, ambiguous=True)]
    summary = gate.trace_summary(traces, IDS, 100)
    assert summary["ambiguous"] == 1 and summary["pairs"]["1>2"]["complete"] == 0
    assert not summary["pairs_complete"] and not harness.run_ok(_run(navlink=summary))
    covered = gate.trace_summary([*_every_pair(), _trace(1, 2, ambiguous=True)], IDS, 100)
    assert covered["pairs_complete"] and covered["lost_before_stop"] == 0
    assert not harness.run_ok(_run(navlink=covered))
    assert gate.trace_rows([_trace(1, 2, ambiguous=True)])[0]["ambiguous"] is True


def test_the_trace_summary_keeps_each_pairs_latency_range():
    traces = [*_every_pair(), _trace(1, 2, delivered=19)]
    summary = gate.trace_summary(traces, IDS, 100)
    assert summary["pairs"]["1>2"] == {"traces": 2, "complete": 2, "latency_ticks": [7, 9]}
    assert summary["complete"] == 7 and summary["pairs_complete"]
    rows = gate.trace_rows(traces[:1])
    assert rows == [{"uid": [1, 77, 5], "msgid": 25004, "sender": 1, "receiver": 2,
                     "hops": [list(hop) for hop in traces[0].hops], "complete": True, "latency_ticks": 7,
                     "ambiguous": False}]


def test_the_navpy_manifest_spawns_each_navpy_and_expects_the_whole_fleet():
    scenario, _pin = harness.load_fleet_scenario()
    plan = harness.fleet_manifest(scenario, 1, navpy=True)
    assert list(plan.roles) == ["navpy1", "navpy2", "navpy3", "backend"]
    assert plan.companion_roles == ("navpy1", "navpy2", "navpy3") and plan.expected_fleet == (1, 2, 3)
    assert plan.initial == (("backend", 4),)  # the launcher port spawns every NavPy
    assert harness.NAVPY_FILES == ("navlink_traces.json", "navpy1_log_lines.txt", "navpy2_log_lines.txt",
                                   "navpy3_log_lines.txt")
    backend_only = harness.fleet_manifest(scenario, 1)
    assert list(backend_only.roles) == ["backend"] and backend_only.expected_fleet == ()


def test_unequal_artifacts_are_not_accepted():
    verdict = harness.verdict([_run(), _run()], {**SAME, "gcs": False})
    assert verdict == {"accepted": False, "repeats": False, "outcomes_ok": True, "passed": False}


def test_a_single_run_is_a_negative_control_with_no_repeat_claim():
    assert harness.verdict([_run()], {}) == {
        "accepted": False, "repeats": None, "outcomes_ok": True, "passed": True}
    assert harness.verdict([_run("timeout")], {})["passed"] is False


def test_the_fixture_is_the_pinned_certified_copy():
    pin = json.loads(harness.FLEET_SCENARIO_LOCK.read_text(encoding="utf-8"))
    assert harness.canonical_sha256(harness.FLEET_SCENARIO) == pin["canonical_sha256"]
    source = subprocess.run(["git", "show", f"{pin['source_revision']}:{pin['source']}"], cwd=ROOT,
                            capture_output=True, check=False)
    if source.returncode != 0:
        pytest.skip(f"source revision {pin['source_revision']} is not in this clone")
    canonical = json.dumps(json.loads(source.stdout), sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    assert hashlib.sha256(canonical.encode("ascii")).hexdigest() == pin["canonical_sha256"]
    scenario, loaded_pin = harness.load_fleet_scenario()
    assert loaded_pin == pin and len(scenario.slots) == 3


def test_a_changed_fixture_is_refused(tmp_path, monkeypatch):
    changed = json.loads(harness.FLEET_SCENARIO.read_text(encoding="utf-8"))
    changed["missions"][0]["waypoints"][0]["lat"] += 0.0001
    path = tmp_path / harness.FLEET_SCENARIO.name
    path.write_text(json.dumps(changed), encoding="utf-8")
    monkeypatch.setattr(harness, "FLEET_SCENARIO", path)
    with pytest.raises(ValueError, match="changed"):
        harness.load_fleet_scenario()


class _Stop:
    def __init__(self, error: BaseException | None = None) -> None:
        self.error, self.calls = error, 0

    def __call__(self) -> None:
        self.calls += 1
        if self.error is not None:
            raise self.error


def test_a_clean_teardown_runs_every_stop():
    roster, fleet = _Stop(), _Stop()
    harness.confirm_teardown(40, roster, fleet)
    assert roster.calls == fleet.calls == 1


@pytest.mark.parametrize("failing", ["roster", "fleet"])
def test_a_failed_stop_is_an_unclean_teardown_and_the_others_still_run(failing):
    # The coordinator only logs a failed participant stop: a backend that is not
    # confirmed gone with the SITLs stopped cleanly must still keep the slot.
    stops = {"roster": _Stop(), "fleet": _Stop()}
    stops[failing] = _Stop(OSError("not confirmed gone"))
    with pytest.raises(harness.UncleanTeardown, match="slot 40") as caught:
        harness.confirm_teardown(40, stops["roster"], stops["fleet"])
    assert isinstance(caught.value.__cause__, OSError)
    assert stops["roster"].calls == stops["fleet"].calls == 1


def _main(tmp_path, monkeypatch, run_once) -> tuple[list[int], BaseException | None]:
    """Run ``main`` for delay seeds 0 7 9 on fake slots 40-42; return the released slots."""
    slots, released = iter([40, 41, 42]), []
    monkeypatch.setattr(harness.lockstep, "claim", lambda **kw: (next(slots), {"owner": "test"}))
    monkeypatch.setattr(harness.lockstep, "release_if_owner", lambda slot, owner: released.append(slot))
    monkeypatch.setattr(harness, "run_once", run_once)
    monkeypatch.setattr(sys, "argv", ["eval", "--run-dir", str(tmp_path), "--delay-seeds", "0", "7", "9"])
    try:
        harness.main()
    except Exception as error:  # noqa: BLE001 - returned for the test to inspect
        return released, error
    return released, None


def test_an_unclean_teardown_keeps_its_slot_and_releases_the_rest(tmp_path, monkeypatch):
    def run_once(run_dir, slot, owner, *, seed, delay_seed, navpy):
        if slot == 41:
            raise harness.UncleanTeardown(f"slot {slot}")
        return _run()

    released, error = _main(tmp_path, monkeypatch, run_once)
    assert isinstance(error, harness.UncleanTeardown)
    assert released == [40, 42]  # 41 keeps its ownership records for gcs_stop.py


def test_a_run_fault_after_a_clean_stop_releases_every_slot(tmp_path, monkeypatch):
    def run_once(run_dir, slot, owner, *, seed, delay_seed, navpy):
        raise RuntimeError("fleet backend run fault")

    released, error = _main(tmp_path, monkeypatch, run_once)
    assert isinstance(error, RuntimeError) and not isinstance(error, harness.UncleanTeardown)
    assert released == [40, 41, 42]
