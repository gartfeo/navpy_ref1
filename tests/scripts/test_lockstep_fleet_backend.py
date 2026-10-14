"""The 10e harness's verdict and its fixture pin (``scripts/eval_lockstep_fleet_backend.py``).

A run passes only with the intended outcome (``operator_returned``) and no
fault; acceptance also needs two or more runs with equal artifacts, and a
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

from scripts import eval_lockstep_fleet_backend as harness

ROOT = Path(__file__).resolve().parents[2]
SAME = {"lockstep_journal.jsonl": True, "fleet_actor.json": True, "backend_log_lines.txt": True, "gcs": True}


def _run(outcome: str = "operator_returned", *, fault: str | None = None, fault_records: int = 0) -> dict:
    return {"result": {"outcome": outcome}, "fault": fault, "fault_records": fault_records}


def test_two_equal_runs_with_the_intended_outcome_are_accepted():
    assert harness.verdict([_run(), _run()], SAME) == {
        "accepted": True, "repeats": True, "outcomes_ok": True, "passed": True}


@pytest.mark.parametrize("bad", [_run("timeout"), _run(fault="HostFault: x"), _run(fault_records=1),
                                 {"result": None, "fault": "RuntimeError: y", "fault_records": 1}])
def test_equal_artifacts_do_not_pass_without_the_intended_outcome(bad):
    for runs in ([bad, bad], [_run(), bad]):
        verdict = harness.verdict(runs, SAME)
        assert not verdict["passed"] and not verdict["accepted"] and not verdict["outcomes_ok"]


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
    def run_once(run_dir, slot, owner, *, seed, delay_seed):
        if slot == 41:
            raise harness.UncleanTeardown(f"slot {slot}")
        return _run()

    released, error = _main(tmp_path, monkeypatch, run_once)
    assert isinstance(error, harness.UncleanTeardown)
    assert released == [40, 42]  # 41 keeps its ownership records for gcs_stop.py


def test_a_run_fault_after_a_clean_stop_releases_every_slot(tmp_path, monkeypatch):
    def run_once(run_dir, slot, owner, *, seed, delay_seed):
        raise RuntimeError("fleet backend run fault")

    released, error = _main(tmp_path, monkeypatch, run_once)
    assert isinstance(error, RuntimeError) and not isinstance(error, harness.UncleanTeardown)
    assert released == [40, 41, 42]
