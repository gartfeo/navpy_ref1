"""Step 8's seed sweep: seeds, reruns, outcomes, failure classes and reader lag (no runs launched)."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts import eval_seed_sweep as sweep
from scripts.eval_gcs_demo_lockstep import EPOCH_US, JOURNAL_FILE, RUN_HOST_FILE
from scripts.eval_seed_sweep_classes import HOST, PASS, PRODUCT, SETUP, RunVerdict, classify, known_todos
from scripts.eval_seed_sweep_lag import READER_LAG_LIMIT_MS, lagging_readers, reader_lag

JOB = sweep.Job("hw", 7)
D1, D2 = "a" * 64, "b" * 64
FAULT = {"fault": "HostFault", "kind": "exchange_budget", "tick": 565, "ordinal": None}


# --- seeds ---


def test_new_seeds_repeat_for_a_commit_and_change_with_it():
    seeds = sweep.new_seeds("caed2cb4", 10)
    assert seeds == sweep.new_seeds("caed2cb4", 10)
    assert len(set(seeds)) == 10 and all(1 <= s <= sweep.SEED_RANGE for s in seeds)
    assert seeds != sweep.new_seeds("7cd4d202", 10)


def test_new_seeds_skip_a_seed_drawn_twice(monkeypatch):
    monkeypatch.setattr(sweep, "SEED_RANGE", 3)
    assert sorted(sweep.new_seeds("caed2cb4", 3)) == [1, 2, 3]


def test_a_seed_runs_once_and_as_a_regression_seed_if_it_is_one():
    jobs = [sweep.Job("stress", 40, True), sweep.Job("hw", 5), sweep.Job("stress", 40), sweep.Job("hw", 5)]
    assert sweep.unique_jobs(jobs) == [sweep.Job("stress", 40, True), sweep.Job("hw", 5)]


def test_the_regression_list_starts_with_stress_seed_40():
    assert sweep.regression_jobs()[0] == sweep.Job("stress", 40, True)


# --- reruns and outcomes ---


def _settle(monkeypatch, tmp_path, *verdicts: RunVerdict) -> dict:
    """``settle`` over scripted verdicts, one per attempt in order; more attempts fail the test."""
    queue = list(verdicts)
    ran: list[str] = []
    monkeypatch.setattr(sweep, "classify", lambda path, code: queue.pop(0))

    def runner(job, directory: Path) -> int:
        ran.append(directory.name)
        assert queue, f"unexpected attempt {directory.name}"
        return 0

    result = sweep.settle(JOB, tmp_path, runner)
    assert not queue, "an expected attempt did not run"
    assert [a["run"] for a in result["attempts"]] == ran == ["hw-7", "hw-7-r1", "hw-7-r2"][:len(ran)]
    return result


def _v(cls, digest=D1, fault=None, known=(), lagging=()):
    return RunVerdict(cls, digest, fault, ["e"] if cls != PASS else [], known, {}, lagging)


@pytest.mark.parametrize(("verdicts", "status"), [
    ([_v(PASS)], "pass"),
    ([_v(PRODUCT), _v(PRODUCT)], "product"),
    ([_v(PRODUCT, known=("77",)), _v(PRODUCT, known=("77",))], "known"),
    ([_v(PRODUCT), _v(PRODUCT, D2)], "determinism_bug"),
    ([_v(PRODUCT), _v(PASS, D2)], "determinism_bug"),
    ([_v(PRODUCT), _v(HOST, D2, FAULT)], "unconfirmed"),
    ([_v(HOST, D1, FAULT), _v(HOST, D1, FAULT)], "repeated_fault"),
    ([_v(HOST, D1, FAULT), _v(HOST, D2, FAULT)], "host"),
    ([_v(HOST, D1, FAULT), _v(HOST, D1, {**FAULT, "tick": 566})], "host"),
    ([_v(SETUP, None), _v(SETUP, None)], "host"),
    ([_v(HOST, D1, FAULT), _v(PASS, D2)], "pass"),
    ([_v(HOST, D1, FAULT), _v(PRODUCT, D2), _v(PRODUCT, D2)], "product"),
    ([_v(HOST, D1, FAULT), _v(PRODUCT, D2), _v(PRODUCT, D1)], "determinism_bug"),
])
def test_each_failing_seed_is_rerun_and_the_first_record_is_kept(monkeypatch, tmp_path, verdicts, status):
    result = _settle(monkeypatch, tmp_path, *verdicts)
    assert result["status"] == status
    assert result["attempts"][0]["class"] == verdicts[0].cls


def test_a_known_failure_names_its_todos(monkeypatch, tmp_path):
    result = _settle(monkeypatch, tmp_path, _v(PRODUCT, known=("121(a)", "77")), _v(PRODUCT, known=("121(a)", "77")))
    assert result["known"] == ["121(a)", "77"]


def test_a_lagging_reader_in_any_attempt_is_named(monkeypatch, tmp_path):
    result = _settle(monkeypatch, tmp_path, _v(HOST, D1, FAULT), _v(PASS, D2, lagging=("gcs 2",)))
    assert result["lagging_readers"] == ["hw-7-r1: gcs 2"]


def _seed(status, regression=False, lagging=()):
    return {"profile": "hw", "seed": 1, "regression": regression, "status": status, "lagging_readers": list(lagging)}


@pytest.mark.parametrize(("seeds", "code"), [
    ([_seed("pass"), _seed("known")], 0),
    ([_seed("pass"), _seed("product")], 1),
    ([_seed("determinism_bug")], 1),
    ([_seed("repeated_fault")], 1),
    ([_seed("pass", lagging=["hw-1: gcs 1"])], 1),
    ([_seed("known", regression=True)], 1),
    ([_seed("host", regression=True)], 1),
    ([_seed("host"), _seed("pass")], 2),
    ([_seed("unconfirmed")], 2),
    ([_seed("host"), _seed("product")], 1),
])
def test_the_sweep_fails_on_a_new_failure_and_is_incomplete_on_a_host_fault(seeds, code):
    assert sweep.verdict(seeds)[0] == code


# --- failure classes ---


def _run(tmp_path, *, report=None, host=None, journal=(), wind=False, digest=D1) -> Path:
    run = tmp_path / "run"
    run.mkdir()
    (run / "lockstep_journal_digest.txt").write_text(digest + "\n", encoding="utf-8")
    (run / JOURNAL_FILE).write_text("".join(json.dumps(r) + "\n" for r in journal), encoding="utf-8")
    (run / "input_manifest.json").write_text(json.dumps({"run_options": {"sim_profile": {"models": {
        "wind": {"on": wind, "params": {}}}}}}), encoding="utf-8")
    if report is not None:
        (run / "report.json").write_text(json.dumps(report), encoding="utf-8")
    if host is not None:
        (run / RUN_HOST_FILE).write_text(json.dumps(host), encoding="utf-8")
    return run


def _report(errors=(), vehicle_errors=()):
    return {"passed": not errors and not vehicle_errors, "errors": list(errors),
            "vehicles": [{"sys_id": 2, "errors": list(vehicle_errors)}]}


FILTERED = "forwarding: NAVLINK ['sitl3', 1, 'sitl3.serial1', 11318] router_filtered at tick 7779"
FALSE_FRAME = {"k": 7779, "radio": [["false", "sitl3.serial1", ["sitl3", 1, "sitl3.serial1", 11318]]]}


def test_a_passed_report_with_exit_0_is_a_pass_and_any_other_exit_is_not(tmp_path):
    run = _run(tmp_path, report=_report())
    assert classify(run, 0).cls == PASS and classify(run, 0).digest == D1
    assert classify(run, 1).cls == PRODUCT


def test_a_scored_failure_is_a_product_failure_with_its_vehicle_errors(tmp_path):
    verdict = classify(_run(tmp_path, report=_report(["x"], ["roll"])), 1)
    assert verdict.cls == PRODUCT and verdict.errors == ["x", "vehicle 2: roll"] and not verdict.known


def test_a_journaled_participant_fault_without_a_report_is_the_products(tmp_path):
    fault = {"fault": "ParticipantFault", "kind": "unclean_end", "tick": 9, "ordinal": 1}
    verdict = classify(_run(tmp_path, host={"fault": "ParticipantFault: x"}, journal=[fault]), 1)
    assert verdict.cls == PRODUCT and verdict.fault == fault


def test_a_host_record_fault_without_a_report_is_a_host_fault_and_nothing_is_setup(tmp_path):
    run = _run(tmp_path, host={"fault": "HostFault: ran out"}, journal=[FAULT])
    assert classify(run, 2).cls == HOST and classify(run, 2).fault == FAULT
    (run / RUN_HOST_FILE).unlink()
    assert classify(run, 1).cls == SETUP
    assert classify(tmp_path / "never_made", 1).cls == SETUP


def test_a_router_filtered_navlink_is_known_only_when_the_radio_made_the_frame(tmp_path):
    run = _run(tmp_path, report=_report([FILTERED]), journal=[FALSE_FRAME])
    assert list(classify(run, 1).known) == ["121(a)"]
    clean = {"k": 7779, "radio": [["false", "sitl3.serial1", ["sitl3", 1, "sitl3.serial1", 11317]]]}
    assert known_todos(run, [clean], [FILTERED]) == []


def test_a_missed_dock_is_todo_77_only_with_wind_on_and_only_if_every_error_is_explained(tmp_path):
    errors = ["guidance episode reached SNAP without DOCKED", "max roll-command step 61.17deg > 50.00deg"]
    windy = _run(tmp_path, report=_report(["vehicle 2: workflow ticks: no reset marker"], errors), wind=True)
    assert list(classify(windy, 1).known) == ["77"]
    assert known_todos(windy, [], errors + ["approval missing"]) == []
    (windy / "input_manifest.json").write_text(json.dumps({"run_options": {"sim_profile": {"models": {
        "wind": {"on": False, "params": {}}}}}}), encoding="utf-8")
    assert known_todos(windy, [], errors) == []
    assert known_todos(windy, [], ["vehicle 2: workflow ticks: no reset marker"]) == []


def test_a_windy_roll_step_is_todo_77_only_on_a_uav_that_missed(tmp_path):
    """A roll step fails without wind too (todo 229), so wind alone does not explain it."""
    windy = _run(tmp_path, report=_report(), wind=True)
    miss, step = "vehicle 2: workflow ticks: no reset marker", "vehicle {}: max roll-command step 54.90deg > 50.00deg"
    assert known_todos(windy, [], [miss, step.format(2)]) == ["77"]
    assert known_todos(windy, [], [miss, step.format(1)]) == []
    assert known_todos(windy, [], [step.format(2)]) == []


# --- reader lag ---


def _arrivals(run: Path, name: str, rows) -> None:
    directory = run / "source-time"
    directory.mkdir(exist_ok=True)
    (directory / f"pose_cadence_attitude_arrival_uav_{name}.csv").write_text(
        "wall_s,boot_ms\n" + "".join(f"{EPOCH_US / 1e6 + t:.6f},{boot}\n" for t, boot in rows), encoding="utf-8")


def test_reader_lag_is_arrival_less_aircraft_time_above_its_best(tmp_path):
    run = _run(tmp_path, report=_report())
    _arrivals(run, "1_gcs", [(10.0 + i * 0.02, 9_900 + i * 20) for i in range(100)] + [(13.0, 11_900)])
    lag = reader_lag(run)
    assert lag["gcs 1"]["p50"] == 0.0 and lag["gcs 1"]["max"] == pytest.approx(1000.0, abs=0.1)
    assert lagging_readers(lag) == ["gcs 1"] and READER_LAG_LIMIT_MS < 1000.0


def test_a_freeze_the_profile_scheduled_is_not_lag_for_its_navpy_only(tmp_path):
    plan = {"id": "late_heartbeat:uav1", "kind": "late_heartbeat", "target": "uav1"}
    journal = [{"grid": {"faults": [plan]}}, {"k": 1, "t_us": 20_000_000, "faults": [["start", plan["id"]]]},
               {"k": 2, "t_us": 28_000_000, "faults": [["heal", plan["id"], 33_000_000]]}]
    run = _run(tmp_path, report=_report(), journal=journal)
    rows = [(10.0, 9_900), (25.0, 16_000), (40.0, 39_900)]  # 9 s behind inside the window
    _arrivals(run, "1_navpy", rows)
    _arrivals(run, "1_gcs", rows)
    lag = reader_lag(run)
    assert lag["navpy 1"]["max"] == pytest.approx(0.0, abs=0.1)
    assert lag["gcs 1"]["max"] == pytest.approx(8900.0, abs=0.1)


# --- a product fault that repeats, against one an outside cause made (todo 202) ---


def _journal(run: Path, ticks: range, fault: dict | None = None, salt: str = "") -> None:
    run.mkdir(parents=True, exist_ok=True)
    records = [{"grid": {"seed": 7}}] + [{"k": k, "t_us": k * 20_000, "x": f"{salt}{k}"} for k in ticks]
    (run / JOURNAL_FILE).write_text("".join(json.dumps(r) + "\n" for r in records + ([fault] if fault else [])),
                                    encoding="utf-8")


UNCLEAN = {"fault": "ParticipantFault", "kind": "unclean_end", "tick": 50, "ordinal": 1}


def _settle_on_disk(monkeypatch, tmp_path, verdicts, journals) -> dict:
    """``settle`` with scripted verdicts, each attempt's journal written as the runner's run."""
    queue, writes = list(verdicts), list(journals)
    monkeypatch.setattr(sweep, "classify", lambda path, code: queue.pop(0))

    def runner(job, directory: Path) -> int:
        writes.pop(0)(directory)
        return 1

    return sweep.settle(JOB, tmp_path, runner)


def test_an_unflushed_end_that_repeats_with_its_digest_is_the_products(monkeypatch, tmp_path):
    """A deterministic fault: the rerun ends at the same tick with the same digest."""
    first = _v(PRODUCT, D1, UNCLEAN)
    result = _settle_on_disk(monkeypatch, tmp_path, [first, _v(PRODUCT, D1, UNCLEAN)],
                             [lambda d: _journal(d, range(1, 51), UNCLEAN)] * 2)
    assert result["status"] == "product"


@pytest.mark.parametrize("kind", ["unclean_end", "stop_budget"])
def test_an_unflushed_end_the_replay_does_not_repeat_is_reported_apart(monkeypatch, tmp_path, kind):
    """Load: the rerun replays every record before the fault tick unchanged and runs on past it, so an
    outside cause is likely (a process killed, a write the OS refused), though not proven."""
    fault = {**UNCLEAN, "kind": kind}
    result = _settle_on_disk(monkeypatch, tmp_path, [_v(PRODUCT, D1, fault), _v(PASS, D2)],
                             [lambda d: _journal(d, range(1, 51), fault), lambda d: _journal(d, range(1, 400))])
    assert result["status"] == "not_repeated"
    assert sweep.verdict([result])[0] == 2  # incomplete, not failed: load alone fails no sweep


def test_any_other_participant_fault_the_replay_does_not_repeat_is_a_determinism_bug(monkeypatch, tmp_path):
    exit_ = {**UNCLEAN, "kind": "unscheduled_exit"}
    result = _settle_on_disk(monkeypatch, tmp_path, [_v(PRODUCT, D1, exit_), _v(PASS, D2)],
                             [lambda d: _journal(d, range(1, 51), exit_), lambda d: _journal(d, range(1, 400))])
    assert result["status"] == "determinism_bug"


def test_a_rerun_that_faults_at_the_same_tick_otherwise_or_diverges_first_is_a_determinism_bug(monkeypatch, tmp_path):
    other = {**UNCLEAN, "kind": "stop_budget"}
    same_tick = _settle_on_disk(monkeypatch, tmp_path / "a", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2, other)],
                                [lambda d: _journal(d, range(1, 51), UNCLEAN),
                                 lambda d: _journal(d, range(1, 51), other)])
    diverged = _settle_on_disk(monkeypatch, tmp_path / "b", [_v(PRODUCT, D1, UNCLEAN), _v(PASS, D2)],
                               [lambda d: _journal(d, range(1, 51), UNCLEAN),
                                lambda d: _journal(d, range(1, 400), salt="y")])
    short = _settle_on_disk(monkeypatch, tmp_path / "c", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2)],
                            [lambda d: _journal(d, range(1, 51), UNCLEAN), lambda d: _journal(d, range(1, 30))])
    stopped = _settle_on_disk(monkeypatch, tmp_path / "d", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2)],
                              [lambda d: _journal(d, range(1, 51), UNCLEAN), lambda d: _journal(d, range(1, 50))])
    at_tick = _settle_on_disk(monkeypatch, tmp_path / "e", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2)],
                              [lambda d: _journal(d, range(1, 51), UNCLEAN), lambda d: _journal(d, range(1, 51))])
    assert [r["status"] for r in (same_tick, diverged, short, stopped, at_tick)] == ["determinism_bug"] * 5


def test_a_rerun_past_an_outside_end_that_still_fails_is_judged_as_a_first_run(monkeypatch, tmp_path):
    """The end hid nothing: a rerun that replays past it and fails a gate is confirmed by a third run."""
    def journals(third):
        return [lambda d: _journal(d, range(1, 51), UNCLEAN), lambda d: _journal(d, range(1, 400)), third]
    confirmed = _settle_on_disk(monkeypatch, tmp_path / "a", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2),
                                                              _v(PRODUCT, D2)], journals(lambda d: _journal(d, range(1, 400))))
    differs = _settle_on_disk(monkeypatch, tmp_path / "b", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2),
                                                            _v(PASS, D1)], journals(lambda d: _journal(d, range(1, 400))))
    assert [r["status"] for r in (confirmed, differs)] == ["product", "determinism_bug"]
    assert [a["run"] for a in confirmed["attempts"]] == ["hw-7", "hw-7-r1", "hw-7-r2"]
    assert sweep.verdict([confirmed])[0] == 1
    later = {**UNCLEAN, "tick": 80}
    loaded = _settle_on_disk(monkeypatch, tmp_path / "c", [_v(PRODUCT, D1, UNCLEAN), _v(PRODUCT, D2, later),
                                                           _v(PRODUCT, "c" * 64)],
                             [lambda d: _journal(d, range(1, 51), UNCLEAN), lambda d: _journal(d, range(1, 81), later),
                              lambda d: _journal(d, range(1, 400))])
    assert loaded["status"] == "unconfirmed" and len(loaded["attempts"]) == 3  # two outside ends: stop


def test_only_a_participant_fault_the_replay_does_not_repeat_is_the_hosts(monkeypatch, tmp_path):
    """A scored failure whose journal ends on another fault class is no unflushed end: a rerun that
    replays past it differs, so it is a determinism bug."""
    coordinator = {**UNCLEAN, "fault": "CoordinatorFault"}
    result = _settle_on_disk(monkeypatch, tmp_path, [_v(PRODUCT, D1, coordinator), _v(PASS, D2)],
                             [lambda d: _journal(d, range(1, 51), coordinator), lambda d: _journal(d, range(1, 400))])
    assert result["status"] == "determinism_bug"


def test_a_scored_failure_with_no_fault_whose_rerun_differs_is_a_determinism_bug(monkeypatch, tmp_path):
    result = _settle_on_disk(monkeypatch, tmp_path, [_v(PRODUCT, D1), _v(PASS, D2)],
                             [lambda d: _journal(d, range(1, 51)), lambda d: _journal(d, range(1, 51))])
    assert result["status"] == "determinism_bug"
