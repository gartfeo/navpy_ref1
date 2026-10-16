"""Step 11's matrix gate: every run passes, digests and artifacts are equal."""

import json

import pytest

from scripts import eval_gcs_demo_lockstep_matrix as matrix

DIGEST = "d" * 64


def _write_run(run_dir, *, digest=DIGEST, passed=True, wall_s=100.0, parked_tick=5000, extra=b"x"):
    run_dir.mkdir()
    (run_dir / matrix.DIGEST_FILE).write_text(digest + "\n", encoding="utf-8")
    vehicle = {"sys_id": 1, "role": "owner", "passed": passed, "errors": [], "metrics": {}}
    (run_dir / "report.json").write_text(json.dumps({
        "passed": passed, "errors": [] if passed else ["gate"], "approvals": [{}, {}, {}],
        "global_assignments": {"1": 1}, "vehicles": [vehicle]}), encoding="utf-8")
    (run_dir / matrix.JOURNAL_FILE).write_text(
        json.dumps({"grid": {"drain_period_us": 40000}}) + "\n{}\n", encoding="utf-8")
    (run_dir / matrix.RUN_HOST_FILE).write_text(json.dumps({
        "wall_s": wall_s, "result": {"outcome": "ok", "terminal_tick": 4900,
                                     "final_tick": 4990, "parked_tick": parked_tick}}), encoding="utf-8")
    (run_dir / "artifact.csv").write_bytes(extra)


def _matrix(tmp_path, **changes):
    runs, inventories = {}, {}
    for name in ("a", "b", "c"):
        _write_run(tmp_path / name, **changes.get(name, {}))
        runs[name] = matrix.read_run(tmp_path / name, 0)
        inventories[name] = matrix.inventory(tmp_path / name)
    return runs, inventories


def test_equal_passing_runs_pass_and_cost_uses_the_journal_tick(tmp_path):
    runs, inventories = _matrix(tmp_path)
    assert matrix.verdict(runs, inventories) == []
    # 5000 ticks of the journal's 40 ms = 200 sim s in 100 wall s.
    assert runs["a"]["cost"] == {"ticks": 5000, "wall_s": 100.0, "wall_ms_per_tick": 20.0,
                                 "sim_over_wall": 2.0}


def test_host_only_files_are_not_compared(tmp_path):
    runs, inventories = _matrix(tmp_path)
    (tmp_path / "b" / "x_host.log").write_text("pid 1", encoding="utf-8")
    inventories["b"] = matrix.inventory(tmp_path / "b")
    assert matrix.verdict(runs, inventories) == []


def test_a_different_digest_fails(tmp_path):
    runs, inventories = _matrix(tmp_path, c={"digest": "e" * 64})
    assert any("journal digests differ" in error for error in matrix.verdict(runs, inventories))



def test_runs_that_all_lack_a_digest_fail(tmp_path):
    runs, inventories = _matrix(tmp_path)
    for name in runs:
        runs[name].pop("digest")
    assert any("journal digests differ or are missing" in error
               for error in matrix.verdict(runs, inventories))

def test_a_failed_report_fails(tmp_path):
    runs, inventories = _matrix(tmp_path, b={"passed": False})
    assert any(error.startswith("b: report not passed") for error in matrix.verdict(runs, inventories))


def test_a_nonzero_exit_fails(tmp_path):
    runs, inventories = _matrix(tmp_path)
    runs["c"]["exit_code"] = 1
    assert matrix.verdict(runs, inventories) == ["c: exit code 1"]


def test_a_differing_artifact_fails(tmp_path):
    runs, inventories = _matrix(tmp_path, c={"extra": b"y"})
    assert matrix.verdict(runs, inventories) == ["c: artifacts differ from a: ['artifact.csv']"]


def test_missing_evidence_fails_without_raising(tmp_path):
    runs, inventories = _matrix(tmp_path)
    runs["b"] = matrix.read_run(tmp_path / "missing", 1)
    errors = matrix.verdict(runs, inventories)
    assert "b: exit code 1" in errors
    assert any(error.startswith("b: FileNotFoundError") for error in errors)
    assert any("journal digests differ" in error for error in errors)


def test_a_zero_wall_time_run_is_recorded_as_failed_evidence(tmp_path):
    _write_run(tmp_path / "a", wall_s=0.0)
    run = matrix.read_run(tmp_path / "a", 0)
    assert run["evidence_error"].startswith("ZeroDivisionError")
    assert matrix.verdict({"a": run}, {"a": matrix.inventory(tmp_path / "a")}) \
        == ["a: ZeroDivisionError: float division by zero"]


def test_a_zero_tick_run_is_recorded_as_failed_evidence(tmp_path):
    _write_run(tmp_path / "a", parked_tick=0)
    assert matrix.read_run(tmp_path / "a", 0)["evidence_error"].startswith("ZeroDivisionError")


def test_runs_whose_digest_files_are_all_blank_fail(tmp_path):
    runs, inventories = _matrix(tmp_path, a={"digest": " "}, b={"digest": " "}, c={"digest": " "})
    errors = matrix.verdict(runs, inventories)
    assert all(f"{name}: ValueError: {matrix.DIGEST_FILE} is not a sha256: ''" in errors
               for name in "abc")
    assert any("journal digests differ or are missing" in error for error in errors)


class _Burner:
    def __init__(self, log):
        self.log = log

    def kill(self):
        self.log.append("kill")

    def wait(self):
        self.log.append("wait")


def test_a_failed_burner_start_stops_the_burners_already_running(tmp_path, monkeypatch):
    log, started = [], []

    def popen(_command):
        if len(started) == 2:
            raise OSError("no more processes")
        started.append(_Burner(log))
        return started[-1]

    monkeypatch.setattr(matrix.os, "cpu_count", lambda: 4)
    monkeypatch.setattr(matrix.subprocess, "Popen", popen)
    monkeypatch.setattr(matrix.subprocess, "run", lambda *a, **k: pytest.fail("ran the eval"))
    with pytest.raises(OSError):
        matrix.run_cell(matrix.Cell("cpuload", 0, True), tmp_path)
    assert log == ["kill", "kill", "wait", "wait"]


def test_the_declared_matrix_is_five_seed_one_runs_with_delays_and_load():
    assert len(matrix.CELLS) == 5 and matrix.SEED == 1
    assert len({cell.name for cell in matrix.CELLS}) == 5
    assert sum(not cell.host_delay_seed and not cell.cpu_load for cell in matrix.CELLS) == 2
    assert len({cell.host_delay_seed for cell in matrix.CELLS if cell.host_delay_seed}) == 2
    assert any(cell.cpu_load for cell in matrix.CELLS)
