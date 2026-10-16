"""Step 11's matrix gate: every run passes, digests and artifacts are equal."""

import json

from scripts import eval_gcs_demo_lockstep_matrix as matrix

DIGEST = "d" * 64


def _write_run(run_dir, *, digest=DIGEST, passed=True, wall_s=100.0, extra=b"x"):
    run_dir.mkdir()
    (run_dir / matrix.DIGEST_FILE).write_text(digest + "\n", encoding="utf-8")
    vehicle = {"sys_id": 1, "role": "owner", "passed": passed, "errors": [], "metrics": {}}
    (run_dir / "report.json").write_text(json.dumps({
        "passed": passed, "errors": [] if passed else ["gate"], "approvals": [{}, {}, {}],
        "global_assignments": {"1": 1}, "vehicles": [vehicle]}), encoding="utf-8")
    (run_dir / matrix.JOURNAL_FILE).write_text(
        json.dumps({"grid": {"drain_period_us": 20000}}) + "\n{}\n", encoding="utf-8")
    (run_dir / matrix.RUN_HOST_FILE).write_text(json.dumps({
        "wall_s": wall_s, "result": {"outcome": "ok", "terminal_tick": 4900,
                                     "final_tick": 4990, "parked_tick": 5000}}), encoding="utf-8")
    (run_dir / "artifact.csv").write_bytes(extra)


def _matrix(tmp_path, **changes):
    runs, inventories = {}, {}
    for name in ("a", "b", "c"):
        _write_run(tmp_path / name, **changes.get(name, {}))
        runs[name] = matrix.read_run(tmp_path / name, 0)
        inventories[name] = matrix.without_host_only(matrix._inventory(tmp_path / name))
    return runs, inventories


def test_equal_passing_runs_pass_and_cost_uses_the_journal_tick(tmp_path):
    runs, inventories = _matrix(tmp_path)
    assert matrix.verdict(runs, inventories) == []
    # 5000 ticks of 20 ms = 100 sim s in 100 wall s.
    assert runs["a"]["cost"] == {"ticks": 5000, "wall_s": 100.0, "wall_ms_per_tick": 20.0,
                                 "sim_over_wall": 1.0}


def test_host_only_files_are_not_compared(tmp_path):
    runs, inventories = _matrix(tmp_path)
    (tmp_path / "b" / "x_host.log").write_text("pid 1", encoding="utf-8")
    inventories["b"] = matrix.without_host_only(matrix._inventory(tmp_path / "b"))
    assert matrix.verdict(runs, inventories) == []


def test_a_different_digest_fails(tmp_path):
    runs, inventories = _matrix(tmp_path, c={"digest": "e" * 64})
    assert any("journal digests differ" in error for error in matrix.verdict(runs, inventories))


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


def test_the_declared_matrix_is_five_seed_one_runs_with_delays_and_load():
    assert len(matrix.CELLS) == 5 and matrix.SEED == 1
    assert len({cell.name for cell in matrix.CELLS}) == 5
    assert sum(not cell.host_delay_seed and not cell.cpu_load for cell in matrix.CELLS) == 2
    assert len({cell.host_delay_seed for cell in matrix.CELLS if cell.host_delay_seed}) == 2
    assert any(cell.cpu_load for cell in matrix.CELLS)
