"""Evidence ownership at the ideal evaluator CLI boundary; no stack launches."""
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
import json
from types import SimpleNamespace

import pytest

from scripts import eval_ideal_three_uav as cli
from scripts import eval_ideal_three_uav_runtime as runtime


@pytest.fixture(autouse=True)
def forbid_real_execution(monkeypatch):
    def forbidden(*_args, **_kwargs):
        raise AssertionError("unit test must not start a stack or take a real lock")
    monkeypatch.setattr(cli, "exclusive_evaluator_lock", forbidden)
    monkeypatch.setattr(cli, "execute_ideal_live_run", forbidden)
    monkeypatch.setattr(runtime, "with_owned_stack", forbidden)


@pytest.mark.parametrize("artifact", [None, "sentinel.bin", "final.json"])
def test_existing_run_is_untouched(monkeypatch, tmp_path, artifact):
    run = tmp_path / "run"
    run.mkdir()
    if artifact:
        (run / artifact).write_bytes(b'{"status":"passed","run":"first"}')
    before = {p.name: p.read_bytes() for p in run.iterdir()}
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {})

    @contextmanager
    def forbidden_lock():
        raise AssertionError("existing run must be rejected before taking the lock")
        yield

    monkeypatch.setattr(cli, "exclusive_evaluator_lock", forbidden_lock)
    assert cli.main(["--run-dir", str(run)]) == 2
    assert {p.name: p.read_bytes() for p in run.iterdir()} == before


def test_existing_verdict_is_preserved_before_runtime_entry(monkeypatch, tmp_path):
    verdict = tmp_path / "final.json"
    original = b'{"status":"passed","run":"first"}'
    verdict.write_bytes(original)
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {})
    monkeypatch.setattr(cli, "exclusive_evaluator_lock", nullcontext)
    monkeypatch.setattr(cli, "execute_ideal_live_run", runtime.execute_ideal_live_run)
    assert cli.main(["--run-dir", str(tmp_path)]) == 2
    assert verdict.read_bytes() == original


def test_runtime_requires_a_claimed_directory(tmp_path):
    with pytest.raises(runtime.RegressionError, match="requires a caller-owned directory"):
        runtime.execute_ideal_live_run(tmp_path / "missing", 30.0)
    assert not (tmp_path / "missing").exists()


def test_existing_file_is_untouched(monkeypatch, tmp_path):
    run = tmp_path / "file"
    run.write_bytes(b"keep")
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {})
    assert cli.main(["--run-dir", str(run)]) == 2
    assert run.read_bytes() == b"keep"


def test_invalid_parent_reports_claim_failure(monkeypatch, tmp_path, capsys):
    parent = tmp_path / "file"
    parent.write_bytes(b"keep")
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {})
    assert cli.main(["--run-dir", str(parent / "run")]) == 2
    output = capsys.readouterr().out
    assert '"status": "failed"' in output
    assert '"setup_error":' in output
    assert parent.read_bytes() == b"keep"


def test_final_writer_never_replaces_a_verdict(tmp_path):
    verdict = tmp_path / "final.json"
    verdict.write_bytes(b'{"status":"passed"}')
    with pytest.raises(FileExistsError):
        cli._write_final(tmp_path, {"status": "failed"})
    assert verdict.read_bytes() == b'{"status":"passed"}'


@pytest.mark.parametrize("failure_stage", ["lock", "runtime"])
def test_new_run_records_its_setup_failure(monkeypatch, tmp_path, failure_stage):
    run = tmp_path / "run"
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {"commit": "fixture"})

    @contextmanager
    def lock():
        assert run.is_dir()
        if failure_stage == "lock":
            raise RuntimeError("lock fixture")
        yield

    def execute(*_):
        assert run.is_dir()
        raise RuntimeError("runtime fixture")

    monkeypatch.setattr(cli, "exclusive_evaluator_lock", lock)
    monkeypatch.setattr(cli, "execute_ideal_live_run", execute)
    assert cli.main(["--run-dir", str(run)]) == 2
    result = json.loads((run / "final.json").read_text())
    assert result["setup_error"] == f"RuntimeError: {failure_stage} fixture"
    assert result["status"] == "failed"
    assert result["candidate"] == {"commit": "fixture"}


@pytest.mark.parametrize("collision", [False, True])
def test_new_run_records_success(monkeypatch, tmp_path, capsys, collision):
    run = tmp_path / "run"
    calls = []
    monkeypatch.setattr(cli, "_candidate_identity", lambda: {})

    @contextmanager
    def lock():
        assert run.is_dir()
        calls.append("lock")
        yield

    def execute(directory, _timeout):
        assert directory == run and run.is_dir()
        calls.append("execute")
        if collision:
            (run / "final.json").write_bytes(b"another verdict")
        return SimpleNamespace(sys_ids=(4, 5, 6), approvals=[])

    @dataclass
    class Report:
        passed: bool = True

    monkeypatch.setattr(cli, "exclusive_evaluator_lock", lock)
    monkeypatch.setattr(cli, "execute_ideal_live_run", execute)
    monkeypatch.setattr(cli, "analyze_ideal_run", lambda *_args, **_kwargs: Report())
    assert cli.main(["--run-dir", str(run)]) == (2 if collision else 0)
    assert calls == ["lock", "execute"]
    output = capsys.readouterr()
    assert '"status": "passed"' in output.out
    if collision:
        assert (run / "final.json").read_bytes() == b"another verdict"
        assert "Could not write final verdict" in output.err
    else:
        assert json.loads((run / "final.json").read_text())["status"] == "passed"
