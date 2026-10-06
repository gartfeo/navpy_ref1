"""A failed evaluation may never tear down a supervisor it did not start."""

from types import SimpleNamespace

import pytest

from scripts import eval_simtime_navigation as evaluator


def test_prepare_failure_does_not_stop_existing_stack(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(evaluator, 'ROOT', tmp_path)
    monkeypatch.setattr(evaluator.swarm_run, "_resolve_chat", lambda *a, **k: 40)
    def artifacts(operation, *args):
        if operation == "prepare":
            raise RuntimeError("instance still has process 123")
        return {}
    monkeypatch.setattr(evaluator, "artifacts", artifacts)
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout="", stderr="", check_returncode=lambda: None)
    monkeypatch.setattr(evaluator.subprocess, "run", run)
    args = SimpleNamespace(speedup=10, delayed=False, camera_hz=40,
                           fault="none", firmware_root="/isolated")
    with pytest.raises(RuntimeError, match="instance still has process"):
        evaluator.run_case(args, tmp_path / "case")
    assert calls == [], "preparation failure stopped a stack owned by another run"


def test_solo_and_fleet_share_exclusive_evaluation_lock(monkeypatch, tmp_path):
    from scripts.eval_gcs_demo_execution import exclusive_evaluator_lock
    from scripts.eval_gcs_demo_models import RegressionError
    monkeypatch.setattr(evaluator, 'ROOT', tmp_path)
    monkeypatch.setattr(evaluator, '_run_case', lambda *a: pytest.fail('second evaluator entered'))
    with exclusive_evaluator_lock(tmp_path / '.sitl-runs/lockstep-evaluator.lock'):
        with pytest.raises(RegressionError, match='already running'):
            evaluator.run_case(SimpleNamespace(), tmp_path / 'case')


def test_foreign_supervisor_is_never_stopped(monkeypatch, tmp_path):
    from scripts import lockstep_evaluation_lifecycle as lifecycle
    monkeypatch.setattr(lifecycle.registry, "get", lambda chat: {"sitl_pid": 999})
    monkeypatch.setattr(lifecycle.subprocess, "run", lambda *a, **k: pytest.fail("foreign stop"))
    with pytest.raises(RuntimeError, match="another supervisor"):
        lifecycle.stop_owned_supervisor(tmp_path, 40, SimpleNamespace(pid=123), tmp_path / "log")


def test_all_three_completions_required():
    from scripts.lockstep_evaluation_lifecycle import completion_count
    line = "NAVPY_NAVIGATION_COMPLETE count=3000\n"
    assert not completion_count(line, 3)
    assert not completion_count(line * 2, 3)
    assert completion_count(line * 3, 3)
    with pytest.raises(RuntimeError, match="extra"):
        completion_count(line * 4, 3)
    with pytest.raises(RuntimeError, match="invalidated"):
        completion_count(line * 3 + "NAVPY_NAVIGATION_INVALID late", 3)
