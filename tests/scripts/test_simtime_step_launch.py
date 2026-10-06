"""A live router must not be confused with ownership of the startup lock."""

from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import simtime_step_launch as launch  # noqa: E402


def test_waits_for_owned_lock_release_without_waiting_for_router(tmp_path, monkeypatch):
    lock = tmp_path / "launch.lock"
    lock.write_text("123 timestamp")
    process = Mock(pid=123)
    process.poll.return_value = None
    sleeps = []
    def release(_duration):
        sleeps.append(True)
        lock.unlink()
    monkeypatch.setattr(launch.time, "sleep", release)
    result = launch.wait_launch_unlock(process, lock)
    assert sleeps == [True]
    assert result["observed_own_lock"] and result["supervisor_pid"] == 123
    process.wait.assert_not_called()


def test_never_changes_another_supervisors_lock(tmp_path):
    lock = tmp_path / "launch.lock"
    lock.write_text("456 timestamp")
    launch.wait_launch_unlock(Mock(pid=123), lock)
    assert lock.read_text() == "456 timestamp"


def test_reports_own_orphan_instead_of_breaking_it(tmp_path):
    lock = tmp_path / "launch.lock"
    lock.write_text("123 timestamp")
    process = Mock(pid=123)
    process.poll.return_value = 1
    with pytest.raises(RuntimeError, match="holding its launch lock"):
        launch.wait_launch_unlock(process, lock)
    assert lock.exists()


def test_release_exit_race_is_not_misreported_as_orphan(tmp_path):
    lock = tmp_path / "launch.lock"
    lock.write_text("123 timestamp")
    process = Mock(pid=123)
    def release_and_exit():
        lock.unlink()
        return 1
    process.poll.side_effect = release_and_exit
    assert launch.wait_launch_unlock(process, lock)["observed_own_lock"]


def test_reject_windows_redirector_before_experiment(monkeypatch):
    monkeypatch.setattr(launch.sys, "platform", "win32")
    monkeypatch.setattr(launch.sys, "prefix", "venv")
    monkeypatch.setattr(launch.sys, "base_prefix", "base")
    with pytest.raises(ValueError, match="redirector"):
        launch.require_base_interpreter()
