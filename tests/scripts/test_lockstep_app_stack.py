"""Step 11: the app stack stops the frontend's whole tree (shell, npm, Vite)."""

from __future__ import annotations

import signal
import subprocess
from types import SimpleNamespace

import pytest

from scripts import lockstep_app_stack as stack


def _posix(killed: list) -> SimpleNamespace:
    def killpg(pid: int, sig: int) -> None:
        killed.append((pid, sig))

    return SimpleNamespace(name="posix", killpg=killpg, environ={})


def test_on_posix_the_frontend_leads_its_own_group_and_stop_signals_the_group(monkeypatch):
    killed: list = []
    monkeypatch.setattr(stack, "os", _posix(killed))
    assert stack.own_tree() == {"start_new_session": True}
    stack.stop_tree(4242)
    assert killed == [(4242, signal.SIGTERM)]


def test_on_posix_a_group_already_gone_is_stopped(monkeypatch):
    def gone(pid: int, sig: int) -> None:
        raise ProcessLookupError(pid)

    monkeypatch.setattr(stack, "os", SimpleNamespace(name="posix", killpg=gone))
    stack.stop_tree(4242)


@pytest.mark.skipif(not hasattr(subprocess, "CREATE_NO_WINDOW"), reason="Windows only")
def test_on_windows_the_tree_is_killed_with_taskkill(monkeypatch):
    runs: list = []
    monkeypatch.setattr(stack, "os", SimpleNamespace(name="nt"))
    monkeypatch.setattr(stack.subprocess, "run", lambda args, **kwargs: runs.append(args))
    assert stack.own_tree() == {"creationflags": subprocess.CREATE_NO_WINDOW}
    stack.stop_tree(4242)
    assert runs == [["taskkill", "/PID", "4242", "/T", "/F"]]
