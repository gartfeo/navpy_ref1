"""Step 11: the app stack stops the frontend's whole tree (shell, npm, Vite)."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from scripts import lockstep_app_stack as stack


def test_stop_kills_the_frontend_tree_as_gcs_stop_does(monkeypatch):
    runs: list = []
    monkeypatch.setattr(stack.subprocess, "run", lambda args, **kwargs: runs.append(args))
    stack.stop_tree(4242)
    assert runs == [["taskkill", "/PID", "4242", "/T", "/F"]]


def test_a_posix_host_is_refused_before_anything_starts(monkeypatch, tmp_path):
    # A POSIX signal would reach only the shell, here and in gcs_stop.py.
    monkeypatch.setattr(stack, "os", SimpleNamespace(name="posix"))
    started: list = []
    monkeypatch.setattr(stack, "AppRelay", lambda target: started.append(target))
    with pytest.raises(RuntimeError, match="Windows host"):
        stack.start_app_stack(tmp_path, 43, {}, 4)
    assert started == []
