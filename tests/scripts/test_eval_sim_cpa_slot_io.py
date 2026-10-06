"""WSL slot IO primitives, with the process runner faked out."""

from __future__ import annotations

from types import SimpleNamespace

from scripts import eval_sim_cpa_slot_io as slot_io


def test_wait_for_sitl_exit_reads_pgrep_exit_codes(monkeypatch) -> None:
    """pgrep exit 1 is the success condition (no matches); exit 0 keeps
    polling; anything else is a probe failure, never silently 'gone'."""
    calls = {"n": 0}

    def _fake_run(argv, capture_output, text, timeout):
        calls["n"] += 1
        code = 0 if calls["n"] < 3 else 1
        return SimpleNamespace(returncode=code, stdout="123 arduplane",
                               stderr="")

    monkeypatch.setattr(slot_io.subprocess, "run", _fake_run)
    monkeypatch.setattr(slot_io, "_wsl", lambda: "wsl.exe")
    exited, error = slot_io.wait_for_sitl_exit(121, timeout_s=5.0)
    assert exited is True and error == ""
    assert calls["n"] == 3

    def _broken_run(argv, capture_output, text, timeout):
        return SimpleNamespace(returncode=2, stdout="", stderr="bad pattern")

    monkeypatch.setattr(slot_io.subprocess, "run", _broken_run)
    exited, error = slot_io.wait_for_sitl_exit(121, timeout_s=5.0)
    assert exited is False and "pgrep exited 2" in error
