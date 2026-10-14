"""The WSL side of a lockstep run: commands, environment, ownership records."""

from __future__ import annotations

import json
import shlex
import subprocess

import pytest

from scripts import lockstep_wsl


def test_the_sitl_runs_serial_step_on_both_ports_with_the_pinned_start_time():
    command = shlex.split(lockstep_wsl.sitl_command("/r", "40.1,44.2,500,0", 120, 1))
    assert command[:3] == ["setarch", "-R", "/r/bin/arduplane"]
    assert "--serial0=sim:navpystep" in command and "--serial1=sim:navpystep" in command
    assert "--serial2=none" in command
    assert command[command.index("--speedup") + 1] == "1"
    assert command[command.index("--start-time") + 1] == "1767225600"
    assert command[-1] == "-I120"


def test_the_sitl_environment_holds_only_serial_step_settings():
    env = shlex.split(lockstep_wsl.sitl_environment(5, 30_000_000, 0))
    assert env[:2] == ["env", "-i"]
    assert dict(item.split("=", 1) for item in env[2:]) == {
        "PATH": "/usr/bin:/bin",
        "NAVPY_SERIAL_STEP": "1",
        "NAVPY_STEP_BOOT_ID": "5",
        "NAVPY_STEP_DEADLINE_US": "30000000",
    }
    assert "NAVPY_STEP_HELPER_JITTER_SEED=9" in lockstep_wsl.sitl_environment(5, 1, 9)


def test_lockstep_defaults_turn_off_the_logger_and_terrain_helpers():
    overrides = lockstep_wsl.LOCKSTEP_DEFAULT_OVERRIDES
    assert overrides["LOG_BACKEND_TYPE"] == 0.0
    assert overrides["TERRAIN_ENABLE"] == 0.0
    assert overrides["SIM_BAUDLIMIT_EN"] == 0.0


def test_the_firmware_lock_pins_a_binary():
    lock = lockstep_wsl.load_firmware_lock()
    assert len(lock["binary_sha256"]) == 64 and lock["branch"] == "feat/navpy-serial-step"


def test_stop_refuses_a_record_outside_the_run_root(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    record.write_text(json.dumps([
        {"role": "sitl", "wsl_pid": 7, "start_ticks": 1, "cwd": "/home/x/other/1"},
    ]))
    monkeypatch.setattr(lockstep_wsl, "wsl", lambda *_a, **_k: pytest.fail("must not kill"))
    with pytest.raises(RuntimeError, match="outside the run root"):
        lockstep_wsl.stop_owned(record, "/home/x/navpy-eval-roots/lockstep-a")


def test_stop_goes_through_the_pidfd_helper_with_the_recorded_identity(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    root = "/home/x/navpy-eval-roots/lockstep-a"
    record.write_text(json.dumps([
        {"role": "relay", "wsl_pid": 41, "start_ticks": 1234, "cwd": f"{root}/120"},
    ]))
    calls = []
    monkeypatch.setattr(lockstep_wsl, "_proc", lambda *args: calls.append(args) or "stopped")
    assert lockstep_wsl.stop_owned(record, root) == ["relay 41: stopped"]
    assert calls == [("stop", 41, 1234, f"{root}/120")]


def test_the_proc_helper_reads_start_ticks_past_a_comm_with_spaces(tmp_path, monkeypatch):
    from scripts import lockstep_wsl_proc as proc

    stat = "41 (my (odd) name) S 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 777 19 20"
    monkeypatch.setattr("builtins.open", lambda *_a, **_k: __import__("io").StringIO(stat))
    assert proc.start_ticks(41) == "777"


class _HeldWslChild:
    """A double of the ``wsl.exe`` launcher: reports its pid, then waits for ``go`` or EOF."""

    instances: list = []

    def __init__(self, *_args, **_kwargs) -> None:
        import io

        self.stdout = io.BytesIO(b"LOCKSTEP_PID 321\n")
        self.written = bytearray()
        self.closed = False
        outer = self

        class Stdin:
            def write(self, data: bytes) -> None:
                outer.written += data

            def flush(self) -> None:
                pass

            def close(self) -> None:
                outer.closed = True  # EOF: the launcher's bash kills the held child

        self.stdin = Stdin()
        _HeldWslChild.instances.append(self)

    def wait(self, timeout=None) -> int:
        return 137

    @property
    def child_runs(self) -> bool:
        return b"go\n" in self.written


def _fake_wsl(monkeypatch):
    _HeldWslChild.instances = []
    monkeypatch.setattr(lockstep_wsl.subprocess, "Popen", _HeldWslChild)
    monkeypatch.setattr(lockstep_wsl, "_proc", lambda *args: "777 /root/lockstep-a/0")


def test_spawn_admits_before_go_and_a_refusal_never_sends_go(tmp_path, monkeypatch):
    _fake_wsl(monkeypatch)
    seen = []

    def refuse(owned):
        seen.append(owned)
        assert not _HeldWslChild.instances[0].written  # admitted after recording, before go
        return False

    with pytest.raises(RuntimeError, match="stopping"):
        lockstep_wsl.spawn("sitl", "x", "/root/lockstep-a/0", tmp_path / "p.json", admit=refuse)
    (child,) = _HeldWslChild.instances
    assert not child.child_runs and child.closed  # killed unstarted
    assert seen[0].wsl_pid == 321 and seen[0].start_ticks == 777


def test_a_stop_racing_record_and_go_leaves_no_running_child(tmp_path, monkeypatch):
    """gcs_stop marks the slot stopping between the run record and go."""
    _fake_wsl(monkeypatch)
    slot = {"phase": "active"}

    def admit(_owned):
        slot["phase"] = "stopping"  # the stop lands in the interval after recording
        return slot["phase"] == "active"

    with pytest.raises(RuntimeError):
        lockstep_wsl.spawn("relay", "x", "/root/lockstep-a/0", tmp_path / "p.json", admit=admit)
    assert not _HeldWslChild.instances[0].child_runs
    _HeldWslChild.instances = []
    process, owned = lockstep_wsl.spawn("relay", "x", "/root/lockstep-a/0", tmp_path / "q.json",
                                        admit=lambda _owned: True)
    assert process.child_runs and owned.wsl_pid == 321


def test_stop_owned_tries_every_record_when_one_stop_fails(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    root = "/home/x/navpy-eval-roots/lockstep-a"
    record.write_text(json.dumps([
        {"role": "relay", "wsl_pid": 41, "start_ticks": 1, "cwd": f"{root}/120"},
        {"role": "sitl", "wsl_pid": 42, "start_ticks": 2, "cwd": f"{root}/120"},
    ]))
    calls = []

    def proc(*args):
        calls.append(args[1])
        if args[1] == 41:
            raise subprocess.TimeoutExpired("wsl.exe", 60)
        return "stopped"

    monkeypatch.setattr(lockstep_wsl, "_proc", proc)
    with pytest.raises(RuntimeError, match=r"relay 41: TimeoutExpired.*sitl 42: stopped"):
        lockstep_wsl.stop_owned(record, root)
    assert calls == [41, 42]  # the SITL was still stopped
