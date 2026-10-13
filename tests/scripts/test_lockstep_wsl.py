"""The WSL side of a lockstep run: commands, environment, ownership records."""

from __future__ import annotations

import json
import shlex

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
