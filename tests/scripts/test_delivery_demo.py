"""The hand-driven delivery demo command and the eval code it reuses."""

from __future__ import annotations

import datetime as dt
import importlib
import json
import sys
from types import SimpleNamespace

import pytest

from scripts import delivery_demo as demo
from scripts.eval_gcs_demo_execution import SystemClock

config = importlib.import_module("scripts.eval_gcs_demo_config")
process = importlib.import_module("scripts.eval_gcs_demo_process")
models = importlib.import_module("scripts.eval_gcs_demo_models")
fixture_module = importlib.import_module("scripts.eval_delivery_fixture")

FIXTURE = fixture_module.load_fixture(fixture_module.FIXTURE_PATH)
SCENARIO = fixture_module.delivery_scenario(FIXTURE, fixture_module.FIXTURE_PATH)
NOW = dt.datetime(2026, 10, 19, 12, 0, 0)


class FakeApi:
    def __init__(self, settings=None, vehicles=()):
        self.settings = settings
        self.vehicles = list(vehicles)
        self.closed = False

    def get_json(self, path):
        return {"/api/settings": self.settings, "/api/vehicles": self.vehicles}[path]

    def close(self):
        self.closed = True


def _launch(monkeypatch, tmp_path, calls):
    sitl_root = SimpleNamespace(launch_arguments=lambda: ["--sitl-root", "r"])
    monkeypatch.setattr(demo, "prepare_sitl_root", lambda _d: (sitl_root, {"root": "r"}, "r"))

    def launch_stack(env, timeout_s, arguments, **kwargs):
        record = json.loads((tmp_path / "20261019-120000" / demo.LAUNCH_RECORD_NAME).read_text())
        calls.append((env, arguments, kwargs, record))
        return SimpleNamespace(api=FakeApi())

    monkeypatch.setattr(demo, "launch_stack", launch_stack)
    return demo.launch(tmp_path, SystemClock(), NOW)


def test_settings_carry_the_fixture_hubs_and_the_explicit_lifecycle():
    settings = demo.demo_settings(FIXTURE, SCENARIO)

    assert settings["default_delivery_hubs"] == FIXTURE["inputs"]["delivery_hubs"]
    assert len(settings["default_delivery_hubs"]) == 3
    assert settings["simulation"]["companion_lifecycle"] == "explicit"
    assert settings["connection"]["auto_connect"] is False


def test_environment_keeps_the_frontend_map_token_and_drops_other_shell_variables(tmp_path):
    env = demo.demo_environment(tmp_path, {"VITE_CESIUM_ION_TOKEN": "t", "NAVPY_X": "1", "PATH": "p"})

    assert env["VITE_CESIUM_ION_TOKEN"] == "t" and env["PATH"] == "p"
    assert "NAVPY_X" not in env
    assert "VITE_CESIUM_ION_TOKEN" not in demo.demo_environment(tmp_path, {"PATH": "p"})


def test_printed_commands_survive_spaces_in_powershell():
    assert demo.powershell_command(r"C:\Program Files\py.exe", r"C: $b`'c\d.py", step="prep") == (
        r"& 'C:\Program Files\py.exe' 'C: $b`''c\d.py' prep"
    )


def test_launch_starts_the_fixture_stack_with_frontend_and_no_browser(monkeypatch, tmp_path):
    calls = []
    run_dir = _launch(monkeypatch, tmp_path, calls)

    env, arguments, kwargs, record = calls[0]
    assert env[demo.FLEET_DOCKS_ENV] == str((run_dir / fixture_module.DOCK_FILE_NAME).resolve())
    assert arguments == ["--sitl-root", "r"]
    assert (kwargs["headless"], kwargs["browser"]) == (False, False)
    # The record exists before the stack starts, under the token it is given.
    assert kwargs["launch_token"] == record["launch_token"]
    assert record["home"] == SCENARIO.home
    assert record["settings_sha256"] == demo.file_sha256(run_dir / demo.SETTINGS_NAME)


def test_launch_refuses_a_running_stack(monkeypatch, tmp_path):
    def busy(*_args, **_kwargs):
        raise process.PreexistingOwnedStackError("exists")

    monkeypatch.setattr(demo, "prepare_sitl_root", lambda _d: (SimpleNamespace(launch_arguments=list), {}, ""))
    monkeypatch.setattr(demo, "launch_stack", busy)

    with pytest.raises(models.RegressionError, match="gcs_stop.py"):
        demo.launch(tmp_path, SystemClock(), NOW)


def test_prep_refuses_without_a_stack_or_with_a_foreign_one(monkeypatch, tmp_path):
    monkeypatch.setattr(demo, "owned_entry", lambda _env: None)
    with pytest.raises(models.RegressionError, match="no GCS stack"):
        demo.prep(tmp_path, SystemClock())

    monkeypatch.setattr(demo, "owned_entry", lambda _env: {"sitl_launch_token": "other"})
    with pytest.raises(models.RegressionError, match="not started by"):
        demo.prep(tmp_path, SystemClock())


def test_prep_refuses_files_changed_since_launch(monkeypatch, tmp_path):
    run_dir = _launch(monkeypatch, tmp_path, [])
    demo.require_unchanged(run_dir)

    (run_dir / demo.SETTINGS_NAME).write_text("{}", encoding="utf-8")
    with pytest.raises(models.RegressionError, match="changed since launch"):
        demo.require_unchanged(run_dir)


def test_prep_refuses_backend_settings_that_differ():
    expected = demo.demo_settings(FIXTURE, SCENARIO)
    actual = json.loads(json.dumps(expected))
    actual["unrelated"] = 1
    demo.require_backend_settings(SimpleNamespace(api=FakeApi(actual)), expected)

    actual["simulation"]["companion_lifecycle"] = "auto"
    actual["default_delivery_hubs"] = []
    with pytest.raises(models.RegressionError, match="simulation.companion_lifecycle, default_delivery_hubs"):
        demo.require_backend_settings(SimpleNamespace(api=FakeApi(actual)), expected)


def test_prep_refuses_a_stack_prep_already_ran_on(monkeypatch, tmp_path):
    monkeypatch.setattr(demo, "companion_ids", lambda _c: ())
    with pytest.raises(models.RegressionError, match="already ran"):
        demo.prepare_scenario(SimpleNamespace(api=FakeApi(vehicles=[{"sys_id": 1}])), tmp_path)

    monkeypatch.setattr(demo, "companion_ids", lambda _c: (1,))
    with pytest.raises(models.RegressionError, match="already ran"):
        demo.prepare_scenario(SimpleNamespace(api=FakeApi()), tmp_path)


def test_prep_runs_the_eval_steps_in_order_then_opens_the_browser(monkeypatch, tmp_path):
    run_dir = _launch(monkeypatch, tmp_path, [])
    token = json.loads((run_dir / demo.LAUNCH_RECORD_NAME).read_text())["launch_token"]
    monkeypatch.setattr(demo, "owned_entry", lambda _env: {"sitl_launch_token": token, "frontend": 3100})
    api = FakeApi(demo.demo_settings(FIXTURE, SCENARIO))
    opened = []

    def owned_stack_context(env, entry, launch_token, backend_log, navigation_dir, **_kwargs):
        assert launch_token == token and navigation_dir == run_dir
        assert backend_log.name == "backend.log"
        return SimpleNamespace(api=api)

    monkeypatch.setattr(demo, "owned_stack_context", owned_stack_context)
    monkeypatch.setattr(demo, "companion_ids", lambda _c: ())
    calls = []
    steps = (
        "connect_vehicles", "wait_for_vehicles", "configure_vehicle_params",
        "upload_scenario_missions", "verify_aas_profile", "start_companions",
        "start_esp32_simulator", "wait_for_missions", "resolve_live_plan",
    )
    for name in steps:
        monkeypatch.setattr(demo, name, lambda *_a, _name=name: calls.append(_name))

    def open_browser(url):
        assert api.closed
        opened.append(url)

    assert demo.prep(tmp_path, SystemClock(), open_browser) == run_dir
    assert tuple(calls) == steps
    assert opened == ["http://localhost:3100"]


def test_launch_command_keeps_the_eval_default_and_maps_the_demo_keywords():
    assert config.launch_command() == [sys.executable, str(config.GCS_LAUNCH), "--headless"]
    demo_argv = config.launch_command(headless=False, browser=False)
    assert "--headless" not in demo_argv and "--no-browser" in demo_argv
    assert "--no-browser" not in config.launch_command(headless=False)


def _owned_context(monkeypatch, tmp_path, entries):
    health = [{"status": "starting"}, {"status": "ok"}]
    api = SimpleNamespace(get_json=lambda _p: health.pop(0), closed=False, health=health)
    api.close = lambda: setattr(api, "closed", True)
    monkeypatch.setattr(process, "GcsApi", lambda _port: api)
    monkeypatch.setattr(process, "flown_sys_ids", lambda _entry, _scenario: (1, 2, 3))
    monkeypatch.setattr(process, "_verified_launch_entry", lambda _env, _token: entries.pop(0))
    entry = {"chat_index": 0, "backend": 8000, "sitl_speedup": 10.0, "sitl_pid": 5, "sitl_pid_start": 1.0}
    call = lambda: process.owned_stack_context(  # noqa: E731
        {}, entry, "ours", tmp_path / "backend.log", tmp_path, clock=SystemClock(), scenario=SCENARIO
    )
    return api, entry, call


def test_owned_stack_context_waits_for_health_and_rechecks_the_generation(monkeypatch, tmp_path):
    entry_after = {"sitl_speedup": 10.0, "sitl_pid": 6, "sitl_pid_start": 2.0}
    api, _entry, call = _owned_context(monkeypatch, tmp_path, [entry_after])

    context = call()

    assert context.api is api and not api.closed and api.health == []
    assert context.launch_evidence["sitl_pid"] == 6 and context.launch_evidence["launch_token"] == "ours"


def test_owned_stack_context_closes_the_api_when_the_generation_changes(monkeypatch, tmp_path):
    api, _entry, call = _owned_context(monkeypatch, tmp_path, [None])

    with pytest.raises(models.RegressionError, match="generation changed"):
        call()
    assert api.closed
