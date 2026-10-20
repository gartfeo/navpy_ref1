"""The hand-driven delivery demo command, its plan file and the eval code it reuses."""

from __future__ import annotations

import datetime as dt
import importlib
import json
import sys
from types import SimpleNamespace

import pytest

from scripts import delivery_demo as demo
from scripts import delivery_demo_plan as plan
from scripts.eval_gcs_demo_execution import SystemClock

config = importlib.import_module("scripts.eval_gcs_demo_config")
process = importlib.import_module("scripts.eval_gcs_demo_process")
models = importlib.import_module("scripts.eval_gcs_demo_models")
fixture_module = importlib.import_module("scripts.eval_delivery_fixture")

FIXTURE = fixture_module.load_fixture(fixture_module.FIXTURE_PATH)
SCENARIO = fixture_module.delivery_scenario(FIXTURE, fixture_module.FIXTURE_PATH)
NOW = dt.datetime(2026, 10, 20, 12, 0, 0)
OWNER = {
    "connection": {"auto_connect": False, "telemetry_rate_hz": 5},
    "simulation": {"companion_lifecycle": "explicit", "dev_mode": True},
    "launch": {"launch_type": "bungee", "esp32_host": "192.168.4.1", "arm_timeout_s": 10.0},
    "camera": {"vision_profile": None},
    "default_delivery_hubs": [{"lat": 1.0, "lon": 2.0}],
}


def test_settings_are_the_owners_with_the_scenario_pins_on_top():
    settings = demo.demo_settings(OWNER, FIXTURE)

    # The normal lifecycle, whatever the owner's file says.
    assert settings["connection"] == {"auto_connect": True, "telemetry_rate_hz": 5}
    assert settings["simulation"] == {
        "companion_lifecycle": "auto", "dev_mode": True,
        "sim_mode": True, "simulated_vehicle_count": 3,
    }
    assert settings["launch"] == {
        "launch_type": "container", "esp32_host": "127.0.0.1",
        "esp32_port": config.UNUSED_ESP32_PORT, "arm_timeout_s": 10.0,
    }
    assert settings["flight"] == {
        "uavs_per_set": 3, "takeoff_altitude_m": 40.0, "altitude_separation_m": 20.0,
    }
    # Camera and hubs stay the owner's: the plan file brings the hubs.
    assert settings["camera"] == OWNER["camera"]
    assert settings["default_delivery_hubs"] == OWNER["default_delivery_hubs"]
    assert OWNER["connection"]["auto_connect"] is False  # not mutated


def test_owner_settings_are_optional_but_must_be_an_object(tmp_path):
    assert demo.owner_settings(tmp_path / "missing.json") == {}
    bad = tmp_path / "bad.json"
    bad.write_text("[1]", encoding="utf-8")
    with pytest.raises(models.RegressionError, match="not an object"):
        demo.owner_settings(bad)


def test_environment_is_the_shells_without_a_dock_world_plus_the_run_paths(tmp_path):
    env = demo.demo_environment(
        tmp_path, {"PATH": "p", "VITE_CESIUM_ION_TOKEN": "t", "NAVPY_SIM_DOCKS": "x", "OTHER": "o"}
    )

    # A shell's shared dock world must not replace the plan's waypoint docks.
    assert "NAVPY_SIM_DOCKS" not in env
    assert (env["PATH"], env["VITE_CESIUM_ION_TOKEN"], env["OTHER"]) == ("p", "t", "o")
    assert env["GCS_SETTINGS_PATH"] == str((tmp_path / demo.SETTINGS_NAME).resolve())
    assert env["NAVPY_LOG_DIR"] == str(tmp_path.resolve())
    # No eval pins: thread counts and hash seed stay the shell's.
    assert not {"PYTHONHASHSEED", "OMP_NUM_THREADS"} & set(env)


def test_printed_commands_survive_spaces_in_powershell():
    assert demo.powershell_command(r"C:\Program Files\py.exe", r"C: $b`'c\d.py") == (
        r"& 'C:\Program Files\py.exe' 'C: $b`''c\d.py'"
    )


class FakeApi:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def _start(monkeypatch, tmp_path, calls, owner=None, failing=None):
    sitl_root = SimpleNamespace(launch_arguments=lambda: ["--sitl-root", "r"])
    monkeypatch.setattr(demo, "prepare_sitl_root", lambda _d: (sitl_root, {}, "r"))
    monkeypatch.setattr(demo, "owner_settings", lambda: owner or {})
    api = FakeApi()

    def launch_stack(env, timeout_s, arguments, **kwargs):
        calls.append(("launch_stack", env, arguments, kwargs))
        return SimpleNamespace(api=api)

    monkeypatch.setattr(demo, "launch_stack", launch_stack)
    monkeypatch.setattr(demo, "owned_entry", lambda _env: {"frontend": 3100})
    monkeypatch.setattr(demo, "wait_for_frontend", lambda entry, _clock: calls.append("wait_for_frontend"))
    for name in ("wait_for_vehicles", "configure_vehicle_params", "verify_aas_profile"):
        monkeypatch.setattr(demo, name, lambda *_a, _name=name: calls.append(_name))
    for name, step in (failing or {}).items():
        monkeypatch.setattr(demo, name, step)

    def open_browser(url):
        assert api.closed
        calls.append(("open", url))

    return demo.start(tmp_path, SystemClock(), NOW, open_browser)


def test_start_launches_a_normal_stack_sets_params_then_opens_the_ui(monkeypatch, tmp_path):
    calls = []
    run_dir = _start(monkeypatch, tmp_path, calls, owner=OWNER)

    (_name, env, arguments, kwargs), *steps = calls
    assert arguments == ["--sitl-root", "r"]
    assert (kwargs["headless"], kwargs["browser"]) == (False, False)
    # The plain fixture scenario: no POI ordinals, so every UAV gets
    # targ_wps 0 until the operator's plan upload writes its own.
    assert kwargs["scenario"] == SCENARIO and SCENARIO.poi_nav_waypoint_ordinals == ()
    assert kwargs["launch_token"]
    # No connect, mission upload or companion start: auto-connect and the
    # operator's plan do those, as in a normal run.
    assert steps == [
        "wait_for_frontend", "wait_for_vehicles", "configure_vehicle_params",
        "verify_aas_profile", ("open", "http://localhost:3100"),
    ]
    written = json.loads((run_dir / demo.SETTINGS_NAME).read_text(encoding="utf-8"))
    assert written == demo.demo_settings(OWNER, FIXTURE)
    assert env["GCS_SETTINGS_PATH"] == str((run_dir / demo.SETTINGS_NAME).resolve())


def test_start_names_the_stop_command_when_a_step_fails(monkeypatch, tmp_path):
    calls = []

    def unlinked(*_args):
        raise models.RegressionError("vehicles never linked")

    with pytest.raises(models.RegressionError, match="never linked; stop the stack: .*gcs_stop.py"):
        _start(monkeypatch, tmp_path, calls, failing={"wait_for_vehicles": unlinked})
    assert calls[-1] == "wait_for_frontend"  # nothing after the failure, no browser


def test_start_refuses_a_running_stack(monkeypatch, tmp_path):
    def busy(*_args, **_kwargs):
        raise process.PreexistingOwnedStackError("exists")

    monkeypatch.setattr(demo, "prepare_sitl_root", lambda _d: (SimpleNamespace(launch_arguments=list), {}, ""))
    monkeypatch.setattr(demo, "owner_settings", dict)
    monkeypatch.setattr(demo, "launch_stack", busy)

    with pytest.raises(models.RegressionError, match="gcs_stop.py"):
        demo.start(tmp_path, SystemClock(), NOW)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_start_refuses_a_frontend_that_never_answers():
    def refused(url):
        assert url == "http://localhost:3100"
        raise ConnectionRefusedError("refused")

    with pytest.raises(models.RegressionError, match="node_modules"):
        demo.wait_for_frontend({"frontend": 3100}, FakeClock(), refused)
    demo.wait_for_frontend({"frontend": 3100}, FakeClock(), lambda _url: True)


def test_committed_plan_file_is_the_builders_output():
    assert plan.PLAN_PATH.read_text(encoding="utf-8") == plan.render(FIXTURE)
    assert plan.main(["--check"]) == 0


def test_plan_file_keeps_the_fixture_inputs_and_marks_two_docks_on_the_lowest_zone():
    data = plan.plan_file(FIXTURE)
    inputs = FIXTURE["inputs"]

    assert data["set_launch_points"] == [inputs["launch_point"]]
    assert data["default_delivery_hubs"] == inputs["delivery_hubs"]
    # Each zone keeps its fixture hub (east, middle, west).
    hub_names = [inputs["delivery_hubs"][i]["name"] for i in data["delivery_hub_assignments"]]
    assert hub_names == FIXTURE["expected"]["home_dock_by_zone"]
    assert data["sim_dock_wps"] == {"2": [1, 2]}
    assert data["detect_after_wps"] == {"0": 0, "1": 0, "2": 0}
    assert data["route_offset_m"] == plan.LINE_SPACING_M == 150.0
    lats = {p["lat"] for p in data["polygon"]}
    assert lats == {p["lat"] for p in inputs["polygon"]}


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
