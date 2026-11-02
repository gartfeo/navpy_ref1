"""The hand demo's commit and input record: allow-listed, and never able to stop a launch."""

from __future__ import annotations

import importlib
import json
import re
import subprocess
from pathlib import Path

import pytest

from scripts import delivery_demo as demo
from scripts import delivery_demo_check as check
from scripts import delivery_demo_manifest as manifest

fixture_module = importlib.import_module("scripts.eval_delivery_fixture")
models = importlib.import_module("scripts.eval_gcs_demo_models")
SCENARIO = fixture_module.delivery_scenario(
    fixture_module.load_fixture(fixture_module.FIXTURE_PATH), fixture_module.FIXTURE_PATH
)
SECRET = "planted-secret-value-7f3a"
SHELL = {
    "PATH": SECRET,
    "VITE_CESIUM_ION_TOKEN": SECRET,
    "NAVPY_API_TOKEN": SECRET,
    "NAVPY_SIM_VIDEO_URI": f"rtsp://user:{SECRET}@host/stream",
    "GCS_PASSWORD": SECRET,
    "AWS_SECRET_ACCESS_KEY": SECRET,
    "OMP_NUM_THREADS": "2",
}


def _record(run_dir, env, warn=None):
    run_dir.mkdir(exist_ok=True)
    kwargs = {} if warn is None else {"warn": warn}
    manifest.record_run_inputs(run_dir, env, {"speedup": 10}, "/home/u/root", SCENARIO, 3.0, **kwargs)


def test_a_planted_secret_variable_never_reaches_the_run_folder(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / demo.SETTINGS_NAME).write_text("{}", encoding="utf-8")
    warnings = []

    # The real chain: the launch's environment, then the real manifest writer.
    _record(run_dir, demo.demo_environment(run_dir, SHELL), warnings.append)

    assert warnings == []
    written = {path.name: path.read_text(encoding="utf-8") for path in run_dir.iterdir()}
    assert {"input_manifest.json", "input_manifest_host.json"} <= set(written)
    assert not [name for name, text in written.items() if SECRET in text]
    variables = json.loads(written["input_manifest.json"])["environment"]["variables"]
    assert variables["OMP_NUM_THREADS"] == "2"
    assert variables["GCS_SETTINGS_PATH"] == str(Path("<run_dir>") / demo.SETTINGS_NAME)
    assert set(variables) <= manifest.MANIFEST_ENV


def test_the_allow_list_covers_the_runs_own_paths_and_no_secret_looking_name():
    assert set(demo.RUN_ENV) <= manifest.MANIFEST_ENV
    assert not [name for name in manifest.MANIFEST_ENV if re.search("TOKEN|KEY|SECRET|PASS|URI|CRED", name)]


def test_only_listed_names_pass_exactly():
    env = {"NAVPY_LOG_DIR": "a", "NAVPY_LOG_DIR_EXTRA": "b", "navpy_log_dir": "c"}
    assert manifest.manifest_environment(env) == {"NAVPY_LOG_DIR": "a"}


@pytest.mark.parametrize(
    "failure",
    [
        OSError("disk full"),
        models.RegressionError("input fault: DEM tile is missing"),
        subprocess.TimeoutExpired("git", 60.0),
        ImportError("no pymavlink"),
    ],
)
def test_a_failed_write_only_warns(monkeypatch, tmp_path, failure):
    def refused(*_args):
        raise failure

    monkeypatch.setattr(manifest, "write_input_manifest", refused)
    warnings = []

    assert _record(tmp_path / "run", {}, warnings.append) is None

    assert len(warnings) == 1
    assert warnings[0].startswith("WARNING: this run has no complete commit and input record")
    assert "The launch goes on." in warnings[0]


@pytest.mark.parametrize("stderr_fault", [OSError("broken pipe"), ValueError("I/O operation on closed file")])
def test_a_warning_that_cannot_be_written_does_not_stop_the_launch(monkeypatch, tmp_path, stderr_fault):
    class BrokenStderr:
        def write(self, _text):
            raise stderr_fault

    def refused(*_args):
        raise OSError("disk full")

    monkeypatch.setattr(manifest, "write_input_manifest", refused)
    monkeypatch.setattr(manifest.sys, "stderr", BrokenStderr())

    assert _record(tmp_path / "run", {}) is None


def test_an_unreadable_plan_file_only_warns(monkeypatch, tmp_path):
    monkeypatch.setattr(manifest, "PLAN_PATH", tmp_path / "gone.json")
    warnings = []

    _record(tmp_path / "run", {}, warnings.append)

    assert len(warnings) == 1
    assert not (tmp_path / "run" / "input_manifest.json").exists()


def test_the_check_prints_the_commit_and_inputs_of_a_real_manifest(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    settings = run_dir / demo.SETTINGS_NAME
    settings.write_text("{}", encoding="utf-8")
    env = {"GCS_SETTINGS_PATH": str(settings), "OMP_NUM_THREADS": "2", "PATH": SECRET}

    _record(run_dir, env)
    text = "\n".join(check.input_lines(run_dir))

    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], capture_output=True, text=True, cwd=check._REPOSITORY_ROOT, check=True
    ).stdout.strip()
    recorded = json.loads((run_dir / "input_manifest.json").read_text(encoding="utf-8"))
    assert f"Commit: {head}" in text
    assert recorded["run_options"]["offered_plan_sha256"][:12] in text
    assert recorded["environment"]["files"]["GCS_SETTINGS_PATH"][:12] in text
    assert "CPU share asked 3.0 min" in text
    assert "SITL speedup 10" in text
    assert "OMP_NUM_THREADS=2" in text
    assert SECRET not in text
