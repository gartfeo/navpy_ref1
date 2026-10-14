"""Pose-cadence recording as a participant's run input (Codex 9g P2-5)."""

from __future__ import annotations

import os
import sys

import pytest

from navpy.core.kernel.logging.pose_cadence_naming import NAMING_ENV
from navpy.sim.lockstep.pose_recording import DEBUG_ENV, POSE_CADENCE_DIR, enable_pose_cadence
from scripts.eval_gcs_demo_lockstep import LockstepOptions, manifest
from scripts.eval_gcs_demo_scenario import load_scenario
from tests.modules.lockstep.coordinator_doubles import REPO

RECORDER = "navpy.core.kernel.logging.pose_cadence_debug"


@pytest.fixture
def clean(monkeypatch):
    for name in (DEBUG_ENV, NAMING_ENV):  # registered for restore even when unset
        monkeypatch.setenv(name, "")
        monkeypatch.delenv(name)
    monkeypatch.delitem(sys.modules, RECORDER, raising=False)
    return monkeypatch


def test_a_run_relative_directory_switches_recording_on_with_producer_names(clean):
    enable_pose_cadence({POSE_CADENCE_DIR: "source-time"})
    assert (os.environ[DEBUG_ENV], os.environ[NAMING_ENV]) == ("source-time", "producer")


def test_no_directory_leaves_recording_off(clean):
    enable_pose_cadence({"argv": []})
    assert DEBUG_ENV not in os.environ and NAMING_ENV not in os.environ


@pytest.mark.parametrize("directory", ["C:/runs/x", "/abs", "../up", "", "1", "on", 3])
def test_a_host_path_or_a_switch_word_is_refused(clean, directory):
    with pytest.raises(ValueError, match="run-relative"):
        enable_pose_cadence({POSE_CADENCE_DIR: directory})
    assert DEBUG_ENV not in os.environ


def test_a_recorder_imported_before_the_run_input_fails_closed(clean):
    clean.setitem(sys.modules, RECORDER, object())
    with pytest.raises(RuntimeError, match="imported before"):
        enable_pose_cadence({POSE_CADENCE_DIR: "source-time"})
    assert DEBUG_ENV not in os.environ


def test_the_lockstep_eval_records_in_both_participants():
    scenario = load_scenario(REPO / "scripts" / "eval_lockstep_slice_scenario.json")
    roles = manifest(scenario, LockstepOptions(1, 600.0, 15.0)).roles
    assert {name: spec.config.get(POSE_CADENCE_DIR) for name, spec in roles.items()} == {
        "backend": "source-time", "navpy": "source-time"}


def test_the_variable_is_the_one_the_recorder_reads():
    import inspect

    import navpy.core.kernel.logging.pose_cadence_debug as recorder

    assert f'"{DEBUG_ENV}"' in inspect.getsource(recorder._resolve_output_dir)
