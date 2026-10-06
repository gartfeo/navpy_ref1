"""Negative evidence must prove the intended cause, not merely a crashed SITL."""

import json

import pytest

from scripts.simtime_navigation_faults import corrupt, validate_failure
from scripts.simtime_navigation_protocol import COMMAND, HEADER, Identity, StepCommand


def test_late_apply_and_nonfinite_injections_hit_the_declared_fields() -> None:
    reply = StepCommand().reply(Identity((1, 2), 5, 100_000, 121, 500))
    assert COMMAND.unpack_from(corrupt(reply, "late"), HEADER.size)[:2] == (5, 7)
    import math
    assert math.isnan(COMMAND.unpack_from(corrupt(reply, "nonfinite"), HEADER.size)[4])
    assert corrupt(reply, "boot")[8] != reply[8]


@pytest.mark.parametrize("reason,step,boots", [
    ("connect", 5, 1), ("apply_step", 6, 1), ("apply_step", 5, 2),
])
def test_unrelated_failure_cannot_pass_a_negative_trial(tmp_path, reason, step, boots) -> None:
    (tmp_path / "supervisor.log").write_text(
        "Starting sketch 'ArduPlane'\n" * boots + f"NAVPY_NAVIGATION_INVALID {reason} step={step}\n")
    (tmp_path / "injection.json").write_text(json.dumps({"fault": "late", "step": 5}))
    with pytest.raises(ValueError):
        validate_failure(tmp_path, "late")


def test_exact_failure_requires_the_injection_record(tmp_path) -> None:
    (tmp_path / "supervisor.log").write_text(
        "Starting sketch 'ArduPlane'\nNAVPY_NAVIGATION_INVALID apply_step step=5\n")
    with pytest.raises(FileNotFoundError):
        validate_failure(tmp_path, "late")
    (tmp_path / "injection.json").write_text(json.dumps({"fault": "late", "step": 5}))
    assert validate_failure(tmp_path, "late")["expected_failure_verified"]


def test_completed_control_evidence_invalidates_a_negative_trial(tmp_path) -> None:
    (tmp_path / "supervisor.log").write_text(
        "Starting sketch 'ArduPlane'\nNAVPY_NAVIGATION_INVALID logger_not_ready step=2\n")
    (tmp_path / "navpy-navigation.csv").touch()
    with pytest.raises(ValueError, match="evidence"):
        validate_failure(tmp_path, "logger")
