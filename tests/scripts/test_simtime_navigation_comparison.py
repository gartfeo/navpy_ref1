"""The acceptance gate must require the declared matrix and one implementation."""

import itertools
from pathlib import Path

import pytest

from scripts import compare_simtime_navigation as comparator


def fake_case(rate: int = 40, speed: float = 10, delayed: bool = False, identity: str = "same", boot: int = 0) -> tuple:
    return ({"case": "fixture", "camera_hz": rate, "requested_speed": speed,
             "delayed": delayed, "identity": {"digest": identity}, "boot": [boot, 1]}, [{"control": 1}])


def test_one_run_cannot_claim_repeatability(monkeypatch) -> None:
    monkeypatch.setattr(comparator, "validate", lambda case: fake_case())
    with pytest.raises(ValueError, match="matrix|comparisons"):
        comparator.compare([Path("one")])


def test_missing_speed_and_delay_cells_cannot_pass(monkeypatch) -> None:
    monkeypatch.setattr(comparator, "validate", lambda case: fake_case())
    with pytest.raises(ValueError, match="matrix"):
        comparator.compare([Path("one"), Path("two")])


def test_different_implementations_cannot_be_combined(monkeypatch) -> None:
    cells = {Path(str(i)): fake_case(40, speed, delayed, str(i), i)
             for i, (speed, delayed, repeat) in enumerate(itertools.product((1., 10.), (False, True), range(2)))}
    monkeypatch.setattr(comparator, "validate", lambda case: cells[case])
    with pytest.raises(ValueError, match="identity"):
        comparator.compare(list(cells))


def test_complete_matrix_compares_every_pair(monkeypatch) -> None:
    cells = {Path(str(i)): fake_case(40, speed, delayed, boot=i)
             for i, (speed, delayed, repeat) in enumerate(itertools.product((1., 10.), (False, True), range(2)))}
    monkeypatch.setattr(comparator, "validate", lambda case: cells[case])
    report = comparator.compare(list(cells))
    assert len(report["comparisons"]) == 28
    assert all(pair["equal"] for pair in report["comparisons"])


def test_reusing_the_same_run_does_not_count_as_a_repeat(monkeypatch) -> None:
    cells = {Path(str(i)): fake_case(40, speed, delayed, boot=i)
             for i, (speed, delayed) in enumerate(itertools.product((1., 10.), (False, True)))}
    monkeypatch.setattr(comparator, "validate", lambda case: cells[case])
    with pytest.raises(ValueError, match="distinct"):
        comparator.compare(list(cells) * 2)


def test_copying_a_run_to_another_directory_does_not_make_a_repeat(monkeypatch) -> None:
    cells = {Path(str(i)): fake_case(40, speed, delayed, boot=0)
             for i, (speed, delayed, repeat) in enumerate(itertools.product((1., 10.), (False, True), range(2)))}
    monkeypatch.setattr(comparator, "validate", lambda case: cells[case])
    with pytest.raises(ValueError, match="distinct"):
        comparator.compare(list(cells))


def test_offline_validator_can_improve_but_runtime_identity_cannot_change(monkeypatch, tmp_path) -> None:
    import hashlib
    (tmp_path / "navigation-defaults.parm").write_bytes(b"profile")
    validator = "scripts/compare_simtime_navigation.py"
    actual = {validator: "new-validator", "src/navpy/runtime.py": "runtime-sha"}
    monkeypatch.setattr(comparator, "source_hashes", lambda root: actual)
    recorded = {"source_sha256": {validator: "old-validator", "src/navpy/runtime.py": "runtime-sha"},
                "defaults_sha256": hashlib.sha256(b"profile").hexdigest()}
    comparator.validate_source_identity(recorded, tmp_path)
    assert recorded["source_sha256"][validator] == "old-validator"
    recorded["source_sha256"]["src/navpy/runtime.py"] = "different-runtime"
    with pytest.raises(ValueError, match="identity"):
        comparator.validate_source_identity(recorded, tmp_path)


def test_standalone_comparator_needs_no_pythonpath(tmp_path) -> None:
    import os
    import subprocess
    import sys

    root = Path(__file__).resolve().parents[2]
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, str(root / "scripts/compare_simtime_navigation.py"), "--help"],
        cwd=tmp_path, env=environment, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout
