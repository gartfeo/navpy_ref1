"""Evaluator runs keep one complete log: rotation can be switched off."""

from __future__ import annotations

import pytest

from navpy.core.kernel.logging.log_rotation import LOG_ROTATION_ENV, rotation_max_bytes


def test_rotation_is_on_by_default():
    assert rotation_max_bytes(5_000_000, {}) == 5_000_000
    assert rotation_max_bytes(5_000_000, {LOG_ROTATION_ENV: "on"}) == 5_000_000


def test_off_never_rolls_over():
    # RotatingFileHandler treats maxBytes=0 as "never roll over".
    assert rotation_max_bytes(5_000_000, {LOG_ROTATION_ENV: " OFF "}) == 0


def test_an_unknown_value_is_rejected():
    with pytest.raises(ValueError, match=LOG_ROTATION_ENV):
        rotation_max_bytes(5_000_000, {LOG_ROTATION_ENV: "0"})


def test_diagnostics_timeline_does_not_rotate_when_off(monkeypatch, tmp_path):
    from gcs.backend import diagnostics

    monkeypatch.setattr(diagnostics, "MAX_FILE_BYTES", 200)
    monkeypatch.setenv(LOG_ROTATION_ENV, "off")
    session = diagnostics.DiagnosticSession(tmp_path)
    session.start()
    for _ in range(20):
        session.emit("vehicle_connect_started", source="backend", sys_id=4)

    assert [path.name for path in session.timeline_files()] == ["timeline.jsonl"]
