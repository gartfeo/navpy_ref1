"""Tests for the shared atomic JSON/text write helper."""
import json
import os

import pytest

from gcs.backend import atomic_json
from gcs.backend.atomic_json import atomic_write_json, atomic_write_text


class _Crash(BaseException):
    """Stands in for the process dying (not an ordinary Exception)."""


def test_write_json_round_trips(tmp_path):
    target = tmp_path / "doc.json"
    atomic_write_json(target, {"a": 1, "b": [1, 2]})
    assert json.loads(target.read_text(encoding="utf-8")) == {"a": 1, "b": [1, 2]}
    assert target.read_text(encoding="utf-8").endswith("\n")


def test_crash_before_commit_leaves_previous_file_intact(tmp_path, monkeypatch):
    """Dying after the new bytes are written but before the rename must leave
    the old document untouched — never a truncated or half-written file."""
    target = tmp_path / "doc.json"
    target.write_text('{"old": true}', encoding="utf-8")

    def crash(*_args, **_kwargs):
        raise _Crash()

    monkeypatch.setattr(atomic_json.os, "replace", crash)
    with pytest.raises(_Crash):
        atomic_write_text(target, '{"new": true, "padding": "' + "x" * 100_000 + '"}')

    assert target.read_text(encoding="utf-8") == '{"old": true}'
    assert sorted(p.name for p in tmp_path.iterdir()) == ["doc.json"]


def test_crash_while_writing_temp_leaves_previous_file_intact(tmp_path, monkeypatch):
    target = tmp_path / "doc.json"
    target.write_text('{"old": true}', encoding="utf-8")

    def crash(_fd):
        raise _Crash()

    monkeypatch.setattr(atomic_json.os, "fsync", crash)
    with pytest.raises(_Crash):
        atomic_write_text(target, '{"new": true}')

    assert target.read_text(encoding="utf-8") == '{"old": true}'
    assert sorted(p.name for p in tmp_path.iterdir()) == ["doc.json"]


def test_temp_file_is_a_sibling_so_replace_stays_on_one_filesystem(tmp_path, monkeypatch):
    target = tmp_path / "doc.json"
    seen = []
    real_replace = os.replace

    def spy(src, dst):
        seen.append((os.path.dirname(os.fspath(src)), os.fspath(dst)))
        real_replace(src, dst)

    monkeypatch.setattr(atomic_json.os, "replace", spy)
    atomic_write_text(target, "{}")
    assert seen == [(os.fspath(tmp_path), os.fspath(target))]
