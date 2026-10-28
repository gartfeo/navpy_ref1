"""gcs_launch.py --cpu-share-s: the backend's share request, else background (bounded_cpu_share.md)."""

from __future__ import annotations

import argparse

import pytest

from gcs.backend.cpu_share_holder import CPU_SHARE_ENV
from scripts import gcs_launch_cpu_share as share


def _parse(*argv: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    share.add_cpu_share_argument(parser)
    return parser.parse_args(argv)


def test_without_a_share_the_launcher_and_its_children_run_as_background(monkeypatch):
    calls = []
    monkeypatch.setattr(share, "apply_background", lambda: calls.append("background") or 1)
    assert share.scheduling_env(_parse()) == {}
    assert calls == ["background"]


def test_without_the_flag_an_inherited_request_is_dropped(monkeypatch):
    monkeypatch.setattr(share, "apply_background", lambda: True)
    monkeypatch.setenv(CPU_SHARE_ENV, "900")
    assert share.scheduling_env(_parse()) == {}
    assert CPU_SHARE_ENV not in share.os.environ


def test_a_share_request_goes_to_the_backend(monkeypatch):
    monkeypatch.setattr(share, "apply_background", lambda: pytest.fail("not background"))
    assert share.scheduling_env(_parse("--cpu-share-s", "900")) == {CPU_SHARE_ENV: "900.0"}


@pytest.mark.parametrize("seconds", ("0", "-1", "nan", "inf"))
def test_the_share_must_be_a_positive_finite_time(seconds):
    with pytest.raises(SystemExit):
        _parse("--cpu-share-s", seconds)
