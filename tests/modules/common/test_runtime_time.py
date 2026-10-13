"""The RuntimeTime port: binding rules and wall delegation."""

from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from navpy.core.kernel import runtime_time as rt


def _runtime(**clock: object) -> rt.RuntimeTime:
    wall = rt.wall_runtime()
    return rt.RuntimeTime(SimpleNamespace(**clock), wall.threads, wall.entropy)


def test_a_binding_starts_on_the_wall_runtime() -> None:
    binding = rt.RuntimeBinding()
    runtime = binding.current()
    assert isinstance(runtime.clock, rt.WallClock)
    assert isinstance(runtime.threads, rt.WallThreads)
    assert binding.current() is runtime


def test_a_runtime_binds_only_before_first_use() -> None:
    bound = _runtime()
    binding = rt.RuntimeBinding()
    binding.bind(bound)
    assert binding.current() is bound
    with pytest.raises(RuntimeError, match="once"):
        binding.bind(_runtime())

    used = rt.RuntimeBinding()
    used.current()
    with pytest.raises(RuntimeError, match="before its first use"):
        used.bind(_runtime())


def test_the_facade_reads_the_bound_runtime(monkeypatch) -> None:
    binding = rt.RuntimeBinding()
    stamp = datetime(2023, 11, 14, 22, 13, 20, 250000, tzinfo=timezone.utc)
    binding.bind(_runtime(monotonic=lambda: 7.5, now=lambda tz=None: stamp))
    monkeypatch.setattr(rt, "_binding", binding)
    assert rt.monotonic() == 7.5
    assert rt.now(timezone.utc) is stamp


def test_the_wall_runtime_calls_the_standard_library_at_call_time(monkeypatch) -> None:
    monkeypatch.setattr(time, "monotonic", lambda: 42.0)
    assert rt.wall_runtime().clock.monotonic() == 42.0


def test_wall_now_is_datetime_now() -> None:
    before = datetime.now(timezone.utc)
    stamp = rt.wall_runtime().clock.now(timezone.utc)
    assert before <= stamp <= datetime.now(timezone.utc)
    assert stamp.tzinfo is timezone.utc


def test_wall_primitives_are_the_standard_library_ones() -> None:
    threads = rt.wall_runtime().threads
    assert isinstance(threads.Event(), threading.Event)
    assert isinstance(threads.Condition(threads.RLock()), threading.Condition)
    thread = threads.Thread(target=lambda: None, name="rt-test", daemon=True)
    assert isinstance(thread, threading.Thread) and thread.daemon
    queue = threads.Queue()
    with pytest.raises(rt.Empty):
        queue.get_nowait()
