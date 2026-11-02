"""Tests for the opt-in GC record (``pose_cadence_gc``)."""
import atexit
import gc
import os
import sys

import pytest

import navpy.core.kernel.logging.pose_cadence_debug as pcd
from navpy.core.kernel.logging.pose_cadence_gc import (
    GcRecord,
    install_gc_record,
    uninstall_gc_record,
)


@pytest.fixture
def enabled(monkeypatch, tmp_path):
    monkeypatch.setattr(pcd, "ENABLED", True)
    monkeypatch.setattr(pcd, "_OUTPUT_DIR", str(tmp_path))
    monkeypatch.setattr(pcd, "_buffers", {})
    monkeypatch.setattr(pcd, "_written", {})
    # Keep this test process's own exit hooks untouched.
    monkeypatch.setattr(atexit, "register", lambda fn: fn)
    return tmp_path


def _clock(values):
    it = iter(values)
    return lambda: next(it)


def test_one_row_per_collection_with_duration_and_counts():
    rows = []
    record = GcRecord(rows, wall_time=_clock([100.0]), perf_counter=_clock([5.0, 5.0125]))

    record("start", {"generation": 2, "collected": 0, "uncollectable": 0})
    record("stop", {"generation": 2, "collected": 7, "uncollectable": 1})

    assert len(rows) == 1
    wall_s, perf_s, generation, dur_ms, collected, uncollectable = rows[0]
    assert (wall_s, perf_s, generation, collected, uncollectable) == (100.0, 5.0, 2, 7, 1)
    assert dur_ms == pytest.approx(12.5)


def test_a_stop_without_its_start_writes_nothing():
    rows = []
    GcRecord(rows, wall_time=_clock([]), perf_counter=_clock([1.0]))(
        "stop", {"generation": 0, "collected": 0, "uncollectable": 0},
    )
    assert rows == []


def test_the_callback_calls_only_its_clocks_and_list_append():
    # Inside the collector a lock could deadlock and a log could allocate
    # without bound: the callback may call nothing but its clocks and append.
    rows = []

    def wall():
        return 1.0

    def perf():
        return 2.0

    record = GcRecord(rows, wall_time=wall, perf_counter=perf)
    calls = []

    def profile(frame, event, arg):
        if event == "call":
            calls.append(frame.f_code.co_name)
        elif event == "c_call":
            calls.append(arg.__qualname__)

    info = {"generation": 1, "collected": 0, "uncollectable": 0}
    sys.setprofile(profile)
    try:
        record("start", info)
        record("stop", info)
    finally:
        sys.setprofile(None)

    called = [name for name in calls if name != "__call__"]
    assert called == ["wall", "perf", "perf", "list.append", "setprofile"]


def test_disabled_installs_nothing(monkeypatch):
    monkeypatch.setattr(pcd, "ENABLED", False)
    before = list(gc.callbacks)
    install_gc_record(3)
    assert gc.callbacks == before


def test_a_real_collection_is_written_and_freed_by_the_flush(enabled):
    install_gc_record(3)
    try:
        gc.collect()
    finally:
        uninstall_gc_record()
    assert not any(isinstance(cb, GcRecord) for cb in gc.callbacks)
    rows = pcd._buffers[(3, "gc")]
    count = len(rows)
    gc.collect()
    assert len(rows) == count  # uninstalled: no row after the final flush

    assert rows and rows[-1][2] == 2  # gc.collect() runs generation 2
    pcd.dump_all()
    assert rows == []

    lines = (enabled / f"pose_cadence_gc_uav_3_pid_{os.getpid()}.csv").read_text(
        encoding="utf-8").splitlines()
    assert lines[0] == "wall_s,perf_s,generation,dur_ms,collected,uncollectable"
    assert lines[-1].split(",")[2] == "2"


_EXIT_WITHOUT_STOP = """
import atexit, gc
def probe():  # registered first, so it runs last: after the flusher's final drain
    import navpy.core.kernel.logging.pose_cadence_debug as pcd
    gc.collect()
    print("left", len(pcd._buffers[(3, "gc")]), flush=True)
atexit.register(probe)
import navpy.core.kernel.logging.pose_cadence_debug as pcd
from navpy.core.kernel.logging.pose_cadence_gc import install_gc_record
install_gc_record(3)
gc.collect()
"""


def test_an_exit_without_a_stop_request_adds_no_row_after_the_final_drain(tmp_path):
    import subprocess

    env = {**os.environ, "NAVPY_POSE_CADENCE_DEBUG": str(tmp_path)}
    env.pop("NAVPY_POSE_CADENCE_NAMING", None)
    out = subprocess.run(
        [sys.executable, "-c", _EXIT_WITHOUT_STOP], env=env,
        capture_output=True, text=True, timeout=60, check=True,
    ).stdout
    assert out.split() == ["left", "0"]
    (path,) = tmp_path.glob("pose_cadence_gc_uav_3_*.csv")
    assert len(path.read_text(encoding="utf-8").splitlines()) >= 2  # header + the collect
