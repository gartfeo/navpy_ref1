"""Tests for the opt-in pose-cadence diagnostic instrumentation.

``pose_cadence_debug`` decides whether it is enabled at import time from the
``NAVPY_POSE_CADENCE_DEBUG`` env var, so these tests reload the module under a
patched environment. Every test restores the disabled import-time state.
"""
import importlib
import os

import pytest

import navpy.modules.vehicle.pose_cadence_debug as pcd


@pytest.fixture(autouse=True)
def _join_flusher_after_test():
    """Join the flush daemon each ENABLED reload starts, so no test leaks it.

    An ENABLED import spawns an unkillable ``while``-loop daemon; without this
    every reload-under-ENABLED test would leave one running for the rest of the
    suite (where a shared ``time.sleep`` monkeypatch turns it into a busy spin).
    """
    yield
    pcd.stop_flusher()


def _reload(monkeypatch, value):
    # Join any flusher the current generation started BEFORE the reload replaces
    # the module globals (and the live thread's stop-event handle); an ENABLED
    # reload spawns a fresh daemon flush loop at import.
    pcd.stop_flusher()
    if value is None:
        monkeypatch.delenv("NAVPY_POSE_CADENCE_DEBUG", raising=False)
    else:
        monkeypatch.setenv("NAVPY_POSE_CADENCE_DEBUG", value)
    return importlib.reload(pcd)


def teardown_module(_module):
    """Leave the module in its default disabled state for the rest of the suite."""
    pcd.stop_flusher()
    os.environ.pop("NAVPY_POSE_CADENCE_DEBUG", None)
    importlib.reload(pcd)


def test_disabled_by_default(monkeypatch):
    mod = _reload(monkeypatch, None)
    assert mod.ENABLED is False
    # Every record call must be a cheap no-op that never raises or buffers.
    mod.record_attitude_arrival(1, 1.0, 100)
    mod.record_position_arrival(1, 1.0, 100)
    mod.record_telemetry_reject(1, 1.0, "ATTITUDE", "stale_boot")
    mod.record_detector_pose(1, 1.0, 0.1, "emitted", 0.1)
    mod.record_observation(1, 1.0, 0.1, 0.12, "fresh")
    mod.record_worker(1, 1.0, 5.0, 0.1, 0.13, "measured")
    mod.dump_all()
    assert mod._buffers == {}


def test_sentinel_values_disable(monkeypatch):
    for value in ("", "0", "off", "false", "no", "  Off  "):
        mod = _reload(monkeypatch, value)
        assert mod.ENABLED is False, f"{value!r} should disable"


def test_enabled_writes_pid_keyed_csv(monkeypatch, tmp_path):
    mod = _reload(monkeypatch, str(tmp_path))
    assert mod.ENABLED is True

    mod.record_attitude_arrival(7, 100.0, 1000)
    mod.record_attitude_arrival(7, 100.02, 1033)
    mod.record_detector_pose(7, 100.0, 1.0, "emitted", 205.5)
    mod.record_detector_pose(7, 100.01, None, "no_attitude", None)  # None -> empty
    mod.record_worker(7, 100.0, 4.5, 1.0, 1.02, "measured")
    mod.record_worker(7, 100.04, 1.5, 1.0, 1.06, "measured", "held")
    mod.dump_all()

    pid = os.getpid()
    arr = tmp_path / f"pose_cadence_attitude_arrival_uav_7_pid_{pid}.csv"
    det = tmp_path / f"pose_cadence_detector_pose_uav_7_pid_{pid}.csv"
    wrk = tmp_path / f"pose_cadence_worker_uav_7_pid_{pid}.csv"
    assert arr.exists() and det.exists() and wrk.exists()

    assert arr.read_text(encoding="utf-8").splitlines() == [
        "wall_s,boot_ms", "100.0,1000", "100.02,1033",
    ]
    assert det.read_text(encoding="utf-8").splitlines() == [
        "wall_s,boot_s,outcome,frame_ts",
        "100.0,1.0,emitted,205.5",
        "100.01,,no_attitude,",
    ]
    assert wrk.read_text(encoding="utf-8").splitlines() == [
        "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome",
        "100.0,4.5,1.0,1.02,measured,fresh",
        "100.04,1.5,1.0,1.06,measured,held",
    ]


def test_new_pipeline_streams_write_their_headers_and_rows(monkeypatch, tmp_path):
    mod = _reload(monkeypatch, str(tmp_path))

    mod.record_position_arrival(5, 50.0, 2000)
    mod.record_telemetry_reject(5, 50.1, "GLOBAL_POSITION_INT", "stale_boot")
    mod.record_observation(5, 50.2, 12.5, 12.53, "fresh")
    mod.record_observation(5, 50.3, None, None, "duplicate")  # Nones -> empty
    mod.dump_all()

    pid = os.getpid()
    pos = tmp_path / f"pose_cadence_position_arrival_uav_5_pid_{pid}.csv"
    rej = tmp_path / f"pose_cadence_telemetry_reject_uav_5_pid_{pid}.csv"
    obs = tmp_path / f"pose_cadence_observation_uav_5_pid_{pid}.csv"

    assert pos.read_text(encoding="utf-8").splitlines() == [
        "wall_s,boot_ms", "50.0,2000",
    ]
    assert rej.read_text(encoding="utf-8").splitlines() == [
        "wall_s,mtype,reason", "50.1,GLOBAL_POSITION_INT,stale_boot",
    ]
    assert obs.read_text(encoding="utf-8").splitlines() == [
        "wall_s,obs_ts,src_now_s,outcome",
        "50.2,12.5,12.53,fresh",
        "50.3,,,duplicate",
    ]


def test_every_stream_has_a_header():
    # A stream without a header row would silently produce a CSV whose first
    # data row is consumed as the header by every reader.
    for stream in ("attitude_arrival", "position_arrival", "telemetry_reject",
                   "detector_pose", "observation", "worker"):
        assert pcd._HEADERS.get(stream), stream


def test_incremental_append_writes_only_new_rows(monkeypatch, tmp_path):
    mod = _reload(monkeypatch, str(tmp_path))
    path = tmp_path / f"pose_cadence_attitude_arrival_uav_9_pid_{os.getpid()}.csv"

    mod.record_attitude_arrival(9, 1.0, 10)
    mod.dump_all()
    assert path.read_text(encoding="utf-8").splitlines() == ["wall_s,boot_ms", "1.0,10"]

    # A second batch + flush must append (header once, no duplicated rows).
    mod.record_attitude_arrival(9, 2.0, 20)
    mod.dump_all()
    assert path.read_text(encoding="utf-8").splitlines() == [
        "wall_s,boot_ms", "1.0,10", "2.0,20",
    ]

    # A flush with nothing new is a no-op (does not rewrite or duplicate).
    mod.dump_all()
    assert path.read_text(encoding="utf-8").splitlines() == [
        "wall_s,boot_ms", "1.0,10", "2.0,20",
    ]


def test_explicit_directory_path_is_used_verbatim(monkeypatch, tmp_path):
    target = tmp_path / "nested" / "cadence"
    mod = _reload(monkeypatch, str(target))
    assert mod.ENABLED is True
    assert mod._OUTPUT_DIR == str(target)
    mod.record_attitude_arrival(3, 1.0, 1)
    mod.dump_all()
    assert (target / f"pose_cadence_attitude_arrival_uav_3_pid_{os.getpid()}.csv").exists()


def test_enabled_reload_starts_daemon_that_stop_flusher_joins(monkeypatch, tmp_path):
    # Integration seam: the recorder wires its OWN dump into a
    # PoseCadenceFlusher and starts it at import under ENABLED, and the module's
    # stop_flusher() delegates to that flusher and joins the generation. The
    # flusher's own start/stop/timeout/atexit contract lives in
    # test_pose_cadence_flusher.py.
    mod = _reload(monkeypatch, str(tmp_path))
    assert mod._flusher._flush is mod.dump_all
    assert mod._flusher._interval_s == mod._FLUSH_INTERVAL_S
    thread = mod._flusher._thread
    assert thread is not None and thread.is_alive()

    mod.stop_flusher()
    assert not thread.is_alive()
    assert mod._flusher._thread is None
