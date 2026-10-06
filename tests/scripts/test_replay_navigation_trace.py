"""Replay instrumentation must preserve commands and reject changed evidence."""

import copy
from pathlib import Path

import pytest

from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.models.location import Location
from scripts.replay_navigation_trace import replay, verify_live_rows
from scripts import simtime_navigation_runtime as runtime
from scripts.simtime_navigation_identity import source_hashes
from scripts.simtime_navigation_protocol import GUIDED, TAKEOFF_ARM, HEADER, STATE, Snapshot, StepCommand
from scripts.simtime_navigation_runtime import SynchronousNavigation
from tests.scripts.test_simtime_navigation_protocol import packet


@pytest.fixture(params=[None, -1, 0, 37, 100])
def capture(tmp_path, request):
    live = tmp_path / "live"
    live.mkdir()
    wall = [100.0]
    dock = (40.01, 44., 1300.)
    engine = SynchronousNavigation(Location(*dock, is_absolute=True), speedup=10,
                                 wall_now=lambda: wall[0], output=live,
                                 trim_throttle_percent=43, configured_throttle_percent=request.param)
    records = []
    try:
        for index in range(60):
            wall[0] = 100 + index * .002
            raw = packet(index + 1, mode=13 if index < 5 else 15)
            values = list(STATE.unpack_from(raw, HEADER.size))
            values[-19] = -1.0
            raw = raw[:HEADER.size] + STATE.pack(*values)
            snapshot = Snapshot.decode(raw)
            if index >= 58:
                command, evidence = StepCommand(), engine.drain()
            else:
                command, evidence = engine.advance(snapshot, wall[0])
            if index == 0:
                command = StepCommand(TAKEOFF_ARM)
            if index == 4:
                command = StepCommand(GUIDED)
            records.append(dict(snapshot=raw.hex(), reply=command.reply(snapshot.identity).hex(),
                                evidence=evidence, wall_s=wall[0], delay_s=0.))
        peer = dict(version=4, throttle_policy={"trim_throttle_percent": 43,
                    "configured_throttle_percent": request.param}, dock=dock, speedup=10, camera_hz=40, guided_step=5,
                    seed_step=engine.seed_step, command_count=engine.command_count, records=records)
    finally:
        engine.close()
    return peer, live


def test_exact_replay_recovers_rows_and_restores_logger(capture, tmp_path):
    peer, live = capture
    output = tmp_path / "replay"
    result = replay(peer, output)
    rows = verify_live_rows(live, output)
    assert result["records"] == 60
    assert result["commands"] > result["noncapture_reissues"] > 0
    assert rows["navigation_debug.csv"]["replay"] > rows["navigation_debug.csv"]["live"]
    assert NavigationLogger.MIN_LOG_INTERVAL == .1
    assert runtime.NavigationLogger is NavigationLogger


@pytest.mark.parametrize("kind", ["reply", "pixel", "clock"])
def test_changed_capture_aborts(capture, tmp_path, kind):
    peer, _ = capture
    peer = copy.deepcopy(peer)
    row = peer["records"][8]
    if kind == "reply":
        row["reply"] = row["reply"][:-2] + "ff"
        reason = "reply mismatch at step 9"
    elif kind == "pixel":
        row["evidence"]["pixels"][0] += .1
        reason = "evidence mismatch at step 9"
    else:
        row["wall_s"] = 0.
        reason = "nonmonotonic wall clock at step 9"
    with pytest.raises(ValueError, match=reason):
        replay(peer, tmp_path / "bad")
    assert NavigationLogger.MIN_LOG_INTERVAL == .1
    assert runtime.NavigationLogger is NavigationLogger


def test_changed_live_diagnostic_aborts(capture, tmp_path):
    peer, live = capture
    output = tmp_path / "replay"
    replay(peer, output)
    path = live / "navigation_debug.csv"
    with path.open("a") as stream:
        stream.write("99.0,EVENT:TERMINAL_RESPONSE_STATE,corrupted=True\n")
    with pytest.raises(ValueError, match="live row mismatch.*99.0"):
        verify_live_rows(live, output)


def test_analyzer_does_not_change_flight_source_identity():
    assert "scripts/replay_navigation_trace.py" not in source_hashes(Path(__file__).resolve().parents[2])


def test_offline_event_is_written_before_return(monkeypatch):
    import io
    from scripts.replay_navigation_trace import FullRateLogger
    from navpy.logger import navigation_log_streams as streams
    from navpy.logger.logger_api import ConsoleLogger
    from navpy.logger.cache_log_level import CacheLogLevel
    from navpy.logger.log_events import LogEvent

    class DeferredWorker:
        first_error = None
        def __init__(self, write):
            self.write, self.pending = write, []
        def submit(self, item):
            self.pending.append(item)
        def wait_until_idle(self):
            for item in self.pending:
                self.write(item)
            self.pending.clear()
        def close(self):
            self.wait_until_idle()

    monkeypatch.setattr(streams, "NavigationStreamWorker", DeferredWorker)
    debug = io.StringIO()
    logger = FullRateLogger(1, ConsoleLogger(CacheLogLevel.ERROR),
                           streams=streams.NavigationLogStreams(None, debug))
    try:
        logger.log_event(LogEvent.FINAL_APPROACH_CMD, {"probe": "fenced"})
        assert "probe=fenced" in debug.getvalue()
    finally:
        logger.close()


@pytest.mark.parametrize("failure", ["environment", "fleet_hash"])
@pytest.mark.parametrize("instances", [1, 3])
def test_run_rejects_unbound_identity_before_replay(monkeypatch, tmp_path, failure, instances):
    import json
    import sys
    from scripts import replay_navigation_trace as analyzer
    case = tmp_path / "121"
    case.mkdir()
    for name in ("peer.json", "identity.json", "navpy-navigation.csv", "navigation_debug.csv", "navigation_compact.csv"):
        (case / name).write_text("{}")
    identity = {"runtime": {"python": "wrong" if failure == "environment" else sys.version, "packages": {}}}
    monkeypatch.setattr(analyzer, "validate", lambda *a, **kw: ({"identity": identity}, []))
    (tmp_path / "fleet.json").write_text(json.dumps({"evidence_sha256": {}}))
    monkeypatch.setattr(analyzer, "replay", lambda *a: pytest.fail("replay must not start"))
    reason = "recorded Python" if failure == "environment" else "unbound fleet evidence"
    with pytest.raises(ValueError, match=reason):
        analyzer.run(case, tmp_path, instances, tmp_path / "output")


def test_replay_requires_declared_zero_interval(capture, tmp_path):
    peer, live = capture
    output = tmp_path / "replay"
    replay(peer, output)
    debug = output / "navigation_debug.csv"
    debug.write_text(debug.read_text().replace("min_log_interval_s=0.0", "min_log_interval_s=0.1"))
    with pytest.raises(ValueError, match="configuration missing"):
        verify_live_rows(live, output)
