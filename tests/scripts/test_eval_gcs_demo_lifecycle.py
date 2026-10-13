"""One explicit lifecycle: ordered connect, a single companion start, acked stop."""

from __future__ import annotations

from types import SimpleNamespace
from urllib.parse import parse_qs, urlsplit

import pytest

from scripts import eval_gcs_demo_lifecycle as lifecycle
from scripts import eval_gcs_demo_process as process
from scripts.eval_gcs_demo_models import RegressionError


SYS_IDS = (7, 8, 9)


class FakeApi:
    def __init__(self, *, running=(), stop_outcome="acked"):
        self.running = set(running)
        self.stop_outcome = stop_outcome
        self.calls: list[tuple[str, object]] = []

    def post_json(self, path, payload=None, *, expected=(200,), timeout=30.0):
        del expected, timeout
        self.calls.append((path, payload))
        if path.startswith("/api/vehicles/connect?"):
            sys_id = int(parse_qs(urlsplit(path).query)["sys_id"][0])
            return 200, {"status": "connected", "sys_id": sys_id, "name": "x"}
        if path.endswith("start-all"):
            ids = [item["sys_id"] for item in payload["instances"]]
            self.running.update(ids)
            return 200, {"status": "started", "results": {
                str(sys_id): {"status": "started", "pid": 1} for sys_id in ids
            }}
        if path.endswith("stop-all"):
            outcomes = {str(sys_id): self.stop_outcome for sys_id in sorted(self.running)}
            self.running.clear()
            return 200, {"status": "stopped_all", "outcomes": outcomes}
        raise AssertionError(path)

    def get_json(self, path, *, timeout=30.0):
        del timeout
        assert path == "/api/control/navpy-sim/status"
        return {"instances": [
            {"sys_id": sys_id, "running": True} for sys_id in sorted(self.running)
        ]}


def _context(api, monitor="udp:0.0.0.0:15553"):
    return SimpleNamespace(api=api, sys_ids=SYS_IDS, monitor_device=monitor)


def test_connect_follows_manifest_order_on_the_slot_monitor():
    api = FakeApi()
    lifecycle.connect_vehicles(_context(api))

    queries = [parse_qs(urlsplit(path).query) for path, _ in api.calls]
    assert [int(query["sys_id"][0]) for query in queries] == list(SYS_IDS)
    assert {query["device"][0] for query in queries} == {"udp:0.0.0.0:15553"}


def test_connect_needs_the_monitor_device():
    with pytest.raises(RegressionError, match="monitor device"):
        lifecycle.connect_vehicles(_context(FakeApi(), monitor=None))


def test_companions_start_once_and_never_on_top_of_a_running_one():
    api = FakeApi()
    lifecycle.start_companions(_context(api), timeout_s=5.0)

    assert [path for path, _ in api.calls] == ["/api/control/navpy-sim/start-all"]
    assert api.running == set(SYS_IDS)
    with pytest.raises(RegressionError, match="already running"):
        lifecycle.start_companions(_context(api), timeout_s=5.0)


def test_acked_stop_passes_and_a_terminated_companion_is_a_host_fault():
    lifecycle.stop_companions(_context(FakeApi(running=SYS_IDS)))

    with pytest.raises(RegressionError, match="host fault"):
        lifecycle.stop_companions(
            _context(FakeApi(running=SYS_IDS, stop_outcome="terminated"))
        )


@pytest.mark.parametrize("action_fails", [False, True])
def test_teardown_stops_companions_before_the_stack(monkeypatch, action_fails):
    order: list[str] = []
    api = FakeApi(running=SYS_IDS)
    api.close = lambda: order.append("close")
    tail = SimpleNamespace(read_new=lambda: order.append("tail"))
    context = SimpleNamespace(api=api, sys_ids=SYS_IDS, backend_tail=tail,
                              monitor_device="udp:0.0.0.0:15553")
    monkeypatch.setattr(process, "launch_stack", lambda *_args: context)
    monkeypatch.setattr(process, "stop_companions",
                        lambda ctx: order.append("companions") or lifecycle.stop_companions(ctx))
    monkeypatch.setattr(process, "stop_stack", lambda _env: order.append("gcs_stop"))

    def action(_context):
        if action_fails:
            raise RuntimeError("mission failed")
        return "done"

    if action_fails:
        with pytest.raises(RegressionError, match="mission failed"):
            process.with_owned_stack({}, 1.0, action)
    else:
        assert process.with_owned_stack({}, 1.0, action) == "done"
    assert order == ["companions", "close", "gcs_stop", "tail"]
    assert api.running == set()


def test_a_second_launch_into_the_same_run_is_refused(tmp_path):
    log_dir = tmp_path / "gcs"
    assert process.backend_log_file({"GCS_LOG_DIR": str(log_dir)}) == log_dir / "backend.log"
    log_dir.mkdir()
    (log_dir / "backend.log").write_text("earlier generation\n", encoding="utf-8")
    with pytest.raises(RegressionError, match="already has a backend log"):
        process.backend_log_file({"GCS_LOG_DIR": str(log_dir)})
    with pytest.raises(RegressionError, match="GCS_LOG_DIR"):
        process.backend_log_file({})


def test_a_tracked_companion_without_an_outcome_is_a_host_fault():
    api = FakeApi(running=SYS_IDS)
    stop_all = api.post_json

    def partial_stop(path, payload=None, **kwargs):
        status, body = stop_all(path, payload, **kwargs)
        body["outcomes"].pop("9", None)
        return status, body

    api.post_json = partial_stop
    with pytest.raises(RegressionError, match="'9': 'no outcome'"):
        lifecycle.stop_companions(_context(api))
