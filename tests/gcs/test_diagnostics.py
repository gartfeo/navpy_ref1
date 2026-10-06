"""Regression tests for bounded GCS operator session diagnostics."""
from __future__ import annotations

import json
import asyncio
import zipfile
from unittest.mock import AsyncMock, patch

from fastapi.testclient import TestClient

from gcs.backend import diagnostics


def _records(session: diagnostics.DiagnosticSession) -> list[dict]:
    records = []
    for path in session.timeline_files():
        records.extend(json.loads(line) for line in path.read_text(encoding="utf-8").splitlines())
    return records


def test_emit_keeps_allowlist_and_drops_sensitive_payload(tmp_path):
    session = diagnostics.DiagnosticSession(tmp_path)
    session.start()
    session.emit(
        "parameter_snapshot_accepted",
        source="frontend",
        sys_id=161,
        request_id="request-1",
        record_count=742,
        latitude=40.123,
        longitude=44.456,
        parameter_value=123,
        device="udp:192.0.2.1:14550",
        body={"secret": True},
    )

    record = _records(session)[-1]
    assert record["record_count"] == 742
    assert record["request_id"] == "request-1"
    for forbidden in ("latitude", "longitude", "parameter_value", "device", "body"):
        assert forbidden not in record


def test_rotation_retains_only_current_and_one_rollover(tmp_path, monkeypatch):
    monkeypatch.setattr(diagnostics, "MAX_FILE_BYTES", 240)
    session = diagnostics.DiagnosticSession(tmp_path)
    session.start()
    for index in range(30):
        session.emit("http_request_finished", request_id=f"request-{index}", outcome="success")

    names = sorted(path.name for path in session.timeline_files())
    assert names == ["timeline.1.jsonl", "timeline.jsonl"]
    assert all(path.stat().st_size <= 300 for path in session.timeline_files())


def test_export_contains_manifest_summary_and_timeline(tmp_path):
    session = diagnostics.DiagnosticSession(tmp_path / "sessions")
    session.start()
    session.emit("vehicle_link_state", source="backend", sys_id=161, outcome="connected")
    session.emit(
        "mission_result_accepted", source="frontend", sys_id=161,
        request_id="mission-161", outcome="accepted", total=9,
    )
    session.emit(
        "plan_rendered", source="frontend", zone_count=1,
        zone_sys_ids=[161], waypoint_counts=[9],
    )
    target = tmp_path / "diagnostics.zip"
    session.export_zip(target)

    with zipfile.ZipFile(target) as archive:
        assert {"timeline.jsonl", "manifest.json", "summary.json"} <= set(archive.namelist())
        manifest = json.loads(archive.read("manifest.json"))
        summary = json.loads(archive.read("summary.json"))
    assert "parameter names/values" in manifest["privacy"]
    assert summary["vehicles"]["161"]["mission"]["client"]["request_id"] == "mission-161"
    assert summary["final_plan"] == {
        "zone_count": 1, "zone_sys_ids": [161], "waypoint_counts": [9],
    }


def test_frontend_event_allowlist_is_narrow():
    assert "plan_rendered" in diagnostics.ALLOWED_FRONTEND_EVENTS
    assert "browser_console" not in diagnostics.ALLOWED_FRONTEND_EVENTS
    assert "raw_telemetry" not in diagnostics.ALLOWED_FRONTEND_EVENTS


def test_storage_failure_disables_recording_without_escaping(tmp_path):
    session = diagnostics.DiagnosticSession(tmp_path)
    session.start()
    timeline = session.session_dir / "timeline.jsonl"
    timeline.unlink()
    timeline.mkdir()

    session.emit("vehicle_link_state", sys_id=161, outcome="connected")

    assert session.active is True
    assert session.recording_enabled is False
    # Further request-path events are harmless no-ops.
    session.emit("mission_request_started", sys_id=161, request_id="req-1")


def test_start_storage_failure_is_non_fatal(tmp_path):
    root = tmp_path / "not-a-directory"
    root.write_text("occupied", encoding="utf-8")
    session = diagnostics.DiagnosticSession(root)

    assert session.start() == ""
    assert session.active is False
    assert session.recording_enabled is False


def test_navpy_disappearance_clears_dedup_so_restart_is_recorded():
    from gcs.backend.routes import navpy_sim

    navpy_sim._last_diagnostic_status.clear()
    snapshots = [
        [{"sys_id": 161, "running": True, "exit_code": None}],
        [],
        [{"sys_id": 161, "running": True, "exit_code": None}],
    ]
    with patch.object(navpy_sim.runtime, "has_manager", return_value=True), \
         patch.object(navpy_sim.runtime, "get_all_status", side_effect=snapshots), \
         patch.object(navpy_sim.ws_manager, "broadcast", new=AsyncMock()), \
         patch.object(navpy_sim, "emit") as emit_mock:
        asyncio.run(navpy_sim.broadcast_sim_status())
        asyncio.run(navpy_sim.broadcast_sim_status())
        asyncio.run(navpy_sim.broadcast_sim_status())

    states = [(call.kwargs["running"], call.kwargs["outcome"]) for call in emit_mock.call_args_list]
    assert states == [(True, "running"), (False, "stopped"), (True, "running")]


def test_two_of_three_timeline_correlates_timeout_with_backend_late_success(tmp_path):
    session = diagnostics.DiagnosticSession(tmp_path)
    session.start()
    for sys_id in (161, 162):
        session.emit("mission_result_accepted", source="frontend", sys_id=sys_id,
                     request_id=f"req-{sys_id}", outcome="accepted", total=9)
    session.emit("client_timeout", source="frontend", sys_id=163,
                 request_id="req-163", phase="mission", outcome="timeout")
    session.emit("mission_result_rejected", source="frontend", sys_id=163,
                 request_id="req-163", outcome="timeout")
    session.emit("plan_rendered", source="frontend", zone_count=2,
                 zone_sys_ids=[161, 162], waypoint_counts=[9, 9])
    session.emit("mission_backend_finished", source="backend", sys_id=163,
                 request_id="req-163", outcome="success", total=9)

    records = _records(session)
    correlated = [r for r in records if r.get("request_id") == "req-163"]
    assert [r["event"] for r in correlated] == [
        "client_timeout", "mission_result_rejected", "mission_backend_finished",
    ]
    assert correlated[-1]["outcome"] == "success"
    summary = session.summary()
    assert summary["vehicles"]["163"]["mission"]["client"]["outcome"] == "timeout"
    assert summary["vehicles"]["163"]["mission"]["backend"]["outcome"] == "success"
    assert summary["final_plan"]["zone_sys_ids"] == [161, 162]


def test_http_middleware_persists_normalized_404_without_query_or_body(tmp_path, monkeypatch):
    from gcs.backend.main import app

    session = diagnostics.diagnostic_session
    monkeypatch.setattr(session, "root", tmp_path)
    session.session_dir = None
    session.start()
    try:
        response = TestClient(app).get(
            "/api/vehicles/999/mission?secret=value",
            headers={"X-GCS-Request-ID": "http-404", "X-GCS-Client-ID": "client-1"},
        )
        assert response.status_code == 404
        assert response.headers["X-GCS-Request-ID"] == "http-404"
        records = _records(session)
    finally:
        session.stop()

    finished = next(
        record for record in records
        if record["event"] == "http_request_finished" and record.get("request_id") == "http-404"
    )
    assert finished["route"] == "/api/vehicles/{sys_id}/mission"
    assert finished["status_code"] == 404
    serialized = json.dumps(finished)
    assert "secret" not in serialized
    assert "Vehicle 999 not connected" not in serialized


def test_frontend_event_rejects_unknown_or_sensitive_field_shapes():
    from gcs.backend.main import app

    client = TestClient(app)
    valid = {
        "event": "client_timeout",
        "fields": {
            "client_id": "client-1", "client_seq": 1, "request_id": "request-1",
            "sys_id": 161, "phase": "mission", "outcome": "timeout",
        },
    }
    assert client.post("/api/diagnostics/events", json=valid).status_code == 204

    connection_string = json.loads(json.dumps(valid))
    connection_string["fields"]["request_id"] = "udp:192.0.2.1:14550"
    assert client.post("/api/diagnostics/events", json=connection_string).status_code == 422

    coordinates = json.loads(json.dumps(valid))
    coordinates["fields"]["latitude"] = 40.123
    assert client.post("/api/diagnostics/events", json=coordinates).status_code == 422

    missing_sequence = json.loads(json.dumps(valid))
    del missing_sequence["fields"]["client_seq"]
    assert client.post("/api/diagnostics/events", json=missing_sequence).status_code == 422
