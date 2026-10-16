"""9e acceptance: the backend participant, driven by the operator's setup calls.

The real backend runs as a participant process under the 9d coordinator,
against the reference autopilot's byte stream through the router (``lockstep:router``). A
scripted API actor makes the operator's setup calls for the slice fixture
(connect ``lockstep:router``, the command bounds read by name, the AAS
parameter writes, the scenario mission's upload and readback) through the
lockstep ports. The serial1 bytes, the responses and the event journal
repeat byte for byte across runs under two slots, each claimed through
``gcs_launch.lockstep_claim``'s registry claim on a temp registry, and with
host delays; so do the backend's log stream (``DONE.log_lines``), its log
file and its diagnostics timeline. Drain ends the backend with its strict
shutdown, flushed.
"""

from __future__ import annotations

import json

import pytest

from gcs.backend import instance_registry_lockstep as lockstep
from gcs.backend.aas_params import AAS_PARAM_MAP
from scripts.lockstep_slice_actor import BACKEND
from tests.modules.lockstep.backend_doubles import BOUND_VALUES, build


def _claim() -> tuple[int, dict]:
    return lockstep.claim(run_token="seed-1", clone="clone", owner="dir:test", port_check=lambda n: True)


def _run(tmp_path, monkeypatch, name: str, slot: int, delay_seed: int = 0):
    # The slot reaches the coordinator's host side only: a participant's
    # environment is hermetic, so its payloads cannot depend on it.
    monkeypatch.setenv("GCS_CHAT_INDEX", str(slot))
    run = build(tmp_path / name, delay_seed=delay_seed)
    result = run.run()
    return run.coordinator, run.sitl, run.operator, result


def _artifacts(run_dir) -> dict[str, bytes]:
    """The backend's log file and diagnostics timeline, by run-relative path."""
    gcs = run_dir / "host" / "gcs"
    return {path.relative_to(gcs).as_posix(): path.read_bytes()
            for path in sorted(gcs.rglob("*")) if path.is_file()}


def test_the_setup_calls_repeat_byte_for_byte_under_two_slots_and_host_delays(tmp_path, monkeypatch):
    monkeypatch.setenv("GCS_INSTANCE_REGISTRY", str(tmp_path / "instances.json"))
    (first, first_owner), (second, second_owner) = _claim(), _claim()  # both held: two slots
    assert first != second
    try:
        plain = _run(tmp_path, monkeypatch, "first", first)
        other = _run(tmp_path, monkeypatch, "second", second, delay_seed=5)
    finally:
        assert lockstep.release_if_owner(first, first_owner)
        assert lockstep.release_if_owner(second, second_owner)
    for run in (plain, other):
        coordinator, sitl, operator, result = run
        assert result.outcome == "timeout"  # no NavPy: the mission phase ends on its budget
        assert operator.done and operator.result["aas"]
    assert plain[0].journal.records == other[0].journal.records
    assert plain[0].journal.digest == other[0].journal.digest
    assert plain[1].serial1 == other[1].serial1  # serial1 both ways, tick by tick
    assert plain[1].snapshot_rows == other[1].snapshot_rows
    assert plain[2].result == other[2].result
    assert plain[2].journal == other[2].journal
    # The backend's log stream (the operator's backend tail), log file and diagnostics.
    log = plain[2].journal.log_lines[BACKEND]
    assert log == other[2].journal.log_lines[BACKEND]
    assert any(line.endswith("Backend file logging -> gcs/backend.log") for line in log)
    assert any(line.endswith("Vehicle 1 (s1-u1) connected") for line in log)
    assert not [line for line in log if str(tmp_path) in line or tmp_path.as_posix() in line]
    artifacts = _artifacts(tmp_path / "first")
    assert artifacts == _artifacts(tmp_path / "second")
    assert {"backend.log"} < set(artifacts) and any(p.endswith("timeline.jsonl") for p in artifacts)

    coordinator, sitl, operator, result = plain
    autopilot = sitl.autopilot
    # The backend's writes reached the autopilot, and the mission is the scenario's.
    written = {AAS_PARAM_MAP[field]: float(value) for field, value in operator.result["aas"]["1"].items()}
    assert {name: autopilot.parameters[name] for name in written} == pytest.approx(written)
    assert len(autopilot.mission) > 6
    kinds = {kind for port, kind, source, _c in autopilot.received if port == 1 and source == 255}
    assert {"HEARTBEAT", "PARAM_REQUEST_READ", "PARAM_SET", "MISSION_CLEAR_ALL", "MISSION_COUNT",
            "MISSION_ITEM_INT", "MISSION_REQUEST_LIST"} <= kinds
    assert not any(port == 0 for port, *_ in autopilot.received)  # serial0 had no participant
    # Bounds were read by name, fresh (never MAVFTP).
    bounds = operator.result["bounds"]["1"]  # the setup keys its results by sysid (10e)
    assert "-25.0" in bounds and str(BOUND_VALUES["TRIM_THROTTLE"] / 100) in bounds
    types = [json.loads(event)["type"] for event in operator.result["events"]]
    assert "telemetry" in types and "upload_progress" in types
    # Drain: the backend ended flushed, with the strict shutdown, and exited 0.
    records = coordinator.journal.records
    ends = [r for r in records if r.get("phase") == "drain"]
    assert ends
    host = json.loads((tmp_path / "first" / "lockstep_participants_host.json").read_text())
    assert [row["exit_code"] for row in host] == [0]
    assert result.parked_tick == result.final_tick + 1
