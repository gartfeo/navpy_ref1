"""Step 13's run manifest (tools/hitl/run_manifest.py): one run's files from one set of producers, each ended."""
from __future__ import annotations

import json

import pytest

from tools.hitl import run_manifest as rm

RUN = "run-1"


def _files(tmp_path):
    folder = tmp_path / "run"
    (folder / "pub").mkdir(parents=True)
    files = {
        "run_record": folder / "approach_run.json",
        "navpy_integration_log": folder / "navpy.jsonl",
        "publisher_status": folder / "pub" / "publisher.json",
        "publisher_frames": folder / "pub" / "frames-001.csv",
        "unreal_log": folder / "unreal.log",
        "emulator_log": folder / "emulator.jsonl",
    }
    files["run_record"].write_text('{"run": {"run_id": "run-1"}}\n', encoding="utf-8")
    files["navpy_integration_log"].write_text(
        '{"event":"gimbal_receipt","run_id":"run-1","session":1,"seq":1,"receipt_s":1.0}\n'
        '{"event":"log_end","run_id":"run-1","records":1}\n', encoding="utf-8")
    files["publisher_status"].write_text(json.dumps({"exit_code": 0, "ended_monotonic_ns": 5}), encoding="utf-8")
    files["publisher_frames"].write_text("frame_index,outcome\n1,pushed\n", encoding="utf-8")
    files["unreal_log"].write_text("[x]LogExit: Exiting.\n[x]Log file closed, 11/03/26\n", encoding="utf-8")
    files["emulator_log"].write_text('{"event":"start"}\n{"event":"summary","valid":true}\n', encoding="utf-8")
    return files


def _producers():
    return {name: {"pid": 10 + index, "argv": [name], "exit_code": 0, "ended_after_stop": True}
            for index, name in enumerate(sorted(set(rm.FILE_PRODUCERS.values())))}


def _manifest(files, producers=None):
    return rm.build(RUN, _producers() if producers is None else producers, list(files.items()))


def test_a_whole_run_verifies(tmp_path):
    files = _files(tmp_path)
    manifest = _manifest(files)
    assert all(entry["ended"] for entry in manifest["files"])
    assert rm.verify(manifest, RUN, list(files.items()), {RUN}) == []


def test_a_log_cut_at_a_line_boundary_after_the_manifest_fails(tmp_path):
    files = _files(tmp_path)
    manifest = _manifest(files)
    navpy = files["navpy_integration_log"]
    navpy.write_text(navpy.read_text(encoding="utf-8").splitlines()[0] + "\n", encoding="utf-8")
    assert any("not a file of this run" in text for text in rm.verify(manifest, RUN, list(files.items()), {RUN}))


@pytest.mark.parametrize("role, text", [
    ("navpy_integration_log", '{"event":"gimbal_receipt","run_id":"run-1","session":1,"seq":1,"receipt_s":1.0}\n'),
    ("emulator_log", '{"event":"start"}\n'),
    ("unreal_log", "[x]LogExit: Exiting.\n"),
    ("publisher_status", json.dumps({"exit_code": None})),
    ("publisher_frames", "frame_index,outcome\n1,pus"),
])
def test_a_producer_cut_off_before_the_manifest_fails(tmp_path, role, text):
    files = _files(tmp_path)
    files[role].write_text(text, encoding="utf-8")
    manifest = _manifest(files)
    failures = rm.verify(manifest, RUN, list(files.items()), {RUN})
    assert failures == [f"{role} {files[role]} does not end cleanly (cut off)"]


def test_another_runs_files_or_producers_fail(tmp_path):
    files = _files(tmp_path)
    manifest = _manifest(files)
    assert any("not the record's run" in text for text in rm.verify(manifest, "run-2", list(files.items()), {RUN}))
    assert any("are not only" in text for text in rm.verify(manifest, RUN, list(files.items()), {RUN, "run-0"}))
    other = tmp_path / "other.jsonl"
    other.write_text('{"event":"start"}\n{"event":"summary","valid":false}\n', encoding="utf-8")
    inputs = [(role, other if role == "emulator_log" else path) for role, path in files.items()]
    assert any("emulator_log" in text and "not a file" in text for text in rm.verify(manifest, RUN, inputs, {RUN}))
    producers = _producers()
    producers["navpy"]["ended_after_stop"] = False
    del producers["unreal"]
    failures = rm.verify(_manifest(files, producers), RUN, list(files.items()), {RUN})
    assert "producer navpy did not end after the runner's stop" in failures and "manifest has no producer unreal" \
        in failures


def test_every_role_must_be_checked(tmp_path):
    files = _files(tmp_path)
    inputs = [(role, path) for role, path in files.items() if role != "unreal_log"]
    assert rm.verify(_manifest(files), RUN, inputs, {RUN}) == ["no unreal_log input was checked"]


def test_a_commissioning_fault_log_is_navpys_ended_by_its_close_and_never_required(tmp_path):
    files = _files(tmp_path)
    log = tmp_path / "run" / "media_fault.jsonl"
    log.write_text('{"event":"open"}\n{"event":"frame"}\n', encoding="utf-8")
    entry = rm.file_entry(log, "media_fault_log")
    assert entry["producer"] == "navpy" and entry["ended"] is False
    log.write_text('{"event":"open"}\n{"event":"close"}\n', encoding="utf-8")
    with_log = {**files, "media_fault_log": log}
    assert rm.verify(_manifest(with_log), RUN, list(with_log.items()), {RUN}) == []
    assert rm.verify(_manifest(files), RUN, list(files.items()), {RUN}) == []
