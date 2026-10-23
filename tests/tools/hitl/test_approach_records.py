"""The run record of a HITL approach (tools/hitl/approach_records.py) and Layer 1's model input
(tools/hitl/model_input.py): synthetic valid, missing and mismatched records; no model is edited."""
from __future__ import annotations

import copy
import json
import os
import subprocess
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from tests.tools.hitl import media_fakes
from tools.hitl import approach_records as ar
from tools.hitl import model_input as mi
from tools.hitl.json_checks import RecordError, sha256_file

CREATED = datetime(2026, 10, 19, 8, 30, 5, tzinfo=timezone(timedelta(hours=2)))
SOURCES = {"navpy": {"git_head": "a" * 40, "uncommitted_diff_sha256": "b" * 64, "untracked_sha256": "c" * 64}}
BRIDGE = ar.BridgeRun(plant="program", jsbsim_version="synthetic-1.0", integration_dt_s=1.0 / 1200.0,
                      loop_rate_hz=200.0, servo_speed_s=0.03)


@pytest.fixture(autouse=True)
def media_runtime(tmp_path, monkeypatch):
    """A passing pc media runtime result over a synthetic install (test_check_media_runtime.py)."""
    global MEDIA
    MEDIA = media_fakes.passing_pc_result(tmp_path / "media", monkeypatch)
    return MEDIA


@pytest.fixture
def model_dir(tmp_path):
    folder = tmp_path / "models" / "Synth1"
    (folder / "Engines").mkdir(parents=True)
    (folder / "Synth1.xml").write_text("<fdm_config name='Synth1'/>\n", encoding="utf-8")
    (folder / "Engines" / "motor.xml").write_text("<turbine_engine/>\n", encoding="utf-8")
    return folder


def declaration(folder: Path) -> dict:
    return declaration_of(folder.name, mi.model_files_sha256(folder))


def declaration_of(name: str, files_sha256: dict) -> dict:
    return {
        "schema_version": 1, "name": name, "version": "synthetic-v1",
        "files_sha256": files_sha256,
        "jsbsim_versions": {"program": "synthetic-1.0"},
        "control_map": dict(mi.BRIDGE_CONTROL_MAP), "control_inputs": dict(mi.BRIDGE_CONTROL_INPUTS),
        "max_integration_dt_s": 0.005,
        "actuator_lag": {"owner": "bridge", "bridge_servo_speed_s": 0.03},
        "envelope": {"eas_m_s": {"min": 10.0, "max": 20.0}},
    }


def make(folder: Path, **changes) -> dict:
    kwargs = dict(run_id="r001-pc-sitl", mode="pc-sitl", created=CREATED, model_input=declaration(folder),
                  model_dir=folder, bridge=BRIDGE, law="vision-nav-pn", sources=SOURCES,
                  media_runtime=MEDIA)
    kwargs.update(changes)
    return ar.new_record(**kwargs)


def link_dir(target: Path, link: Path) -> None:
    """A directory link: a symbolic link, or a junction where Windows refuses symbolic links."""
    try:
        os.symlink(target, link, target_is_directory=True)
    except OSError:
        if os.name != "nt":
            raise
        import _winapi
        _winapi.CreateJunction(str(target), str(link))


def key_paths(data, prefix=()):
    for key, value in data.items():
        yield prefix + (key,)
        if isinstance(value, dict):
            yield from key_paths(value, prefix + (key,))


def without(data: dict, path: tuple) -> dict:
    out = copy.deepcopy(data)
    parent = out
    for key in path[:-1]:
        parent = parent[key]
    del parent[path[-1]]
    return out


def edited(data: dict, path: tuple, value) -> dict:
    out = copy.deepcopy(data)
    parent = out
    for key in path[:-1]:
        parent = parent[key]
    parent[path[-1]] = value
    return out


def refused(check, data) -> str:
    with pytest.raises(RecordError) as caught:
        check(data)
    return "; ".join(caught.value.problems)


def test_valid_record_is_one_complete_manifest_per_run(model_dir, tmp_path):
    record = make(model_dir)
    assert record["run"] == {"run_id": "r001-pc-sitl", "mode": "pc-sitl", "created_utc": "2026-10-19T06:30:05Z"}
    assert record["substitutions"] == dict(ar.MODE_SUBSTITUTIONS["pc-sitl"])
    assert record["bridge"]["sensor_map"] == mi.BRIDGE_SENSOR_MAP
    private = tmp_path / "private"
    path = ar.write_record(record, private / "runs" / "r001-pc-sitl", private_root=private)
    assert path.name == ar.RECORD_FILE and ar.read_record(path) == record
    assert "one record per run" in refused(lambda r: ar.write_record(r, path.parent, private_root=private), record)
    assert json.loads(path.read_text(encoding="utf-8")) == record


def test_extra_substitutions_are_kept_and_mode_ones_cannot_be_dropped(model_dir):
    record = make(model_dir, extra_substitutions={"airspeed": "JSBSim calibrated airspeed"})
    assert record["substitutions"]["airspeed"] == "JSBSim calibrated airspeed"
    assert "substitutions.gimbal is missing for mode pc-sitl" in refused(
        ar.validate_record, without(record, ("substitutions", "gimbal")))
    assert "substitutions.gimbal 'the real ZR10' is not mode pc-sitl's" in refused(
        ar.validate_record, edited(record, ("substitutions", "gimbal"), "the real ZR10"))
    with pytest.raises(RecordError, match="is not mode pc-sitl's"):
        make(model_dir, extra_substitutions={"gimbal": "the real ZR10"})


def test_every_missing_field_is_refused(model_dir):
    record = make(model_dir)
    # Every key but one: a file entry other than the model file leaves a self-consistent record; the
    # model folder check refuses it instead (test_model_folder_must_match_the_declaration).
    optional = {("model_input", "files_sha256", "Engines/motor.xml")}
    paths = [p for p in key_paths(record) if p not in optional]
    assert len(paths) > 60
    for path in paths:
        refused(ar.validate_record, without(record, path))
    for path in key_paths(record["model_input"]):
        if ("model_input",) + path not in optional:
            refused(mi.validate_model_input, without(record["model_input"], path))


def test_missing_fields_are_named(model_dir):
    record = make(model_dir)
    assert "record.run.created_utc is missing" in refused(ar.validate_record, without(record, ("run", "created_utc")))
    assert "record.sources.navpy is missing" in refused(
        ar.validate_record, edited(record, ("sources",), {"other": SOURCES["navpy"]}))
    assert "record.model_input.envelope is missing" in refused(
        ar.validate_record, without(record, ("model_input", "envelope")))
    assert "record.model_input.envelope must be a non-empty object" in refused(
        ar.validate_record, edited(record, ("model_input", "envelope"), {}))


@pytest.mark.parametrize("path, value, reason", [
    (("schema_version",), True, "must be 2"),
    (("schema_version",), 2.0, "must be 2"),
    (("schema_version",), 1, "must be 2"),
    (("run", "run_id"), "run 1", "must be a name"),
    (("run", "run_id"), "CON", "not a Windows device name"),
    (("run", "run_id"), "aux.r001", "not a Windows device name"),
    (("run", "run_id"), "com1", "not a Windows device name"),
    (("run", "run_id"), "r001.", "not ending in '.'"),
    (("model_input", "name"), "NUL", "not a Windows device name"),
    (("run", "created_utc"), "2026-10-19 06:30:05", "must be a UTC time"),
    (("run", "surprise"), 1, "record.run.surprise is not a known field"),
    (("sources", "navpy", "git_head"), "A" * 40, "40 lowercase hex digits"),
    (("model_input", "files_sha256", "Synth1.xml"), "0" * 63, "64 lowercase hex digits"),
    (("model_input", "control_map", "throttle"), -1, "whole number >= 0"),
    (("model_input", "control_map", "throttle"), True, "whole number >= 0"),
    (("model_input", "jsbsim_versions", "binary"), "1", "unknown entry 'binary'"),
    (("model_input", "envelope", "eas_m_s", "max"), "20", "must be a finite number"),
    (("model_input", "max_integration_dt_s"), 0, "above 0"),
    (("model_input", "max_integration_dt_s"), 10 ** 400, "above 0"),
    (("model_input", "envelope", "eas_m_s", "min"), -10 ** 400, "must be a finite number"),
    (("bridge", "loop_rate_hz"), -200.0, "above 0"),
    (("law", "algorithm"), "", "non-empty text"),
])
def test_malformed_fields_are_refused(model_dir, path, value, reason):
    assert reason in refused(ar.validate_record, edited(make(model_dir), path, value))


@pytest.mark.parametrize("path, value, reason", [
    (("model_input", "control_map"), {"left_elevon": 1, "right_elevon": 0, "throttle": 2}, "is not the bridge's"),
    (("model_input", "control_inputs", "aileron"), "fcs/roll-cmd-norm", "are not the bridge's"),
    (("bridge", "sensor_map", "ind_airspeed"), "velocities/vtrue-fps", "not the bridge's state properties"),
    (("bridge", "plant"), "package", "not a version the model was validated with (none for this plant)"),
    (("bridge", "jsbsim_version"), "synthetic-2.0", "not a version the model was validated with"),
    (("bridge", "integration_dt_s"), 0.0007, "is not 1/N s for a whole N"),
    (("bridge", "integration_dt_s"), 0.01, "longer than the model's maximum"),
    (("bridge", "integration_dt_s"), 5e-324, "is too short to be 1/N s"),
    (("bridge", "servo_speed_s"), 0.14, "is not the model's 0.03"),
    (("model_input", "actuator_lag"), {"owner": "model", "bridge_servo_speed_s": 0.03}, "applied twice"),
    (("model_input", "actuator_lag"), {"owner": "bridge", "bridge_servo_speed_s": 0.0}, "must be above 0"),
    (("model_input", "envelope", "eas_m_s"), {"min": 20.0, "max": 20.0}, "is not below max"),
    (("model_input", "name"), "Other", "lacks the model file Other.xml"),
    (("model_input", "files_sha256", "../Synth1.xml"), "0" * 64, "not a plain relative POSIX path"),
    (("model_input", "files_sha256", "Engines\\motor.xml"), "0" * 64, "not a plain relative POSIX path"),
    (("law", "algorithm"), "pn", "is not an approach law"),
    (("law", "algorithm"), "pid", "is not an approach law"),
    (("run", "mode"), "jetson-sitl", "must be one of pc-sitl"),
])
def test_mismatched_records_are_refused(model_dir, path, value, reason):
    assert reason in refused(ar.validate_record, edited(make(model_dir), path, value))


def test_model_folder_must_match_the_declaration(model_dir, tmp_path):
    declared = declaration(model_dir)
    (model_dir / "Engines" / "motor.xml").write_text("<turbine_engine thrust='more'/>\n", encoding="utf-8")
    (model_dir / "notes.txt").write_text("not declared\n", encoding="utf-8")
    (model_dir / "Synth1.xml").unlink()
    reason = refused(lambda d: mi.check_model_files(d, model_dir), declared)
    assert "model file Synth1.xml is declared but missing" in reason
    assert "model file notes.txt is not declared" in reason
    assert "model file Engines/motor.xml differs from its declared sha256" in reason
    with pytest.raises(RecordError, match="differs from its declared sha256"):
        make(model_dir, model_input=declared)
    renamed = tmp_path / "Synth2"
    model_dir.rename(renamed)
    assert "model folder 'Synth2' is not the declared model 'Synth1'" in refused(
        lambda d: mi.check_model_files(d, renamed), declaration(renamed) | {"name": "Synth1"})
    assert "no model folder" in refused(lambda d: mi.check_model_files(d, tmp_path / "absent"), declared)


def test_links_in_the_model_folder_are_refused(model_dir, tmp_path):
    outside = tmp_path / "outside_engines"
    outside.mkdir()
    (outside / "motor.xml").write_text("<turbine_engine thrust='elsewhere'/>\n", encoding="utf-8")
    link = model_dir / "Linked"
    link_dir(outside, link)
    try:
        assert "model folder entry Linked is a link" in refused(mi.model_files_sha256, model_dir)
        assert "is a link" in refused(lambda d: mi.check_model_files(d, model_dir), declaration_without_link(model_dir))
    finally:
        os.unlink(link) if link.is_symlink() else os.rmdir(link)
    assert "Linked/motor.xml" not in mi.model_files_sha256(model_dir)


def declaration_without_link(folder: Path) -> dict:
    """The fixture's declaration with its two plain files, built without hashing the folder."""
    return declaration_of(folder.name, {"Synth1.xml": sha256_file(folder / "Synth1.xml"),
                                        "Engines/motor.xml": sha256_file(folder / "Engines" / "motor.xml")})


def test_records_and_checks_never_edit_the_model(model_dir, tmp_path):
    def snapshot():
        return {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in sorted(model_dir.rglob("*")) if p.is_file()}
    before = snapshot()
    record = make(model_dir)
    mi.check_model_files(record["model_input"], model_dir)
    ar.write_record(record, tmp_path / "private" / "runs" / "r001", private_root=tmp_path / "private")
    assert snapshot() == before


def test_declaration_file_round_trip_and_non_numbers(model_dir, tmp_path):
    path = tmp_path / "model_input.json"
    path.write_text(json.dumps(declaration(model_dir)), encoding="utf-8")
    assert mi.read_model_input(path) == declaration(model_dir)
    path.write_text(json.dumps(declaration(model_dir)).replace('"max": 20.0', '"max": NaN'), encoding="utf-8")
    assert "not valid JSON" in refused(mi.read_model_input, path)
    assert "no file" in refused(mi.read_model_input, tmp_path / "absent.json")


def test_record_folder_must_be_in_the_private_folder(model_dir, tmp_path, monkeypatch):
    record = make(model_dir)
    private = tmp_path / "private"
    monkeypatch.setenv("NAVPY_HITL_TOOLS_DIR", str(private))
    assert ar.write_record(record, private / "runs" / "r001").parent == (private / "runs" / "r001").resolve()
    other_checkout = tmp_path / "other_checkout" / ".logs" / "r001"
    assert "not in the private folder" in refused(lambda r: ar.write_record(r, other_checkout), record)
    in_repo = ar.REPO_ROOT / ".logs" / "approach-record-refused-test"
    assert "not in the private folder" in refused(lambda r: ar.write_record(r, in_repo), record)
    for source in (ar.NAVPY_SOURCE, Path("no-such-source")):     # this repository is refused even without its marker
        monkeypatch.setattr(ar, "NAVPY_SOURCE", source)
        assert f"is in navpy checkout {ar.REPO_ROOT}" in refused(
            lambda r: ar.write_record(r, in_repo, private_root=ar.REPO_ROOT), record)
    assert not other_checkout.exists() and not in_repo.exists()


def git_in(repo: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(repo), "-c", "user.name=t", "-c", "user.email=t@example.invalid",
                           *args], check=True, capture_output=True).stdout.decode().strip()


def committed_repo(repo: Path) -> Path:
    """A synthetic git repository with one commit of its own."""
    repo.mkdir()
    git_in(repo, "init", "-q")
    (repo / "a.txt").write_text(f"{repo.name}\n", encoding="utf-8")
    git_in(repo, "add", "a.txt")
    git_in(repo, "commit", "-q", "-m", "one")
    return repo.resolve()


def test_record_folder_in_any_navpy_checkout_is_refused(model_dir, tmp_path, monkeypatch):
    """Another checkout is found by its source tree, so git need not read it; a private git repository is kept."""
    checkout = tmp_path.resolve() / "other_checkout"
    (checkout / ar.NAVPY_SOURCE).mkdir(parents=True)
    private_repo = committed_repo(tmp_path / "private_repo")
    record = make(model_dir)
    monkeypatch.setenv("NAVPY_HITL_TOOLS_DIR", str(checkout / "private"))
    for run_dir, root in ((checkout / "private" / "r001", None), (checkout / "runs" / "r001", checkout)):
        assert f"is in navpy checkout {checkout}" in refused(
            lambda r: ar.write_record(r, run_dir, private_root=root), record)
        assert not run_dir.parent.exists()
    assert ar.write_record(record, private_repo / "runs" / "r001", private_root=private_repo).is_file()


def test_created_time_needs_its_zone(model_dir):
    with pytest.raises(RecordError, match="time zone"):
        make(model_dir, created=datetime(2026, 10, 19, 8, 30))


def test_source_identity_follows_commit_diff_and_untracked_files(tmp_path):
    repo = committed_repo(tmp_path / "repo")

    def git(*args):
        return git_in(repo, *args)
    clean = ar.source_identity(repo)
    assert clean["git_head"] == git("rev-parse", "HEAD")
    (repo / "a.txt").write_text("two\n", encoding="utf-8")
    edited_source = ar.source_identity(repo)
    assert edited_source["uncommitted_diff_sha256"] != clean["uncommitted_diff_sha256"]
    (repo / "b.txt").write_text("new\n", encoding="utf-8")
    added = ar.source_identity(repo)
    assert added["untracked_sha256"] != edited_source["untracked_sha256"]
    assert sha256_file(repo / "b.txt") != added["untracked_sha256"]
    with pytest.raises(RecordError, match="not a git repository"):
        ar.source_identity(tmp_path)


def test_media_runtime_is_frozen_in_the_record(model_dir):
    record = make(model_dir)
    assert record["media_runtime"] == MEDIA
    assert "record.media_runtime is missing" in refused(ar.validate_record, without(record, ("media_runtime",)))


@pytest.mark.parametrize("path, value, reason", [
    (("verdict",), "blocked", "media_runtime.verdict must be 'pass'"),
    (("blockers",), ["no GStreamer root"], "media_runtime.blockers must be []"),
    (("host",), "orin", "media_runtime.host must be one of pc"),
    (("surprise",), 1, "media_runtime.surprise is not a known field"),
    (("python", "platform"), "win32", "must be 'win-amd64'"),
    (("python", "gi_extensions"), [], "gi_extensions must be a non-empty list"),
    (("gstreamer", "nano"), 1, "nano must be 0"),
    (("gstreamer", "version"), "1.26.10", "gstreamer.version 1.26.10 is not the pinned 1.26.11"),
    (("gstreamer", "pinned"), "1.28.0", "gstreamer.pinned 1.28.0 is not the pinned 1.26.11"),
    (("installers",), {"gstreamer-1.0-msvc-x86_64-1.26.11.msi": "0" * 64}, "are not the official 1.26.11"),
    (("selected", "encoder"), "openh264enc", "selected.encoder openh264enc is not one of"),
    (("selected", "decoder"), "nvv4l2decoder", "selected.decoder nvv4l2decoder is not one of"),
    (("elements", "rtph265pay", "version"), "1.24.0", "elements.rtph265pay.version 1.24.0 is not 1.26.11"),
    (("elements", "rtph265pay", "rank"), -1, "whole number >= 0"),
    (("gpu_driver",), "", "non-empty text"),
])
def test_media_runtime_that_is_not_a_pinned_pass_is_refused(model_dir, path, value, reason):
    record = make(model_dir)
    assert reason in refused(ar.validate_record, edited(record, ("media_runtime",) + path, value))


def test_media_runtime_needs_every_element_it_selected(model_dir):
    record = make(model_dir)
    assert "media_runtime.elements are" in refused(
        ar.validate_record, without(record, ("media_runtime", "elements", "nvh265enc")))
    assert "media_runtime.elements are" in refused(
        ar.validate_record, edited(record, ("media_runtime", "selected", "encoder"), "x265enc"))
