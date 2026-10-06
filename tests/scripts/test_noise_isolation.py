"""Reject stale logs, ambiguous provenance and incomplete investigation evidence."""

import copy
import json

import pytest

from scripts import noise_log_artifacts as logs
from scripts import noise_isolation_profiles as profiles
from scripts.compare_noise_matrix import verify_ledger
from scripts.compare_lockstep_fleet import read_run
from scripts.read_noise_parameters import summarize


def inventories():
    before = dict(directory="/own/121", lastlog=1,
                  entries={"00000001.BIN": dict(size=5, mtime_ns=10, inode=1)})
    after = copy.deepcopy(before)
    after["lastlog"] = 2
    after["entries"]["00000002.BIN"] = dict(size=15, mtime_ns=20, inode=2)
    return before, after


def test_exactly_one_new_log_is_required():
    before, after = inventories()
    assert logs.new_log(before, after) == "00000002.BIN"


@pytest.mark.parametrize("mutation", ["none", "two", "changed", "removed", "counter", "owner"])
def test_ambiguous_or_reused_logs_reject(mutation):
    before, after = inventories()
    if mutation == "none":
        after = copy.deepcopy(before)
    elif mutation == "two":
        after["entries"]["00000003.BIN"] = {}
    elif mutation == "changed":
        after["entries"]["00000001.BIN"]["mtime_ns"] += 1
    elif mutation == "removed":
        del after["entries"]["00000001.BIN"]
    elif mutation == "counter":
        after["lastlog"] += 1
    else:
        after["directory"] = "/other/121"
    with pytest.raises(ValueError):
        logs.new_log(before, after)


def test_owned_lifecycle_copies_and_binds_log(tmp_path, monkeypatch):
    directory, case = tmp_path / "121", tmp_path / "case"
    (directory / "logs").mkdir(parents=True)
    case.mkdir()
    monkeypatch.setattr(logs, "owned_directory", lambda *a: directory)
    logs.run("before", tmp_path, 121, case)
    (directory / "logs/00000001.BIN").write_bytes(b"flight")
    (directory / "logs/LASTLOG.TXT").write_text("1\n")
    result = logs.run("after", tmp_path, 121, case)
    assert (case / "flight.BIN").read_bytes() == b"flight"
    assert result["source"] == str(directory / "logs/00000001.BIN")
    assert json.loads((case / "logs-after.json").read_text())["lastlog"] == 1


@pytest.mark.parametrize("records", [[], [{"name": "SIM_RATE_HZ", "value": 1200, "time_us": 1}],
    [{"name": "SIM_RATE_HZ", "value": 1000, "time_us": 1},
     {"name": "SIM_RATE_HZ", "value": 1200, "time_us": 2}]])
def test_missing_or_conflicting_parameters_reject(records):
    with pytest.raises(ValueError, match="missing or conflicting"):
        summarize(records, {"SIM_RATE_HZ": 1000})


def test_parameter_counts_and_first_time_are_retained():
    rows = [dict(name="SIM_RATE_HZ", value=1000, time_us=t) for t in (1, 2)]
    assert summarize(rows, {"SIM_RATE_HZ": 1000}) == {
        "SIM_RATE_HZ": dict(value=1000, count=2, first_time_us=1)}


def ledger():
    return dict(version=1, attempts=[dict(cell, directory=str(i), status="captured")
                                   for i, cell in enumerate(profiles.cells())])


def test_matrix_declares_each_interleaved_cell_once():
    attempts = verify_ledger(ledger())
    assert len(attempts) == 12
    assert all(a["noise_profile"] != b["noise_profile"] for a, b in zip(attempts, attempts[1:]))


@pytest.mark.parametrize("kind", ["missing", "rejected", "reorder", "duplicate"])
def test_matrix_rejects_incomplete_or_rearranged_attempts(kind):
    value = ledger()
    attempts = value["attempts"]
    if kind == "missing":
        attempts.pop()
    elif kind == "rejected":
        attempts[0]["status"] = "rejected"
    elif kind == "reorder":
        attempts[0], attempts[1] = attempts[1], attempts[0]
    else:
        attempts[1]["directory"] = attempts[0]["directory"]
    with pytest.raises(ValueError):
        verify_ledger(value)


def test_investigation_cannot_enter_normal_repeatability_matrix(tmp_path):
    (tmp_path / "fleet.json").write_text(json.dumps(dict(noise_profile="stock")))
    with pytest.raises(ValueError, match="normal matrix"):
        read_run(tmp_path)


@pytest.mark.parametrize("firmware", ["/explicit", "/home/gart/navpy-simtime-step-handshake", "/long/" + "Ã©" * 120])
def test_profile_uses_explicit_firmware_template_and_no_duplicate_rows(tmp_path, monkeypatch, firmware):
    from scripts import eval_preboot_profile as composer
    # Module imports in the runner use the direct scripts path, so patch that instance too.
    import eval_preboot_profile as direct_composer
    template = "SIM_RATE_HZ 1200\nLOG_DISARMED 0\nSERVO1_MIN 1000\n"
    seen = []
    def read(path):
        seen.append(path)
        return template
    monkeypatch.setattr(profiles, "_read_template", read)
    monkeypatch.setattr(composer, "_read_template", read)
    monkeypatch.setattr(direct_composer, "_read_template", read)
    profiles.prepare_profile(tmp_path, firmware, "noise-off-1000")
    text = (tmp_path / "navigation-defaults.parm").read_text()
    assert profiles.parse_parm(text) == dict(SIM_RATE_HZ=1000, SIM_NOISE_OFF=8191,
        LOG_DISARMED=1, LOG_FILE_DSRMROT=0, SERVO1_MIN=1000)
    assert text.count("SIM_RATE_HZ") == 1
    assert set(seen) == {firmware + "/Tools/autotest/models/plane.parm"}
    assert all(len(line.encode("utf-8")) <= 98 for line in text.splitlines())


def comparison_fixture(tmp_path, monkeypatch):
    from scripts import compare_noise_matrix as comparison
    value = ledger()
    fixtures = {}
    for index, attempt in enumerate(value["attempts"]):
        directory = tmp_path / str(index)
        case = directory / "121"
        case.mkdir(parents=True)
        (directory / "fleet.json").write_text("{}")
        (case / "firmware-template.parm").write_text("same template")
        (case / "peer.json").write_text(json.dumps(dict(dock=[1, 2, 3], guided_step=4)))
        attempt.update(directory=str(directory), run_id=str(index))
        manifest = dict(attempt, vehicles=[121], chat=40, home="fixed", camera_hz=40,
            launcher_sha256="same", wrapper_sha256="same", firmware_root="/own", driver_sha256={})
        summary = dict(identity=dict(defaults_sha256=attempt["noise_profile"], version=1),
                       boot=[index, 1], rows=3000)
        history = [dict(value=attempt["noise_profile"]) for _ in range(3000)]
        fixtures[directory] = (manifest, {121: (summary, history)})
    path = tmp_path / "attempts.json"
    path.write_text(json.dumps(value))
    monkeypatch.setattr(comparison, "read_run", lambda path, **kw: fixtures[path])
    monkeypatch.setattr(comparison, "verify_profile", lambda *a: {})
    return comparison, path, fixtures


def test_complete_noise_matrix_checks_all_eighteen_pairs(tmp_path, monkeypatch):
    comparison, path, _ = comparison_fixture(tmp_path, monkeypatch)
    result = comparison.compare(path)
    assert result["control_steps"] == 36000
    assert len(result["comparisons"]) == 18
    assert all(pair["equal"] for pair in result["comparisons"])


def test_matrix_report_uses_absolute_ledger_reference(tmp_path, monkeypatch):
    from pathlib import Path
    comparison, path, _ = comparison_fixture(tmp_path, monkeypatch)
    monkeypatch.chdir(tmp_path)
    result = comparison.compare(Path(path.name))
    assert Path(result["ledger"]).is_absolute()


@pytest.mark.parametrize("kind", ["boot", "run", "implementation", "dock", "mask_no_effect"])
def test_noise_matrix_rejects_cross_profile_confounds(tmp_path, monkeypatch, kind):
    comparison, path, fixtures = comparison_fixture(tmp_path, monkeypatch)
    left, right = list(fixtures.values())[:2]
    if kind == "boot":
        right[1][121][0]["boot"] = left[1][121][0]["boot"]
    elif kind == "run":
        right[0]["run_id"] = left[0]["run_id"]
    elif kind == "implementation":
        right[1][121][0]["identity"]["version"] = 2
    elif kind == "dock":
        (tmp_path / "1/121/peer.json").write_text(json.dumps(dict(dock=[9, 2, 3], guided_step=4)))
    else:
        for manifest, vehicles in fixtures.values():
            if manifest["noise_profile"] == "noise-off":
                vehicles[121][1][:] = left[1][121][1]
    with pytest.raises(ValueError):
        comparison.compare(path)


def test_noise_matrix_reports_first_divergence(tmp_path, monkeypatch):
    comparison, path, fixtures = comparison_fixture(tmp_path, monkeypatch)
    next(iter(fixtures.values()))[1][121][1][22]["value"] = "changed"
    failures = [pair for pair in comparison.compare(path)["comparisons"] if not pair["equal"]]
    assert len(failures) == 3
    assert all(pair["first_divergence"] == 23 for pair in failures)


def profile_fixture(tmp_path, monkeypatch):
    from scripts import compare_noise_matrix as comparison
    from types import SimpleNamespace
    case = tmp_path / "121"
    case.mkdir()
    manifest = dict(noise_profile="noise-off-1000", firmware_root="/own", speedup=10., delayed=False)
    (case / "firmware-template.parm").write_text("SERVO1_MIN 1000\n")
    delta = "SIM_NOISE_OFF 8191\nSIM_RATE_HZ 1000\nLOG_DISARMED 1\nLOG_FILE_DSRMROT 0\n"
    (case / "noise-profile.parm").write_text(delta)
    (case / "navigation-defaults.parm").write_text("SERVO1_MIN 1000\n" + delta)
    before, after = inventories()
    (case / "flight.BIN").write_bytes(b"123456789012345")
    for when, value in (("before", before), ("after", after)):
        (case / f"logs-{when}.json").write_text(json.dumps(value))
    checksum = comparison.digest(case / "flight.BIN")
    (case / "bin-binding.json").write_text(json.dumps(dict(source="/own/121/logs/00000002.BIN", sha256=checksum)))
    (case / "identity.json").write_text(json.dumps(dict(peer_python="/peer/python")))
    command = ["wsl.exe", "--exec", "/peer/python", comparison.wsl_path(comparison.ROOT / "scripts/simtime_navigation_peer.py"),
               "--defaults", comparison.wsl_path(case / "navigation-defaults.parm"),
        "--directory", "/own/121", "--result", comparison.wsl_path(case / "peer.json"),
        "--pid-file", comparison.wsl_path(case / "peer.pid"), "--speedup", "10.0",
        "--camera-hz", "40", "--fault", "none"]
    (case / "peer-command.json").write_text(json.dumps(command))
    history = [dict(snapshot=dict(identity=dict(source_us=t))) for t in (40000000, 40020000, 40040000)]
    monkeypatch.setattr(comparison.subprocess, "run", lambda *a, **kw:
        SimpleNamespace(stdout=json.dumps(dict(binary_sha256=checksum))))
    return comparison, case, manifest, history


def test_profile_evidence_accepts_exact_cadence_and_binding(tmp_path, monkeypatch):
    comparison, case, manifest, history = profile_fixture(tmp_path, monkeypatch)
    assert comparison.verify_profile(case, manifest, history)["control_intervals_us"] == {20000: 2}


@pytest.mark.parametrize("kind", ["cadence", "hash", "peer", "defaults", "owner"])
def test_profile_evidence_rejects_confounds(tmp_path, monkeypatch, kind):
    comparison, case, manifest, history = profile_fixture(tmp_path, monkeypatch)
    if kind == "cadence":
        history[1]["snapshot"]["identity"]["source_us"] = 40024000
    elif kind == "hash":
        (case / "flight.BIN").write_bytes(b"different bytes")
    elif kind == "peer":
        (case / "peer-command.json").write_text("[]")
    elif kind == "defaults":
        with (case / "navigation-defaults.parm").open("a") as stream:
            stream.write("EK3_HGT_DELAY 0\n")
    else:
        manifest["firmware_root"] = "/other"
    with pytest.raises(ValueError):
        comparison.verify_profile(case, manifest, history)


@pytest.mark.parametrize("intervals,valid", [([19992, 20825], True), ([20000, 20000], False)])
def test_1200hz_cadence_gate_matches_measured_integer_grid(tmp_path, monkeypatch, intervals, valid):
    comparison, case, manifest, history = profile_fixture(tmp_path, monkeypatch)
    manifest["noise_profile"] = "noise-off"
    for name in ("noise-profile.parm", "navigation-defaults.parm"):
        path = case / name
        path.write_text(path.read_text().replace("SIM_RATE_HZ 1000", "SIM_RATE_HZ 1200"))
    now = history[0]["snapshot"]["identity"]["source_us"]
    for row, dt in zip(history[1:], intervals):
        now += dt
        row["snapshot"]["identity"]["source_us"] = now
    if valid:
        assert comparison.verify_profile(case, manifest, history)["control_intervals_us"] == {19992: 1, 20825: 1}
    else:
        with pytest.raises(ValueError, match="cadence"):
            comparison.verify_profile(case, manifest, history)
