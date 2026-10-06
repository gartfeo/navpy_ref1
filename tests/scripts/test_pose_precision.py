"""Full-precision rendering requires an explicit, verifiable source selection."""

from pathlib import Path
from dataclasses import replace

import pytest

from scripts import simtime_step_artifacts as artifacts
from scripts.pose_precision_evidence import PRECAST_FIELDS, validate_precision
from scripts.pose_rounding_evidence import validate_pairs
from tests.scripts.test_pose_rounding import pair_fixture


@pytest.mark.parametrize("source", ["rounded", "precast"])
def test_wrapper_declares_pose_source_and_clears_inherited_choice(tmp_path, monkeypatch, source):
    instance, case = tmp_path / "instance", tmp_path / "case"
    instance.mkdir()
    case.mkdir()
    template = tmp_path / "Tools/autotest/models/plane.parm"
    template.parent.mkdir(parents=True)
    template.write_text("SIM_RATE_HZ 1000\n")
    monkeypatch.setattr(artifacts, "owned_directory", lambda *args: instance)
    result = artifacts.prepare(tmp_path, 121, case, f"navigation-pose-{source}")
    lines = Path(result["wrapper"]).read_text().splitlines()
    assert lines.count("unset NAVPY_RENDER_POSE") == 1
    assert lines.count("export NAVPY_POSE_CAPTURE=1") == 1
    assert lines.count(f"export NAVPY_RENDER_POSE={source}") == 1


def test_precision_collector_requires_pose_evidence(tmp_path, monkeypatch):
    instance, case = tmp_path / "instance", tmp_path / "case"
    instance.mkdir()
    case.mkdir()
    (instance / "navpy-navigation.csv").write_text("control")
    monkeypatch.setattr(artifacts, "owned_directory", lambda *args: instance)
    with pytest.raises(ValueError, match="missing"):
        artifacts.collect(tmp_path, 121, case, "navigation-pose-precast")
    assert (instance / "navpy-navigation.csv").is_file()


def precision_fixture(source):
    rows, snapshots = pair_fixture()
    pairs = validate_pairs(rows, snapshots)
    for i, (row, pair) in enumerate(zip(rows, pairs)):
        row.update(version="2", pose_source=source)
        row.update({k: v.hex() for k, v in zip(PRECAST_FIELDS, pair.precast)})
        if source == "precast":
            snapshots[i] = replace(snapshots[i], truth=replace(snapshots[i].truth,
                latitude=pair.precast[0], longitude=pair.precast[1], altitude=pair.precast[2]))
    return rows, snapshots


@pytest.mark.parametrize("source", ["rounded", "precast"])
def test_precision_packet_matches_producer_and_legacy_conversion(source):
    rows, snapshots = precision_fixture(source)
    assert len(validate_precision(rows, snapshots, source)) == 2


@pytest.mark.parametrize("field,value", [("version", "1"), ("pose_source", "rounded"),
    ("valid", "0"), ("expected_physics_sequence", "1"), ("precast_lat_hex", (40.).hex()),
    ("precast_alt_hex", "nan"), ("publish_us", "1")])
def test_precision_rejects_wrong_source_generation_or_producer(field, value):
    rows, snapshots = precision_fixture("precast")
    rows[0][field] = value
    with pytest.raises(ValueError):
        validate_precision(rows, snapshots, "precast")


def test_precision_rejects_rounded_packet_disguised_as_precise():
    rows, snapshots = precision_fixture("precast")
    snapshots[0] = replace(snapshots[0], truth=replace(snapshots[0].truth, latitude=40.))
    with pytest.raises(ValueError, match="packet truth bits"):
        validate_precision(rows, snapshots, "precast")


def test_precision_requires_distinguishable_sources():
    rows, snapshots = precision_fixture("rounded")
    for row in rows:
        row.update(dlat_hex=(0.).hex(), dlng_hex=(0.).hex(), alt_cm_hex=(100000.).hex(),
            precast_lat_hex=(40.).hex(), precast_lng_hex=(44.).hex(), precast_alt_hex=(1000.).hex())
    with pytest.raises(ValueError, match="cannot distinguish"):
        validate_precision(rows, snapshots, "rounded")


def test_v2_capture_cannot_enter_v1_matrix(tmp_path):
    import json
    from scripts.compare_lockstep_fleet import read_run
    (tmp_path / "fleet.json").write_text(json.dumps(dict(noise_profile="noise-off-1000",
        pose_capture=True, pose_source="precast")))
    with pytest.raises(ValueError, match="undeclared"):
        read_run(tmp_path, noise_profile="noise-off-1000", pose_capture=True)


def test_precision_matrix_predeclares_control_and_all_four_timings():
    from scripts.run_pose_precision_matrix import cells
    declared = cells()
    assert len(declared) == 5 and declared[0]["pose_source"] == "rounded"
    assert all(r["pose_source"] == "precast" for r in declared[1:])
    assert {(r["speedup"], r["delayed"]) for r in declared[1:]} == {
        (1., False), (1., True), (10., False), (10., True)}
