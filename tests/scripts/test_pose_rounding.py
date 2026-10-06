"""Pose evidence must prove exact pairing before any counterfactual is accepted."""

from dataclasses import replace
import json

import pytest

from scripts.pose_rounding_evidence import FIELDS, validate_pairs
from scripts import simtime_step_artifacts as artifacts
from scripts.simtime_navigation_protocol import Snapshot
from tests.scripts.test_simtime_navigation_protocol import packet


def pair_fixture():
    rows, snapshots = [], []
    for i in range(2):
        snap = Snapshot.decode(packet(i+1))
        source = 1000000 + i*20000
        snap = replace(snap, identity=replace(snap.identity, source_us=source), truth_us=source,
                       truth=replace(snap.truth, latitude=40., longitude=44., altitude=1000.))
        row = dict.fromkeys(FIELDS, "0")
        row.update({key: str(value) for key, value in dict(version=1,
            boot0=snap.identity.boot[0], boot1=snap.identity.boot[1], vehicle=snap.identity.vehicle,
            step=i+1, tick=snap.identity.tick, source_us=source, truth_us=source,
            publish_us=source, conversion_us=source, sequence=200+i*40,
            physics_sequence=100+i*20, expected_physics_sequence=100+i*20,
            branch=2, ftype_size=8, valid=1, origin_lat=400000000, origin_lng=440000000,
            home_alt=100000, rounded_lat=400000000, rounded_lng=440000000,
            rounded_alt=100000).items()})
        row.update(dlat_hex=(-0.75).hex(), dlng_hex=(0.75).hex(), alt_cm_hex=(100000.75).hex())
        rows.append(row)
        snapshots.append(snap)
    return rows, snapshots


def test_negative_offset_is_truncated_before_origin_addition():
    rows, snapshots = pair_fixture()
    pairs = validate_pairs(rows, snapshots)
    assert pairs[0].rounded == (40., 44., 1000.)
    assert pairs[0].precast[0] < pairs[0].rounded[0]
    assert pairs[0].precast[1] > pairs[0].rounded[1]
    assert pairs[0].precast[2] > pairs[0].rounded[2]


@pytest.mark.parametrize("key,value", [
    ("boot0", "0"), ("vehicle", "222"), ("step", "3"), ("tick", "0"),
    ("truth_us", "999999"), ("publish_us", "999999"), ("conversion_us", "999999"),
    ("version", "2"), ("valid", "0"), ("ftype_size", "4"), ("branch", "3"),
    ("sequence", "0"), ("physics_sequence", "0"), ("expected_physics_sequence", "0"),
    ("dlat_hex", "nan"), ("dlng_hex", "inf"), ("alt_cm_hex", "nan"),
    ("rounded_lat", "400000001"), ("rounded_lng", "440000001"), ("rounded_alt", "100001"),
    ("origin_lat", "2147483647"), ("origin_lng", "2147483647"),
    ("valid", "01"), ("home_alt", "99999999999999999"),
])
def test_corrupt_pair_is_rejected(key, value):
    rows, snapshots = pair_fixture()
    rows[0][key] = value
    with pytest.raises(ValueError):
        validate_pairs(rows, snapshots)


def test_integer_match_does_not_hide_different_truth_bits():
    rows, snapshots = pair_fixture()
    snapshots[0] = replace(snapshots[0], truth=replace(snapshots[0].truth, latitude=40.00000000000001))
    with pytest.raises(ValueError, match="truth bits"):
        validate_pairs(rows, snapshots)


def test_missing_and_duplicate_pose_steps_are_rejected():
    rows, snapshots = pair_fixture()
    with pytest.raises(ValueError, match="incomplete"):
        validate_pairs(rows[:1], snapshots)
    rows[1]["sequence"] = rows[0]["sequence"]
    with pytest.raises(ValueError, match="cadence"):
        validate_pairs(rows, snapshots)


def test_raw_reset_uses_publish_time_not_conversion_clock():
    rows, snapshots = pair_fixture()
    for row in rows:
        row["branch"] = "3"
        row["conversion_us"] = str(int(row["publish_us"])-1000)
    assert len(validate_pairs(rows, snapshots)) == 2


def test_collector_keeps_originals_if_pose_evidence_is_missing(tmp_path, monkeypatch):
    instance, case = tmp_path / "instance", tmp_path / "case"
    instance.mkdir()
    case.mkdir()
    (instance / "navpy-navigation.csv").write_text("original")
    monkeypatch.setattr(artifacts, "owned_directory", lambda *args: instance)
    with pytest.raises(ValueError, match="missing"):
        artifacts.collect(tmp_path, 121, case, "navigation-pose")
    assert (instance / "navpy-navigation.csv").read_text() == "original"


def test_collector_preserves_both_verified_files(tmp_path, monkeypatch):
    instance, case = tmp_path / "instance", tmp_path / "case"
    instance.mkdir()
    case.mkdir()
    names = ("navpy-navigation.csv", "navpy-pose.csv")
    for name in names:
        (instance / name).write_text(name)
    monkeypatch.setattr(artifacts, "owned_directory", lambda *args: instance)
    assert artifacts.collect(tmp_path, 121, case, "navigation-pose")["collected"]
    for name in names:
        assert (case / name).read_text() == name
        assert not (instance / name).exists()


def test_ordinary_collector_rejects_undeclared_capture(tmp_path, monkeypatch):
    (tmp_path / "navpy-navigation.csv").touch()
    (tmp_path / "navpy-pose.csv").touch()
    monkeypatch.setattr(artifacts, "owned_directory", lambda *args: tmp_path)
    with pytest.raises(ValueError, match="unexpected pose"):
        artifacts.collect(tmp_path, 121, tmp_path, "navigation")


def test_declared_matrix_has_disabled_control_and_all_timing_cells():
    from scripts.run_pose_matrix import cells
    declared = cells()
    assert len(declared) == 5
    assert declared[0]["pose_capture"] is False
    assert {(cell["speedup"], cell["delayed"]) for cell in declared[1:]} == {
        (1., False), (1., True), (10., False), (10., True)}
    assert all(cell["pose_capture"] and cell["noise_profile"] == "noise-off-1000" for cell in declared[1:])


def test_pose_manifest_cannot_enter_normal_matrix(tmp_path):
    from scripts.compare_lockstep_fleet import read_run
    (tmp_path / "fleet.json").write_text(json.dumps(dict(noise_profile="noise-off-1000", pose_capture=True)))
    with pytest.raises(ValueError, match="undeclared"):
        read_run(tmp_path, noise_profile="noise-off-1000")
