"""Predeclared live acceptance cannot hide an approach regression."""

import math
from pathlib import PurePosixPath

import pytest

from scripts.pose_precision_analysis import acceptance, closest_approach, phase
from scripts.pose_rounding_evidence import PosePair


@pytest.mark.parametrize("variation,distance,accepted", [
    (3., 0.1, True), (3., 0.2, True), (3., 0.21, False),
    (34., 0.1, False), (35., 0.1, False)])
def test_both_improvement_and_approach_gate_are_required(variation, distance, accepted):
    arms = {"rounded": dict(common_window={"roll": {"variation_per_s": 34.}},
                            closest_approach={"distance_m": 0.2}),
            "precast": dict(common_window={"roll": {"variation_per_s": variation}},
                            closest_approach={"distance_m": distance})}
    assert acceptance(arms)["accepted"] is accepted


def test_nonfinite_metric_never_passes():
    arms = {k: dict(common_window={"roll": {"variation_per_s": math.nan}},
                    closest_approach={"distance_m": 0.}) for k in ("rounded", "precast")}
    with pytest.raises(ValueError, match="nonfinite"):
        acceptance(arms)


def test_scoring_uses_precast_for_both_sources_and_includes_pass_snapshot():
    dock = [40., 44., 1000.]
    pairs = [PosePair(tuple(dock), (40., 44., 1005.)),
             PosePair((0., 0., 0.), tuple(dock))]
    result = closest_approach(pairs, dock, 1, 2)
    assert result["distance_m"] == 0. and result["sampled_step"] == 2 and result["samples"] == 2


def test_segment_scoring_removes_sample_phase_error():
    dock = [40., 44., 1000.]
    pairs = [PosePair(tuple(dock), (40.,44.,999.)), PosePair(tuple(dock), (40.,44.,1001.))]
    result = closest_approach(pairs,dock,1,2)
    assert result["distance_m"] < 1e-8
    assert result["sampled_distance_m"] > 0.99
    assert abs(result["segment_fraction"]-0.5) < 1e-8


def test_stationary_segment_and_endpoint_minimum_are_valid():
    dock = [40.,44.,1000.]
    pair = PosePair(tuple(dock),(40.,44.,1002.))
    stationary = closest_approach([pair,pair],dock,1,2)
    assert abs(stationary["distance_m"]-2.) < 1e-8
    away = PosePair(tuple(dock),(40.,44.,1003.))
    endpoint = closest_approach([pair,away],dock,1,2)
    assert endpoint["segment_fraction"] == 0.


def test_missing_pass_is_rejected_before_scoring():
    from tests.scripts.test_simtime_navigation_protocol import packet
    peer = dict(seed_step=1, records=[dict(snapshot=packet(1).hex(), evidence={})])
    with pytest.raises(ValueError, match="visual pass"):
        phase(peer)


def test_launch_evidence_path_already_in_linux_is_not_converted_again():
    from scripts.compare_lockstep_fleet import launch_path
    assert launch_path(PurePosixPath("/mnt/c/capture/peer.json")) == "/mnt/c/capture/peer.json"
    with pytest.raises(ValueError, match="absolute"):
        launch_path(PurePosixPath("relative/peer.json"))
