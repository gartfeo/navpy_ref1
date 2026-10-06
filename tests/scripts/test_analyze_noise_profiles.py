"""Descriptive metrics must use source time and refuse failed matrices."""

import pytest

from scripts.analyze_noise_profiles import command_metrics, require_matrix


def sample(time, roll, raw=None):
    return dict(obs_ts=time, cmd_roll_deg=roll, raw_roll_deg=roll if raw is None else raw,
        plan_reason="update",
        cmd_pitch_deg=-10., raw_pitch_deg=-10., lateral_rate_deg_s=time,
        aircraft_turn_rate_deg_s=2*time)


def test_variation_uses_elapsed_source_time_and_counts_clipping():
    result = command_metrics([sample(10., 0.), sample(10.02, 5.), sample(10.06, -10., -12.)])
    assert result["roll"]["total_variation_deg"] == 20.
    assert result["roll"]["variation_per_s"] == pytest.approx(20/.06)
    assert result["roll"]["clipped_commands"] == 1
    assert result["pitch"]["total_variation_deg"] == 0.
    assert result["largest_roll_step"]["dt_s"] == pytest.approx(.04)


@pytest.mark.parametrize("samples", [[], [sample(1., 0.)], [sample(1., 0.), sample(1., 2.)]])
def test_insufficient_or_zero_duration_commands_reject(samples):
    with pytest.raises(ValueError):
        command_metrics(samples)


def test_failed_comparison_report_cannot_be_analyzed():
    profiles = ("stock", "noise-off", "noise-off-1000")
    report = dict(control_steps=36000, comparisons=[dict(profile=name, equal=True)
        for name in profiles for _ in range(6)], runs=[dict(profile=name) for name in profiles for _ in range(4)])
    report["comparisons"][3]["equal"] = False
    with pytest.raises(ValueError, match="complete accepted"):
        require_matrix(report)


def test_phase_range_and_time_boundaries_follow_captured_indices(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace
    from scripts import analyze_noise_profiles as analyzer
    case, replay = tmp_path / "case", tmp_path / "replay"
    case.mkdir()
    replay.mkdir()
    peer = dict(seed_step=2, guided_step=1, dock=[0., 0., 0.],
                records=[dict(snapshot=f"{i:02x}") for i in range(5)])
    (case / "peer.json").write_text(json.dumps(peer))
    snapshots = [SimpleNamespace(identity=SimpleNamespace(source_us=(i+1)*1000000),
        truth_us=(i+1)*1000000-1000,
        truth=SimpleNamespace(latitude=position, longitude=0., altitude=0.))
        for i, position in enumerate((100., 10., 6., 1., 0.))]
    monkeypatch.setattr(analyzer.Snapshot, "decode", lambda payload: snapshots[payload[0]])
    # Range is an explicit fixture here; the test is about which snapshots are scored.
    monkeypatch.setattr(analyzer.pymap3d, "geodetic2ned", lambda lat, *args: (lat, 0., 0.))
    monkeypatch.setattr(analyzer, "_samples", lambda path: [sample(i+1., i*2.) for i in range(5)])
    raw = dict(causality={}, capture_intervals_us={}, noncapture_reissues=0,
        inputs=[dict(source_us=(i+1)*1000000, roll_estimate_minus_truth_deg=float(i),
                     pitch_estimate_minus_truth_deg=0.) for i in range(5)])
    (replay / "report.json").write_text(json.dumps(raw))
    result = analyzer.describe(case, replay, dict(first_pass_step=4))
    assert result["fresh_commands"] == 2  # seed included, pass excluded
    assert result["sampled_closest_approach_m"] == 1.  # pass included, following sample excluded
    assert result["closest_at_window_boundary"]
    assert result["closest_source_s"] == 4.
    assert result["closest_truth_s"] == 3.999
    assert result["source_minus_truth_us"] == {1000: 5}
    assert result["largest_roll_step"]["source_s"] == 3.
    assert result["largest_roll_step"]["sampled_range_m"] == 6.
    assert result["estimate_minus_truth_rms_deg"]["roll"] == pytest.approx((2.5)**.5)
    assert result["law_branches"] == {"update": 2}
