"""Step 12's offline timing check (tools/hitl/check_integration_timing.py) on synthetic runs with known truth:
board-time and gimbal-sample joins, missing and ambiguous identities, drift and slips."""
from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest

from tests.tools.hitl import timing_scenario as ts
from tools.hitl.encoded_capture_score import INTERVAL_TOLERANCE_NS
from tools.hitl import check_integration_timing as cit
from tools.hitl.timing_inputs import (
    TimingInputError,
    read_emulator_log,
    read_navpy_log,
    read_publisher_frames,
    read_unreal_slots,
)
from tools.hitl.timing_inputs import PublisherFrame
from tools.hitl.timing_fit import calibration_pairs as cit_pairs
from tools.hitl.timing_stats import block_count, fit_median
from tools.hitl.timing_joins import AMBIGUOUS, INVALID, MISSING, JoinError, NavpyFrame, join_publisher

CALIBRATION = ts.World()
# Unreal's missed slots across the whole sender log, irregular: every shift of a short receiver window crosses one.
SPREAD_HOLES = tuple(17 + i * 97 + (i * i * 7) % 41 for i in range(19))


def joined(logs: ts.Logs, tmp_path, *, navpy=None, unreal=None, emulator=None, publisher=None):
    path = ts.publisher_file(logs, tmp_path) if publisher is None else publisher
    return cit.join_run(logs.config, read_navpy_log(logs.navpy if navpy is None else navpy),
                        read_publisher_frames([path]), read_unreal_slots(logs.unreal if unreal is None else unreal),
                        read_emulator_log(logs.emulator if emulator is None else emulator))


@pytest.fixture(scope="module")
def calibration(tmp_path_factory):
    logs = ts.build(CALIBRATION)
    return logs, cit.fit(joined(logs, tmp_path_factory.mktemp("cal")))


def test_fit_recovers_each_correction_from_receiver_receipts(calibration):
    _, result = calibration
    assert result["passed"], result["failures"]
    attitude, gimbal = result["attitude"], result["gimbal"]
    assert attitude["pairs"] >= cit.MIN_PAIRS and gimbal["pairs"] >= cit.MIN_PAIRS
    # Ds - Dc, plus the median of the source jitter (exponential, 1 ms and 0.5 ms means) and the ms cut of time_boot.
    assert attitude["correction_s"] == pytest.approx(CALIBRATION.true_attitude_c, abs=0.0015)
    assert gimbal["correction_s"] == pytest.approx(CALIBRATION.true_gimbal_c, abs=0.0006)
    assert attitude["ci_s"][0] <= attitude["correction_s"] <= attitude["ci_s"][1]
    assert 0.0 < attitude["uncertainty_s"] < 0.001 and 0.0 < gimbal["uncertainty_s"] < 0.001
    assert result["run_ids"] == ["cal-001"]


def test_known_board_time_and_gimbal_sample_joins(calibration, tmp_path):
    logs, _ = calibration
    run = joined(logs, tmp_path)
    world = logs.world
    first = run.frames[0]
    sequence = int(first.get("frame_sequence"))
    # Frame `sequence` is the (first_received + sequence - 1)-th rendered slot, holes skipped.
    rendered = [slot for slot in range(int(world.seconds * ts.FPS)) if slot not in world.holes]
    slot = rendered[world.first_received + sequence - 1]
    assert run.board_s[sequence] == pytest.approx(ts.BOARD_START_S + slot / ts.FPS, abs=1e-6)
    expected = world.receiver(run.board_s[sequence]) - ts.EPOCH_G - ts.RENDER_LAG_S
    assert run.rendered_t[sequence] == pytest.approx(expected, abs=1e-6)
    # Every readback joins to the 0x0D frame that reported plant time = its step - report lag, across the wrap.
    assert run.gimbal.destination == ("127.0.0.1", 40001)
    assert all(later - earlier == pytest.approx(0.02) for earlier, later in zip(run.gimbal.plant_t,
                                                                                  run.gimbal.plant_t[1:]))
    assert {65535, 1} <= {sample.seq for sample in run.gimbal.samples}     # across the 16-bit wrap


def held_out_check(calibration, tmp_path, observation=False, **changes):
    world, result = calibration[0].world, calibration[1]
    logs = ts.build(ts.held_out(world, result, **changes))
    return cit.check(joined(logs, tmp_path), result, observation)


def test_held_out_run_passes_with_small_skews_and_reports_them(calibration, tmp_path):
    report = held_out_check(calibration, tmp_path)
    assert report["passed"], report["failures"]
    for name in ("attitude", "gimbal"):
        section = report[name]
        assert section["count"] > 1000
        assert section["max_abs_s"] + section["uncertainty_s"] <= cit.SKEW_BUDGET_S
        assert section["p95_abs_s"] <= section["max_abs_s"]
    assert report["counts"]["gimbal:replayed"] > 1000
    assert report["intervals"]["max_pts_error_s"] <= INTERVAL_TOLERANCE_NS / 1e9


def test_live_gimbal_associations_must_equal_their_replay(calibration, tmp_path):
    report = held_out_check(calibration, tmp_path, fixed_lens=True)
    assert report["passed"], report["failures"]
    assert report["counts"]["gimbal:live"] > 1000 and "gimbal:replayed" not in report["counts"]


def test_drift_between_board_and_receiver_clocks_is_absorbed(tmp_path):
    world = replace(CALIBRATION, ppm=300.0)
    fitted = cit.fit(joined(ts.build(world), tmp_path))
    assert fitted["passed"], fitted["failures"]
    report = held_out_check((ts.build(world), fitted), tmp_path, ppm=-300.0)
    assert report["passed"], report["failures"]


@pytest.mark.parametrize("slip_s, passes", [(0.002, True), (0.012, False), (-0.012, False)])
def test_a_link_delay_slip_after_the_fit_shows_as_skew(calibration, tmp_path, slip_s, passes):
    report = held_out_check(calibration, tmp_path, attitude_slip=(ts.BOARD_START_S + 30.0, slip_s))
    assert report["passed"] is passes, report["failures"]
    assert report["attitude"]["max_abs_s"] >= abs(slip_s)
    if not passes:
        assert any(text.startswith("attitude: max |skew|") for text in report["failures"])


def test_receipt_jitter_over_the_budget_fails_even_with_true_constants(calibration, tmp_path):
    # Exponential receipt jitter of 3 ms mean puts its tail far past 8 ms: the fit's median cannot hide it.
    report = held_out_check(calibration, tmp_path, attitude_jitter_s=0.003)
    assert not report["passed"]
    assert report["attitude"]["max_abs_s"] > cit.SKEW_BUDGET_S
    assert any(text.startswith("attitude: max |skew|") for text in report["failures"])


def test_a_map_abort_fails_the_run_even_with_a_good_prefix(calibration, tmp_path):
    report = held_out_check(calibration, tmp_path, abort_after_s=30.0, holes=SPREAD_HOLES)
    assert not report["passed"]
    assert any(text.startswith("map interrupted") for text in report["failures"])


def test_the_observation_needs_ten_minutes_of_scored_frames(calibration, tmp_path):
    report = held_out_check(calibration, tmp_path, observation=True)
    assert not report["passed"]
    assert any("under 600 s" in text for text in report["failures"])


def test_ten_minute_observation_passes(calibration, tmp_path):
    report = held_out_check(calibration, tmp_path, observation=True, seconds=605.0,
                            holes=(17, 400, 905, 9000, 15000))
    assert report["passed"], report["failures"]
    assert report["attitude"]["span_s"] >= cit.OBSERVATION_S and report["gimbal"]["span_s"] >= cit.OBSERVATION_S


def test_a_held_out_run_must_not_be_a_fit_run(calibration, tmp_path):
    logs, result = calibration
    report = cit.check(joined(ts.build(ts.held_out(logs.world, result, run_id="cal-001")), tmp_path), result, False)
    assert "the checked run was used to fit the constants (not held out)" in report["failures"]


def test_constants_must_be_the_ones_the_run_froze(calibration, tmp_path):
    logs, result = calibration
    world = ts.held_out(logs.world, result)
    world = replace(world, corrections=(world.corrections[0] + 0.001, world.corrections[1]))
    report = cit.check(joined(ts.build(world), tmp_path), result, False)
    assert any("is not the fitted constant" in text for text in report["failures"])


def test_too_few_pairs_refuse_the_fit(tmp_path):
    result = cit.fit(joined(ts.build(replace(CALIBRATION, seconds=1.5)), tmp_path))
    assert not result["passed"]
    assert any("fewer than 30" in text for text in result["failures"])


# --- missing and ambiguous identities -----------------------------------------------------------------------------
def _expect(kind, function):
    with pytest.raises(JoinError) as raised:
        function()
    assert raised.value.kind == kind, str(raised.value)


def test_an_anchor_implying_fewer_lost_frames_is_still_ambiguous():
    # navpy got frames 0 and 2 (frame 1 lost on the way); frames 3 and 4 fit the same PTS delta with no loss.
    publisher = [PublisherFrame(index, index * 1000, pts_ms * 1_000_000)
                 for index, pts_ms in enumerate((0, 33, 66, 330, 396))]
    _expect(AMBIGUOUS, lambda: join_publisher([NavpyFrame(1, 0), NavpyFrame(2, 66_000_000)], publisher))


def test_a_short_window_past_the_last_missed_slot_is_ambiguous(calibration, tmp_path):
    # Every navpy gap can be a frame lost on the way, so a window that crosses no sender gap fits many anchors.
    logs = ts.build(ts.held_out(calibration[0].world, calibration[1], abort_after_s=30.0))
    _expect(AMBIGUOUS, lambda: joined(logs, tmp_path))


def test_a_stream_without_missed_slots_joined_late_is_ambiguous(tmp_path):
    logs = ts.build(replace(CALIBRATION, holes=(), seconds=10.0))
    _expect(AMBIGUOUS, lambda: joined(logs, tmp_path))


def test_a_missing_unreal_slot_fails(calibration, tmp_path):
    logs, _ = calibration
    unreal = [line for line in logs.unreal if "frame=300 " not in line]
    _expect(MISSING, lambda: joined(logs, tmp_path, unreal=unreal))


def test_a_duplicate_unreal_slot_is_ambiguous(calibration, tmp_path):
    logs, _ = calibration
    duplicate = next(line for line in logs.unreal if "frame=300 " in line)
    _expect(AMBIGUOUS, lambda: joined(logs, tmp_path, unreal=logs.unreal + [duplicate]))


def test_an_invalid_gimbal_bracket_fails(calibration, tmp_path):
    logs, _ = calibration
    unreal = [line.replace("gimbal_valid=1", "gimbal_valid=0") if "frame=300 " in line else line
              for line in logs.unreal]
    _expect(INVALID, lambda: joined(logs, tmp_path, unreal=unreal))


def test_another_gimbal_session_fails(calibration, tmp_path):
    logs, _ = calibration
    emulator = [line.replace(ts.SESSION, "other") for line in logs.emulator]
    _expect(MISSING, lambda: joined(logs, tmp_path, emulator=emulator))


def test_a_lost_emulator_frame_record_fails(calibration, tmp_path):
    logs, _ = calibration
    index = next(i for i, line in enumerate(logs.emulator) if '"cmd": "18"' in line)
    _expect(INVALID, lambda: joined(logs, tmp_path, emulator=logs.emulator[:index] + logs.emulator[index + 1:]))


def test_a_readback_no_0x0d_frame_carries_fails(calibration, tmp_path):
    logs, _ = calibration
    index = next(i for i, line in enumerate(logs.emulator) if '"cmd": "0d"' in line and '"seq": 100,' in line)
    emulator = list(logs.emulator)
    emulator[index] = emulator[index].replace('"cmd": "0d"', '"cmd": "0e"')
    _expect(MISSING, lambda: joined(logs, tmp_path, emulator=emulator))


def test_a_gimbal_reconnect_is_refused(calibration, tmp_path):
    logs, _ = calibration
    navpy = list(logs.navpy)
    index = max(i for i, line in enumerate(navpy) if '"gimbal_receipt"' in line)
    navpy[index] = navpy[index].replace('"session": 1', '"session": 2')
    _expect(AMBIGUOUS, lambda: joined(logs, tmp_path, navpy=navpy))


def test_a_second_capture_open_is_refused(calibration, tmp_path):
    logs, _ = calibration
    navpy = list(logs.navpy)
    index = max(i for i, line in enumerate(navpy) if '"association"' in line)
    navpy[index] = navpy[index].replace('"open-1"', '"open-2"')
    _expect(AMBIGUOUS, lambda: joined(logs, tmp_path, navpy=navpy))


def test_a_navpy_frame_the_publisher_never_pushed_fails(calibration, tmp_path):
    logs, _ = calibration
    navpy = list(logs.navpy)
    index = max(i for i, line in enumerate(navpy) if '"mapped_s"' in line)
    record = json.loads(navpy[index])
    record["pts_ns"] += 5_000_000
    navpy[index] = json.dumps(record)
    _expect(MISSING, lambda: joined(logs, tmp_path, navpy=navpy))


def test_a_replay_snapshot_that_lost_receipts_fails(calibration, tmp_path):
    logs, result = calibration
    held = ts.build(ts.held_out(logs.world, result))
    navpy = list(held.navpy)
    index = next(i for i, line in enumerate(navpy) if '"gimbal_receipt"' in line and i > len(navpy) // 2)
    del navpy[index]
    run = joined(held, tmp_path, navpy=navpy)
    _expect(MISSING, lambda: cit.check(run, result, False))


def test_unknown_or_unreadable_records_are_errors():
    with pytest.raises(TimingInputError):
        read_navpy_log(['{"event": "something", "run_id": "r"}'])
    with pytest.raises(TimingInputError):
        read_navpy_log(['{"event": "gimbal_receipt", "run_id": "r", "session": 1, "seq": 1, "receipt_s": NaN}'])
    with pytest.raises(TimingInputError):
        read_emulator_log(['{"event": "step", "seq": 1, "device_t": 0.0}'])     # no start: no session


def _edit_associations(navpy, change, after_board_s=None, world=None):
    """navpy lines with ``change(record)`` applied to each mapped association (from a board time on, if given)."""
    edited = []
    for line in navpy:
        record = json.loads(line)
        if record["event"] == "association" and record.get("mapped_s") is not None and (
                after_board_s is None or record["mapped_s"] - world.video_delay_s - ts.EPOCH_R
                > ts.BOARD_START_S + after_board_s):
            change(record)
            line = json.dumps(record)
        edited.append(line)
    return edited


def test_a_good_prefix_then_frames_without_results_fails(calibration, tmp_path):
    logs, result = calibration
    held = ts.build(ts.held_out(logs.world, result))

    def refuse(record):
        record["status"] = "real_frame_integration_source_stale"
        record.pop("attitude_time_boot_s", None)

    navpy = _edit_associations(held.navpy, refuse, after_board_s=40.0, world=held.world)
    report = cit.check(joined(held, tmp_path, navpy=navpy), result, False)
    assert not report["passed"]
    assert report["attitude"]["unscored"] > 400 and report["gimbal"]["unscored"] > 400
    assert any(text.startswith("attitude:") and "have no result" in text for text in report["failures"])


def test_non_finite_association_values_are_input_errors(calibration):
    logs, _ = calibration
    index = next(i for i, line in enumerate(logs.navpy) if '"attitude_time_boot_s"' in line)
    for name in ("attitude_time_boot_s", "mapped_s", "now_s"):
        record = json.loads(logs.navpy[index])
        record[name] = float("nan")
        with pytest.raises(TimingInputError):
            read_navpy_log([json.dumps(record)])
    record = json.loads(logs.navpy[index])
    del record["seam_tolerance_s"]
    with pytest.raises(TimingInputError):
        read_navpy_log([json.dumps(record)])


@pytest.mark.parametrize("uncertainty", [float("nan"), -0.001, None])
def test_a_fit_without_a_finite_uncertainty_fails_the_check(calibration, tmp_path, uncertainty):
    logs, result = calibration
    constants = json.loads(json.dumps(result))
    constants["attitude"]["uncertainty_s"] = uncertainty
    report = cit.check(joined(ts.build(ts.held_out(logs.world, result)), tmp_path), constants, False)
    assert not report["passed"]
    assert any("fitted uncertainty" in text for text in report["failures"])


def test_a_sample_without_board_time_breaks_brackets_as_it_does_live(calibration, tmp_path):
    logs, result = calibration
    navpy = list(logs.navpy)
    index = [i for i, line in enumerate(navpy) if '"attitude_receipt"' in line][len(navpy) // 4]
    record = json.loads(navpy[index])
    record["time_boot_s"] = None
    navpy[index] = json.dumps(record)
    attitude, _ = cit_pairs(joined(logs, tmp_path, navpy=navpy))
    baseline, _ = cit_pairs(joined(logs, tmp_path))
    # The seam's 2 s quarantine refuses about 60 frames of 30 fps, as navpy's own lookup would.
    assert attitude.unpaired - baseline.unpaired >= 30


def test_a_clock_seam_breaks_brackets_as_it_does_live(calibration, tmp_path):
    logs, _ = calibration
    navpy = list(logs.navpy)
    receipts = [i for i, line in enumerate(navpy) if '"attitude_receipt"' in line]
    for index in receipts[len(receipts) // 2:]:
        record = json.loads(navpy[index])
        record["capture_receipt_s"] += 0.03     # the link delay steps by 30 ms: a seam over the 25 ms tolerance
        navpy[index] = json.dumps(record)
    attitude, _ = cit_pairs(joined(logs, tmp_path, navpy=navpy))
    baseline, _ = cit_pairs(joined(logs, tmp_path))
    assert attitude.unpaired - baseline.unpaired >= 30


def test_attitude_limits_that_change_within_a_run_are_invalid(calibration, tmp_path):
    logs, _ = calibration

    def widen(record):
        record["attitude_bracket_max_s"] = 0.1

    navpy = _edit_associations(logs.navpy, widen, after_board_s=30.0, world=logs.world)
    _expect(INVALID, lambda: cit_pairs(joined(logs, tmp_path, navpy=navpy)))


def test_blocks_count_only_bins_that_hold_pairs():
    cluster = [0.01 * index for index in range(100)] + [25.0]
    assert block_count(cluster, 10.0) == 2
    with pytest.raises(ValueError):
        fit_median([0.0] * len(cluster), cluster, 10.0)


def test_bootstrap_interval_reflects_correlation_within_blocks():
    # Six 10 s stretches, each with its own offset (correlated within a block): the median's interval must reflect
    # the offsets' spread, not the tiny noise inside each block.
    rng = np.random.default_rng(3)
    offsets = rng.normal(0.0, 0.001, 6)
    times = [block * 10.0 + 0.1 * index for block in range(6) for index in range(100)]
    values = [offsets[int(time // 10.0)] + rng.normal(0.0, 1e-6) for time in times]
    fitted = fit_median(values, times, 10.0)
    assert fitted.uncertainty_s > 0.0002
    uneven = [time for time in times if time % 10.0 < 1.0 or time > 40.0]     # gaps and an uneven density
    fitted = fit_median([values[times.index(time)] for time in uneven], uneven, 10.0)
    assert fitted.uncertainty_s > 50e-6 and fitted.blocks == 6     # still far over the 1 us noise


# --- command line -----------------------------------------------------------------------------------------------
def _record(config):
    return {"run": {"run_id": config.run_id, "mode": config.mode}, "integration_timing": config.record()}


def _write_logs(logs, folder):
    folder.mkdir()
    (folder / "navpy.jsonl").write_text("\n".join(logs.navpy) + "\n", encoding="utf-8")
    (folder / "unreal.log").write_text("\n".join(logs.unreal) + "\n", encoding="utf-8")
    (folder / "emulator.jsonl").write_text("\n".join(logs.emulator) + "\n", encoding="utf-8")
    publisher = ts.publisher_file(logs, folder)
    return ["--run-record", str(folder / "record.json"), "--navpy", str(folder / "navpy.jsonl"),
            "--publisher", str(publisher), "--unreal", str(folder / "unreal.log"),
            "--emulator", str(folder / "emulator.jsonl")]


def test_command_line_fit_then_check(calibration, tmp_path, monkeypatch):
    logs, _ = calibration
    records = {}
    monkeypatch.setattr(cit.approach_records, "read_record", lambda path: records[path.parent.name])
    records["cal"] = _record(logs.config)
    args = _write_logs(logs, tmp_path / "cal")
    assert cit.main(["fit", *args, "--out", str(tmp_path / "fit.json")]) == 0
    fitted = json.loads((tmp_path / "fit.json").read_text(encoding="utf-8"))
    held = ts.build(ts.held_out(logs.world, fitted))
    records["obs"] = _record(held.config)
    args = _write_logs(held, tmp_path / "obs")
    assert cit.main(["check", *args, "--constants", str(tmp_path / "fit.json"),
                     "--out", str(tmp_path / "report.json")]) == 0
    assert json.loads((tmp_path / "report.json").read_text(encoding="utf-8"))["passed"]
    # A join failure is a failed check (1), not an input error.
    (tmp_path / "obs" / "unreal.log").write_text("", encoding="utf-8")
    assert cit.main(["check", *args, "--constants", str(tmp_path / "fit.json"),
                     "--out", str(tmp_path / "report.json")]) == 1
