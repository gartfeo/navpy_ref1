"""Step 12's offline timing check (tools/hitl/check_integration_timing.py) on synthetic runs with known truth:
board-time and gimbal-sample joins, missing and ambiguous identities, drift and slips."""
from __future__ import annotations

import json
from dataclasses import replace

import pytest

from tests.tools.hitl import timing_scenario as ts
from tools.hitl import check_integration_timing as cit
from tools.hitl.timing_inputs import (
    TimingInputError,
    read_emulator_log,
    read_navpy_log,
    read_publisher_frames,
    read_unreal_slots,
)
from tools.hitl.timing_joins import AMBIGUOUS, INVALID, MISSING, JoinError

CALIBRATION = ts.World()


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
    assert report["intervals"]["max_pts_error_s"] <= cit.INTERVAL_TOLERANCE_NS / 1e9


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
    report = held_out_check(calibration, tmp_path, abort_after_s=30.0)
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
