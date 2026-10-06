"""Certificate support: measured clock, identity pinning, no-selection aggregates.

The theme running through this file is that a certificate must never IMPROVE a
result by omission. A rate that could not be measured is not the requested rate;
an identity that could not be read is not "clean"; and an aggregate over the
runs that happened to produce numbers is not an aggregate over the runs flown.
Each test below pins one of those.
"""
from __future__ import annotations

import subprocess

import pytest

from scripts import eval_certificate as cert
from scripts import eval_certificate_process as cert_process


def _feed(tracker, pairs):
    """Feed (source_s, monotonic_wall_s) pairs and return the verdict.

    The wall side is a MONOTONIC reading, which is the caller contract: the
    tracker divides by a wall span, so an epoch stamp could be stepped by NTP
    and would silently rescale the result.
    """
    for source, wall in pairs:
        tracker.add(source, wall)
    return tracker.result


# --------------------------------------------------------------------------
# ClockRateTracker
# --------------------------------------------------------------------------


def test_ten_sim_seconds_per_wall_second_measures_as_ten():
    result = _feed(cert.ClockRateTracker(),
                   [(0.0, 0.0), (50.0, 5.0), (100.0, 10.0)])
    assert result.rate == pytest.approx(10.0)
    assert result.error is None
    assert result.sample_count == 3


def test_a_one_x_clock_when_ten_was_requested_measures_as_one():
    # The failure this whole gate exists for: everything reports 10x, the
    # clock runs at 1x. The measurement has to show 1, not the request.
    result = _feed(cert.ClockRateTracker(),
                   [(0.0, 0.0), (5.0, 5.0), (10.0, 10.0)])
    assert result.rate == pytest.approx(1.0)


def test_too_short_a_window_is_unmeasured_not_estimated():
    result = _feed(cert.ClockRateTracker(min_span_s=2.0),
                   [(0.0, 0.0), (5.0, 0.5)])
    assert result.rate is None
    assert "under the" in result.error


def test_no_timestamped_samples_at_all_is_unmeasured():
    result = cert.ClockRateTracker().result
    assert result.rate is None
    assert result.error == "no timestamped position samples"
    result = _feed(cert.ClockRateTracker(), [(None, 1.0), (2.0, None)])
    assert result.rate is None


def test_reordered_arrivals_do_not_corrupt_the_slope():
    # UDP delivers two chronological neighbours in the wrong order -- a 20 ms
    # inversion, as measured on the monitor link. Ending the slope on the last
    # ARRIVAL would take the older frame as the window's end.
    in_order = _feed(cert.ClockRateTracker(),
                     [(0.0, 0.0), (50.0, 5.0), (99.8, 9.98), (100.0, 10.0)])
    swapped = _feed(cert.ClockRateTracker(),
                    [(0.0, 0.0), (50.0, 5.0), (100.0, 10.0), (99.8, 9.98)])
    assert swapped.rate == pytest.approx(in_order.rate)
    assert swapped.error is None


def test_a_source_clock_reset_fails_closed_instead_of_measuring_nonsense():
    # Past the reorder horizon this is not a transport artefact; the clock
    # restarted, and any slope across the discontinuity is meaningless.
    result = _feed(cert.ClockRateTracker(),
                   [(100.0, 0.0), (150.0, 5.0), (2.0, 10.0)])
    assert result.rate is None
    assert "backwards" in result.error


def test_a_zero_length_window_is_unmeasured_even_with_no_minimum():
    result = _feed(cert.ClockRateTracker(min_span_s=0.0),
                   [(1.0, 7.0), (2.0, 7.0)])
    assert result.rate is None


# --------------------------------------------------------------------------
# Aggregation
# --------------------------------------------------------------------------


def test_sample_sd_not_population_sd():
    stats = cert.summarize([1.0, 2.0, 3.0])
    # n-1 form: 1.0. The population form would be ~0.816 and would understate
    # the spread a repeatability claim is supposed to bound.
    assert stats.sample_sd == pytest.approx(1.0)
    assert stats.mean == pytest.approx(2.0)
    assert stats.value_range == pytest.approx(2.0)


def test_a_single_run_reports_no_sd_rather_than_zero():
    stats = cert.summarize([0.3])
    assert stats.sample_sd is None  # zero would read as "perfectly repeatable"


def test_summarize_ignores_non_numbers_and_empties():
    assert cert.summarize([]) is None
    assert cert.summarize(["", None, float("nan")]) is None


def _row(index, *, passed, snap=None, coordinate=None, rate=None, error=""):
    return {
        "index": 0,
        "repetition": index,
        "name": f"run-{index}",
        "passed": passed,
        "dist_3d_m": snap,
        "coordinate_dist_3d_m": coordinate,
        "measured_clock_rate": rate,
        "speedup": 10,
        "error": error,
    }


def test_every_repetition_is_reported_and_a_failure_is_not_replaced():
    rows = [
        _row(1, passed=True, snap=0.21, coordinate=0.25, rate=10.1),
        _row(2, passed=False, snap=1.90, coordinate=1.95, rate=10.0,
             error="no SNAP found before timeout"),
        _row(3, passed=True, snap=0.30, coordinate=0.28, rate=9.9),
    ]
    summary = cert.certificate_summary(rows)
    assert summary["runs"] == 3
    assert summary["passed_runs"] == 2
    assert summary["failed_runs"] == 1
    assert [run["repetition"] for run in summary["per_run"]] == [1, 2, 3]
    # The failed run's numbers are IN the aggregate. Dropping them would report
    # the best of 3 under the name of all 3.
    assert summary["metrics"]["snap_dist_3d_m"]["count"] == 3
    assert summary["metrics"]["snap_dist_3d_m"]["max"] == pytest.approx(1.90)
    assert summary["per_run"][1]["error"]


def test_snap_versus_external_cpa_disagreement_is_reported_per_run():
    summary = cert.certificate_summary(
        [_row(1, passed=True, snap=0.20, coordinate=0.26)])
    assert summary["per_run"][0]["snap_minus_coordinate_m"] == pytest.approx(0.06)
    assert summary["metrics"]["snap_minus_coordinate_m"]["mean"] == pytest.approx(0.06)


def test_a_run_with_no_number_is_counted_as_missing_not_dropped():
    summary = cert.certificate_summary([
        _row(1, passed=True, snap=0.2, coordinate=0.2, rate=10.0),
        _row(2, passed=False),
    ])
    assert summary["runs"] == 2
    assert summary["runs_without_snap"] == 1
    assert summary["runs_without_coordinate"] == 1
    assert summary["runs_without_measured_rate"] == 1
    assert summary["metrics"]["snap_dist_3d_m"]["count"] == 1


# --------------------------------------------------------------------------
# Identity pinning
# --------------------------------------------------------------------------


def test_navpy_identity_records_a_dirty_tree_as_dirty(monkeypatch, tmp_path):
    def fake_run(command, **_kwargs):
        text = {
            "HEAD": "abc123\n",
            "--abbrev-ref": "feat/x\n",
            "--porcelain": " M src/navpy/main.py\n",
        }
        for key, value in text.items():
            if key in command:
                return subprocess.CompletedProcess(command, 0, value, "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(cert_process.subprocess, "run", fake_run)
    identity = cert.navpy_identity(tmp_path)
    assert identity["commit"] == "abc123"
    assert identity["dirty"] is True
    assert identity["dirty_files"] == ["src/navpy/main.py"]


def test_an_unreadable_tree_reports_unknown_rather_than_clean(monkeypatch, tmp_path):
    # "We could not look" must never render as "nothing was modified": a
    # certificate would then name a commit the run may not have been flown from.
    def explode(command, **_kwargs):
        raise FileNotFoundError(command[0])

    monkeypatch.setattr(cert_process.subprocess, "run", explode)
    identity = cert.navpy_identity(tmp_path)
    assert identity["commit"] is None
    assert identity["dirty"] is None
    assert identity["unavailable"]


def test_mission_identity_changes_when_a_waypoint_moves():
    class Item:
        def __init__(self, seq, lat):
            self.seq = seq
            self.command = 16
            self.frame = 3
            self.lat_deg = lat
            self.lon_deg = 44.0
            self.mission_alt_m = 100.0

    first = cert.mission_identity([Item(0, 40.0), Item(1, 40.1)])
    same = cert.mission_identity([Item(0, 40.0), Item(1, 40.1)])
    moved = cert.mission_identity([Item(0, 40.0), Item(1, 40.2)])
    assert first == same
    assert first["item_count"] == 2
    assert first["sha256"] != moved["sha256"]


def _snapshot(values, count=None):
    return cert.ParameterSnapshot(
        values=values,
        vehicle_param_count=len(values) if count is None else count)


def test_parameter_snapshot_hash_ignores_ordering_but_not_values():
    a = cert.parameter_snapshot_identity(_snapshot({"B": 2.0, "A": 1.0}))
    b = cert.parameter_snapshot_identity(_snapshot({"A": 1.0, "B": 2.0}))
    c = cert.parameter_snapshot_identity(_snapshot({"A": 1.0, "B": 2.5}))
    assert a == b
    assert a["sha256"] != c["sha256"]
    assert a["parameter_count"] == 2
    assert a["complete"] is True


def test_a_snapshot_is_only_complete_when_the_vehicle_stated_its_count():
    # Unknown total must not read as satisfied: the parameter protocol has no
    # completion message, so "the link went quiet" proves nothing on its own.
    assert not cert.ParameterSnapshot(values={"A": 1.0}).complete
    assert not cert.ParameterSnapshot(
        values={"A": 1.0}, vehicle_param_count=2).complete
    assert cert.ParameterSnapshot(
        values={"A": 1.0}, vehicle_param_count=1).complete
    assert not cert.ParameterSnapshot(
        values={"A": 1.0}, vehicle_param_count=1, missing_indices=(0,)).complete


def test_autopilot_identity_reports_the_build_the_vehicle_says_it_runs():
    class Mav:
        def command_long_send(self, *_args):
            return None

    class Master:
        target_system = 121
        target_component = 1
        mav = Mav()

        def recv_match(self, **_kwargs):
            return type("Version", (), {
                "flight_sw_version": 67_305_472,
                "board_version": 0,
                "flight_custom_version": [1, 2, 3],
                "os_custom_version": b"\xaa\xbb",
            })()

    identity = cert.autopilot_identity(Master())
    assert identity["available"] is True
    assert identity["flight_custom_version"] == "010203"
    assert identity["os_custom_version"] == "aabb"


def test_a_silent_autopilot_is_recorded_as_unavailable_not_omitted():
    class Master:
        target_system = 121
        target_component = 1
        mav = type("Mav", (), {"command_long_send": lambda *a: None})()

        def recv_match(self, **_kwargs):
            return None

    identity = cert.autopilot_identity(Master())
    assert identity["available"] is False
    assert identity["reason"]


def test_wsl_probes_degrade_to_a_reason_when_wsl_is_absent(monkeypatch):
    # A machine without WSL still produces a valid certificate; it just records
    # what it could not obtain. Blocking the run on a diagnostic would be worse.
    monkeypatch.setattr(cert_process.shutil, "which", lambda _name: None)
    source = cert.wsl_ardupilot_identity()
    eeprom = cert.eeprom_identity(121)
    assert source["available"] is False and source["reason"]
    assert eeprom["available"] is False and eeprom["reason"]


def test_eeprom_identity_parses_size_mtime_and_hash(monkeypatch):
    checksum = "d" * 64

    def fake_run(command, **_kwargs):
        if command[-2:] == ["printenv", "HOME"]:
            output = "/home/evaluator\n"
        elif "stat" in command:
            output = "8192 1750000000\n"
        else:
            output = f"{checksum}  eeprom.bin\n"
        return subprocess.CompletedProcess(command, 0, output, "")

    monkeypatch.setattr(cert_process.shutil, "which", lambda _name: "wsl.exe")
    monkeypatch.setattr(cert_process.subprocess, "run", fake_run)
    identity = cert.eeprom_identity(121)
    assert identity == {
        "available": True,
        "path": "~/ardupilot/121/eeprom.bin",
        "size_bytes": 8192,
        "mtime_epoch_s": 1750000000,
        "sha256": checksum,
    }


def test_unparsable_eeprom_output_is_a_reason_not_a_crash(monkeypatch):
    def fake_run(command, **_kwargs):
        if command[-2:] == ["printenv", "HOME"]:
            output = "/home/evaluator\n"
        elif "stat" in command:
            output = "nonsense\n"
        else:
            output = f"{'d' * 64}  eeprom.bin\n"
        return subprocess.CompletedProcess(command, 0, output, "")

    monkeypatch.setattr(cert_process.shutil, "which", lambda _name: "wsl.exe")
    monkeypatch.setattr(cert_process.subprocess, "run", fake_run)
    assert cert.eeprom_identity(121)["available"] is False


# --------------------------------------------------------------------------
# Source-time observability (Phase 2a)
# --------------------------------------------------------------------------
# These aggregates are DESCRIPTIVE: nothing they produce may enter certificate
# validity, and the tests below also pin that the math is exact -- a percentile
# off by one index reads as a cadence stall that never happened.


def test_percentile_is_linear_interpolated_and_exact():
    assert cert.percentile([], 99) is None
    assert cert.percentile([7.0], 50) == 7.0
    assert cert.percentile([10.0, 20.0, 30.0, 40.0], 50) == 25.0
    assert cert.percentile([10.0, 20.0, 30.0, 40.0], 100) == 40.0
    assert cert.percentile([10.0, 20.0, 30.0, 40.0], 0) == 10.0
    # Unsorted input must not matter, and q=95 of 1..100 is 95.05 by linear
    # interpolation over (n-1) intervals.
    values = list(range(100, 0, -1))
    assert cert.percentile(values, 95) == pytest.approx(95.05)


def test_distribution_reports_count_tails_and_extremes():
    stats = cert.distribution([33.0, 33.0, 34.0, 200.0])
    assert stats.count == 4
    assert stats.p50 == pytest.approx(33.5)
    assert stats.maximum == 200.0
    assert set(stats.as_dict()) == {
        "count", "mean", "p50", "p95", "p99", "min", "max"}
    assert cert.distribution([]) is None


def _write_stream(directory, stream, sys_id, pid, header, rows):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"pose_cadence_{stream}_uav_{sys_id}_pid_{pid}.csv"
    path.write_text(
        "\n".join([header] + rows) + "\n", encoding="utf-8")
    return path


def test_source_time_summary_of_a_missing_dir_is_empty_not_an_error(tmp_path):
    summary = cert.source_time_summary(tmp_path / "absent")
    assert summary["files"] == 0
    assert summary["streams"] == {}


def test_source_time_summary_measures_gaps_ages_and_outcomes(tmp_path):
    _write_stream(tmp_path, "attitude_arrival", 121, 100, "wall_s,boot_ms",
                  ["100.0,1000", "100.003,1033", "100.006,1066"])
    _write_stream(tmp_path, "telemetry_reject", 121, 100, "wall_s,mtype,reason",
                  ["100.0,ATTITUDE,stale_boot"])
    _write_stream(tmp_path, "detector_pose", 121, 100,
                  "wall_s,boot_s,outcome,frame_ts",
                  ["100.0,1.0,emitted,1.0",
                   "100.001,1.0,dup_attitude,",
                   "100.002,0.9,reordered_attitude,",
                   "100.003,1.033,emitted,1.033",
                   "100.004,,no_attitude,",
                   "100.005,,missing_attitude_dropped,",
                   "100.006,1.02,overload_dropped,"])
    _write_stream(tmp_path, "observation", 121, 100,
                  "wall_s,obs_ts,src_now_s,outcome",
                  ["100.0,5.0,5.020,fresh",
                   "100.001,5.0,5.021,duplicate",
                   "100.003,5.033,5.043,fresh"])
    _write_stream(tmp_path, "worker", 121, 100,
                  "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome",
                  ["100.0,2.0,5.0,5.030,measured,fresh",
                   "100.04,2.0,5.0,5.070,measured,held",
                   "100.08,2.0,5.050,5.051,measured,fresh"])

    summary = cert.source_time_summary(tmp_path)
    streams = summary["streams"]

    attitude = streams["attitude_arrival"]
    assert attitude["rows"] == 3
    assert attitude["source_gap_ms"]["p50"] == pytest.approx(33.0)

    assert streams["telemetry_reject"]["counts"] == {"ATTITUDE/stale_boot": 1}

    detector = streams["detector_pose"]
    assert detector["outcomes"] == {
        "emitted": 2,
        "dup_attitude": 1,
        "reordered_attitude": 1,
        "no_attitude": 1,
        "missing_attitude_dropped": 1,
        "overload_dropped": 1,
    }
    assert detector["frame_source_gap_ms"]["count"] == 1
    assert detector["frame_source_gap_ms"]["mean"] == pytest.approx(33.0)
    metrics = cert.source_time_row_metrics(summary)
    assert metrics["frame_emitted_count"] == 2
    assert metrics["frame_dropped_count"] == 4
    assert metrics["frame_reanchor_count"] == 0
    assert metrics["frame_reordered_count"] == 1

    observation = streams["observation"]
    assert observation["outcomes"] == {"fresh": 2, "duplicate": 1}
    # Gaps and ages over FRESH rows only: the duplicate's zero gap would
    # flatter the cadence it degrades.
    assert observation["source_gap_ms"]["count"] == 1
    assert observation["source_gap_ms"]["mean"] == pytest.approx(33.0)
    assert observation["age_ms"]["min"] == pytest.approx(10.0)
    assert observation["age_ms"]["max"] == pytest.approx(20.0)
    assert observation["future_count"] == 0
    assert observation["nonmonotonic_count"] == 0

    worker = streams["worker"]
    assert "sources" not in worker
    assert worker["outcomes"] == {"fresh": 2, "held": 1}
    assert worker["source_gap_ms"]["count"] == 1
    assert worker["source_gap_ms"]["mean"] == pytest.approx(50.0)
    assert worker["age_ms"]["max"] == pytest.approx(70.0)
    assert worker["wall_gap_ms"]["count"] == 2


def test_source_time_summary_flags_future_and_nonmonotonic_observations(tmp_path):
    _write_stream(tmp_path, "observation", 121, 100,
                  "wall_s,obs_ts,src_now_s,outcome",
                  ["100.0,5.0,4.9,fresh",      # src "now" BEHIND the frame
                   "100.1,4.5,4.6,fresh"])     # source time went backwards
    observation = cert.source_time_summary(tmp_path)["streams"]["observation"]
    assert observation["future_count"] == 1
    assert observation["nonmonotonic_count"] == 1


def test_source_time_gaps_never_span_two_process_generations(tmp_path):
    # Same stream from two pids (an eval relaunch). The 3967 ms jump between
    # the generations is a statement about the harness, not the pipeline, and
    # must not appear as a cadence gap.
    _write_stream(tmp_path, "attitude_arrival", 121, 100, "wall_s,boot_ms",
                  ["100.0,1000", "100.003,1033"])
    _write_stream(tmp_path, "attitude_arrival", 121, 200, "wall_s,boot_ms",
                  ["200.0,5000", "200.003,5033"])
    attitude = cert.source_time_summary(tmp_path)["streams"]["attitude_arrival"]
    assert attitude["rows"] == 4
    assert attitude["source_gap_ms"]["count"] == 2
    assert attitude["source_gap_ms"]["max"] == pytest.approx(33.0)


def test_source_time_row_metrics_flattens_and_blanks_missing_streams(tmp_path):
    _write_stream(tmp_path, "worker", 121, 100,
                  "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome",
                  ["100.0,2.0,5.0,5.030,measured,fresh",
                   "100.04,2.0,5.0,5.040,measured,held",
                   "100.08,2.0,5.050,5.051,measured,fresh"])
    metrics = cert.source_time_row_metrics(cert.source_time_summary(tmp_path))
    assert metrics["cmd_age_max_ms"] == pytest.approx(40.0)
    assert metrics["cmd_wall_gap_p99_ms"] == pytest.approx(40.0)
    assert metrics["cmd_fresh_count"] == 2
    assert metrics["cmd_held_count"] == 1
    assert "cmd_prediction_count" not in metrics
    # Streams that produced no file report BLANK, not zero: "no instrument
    # output" and "measured zero" are different claims.
    assert metrics["obs_source_gap_p99_ms"] == ""
    assert metrics["obs_duplicate_count"] == ""
    assert metrics["frame_emitted_count"] == ""


def test_source_time_metrics_do_not_affect_certificate_validity():
    row = {
        "index": 0, "repetition": 1, "name": "case", "passed": True,
        "dist_3d_m": 0.2, "coordinate_dist_3d_m": 0.25, "speedup": 10,
        "launch_measured_clock_rate": 10.1, "measured_clock_rate": 10.0,
        "certificate_invalid_reason": "",
        "error": "",
        "identity_navpy_commit": "abc", "identity_mission_sha": "m",
        "identity_parameters_sha": "p", "identity_autopilot": "a",
        # Terrible observability numbers on a passing run:
        "obs_source_gap_p99_ms": 5000.0, "cmd_source_gap_p99_ms": 5000.0,
        "cmd_age_p95_ms": 5000.0, "cmd_age_max_ms": 9000.0,
    }
    summary = cert.certificate_summary([row])
    assert summary["valid"] is True
    # ...but they ARE aggregated for the record.
    assert summary["metrics"]["cmd_age_max_ms"]["max"] == 9000.0


def test_source_time_env_is_the_instrument_modules_env_var():
    import inspect

    import navpy.modules.vehicle.pose_cadence_debug as pcd_mod

    # The evaluator sets cert.SOURCE_TIME_ENV; the instrument reads its own
    # env var at import. If the two names drift apart the gate silently
    # enables nothing.
    assert cert.SOURCE_TIME_ENV in inspect.getsource(pcd_mod._resolve_output_dir)


def test_flush_grace_covers_at_least_two_flush_intervals():
    import navpy.modules.vehicle.pose_cadence_debug as pcd_mod

    # The evaluator waits SOURCE_TIME_FLUSH_GRACE_S before hard-terminating
    # NavPy so the daemon flusher can land the terminal tail. Two periods, not
    # one: a flush that began just before the last rows were buffered writes
    # them only on the NEXT pass.
    assert cert.SOURCE_TIME_FLUSH_GRACE_S >= 2 * pcd_mod._FLUSH_INTERVAL_S
