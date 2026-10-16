"""Detector poll and frame-lineage summaries in the source-time report."""

from __future__ import annotations

from pathlib import Path

import pytest

from scripts.eval_certificate_poll_streams import (
    cpu_fraction_by_second,
    reconcile_lineage,
    summarize_poll,
)
from scripts.eval_certificate_source_time import source_time_summary
from navpy.core.kernel.logging.pose_cadence_streams import HEADERS


def _poll(start: float, outcome: str, work: float, cpu: float) -> dict[str, str]:
    return {
        "start_wall_s": str(start), "outcome": outcome, "work_ms": str(work),
        "work_cpu_ms": str(cpu), "requested_wait_ms": "20", "actual_wait_ms": "25",
        "read_ms": "1", "project_ms": "2", "publish_ms": "3", "log_ms": "0",
        "residual_before_ms": "",
    }


def test_cpu_is_compared_with_wall_per_second_not_per_row() -> None:
    rows = [_poll(10.1, "paired", 10.0, 15.625), _poll(10.5, "paired", 10.0, 0.0),
            _poll(11.2, "empty", 4.0, 0.0)]

    assert cpu_fraction_by_second(rows, "start_wall_s", "work_ms") == [
        [15.625, 20.0], [0.0, 4.0],
    ]
    summary = summarize_poll([rows])
    assert summary["outcomes"] == {"paired": 2, "empty": 1}
    assert summary["residual_before_ms"] is None  # blank on every row here
    assert summary["actual_wait_ms"]["p50"] == pytest.approx(25.0)


def _streams(**rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    return dict(rows)


def test_reconciliation_counts_lost_and_unfinished_frames() -> None:
    vehicle = _streams(
        detector_pose=[{"outcome": "emitted", "frame_ts": ts} for ts in ("1.0", "1.1", "1.2")]
        + [{"outcome": "unpaired_pose_dropped", "frame_ts": ""}],
        observation=[{"obs_ts": "1.0", "outcome": "fresh"},
                     {"obs_ts": "1.1", "outcome": "fresh"},
                     {"obs_ts": "", "outcome": "invalid_source"}],
        frame_lineage=[{"frame_ts": "1.0", "stage": "worker", "disposition": "consumed"}],
    )

    assert reconcile_lineage([vehicle]) == {
        "emitted": 3,
        "emitted_without_observation": 1,  # 1.2
        "fresh": 2,
        "fresh_without_disposition": 1,  # 1.1: admitted, never disposed
        "disposition_without_emit": 0,
        "vehicles_skipped_multi_epoch": 0,
        "vehicles_without_emit_stream": 0,
    }


def test_two_epochs_are_skipped_not_joined_and_a_missing_emit_stream_is_named() -> None:
    restarted = _streams(
        detector_pose=[{"outcome": "emitted", "frame_ts": "1.0", "epoch": "0"}],
        observation=[{"obs_ts": "1.0", "outcome": "fresh", "epoch": "1"}],
    )
    no_emit = _streams(
        frame_lineage=[{"frame_ts": "2.0", "stage": "worker", "disposition": "consumed"}],
    )

    totals = reconcile_lineage([restarted, no_emit])

    assert totals["vehicles_skipped_multi_epoch"] == 1
    assert totals["vehicles_without_emit_stream"] == 1
    assert totals["emitted"] == 0 and totals["disposition_without_emit"] == 0


def test_a_disposition_without_any_emission_is_counted() -> None:
    vehicle = _streams(
        detector_pose=[{"outcome": "unpaired_pose_dropped", "frame_ts": ""}],
        frame_lineage=[{"frame_ts": "2.0", "stage": "worker", "disposition": "consumed"}],
    )

    assert reconcile_lineage([vehicle])["disposition_without_emit"] == 1


def test_reconciliation_never_joins_two_uavs() -> None:
    first = _streams(detector_pose=[{"outcome": "emitted", "frame_ts": "1.0"}])
    second = _streams(observation=[{"obs_ts": "1.0", "outcome": "fresh"}])

    totals = reconcile_lineage([first, second])

    assert totals["emitted_without_observation"] == 1
    assert totals["fresh_without_disposition"] == 1


def _write(directory: Path, stream: str, lines: list[str]) -> None:
    (directory / f"pose_cadence_{stream}_uav_4_pid_9.csv").write_text(
        "\n".join([HEADERS[stream], *lines]) + "\n", encoding="utf-8"
    )


def test_source_time_summary_reads_the_new_streams(tmp_path: Path) -> None:
    _write(tmp_path, "detector_poll", ["1.0,paired,10,15.625,20,25,1,2,3,4,"])
    _write(tmp_path, "detector_pose", ["1.0,1.0,emitted,1.0,0"])
    _write(tmp_path, "observation", ["1.0,1.0,1.01,fresh,0"])
    _write(tmp_path, "frame_lineage", ["1.0,worker,0,1.0,consumed"])
    _write(tmp_path, "worker", ["1.0,5.0,1.0,1.02,measured,fresh,3.0,0"])

    streams = source_time_summary(tmp_path, sys_id=4)["streams"]

    assert streams["detector_poll"]["rows"] == 1
    assert streams["frame_lineage"]["dispositions"] == {"worker/consumed": 1}
    assert streams["frame_lineage"]["reconciliation"]["fresh_without_disposition"] == 0
    assert streams["worker"]["cpu_fraction_per_second"]["p50"] == pytest.approx(0.6)
