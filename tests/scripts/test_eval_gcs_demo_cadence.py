"""Approach-window observation cadence: source time, boundaries, keys."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import pytest

from navpy.core.approach.logs.log_events import format_standard_event_row
from navpy.core.approach.vision_nav.diagnostics import NavigationFinalApproachDiagnostics
from navpy.core.ports.dock_contact import DockContactIdentity
from scripts.eval_gcs_demo_cadence import (
    COVERAGE_GAP_BOUND_MS,
    UNTERMINATED,
    Window,
    approach_windows,
    cadence_annotation,
    episode_cadence,
)

_HEADER = "ts,event,payload,,,,,,,,,"
_IDENTITY = "vehicle=4;task=1;dock=11;episode=1"
# Local noon, the debug rows' clock: observation wall_s are offsets from it.
_NOON_S = datetime(2026, 10, 16, 12).timestamp()
_ARMED = f"{_IDENTITY};source_ts=10.0;wall_s={_NOON_S}"


def _event(name: str, payload: str, wall: str = "12:00:00.000") -> str:
    return f"{wall},EVENT:{name},{payload},,,,,,,,,"


def _write_debug(log_dir: Path, *rows: str) -> None:
    (log_dir / "uav_4_navigation_debug.csv").write_text(
        "\n".join((_HEADER, *rows)) + "\n", encoding="utf-8"
    )


def _write_stream(log_dir: Path, stream: str, header: str, rows: list[str]) -> None:
    source = log_dir / "source-time"
    source.mkdir(exist_ok=True)
    (source / f"pose_cadence_{stream}_uav_4_pid_7.csv").write_text(
        "\n".join((header, *rows)) + "\n", encoding="utf-8"
    )


def _write_observations(log_dir: Path, rows: list[tuple[float, float, str, int]]) -> None:
    """(seconds after local noon, obs_ts, outcome, epoch) rows."""
    _write_stream(log_dir, "observation", "wall_s,obs_ts,src_now_s,outcome,epoch", [
        f"{_NOON_S + wall:.3f},{obs:.3f},{obs + 0.01:.3f},{outcome},{epoch}"
        for wall, obs, outcome, epoch in rows
    ])


def _cadence(window, keys, **kwargs):
    """(epoch, obs_ts) keys that all arrived inside the window."""
    return episode_cadence(window, [(*key, 0.0) for key in keys], **kwargs)


def _window(entry: float = 10.0, exit_s: float = 11.0) -> Window:
    return Window(1, entry, exit_s, "docked", 0.0, None)


def test_boundary_gaps_count_and_the_longest_gap_is_kept() -> None:
    # Starved start: the first observation is 400 ms after entry.
    keys = [(0, 10.4), (0, 10.49), (0, 10.58), (0, 10.67), (0, 10.76), (0, 10.85)]

    result = _cadence(_window(exit_s=10.94), keys)

    assert result["status"] == "ok"
    assert result["gap_count"] == 7  # entry->first, 5 inner, last->exit
    assert result["longest_gap_ms"] == pytest.approx(400.0)
    assert (result["longest_gap_from_s"], result["longest_gap_to_s"]) == (10.0, 10.4)
    assert result["gap_median_ms"] == pytest.approx(90.0)
    # Only the 400 ms gap is over the 100 ms bound.
    assert result["coverage"] == pytest.approx(1.0 - 400.0 / 940.0)


def test_starved_exit_is_a_gap_too() -> None:
    result = _cadence(_window(), [(0, 10.05), (0, 10.1), (0, 10.15)])

    assert result["longest_gap_ms"] == pytest.approx(850.0)
    assert result["longest_gap_to_s"] == 11.0


def test_duplicates_count_once() -> None:
    keys = [(0, 10.2), (0, 10.2), (0, 10.2), (0, 10.6)]

    result = _cadence(_window(), keys)

    assert result["observations"] == 2
    assert result["duplicates"] == 2
    assert result["gap_count"] == 3


def test_out_of_order_keys_are_counted_and_not_used_for_gaps() -> None:
    result = _cadence(_window(), [(0, 10.5), (0, 10.3), (0, 10.6)])

    assert result["out_of_order"] == 1
    assert result["observations"] == 2
    assert result["longest_gap_ms"] == pytest.approx(500.0)


def test_source_restart_inside_the_window_ends_the_segment() -> None:
    # Boot time resets to zero; the NAV-exit row is logged after the reset.
    keys = [(0, 10.1), (0, 10.2), (1, 0.05), (1, 0.1)]

    result = _cadence(Window(1, 10.0, 0.3, "nav_exit", 0.0, None), keys)

    assert result["ended_by"] == "source_restart"
    assert result["observations"] == 2
    assert result["gap_count"] == 2  # no last->exit gap across the restart
    assert result["longest_gap_ms"] == pytest.approx(100.0)


def test_two_epochs_inside_one_window_are_ambiguous_not_merged() -> None:
    keys = [(0, 10.1), (0, 10.2), (1, 10.25), (1, 10.3)]

    result = _cadence(_window(), keys)

    assert result["status"] == "ambiguous source epoch"
    assert "gap_count" not in result


def test_gaps_at_the_bound_do_not_count_against_coverage() -> None:
    # 10.4 - 10.3 is 100.00000000000142 ms in floating point.
    keys = [(0, 10.0 + 0.1 * step) for step in range(1, 10)]

    result = _cadence(_window(), keys)

    assert result["coverage"] == 1.0
    assert _cadence(_window(), keys, bound_ms=99.0)["coverage"] == 0.0


def test_zero_and_one_observation_report_no_gap_stats() -> None:
    for keys, count in (([], 0), ([(0, 10.5)], 1)):
        result = _cadence(_window(), keys)
        assert result["status"] == f"entered with {count} observation(s)"
        assert "gap_count" not in result


def test_observations_outside_the_window_are_not_used() -> None:
    keys = [(0, 9.9), (0, 10.5), (0, 10.6), (0, 11.2)]

    assert _cadence(_window(), keys)["observations"] == 2


def test_missing_source_time_is_reported_not_guessed() -> None:
    result = _cadence(Window(1, None, 11.0, "docked"), [(0, 10.5)])

    assert result["status"] == "source time unavailable"


def test_windows_close_at_docked_disarmed_or_log_end(tmp_path: Path) -> None:
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", _ARMED),
        _event("DOCKED", f"{_IDENTITY};aircraft_ts=10.8;closest_3d_m=0.2;"
               "closest_h_m=0.1;closest_v_m=0.1", wall="12:00:00.800"),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=10.9"),
        _event("DOCK_ARMED", "vehicle=4;task=1;dock=11;episode=2;source_ts=20.0;"
               f"wall_s={_NOON_S + 10}", wall="12:00:10.000"),
        _event("SNAP_COMPONENTS", "x=1"),
        _event("DOCK_DISARMED", "vehicle=4;task=1;dock=11;episode=2;source_ts=21.0",
               wall="12:00:11.000"),
        _event("DOCK_ARMED", "vehicle=4;task=1;dock=11;episode=3;source_ts=30.0;"
               f"wall_s={_NOON_S + 20}", wall="12:00:20.000"),
    )

    windows = approach_windows(tmp_path / "uav_4_navigation_debug.csv", 32.0)

    assert windows == [
        Window(1, 10.0, 10.8, "docked", _NOON_S, pytest.approx(_NOON_S + 0.8)),
        Window(2, 20.0, 21.0, "snap", _NOON_S + 10, _NOON_S + 11),
        Window(3, 30.0, 32.0, "log_end", _NOON_S + 20, None),
    ]


def test_docked_logged_after_the_nav_exit_still_closes_the_window(tmp_path: Path) -> None:
    # The wall eval's order: NAV exit's DOCK_DISARMED, then the contact row.
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", _ARMED),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=10.9"),
        _event("DOCKED", f"{_IDENTITY};aircraft_ts=10.85;closest_3d_m=0.2;"
               "closest_h_m=0.1;closest_v_m=0.1"),
    )

    assert approach_windows(tmp_path / "uav_4_navigation_debug.csv", None) == [
        Window(1, 10.0, 10.85, "docked", _NOON_S, _NOON_S)
    ]


def test_legacy_armed_row_has_no_source_time(tmp_path: Path) -> None:
    _write_debug(tmp_path, _event("DOCK_ARMED", _IDENTITY))

    (window,) = approach_windows(tmp_path / "uav_4_navigation_debug.csv", None)

    assert window.entry_s is None


def test_a_truncated_terminal_row_is_a_status_not_a_crash(tmp_path: Path) -> None:
    _write_debug(tmp_path, _event("DOCK_ARMED", _ARMED), "12:00:01.000,EVENT:DOCK_DISARMED")

    assert cadence_annotation(tmp_path, 4)["status"].startswith("unavailable")


def test_an_armed_row_without_wall_time_fails_closed(tmp_path: Path) -> None:
    _write_observations(tmp_path, [(0.1, 10.1, "fresh", 0), (0.2, 10.2, "fresh", 0)])
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", f"{_IDENTITY};source_ts=10.0"),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=11.0", wall="12:00:01.000"),
    )

    (episode,) = cadence_annotation(tmp_path, 4)["episodes"]

    assert episode["status"] == "arrival time unavailable"
    assert "gap_count" not in episode


def test_an_unterminated_episode_does_not_absorb_the_next(tmp_path: Path) -> None:
    # Episode 1 has no terminal row; episode 2 reuses its source times.
    _write_observations(tmp_path, [
        (0.1 * step, 10.0 + 0.1 * step, "fresh", 0) for step in range(1, 5)
    ] + [(30.0 + 0.1 * step, 10.0 + 0.1 * step, "fresh", 1) for step in range(1, 9)])
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", _ARMED),
        _event("DOCK_ARMED", "vehicle=4;task=1;dock=11;episode=2;source_ts=10.0;"
               f"wall_s={_NOON_S + 30}", wall="12:00:30.000"),
        _event("DOCK_DISARMED", "vehicle=4;task=1;dock=11;episode=2;source_ts=11.0",
               wall="12:00:31.000"),
    )

    first, second = cadence_annotation(tmp_path, 4)["episodes"]

    assert first["status"] == UNTERMINATED and "gap_count" not in first
    assert second["status"] == "ok" and second["observations"] == 8


def test_arrival_bounds_cross_midnight_from_the_armed_wall_time(tmp_path: Path) -> None:
    # An epoch far from local time: only the armed row's own pair places rows.
    _write_stream(tmp_path, "observation", "wall_s,obs_ts,src_now_s,outcome,epoch", [
        f"{1000.0 + 0.1 * step:.3f},{10.0 + 0.1 * step:.3f},{10.0:.3f},fresh,0"
        for step in range(1, 10)
    ])
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", f"{_IDENTITY};source_ts=10.0;wall_s=1000.0",
               wall="23:59:59.500"),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=11.0", wall="00:00:00.500"),
    )

    (episode,) = cadence_annotation(tmp_path, 4)["episodes"]

    assert episode["status"] == "ok"
    assert episode["observations"] == 9


def _run_with_wall_jitter(tmp_path: Path) -> dict:
    """Source gaps 100 ms with one 300 ms hole; wall time evenly 50 ms."""
    obs = [10.1, 10.2, 10.5, 10.6, 10.7, 10.8, 10.9]
    rows = [(0.1 + 0.05 * index, ts, "fresh", 0) for index, ts in enumerate(obs)]
    rows.insert(2, (0.17, 10.2, "fresh", 0))  # a duplicate key
    rows.insert(0, (0.05, 10.15, "stale_source", 0))  # not fresh
    _write_observations(tmp_path, rows)
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", _ARMED),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=11.0", wall="12:00:05.000"),
    )
    return cadence_annotation(tmp_path, 4)


def test_annotation_uses_source_time_not_wall_time(tmp_path: Path) -> None:
    annotation = _run_with_wall_jitter(tmp_path)

    (episode,) = annotation["episodes"]
    assert annotation["gate"] is False
    assert annotation["coverage_gap_bound_ms"] == COVERAGE_GAP_BOUND_MS
    # Wall rows are 50 ms apart; only source time shows the 300 ms hole.
    assert episode["longest_gap_ms"] == pytest.approx(300.0)
    assert episode["observations"] == 7
    assert episode["duplicates"] == 1
    assert episode["ended_by"] == "nav_exit"


def test_not_entered_and_unreadable_evidence_are_statuses(tmp_path: Path) -> None:
    assert cadence_annotation(tmp_path, 4)["status"].startswith("unavailable")
    _write_debug(tmp_path)

    assert cadence_annotation(tmp_path, 4)["status"] == "approach not entered"


def test_log_end_is_the_last_received_attitude_boot_time(tmp_path: Path) -> None:
    # Received last (wall 3.0), not the largest boot time (12000 at wall 1.5).
    _write_stream(tmp_path, "attitude_arrival", "wall_s,boot_ms",
                  ["1.0,10500", "1.5,12000", "3.0,11500", "2.0,11000"])
    source = tmp_path / "source-time"  # the GCS's own arrivals do not count
    (source / "pose_cadence_attitude_arrival_uav_4_gcs.csv").write_text(
        "wall_s,boot_ms\n9.0,99000\n", encoding="utf-8"
    )
    _write_observations(tmp_path, [(1.0, 10.2, "fresh", 0), (1.1, 10.3, "fresh", 0)])
    _write_debug(tmp_path, _event("DOCK_ARMED", _ARMED))

    (episode,) = cadence_annotation(tmp_path, 4)["episodes"]

    assert episode["exit_s"] == pytest.approx(11.5)
    assert episode["longest_gap_ms"] == pytest.approx(1200.0)


def test_producer_rows_parse_into_one_window(tmp_path: Path) -> None:
    rows: list[str] = []

    class _Logger:
        def wall_time_s(self) -> float:
            return 5000.0

        def log_event(self, event, payload, timestamp=None):
            rows.append(format_standard_event_row(
                "12:00:00.000", event, ((k, str(v)) for k, v in payload.items())
            ).rstrip("\n"))

    diagnostics = NavigationFinalApproachDiagnostics(_Logger(), reader=None)  # type: ignore[arg-type]
    identity = DockContactIdentity(4, 1, 11, 1)
    diagnostics.record_dock_armed(identity, 10.0)
    diagnostics.record_dock_disarmed(identity, 10.9)
    _write_debug(tmp_path, *rows)

    assert approach_windows(tmp_path / "uav_4_navigation_debug.csv", None) == [
        Window(1, 10.0, 10.9, "nav_exit", 5000.0, 5000.0)
    ]


def test_rows_that_arrived_outside_the_episode_are_not_its_own(tmp_path: Path) -> None:
    # Another process (file) logged the same source times an hour earlier.
    _write_stream(tmp_path, "observation", "wall_s,obs_ts,src_now_s,outcome,epoch", [
        f"{_NOON_S - 3600 + 0.1 * step:.3f},{10.0 + 0.1 * step:.3f},{10.0:.3f},fresh,0"
        for step in range(1, 9)
    ])
    _write_debug(
        tmp_path,
        _event("DOCK_ARMED", _ARMED),
        _event("DOCK_DISARMED", f"{_IDENTITY};source_ts=11.0", wall="12:00:01.000"),
    )

    (episode,) = cadence_annotation(tmp_path, 4)["episodes"]

    assert episode["status"] == "entered with 0 observation(s)"


def test_a_restart_seen_on_arrival_ends_the_segment_without_an_exit_gap() -> None:
    # The new epoch's rows arrive inside the episode, below its entry time.
    window = Window(1, 10.0, 11.0, "nav_exit", 100.0, 200.0)
    rows = [(0, 10.1, 101.0), (0, 10.2, 102.0), (1, 0.05, 103.0), (1, 0.1, 104.0)]

    result = episode_cadence(window, rows)

    assert result["ended_by"] == "source_restart"
    assert result["gap_count"] == 2


def test_a_later_episode_epoch_does_not_end_this_one() -> None:
    window = Window(1, 10.0, 11.0, "docked", 100.0, 102.5)
    rows = [(0, 10.1, 101.0), (0, 10.2, 102.0), (1, 0.05, 300.0)]

    assert episode_cadence(window, rows)["ended_by"] == "docked"
