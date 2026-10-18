"""Eval evidence: docking counts only from the DOCKED row (physical_docking.md)."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import pytest

from navpy.core.approach.logs.log_events import LogEvent, format_standard_event_row
from navpy.core.approach.vision_nav.diagnostics import (
    NavigationFinalApproachDiagnostics,
)
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from scripts import eval_gcs_demo_evidence as evidence
from tests.scripts.test_eval_gcs_navigation_demo_review_regressions import (
    _final_approach_row,
)

VEHICLE = 7
SNAP_ROW = "12:00:00.400,EVENT:SNAP_COMPONENTS,algorithm=VISION-NAV-PN,,,,,,,,,\n"


class _EventLogger:
    def __init__(self) -> None:
        self.events: list[tuple[LogEvent, Mapping[str, Any]]] = []

    def log_event(
        self,
        event: LogEvent,
        payload: Mapping[str, Any],
        *,
        timestamp: str | None = None,
    ) -> None:
        del timestamp
        self.events.append((event, payload))

    def wall_time_s(self) -> float:
        return 1000.0


def _docked_row(
    wall: str = "12:00:00.300",
    *,
    vehicle: int = VEHICLE,
    task: int = 1,
    dock: int = 11,
    episode: int = 1,
    closest_3d_m: float = 0.3,
    aircraft_ts: float = 1.15,
) -> str:
    """Produced by the runtime's own diagnostics, then the standard formatter."""
    logger = _EventLogger()
    diagnostics = NavigationFinalApproachDiagnostics(logger, reader=None)  # type: ignore[arg-type]
    diagnostics.record_docked(
        DockContactEvent(
            DockContactIdentity(vehicle, task, dock, episode),
            aircraft_ts,
            closest_3d_m,
            0.2,
            0.1,
        )
    )
    return _formatted(wall, logger, LogEvent.DOCKED)


def _armed_row(
    wall: str = "12:00:00.000",
    *,
    vehicle: int = VEHICLE,
    task: int = 1,
    dock: int = 11,
    episode: int = 1,
) -> str:
    logger = _EventLogger()
    diagnostics = NavigationFinalApproachDiagnostics(logger, reader=None)  # type: ignore[arg-type]
    diagnostics.record_dock_armed(DockContactIdentity(vehicle, task, dock, episode))
    return _formatted(wall, logger, LogEvent.DOCK_ARMED)


def _formatted(wall: str, logger: _EventLogger, expected: LogEvent) -> str:
    [(event, payload)] = logger.events
    assert event is expected
    return format_standard_event_row(
        wall, event, ((key, str(value)) for key, value in payload.items())
    )


def _issued(count: int = 3, start: int = 0) -> str:
    return "".join(
        _final_approach_row(
            f"12:00:00.{(start + index) * 50:03d}",
            obs=1.0 + (start + index) * 0.1,
            roll="10.0",
        )
        for index in range(count)
    )


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "navigation_debug.csv"
    path.write_text(text, encoding="utf-8")
    return path


def _parse(path: Path) -> list:
    return evidence.parse_final_approach_command_episodes(path, vehicle_id=VEHICLE)


def test_docked_row_closes_the_episode(tmp_path: Path) -> None:
    episodes = _parse(_write(tmp_path, _armed_row() + _issued() + _docked_row() + SNAP_ROW))
    assert len(episodes) == 1
    assert len(episodes[0]) == 3


def test_snap_without_docked_is_rejected(tmp_path: Path) -> None:
    """Losing the dock, or flying past it, is never counted as docking."""
    with pytest.raises(ValueError, match="SNAP without DOCKED"):
        _parse(_write(tmp_path, _armed_row() + _issued() + SNAP_ROW))


def test_duplicate_docked_is_rejected(tmp_path: Path) -> None:
    path = _write(
        tmp_path, _armed_row() + _issued() + _docked_row() + _docked_row() + SNAP_ROW,
    )
    with pytest.raises(ValueError, match="duplicate DOCKED"):
        _parse(path)


def test_stale_docked_episode_is_rejected(tmp_path: Path) -> None:
    text = (
        _armed_row(episode=2) + _issued() + _docked_row(episode=2) + SNAP_ROW
        + _armed_row(episode=2) + _issued(start=4)
        + _docked_row(episode=2, aircraft_ts=1.45) + SNAP_ROW
    )
    # The repeated DOCK_ARMED is now rejected before its DOCKED row (missed_dock.md).
    with pytest.raises(ValueError, match="DOCK_ARMED episode 2 repeats or regresses"):
        _parse(_write(tmp_path, text))


def test_docked_outside_an_episode_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="outside a final-approach episode"):
        _parse(_write(tmp_path, _armed_row() + _docked_row() + _issued() + SNAP_ROW))


@pytest.mark.parametrize(
    ("changes", "message"),
    (
        ({"vehicle": 8}, "not this UAV"),
        ({"dock": 12}, "not the UAV's own dock"),
        ({"task": 2}, "not the episode's task"),
    ),
)
def test_docked_for_another_uav_dock_or_task_is_rejected(
    tmp_path: Path, changes: dict, message: str,
) -> None:
    path = _write(
        tmp_path, _armed_row(**changes) + _issued() + _docked_row(**changes) + SNAP_ROW,
    )
    with pytest.raises(ValueError, match=message):
        _parse(path)


def test_command_row_after_docked_is_rejected(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        _armed_row() + _issued() + _docked_row() + _issued(count=1, start=7)
        + SNAP_ROW,
    )
    with pytest.raises(ValueError, match="after DOCKED"):
        _parse(path)


@pytest.mark.parametrize("closest_3d_m", (0.5, 0.75))
def test_docked_row_outside_the_net_contradicts_itself(
    tmp_path: Path, closest_3d_m: float,
) -> None:
    path = _write(
        tmp_path,
        _armed_row() + _issued() + _docked_row(closest_3d_m=closest_3d_m) + SNAP_ROW,
    )
    with pytest.raises(ValueError, match="not inside"):
        _parse(path)


def test_docked_field_order_is_strict(tmp_path: Path) -> None:
    row = _docked_row().replace("vehicle=", "uav=")
    with pytest.raises(ValueError, match="DOCKED fields/order"):
        _parse(_write(tmp_path, _armed_row() + _issued() + row + SNAP_ROW))


def test_docked_without_its_episode_armed_row_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="without a DOCK_ARMED row"):
        _parse(_write(tmp_path, _issued() + _docked_row() + SNAP_ROW))


@pytest.mark.parametrize(
    "armed_changes",
    ({"episode": 2}, {"dock": 12}, {"task": 2}, {"vehicle": 8}),
)
def test_docked_for_another_armed_episode_is_rejected(
    tmp_path: Path, armed_changes: dict,
) -> None:
    """The DOCKED identity must be the episode's own armed identity."""
    path = _write(
        tmp_path, _armed_row(**armed_changes) + _issued() + _docked_row() + SNAP_ROW,
    )
    with pytest.raises(ValueError, match="is not the armed episode"):
        _parse(path)


def test_an_armed_row_from_a_previous_episode_does_not_carry_over(
    tmp_path: Path,
) -> None:
    text = (
        _armed_row() + _issued() + _docked_row() + SNAP_ROW
        + _issued(start=4) + _docked_row(episode=2, aircraft_ts=1.45) + SNAP_ROW
    )
    with pytest.raises(ValueError, match="without a DOCK_ARMED row"):
        _parse(_write(tmp_path, text))


@pytest.mark.parametrize(
    "aircraft_ts",
    (0.99, 1.2 + evidence.DOCKED_AFTER_LAST_FRAME_MAX_S + 0.01),
)
def test_docked_outside_the_episodes_command_span_is_rejected(
    tmp_path: Path, aircraft_ts: float,
) -> None:
    """Commands span obs_ts 1.0..1.2: a contact before the first frame, or
    after a held command would have expired, belongs to no live approach."""
    path = _write(
        tmp_path,
        _armed_row() + _issued() + _docked_row(aircraft_ts=aircraft_ts) + SNAP_ROW,
    )
    with pytest.raises(ValueError, match="outside the episode's command span"):
        _parse(path)


@pytest.mark.parametrize(
    "aircraft_ts", (1.0, 1.2 + evidence.DOCKED_AFTER_LAST_FRAME_MAX_S),
)
def test_docked_at_the_span_edges_is_accepted(
    tmp_path: Path, aircraft_ts: float,
) -> None:
    path = _write(
        tmp_path,
        _armed_row() + _issued() + _docked_row(aircraft_ts=aircraft_ts) + SNAP_ROW,
    )
    assert len(_parse(path)) == 1


def test_an_armed_row_inside_an_open_episode_is_rejected(tmp_path: Path) -> None:
    text = _armed_row() + _issued() + _armed_row(episode=2) + SNAP_ROW
    with pytest.raises(ValueError, match="DOCK_ARMED inside an open"):
        _parse(_write(tmp_path, text))


# DOCK_FAILED (missed_dock.md): matched to its armed episode by identity


def _failed_row(
    wall: str = "12:00:00.500",
    *,
    vehicle: int = VEHICLE,
    task: int = 1,
    dock: int = 11,
    episode: int = 1,
    reason: str = "no_detection",
) -> str:
    logger = _EventLogger()
    diagnostics = NavigationFinalApproachDiagnostics(logger, reader=None)  # type: ignore[arg-type]
    diagnostics.record_dock_failed(DockContactIdentity(vehicle, task, dock, episode), reason)
    return _formatted(wall, logger, LogEvent.DOCK_FAILED)


def _episodes(path: Path) -> list:
    return evidence.parse_final_approach_episodes(path, vehicle_id=VEHICLE)


def test_a_failed_row_after_snap_closes_an_undocked_episode(tmp_path: Path) -> None:
    [episode] = _episodes(_write(tmp_path, _armed_row() + _issued() + SNAP_ROW + _failed_row()))
    assert not episode.docked


def test_a_failure_before_any_command_is_kept(tmp_path: Path) -> None:
    """GUIDED accepted, then lost at once: no command row, still one failure."""
    for text in (_armed_row() + SNAP_ROW + _failed_row(), _armed_row() + _failed_row()):
        episodes, failures = evidence.parse_final_approach_outcomes(
            _write(tmp_path, text), vehicle_id=VEHICLE,
        )
        assert episodes == []
        assert [(item.episode, item.reason) for item in failures] == [(1, "no_detection")]


def test_a_failed_attempt_after_a_docked_one_is_not_hidden(tmp_path: Path) -> None:
    text = (
        _armed_row() + _issued() + _docked_row() + SNAP_ROW
        + _armed_row(episode=2) + _failed_row(episode=2)
    )
    episodes, failures = evidence.parse_final_approach_outcomes(
        _write(tmp_path, text), vehicle_id=VEHICLE,
    )
    assert [episode.docked for episode in episodes] == [True]
    assert [item.episode for item in failures] == [2]


def test_a_new_episode_may_command_after_a_failed_one(tmp_path: Path) -> None:
    text = (
        _armed_row() + _issued() + SNAP_ROW + _failed_row()
        + _armed_row(episode=2) + _issued(start=4) + SNAP_ROW
    )
    assert len(_episodes(_write(tmp_path, text))) == 2


@pytest.mark.parametrize(
    ("text", "message"),
    (
        (_armed_row() + _issued() + SNAP_ROW + _failed_row() + _failed_row(),
         "duplicate DOCK_FAILED"),
        (_issued() + SNAP_ROW + _failed_row(), "without a DOCK_ARMED row"),
        (_armed_row() + _issued() + SNAP_ROW + _failed_row(episode=2),
         "is not the armed episode"),
        (_armed_row() + _issued() + SNAP_ROW + _failed_row(dock=12),
         "is not the armed episode"),
        (_armed_row(vehicle=8) + _issued() + SNAP_ROW + _failed_row(vehicle=8),
         "not this UAV"),
        (_armed_row() + _issued() + _docked_row() + SNAP_ROW + _failed_row(),
         "DOCK_FAILED and DOCKED"),
        (_armed_row() + _issued() + SNAP_ROW + _failed_row() + _issued(count=1, start=7),
         "after DOCK_FAILED"),
        (_armed_row() + _failed_row(reason=""), "no reason"),
        (_armed_row() + _issued() + _failed_row() + _docked_row() + SNAP_ROW,
         "DOCKED appeared after DOCK_FAILED"),
        (_armed_row() + _failed_row() + _armed_row() + _issued() + SNAP_ROW,
         "repeats or regresses"),
        (_armed_row(episode=5) + _failed_row(episode=5) + _armed_row(episode=2)
         + _failed_row(episode=2), "repeats or regresses"),
        (_armed_row(episode=3) + _failed_row(episode=3) + _armed_row(episode=3, dock=12)
         + _issued() + SNAP_ROW, "repeats or regresses"),
        (_armed_row(episode=5) + _issued() + _docked_row(episode=5) + SNAP_ROW
         + _armed_row(episode=2) + _failed_row(episode=2), "repeats or regresses"),
        (_armed_row(episode=5) + SNAP_ROW + _armed_row(episode=2) + _failed_row(episode=2),
         "repeats or regresses"),  # an arm with no outcome still sets the watermark
        (_armed_row(episode=2) + SNAP_ROW + _armed_row(episode=2) + _failed_row(episode=2),
         "repeats or regresses"),
        (_armed_row(episode=2) + _armed_row(episode=2) + _failed_row(episode=2),
         "repeats or regresses"),
        (_armed_row(episode=3) + _failed_row(episode=3)
         + _armed_row(episode=2) + _issued() + _docked_row(episode=2)
         + SNAP_ROW, "repeats or regresses"),  # an episode number regressed
    ),
)
def test_an_invalid_failed_row_is_rejected(tmp_path: Path, text: str, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        _episodes(_write(tmp_path, text))


def test_the_delivery_gate_reports_a_failed_dock_with_its_reason(tmp_path: Path) -> None:
    from types import SimpleNamespace

    from scripts.eval_delivery_gates import docking_errors

    rows = _armed_row(vehicle=2, dock=0) + _failed_row(vehicle=2, dock=0, reason="missed_dock")
    _, failed = evidence.parse_final_approach_outcomes(_write(tmp_path, rows), vehicle_id=2)
    errors = docking_errors(2, "", [], [SimpleNamespace(name="kept")], 0, None, failed)

    assert errors[0] == "UAV 2 dock failed at kept: missed_dock"
    assert "has 0 final-approach episodes" in errors[1]
