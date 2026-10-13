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
    with pytest.raises(ValueError, match="stale DOCKED episode"):
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
