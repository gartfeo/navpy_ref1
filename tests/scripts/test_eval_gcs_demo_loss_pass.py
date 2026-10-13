"""Eval evidence for a pass latched when the dock was lost centred (a7)."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import pytest

from navpy.core.approach.logs.log_events import LogEvent, format_standard_event_row
from navpy.core.approach.vision_nav.diagnostics import (
    NavigationFinalApproachDiagnostics,
)
from navpy.core.ports.frame import FinalApproachVisionFrame
from scripts import eval_gcs_demo_evidence as evidence
from scripts import eval_gcs_demo_metrics as metrics
from tests.scripts.test_eval_gcs_navigation_demo_review_regressions import (
    _final_approach_row,
)

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


def _pass_row(
    wall: str = "12:00:00.300",
    *,
    source: str = "measured",
    generation: int = 4,
    task: int = 1,
    obj: int = 11,
    obs: float = 1.2,
    off_axis_deg: float = 2.3,
    half_fov_deg: float = 14.4,
) -> str:
    """Produced by the runtime's own diagnostics, then the standard formatter."""
    logger = _EventLogger()
    diagnostics = NavigationFinalApproachDiagnostics(logger, reader=None)  # type: ignore[arg-type]
    diagnostics.record_loss_pass(
        FinalApproachVisionFrame(
            source, generation, task, obj, obs,
            1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            camera_off_axis_deg=off_axis_deg,
            camera_half_fov_deg=half_fov_deg,
        )
    )
    [(event, payload)] = logger.events
    assert event is LogEvent.FINAL_APPROACH_PASS
    return format_standard_event_row(
        wall, event, ((key, str(value)) for key, value in payload.items())
    )


def _issued(count: int = 3) -> str:
    return "".join(
        _final_approach_row(
            f"12:00:00.{index * 50:03d}", obs=1.0 + index * 0.1, roll="10.0"
        )
        for index in range(count)
    )


def _write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "navigation_debug.csv"
    path.write_text(text, encoding="utf-8")
    return path


def test_centred_loss_pass_row_closes_the_episode(tmp_path: Path) -> None:
    path = _write(tmp_path, _issued() + _pass_row() + SNAP_ROW)

    [episode] = evidence.parse_final_approach_command_episodes(path)

    assert [command.passed for command in episode] == [False, False, False, True]
    assert episode[-1].pass_reason == "centred_loss"
    assert not episode[-1].issued
    score = metrics.score_final_approach_commands(
        episode,
        roll_limit_deg=45.0,
        saturation_margin_deg=0.5,
        significant_roll_deg=10.0,
    )
    assert score.sample_count == 3
    timing = metrics.final_approach_timing(episode)
    # Cadence uses the issued frames only; the pass row repeats the last obs_ts.
    assert timing.observed_speedup == pytest.approx(2.0)
    assert timing.max_wall_gap_s == pytest.approx(0.05)


def test_pass_row_with_another_identity_is_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, _issued() + _pass_row(obj=12) + SNAP_ROW)

    with pytest.raises(ValueError, match="FINAL_APPROACH_PASS changed atomic source identity"):
        evidence.parse_final_approach_command_episodes(path)


def test_pass_row_outside_an_episode_is_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, _pass_row() + SNAP_ROW)

    with pytest.raises(ValueError, match="outside a final-approach episode"):
        evidence.parse_final_approach_command_episodes(path)


def test_pass_row_after_a_visual_pass_is_rejected(tmp_path: Path) -> None:
    suppressed = _final_approach_row(
        "12:00:00.200", obs=1.3, roll="", issued="False", passed="True"
    )
    path = _write(tmp_path, _issued() + suppressed + _pass_row() + SNAP_ROW)

    with pytest.raises(ValueError, match="already passed"):
        evidence.parse_final_approach_command_episodes(path)


def test_issued_command_after_a_loss_pass_is_rejected(tmp_path: Path) -> None:
    late = _final_approach_row("12:00:00.350", obs=1.4, roll="11.0")
    path = _write(tmp_path, _issued() + _pass_row() + late + SNAP_ROW)

    with pytest.raises(ValueError, match="after pass"):
        evidence.parse_final_approach_command_episodes(path)


def test_snap_without_any_pass_is_still_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, _issued() + SNAP_ROW)

    with pytest.raises(ValueError, match="SNAP without passed=True"):
        evidence.parse_final_approach_command_episodes(path)


def test_unknown_pass_reason_is_rejected(tmp_path: Path) -> None:
    row = _pass_row().replace("reason=centred_loss", "reason=guess")
    path = _write(tmp_path, _issued() + row + SNAP_ROW)

    with pytest.raises(ValueError, match="reason"):
        evidence.parse_final_approach_command_episodes(path)


@pytest.mark.parametrize(
    ("off_axis_deg", "half_fov_deg"),
    [(3.5, 14.4), (1.5, 2.0), (0.0, 0.0)],
)
def test_pass_row_outside_the_centred_predicate_is_rejected(
    tmp_path: Path, off_axis_deg: float, half_fov_deg: float
) -> None:
    """Above 3 deg, or above half the short half-FOV (high zoom)."""
    row = _pass_row(off_axis_deg=off_axis_deg, half_fov_deg=half_fov_deg)
    path = _write(tmp_path, _issued() + row + SNAP_ROW)

    with pytest.raises(ValueError, match="exceeds the centred limit"):
        evidence.parse_final_approach_command_episodes(path)


def test_pass_row_older_than_command_evidence_is_rejected(tmp_path: Path) -> None:
    path = _write(tmp_path, _issued() + _pass_row(obs=1.1) + SNAP_ROW)

    with pytest.raises(ValueError, match="precedes earlier command evidence"):
        evidence.parse_final_approach_command_episodes(path)


def test_pass_row_at_the_limit_is_accepted(tmp_path: Path) -> None:
    path = _write(tmp_path, _issued() + _pass_row(off_axis_deg=3.0) + SNAP_ROW)

    [episode] = evidence.parse_final_approach_command_episodes(path)
    assert episode[-1].passed
