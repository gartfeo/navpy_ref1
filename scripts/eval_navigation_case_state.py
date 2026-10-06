"""Nested mutable state for one evaluator case lifecycle."""

from __future__ import annotations

import subprocess
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from scripts import eval_certificate as cert

from eval_navigation_models import (
    EvidenceGateResult,
    LaunchVerdict,
    PositionStreamAnchor,
    SelectionEvidence,
    PoiExpectation,
)
from eval_navigation_scoring import CoordinateScorer


@dataclass
class CasePaths:
    case_dir: Path
    compact: Path | None = None
    navigation: Path | None = None
    debug: Path | None = None


@dataclass
class CaseProcesses:
    swarm: subprocess.Popen[Any] | None = None
    navpy: subprocess.Popen[Any] | None = None
    master: Any | None = None
    chat: int | None = None
    sysid: int | None = None
    launch_verdict: LaunchVerdict | None = None


@dataclass
class ScoringIntervalWindow:
    """Scoring and clock-rate evidence that always share one start instant."""

    scorer: CoordinateScorer
    rate_tracker: cert.ClockRateTracker


@dataclass
class CaseEvidence:
    expectation: PoiExpectation | None = None
    home_abs_alt_m: float | None = None
    selection: SelectionEvidence | None = None
    gate: EvidenceGateResult | None = None
    scoring_interval: ScoringIntervalWindow | None = None
    live_anchors: deque[PositionStreamAnchor] = field(
        default_factory=lambda: deque(maxlen=2)
    )
    coordinate_stream_acknowledged: bool = False
    coordinate_stream_live_at_snap: bool = False
    episode_error: str | None = None
    identity: dict[str, Any] | None = None


@dataclass
class CaseState:
    name: str
    poi_rel_alt_m: float
    certificate: bool
    paths: CasePaths
    processes: CaseProcesses = field(default_factory=CaseProcesses)
    evidence: CaseEvidence = field(default_factory=CaseEvidence)
    error: str = ""
