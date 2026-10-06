"""Thin lifecycle orchestration for one navigation evaluation case."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from eval_navigation_case_flight import fly_case, teardown_case
from eval_navigation_case_ports import CaseProcessPorts
from eval_navigation_case_result import finalize_case
from eval_navigation_case_setup import prepare_case
from eval_navigation_case_state import CasePaths, CaseState
from eval_navigation_models import MatrixCase


def run_case(
    index: int,
    case: MatrixCase,
    attempt: int,
    run_dir: Path,
    args: argparse.Namespace,
    *,
    worktree: Path,
    process_ports: CaseProcessPorts,
) -> dict[str, Any]:
    """Prepare, fly, tear down, and summarize one isolated case."""
    name = (
        f"eval-nav-a{case.poi_alt_m}-s{case.speedup}"
        f"-g{case.navigation_speedup:g}-w{case.wind_speed_mps:g}"
        f"-d{case.wind_direction_deg}-i{index}-a{attempt}"
    )
    case_dir = run_dir / name
    case_dir.mkdir(parents=True, exist_ok=True)
    state = CaseState(
        name=name,
        poi_rel_alt_m=float(case.poi_alt_m),
        certificate=bool(getattr(args, "repetitions", None)),
        paths=CasePaths(case_dir),
    )
    try:
        prepare_case(
            state,
            case,
            args,
            worktree=worktree,
            process_ports=process_ports,
        )
        fly_case(state, args)
    except Exception as exc:
        state.error = str(exc)
    finally:
        teardown_case(state, args, process_ports=process_ports)
    return finalize_case(state, case, index, attempt, args)
