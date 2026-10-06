"""Explicit process lifecycle dependencies for one evaluator case."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from eval_navigation_models import LaunchVerdict


@dataclass(frozen=True)
class CaseProcessPorts:
    """Process and transport operations supplied by the composition root."""

    stop_stack: Callable[..., None]
    start_swarm: Callable[
        [Path, Path, int], tuple[subprocess.Popen[Any], LaunchVerdict]
    ]
    wait_for_heartbeat: Callable[[str, float], Any | None]
    request_coordinate_stream: Callable[[Any], bool]
    launch_navpy: Callable[..., subprocess.Popen[Any]]
    terminate_child: Callable[[subprocess.Popen[Any] | None], None]


__all__ = ["CaseProcessPorts"]
