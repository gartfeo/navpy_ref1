#!/usr/bin/env python
"""Static ideal-camera point-mass diagnostics for final approach."""

from __future__ import annotations

import sys
from pathlib import Path

if not __package__:
    repository_root = Path(__file__).resolve().parents[1]
    for import_root in (repository_root, repository_root / "src"):
        import_path = str(import_root)
        while import_path in sys.path:
            sys.path.remove(import_path)
        sys.path.insert(0, import_path)

from scripts.vision_static_point_mass_cli import build_parser, main
from scripts.vision_static_point_mass_run import (
    default_cases,
    run_case,
    run_case_analysis,
)
from scripts.vision_static_point_mass_sensor import (
    FOCAL_X_PX,
    FOCAL_Y_PX,
    FRAME_HEIGHT_PX,
    FRAME_WIDTH_PX,
    PRINCIPAL_X_PX,
    PRINCIPAL_Y_PX,
    build_static_detection,
)
from scripts.vision_static_point_mass_types import (
    StaticPointMassCase,
    StaticPointMassMiss,
    StaticPointMassRun,
)

__all__ = [
    "FOCAL_X_PX",
    "FOCAL_Y_PX",
    "FRAME_HEIGHT_PX",
    "FRAME_WIDTH_PX",
    "PRINCIPAL_X_PX",
    "PRINCIPAL_Y_PX",
    "StaticPointMassCase",
    "StaticPointMassMiss",
    "StaticPointMassRun",
    "build_parser",
    "build_static_detection",
    "default_cases",
    "main",
    "run_case",
    "run_case_analysis",
]

if __name__ == "__main__":
    raise SystemExit(main())
