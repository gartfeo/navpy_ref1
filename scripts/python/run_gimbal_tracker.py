#!/usr/bin/env python
"""Interactive SIYI and YOLO gimbal-tracking demo."""

from __future__ import annotations

import sys
from pathlib import Path

if not __package__:
    repository_root = Path(__file__).resolve().parents[2]
    for import_root in (repository_root, repository_root / "src"):
        import_path = str(import_root)
        while import_path in sys.path:
            sys.path.remove(import_path)
        sys.path.insert(0, import_path)

from scripts.python.run_gimbal_tracker_args import (
    ANCHOR_NAMES,
    ANCHOR_PADDING_PX,
    DETECTOR_PRESETS,
    SIYI_PROFILE_NAME,
    build_parser,
    parse_args,
)
from scripts.python.run_gimbal_tracker_assembly import (
    assemble_runner,
    resolve_model_path,
)
from scripts.python.run_gimbal_tracker_commands import (
    RunnerControlState,
    TrackSnapshot,
    anchor_pixel,
    build_anchor_k,
)
from scripts.python.run_gimbal_tracker_profile import (
    SiyiRunnerProfile,
    apply_detector_overrides,
    build_deep_search_config,
    build_tracker_config,
    load_siyi_profile,
    pick_setting,
)
from scripts.python.run_gimbal_tracker_runtime import main

__all__ = [
    "ANCHOR_NAMES", "ANCHOR_PADDING_PX", "DETECTOR_PRESETS",
    "SIYI_PROFILE_NAME",
    "RunnerControlState", "SiyiRunnerProfile", "TrackSnapshot", "anchor_pixel",
    "apply_detector_overrides", "assemble_runner", "build_anchor_k",
    "build_deep_search_config", "build_parser", "build_tracker_config",
    "load_siyi_profile", "main",
    "parse_args", "pick_setting", "resolve_model_path",
]

if __name__ == "__main__":
    main()
