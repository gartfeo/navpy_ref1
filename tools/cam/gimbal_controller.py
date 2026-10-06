#!/usr/bin/env python
"""Public CLI façade for the detector-backed SIYI tuning tool."""

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
    from tools.cam.gimbal_tuning_args import (
        DEFAULT_MAX_RATE,
        MAX_ZOOM,
        MIN_ZOOM,
        build_parser,
        parse_args,
    )
    from tools.cam.gimbal_tuning_assembly import capture_calibrated_intrinsics
    from tools.cam.gimbal_tuning_geometry import (
        ANCHOR_NAMES,
        TrackingCommand,
        TrackingIntrinsics,
        bbox_center,
        build_bbox_tracking_command,
        pixel_to_delta,
        resolve_tracking_intrinsics,
        undistort_point,
    )
    from tools.cam.gimbal_tuning_runtime import main
    from tools.cam.gimbal_tuning_sample import (
        build_tracker_target,
        tick_gimbal_tracker,
    )
else:
    from .gimbal_tuning_args import (
        DEFAULT_MAX_RATE,
        MAX_ZOOM,
        MIN_ZOOM,
        build_parser,
        parse_args,
    )
    from .gimbal_tuning_assembly import capture_calibrated_intrinsics
    from .gimbal_tuning_geometry import (
        ANCHOR_NAMES,
        TrackingCommand,
        TrackingIntrinsics,
        bbox_center,
        build_bbox_tracking_command,
        pixel_to_delta,
        resolve_tracking_intrinsics,
        undistort_point,
    )
    from .gimbal_tuning_runtime import main
    from .gimbal_tuning_sample import (
        build_tracker_target,
        tick_gimbal_tracker,
    )


__all__ = [
    "ANCHOR_NAMES",
    "DEFAULT_MAX_RATE",
    "MAX_ZOOM",
    "MIN_ZOOM",
    "TrackingCommand",
    "TrackingIntrinsics",
    "bbox_center",
    "build_bbox_tracking_command",
    "build_parser",
    "build_tracker_target",
    "capture_calibrated_intrinsics",
    "main",
    "parse_args",
    "pixel_to_delta",
    "resolve_tracking_intrinsics",
    "tick_gimbal_tracker",
    "undistort_point",
]


if __name__ == "__main__":
    main()
