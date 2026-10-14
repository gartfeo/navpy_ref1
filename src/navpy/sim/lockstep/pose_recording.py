"""Pose-cadence recording as a participant's run input (step 9g).

The hermetic environment passes no ambient recording setting
(``coordinator_spawn.hermetic_environment``). A run that records the
pose-cadence CSVs names a run-relative directory in the role config
(:data:`POSE_CADENCE_DIR`); the participant's hooks call
:func:`enable_pose_cadence` in ``construct``, before the product import,
because ``pose_cadence_debug`` reads its switch once, at import. Files are
named by producer (``navpy``, the backend's ``gcs``), so equal runs write
equal names. The participant's working directory is the run directory, so
the files land in ``<run>/<dir>``.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Mapping
from pathlib import PurePath

#: The role config key: the run-relative pose-cadence directory.
POSE_CADENCE_DIR = "pose_cadence_dir"
_RECORDER = "navpy.core.kernel.logging.pose_cadence_debug"
#: The variable the recorder reads at its import (``_resolve_output_dir``).
DEBUG_ENV = "NAVPY_POSE_CADENCE_DEBUG"
#: Values ``pose_cadence_debug`` reads as a switch, not a directory.
_SWITCH_WORDS = {"0", "1", "on", "off", "true", "false", "yes", "no"}


def enable_pose_cadence(config: Mapping[str, object]) -> None:
    """Switch recording on for this process, if the role config names a directory."""
    directory = config.get(POSE_CADENCE_DIR)
    if directory is None:
        return
    if not isinstance(directory, str) or not directory or PurePath(directory).anchor \
            or ".." in PurePath(directory).parts or directory.strip().lower() in _SWITCH_WORDS:
        raise ValueError(f"{POSE_CADENCE_DIR} must be a run-relative directory, got {directory!r}")
    if _RECORDER in sys.modules:
        raise RuntimeError("pose_cadence_debug was imported before its run input was set")
    from navpy.core.kernel.logging.pose_cadence_naming import NAMING_ENV

    os.environ[DEBUG_ENV] = directory
    os.environ[NAMING_ENV] = "producer"


__all__ = ["DEBUG_ENV", "POSE_CADENCE_DIR", "enable_pose_cadence"]
