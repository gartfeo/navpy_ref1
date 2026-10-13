"""Sim-only OpenCV thread count, set by evaluators that need reproducible runs.

OpenCV sizes its thread pool from the host's cores, and OpenCV 5 ignores its
own ``OPENCV_FOR_THREADS_NUM``. Interactive sim keeps the default; an
evaluator pins the count through this variable.
"""

from __future__ import annotations

import os
from collections.abc import Mapping

import cv2


SIM_CV_THREADS_ENV = "NAVPY_SIM_CV_THREADS"


def apply_sim_cv_threads(env: Mapping[str, str] | None = None) -> int | None:
    """Apply the requested thread count; return it, or None when unset."""
    raw = (os.environ if env is None else env).get(SIM_CV_THREADS_ENV)
    if raw is None:
        return None
    count = int(raw)
    if count < 1:
        raise ValueError(f"{SIM_CV_THREADS_ENV} must be >= 1, got {raw!r}")
    cv2.setNumThreads(count)
    return count


__all__ = ["SIM_CV_THREADS_ENV", "apply_sim_cv_threads"]
