"""Characteristic pixel size of a detection bbox.

The confirmation source gate and the best-frame ranking both ask "is the
target big enough in pixels to recognize?". The answer uses the bbox
DIAGONAL — ``sqrt(w**2 + h**2)`` — a single extent that accounts for
both dimensions instead of height alone. A wide detection (recognizable mostly from its width) is no longer under-read by height; a
tall, narrow target (a standing person) still reads ~its height because
the larger dimension dominates the diagonal. One measure, no per-class
special-casing.
"""

from __future__ import annotations

import math


def characteristic_pixels(width: float, height: float) -> float:
    """Recognizable pixel extent of a bbox: the diagonal ``sqrt(w**2+h**2)``.

    ``width``/``height`` are the bbox dimensions in pixels and must be
    finite and positive — callers extract them via a validating accessor,
    so a non-finite or non-positive value is a contract violation, not an
    expected runtime state (and must never reach a recognition gate as
    NaN/inf).
    """
    if not (math.isfinite(width) and math.isfinite(height)):
        raise ValueError(
            f"bbox dimensions must be finite, got w={width}, h={height}"
        )
    if width <= 0.0 or height <= 0.0:
        raise ValueError(
            f"bbox dimensions must be positive, got w={width}, h={height}"
        )
    return math.sqrt(width * width + height * height)
