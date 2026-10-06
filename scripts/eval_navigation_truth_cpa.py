"""Canonical-sequence CPA and closure math for truth certification.

Certification must never run on raw arrivals: a duplicated 20 Hz stream looks
like 40 Hz by arrival count, and tolerated reordering can put chronologically
earlier points into the apparent post-CPA tail.  Everything here therefore
works on the canonical sequence -- source-time sorted, exact-duplicate
timestamps collapsed -- which matches the order the scorer's reorder window
releases samples in.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import PositionSample, PoiLocation  # noqa: E402
from eval_navigation_scoring import _local_vector  # noqa: E402


def canonical_samples(
    samples: Sequence[PositionSample],
) -> tuple[list[PositionSample], int]:
    """Source-time sorted samples with exact-duplicate timestamps collapsed.

    Returns the canonical list and the number of collapsed arrivals.
    """
    ordered = sorted(samples, key=lambda sample: sample.source_time_s)
    unique: list[PositionSample] = []
    collapsed = 0
    for sample in ordered:
        if unique and sample.source_time_s == unique[-1].source_time_s:
            collapsed += 1
            continue
        unique.append(sample)
    return unique, collapsed


@dataclass(frozen=True)
class SegmentCpa:
    dist_3d_m: float
    segment_index: int
    fraction: float
    source_time_s: float


def segment_cpa(
    samples: Sequence[PositionSample],
    poi: PoiLocation,
) -> SegmentCpa | None:
    """Exact minimum of the piecewise-linear track, with its source time.

    Same geometry as ``CoordinateScorer`` (projection onto each segment);
    additionally keeps which segment won and where on it, so the CPA has an
    interpolated source time the correlation work can use.
    """
    if not samples:
        return None
    vectors = [_local_vector(sample, poi) for sample in samples]
    best: SegmentCpa | None = None
    for index, vector in enumerate(vectors):
        if index == 0:
            distance = math.sqrt(sum(c * c for c in vector))
            candidate = SegmentCpa(
                distance, 0, 0.0, samples[0].source_time_s
            )
        else:
            previous = vectors[index - 1]
            delta = tuple(vector[i] - previous[i] for i in range(3))
            denominator = sum(c * c for c in delta)
            fraction = 0.0
            if denominator > 1e-12:
                fraction = max(0.0, min(
                    1.0,
                    -sum(previous[i] * delta[i] for i in range(3))
                    / denominator,
                ))
            point = tuple(
                previous[i] + fraction * delta[i] for i in range(3)
            )
            distance = math.sqrt(sum(c * c for c in point))
            t_prev = samples[index - 1].source_time_s
            t_cur = samples[index].source_time_s
            candidate = SegmentCpa(
                distance,
                index - 1,
                fraction,
                t_prev + fraction * (t_cur - t_prev),
            )
        if best is None or candidate.dist_3d_m < best.dist_3d_m:
            best = candidate
    return best


def closure_evidence(
    samples: Sequence[PositionSample],
    poi: PoiLocation,
) -> tuple[int, float, SegmentCpa | None]:
    """Post-CPA sample count and distance rise, on the canonical sequence."""
    cpa = segment_cpa(samples, poi)
    if cpa is None:
        return 0, 0.0, None
    # fraction 1.0 means the minimum IS the segment's end sample, so the tail
    # of strictly-post-CPA evidence starts one sample later; counting the CPA
    # sample itself would let certification pass one sample short.
    tail_start = cpa.segment_index + (2 if cpa.fraction >= 1.0 else 1)
    distances = [
        math.sqrt(sum(c * c for c in _local_vector(sample, poi)))
        for sample in samples[tail_start:]
    ]
    rise = max(distances, default=cpa.dist_3d_m) - cpa.dist_3d_m
    return len(distances), rise, cpa


__all__ = [
    "SegmentCpa",
    "canonical_samples",
    "closure_evidence",
    "segment_cpa",
]
