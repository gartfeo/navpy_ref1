"""Certified simulator-truth capture and scoring for the direct-pixel harness.

The harness's snap/coordinate scores are EKF position outputs; the 2026-09-03
proof pass showed they equal the projection of the EKF position error and can
rank runs opposite to simulator truth.  This
module scores against the simulator's own state instead: SIM_STATE messages on
the evaluator monitor link, which carry the same int32 degE7 coordinates as
the DataFlash SIM truth record plus the navlink ``time_us`` source stamp.

Certification is deliberately stricter than ``CoordinateScorer``'s invariants,
and runs on the CANONICAL sequence (source-time sorted, duplicate timestamps
collapsed -- ``eval_navigation_truth_cpa``), never on raw arrivals: a duplicated
20 Hz stream counts as 40 Hz by arrivals, and tolerated reordering could put
chronologically earlier points into the apparent post-CPA tail.  The scorer's
0.15 s gap gate would certify a stream degraded to 10 Hz and cannot see a
trailing stall, and a command ACK does not prove delivery, so the recorder
certifies the observed stream itself: degE7 fields actually present (the float
fallback of the runtime parser is never authoritative), monotonic ``time_us``,
delivered rate over the scored span, a gap gate sized for the nominal 40 Hz
stream, trailing freshness on a monotonic clock at the required finalize, and
post-CPA closure (an interior minimum with the distance rising afterwards).
"""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

# Siblings are imported as top-level modules, which only resolves when this
# directory is on the path.  Do it here rather than relying on another script
# having been imported first: without this the module (and its test) fails
# standalone with ModuleNotFoundError.
_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_navigation_models import PositionSample, PoiLocation  # noqa: E402
from eval_navigation_scoring import CoordinateScorer  # noqa: E402
from eval_navigation_truth_cpa import (  # noqa: E402
    canonical_samples, closure_evidence,
)
from eval_navigation_truth_samples import (  # noqa: E402
    TruthRecord, partial_truth_fields, truth_position_sample, write_track_csv,
)


# The ArduPlane scheduler caps message streams at SCHED_LOOP_RATE (50 Hz for
# plane) times the 0.8 ArduPilot rate-scheduler fraction, so 40 Hz is the
# fastest SIM_STATE stream a request can obtain -- the same figure the
# companion's pose stream uses (navpy pose_streams.POSE_STREAM_RATE_HZ).
TRUTH_SCORE_RATE_HZ = 40.0
# Three nominal 25 ms intervals: local burst loss beyond that voids the score.
TRUTH_MAX_SOURCE_GAP_S = 3.0 / TRUTH_SCORE_RATE_HZ
# Catches a watcher halving the stream, which the gap gate alone would pass.
TRUTH_MIN_DELIVERED_RATE_HZ = 30.0
TRUTH_MIN_SCORING_SAMPLES = 100
TRUTH_MIN_SCORING_SPAN_S = 5.0
# Monotonic age of the newest scored sample when the flight finalizes; a
# stream that died mid-leg produces no late samples and fails here.
TRUTH_TRAILING_MAX_AGE_S = 2.0
# Interior-minimum evidence: the trajectory must continue past the closest
# point and the distance must rise by more than position quantization noise
# (degE7 is 1.11 cm north / 0.81 cm east at the test site).
TRUTH_CLOSURE_MIN_POST_SAMPLES = 8
TRUTH_CLOSURE_MIN_RISE_M = 0.05
# Bounded in-flight buffer; overflow invalidates the score rather than
# silently dropping evidence.
TRUTH_TRACK_MAX_RECORDS = 100_000


class TruthRecorder:
    """Buffer, score, and certify the SIM_STATE truth stream for one case."""

    def __init__(
        self,
        poi: PoiLocation,
        home_abs_alt_m: float,
        *,
        monotonic_now: Callable[[], float] = time.monotonic,
    ) -> None:
        self._poi = poi
        self._home_abs_alt_m = home_abs_alt_m
        self._monotonic_now = monotonic_now
        self._scorer = CoordinateScorer(poi)
        self._records: list[TruthRecord] = []
        self._scoring_active_samples: list[PositionSample] = []
        self._overflowed = False
        self._scoring_active_live_edge_s: float | None = None
        self._last_scoring_active_monotonic_s: float | None = None
        self._final_monotonic_s: float | None = None

    def add_message(
        self,
        message: Any,
        received_wall_time_s: float,
        *,
        scoring_active: bool,
    ) -> None:
        sample, reason = truth_position_sample(
            message, received_wall_time_s, self._home_abs_alt_m
        )
        if len(self._records) >= TRUTH_TRACK_MAX_RECORDS:
            self._overflowed = True
            return
        if sample is None:
            # Keep whatever the rejected message still decodes to: the CSV is
            # the audit trail for exactly these messages.
            self._records.append(TruthRecord(
                received_wall_time_s, *partial_truth_fields(message),
                scoring_active, False, reason,
            ))
            return
        self._records.append(TruthRecord(
            received_wall_time_s, sample.source_time_s,
            sample.lat_deg, sample.lon_deg, sample.abs_alt_m,
            scoring_active, True, None,
        ))
        if scoring_active:
            self._scoring_active_samples.append(sample)
            self._scorer.add(sample)
            # Trailing freshness follows the SOURCE clock's live edge, not
            # mere arrival: a source that stalls and re-sends its final
            # sample would otherwise look fresh at finalize, because exact
            # duplicates collapse out of every other certification check.
            if (
                self._scoring_active_live_edge_s is None
                or sample.source_time_s > self._scoring_active_live_edge_s
            ):
                self._scoring_active_live_edge_s = sample.source_time_s
                self._last_scoring_active_monotonic_s = self._monotonic_now()

    def finalize(self) -> None:
        # First declaration binds: a later bookkeeping echo (fleet verdict
        # pass, failure salvage) must not move the freshness stamp.
        if self._final_monotonic_s is None:
            self._final_monotonic_s = self._monotonic_now()

    def closure_ready(self) -> bool:
        """Cheap post-CPA check for the flight loop's trailing drain."""
        canonical, _ = canonical_samples(self._scoring_active_samples)
        post_samples, rise, _ = closure_evidence(canonical, self._poi)
        return (
            post_samples >= TRUTH_CLOSURE_MIN_POST_SAMPLES
            and rise >= TRUTH_CLOSURE_MIN_RISE_M
        )

    def write_track(self, path: Path) -> None:
        write_track_csv(self._records, path)

    def _certification_errors(self) -> list[str]:
        errors: list[str] = []
        if self._overflowed:
            errors.append(
                f"truth track exceeded {TRUTH_TRACK_MAX_RECORDS} records"
            )
        # Benign UDP reorder is tolerated the way the scorer tolerates it
        # (0.15 s window); jumps beyond that already invalidate the scorer.
        scorer_error = self._scorer.certification_error
        if scorer_error:
            errors.append(f"truth stream: {scorer_error}")
        if self._final_monotonic_s is None:
            errors.append("truth recorder was never finalized")
        canonical, _ = canonical_samples(self._scoring_active_samples)
        count = len(canonical)
        if count < TRUTH_MIN_SCORING_SAMPLES:
            errors.append(
                f"insufficient scoring-window truth samples: {count} unique "
                f"(need {TRUTH_MIN_SCORING_SAMPLES})"
            )
            return errors
        times = [sample.source_time_s for sample in canonical]
        span = times[-1] - times[0]
        if span < TRUTH_MIN_SCORING_SPAN_S:
            errors.append(
                f"scoring-window truth span {span:.2f}s below "
                f"{TRUTH_MIN_SCORING_SPAN_S:.2f}s"
            )
            return errors
        rate = (count - 1) / span
        if rate < TRUTH_MIN_DELIVERED_RATE_HZ:
            errors.append(
                f"truth delivered rate {rate:.1f}Hz below "
                f"{TRUTH_MIN_DELIVERED_RATE_HZ:.1f}Hz"
            )
        gap = max(
            (second - first for first, second in zip(times, times[1:])),
            default=0.0,
        )
        if gap > TRUTH_MAX_SOURCE_GAP_S:
            errors.append(
                f"truth source gap {gap:.3f}s exceeds "
                f"{TRUTH_MAX_SOURCE_GAP_S:.3f}s"
            )
        if (
            self._final_monotonic_s is not None
            and self._last_scoring_active_monotonic_s is not None
        ):
            age = self._final_monotonic_s - self._last_scoring_active_monotonic_s
            if age > TRUTH_TRAILING_MAX_AGE_S:
                errors.append(
                    f"truth stream stale at finalize: last sample {age:.2f}s "
                    f"old (limit {TRUTH_TRAILING_MAX_AGE_S:.2f}s)"
                )
        post_samples, rise, cpa = closure_evidence(canonical, self._poi)
        if post_samples < TRUTH_CLOSURE_MIN_POST_SAMPLES:
            errors.append(
                f"CPA not closed: {post_samples} samples after the minimum "
                f"(need {TRUTH_CLOSURE_MIN_POST_SAMPLES})"
            )
        elif rise < TRUTH_CLOSURE_MIN_RISE_M:
            errors.append(
                f"CPA not closed: distance rises {rise:.3f}m after the "
                f"minimum (need {TRUTH_CLOSURE_MIN_RISE_M:.3f}m)"
            )
        best = self._scorer.result
        if best is not None and cpa is not None and (
            abs(best.dist_3d_m - cpa.dist_3d_m) > 1e-6
        ):
            errors.append(
                "internal inconsistency: scorer CPA and canonical segment "
                "CPA disagree"
            )
        return errors

    def verdict(self) -> dict[str, Any]:
        """Truth block for the case verdict; certification decides validity."""
        best = self._scorer.result
        errors = self._certification_errors()
        canonical, collapsed = canonical_samples(self._scoring_active_samples)
        times = [sample.source_time_s for sample in canonical]
        span = times[-1] - times[0] if len(times) >= 2 else 0.0
        post_samples, rise, cpa = closure_evidence(canonical, self._poi)
        return {
            "dist_3d_m": None if best is None else best.dist_3d_m,
            "horizontal_m": None if best is None else best.horizontal_m,
            "vertical_m": None if best is None else best.vertical_m,
            "cpa_source_time_s": None if cpa is None else cpa.source_time_s,
            "cpa_segment_fraction": None if cpa is None else cpa.fraction,
            "scoring_samples": len(canonical),
            "scoring_span_s": span,
            "scoring_sample_rate_hz": (
                (len(canonical) - 1) / span if span > 0.0 else 0.0
            ),
            "collapsed_duplicate_arrivals": collapsed,
            "scorer_duplicates": self._scorer.duplicate_sample_count,
            "scorer_reordered": self._scorer.reordered_sample_count,
            "scorer_late": self._scorer.late_sample_count,
            "closure_post_samples": post_samples,
            "closure_rise_m": rise,
            "pre_scoring_samples": sum(
                1 for record in self._records
                if record.converted and not record.scoring_active
            ),
            "rejected_samples": sum(
                1 for record in self._records if not record.converted
            ),
            "finalized": self._final_monotonic_s is not None,
            "certification_error": "; ".join(errors) if errors else None,
        }

    @property
    def certified(self) -> bool:
        return not self._certification_errors()


def salvage_truth(
    recorder: TruthRecorder | None,
    case_dir: Path,
    errors: list[str],
) -> dict[str, Any] | None:
    """Persist and score what the recorder observed, best-effort.

    For a case that failed late: the audit trail must survive the
    failure, and salvage itself must never mask the original error -- a
    salvage crash is appended to ``errors`` and the block stays absent.
    """
    if recorder is None:
        return None
    try:
        recorder.finalize()
        recorder.write_track(case_dir / "truth_track.csv")
        return recorder.verdict()
    except Exception as salvage_error:
        errors.append(
            "truth salvage failed: "
            f"{type(salvage_error).__name__}: {salvage_error}"
        )
        return None


__all__ = [
    "TRUTH_SCORE_RATE_HZ",
    "TruthRecorder",
    "salvage_truth",
    "truth_position_sample",
]
