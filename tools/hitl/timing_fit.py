"""Step 12's fit: each source's receiver-local correction C from a calibration-only run (check_integration_timing.py).

M = the frame's mapped receiver time. Attitude: A = navpy's ATTITUDE receipts interpolated at the frame's
represented board time B; pair A - M. Gimbal: G = navpy's readback receipts interpolated at the rendered plant time
T_r over the plant times the joined 0x0D frames reported; pair G - M. No host's clock is subtracted from another's.
C is the median of at least MIN_PAIRS pairs over at least MIN_BLOCKS blocks; only the constants go into navpy.
"""
from __future__ import annotations

import bisect
from collections.abc import Sequence
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.capture_interpolation import interpolate_attitude_at
from navpy.core.perception.camera.integration_timing import IntegrationTimingLimits
from tools.hitl.timing_inputs import AttitudeReceipt
from tools.hitl.timing_joins import INVALID, JoinError
from tools.hitl.timing_run import CORRECTION_MATCH_S, Joined
from tools.hitl.timing_stats import MIN_BLOCKS, block_count, fit_median

FIT_SCHEMA = "navpy.hitl.integration_timing_fit.v1"
MIN_PAIRS = 30                       # plan: >= 30 calibration-only pairs per source
# My value, not set by the plan: the moving-block length is the map's slowest memory, step 11's 10 s rate window.
BOOTSTRAP_BLOCK_S = IntegrationTimingLimits().rate_window_s


def _attitude_limits(joined: Joined) -> tuple[float, float]:
    """The one (bracket, seam) pair navpy's attitude lookups ran with in this run."""
    limits = {(float(item.get("attitude_bracket_max_s")), float(item.get("seam_tolerance_s")))
              for item in joined.log.associations}
    if len(limits) != 1:
        raise JoinError(INVALID, f"navpy's attitude bracket/seam limits changed within the run: {sorted(limits)}")
    return limits.pop()


class AttitudeReceipts:
    """navpy's ATTITUDE receipts in receipt order, and the receipt navpy's own lookup would place at a board time.

    A pair counts only if navpy's ``interpolate_attitude_at`` at its interpolated receipt, with the run's bracket,
    seam and quarantine limits, accepts it and lands on the same board time: the fit never uses a bracket the live
    association would refuse. A sample without board time stays in the history and breaks adjacency, as it does live.
    """

    def __init__(self, receipts: Sequence[AttitudeReceipt], bracket_s: float, seam_s: float,
                 quarantine_s: float) -> None:
        for earlier, later in zip(receipts, receipts[1:]):
            if later.receipt_s <= earlier.receipt_s:
                raise JoinError(INVALID, "navpy ATTITUDE receipts do not rise in log order (a reorder)")
        self._samples = [SimpleNamespace(time_boot_s=sample.time_boot_s, capture_receipt_s=sample.receipt_s,
                                         attitude=Attitude(0.0, 0.0, 0.0), body_rates_rad_s=None)
                         for sample in receipts]
        self._receipts = [sample.receipt_s for sample in receipts]
        self._valid = [index for index, sample in enumerate(receipts) if sample.time_boot_s is not None]
        self._boots = [receipts[index].time_boot_s for index in self._valid]
        if any(later < earlier for earlier, later in zip(self._boots, self._boots[1:])):
            raise JoinError(INVALID, "navpy ATTITUDE time_boot runs backwards (a reset)")
        self._bracket_s, self._seam_s, self._quarantine_s = bracket_s, seam_s, quarantine_s

    def at(self, board_s: float) -> float | None:
        position = bisect.bisect_right(self._boots, board_s)
        if position == len(self._boots) and self._boots and self._boots[-1] == board_s:
            position -= 1
        if position == 0 or position >= len(self._boots):
            return None
        first, second = self._valid[position - 1], self._valid[position]
        if second != first + 1:
            return None                     # a sample without board time lies between them
        earlier, later = self._samples[first], self._samples[second]
        if later.time_boot_s <= earlier.time_boot_s:
            return None                     # equal board times cannot be inverted
        fraction = (board_s - earlier.time_boot_s) / (later.time_boot_s - earlier.time_boot_s)
        receipt = earlier.capture_receipt_s + fraction * (later.capture_receipt_s - earlier.capture_receipt_s)
        low = bisect.bisect_left(self._receipts, receipt - self._quarantine_s - self._bracket_s)
        found, _ = interpolate_attitude_at(self._samples[max(0, low - 1):second + 1], receipt, self._quarantine_s,
                                           bracket_max_width_s=self._bracket_s, seam_tolerance_s=self._seam_s)
        if found is None or abs(found.time_boot_s - board_s) > CORRECTION_MATCH_S:
            return None
        return receipt


def _receipt_at(source_t: Sequence[float], receipts: Sequence[float], at: float, width_max_s: float) -> float | None:
    """The receipt interpolated at source time ``at`` between two adjacent samples, or None (no bracket).

    ``source_t`` never falls (timing_joins checks it); a pair with equal source times cannot be inverted.
    """
    index = bisect.bisect_right(source_t, at)
    if index == len(source_t) and source_t and source_t[-1] == at:
        index -= 1
    if index == 0 or index >= len(source_t):
        return None
    first, second = source_t[index - 1], source_t[index]
    if not first <= at <= second or second <= first:
        return None
    width = receipts[index] - receipts[index - 1]
    if width > width_max_s:
        return None
    return receipts[index - 1] + (at - first) / (second - first) * width


@dataclass(frozen=True)
class Pairs:
    values: list[float]
    times: list[float]            # the source time of each pair, for the bootstrap's blocks
    unpaired: int


def calibration_pairs(joined: Joined) -> tuple[Pairs, Pairs]:
    limits = joined.config.limits
    attitude = AttitudeReceipts(joined.log.attitude, *_attitude_limits(joined), limits.window_s)
    gimbal_receipts = joined.gimbal_receipts
    plant = list(joined.gimbal.plant_t)
    a_values, a_times, g_values, g_times = [], [], [], []
    a_missing = g_missing = 0
    for item in joined.frames:
        sequence = int(item.get("frame_sequence"))
        mapped_s = float(item.get("mapped_s"))
        board = joined.board_s[sequence]
        receipt = attitude.at(board)
        if receipt is None:
            a_missing += 1
        else:
            a_values.append(receipt - mapped_s)
            a_times.append(board)
        rendered = joined.rendered_t[sequence]
        receipt = _receipt_at(plant, gimbal_receipts, rendered, limits.gimbal_bracket_max_s)
        if receipt is None:
            g_missing += 1
        else:
            g_values.append(receipt - mapped_s)
            g_times.append(rendered)
    return Pairs(a_values, a_times, a_missing), Pairs(g_values, g_times, g_missing)


def fit(joined: Joined) -> dict[str, Any]:
    attitude, gimbal = calibration_pairs(joined)
    result: dict[str, Any] = {"schema": FIT_SCHEMA, "run_ids": sorted(joined.log.run_ids),
                              "source_id": joined.config.source_id, "link_id": joined.config.link_id,
                              "method": {"statistic": "median", "interval": "block bootstrap 95 % over occupied bins",
                                         "block_s": BOOTSTRAP_BLOCK_S, "min_pairs": MIN_PAIRS,
                                         "min_blocks": MIN_BLOCKS}}
    failures = []
    for name, pairs in (("attitude", attitude), ("gimbal", gimbal)):
        section: dict[str, Any] = {"pairs": len(pairs.values), "unpaired": pairs.unpaired}
        if len(pairs.values) < MIN_PAIRS:
            failures.append(f"{name}: {len(pairs.values)} pairs, fewer than {MIN_PAIRS}")
        elif block_count(pairs.times, BOOTSTRAP_BLOCK_S) < MIN_BLOCKS:
            failures.append(f"{name}: pairs fill fewer than {MIN_BLOCKS} blocks of {BOOTSTRAP_BLOCK_S:g} s")
        else:
            section.update(fit_median(pairs.values, pairs.times, BOOTSTRAP_BLOCK_S).record())
        result[name] = section
    result["failures"] = failures
    result["passed"] = not failures
    return result


__all__ = ["AttitudeReceipts", "BOOTSTRAP_BLOCK_S", "FIT_SCHEMA", "MIN_PAIRS", "Pairs", "calibration_pairs", "fit"]
