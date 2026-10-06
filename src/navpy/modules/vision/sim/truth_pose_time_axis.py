"""Clock-axis policy for the simulator truth-pose history.

Two clocks can position truth samples and render queries. ``receipt``: host
wall stamps taken when SIM_STATE arrived — the original axis, whose 2-3 ms
per-frame error against the true state time was confirmed by intervention
A/B (2026-08-26 findings note) to be the bearing-noise channel. ``source``:
the SIM_STATE ``time_us`` navlink extension — the FDM state-sample time on
the autopilot clock, the same clock ATTITUDE ``time_boot_ms`` lives on
(SITL stops its clock TO the FDM stamp), so the render query pairs against
it exactly and the host clock drops out. ``source`` is the default: the
2026-08-27 confirmation A/B on merged dev replicated the pre-merge result
(bearing residual 1.77 vs 1.42 mdeg, p=0.001, ~1.15 mdeg removed). Runs
without ``time_us`` stamps fall back sticky to ``receipt``.
"""

from __future__ import annotations

import os
from collections.abc import Callable, Mapping

from navpy.modules.vision.sim.pose_associator import AssociatedPose


TRUTH_POSE_TIME_AXIS_ENV = "AAS_TRUTH_POSE_TIME_AXIS"
TIME_AXIS_RECEIPT = "receipt"
TIME_AXIS_SOURCE = "source"
TIME_AXES = (TIME_AXIS_RECEIPT, TIME_AXIS_SOURCE)

# One integer autopilot clock feeds both source-axis inputs, but through two
# different float conversions: the ATTITUDE query is ``time_boot_ms * 1e-3``
# and the SIM_STATE stamp is ``time_us * 1e-6``. The two products round
# differently for ~40% of tick values (7 ms -> 0.007 vs 0.006999999999999999),
# so a query taken on the SAME tick as the newest stamp can land one ulp past
# it and read as extrapolation. Comparing in integer microseconds — the finer
# clock's own quantum — makes same-tick equality exact again. Receipt stamps
# are host wall-clock floats with no shared quantum and stay raw.
SOURCE_CLOCK_QUANTUM_S = 1e-6


def resolve_truth_pose_time_axis(env: Mapping[str, str] | None = None) -> str:
    """Read the truth-pose interpolation axis from the environment.

    An unknown value raises instead of defaulting: a mistyped A/B arm must
    fail loud, never silently fly the wrong axis.
    """
    raw = (os.environ if env is None else env).get(
        TRUTH_POSE_TIME_AXIS_ENV, TIME_AXIS_SOURCE
    )
    value = str(raw).strip().lower()
    if value not in TIME_AXES:
        raise ValueError(
            f"{TRUTH_POSE_TIME_AXIS_ENV} must be one of {TIME_AXES}, "
            f"got {raw!r}"
        )
    return value


def pair_span_s(
    axis: str,
    scheduler_period_s: float,
    receipt_skew_s: Callable[[], float],
) -> float:
    """The widest sample gap a bracketing pair may span on ``axis``.

    Source stamps advance in SIM seconds (the scheduler period as written);
    receipt stamps advance in WALL seconds (scaled by speedup).
    """
    if axis == TIME_AXIS_SOURCE:
        return scheduler_period_s
    return receipt_skew_s()


def render_clock_s(axis: str, pending: AssociatedPose) -> float:
    """The clock to interpolate the truth pose at, on ``axis``.

    On the source axis the query is the ATTITUDE boot stamp — the same
    autopilot clock the SIM_STATE ``time_us`` stamps live on. Known limit:
    ``time_boot_ms`` is uint32 and wraps after ~49.7 simulated days while
    the uint64 ``time_us`` keeps counting, after which every source-axis
    query is refused (frames stop: fail-stop, not fail-wrong). Sim runs
    last minutes and SITL restarts per run, so the wrap is not handled.
    """
    if axis == TIME_AXIS_SOURCE:
        return pending.attitude_timestamp_s
    return pending.attitude_receipt_s


__all__ = [
    "SOURCE_CLOCK_QUANTUM_S",
    "TIME_AXES",
    "TIME_AXIS_RECEIPT",
    "TIME_AXIS_SOURCE",
    "TRUTH_POSE_TIME_AXIS_ENV",
    "pair_span_s",
    "render_clock_s",
    "resolve_truth_pose_time_axis",
]
