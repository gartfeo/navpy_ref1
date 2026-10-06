"""The vertical (pitch) command formula for the final-approach visual law.

One formula, `raw_pitch_current`: a proportional-navigation integrator that
walks the command anchor down by `N * rate * dt` every cycle. The clamped
output becomes the next cycle's anchor, so the constant-bearing course is the
recursion's fixed point.

WHY THIS IS THE ONLY ONE LEFT
=============================

An `AAS_PITCH_LAW` selector once offered three alternatives. All three were
measured, all three failed, and all three were removed on 2026-08-20. The
reasoning is kept here because the same ideas will occur to the next reader.

`telescope` -- a windowed difference, `p(t) = p(t-W) - N*[L(t) - L(t-W)]`,
proposed as bounded memory. It is not. Substituting `z = p + N*L` gives
`z(t) = z(t-W)`, so `p(t) = z(seed) - N*L(t)`: it chains back to the seed and
carries ALL earlier history. Retaining `2*W` of samples bounds STORAGE, not the
influence horizon. It was an navigation task-long accumulator with a window-quantised
update, no more compliant than the integrator it was meant to replace.

`memoryless` -- pure instantaneous P+D, `-elevation - GAIN * rate`. Trivially
free of accumulation, and it lost every tailwind block of a 64-run SITL wind
matrix by 7.6 to 14.9 m. Root cause: it equates a GROUND-relative LOS elevation
with an AIR-relative pitch attitude. The achieved flight path is shallower by
angle of attack plus the wind's effect on along-track ground speed, so it
under-dives every cycle until the pitch floor saturates. Measured
`mean(elevation - true flight-path angle)` over the final-approach window: +0.11 to
+0.57 deg for this formula, +3.88 calm and +14.72 tailwind for `memoryless`.

That is the deeper point, and it is why the accumulator stays: rejecting a
constant unknown disturbance REQUIRES integral action. A pure P+D final-approach law
cannot do it at any gain. This integrator is not inertia -- it is disturbance
rejection. Confirmed from the other side on the noise-free point-mass bench,
where angle of attack 0 vs 5 deg produces identical misses to three decimals
across every wind: the integrator absorbs the standing bias structurally.

`current_los_seed` -- not a candidate law. It existed only to decompose the
`memoryless` result, separating the steady-state formula from the handoff, and
died with the experiment it was measuring.

THE COST, WHICH IS REAL
=======================

A bias living in the RATE estimate rather than in a true elevation change is
exactly what a running integral cannot forget, and the drift scales with active navigation
time. `test_current_accumulates_unbounded_drift_from_a_pure_rate_bias` pins that
property deliberately. It is the price of the disturbance rejection above, and
it is why the rate channel is filtered (`VERTICAL_RATE_FILTER_TAU_S` in law.py)
rather than fed raw.

Full campaign record: `docs/decisions/`, and the wind matrix in
`navpy-worktrees/_shared/knowledge/pitch-handoff-ab-2026-08-20/`.
"""

from __future__ import annotations

import math

__all__ = ["raw_pitch_current"]


def raw_pitch_current(
    anchor_cmd_pitch_deg: float,
    navigation_constant: float,
    rate_rad_s: float,
    dt_s: float,
) -> float:
    """Walk the anchor command down by the PN increment for this cycle.

    rate_rad_s is the filtered rate averaged over this interval, so rate*dt
    is its integral, including when consecutive frame intervals differ.
    The caller clamps the result and feeds it back as the next anchor, which is
    what makes the constant-bearing course the fixed point of the recursion.
    """
    return anchor_cmd_pitch_deg - math.degrees(
        navigation_constant * rate_rad_s * dt_s
    )
