"""What the law is told about the aircraft itself.

Truth builds the visual ray, because a camera is never wrong about where a thing
appears. Everything the law reads about the AIRCRAFT comes from here instead, and
it is the only place sim-truth attitude may enter the command path -- under the
branch that names the diagnostic control arm, and nowhere else. A guard test
parses this module and fails if a truth read appears anywhere outside it.

Three axes meet here rather than being scattered through the flight loop, so a
run that changes one has changed only one.
"""

from __future__ import annotations

import argparse
import random
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
for _root in (WORKTREE, WORKTREE / "src"):
    _path = str(_root)
    while _path in sys.path:
        sys.path.remove(_path)
    sys.path.insert(0, _path)

from scripts.scratch_navigation_attitude_lag import (  # noqa: E402
    AttitudeLagSampler,
    AttitudeSample,
)
from scripts.sitl_truth_pose import TruthPose  # noqa: E402


@dataclass(frozen=True)
class EstimateState:
    """Everything the law reads about the aircraft, with matched clocks.

    `rate_pitch_deg`/`rate_roll_deg` are the angles AT THE RATES' INSTANT,
    carried separately because the body-to-Euler yaw-rate conversion needs
    the attitude of the same moment as the rates it converts. Found in
    review: the `full`-scope arm returned lagged rates while the conversion
    still read the current pose angles -- the arm built to prove
    'every input on one clock' was itself mixing two.
    """

    pitch_deg: float
    roll_deg: float
    rates: "tuple[float, float, float]"
    rate_pitch_deg: float
    rate_roll_deg: float


def de_rotating_attitude(
    options: argparse.Namespace,
    pose: TruthPose,
    estimates: "deque[tuple[float, float]]",
    lag_sampler: AttitudeLagSampler,
    jitter: random.Random,
) -> "EstimateState | None":
    """The pitch/roll handed to the law, or None when it cannot be established.

    THE ONLY PLACE SIM-TRUTH ATTITUDE MAY REACH THE COMMAND PATH, and only under
    the branch that names the `truth` control arm. That arm hands the law
    information the flying article does not have, which is legitimate for a
    diagnostic bound and invalid for anything else, so a guard test parses this
    function and fails if a truth read appears anywhere else in it.

    Three axes meet here and are applied in a fixed order so a run that changes
    one has changed only one:

      FIDELITY  which source the pitch/roll comes from.
      TIMING    how far back it is sampled -- `estimate_delay_poses` quantised to
                whole poses, or `estimate_lag_s` interpolated between samples.
                Mutually exclusive; `check_lag_options` refuses both.
      NOISE     applied LAST, after the timing selection, so the two compose
                instead of the noise being smoothed by the delay.

    SCOPE decides whether the body rates share the angles' timestamp. The
    default lags angles only, which leaves the law's rate filter mixing two
    time bases -- delayed angles differentiated against a current yaw rate.
    `full` lags the rates by the same interval, so every input the law reads
    shares one instant. The two arms differ ONLY in that consistency, which is
    what separates a mixed-time artifact from genuine loop-phase compensation.
    """
    if options.estimate_source == "truth":
        estimates.append((pose.pitch_deg, pose.roll_deg))
    else:
        estimates.append((pose.est_pitch_deg, pose.est_roll_deg))
    # A HISTORY SHORTER THAN THE DELAY IS NOT A SHALLOWER DELAY, it is no
    # delay yet. `estimates[0]` is the oldest RETAINED pose, so a partly-filled
    # deque hands back a delay that ramps 0, 1, ... N over the first N poses --
    # and the scoring interval clock, the target placement and the scoring all start
    # inside that ramp. The continuous-lag arm already refuses this case (its
    # sampler returns None when the interval is unbracketed); returning None
    # here puts the pose-delay arm on the SAME warm-up path, so both treatments
    # begin with their full treatment applied. Depth 1 -- the undelayed arm --
    # is full on its first pose, so nothing about it changes.
    if len(estimates) < (estimates.maxlen or 1):
        return None
    est_pitch_deg, est_roll_deg = estimates[0]
    # Fed from the SAME source selection, so no second truth read exists. The
    # rates ride along so a `full`-scope lag can keep them on one clock.
    lag_sampler.observe(AttitudeSample(
        pose.t_s, *estimates[-1],
        pose.est_roll_rate_rad_s, pose.est_pitch_rate_rad_s,
        pose.est_yaw_rate_rad_s))
    rates = (pose.est_roll_rate_rad_s, pose.est_pitch_rate_rad_s,
             pose.est_yaw_rate_rad_s)
    # The conversion's angles follow the RATES' clock, not the de-rotation's:
    # current rates convert with current angles whatever the lag, and lagged
    # rates (`full` scope) convert with the lagged angles.
    rate_pitch_deg, rate_roll_deg = pose.est_pitch_deg, pose.est_roll_deg
    # `atomic` scope delays the WHOLE pose upstream; lagging the estimates a
    # second time here would double the interval and test nothing nameable.
    if options.estimate_lag_s > 0.0 and options.estimate_lag_scope != "atomic":
        lagged = lag_sampler.at(pose.t_s)
        if lagged is None:
            return None
        est_pitch_deg, est_roll_deg = lagged.pitch_deg, lagged.roll_deg
        if options.estimate_lag_scope == "full":
            rates = (lagged.roll_rate_rad_s, lagged.pitch_rate_rad_s,
                     lagged.yaw_rate_rad_s)
            rate_pitch_deg, rate_roll_deg = lagged.pitch_deg, lagged.roll_deg
    if options.estimate_jitter_deg:
        est_pitch_deg += jitter.gauss(0.0, options.estimate_jitter_deg)
        est_roll_deg += jitter.gauss(0.0, options.estimate_jitter_deg)
    return EstimateState(
        est_pitch_deg, est_roll_deg, rates, rate_pitch_deg, rate_roll_deg)


class AtomicDelayBuffer:
    """The pose from `lag` seconds ago, whole, or nothing.

    Exists to separate two interventions the angle-lag arms cannot tell apart:
    delaying an ATOMIC observation (ray, attitude and timestamp from one
    earlier instant -- a plain transport delay, which control theory says
    should HURT) versus lagging the attitude UNDER a current ray (which leaves
    the aircraft's own motion over the lag window inside the perceived error).
    The lag arms all do the second. If this buffer's arm reproduces their win,
    the win is true delay compensation; if it does not, the win is the
    attitude-motion residual itself, and the law fix is an explicit
    own-motion term rather than any filter or delay.
    """

    def __init__(self, lag_s: float) -> None:
        self._lag_s = float(lag_s)
        self._poses: deque[TruthPose] = deque(maxlen=256)

    def push(self, pose: TruthPose) -> "TruthPose | None":
        """Absorb the current pose; return the newest one at least `lag` old.

        Newest-not-younger rather than interpolated: the arm models a camera
        whose whole observation arrives late, and real late frames arrive
        discrete. Returns None during warm-up, which the caller treats exactly
        like an unbracketable attitude lag.
        """
        self._poses.append(pose)
        cutoff = pose.t_s - self._lag_s
        candidate = None
        for old in self._poses:
            if old.t_s <= cutoff:
                candidate = old
            else:
                break
        return candidate
