"""What a run IS, and whether its number may be believed.

Split from the flying code because it answers a different question. `run` and
`run_navigation_episode` produce an outcome; this module decides which experiment produced it
and whether that outcome counts. Both halves have failed silently before:

  * a result that did not name its own configuration, so two arms of a
    comparison were indistinguishable in the artifact;
  * a run that finished cleanly at the wrong simulated clock rate, which is not
    a slower version of the right run -- every command interval and every rate
    the law differentiates moves with it, so the miss answers a question nobody
    asked.

Neither shows up as an error, which is why they are decided here rather than
left to whoever reads the numbers.
"""

from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

from scripts import scratch_navigation_arms as arms
from scripts.sitl_truth_pose import TruthPose

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent

# How far the MEASURED clock rate may sit from the requested one. Generous,
# because it is not measuring precision: it exists to catch the whole-multiple
# error, where the request never applied and the aircraft ran at whatever the
# simulated eeprom already held. A 20% band passes ordinary scheduler jitter
# and still fails 1x-requested-ran-at-10x by a factor of eight.
SPEEDUP_TOLERANCE = 0.20


def classification_errors(
    *,
    scoring_end: str,
    miss_m: float | None,
    certification: str | None,
    measured_speedup: float | None = None,
    requested_speedup: float | None = None,
    held_commands: int = 0,
) -> list[str]:
    """Every reason this run's number is not a score. Empty means it is one.

    ONLY a completed pass measures a closest approach. Every other outcome still
    leaves a `miss_m` in the record -- the smallest range reached before the run
    stopped -- and that number is not an approach error. It is where the aircraft
    happened to be when it ran out of simulated time or patience, and reading it
    as a score would rank a run that quit early ABOVE one that flew the
    scoring interval out. So anything but a pass is an error, and the exit code is
    enough for a caller to tell the difference.
    """
    errors: list[str] = []
    if certification:
        # The miss stays in the record, but it is not a measurement of the law:
        # a starved or stalled pose feed means the aircraft flew part of the
        # scoring interval on held commands.
        errors.append(f"pose feed not certified: {certification}")
    if scoring_end != "passed":
        errors.append(
            f"scoring window did not complete: {scoring_end} -- 'miss_m' is the "
            "closest range reached before stopping, not a closest approach"
        )
    if miss_m is None:
        errors.append("no closest approach was measured")
    # A command instant the harness could not supply a frame for. The aircraft
    # still flew -- it held the previous attitude -- so the scoring interval continued
    # on a command that no longer answered the geometry. That is the same defect
    # the pose certification exists to catch, one layer further in, and it is
    # counted here rather than tolerated because the alternative is a run that
    # looks complete while part of it was flown open-loop.
    if held_commands:
        errors.append(
            f"{held_commands} command instants had no frame from that pose -- "
            "the aircraft held the previous attitude through them, so the "
            "command sequence is not the one the law would have produced"
        )
    # The clock is checked against what was asked for, not merely reported. A
    # run at the wrong rate is not a slower version of the right one: every
    # command interval, every rate the law differentiates, and the plant's own
    # response all move together, so the miss it produces answers a question
    # nobody asked. SPEEDUP_TOLERANCE is fractional because the error that
    # matters is a whole multiple, not a few percent of scheduler jitter.
    if measured_speedup is not None and requested_speedup:
        drift = abs(measured_speedup - requested_speedup) / requested_speedup
        if drift > SPEEDUP_TOLERANCE:
            errors.append(
                f"ran at {measured_speedup:g}x, not the requested "
                f"{requested_speedup:g}x -- the simulated clock did not advance "
                "at the rate this run claims"
            )
    return errors


def initial_result(
    options: argparse.Namespace, law_source: Path
) -> dict:
    """What the run is, before it is anything else.

    `law_source` is passed IN rather than looked up: resolving it needs the
    imported law class, which belongs to the flying module. Reaching back
    for it here would make this module depend on its own caller.

    Separate from `run` because it answers a different question: not what
    happened, but WHICH EXPERIMENT this is. Every field here identifies the
    configuration rather than the outcome, and a result that cannot name its
    own configuration cannot be compared to another one.
    """
    return {
        "sysid": options.sysid,
        "connection": options.connection,
        # Overwritten with the RESOLVED coordinates once the scoring interval
        # starts. Null here means the run never got far enough to place one.
        "poi": None,
        # Recorded because the arms are only comparable against each other, and
        # a result that does not say which arm produced it cannot be compared
        # to anything.
        "estimate_source": options.estimate_source,
        "estimate_delay_poses": options.estimate_delay_poses,
        "estimate_jitter_deg": options.estimate_jitter_deg,
        # The SEED, not only the magnitude. Two runs at 2 deg RMS with
        # different seeds are different experiments, and a result that names
        # only the magnitude cannot say which one it was or reproduce it.
        "estimate_jitter_seed": options.estimate_jitter_seed,
        "estimate_lag_s": options.estimate_lag_s,
        # WHAT the lag applied to -- without it, the angles-only and the
        # full-consistency arms are indistinguishable in the artifact.
        "estimate_lag_scope": getattr(options, "estimate_lag_scope", "angles"),
        # WHICH law.py actually flew, resolved from the imported class rather
        # than from whatever the launch intended. The import block above
        # rebuilds sys.path from THIS FILE's location, removing and
        # re-inserting the worktree ahead of everything else, so an arm
        # selected by PYTHONPATH alone is discarded in silence and both arms
        # fly identical code. That failure has no error and no symptom: it
        # reports a true difference of zero, which reads as "the change did
        # nothing". Companions have already been caught running another
        # tree's code once, so this is recorded, not trusted.
        "law_source": str(law_source),
        # Content, because the path alone cannot tell a correctly-placed stale
        # copy from the intended one. Within an arm every aircraft must show
        # the SAME hash, and between arms it must DIFFER; either violation
        # means the comparison measured something other than the gain.
        "law_sha256": arms.file_digest(law_source),
        "errors": [],
    }


def entry_state(pose: "TruthPose", airspeed_mps: "float | None",
                path_angle_deg: "float | None" = None,
                ground_speed_mps: "float | None" = None,
                course_deg: "float | None" = None) -> dict:
    """THE WHOLE SCORING ENTRY STATE, not just the POI-placement fields.

    Five groups of runs were once compared as repeats of an identical start
    when only four pose fields had been recorded -- entry pitch, roll,
    airspeed, body rates and where the truth sample fell inside its ATTITUDE
    interval were all unrecorded, and every one of them is a plausible
    determinant of which regime the run enters.

    `path_angle_deg` is the GROUND-frame flight-path angle, recorded so a
    level-POI cell's claim of a level entry is a checkable fact of each
    artifact rather than an assumption about the level-off phase: pitch at
    trim does not by itself mean the aircraft has stopped climbing.

    `ground_speed_mps` and `course_deg` make a WIND cell self-verifying. A wind
    cell names a direction in the SIMULATOR's frame while the case it means --
    head, tail, or cross -- is relative to the aircraft's heading, and an
    earlier wind run recorded neither: its `cross` and `tail` labels cannot be
    checked from the artifact at all, and read against the heading those runs
    actually flew they look swapped. Ground speed against airspeed settles
    head from tail; course against yaw gives the crab angle, which is the
    quantity a cross cell exists to produce.

    All three are REPORT ONLY. Ground speed, course and altitude-derived
    vertical state are barred from the command path, and nothing in this dict
    reaches a frame.
    """
    return {
        "path_angle_deg": path_angle_deg,
        "ground_speed_mps": ground_speed_mps,
        "course_deg": course_deg,
        "lat_deg": pose.lat_deg, "lon_deg": pose.lon_deg,
        "alt_m": pose.alt_m, "yaw_deg": pose.yaw_deg,
        "truth_pitch_deg": pose.pitch_deg,
        "truth_roll_deg": pose.roll_deg,
        "est_pitch_deg": pose.est_pitch_deg,
        "est_roll_deg": pose.est_roll_deg,
        "est_roll_rate_rad_s": pose.est_roll_rate_rad_s,
        "est_pitch_rate_rad_s": pose.est_pitch_rate_rad_s,
        "est_yaw_rate_rad_s": pose.est_yaw_rate_rad_s,
        "airspeed_mps": airspeed_mps,
        "pose_skew_s": pose.skew_s,
    }


class RollSeries:
    """Magnitude, step size and sign reversals of a signed roll series.

    Three numbers, because no one of them settles what the series is doing:

      magnitude  how far from level. A steady bank and an oscillation of the
                 same size are IDENTICAL here, which is why it is not enough.
      step       how far it moved between consecutive samples. This is what
                 separates the two: a held bank steps by ~0, an oscillation of
                 amplitude A steps by ~2A.
      reversals  how often it changed side.

    Used for the COMMAND and for the aircraft's ACTUAL roll, so the two are
    summarised identically and can be read against each other -- the command
    dithering matters only if the airframe follows it.

    The sign threshold is not zero. A series crossing zero by a hair would
    otherwise register a reversal every sample, counting sensor noise as
    behaviour; a reversal has to pass through a band to count.
    """

    SIGN_THRESHOLD_DEG = 1.0

    def __init__(self) -> None:
        self._magnitudes: list[float] = []
        self._steps: list[float] = []
        self._previous: float | None = None
        self._sign = 0
        self.reversals = 0

    def observe(self, roll_deg: float) -> None:
        self._magnitudes.append(abs(roll_deg))
        if self._previous is not None:
            self._steps.append(abs(roll_deg - self._previous))
        self._previous = roll_deg
        sign = (0 if abs(roll_deg) < self.SIGN_THRESHOLD_DEG
                else int(math.copysign(1, roll_deg)))
        if sign and self._sign and sign != self._sign:
            self.reversals += 1
        if sign:
            self._sign = sign

    def summary(self) -> dict:
        return {
            "magnitude": _spread(self._magnitudes),
            "step": _spread(self._steps),
            "reversals": self.reversals,
        }


def _spread(values: list[float]) -> dict | None:
    """Median, p95 and max of a command series, or None if it never commanded.

    Reported together because the question they answer needs all three: a
    median near zero with a large p95 is an occasional correction, while a
    median and a p95 that sit close together is a command holding that
    magnitude for the whole run. Either shape can produce the same count of
    sign changes, which is why the count on its own settles nothing.
    """
    if not values:
        return None
    ordered = sorted(values)
    return {
        # statistics.median, NOT ordered[n // 2]. On an even-length series the
        # index form returns the upper of the two middle values, which is
        # exactly wrong for the shape this is here to detect: a command
        # dithering between 0 and 20 degrees has a median of 10, and the index
        # form reports 20 -- reading as a command that HELD 20 degrees.
        "median_deg": round(statistics.median(ordered), 3),
        # Nearest-rank: the smallest value at or above the 95th percentile.
        # Stated because the alternatives interpolate, and a diagnostic that
        # reports a magnitude never actually commanded invites the same
        # misreading the median just caused.
        "p95_deg": round(ordered[math.ceil(0.95 * len(ordered)) - 1], 3),
        "max_deg": round(ordered[-1], 3),
        "samples": len(ordered),
    }
