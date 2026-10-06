#!/usr/bin/env python
"""Fly one SITL aircraft at a fixed POI using the final-approach law and no vision.

WHAT THIS ISOLATES
------------------
A real scoring interval fails for many reasons at once: the detector, the camera
model, the gimbal, the link, the plant, the law. This harness removes every one
of those except the plant and the law. The aircraft is a real ArduPilot SITL, so
its aerodynamics, control loops and limits are genuine, but the line of sight is
synthesised perfectly from simulator truth. If the aircraft misses here, the
navigation law missed.

Nothing else is in the loop: no GCS backend, no companion, no detector, no swarm
coordination, no router. One process, one aircraft, one MAVLink link.

WHY THE HOST CLOCK IS KEPT OUT
------------------------------
Every decision is timed on the autopilot's own clock, which advances with the
simulated aircraft rather than with this machine. A loaded host then produces a
slower run, not a different one. Two places make this real:

  * Commands are issued when the SIMULATED time reaches the next command
    instant, not on a wall-clock cadence. scratch_sitl_uav.py's phase loop uses
    time.monotonic() scaled by the configured speedup, which is fine when a
    single fixed attitude is held for the whole phase -- the exact moment each
    repeat goes out changes nothing. It is not fine here: a navigation command
    depends on when it was computed, so a host stall would silently change the
    command sequence and therefore the miss.
  * The run's own length is measured in simulated seconds.

A wall-clock guard still exists, but only to abandon a hung run. It can end a
run; it can never change one, because it is never consulted by anything that
produces a number.

WHY TRUTH DOES NOT DECIDE WHEN TO STOP
--------------------------------------
Truth builds the line of sight and scores the miss, and touches nothing else --
including when the run ends. That last part is easy to get wrong. A termination
check reads as passive, but it decides whether the loop runs again and so
whether the law is handed any further frame; a run stopped on truth range or
truth altitude has had its command sequence shaped by truth, wherever in the
loop the check sits. So the run ends on the forward component of the body ray,
the sensor output the law itself reads, or on the simulated clock. Truth
altitude is still recorded at ground contact, purely as a report field.

WHAT IT CANNOT TELL YOU
-----------------------
A pass here is necessary, not sufficient. There is no field of view, so the law
can never lose the POI; no detection dropout or latency; no gimbal dynamics;
no false or swapped tracks; and the line of sight is exact. A law that passes
here can still fail the moment a real sensor is put in front of it.
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
import random
import statistics
import sys
import time
from collections import deque
from typing import Callable, Protocol
from pathlib import Path

import numpy as np

if not __package__:
    repository_root = Path(__file__).resolve().parents[1]
    for import_root in (repository_root, repository_root / "src"):
        import_path = str(import_root)
        while import_path in sys.path:
            sys.path.remove(import_path)
        sys.path.insert(0, import_path)

from navpy.modules.navigation.nav.vision_nav.law import (  # noqa: E402
    FixedFinalApproachLawConfigProvider,
    FinalApproachLawConfig,
    VisionNavLaw,
)
from navpy.modules.vision.models.pixel_observation import (  # noqa: E402
    aircraft_yaw_rate_rad_s,
)
from scripts import scratch_navigation_arms as arms  # noqa: E402
from scripts import scratch_navigation_attitude_lag as lag  # noqa: E402
from scripts.scratch_navigation_attitude_lag import (  # noqa: E402
    AttitudeLagSampler,
)
from scripts.scratch_navigation_estimate import (  # noqa: E402
    AtomicDelayBuffer,
    de_rotating_attitude,
)
# Result identity, result validity and how a command series is summarised are
# one concern, owned there. `RollSeries`/`_spread` are re-exported because the
# tests and the guard both name them on this module.
from scripts.scratch_navigation_result import (  # noqa: E402,F401
    SPEEDUP_TOLERANCE,
    RollSeries,
    _spread,
    classification_errors,
    entry_state,
    initial_result,
)
from scripts.scratch_navigation_final_approach import (  # noqa: E402,F401
    RollRecord,
    FinalApproachGeometry,
)
from scripts.scratch_navigation_oracle import (  # noqa: E402
    MODES as ORACLE_MODES,
    substitute_speed,
)
from scripts.navigation_truth_sensor import (  # noqa: E402
    ClosestApproach,
    build_frame,
    coordinates_from_offset_ned,
    poi_offset_ned_m,
)
from scripts.scratch_sitl_uav import (  # noqa: E402
    ATTITUDE_HOLD_MASK,
    CLIMB_PITCH_DEG,
    CLIMB_THROTTLE,
    CLIMB_LIMIT_S,
    WALL_GUARD_FACTOR,
    WALL_GUARD_FLOOR_S,
    Chatter,
    Flight,
    SPEEDUP_PARAM,
    _arm,
    _connect,
    _euler_to_quaternion,
    _mode,
    _read_param,
    _set_param,
    _wait_ready,
)
from scripts.eval_nav_origin import NAV_SOLUTION_TIMEOUT_S  # noqa: E402
from scripts.sitl_truth_pose import (  # noqa: E402
    TRUTH_POSE_RATE_HZ,
    TruthPose,
    TruthPoseStream,
    achievable_pose_rate_hz,
    pose_frame_association_max_skew_s,
    pose_gap_limit_s,
    request_truth_pose_streams,
)

# How often the law is given a fresh frame, in SIMULATED seconds. 20 Hz is well
# under the 50 Hz pose feed, so every command is computed from a pose that
# actually arrived rather than from a held one.
COMMAND_INTERVAL_S = 0.05


# pymavlink has no type stubs; describe the shape used, as `sitl_truth_pose.py`
# and `pose_streams.py:43-56` both do.
class MavlinkLink(Protocol):
    @property
    def mav(self) -> object: ...

    def recv_match(self, **criteria: object) -> object: ...

    def close(self) -> None: ...

# WHERE the pitch/roll that de-rotates the camera ray comes from. This is the
# FIDELITY axis; `--estimate-delay-poses` is the TIMING axis, and they are
# deliberately independent.
#
#   attitude  what the aircraft flies on: pitch/roll from ATTITUDE, a DIFFERENT
#             message from the SIM_STATE that builds the ray. The offset between
#             the two is bounded by the ATTITUDE interval and NOT measurable --
#             `TruthPoseStream.absorb` says so itself -- so the size of the
#             misalignment on this arm is unknown.
#   truth     pitch/roll from the SAME SIM_STATE message that built the ray, so
#             the ray and its de-rotation are one instant BY CONSTRUCTION and
#             the pairing misalignment is exactly zero.
#
# THE TWO AXES MUST NOT BE READ TOGETHER. An earlier version made `delayed` a
# third source, so the only zero-skew arm was also the only truth-fidelity arm
# and the two moved as one. That arm then produced MORE roll chatter than the
# baseline (10.6/s against 7.4/s, `.sitl-runs/navigation-20260815-145645`), and
# because timing and fidelity had changed together the result could not say
# which had done it -- or whether either had. Delay now applies to WHICHEVER
# source is selected, so a row of the resulting 2x2 varies timing alone:
#
#              delay 0                    delay N
#   attitude   what the aircraft flies    timing, fidelity held at estimate
#   truth      pairing skew removed       timing, fidelity held at truth
#
# `truth` is a DIAGNOSTIC CONTROL ONLY. It hands the command path sim-truth
# attitude, which the flying article does not have; it exists to bound the
# question and must never be a flying configuration.
ESTIMATE_SOURCES = ("attitude", "truth")

# The pass is detected from the LINE OF SIGHT, not from range.
#
# The forward component of the body ray says where the POI sits relative to
# the nose: near +1 is dead ahead, 0 is abeam, negative is behind. A POI that
# was clearly ahead and is now clearly behind has been passed, and the closest
# approach is in the past. This is the sensor's own output -- the identical
# quantity the law reads -- so terminating on it uses nothing the real aircraft
# lacks.
#
# An earlier version tested direct range and altitude instead. Wrong twice
# over: it needed truth range, which the command path may not use, and by
# deciding whether the loop continued it governed whether LATER commands
# existed. Reordering only moved that by one iteration; removing the illegal
# input closes it.
#
# PASSED_AHEAD_MIN is deliberately loose -- 0.5 is anywhere within 60 degrees of
# the nose -- because it only has to establish that the POI really was in
# front before it went behind, not that the run was well aimed.
#
# PASSED_BEHIND_MAX is NOT zero. Zero is abeam, roughly where the closest
# approach happens, so stopping there would leave the minimum at the last
# sample and _segment_minimum would clamp to that endpoint instead of the true
# miss. Requiring the POI clearly behind keeps the closest approach
# BRACKETED. At 35 m/s and 50 Hz that costs a few tens of milliseconds.
PASSED_AHEAD_MIN = 0.5
PASSED_BEHIND_MAX = -0.2



def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", required=True)
    parser.add_argument("--sysid", type=int, default=1)
    # Two ways to say where the POI is, and exactly one must be used.
    #
    # Absolute coordinates are the reproducible form: the same three numbers fly
    # the same scoring interval every time. But they are hard to CHOOSE, because they
    # have to be picked before the run and the aircraft's position at the end of
    # its climb is not known in advance -- guess short and the POI is already
    # behind the aircraft when navigation starts, which is the one geometry the
    # law cannot recover from.
    #
    # So the placement form exists: range, off-boresight angle and height below,
    # resolved ONCE against the first pose of the scoring interval and then frozen.
    # Every run writes the absolute coordinates it resolved into its result, so
    # any run can be repeated exactly by feeding those back in as --poi-lat/
    # lon/alt.
    parser.add_argument("--poi-lat", type=float, default=None)
    parser.add_argument("--poi-lon", type=float, default=None)
    parser.add_argument(
        "--poi-alt", type=float, default=None,
        help="POI altitude in metres AMSL, the same datum SIM_STATE reports",
    )
    parser.add_argument(
        "--poi-range-m", type=float, default=None,
        help="place the POI this far from the aircraft at scoring start",
    )
    parser.add_argument(
        "--poi-off-boresight-deg", type=float, default=0.0,
        help="placement angle from the nose, positive right. 0 is dead ahead",
    )
    parser.add_argument(
        "--poi-below-m", type=float, default=None,
        help="place the POI this far below the aircraft at scoring start",
    )
    parser.add_argument(
        "--settle-s", type=float, default=20.0,
        help="aircraft-seconds of TAKEOFF climb-out before GUIDED is entered",
    )
    parser.add_argument("--climb-to-m", type=float, default=400.0)
    parser.add_argument(
        "--level-settle-s", type=float, default=0.0,
        help="aircraft-seconds of commanded-level flight between the climb and "
             "the scoring window. The climb ends at CLIMB_PITCH_DEG the instant the "
             "altitude test passes, so an scoring starting there inherits a "
             "+15 deg transient: a level-POI cell meant to isolate the roll "
             "channel is not level until this has run. 0 keeps the entry every "
             "dive artifact was flown with",
    )
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--pitch-min-deg", type=float, default=-70.0)
    parser.add_argument("--pitch-max-deg", type=float, default=3.0)
    parser.add_argument("--roll-limit-deg", type=float, default=35.0)
    parser.add_argument(
        "--scoring-duration-s", type=float, default=180.0,
        help="maximum SIMULATED seconds of navigation before the run is abandoned",
     dest='scoring_duration_s')
    parser.add_argument(
        "--floor-m", type=float, default=15.0,
        help="REPORTING ONLY: altitude below which ground contact is recorded. "
             "It cannot stop the run -- altitude is barred from the command "
             "path, and a check that ends the run also decides whether later "
             "commands exist",
    )
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument(
        "--expect-law-source", type=Path, default=None,
        help="ARM PROOF: source root the navigation law MUST be imported from. "
             "Refuses at startup when the imported law resolves elsewhere. An "
             "A/B that selects its arm by source tree cannot verify itself: "
             "this file rebuilds sys.path from its OWN location, so a child "
             "launched from the original tree ignores the alternate one, both "
             "arms fly identical code, and the honest difference of zero is "
             "indistinguishable from a change that does nothing. Recording "
             "the source after the fact does not prevent that -- only "
             "refusing to fly does",
    )
    parser.add_argument(
        "--estimate-source", choices=ESTIMATE_SOURCES, default="attitude",
        help="FIDELITY axis: where the de-rotating pitch/roll comes from. "
             "Independent of --estimate-delay-poses; see ESTIMATE_SOURCES",
    )
    parser.add_argument(
        "--estimate-jitter-deg", type=float, default=0.0,
        help="RMS white noise added to the de-rotating pitch/roll. Splits the "
             "two explanations for why DELAY helps: a phase shift and a "
             "decorrelation of the same magnitude look identical in the delay "
             "sweep, but noise decorrelates WITHOUT shifting phase, so if "
             "jitter helps as much as delay the mechanism is decorrelation",
    )
    parser.add_argument(
        "--estimate-lag-s", type=float, default=0.0,
        help="TIMING axis, CONTINUOUS: sample the de-rotating pitch/roll this "
             "many SIMULATED seconds before the ray, interpolated between the "
             "two bracketing samples. Supersedes --estimate-delay-poses, which "
             "can only express multiples of the ~25 ms attitude interval -- a "
             "quantisation as large as the effect under measurement. Applies to "
             "whichever --estimate-source is selected, which is what separates "
             "a pairing fix from a loop-phase effect: a pairing correction "
             "cannot help a source whose ray and attitude are one message",
    )
    parser.add_argument(
        "--estimate-lag-scope", choices=("angles", "full", "atomic"),
        default="angles",
        help="WHAT the lag applies to. `angles` delays only the de-rotating "
             "pitch/roll, leaving the yaw rate current -- the law's rate "
             "filter then mixes two time bases. `full` delays the body rates "
             "by the same interval, so every input shares one instant. The "
             "difference separates a mixed-time artifact from genuine "
             "loop-phase compensation. `atomic` delays the WHOLE observation "
             "-- ray, attitude and timestamp from one earlier pose -- a plain "
             "transport delay; if it does not reproduce the angle-lag win, "
             "the win is the attitude-motion residual, not delay",
    )
    parser.add_argument(
        "--estimate-jitter-seed", type=int, default=0,
        help="so a jitter run is reproducible; the noise must be a controlled "
             "input, not a fresh accident each run",
    )
    parser.add_argument(
        "--estimate-delay-poses", type=int, default=0,
        help="TIMING axis: whole poses to hold the de-rotating attitude back, "
             "ON TOP of whatever skew the pairing already has. Applies to "
             "EITHER source, which is what keeps timing separable from fidelity",
    )
    parser.add_argument(
        "--pose-rate-hz", type=float, default=None,
        help="override the rate. Default: read SCHED_LOOP_RATE from the "
             "aircraft and use the 0.8x ArduPilot will actually grant",
    )
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    parser.add_argument("--ready-timeout", type=float, default=180.0)
    parser.add_argument("--arm-attempts", type=int, default=20)
    parser.add_argument("--arm-timeout", type=float, default=6.0)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--oracle-gain", choices=ORACLE_MODES, default="none",
        help="TRUTH-FED closing speed. DIAGNOSTIC ONLY -- violates the "
             "pure-vision constraint on purpose; see "
             "scripts/scratch_navigation_oracle.py")
    parser.add_argument("--wind-speed", type=float, default=None)
    parser.add_argument("--wind-dir", type=float, default=0.0)
    return parser


# How long the scoring interval waits for its FIRST complete pose before giving up.
# Simulated seconds are not available yet -- that is the point, no pose has
# arrived to carry a clock -- so this one is wall time, and like the hung-run
# guard it can only end a run, never change one.
#
# It exists because the alternative is what happened: with no pose the loop
# spins silently until the wall guard at 900 s, the parent kills it first, and
# the run produces no result and no reason. "The truth stream never arrived" is
# a five-second answer, not a fifteen-minute one.
FIRST_POSE_TIMEOUT_S = 45.0

# Message census window after the streams are requested. Cheap on a healthy run
# and decisive on a broken one: it says which messages the link actually
# carries, which is the difference between "the aircraft is not talking" and
# "the aircraft is talking but not sending SIM_STATE".
CENSUS_S = 5.0

def _default(function: Callable[..., object], name: str) -> float:
    """A function's own default for one keyword argument.

    Read rather than retyped. The budget below has to agree with what these
    calls actually wait for, and a copied number is a number that drifts.
    """
    return float(inspect.signature(function).parameters[name].default)


PARAM_SET_TIMEOUT_S = _default(_set_param, "timeout_s")
MODE_TIMEOUT_S = _default(_mode, "timeout_s")
STREAM_REQUEST_TIMEOUT_S = _default(request_truth_pose_streams, "timeout_s")
STREAM_REQUESTS = 2  # SIM_STATE and ATTITUDE, one wait each.


def _phase_guard_s(aircraft_s: float, speedup: float) -> float:
    """Flight.phase's own wall guard for a phase asked for in aircraft-seconds.

    The phase is requested in AIRCRAFT-seconds; the wall time it may occupy is
    this. Budgeting the aircraft-seconds instead is how a 240 s climb came to be
    sized at 240 s when it is allowed 1200 (scratch_sitl_uav.py:437-438).
    """
    return max(WALL_GUARD_FLOOR_S, WALL_GUARD_FACTOR * aircraft_s / speedup)


def worst_case_wall_s(options: argparse.Namespace) -> float:
    """Longest this run can LEGITIMATELY take before it writes its result.

    Every phase this process can sit inside, summed from that phase's own
    timeout. It exists so a parent can size its kill timer without guessing:
    two hand-written estimates in a row were short -- the first killed a child
    before its own hung-run guard could fire, the second budgeted the climb in
    aircraft-seconds and then omitted the parameter, stream, mode and arming
    waits entirely. Both produced a run with no result and no reason.

    FIRST_POSE_TIMEOUT_S is deliberately absent: it lives INSIDE the scoring interval
    and is bounded by the same guard, so it can only make the run end sooner.
    """
    speedup = max(options.speedup, 0.1)
    parameters = 1 + (STREAM_REQUESTS if options.wind_speed is not None else 0)
    return (
        options.connect_timeout
        + options.ready_timeout
        + parameters * PARAM_SET_TIMEOUT_S
        + STREAM_REQUESTS * STREAM_REQUEST_TIMEOUT_S
        + CENSUS_S
        + MODE_TIMEOUT_S
        # `_arm` holds the ground for the EKF's NED origin before it commands
        # anything, so the arm is now two waits, not one. Wall-clock and NOT
        # divided by speedup: it waits on the filter publishing a solution,
        # not on aircraft time.
        + NAV_SOLUTION_TIMEOUT_S
        + options.arm_attempts * options.arm_timeout
        # TAKEOFF mode and ARMING_CHECK, added when the run turned out to need a
        # launch before GUIDED would do anything at all.
        + MODE_TIMEOUT_S
        + PARAM_SET_TIMEOUT_S
        # Every remaining guard is the one the code below actually applies,
        # written the same way so the two cannot disagree.
        + _phase_guard_s(options.settle_s, speedup)
        + _phase_guard_s(CLIMB_LIMIT_S, speedup)
        + _phase_guard_s(options.level_settle_s, speedup)
        + max(600.0, 5.0 * options.scoring_duration_s / speedup)
    )


ABSOLUTE = ("poi_lat", "poi_lon", "poi_alt")
PLACEMENT = ("poi_range_m", "poi_below_m")


def say(message: str) -> None:
    """Progress, on stderr, unbuffered.

    A run takes minutes and used to print nothing until it finished, so a stall
    and a slow climb looked identical from outside -- and when the parent killed
    it, both logs were empty and the run had to be repeated to learn anything.
    """
    print(message, file=sys.stderr, flush=True)


def census(link: MavlinkLink, sysid: int, seconds: float) -> dict[str, int]:
    """Count message types on this link for a few WALL seconds."""
    counts: dict[str, int] = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        message = link.recv_match(blocking=True, timeout=0.5)
        if message is None or message.get_srcSystem() != sysid:
            continue
        kind = message.get_type()
        counts[kind] = counts.get(kind, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: -item[1]))


def law_source_path() -> Path:
    """Where the navigation law this process will fly actually came from.

    Resolved from the imported CLASS, so it reports the tree Python chose, not
    the tree the launcher believed it selected.
    """
    return Path(inspect.getfile(VisionNavLaw)).resolve()


def check_law_source(options: argparse.Namespace) -> None:
    """Refuse before flying when the law came from the wrong tree."""
    try:
        arms.check_source_root(law_source_path(), options.expect_law_source)
    except ValueError as mismatch:
        raise SystemExit(str(mismatch)) from mismatch


def check_poi_options(options: argparse.Namespace) -> None:
    """Exactly one way of saying where the POI is. Refuse a mixture."""
    absolute = [name for name in ABSOLUTE if getattr(options, name) is not None]
    placement = [name for name in PLACEMENT if getattr(options, name) is not None]
    if absolute and placement:
        raise SystemExit(
            "give absolute coordinates OR a placement, not both: "
            f"{sorted(absolute + placement)}"
        )
    if absolute and len(absolute) != len(ABSOLUTE):
        raise SystemExit(
            f"absolute placement needs all of {list(ABSOLUTE)}, got {absolute}"
        )
    if placement and len(placement) != len(PLACEMENT):
        raise SystemExit(
            f"relative placement needs all of {list(PLACEMENT)}, got {placement}"
        )
    if not absolute and not placement:
        raise SystemExit(
            "no POI: give --poi-lat/--poi-lon/--poi-alt, or "
            "--poi-range-m/--poi-below-m"
        )
    if placement:
        # `below` is one leg of a right triangle whose hypotenuse is `range`, so
        # it cannot be the longer of the two. Asked for a 100 m range 500 m
        # below, the geometry has no solution -- and the arithmetic silently
        # produced a POI 500 m away, five times the range requested, with
        # nothing in the result saying so.
        if options.poi_range_m <= 0.0:
            raise SystemExit(
                f"--poi-range-m must be positive, got {options.poi_range_m}"
            )
        if abs(options.poi_below_m) >= options.poi_range_m:
            raise SystemExit(
                f"--poi-below-m {options.poi_below_m} is not below a range "
                f"of {options.poi_range_m}: the drop cannot equal or exceed "
                "the slant range, so no POI position satisfies both"
            )
    check_estimate_options(options)


def check_estimate_options(options: argparse.Namespace) -> None:
    """The delay is a count of poses, so it cannot be negative.

    Nothing else is refused here. Delay is valid on EITHER source -- that
    independence is the whole point of the 2x2 -- and a zero delay is a real
    cell of it rather than a mistake.
    """
    lag.check_lag_options(options)
    if options.estimate_delay_poses < 0:
        raise SystemExit(
            f"--estimate-delay-poses must not be negative, got "
            f"{options.estimate_delay_poses}"
        )
    if options.estimate_jitter_deg < 0.0:
        # An RMS cannot be negative. Gauss would accept it and silently return
        # the same distribution as the positive value, so a typo would produce
        # a run that looks like a different arm and is not.
        raise SystemExit(
            f"--estimate-jitter-deg is an RMS and must not be negative, got "
            f"{options.estimate_jitter_deg}"
        )


def resolve_poi(
    options: argparse.Namespace, pose: TruthPose,
) -> tuple[float, float, float]:
    """Where the POI is, decided once and never revisited.

    Truth heading appears here and nowhere else. This is the harness CHOOSING
    where to put a POI before the scoring interval begins, not the aircraft
    working out where one is: the resolved coordinates are three constants from
    the first pose onward, and every line of sight after this is built from
    them. Placing a POI relative to the nose is what makes a run repeatable
    across aircraft that finished their climb in different places, which is the
    whole reason this is not a fixed coordinate chosen up front.
    """
    if options.poi_lat is not None:
        return options.poi_lat, options.poi_lon, options.poi_alt
    bearing = math.radians(pose.yaw_deg + options.poi_off_boresight_deg)
    # No clamp. An earlier version wrote `max(..., 0.0)` here, which turned an
    # impossible geometry into a silently different one -- range 100 with a
    # 500 m drop resolved to a POI 500 m away and reported nothing.
    # check_poi_options refuses that at the command line; this raises for any
    # caller that skipped it, because a wrong POI is not recoverable later.
    squared = options.poi_range_m ** 2 - options.poi_below_m ** 2
    if squared <= 0.0:
        raise ValueError(
            f"range {options.poi_range_m} cannot contain a drop of "
            f"{options.poi_below_m}"
        )
    horizontal = math.sqrt(squared)
    return coordinates_from_offset_ned(
        lat_deg=pose.lat_deg, lon_deg=pose.lon_deg, alt_m=pose.alt_m,
        offset_ned_m=np.array([
            horizontal * math.cos(bearing),
            horizontal * math.sin(bearing),
            options.poi_below_m,
        ]),
    )


def _law(options: argparse.Namespace) -> VisionNavLaw:
    return VisionNavLaw(
        FixedFinalApproachLawConfigProvider(
            FinalApproachLawConfig(
                pitch_min_deg=options.pitch_min_deg,
                pitch_max_deg=options.pitch_max_deg,
                roll_limit_deg=options.roll_limit_deg,
                pitch_time_constant_s=None,
                throttle=options.throttle,
            )
        )
    )


def _record_exit_evidence(
    result: dict,
    poses: TruthPoseStream,
    started_wall_s: float | None,
    started_t_s: float | None,
) -> None:
    """Stamp pose evidence and the measured clock rate onto every run exit.

    Called from EVERY return in `run_navigation_episode`, including the abandoned ones. An
    abandoned run is exactly when you most want to know whether the feed was
    healthy, and an earlier version returned from the wall guard without
    recording it.

    The clock rate is MEASURED, not assumed from the request. SIM_SPEEDUP is
    persisted in the simulated eeprom, and a request that fails to apply leaves
    the aircraft running at the stored rate while the run reports the rate it
    asked for -- which has already silently invalidated a whole sweep once.
    Simulated seconds over wall seconds is the rate the aircraft actually ran
    at, whatever any parameter says.
    """
    result["pose_summary"] = poses.summary()
    result["pose_certification"] = poses.certification_error()
    wall_s = (None if started_wall_s is None
              else time.monotonic() - started_wall_s)
    result["measured_speedup"] = (
        None if not wall_s or started_t_s is None
        else round((result.get("engage_s") or 0.0) / wall_s, 3)
    )


def run_navigation_episode(link: MavlinkLink, options: argparse.Namespace, flight: Flight,
           result: dict) -> str:
    """Navigate until the POI goes behind, or simulated time runs out.

    Returns why it stopped. The reason matters as much as the miss: a run that
    stopped on the clock did not measure a closest approach, it measured where
    it gave up.

    Both stopping conditions are legal command-path inputs. The pass is read off
    the line of sight -- the same frame the law is handed -- and the limit is the
    simulated clock. Nothing the aircraft could not observe decides when this
    loop ends, and therefore nothing it could not observe decides how many
    commands the law gets to issue.
    """
    law = _law(options)
    # The gates follow the rate this run actually got. Built from the module
    # defaults instead, a 160 Hz feed would be judged against a 40 Hz standard
    # and could lose fifteen consecutive poses while still certifying.
    rate_hz = (result.get("pose_rate") or {}).get("requested_hz",
                                                 TRUTH_POSE_RATE_HZ)
    poses = TruthPoseStream(
        max_skew_s=pose_frame_association_max_skew_s(rate_hz),
        max_gap_s=pose_gap_limit_s(rate_hz),
    )
    approach = ClosestApproach()
    commanded = {"pitch": 0.0, "roll": 0.0}
    poi: tuple[float, float, float] | None = None
    started_t_s: float | None = None
    started_wall_s: float | None = None
    next_command_t_s: float | None = None
    latest_frame = None
    was_ahead = False
    ground_contact_t_s: float | None = None
    commands = 0
    rejected_frames = 0
    saturated = 0
    # Depth 1 holds only the current pose, which IS the undelayed arm, so the
    # delayed arm needs no separate branch at the point of use.
    estimates: deque[tuple[float, float]] = deque(
        maxlen=max(options.estimate_delay_poses, 0) + 1)
    # The CONTINUOUS arm. `estimate_delay_poses` can only express multiples of
    # the ~25 ms attitude interval, which is coarser than the effect being
    # measured; a lag in seconds separates the quantisation from the lag.
    lag_sampler = AttitudeLagSampler(options.estimate_lag_s)
    atomic = (AtomicDelayBuffer(options.estimate_lag_s)
              if options.estimate_lag_scope == "atomic" else None)
    latest_frame_t_s: float | None = None
    unconvertible_yaw_rate = 0
    # Counted rather than assumed. A run whose scoring interval fell back to the
    # untreated airspeed would otherwise be scored as a treated one and read as
    # "the treatment did nothing".
    oracle_applied = 0
    oracle_fallback = 0
    oracle_speeds: list[float] = []
    warmup_unbracketed = 0
    held_commands = 0
    jitter = random.Random(options.estimate_jitter_seed)
    roll = RollRecord()

    # Wall clock ONLY to abandon a hung run. Nothing it touches becomes a
    # result: it can end the run, never alter it.
    wall_guard_s = time.monotonic() + max(
        600.0, 5.0 * options.scoring_duration_s / max(options.speedup, 0.1)
    )
    first_pose_deadline_s = time.monotonic() + FIRST_POSE_TIMEOUT_S

    def finish(reason: str) -> str:
        _record_exit_evidence(result, poses, started_wall_s, started_t_s)
        return reason

    while True:
        if time.monotonic() >= wall_guard_s:
            return finish("stalled")
        # Checked at the TOP of the loop, before anything can `continue` past
        # it. An earlier version put it inside the `pose is None` branch, which
        # is only reached when a message arrived -- so the one failure it most
        # needed to name, a link that went completely silent, was the one it
        # could not reach. That run fell through to the hung-run guard instead
        # and reported "stalled" fifteen minutes later.
        if started_t_s is None and time.monotonic() >= first_pose_deadline_s:
            # The pose summary that finish() records says which half of the
            # pair was missing, or whether nothing arrived at all.
            return finish("no_pose")
        message = link.recv_match(blocking=True, timeout=0.01)
        if message is None:
            continue
        if message.get_srcSystem() != options.sysid:
            continue
        flight._absorb(message)
        pose = poses.absorb(message)
        if pose is None:
            continue

        # What the LAW observes. `atomic` hands it a whole earlier pose --
        # ray, attitude and timestamp from one instant -- while scoring and
        # the run clocks below stay on the current pose, because the miss is
        # a fact about reality, not about the observation the law acted on.
        observed = pose if atomic is None else atomic.push(pose)
        de_rotation = None if observed is None else de_rotating_attitude(
            options, observed, estimates, lag_sampler, jitter)
        if started_t_s is None:
            if de_rotation is None:
                continue  # warm-up: lag history shorter than the lag
            warmup_unbracketed = lag_sampler.unbracketed
            started_t_s = pose.t_s
            started_wall_s = time.monotonic()
            say(f"  scoring start: t={pose.t_s:.1f}s "
                f"alt={pose.alt_m:.0f}m yaw={pose.yaw_deg:.0f}deg")
            next_command_t_s = None
            # Resolved from the FIRST pose and then constant. Recorded in full
            # so the run can be repeated exactly with absolute coordinates.
            poi = resolve_poi(options, pose)
            result["poi"] = list(poi)
            result["poi_from"] = entry_state(
                pose, flight.airspeed_now, flight.path_angle_now_deg,
                flight.ground_speed_now, flight.course_now_deg)
        elapsed_s = pose.t_s - started_t_s

        offset = poi_offset_ned_m(
            lat_deg=pose.lat_deg, lon_deg=pose.lon_deg, alt_m=pose.alt_m,
            poi_lat_deg=poi[0],
            poi_lon_deg=poi[1],
            poi_alt_m=poi[2],
        )
        approach.observe(pose.t_s, offset)
        range_m = float(np.linalg.norm(offset))
        roll.observe_geometry(range_m, offset)

        # The frame is built on EVERY pose, not only when a command is due,
        # because it is also the pass detector. One frame, built once, read by
        # both: the law and the termination test therefore see the identical
        # view a camera would have produced, and cannot silently disagree about
        # where the POI is.
        #
        # TRUTH builds the ray, because that is the camera's job and a camera is
        # never wrong about where a thing appears. The ESTIMATE supplies
        # everything the law reads about the aircraft itself, because that is all
        # a real aircraft has. Handing the law truth pitch or a truth body rate
        # would let it fly on information the flying article does not possess,
        # and the test would pass for a reason that does not transfer.

        # BODY r rate to EULER yaw rate, via PRODUCTION's own function --
        # `real_detected_object_builder.py:122-125` does the same on the real
        # path, and a private copy here is how the two would come to disagree.
        euler_yaw_rate_rad_s = None if de_rotation is None else (
            aircraft_yaw_rate_rad_s(  # angles matched to the rates' instant
                de_rotation.rate_pitch_deg, de_rotation.rate_roll_deg,
                de_rotation.rates)
        )
        airspeed = flight.airspeed_now
        if options.oracle_gain != "none" and airspeed:
            # TRUTH-FED, and only when a cell asked for it. Substituted into
            # the AIRSPEED because that is the law's only multiplier
            # (law.py:164), so this moves the PN gain and nothing else.
            oracle = substitute_speed(
                options.oracle_gain,
                channel_airspeed_mps=airspeed,
                ground_speed_mps=flight.ground_speed_now,
                course_deg=flight.course_now_deg,
                wind_speed_mps=(options.wind_speed or 0.0),
                wind_dir_deg=options.wind_dir,
                offset_ned_m=offset,
            )
            if oracle.applied:
                oracle_applied += 1
                oracle_speeds.append(oracle.speed_mps)
            else:
                oracle_fallback += 1
            airspeed = oracle.speed_mps
        if euler_yaw_rate_rad_s is None:
            # Near-vertical pitch divides by cos(pitch) and the conversion has
            # no answer. Counted rather than substituted: a fabricated rate here
            # would be indistinguishable in the result from a measured one.
            unconvertible_yaw_rate += 1
        elif (de_rotation is not None
              and airspeed is not None and airspeed > 0.0):
            est_pitch_deg = de_rotation.pitch_deg
            est_roll_deg = de_rotation.roll_deg
            latest_frame = build_frame(
                offset_ned_m=offset if observed is pose else
                poi_offset_ned_m(
                    lat_deg=observed.lat_deg, lon_deg=observed.lon_deg,
                    alt_m=observed.alt_m, poi_lat_deg=poi[0],
                    poi_lon_deg=poi[1], poi_alt_m=poi[2]),
                truth_pitch_deg=observed.pitch_deg,
                truth_roll_deg=observed.roll_deg,
                truth_yaw_deg=observed.yaw_deg,
                est_pitch_deg=est_pitch_deg,
                est_roll_deg=est_roll_deg,
                airspeed_mps=airspeed,
                yaw_rate_rad_s=euler_yaw_rate_rad_s,
                source_timestamp_s=observed.t_s,
            )
            # Which pose this frame describes. Compared against the current pose
            # before commanding, so a frame can never be presented twice.
            latest_frame_t_s = pose.t_s  # arrival identity, not content time
            next_command_t_s = next_command_t_s or pose.t_s
            if latest_frame.body_x >= PASSED_AHEAD_MIN:
                was_ahead = True

        if next_command_t_s is not None and pose.t_s >= next_command_t_s:
            if latest_frame is None or latest_frame_t_s != pose.t_s:
                # NO FRAME FOR THIS POSE, so there is nothing current to command
                # from. The previous frame describes an EARLIER instant, and
                # handing it back to the law would present a stale view as a
                # fresh one: the law's own dt would be zero or negative, it
                # would return a held command, and that held command would then
                # be counted in `commands`, in the reversal tally and in the
                # roll amplitude -- indistinguishable in the result from a
                # command actually computed for this moment.
                #
                # Two ways to get here, and both are the harness failing to
                # supply an input rather than the law declining one: the yaw
                # rate would not convert, or airspeed was not yet known.
                held_commands += 1
            elif (plan := law.plan(latest_frame)) is None:
                # The law refused this frame. Counted, not silently retried: a
                # law that refuses often is not being tested.
                rejected_frames += 1
            else:
                law.commit(plan)
                commanded["pitch"] = plan.command.cmd_pitch_deg
                commanded["roll"] = plan.command.cmd_roll_deg
                commands += 1
                if plan.lateral_held:
                    roll.held(pose.t_s)
                if (
                    abs(commanded["roll"]) >= options.roll_limit_deg - 1e-6
                    or commanded["pitch"] <= options.pitch_min_deg + 1e-6
                    or commanded["pitch"] >= options.pitch_max_deg - 1e-6
                ):
                    saturated += 1
                # The COMMAND and the aircraft's RESPONSE to it, summarised the
                # same way so they can be read against each other. A command
                # that dithers is only harmless if the airframe ignores it, and
                # nothing recorded before this could tell the difference:
                # `abs(roll)` alone reads identically for a steady 1.3 degree
                # bank and a 1.3 degree oscillation, which is exactly the
                # distinction at issue.
                roll.observe_command(
                    commanded["roll"], pose.est_roll_deg, pose.roll_deg,
                    plan.lateral_rate)
            next_command_t_s += COMMAND_INTERVAL_S

        quaternion = _euler_to_quaternion(
            math.radians(commanded["roll"]), math.radians(commanded["pitch"]), 0.0
        )
        link.mav.set_attitude_target_send(
            int(pose.t_s * 1000.0) & 0xFFFFFFFF, options.sysid, 0,
            ATTITUDE_HOLD_MASK, quaternion, 0.0, 0.0, 0.0, options.throttle,
        )
        result.update({
            "commands": commands,
            "rejected_frames": rejected_frames,
            "unconvertible_yaw_rate": unconvertible_yaw_rate,
            "oracle_gain": options.oracle_gain,
            "oracle_applied": oracle_applied,
            "oracle_fallback": oracle_fallback,
            "oracle_speed_median_mps": (
                round(statistics.median(oracle_speeds), 3)
                if oracle_speeds else None),
            "held_commands": held_commands,
            # Reported, never hidden: a lag whose bracket is missing is a
            # command that did not happen, and a run with many of them is not a
            # slower version of the same experiment.
                "lag_unbracketed": lag_sampler.unbracketed - warmup_unbracketed,
            "lag_warmup_poses": warmup_unbracketed,
            "saturated_commands": saturated,
            # Total held-roll commands, and how many fell in the last 3 s: the
            # guard triggers on LOS geometry that tightens toward closest
            # approach, so a total alone cannot say whether the roll law was
            # frozen where the miss was decided.
            **roll.summary(pose.t_s),
            "engage_s": round(elapsed_s, 3),
            "miss_m": (None if approach.miss_m is None
                       else round(approach.miss_m, 3)),
            # Split so the roll law and the pitch law each answer for their
            # own channel: horizontal is the roll law's number.
            "miss_horizontal_m": (None if approach.miss_horizontal_m is None
                                  else round(approach.miss_horizontal_m, 3)),
            "miss_vertical_m": (None if approach.miss_vertical_m is None
                                else round(approach.miss_vertical_m, 3)),
            "miss_at_t_s": approach.at_t_s,
            "final_range_m": round(range_m, 2),
            "ground_contact_t_s": ground_contact_t_s,
        })
        # Ground contact is RECORDED, never acted on. Altitude is barred from
        # the command path, and "stop the run" is part of the command path: a
        # check that ends the loop decides whether the law issues any further
        # command. So this sets a field in the report and nothing else. A run
        # that flew into the ground still ends on the pass test or the clock.
        if ground_contact_t_s is None and (flight.alt_now_m or 0.0) <= options.floor_m:
            ground_contact_t_s = round(elapsed_s, 3)
            result["ground_contact_t_s"] = ground_contact_t_s

        # ---- RUN TERMINATION -- LEGAL INPUTS ONLY -------------------------
        # Both conditions below are things the flying article can observe.
        #
        # An earlier version stopped on truth range and truth altitude and put
        # those checks after the command send, arguing the ordering made them
        # safe. It did not. Whatever their position, they decided whether the
        # loop ran again, and therefore whether the law was given any further
        # frame to act on. Truth was still gating the command sequence; the
        # ordering only moved the gate by one iteration.
        #
        # The fix is not ordering, it is the input. The forward component of the
        # body ray is the sensor's own output -- literally the field the law
        # reads -- so a POI that was ahead and is now behind is a pass
        # detected from vision alone. Time is the simulated clock, which the law
        # also uses. Neither requires anything the aircraft lacks.
        #
        # One honest limit remains, unchanged by any of this: the HARNESS holds
        # the pass test, so this says nothing about whether the LAW could reach
        # the same conclusion on its own. Pass-state logic still has to be
        # validated somewhere else.
        if was_ahead and latest_frame.body_x <= PASSED_BEHIND_MAX:
            return finish("passed")
        if elapsed_s >= options.scoring_duration_s:
            return finish("limit")
        # ---- END TERMINATION BLOCK ---------------------------------------







def run(options: argparse.Namespace) -> dict:
    result: dict = initial_result(options, law_source_path())
    say(f"connecting to {options.connection} sysid {options.sysid}")
    link = _connect(options.connection, options.sysid, options.connect_timeout)
    if link is None:
        result["errors"].append("no heartbeat")
        return result
    say("  heartbeat")
    try:
        chatter = Chatter()
        ready, _waited = _wait_ready(
            link, options.sysid, options.ready_timeout, chatter=chatter
        )
        if not ready:
            result["errors"].append("never became ready")
            return result
        say(f"  ready after {_waited:.0f}s")
        # Every parameter set is CONFIRMED, because an unconfirmed one produces
        # a run that answers a different question while looking like a clean
        # result. Wind is the sharp case: a rejected SIM_WIND_SPD leaves the
        # aircraft flying in still air, and a calm-air miss filed as a windy one
        # is worse than no data. The clock has a second, stronger check -- it is
        # measured from the pose feed -- but its echo is recorded here too.
        settings = {
            SPEEDUP_PARAM: _set_param(
                link, options.sysid, SPEEDUP_PARAM, options.speedup),
        }
        if options.wind_speed is not None:
            settings["SIM_WIND_SPD"] = _set_param(
                link, options.sysid, "SIM_WIND_SPD", options.wind_speed)
            settings["SIM_WIND_DIR"] = _set_param(
                link, options.sysid, "SIM_WIND_DIR", options.wind_dir)
        result["parameters_set"] = settings
        refused = sorted(name for name, ok in settings.items() if not ok)
        if refused:
            result["errors"].append(
                f"parameter not confirmed: {refused} -- the aircraft did not "
                "echo the value, so this run did not fly the condition it claims"
            )
            return result

        # Recorded, not gated. Both streams ARE required -- SIM_STATE is the
        # truth and ATTITUDE is the clock, and one without the other is not a
        # pose -- but the acknowledgement is not what proves they arrived. An
        # earlier version aborted here, and did so on a broadcast the aircraft
        # was never addressed by, throwing away a healthy run before it flew.
        # What settles it is the pose feed itself, judged by
        # TruthPoseStream.certification_error() at the end.
        # ASKED OF THE AIRCRAFT, not assumed. The cap is 0.8 * SCHED_LOOP_RATE
        # and that parameter is 50 on this SITL Plane but 200 by default on a
        # Cube 6X, up to 400. A fixed request is refused outright on the slow
        # aircraft and wastes three quarters of the available rate on the fast
        # one -- and pose skew, which bounds the whole measurement, scales
        # directly with it.
        rate_hz, scheduler_hz = achievable_pose_rate_hz(
            lambda sysid, name: _read_param(link, sysid, name), options.sysid)
        if options.pose_rate_hz is not None:
            rate_hz = options.pose_rate_hz
        result["pose_rate"] = {
            "sched_loop_rate_hz": scheduler_hz,
            "requested_hz": rate_hz,
            "max_skew_s": pose_frame_association_max_skew_s(rate_hz),
            "max_gap_s": pose_gap_limit_s(rate_hz),
        }
        say(f"  pose rate {rate_hz:g}Hz "
            f"(SCHED_LOOP_RATE={scheduler_hz})")
        result["stream_requests"] = request_truth_pose_streams(
            link, options.sysid, rate_hz=rate_hz
        )
        say(f"  streams: {result['stream_requests']}")
        # WHAT THE LINK ACTUALLY CARRIES, not what was asked for. The first run
        # to reach this point died with two empty logs; a census would have said
        # in five seconds whether SIM_STATE was arriving at all.
        result["message_census"] = census(link, options.sysid, CENSUS_S)
        say(f"  census over {CENSUS_S:g}s: {result['message_census']}")
        if not result["message_census"].get("SIM_STATE"):
            result["errors"].append(
                "no SIM_STATE on this link -- simulator truth is the sensor "
                "here, so there is nothing to build a line of sight from"
            )
            return result

        # THE ORDER HERE IS THE PLANT HARNESS'S, and it is not decorative
        # (scratch_sitl_uav.py:572-594). An earlier version armed straight into
        # GUIDED on the runway: ArduPlane has no launch in GUIDED, so the
        # aircraft sat still for the whole climb limit and the scoring interval began
        # at ground level, with the POI placed below the terrain. The run
        # looked healthy the whole way -- armed, commanded, telemetry flowing.
        #
        #   TAKEOFF gives the aircraft a launch.
        #   ARMING_CHECK off, CONFIRMED: when that set silently fails the
        #     aircraft arms under checks that were never lifted, which is one of
        #     the few states that produces exactly "armed but never climbed".
        #   The settle phase reads THROUGH the climb-out rather than sleeping,
        #     which keeps the link drained so what follows times live flight
        #     instead of a backlog.
        #   GUIDED last, because ArduPlane DISCARDS SET_ATTITUDE_TARGET outside
        #     it and would hold trim attitude whatever it was told.
        if not _mode(link, options.sysid, "TAKEOFF", chatter=chatter):
            result["errors"].append("TAKEOFF not entered")
            return result
        if not _set_param(link, options.sysid, "ARMING_CHECK", 0.0):
            result["errors"].append("ARMING_CHECK not confirmed off")
            return result
        say("  TAKEOFF, arming checks off")
        if not _arm(link, options.sysid, options.arm_attempts,
                    options.arm_timeout, chatter=chatter):
            result["errors"].append("never armed")
            return result

        flight = Flight(link, options.sysid, options.speedup, chatter)
        say(f"  armed, climbing out for {options.settle_s:g} aircraft-seconds")
        result["settle_end"] = flight.phase(
            None, 0.0, 0.0, options.settle_s, lambda: False)
        say(f"  climbed out to {flight.alt_now_m}m")

        if not _mode(link, options.sysid, "GUIDED", chatter=chatter):
            result["errors"].append("GUIDED not entered")
            return result
        say(f"  GUIDED, climbing to {options.climb_to_m:g}m")
        climb_end = flight.phase(
            CLIMB_PITCH_DEG, 0.0, CLIMB_THROTTLE, CLIMB_LIMIT_S,
            lambda: (flight.alt_now_m or 0.0) >= options.climb_to_m,
        )
        result["climb_end"] = climb_end
        say(f"  climb {climb_end} at {flight.alt_now_m}m")
        if options.level_settle_s > 0.0:
            # At the SCORING throttle, so the trim the settle reaches is the
            # one the scoring interval inherits: settling at climb throttle would
            # trade the pitch transient for an airspeed one.
            say(f"  level-off for {options.level_settle_s:g} aircraft-seconds")
            result["level_end"] = flight.phase(
                0.0, 0.0, options.throttle, options.level_settle_s,
                lambda: False)
        scoring_end = run_navigation_episode(link, options, flight, result)
        result["engage_end"] = scoring_end
        say(f"  scoring window {scoring_end}: miss={result.get('miss_m')}m "
            f"commands={result.get('commands')} "
            f"clock={result.get('measured_speedup')}x")
        result["statustext"] = chatter.lines
        result["errors"].extend(classification_errors(
            scoring_end=scoring_end,
            miss_m=result.get("miss_m"),
            certification=result.get("pose_certification"),
            measured_speedup=result.get("measured_speedup"),
            requested_speedup=options.speedup,
            held_commands=result.get("held_commands") or 0,
        ))
    finally:
        link.close()
    return result


def main() -> int:
    options = _parser().parse_args()
    check_law_source(options)
    check_poi_options(options)
    result = run(options)
    text = json.dumps(result, indent=2, sort_keys=True)
    if options.out is not None:
        options.out.parent.mkdir(parents=True, exist_ok=True)
        options.out.write_text(text, encoding="utf-8")
    print(text)
    return 1 if result["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
