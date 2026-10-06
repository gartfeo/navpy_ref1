"""Command-line options shared by the pixel-based approach evaluators."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scripts.eval_direct_pixel_verdict import (
    SCORING_POLICIES, SCORING_POLICY_SITL_TRUTH, SCORING_POLICY_VEHICLE_ESTIMATE,
)
from scripts.eval_sim_cpa_config import SIM_CPA_MODES
from scripts.eval_sim_parameters import add_sitl_param_argument
from scripts.upload_north_line_mission import (
    DEFAULT_ALT_M, DEFAULT_GATE_OFFSET_M, DEFAULT_LOITER_OFFSET_M,
    DEFAULT_WAYPOINT_OFFSET_M,
)

# Open water in the central Black Sea: exactly flat, at sea-level air density.
# Terrain is the reason -- the previous land site climbed 207 m over 5 km, so a
# level mission leg ended up below ground and no vertical result meant anything.
# Keep in step with DEFAULT_HOME in scripts/upload_north_line_mission.py, which
# builds the mission that assumes this start point.
DEFAULT_HOME_COORDS = "43.0,34.0,0,0"


def build_parser(
    *,
    cruise_speedup: bool = True,
    scoring_policy: bool = False,
    sim_cpa: bool = False,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--speedups", default="10,1")
    if cruise_speedup:  # a swarm shares no clock: three vehicles, three SITLs
        parser.add_argument(  # see pixel_pn_final_approach_speed for the step-down
            "--cruise-speedup", type=float, default=0.0,
            help="cruise to the gate at this speed; 0 flies the scored speed throughout")
    parser.add_argument("--repetitions", type=int, default=3)
    # The uploaded line has two NAV_WAYPOINTs: ordinal 1 is the handover gate
    # and ordinal 2 is the POI.  Both options must name the POI, ordinal
    # 2: scoring interval fires when MISSION_CURRENT reaches the POI's sequence,
    # which happens when the aircraft *reaches the gate*.  Naming ordinal 1
    # instead would score the gate as the POI and hand over at the loiter
    # exit, throwing away the fixed-length scored leg the gate exists to give.
    # LOITER_TO_ALT is not a NAV_WAYPOINT and is not counted in the ordinals.
    parser.add_argument("--poi-wp", type=int, default=2)
    parser.add_argument("--scoring-start-wp", type=int, default=2, dest='scoring_start_wp')
    parser.add_argument(
        "--mission-alt",
        type=float,
        default=DEFAULT_ALT_M,
        help="cruise altitude of the mission leg, relative to home",
    )
    # The scored leg is POI offset minus gate offset.  Both are exposed so
    # a run can vary leg length while holding the required glide angle fixed:
    # comparing an 888 m leg at 340 m of drop against a 300 m leg at 340 m
    # would change the glide from 21 to 49 degrees, so any difference could not
    # be attributed to leg length.  Matching the angle (300 m leg, 115 m drop,
    # i.e. --mission-alt 400 --poi-alt 285) isolates it.
    # Pinned to the default, a moved gate left the climb overshooting the loiter.
    parser.add_argument(
        "--loiter-offset", type=float, default=DEFAULT_LOITER_OFFSET_M)
    parser.add_argument(
        "--gate-offset",
        type=float,
        default=DEFAULT_GATE_OFFSET_M,
        help="metres north of home for the handover gate",
    )
    parser.add_argument(
        "--poi-offset",
        type=float,
        default=DEFAULT_WAYPOINT_OFFSET_M,
        help="metres north of home for the POI waypoint",
    )
    parser.add_argument("--poi-alt", type=float, default=60.0)
    parser.add_argument("--max-distance", type=float, default=1.0)
    # A separate, tighter bar. --max-distance is the gate that decides whether a
    # run is scoreable at all; this is the accuracy the project is actually
    # aiming at. Reporting only the gate invites reading "passed" as "goal met"
    # when the two differ by a factor of ten.
    parser.add_argument(
        "--goal-distance",
        type=float,
        default=0.1,
        help="accuracy target, reported alongside the pass gate, never as it",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    # Opt-in: only evaluators that actually IMPLEMENT truth scoring may
    # advertise the flag.  The three-UAV and SIYI evaluators inherit this
    # parser but still score the legacy EKF way; accepting-but-ignoring a
    # safety-relevant option would misrepresent what they measure, so they
    # get a fixed vehicle-estimate policy instead of the flag.
    if scoring_policy:
        parser.add_argument(
            "--scoring-policy",
            choices=SCORING_POLICIES,
            default=SCORING_POLICY_SITL_TRUTH,
            help=(
                "which source is authoritative for accuracy: certified "
                "simulator truth (SITL default; fails closed without it) or "
                "the legacy EKF estimate for runs with no simulator"
            ),
        )
    else:
        parser.set_defaults(scoring_policy=SCORING_POLICY_VEHICLE_ESTIMATE)
    # Same opt-in rule as --scoring-policy: only the evaluator that actually
    # RUNS the SIM_CPA cross-check advertises the flag.  The three-UAV and
    # SIYI harnesses inherit this parser but never configure the module, so
    # accepting the option there would misrepresent what they record.
    if sim_cpa:
        parser.add_argument(
            "--sim-cpa",
            choices=SIM_CPA_MODES,
            default=None,
            help=(
                "SIM_CPA firmware cross-check: auto (default) derives the "
                "module POI from the case POI and scores every run; "
                "off keeps the module dark (baseline arms). The module "
                "POI can never be redirected by hand; explicit "
                "--sitl-param SIM_CPA_* values other than ENABLE=0 are "
                "rejected."
            ),
        )
    else:
        parser.set_defaults(sim_cpa=None)
    # Wind is an experiment input, not a fixed property of the harness. It used
    # to be pinned to zero here, which silently made every run calm and made a
    # wind sweep impossible without editing this file.
    parser.add_argument("--wind-speed", type=float, default=0.0)
    parser.add_argument(
        "--wind-dir",
        type=float,
        default=0.0,
        help="degrees the wind blows FROM (ArduPilot SIM_WIND_DIR convention)",
    )
    add_sitl_param_argument(parser)
    parser.add_argument(
        "--home",
        default=DEFAULT_HOME_COORDS,
        metavar="LAT,LON,ALT,HEADING",
        help=(
            "swarm start point, forwarded to swarm_run. Home is fixed when SITL "
            "launches and no mission upload can move it, so it has to be set "
            f"here. Default {DEFAULT_HOME_COORDS} is flat open water."
        ),
    )
    return parser
