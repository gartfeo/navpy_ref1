"""Configuration options for simultaneous fleet evaluations."""
from __future__ import annotations

import argparse
from typing import Callable
from scripts.eval_fleet_cells import DEFAULT_CELLS

def build_parser(base_parser: Callable[..., argparse.ArgumentParser]) -> argparse.ArgumentParser:
    # Opt in to --scoring-policy and --sim-cpa: this evaluator implements
    # both. The bare parser would silently pin vehicle-estimate scoring, and
    # a fleet flown that way would look truth-scored without being it.
    parser = base_parser(scoring_policy=True, sim_cpa=True)
    parser.set_defaults(speedups="1", repetitions=1)
    parser.add_argument("--cells", default=DEFAULT_CELLS)
    parser.add_argument(
        "--sitl-defaults",
        default=None,
        metavar="WSL_PATH",
        help=(
            "pre-boot SITL parameter file (WSL path) replacing run_swarm.sh's "
            "shared models/plane.parm. Required for parameters consumed at "
            "buffer-allocation or calibration time, such as EK3_HGT_DELAY: "
            "the harness pushes parameters after boot, which is too late. "
            "Given, it replaces the bench profile rather than adding to it."
        ),
    )
    parser.add_argument(
        "--no-preboot-profile", action="store_true",
        help=(
            "boot the launcher template unmodified, without "
            "eval_preboot_defaults.parm. For reproducing a stock run; the "
            "estimator timing mismatch it removes is worth ~0.15 m of CPA "
            "error, so results are not comparable with a default run."
        ),
    )
    parser.add_argument(
        "--cell-reps", type=int, default=4,
        help="aircraft per wind cell; fleet size is cells x this",
    )
    parser.add_argument(
        "--assign-seed", type=int, default=20260821,
        help="seed for the within-replicate cell shuffle; recorded per case",
    )
    parser.add_argument(
        "--chat", type=int, default=None,
        help="base chat slot; defaults to the eval band floor",
    )
    return parser
