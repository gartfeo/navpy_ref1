"""Which SITL parameters a scored case sets before it flies.

Its own module because two harnesses need the same answer. The one-UAV and
three-UAV evaluators both take their flags from `eval_direct_pixel_pn._parser`,
so a flag added for one already parses for the other -- but each used to hold
its own copy of the parameter tuple. A flag added to the shared parser would
then be accepted by a three-UAV run and silently ignored, which is worse than
rejecting it: the run looks like it flew what you asked for.
"""

from __future__ import annotations

import argparse
import math
from typing import Any

# SITL steps physics in INTEGER microseconds: `SIM_Aircraft.cpp` casts
# 1e6/rate_hz, so 1200 Hz becomes 833 us steps and 24 of them make 19.992 ms
# -- an 8 us deficit per 20 ms scheduler tick that surfaces as a boot-locked
# 2.082 s clock beat (one 21 ms tick per ~104). EKF3 turns that beat into a
# 0.033 deg pitch-estimate tone at 0.48 Hz, the closed navigation loop amplifies
# it into a 0.30 deg pitch wobble, and the wobble's phase at closest approach
# set most of the vertical miss scatter (truth-CPA median 0.516 m at 1200 Hz,
# n=15, vs 0.120 m at 1000 Hz, n=30; 2026-08-24 intervention batches).
#
# 1000 Hz steps are exactly 1000 us, so the beat cannot exist. Any override
# must keep 1e6/rate an integer or the beat returns.
SIM_RATE_HZ_DEFAULT = 1000.0

# Parameters the harness derives OTHER recorded numbers from. Overriding one
# through the escape hatch would desynchronise SITL from the artifacts: a
# `--sitl-param SIM_SPEEDUP=10` run would fly at 10x while freshness and the
# scored results still assume the `--speedups` clock, and an overridden wind
# would leave the ground-track summary computed against the CLI wind. Each is
# rejected with the flag that actually controls it.
HARNESS_OWNED = {
    "ARMING_CHECK": "always 0.0 for an eval run",
    # Isolation policy like ARMING_CHECK: a live fence RTLs the scoring interval
    # leg, so overriding it through the escape hatch would fly a run whose
    # score means nothing.
    "FENCE_ENABLE": "always 0.0 for an eval run",
    "FENCE_AUTOENABLE": "always 0.0 for an eval run",
    "SIM_SPEEDUP": "--speedups",
    "SIM_WIND_SPD": "--wind-speed",
    "SIM_WIND_DIR": "--wind-dir",
}


def parse_sitl_param(raw: str) -> tuple[str, float]:
    """One NAME=VALUE pair, validated; fails loudly on a bad value."""
    name, separator, value = raw.partition("=")
    name = name.strip()
    if not separator or not name:
        raise ValueError(f"--sitl-param must be NAME=VALUE; got {raw!r}")
    if not name.isascii():
        # BEFORE upper(): some Unicode uppercases INTO ASCII (long s -> S,
        # sharp s -> SS) and would bypass this check. `set_param` does
        # name.encode("ascii") only AFTER SITL is up; fail here instead.
        raise ValueError(f"--sitl-param name must be ASCII; got {raw!r}")
    name = name.upper()
    owner = HARNESS_OWNED.get(name)
    if owner is not None:
        raise ValueError(
            f"--sitl-param cannot override {name}: the harness derives "
            f"recorded results from it ({owner})"
        )
    try:
        number = float(value)
    except ValueError:
        raise ValueError(
            f"--sitl-param value must be a number; got {raw!r}"
        ) from None
    if not math.isfinite(number):
        # inf/nan would make set_param's relative tolerance infinite (any
        # echo passes) and serialize as non-standard JSON in the manifest.
        raise ValueError(f"--sitl-param value must be finite; got {raw!r}")
    return name, number


def _checked_at_parse_time(raw: str) -> str:
    """Reject a bad override BEFORE SITL boots, not minutes into the case."""
    try:
        parse_sitl_param(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None
    return raw


def add_sitl_param_argument(parser: argparse.ArgumentParser) -> None:
    """One escape hatch instead of one flag per SITL knob.

    Intervention experiments (clock stepping, GPS cadence, estimator choice)
    each need a different parameter for a handful of runs. A dedicated flag
    per knob would grow this module forever; an undeclared knob would mean
    editing the harness between batches, which aborts the batch on the
    source-identity gate. Every value pushed this way is recorded per case,
    so an archived run says exactly what it flew.
    """
    parser.add_argument(
        "--sitl-param",
        action="append",
        default=None,
        type=_checked_at_parse_time,
        metavar="NAME=VALUE",
        help=(
            "extra SITL parameter to push before flight, repeatable "
            "(e.g. --sitl-param SIM_RATE_HZ=1200). Pushed after the built-in "
            "parameters, so it can override them. Recorded per case. "
            "Harness-owned parameters (SIM_SPEEDUP, wind, ARMING_CHECK) are "
            "rejected: use their own flags."
        ),
    )


def extra_sitl_params(args: Any) -> tuple[tuple[str, float], ...]:
    """`--sitl-param NAME=VALUE` pairs, parsed."""
    return tuple(parse_sitl_param(raw) for raw in args.sitl_param or [])


def sim_parameters(args: Any) -> tuple[tuple[str, float], ...]:
    """Every SITL parameter a case sets, in the order it sets them."""
    return (
        ("ARMING_CHECK", 0.0),
        # The launcher syncs each instance's eeprom from a MUTABLE template;
        # an earlier session's geofence survives in it and auto-enables on
        # takeoff, RTLing the climb before the scoring interval leg (2026-09-03:
        # both cases of the first truth-scoring sweep died this way). Fence
        # state is isolation policy for an eval run, same as ARMING_CHECK.
        ("FENCE_ENABLE", 0.0),
        ("FENCE_AUTOENABLE", 0.0),
        ("SIM_WIND_SPD", args.wind_speed),
        ("SIM_WIND_DIR", args.wind_dir),
        ("SIM_RATE_HZ", SIM_RATE_HZ_DEFAULT),
    ) + extra_sitl_params(args)


__all__ = [
    "HARNESS_OWNED",
    "SIM_RATE_HZ_DEFAULT",
    "add_sitl_param_argument",
    "extra_sitl_params",
    "parse_sitl_param",
    "sim_parameters",
]
