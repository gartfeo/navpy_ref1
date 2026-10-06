"""Certify that EVERY aircraft in a fleet is running at the requested rate.

The launcher does not do this past the third aircraft. `swarm_run_runner.py`
and `swarm_run_verifier.py` both verify `sysids_for_chat(chat)[:instances]`,
and that list is never longer than three, so a fleet of thirty-six gets three
aircraft certified and thirty-three assumed.

Assuming is not safe here. A starved SITL still integrates its physics
correctly in simulator time: it reports entirely plausible attitudes and
positions while its seconds quietly stop being seconds. Nothing in the miss
distance reveals it, so an aircraft whose clock slipped produces a number that
looks like a navigation result and is not one.

A gated run gets a second, independent check: each child measures its own clock
when it steps down at the gate and raises if it did not settle. That covers the
scored leg but only when a cruise speedup was asked for -- with none,
`_step_to_final_approach_speed` returns immediately and no child measures anything.
This module is what covers the fleet in both cases, so it is not conditional on
gating.

MEASURED ON THE SHARED LINK, IN ONE PASS. Every aircraft already streams
GLOBAL_POSITION_INT at 30 Hz because the coordinate scorer asked for it, and
that message carries `time_boot_ms` -- the autopilot's own clock. Reading them
all from the one connection and bucketing by source system certifies the whole
fleet over a single span instead of thirty-six sequential ones.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
if str(WORKTREE / "src") not in sys.path:
    sys.path.insert(0, str(WORKTREE / "src"))

import swarm_run_verification_model as model  # noqa: E402

CLOCK_TIMEOUT_S = 60.0


def measure_fleet_rates(
    master: object,
    sys_ids: list[int],
    *,
    span_s: float = model.CLOCK_MIN_SPAN_S,
    timeout_s: float = CLOCK_TIMEOUT_S,
) -> dict[int, float | None]:
    """Simulator seconds per wall second, per aircraft.

    `None` means the aircraft never produced two samples far enough apart to
    measure -- which is itself a failure, not a missing datum: an aircraft that
    is not reporting cannot be certified.
    """
    wanted = set(sys_ids)
    first: dict[int, tuple[float, float]] = {}
    last: dict[int, tuple[float, float]] = {}
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0
        )
        if message is None:
            continue
        sys_id = message.get_srcSystem()
        if sys_id not in wanted:
            continue
        sample = (message.time_boot_ms / 1000.0, time.monotonic())
        first.setdefault(sys_id, sample)
        last[sys_id] = sample
        if len(first) == len(wanted) and all(
            last[other][1] - first[other][1] >= span_s for other in wanted
        ):
            break

    rates: dict[int, float | None] = {}
    for sys_id in sys_ids:
        if sys_id not in first:
            rates[sys_id] = None
            continue
        wall = last[sys_id][1] - first[sys_id][1]
        rates[sys_id] = (
            None if wall < span_s
            else (last[sys_id][0] - first[sys_id][0]) / wall
        )
    return rates


def verify_fleet_clock(
    master: object,
    sys_ids: list[int],
    expected_rate: float,
    *,
    span_s: float = model.CLOCK_MIN_SPAN_S,
    timeout_s: float = CLOCK_TIMEOUT_S,
) -> dict[int, float | None]:
    """Raise unless every aircraft is keeping the rate the run claims.

    Raises rather than warns, and names the aircraft. A fleet with one starved
    member does not produce a slightly worse matrix -- it produces one cell
    whose numbers mean something different from the rest, which is worse than
    no result at all.
    """
    rates = measure_fleet_rates(
        master, sys_ids, span_s=span_s, timeout_s=timeout_s
    )
    failed = {
        sys_id: rate
        for sys_id, rate in rates.items()
        if rate is None or not model.rate_matches(rate, expected_rate)
    }
    if failed:
        detail = ", ".join(
            f"{sys_id}={'no samples' if rate is None else f'{rate:.3f}x'}"
            for sys_id, rate in sorted(failed.items())
        )
        raise RuntimeError(
            f"{len(failed)} of {len(sys_ids)} aircraft are not running at "
            f"{expected_rate:g}x (tolerance "
            f"{model.CLOCK_RATE_TOLERANCE:g}): {detail}"
        )
    return rates


__all__ = ["CLOCK_TIMEOUT_S", "measure_fleet_rates", "verify_fleet_clock"]
