"""Verification data and clock/parameter measurement helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


VERIFY_TIMEOUT = 60.0
PARAM_TIMEOUT = 15.0
PARAM_RESEND_INTERVAL = 3.0
BOOT_EVIDENCE_TIMEOUT = 6.0
BOOT_REQUEST_INTERVAL = 3.0
BOOT_WALL_GRACE = 2.0
BOOT_SIM_SLACK_MS = 250.0
SYSTEM_TIME_MSG_ID = 2
SYSTEM_TIME_MSG = "SYSTEM_TIME"
BOOT_TIME_MESSAGES = frozenset({SYSTEM_TIME_MSG, "ATTITUDE"})
CLOCK_RATE_TOLERANCE = 0.25
CLOCK_MIN_SPAN_S = 1.0
SPEEDUP_PARAM = "SIM_SPEEDUP"


@dataclass(frozen=True)
class VerificationResult:
    """Evidence collected for one swarm launch attempt."""

    heartbeats: frozenset[int]
    speedups: dict[int, float]
    error: str | None = None
    boot_evidence: frozenset[int] = frozenset()
    stale: dict[int, float] = field(default_factory=dict)
    rates: dict[int, float] = field(default_factory=dict)


def param_name(message: Any) -> str:
    """Normalize a NUL-padded MAVLink parameter name."""
    name = message.param_id
    if isinstance(name, bytes):
        name = name.decode("ascii", "ignore")
    return str(name).rstrip("\x00")


def boot_ms(message: Any) -> float | None:
    """Return a valid uint32 autopilot boot clock from a message."""
    value = getattr(message, "time_boot_ms", None)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value) or not 0 <= value <= 0xFFFFFFFF:
        return None
    return float(value)


def boot_limit_ms(speedup: float, launched_at: float, now: float) -> float:
    """Return the largest boot clock attributable to this launch."""
    return (
        1000.0 * speedup * (now - launched_at + BOOT_WALL_GRACE)
        + BOOT_SIM_SLACK_MS
    )


def drain(connection: Any) -> None:
    """Discard every message already queued on a MAVLink connection."""
    while connection.recv_match(blocking=False) is not None:
        pass


def stale_instances(
    worst: dict[int, tuple[float, float]],
    speedup: float,
    launched_at: float,
) -> dict[int, float]:
    """Return boot clocks too old to belong to the current launch."""
    return {
        sysid: boot
        for sysid, (boot, seen_at) in worst.items()
        if boot > boot_limit_ms(speedup, launched_at, seen_at)
    }


def measured_rates(
    first: dict[int, tuple[float, float]],
    last: dict[int, tuple[float, float]],
    min_span: float = CLOCK_MIN_SPAN_S,
) -> dict[int, float]:
    """Return measured sim-seconds per wall-second for usable sample spans."""
    rates: dict[int, float] = {}
    for sysid, (first_wall, first_boot) in first.items():
        last_wall, last_boot = last[sysid]
        wall_span = last_wall - first_wall
        if wall_span >= min_span:
            rates[sysid] = (last_boot - first_boot) / 1000.0 / wall_span
    return rates


def speedup_matches(actual: float, requested: float) -> bool:
    """Return whether reported configuration exactly matches the request."""
    return math.isclose(actual, requested, rel_tol=1e-6, abs_tol=1e-4)


def rate_matches(actual: float, requested: float) -> bool:
    """Return whether measured clock rate is within the verification tolerance."""
    return abs(actual - requested) <= CLOCK_RATE_TOLERANCE * requested
