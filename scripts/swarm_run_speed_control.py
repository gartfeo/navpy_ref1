"""SITL speed correction and verification verdict policy."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from gcs.backend import instance_ports as ip
from pymavlink import mavutil

import swarm_run_verification_model as model


HEAL_TIMEOUT = 10.0
HEAL_RESEND_INTERVAL = 2.0


@dataclass(frozen=True)
class SpeedCorrectionResult:
    """A possible send is separate from matching echoes or durable state."""

    write_attempted: bool
    all_echoes_matched: bool
    detail: str


def await_peer(connection: Any, deadline: float) -> bool:
    """Wait for an inbound datagram so pymavlink learns its reply address."""
    while time.monotonic() < deadline:
        if connection.recv_match(blocking=True, timeout=1.0) is not None:
            return True
    return False


def force_speedup(
    chat: int,
    expected: set[int],
    speedup: float,
    timeout: float = HEAL_TIMEOUT,
    *,
    resend_interval: float = HEAL_RESEND_INTERVAL,
) -> SpeedCorrectionResult:
    """Attempt SIM_SPEEDUP writes and report possible sends and observed echoes."""
    port = ip.verify_port(chat)
    try:
        connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
    except OSError as exc:
        return SpeedCorrectionResult(False, False, f"could not bind verify port {port} ({exc})")
    confirmed: dict[int, float] = {}
    write_attempted = False
    try:
        deadline = time.monotonic() + timeout
        if not await_peer(connection, deadline):
            return SpeedCorrectionResult(False, False,
                f"nothing arrived on verify port {port}, so there was no "
                f"peer address to send {model.SPEEDUP_PARAM} to"
            )
        last_send: float | None = None
        while time.monotonic() < deadline and len(confirmed) < len(expected):
            if last_send is None or time.monotonic() - last_send >= resend_interval:
                write_attempted = True
                _send_speedup(connection, expected - set(confirmed), speedup)
                last_send = time.monotonic()
            message = connection.recv_match(blocking=True, timeout=1.0)
            if message is None or message.get_type() != "PARAM_VALUE":
                continue
            source = message.get_srcSystem()
            if (
                source in expected
                and model.param_name(message) == model.SPEEDUP_PARAM
                and model.speedup_matches(float(message.param_value), speedup)
            ):
                confirmed[source] = float(message.param_value)
    except OSError as exc:
        return SpeedCorrectionResult(write_attempted, False, f"heal socket error ({exc})")
    finally:
        connection.close()
    missing = sorted(expected - set(confirmed))
    if missing:
        return SpeedCorrectionResult(write_attempted, False,
            f"sys_ids {missing} did not acknowledge "
            f"{model.SPEEDUP_PARAM}={speedup:g}"
        )
    return SpeedCorrectionResult(write_attempted, True,
        f"all {len(confirmed)} instances acknowledged "
        f"{model.SPEEDUP_PARAM}={speedup:g}"
    )


def _send_speedup(connection: Any, sysids: set[int], speedup: float) -> None:
    for sysid in sorted(sysids):
        connection.mav.param_set_send(
            sysid,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            model.SPEEDUP_PARAM.encode(),
            float(speedup),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )


def speed_is_fault(
    result: model.VerificationResult,
    expected: set[int],
    speedup: float,
) -> bool:
    """Return whether a live swarm's reported or measured speed is wrong."""
    if result.error is not None or not expected.issubset(result.heartbeats):
        return False
    reported_wrong = any(
        sysid in result.speedups
        and not model.speedup_matches(result.speedups[sysid], speedup)
        for sysid in expected
    )
    measured_wrong = any(
        sysid in result.rates
        and not model.rate_matches(result.rates[sysid], speedup)
        for sysid in expected
    )
    return reported_wrong or measured_wrong


def describe_failure(
    result: model.VerificationResult,
    expected: set[int],
    speedup: float,
) -> str:
    """Return one operator-facing line naming the failed verification gate."""
    if result.error:
        return result.error
    missing = sorted(expected - set(result.heartbeats))
    if missing:
        return f"sys_ids {missing} never streamed"
    silent = sorted(expected - set(result.speedups))
    if silent:
        return f"sys_ids {silent} never answered {model.SPEEDUP_PARAM}"
    wrong = {
        sysid: value
        for sysid, value in sorted(result.speedups.items())
        if not model.speedup_matches(value, speedup)
    }
    if wrong:
        values = ", ".join(
            f"sys_id {sysid}={value:g}" for sysid, value in wrong.items()
        )
        return f"{model.SPEEDUP_PARAM} mismatch (requested {speedup:g}): {values}"
    blind = sorted(expected - set(result.boot_evidence))
    if blind:
        return f"sys_ids {blind} never reported {model.SYSTEM_TIME_MSG}"
    rate_error = _rate_failure(result, expected, speedup)
    if rate_error is not None:
        return rate_error
    if result.stale:
        values = ", ".join(
            f"sys_id {sysid} time_boot_ms={value:.0f}"
            for sysid, value in sorted(result.stale.items())
        )
        return f"not booted by this launch (survivor of an earlier swarm): {values}"
    return "unknown verification failure"


def _rate_failure(
    result: model.VerificationResult,
    expected: set[int],
    speedup: float,
) -> str | None:
    unmeasured = sorted(expected - set(result.rates))
    if unmeasured:
        return (
            f"sys_ids {unmeasured} produced no usable clock-rate measurement "
            f"(need boot-clock samples spanning {model.CLOCK_MIN_SPAN_S:g}s)"
        )
    off = {
        sysid: value
        for sysid, value in sorted(result.rates.items())
        if not model.rate_matches(value, speedup)
    }
    if not off:
        return None
    values = ", ".join(
        f"sys_id {sysid}={value:.2f}x" for sysid, value in off.items()
    )
    return (
        f"MEASURED clock rate disagrees with requested {speedup:g}x: {values} "
        f"(tolerance {model.CLOCK_RATE_TOLERANCE:.0%})"
    )


def verification_passed(
    result: model.VerificationResult,
    expected: set[int],
    speedup: float,
) -> bool:
    """Return whether all four verification gates passed for every vehicle."""
    return bool(
        result.error is None
        and expected.issubset(result.heartbeats)
        and expected.issubset(result.speedups)
        and all(
            model.speedup_matches(result.speedups[sysid], speedup)
            for sysid in expected
        )
        and expected.issubset(result.boot_evidence)
        and not result.stale
        and expected.issubset(result.rates)
        and all(
            model.rate_matches(result.rates[sysid], speedup)
            for sysid in expected
        )
    )
