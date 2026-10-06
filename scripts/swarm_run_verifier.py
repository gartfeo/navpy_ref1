"""Three-phase swarm launch verification."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable

from gcs.backend import instance_ports as ip
from pymavlink import mavutil

import swarm_run_verification_model as model


@dataclass
class VerificationEvidence:
    """Mutable evidence accumulated across one verification socket."""

    heartbeats: set[int] = field(default_factory=set)
    speedups: dict[int, float] = field(default_factory=dict)
    boot_evidence: set[int] = field(default_factory=set)
    first_sample: dict[int, tuple[float, float]] = field(default_factory=dict)
    last_sample: dict[int, tuple[float, float]] = field(default_factory=dict)
    worst_sample: dict[int, tuple[float, float]] = field(default_factory=dict)
    stale: dict[int, float] = field(default_factory=dict)


def verify_swarm(
    chat: int,
    instances: int,
    speedup: float,
    launched_at: float,
    timeout: float = model.VERIFY_TIMEOUT,
    param_timeout: float = model.PARAM_TIMEOUT,
    boot_timeout: float = model.BOOT_EVIDENCE_TIMEOUT,
    min_span: float = model.CLOCK_MIN_SPAN_S,
    *,
    drain: Callable[[Any], None] = model.drain,
    param_resend_interval: float = model.PARAM_RESEND_INTERVAL,
    boot_request_interval: float = model.BOOT_REQUEST_INTERVAL,
) -> model.VerificationResult:
    """Prove liveness, configured speed, provenance, and measured clock rate."""
    expected = set(ip.sysids_for_chat(chat)[:instances])
    port = ip.verify_port(chat)
    try:
        connection = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
    except OSError as exc:
        return model.VerificationResult(
            frozenset(),
            {},
            error=f"could not bind verify port {port} ({exc})",
        )
    evidence = VerificationEvidence()
    try:
        _collect_heartbeats(
            connection,
            expected=expected,
            seen=evidence.heartbeats,
            timeout=timeout,
        )
        if not expected.issubset(evidence.heartbeats):
            return _result(evidence, speedup, launched_at, min_span)
        drain(connection)
        _collect_speedups(
            connection,
            expected=expected,
            values=evidence.speedups,
            timeout=param_timeout,
            resend_interval=param_resend_interval,
        )
        if not _configuration_matches(evidence.speedups, expected, speedup):
            return _result(evidence, speedup, launched_at, min_span)
        drain(connection)
        _collect_boot_samples(
            connection,
            expected=expected,
            evidence=evidence,
            timeout=boot_timeout,
            request_interval=boot_request_interval,
            speedup=speedup,
            launched_at=launched_at,
        )
    except OSError as exc:
        return _result(
            evidence,
            speedup,
            launched_at,
            min_span,
            error=f"verify socket error ({exc})",
            include_stale=False,
        )
    finally:
        connection.close()
    return _result(evidence, speedup, launched_at, min_span)


def _collect_heartbeats(
    connection: Any,
    *,
    expected: set[int],
    seen: set[int],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not expected.issubset(seen):
        message = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=1.0,
        )
        if message is not None and message.get_srcSystem() in expected:
            seen.add(message.get_srcSystem())


def _collect_speedups(
    connection: Any,
    *,
    expected: set[int],
    values: dict[int, float],
    timeout: float,
    resend_interval: float,
) -> None:
    deadline = time.monotonic() + timeout
    last_send: float | None = None
    while time.monotonic() < deadline and len(values) < len(expected):
        if last_send is None or time.monotonic() - last_send >= resend_interval:
            _request_speedups(connection, expected - set(values))
            last_send = time.monotonic()
        message = connection.recv_match(blocking=True, timeout=1.0)
        if message is None or message.get_type() != "PARAM_VALUE":
            continue
        source = message.get_srcSystem()
        if (
            source in expected
            and source not in values
            and model.param_name(message) == model.SPEEDUP_PARAM
        ):
            values[source] = float(message.param_value)


def _request_speedups(connection: Any, sysids: set[int]) -> None:
    for sysid in sorted(sysids):
        connection.mav.param_request_read_send(
            sysid,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            model.SPEEDUP_PARAM.encode(),
            -1,
        )


def _configuration_matches(
    values: dict[int, float],
    expected: set[int],
    speedup: float,
) -> bool:
    return expected.issubset(values) and all(
        model.speedup_matches(values[sysid], speedup) for sysid in expected
    )


def _collect_boot_samples(
    connection: Any,
    *,
    expected: set[int],
    evidence: VerificationEvidence,
    timeout: float,
    request_interval: float,
    speedup: float,
    launched_at: float,
) -> None:
    deadline = time.monotonic() + timeout
    last_send: float | None = None
    while time.monotonic() < deadline:
        if last_send is None or time.monotonic() - last_send >= request_interval:
            _request_boot_clocks(connection, expected)
            last_send = time.monotonic()
        message = connection.recv_match(blocking=True, timeout=1.0)
        if not _is_autopilot_boot_message(message, expected):
            continue
        boot = model.boot_ms(message)
        if boot is None:
            continue
        source = message.get_srcSystem()
        now = time.monotonic()
        evidence.first_sample.setdefault(source, (now, boot))
        evidence.last_sample[source] = (now, boot)
        if boot > model.boot_limit_ms(speedup, launched_at, now):
            evidence.stale[source] = max(evidence.stale.get(source, 0.0), boot)
        if boot > evidence.worst_sample.get(source, (-1.0, 0.0))[0]:
            evidence.worst_sample[source] = (boot, now)
        if message.get_type() == model.SYSTEM_TIME_MSG:
            evidence.boot_evidence.add(source)


def _request_boot_clocks(connection: Any, expected: set[int]) -> None:
    for sysid in sorted(expected):
        connection.mav.command_long_send(
            sysid,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            model.SYSTEM_TIME_MSG_ID,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def _is_autopilot_boot_message(message: Any, expected: set[int]) -> bool:
    return bool(
        message is not None
        and message.get_type() in model.BOOT_TIME_MESSAGES
        and message.get_srcSystem() in expected
        and message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    )


def _result(
    evidence: VerificationEvidence,
    speedup: float,
    launched_at: float,
    min_span: float,
    *,
    error: str | None = None,
    include_stale: bool = True,
) -> model.VerificationResult:
    stale = (
        dict(evidence.stale)
        if include_stale
        else {}
    )
    return model.VerificationResult(
        frozenset(evidence.heartbeats),
        dict(evidence.speedups),
        error=error,
        boot_evidence=frozenset(evidence.boot_evidence),
        stale=stale,
        rates=model.measured_rates(
            evidence.first_sample,
            evidence.last_sample,
            min_span,
        ),
    )
