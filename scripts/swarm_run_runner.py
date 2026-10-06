"""Retry orchestration for one chat-scoped SITL swarm supervisor."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from gcs.backend import instance_ports as ip

import swarm_run_relay as relay_io
import swarm_run_verification_model as model
from swarm_run_speed_control import SpeedCorrectionResult


@dataclass(frozen=True)
class LaunchServices:
    """Operations whose behavior is patched independently in launch tests."""

    start: Callable[[str], tuple[Any, list[relay_io.Relay]]]
    verify: Callable[[int, int, float, float], model.VerificationResult]
    passed: Callable[[model.VerificationResult, set[int], float], bool]
    describe: Callable[[model.VerificationResult, set[int], float], str]
    speed_fault: Callable[[model.VerificationResult, set[int], float], bool]
    force_speed: Callable[[int, set[int], float], SpeedCorrectionResult]
    kill: Callable[[Any, int, list[relay_io.Relay]], None]


@dataclass(frozen=True)
class RuntimeServices:
    """Registry, teardown, logging, and command-boundary operations."""

    registry: Any
    exit_hooks: Any
    cleanup: Callable[[int], None]
    emit: Callable[..., None]
    record_status: Callable[..., None]
    retract: Callable[[int], None]
    retire_relays: Callable[..., None]
    launch_command: Callable[..., str]


@dataclass
class LaunchOutcome:
    """Terminal result of the bounded verification/relaunch loop."""

    process: Any | None = None
    relays: list[relay_io.Relay] = field(default_factory=list)
    reason: str = "no launch attempted"


from swarm_run_cli import _positive_speedup, _home_coords, build_parser


def run_supervisor(
    chat: int,
    args: argparse.Namespace,
    *,
    launch: LaunchServices,
    runtime: RuntimeServices,
    max_attempts: int,
) -> None:
    """Launch, verify, supervise, and propagate the WSL process exit code."""
    offset = ip.VEHICLES_PER_CHAT * chat
    win_ports = ",".join(
        str(port)
        for port in ip.router_win_ports(chat) + [ip.verify_port(chat)]
    )
    runtime.exit_hooks.register(runtime.cleanup, chat)
    runtime.cleanup(chat)
    runtime.emit(
        f"SITL chat {chat}: sys_ids {ip.sysids_for_chat(chat)}, "
        f"SWARM_OFFSET={offset}, ROUTER_WIN_PORTS={win_ports}"
    )
    expected = set(ip.sysids_for_chat(chat)[: args.instances])
    with runtime.registry.sitl_launch_lock() as got_lock:
        runtime.emit(
            f"SITL chat {chat}: launch lock "
            + (
                "acquired - serializing init"
                if got_lock
                else "busy, proceeding unserialized"
            )
        )
        outcome = _launch_with_retries(
            chat,
            args,
            offset=offset,
            win_ports=win_ports,
            expected=expected,
            launch=launch,
            runtime=runtime,
            max_attempts=max_attempts,
        )
        if outcome.process is None:
            _fail_terminally(
                chat,
                args,
                outcome.reason,
                runtime=runtime,
                max_attempts=max_attempts,
            )
    code = outcome.process.wait()
    runtime.retire_relays(chat, outcome.relays, reaped=True)
    sys.exit(code)


def _launch_with_retries(
    chat: int,
    args: argparse.Namespace,
    *,
    offset: int,
    win_ports: str,
    expected: set[int],
    launch: LaunchServices,
    runtime: RuntimeServices,
    max_attempts: int,
) -> LaunchOutcome:
    outcome = LaunchOutcome()
    possible_write = False
    for attempt in range(1, max_attempts + 1):
        launched_at = time.monotonic()
        command = runtime.launch_command(
            args.speedup,
            offset,
            win_ports,
            args.instances,
            args.dist,
            keep_instance_state=possible_write,
            home_coords=getattr(args, "home", None),
            defaults_path=getattr(args, "defaults", None),
            firmware_root=getattr(args, "sitl_root", None),
            binary_path=getattr(args, "sitl_binary", None),
        )
        process, relays = launch.start(command)
        result = launch.verify(chat, args.instances, args.speedup, launched_at)
        if launch.passed(result, expected, args.speedup):
            _report_success(
                chat,
                args.speedup,
                attempt,
                expected,
                result,
                runtime=runtime,
            )
            return LaunchOutcome(process=process, relays=relays)
        outcome.reason = launch.describe(result, expected, args.speedup)
        if (
            not getattr(args, "single_boot", False)
            and launch.speed_fault(result, expected, args.speedup)
        ):
            if attempt == max_attempts:
                runtime.emit(
                    f"SITL chat {chat}: no further correction attempted: no relaunch remains",
                    err=True,
                )
            elif not possible_write:
                correction = _heal_live_swarm(
                    chat,
                    args.speedup,
                    expected,
                    outcome.reason,
                    launch=launch,
                    runtime=runtime,
                )
                possible_write = correction.write_attempted
        runtime.emit(
            f"SITL chat {chat}: attempt {attempt}/{max_attempts}: "
            f"{outcome.reason} - "
            + ("stopping swarm" if attempt == max_attempts else "restarting swarm")
        )
        launch.kill(process, chat, relays)
        if attempt < max_attempts:
            time.sleep(2.0)
    return outcome


def _report_success(
    chat: int,
    speedup: float,
    attempt: int,
    expected: set[int],
    result: model.VerificationResult,
    *,
    runtime: RuntimeServices,
) -> None:
    measured = ", ".join(
        f"{sysid}={result.rates[sysid]:.2f}x" for sysid in sorted(expected)
    )
    runtime.emit(
        f"SITL chat {chat}: all {len(expected)} instances streaming telemetry "
        f"at {model.SPEEDUP_PARAM}={speedup:g} (attempt {attempt}), "
        f"measured clock {measured} - releasing launch lock"
    )
    runtime.record_status(
        chat,
        speedup,
        verified=True,
        measured_rates=dict(result.rates),
    )


def _heal_live_swarm(
    chat: int,
    speedup: float,
    expected: set[int],
    reason: str,
    *,
    launch: LaunchServices,
    runtime: RuntimeServices,
) -> SpeedCorrectionResult:
    result = launch.force_speed(chat, expected, speedup)
    if not result.write_attempted:
        message = "could NOT send SIM_SPEEDUP; next launch uses default instance setup"
    elif not result.all_echoes_matched:
        message = (
            "PARAM_SET attempted; not all expected echoes matched; "
            "preserving instance state for next verification"
        )
    else:
        message = "matching echoes observed; preserving instance state for next verification"
    runtime.emit(
        f"SITL chat {chat}: {reason} - {message} ({result.detail})",
        err=not (result.write_attempted and result.all_echoes_matched),
    )
    return result


def _fail_terminally(
    chat: int,
    args: argparse.Namespace,
    reason: str,
    *,
    runtime: RuntimeServices,
    max_attempts: int,
) -> None:
    runtime.emit(
        f"SITL chat {chat}: FAILED to bring up a verified swarm after "
        f"{max_attempts} attempts: {reason}",
        err=True,
    )
    runtime.record_status(chat, args.speedup, verified=False, error=reason)
    runtime.cleanup(chat)
    runtime.exit_hooks.unregister(runtime.cleanup)
    runtime.retract(chat)
    sys.exit(1)
