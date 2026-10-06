"""Launch and supervise an isolated chat-scoped ArduPilot SITL swarm."""

from __future__ import annotations

import atexit
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Sequence


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(SCRIPTS))

from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402
from gcs.backend.instance_registry import SitlSupervisorActive  # noqa: E402

import swarm_run_process as process_io  # noqa: E402
import swarm_run_relay as relay_io  # noqa: E402
import swarm_run_runner as runner  # noqa: E402
import swarm_run_speed_control as speed_control  # noqa: E402
import swarm_run_verification_model as verify_model  # noqa: E402
import swarm_run_verifier as verifier  # noqa: E402
import swarm_run_wsl as wsl  # noqa: E402


_WRITE_LOCK = relay_io.WRITE_LOCK
_RELAY_CHUNK = relay_io.RELAY_CHUNK
_MAX_PENDING_RECORD = relay_io.MAX_PENDING_RECORD
_Relay = relay_io.Relay
WSL_BASH_ARGV = wsl.WSL_BASH_ARGV

VERIFY_TIMEOUT = verify_model.VERIFY_TIMEOUT
PARAM_TIMEOUT = verify_model.PARAM_TIMEOUT
PARAM_RESEND_INTERVAL = verify_model.PARAM_RESEND_INTERVAL
BOOT_EVIDENCE_TIMEOUT = verify_model.BOOT_EVIDENCE_TIMEOUT
BOOT_REQUEST_INTERVAL = verify_model.BOOT_REQUEST_INTERVAL
BOOT_WALL_GRACE = verify_model.BOOT_WALL_GRACE
BOOT_SIM_SLACK_MS = verify_model.BOOT_SIM_SLACK_MS
SYSTEM_TIME_MSG_ID = verify_model.SYSTEM_TIME_MSG_ID
SYSTEM_TIME_MSG = verify_model.SYSTEM_TIME_MSG
BOOT_TIME_MESSAGES = verify_model.BOOT_TIME_MESSAGES
CLOCK_RATE_TOLERANCE = verify_model.CLOCK_RATE_TOLERANCE
CLOCK_MIN_SPAN_S = verify_model.CLOCK_MIN_SPAN_S
SPEEDUP_PARAM = verify_model.SPEEDUP_PARAM
HEAL_TIMEOUT = speed_control.HEAL_TIMEOUT
HEAL_RESEND_INTERVAL = speed_control.HEAL_RESEND_INTERVAL
MAX_LAUNCH_ATTEMPTS = 3
RELAY_DRAIN_TIMEOUT = 5.0
DEFAULT_SPEEDUP = 10
VerificationResult = verify_model.VerificationResult


_is_regular_file = relay_io.is_regular_file
_write_locked = relay_io.write_locked
_write_all = relay_io.write_all
_emit = relay_io.emit
_relay_write = relay_io.relay_write
_retire = relay_io.retire
_wsl_argv = wsl.wsl_argv
ensure_fork_supports_companion_udp = wsl.ensure_fork_supports_companion_udp
_sysid_process_pattern = wsl.sysid_process_pattern
_router_process_pattern = wsl.router_process_pattern
_param_name = verify_model.param_name
_boot_ms = verify_model.boot_ms
_boot_limit_ms = verify_model.boot_limit_ms
_drain = verify_model.drain
_stale_instances = verify_model.stale_instances
_measured_rates = verify_model.measured_rates
_speedup_matches = verify_model.speedup_matches
_rate_matches = verify_model.rate_matches
_await_peer = speed_control.await_peer
_launch_command = wsl.launch_command
_speed_is_the_fault = speed_control.speed_is_fault
_describe_failure = speed_control.describe_failure
_verification_passed = speed_control.verification_passed


def _relay_stream(relay: _Relay) -> None:
    relay_io.relay_stream(relay, write=_relay_write)


def _resolve_chat(explicit: int | None, eval_mode: bool = False) -> int:
    return wsl.resolve_chat(
        explicit,
        eval_mode=eval_mode,
        root=ROOT,
        registry=reg,
    )


def _wsl(command: str) -> None:
    subprocess.run(_wsl_argv(command), capture_output=True)


def cleanup(chat: int) -> None:
    wsl.cleanup(chat, wsl_run=_wsl)


def _verify_swarm(
    chat: int,
    instances: int,
    speedup: float,
    launched_at: float,
    timeout: float = VERIFY_TIMEOUT,
    param_timeout: float = PARAM_TIMEOUT,
    boot_timeout: float = BOOT_EVIDENCE_TIMEOUT,
    min_span: float = CLOCK_MIN_SPAN_S,
) -> VerificationResult:
    return verifier.verify_swarm(
        chat,
        instances,
        speedup,
        launched_at,
        timeout,
        param_timeout,
        boot_timeout,
        min_span,
        drain=_drain,
        param_resend_interval=PARAM_RESEND_INTERVAL,
        boot_request_interval=BOOT_REQUEST_INTERVAL,
    )


def _force_speedup(
    chat: int,
    expected: set[int],
    speedup: float,
    timeout: float = HEAL_TIMEOUT,
) -> speed_control.SpeedCorrectionResult:
    return speed_control.force_speedup(
        chat,
        expected,
        speedup,
        timeout,
        resend_interval=HEAL_RESEND_INTERVAL,
    )


def _start_launcher(command: str) -> tuple[subprocess.Popen[bytes], list[_Relay]]:
    return process_io.start_launcher(
        command,
        is_regular_file=_is_regular_file,
        wsl_argv=_wsl_argv,
        relay_stream=_relay_stream,
        retire=_retire,
    )


def _relay_warning(relay: _Relay, timed_out: bool, *, reaped: bool) -> str | None:
    return process_io.relay_warning(
        relay,
        timed_out,
        reaped=reaped,
        drain_timeout=RELAY_DRAIN_TIMEOUT,
    )


def _warn_about_relay(
    chat: int, relay: _Relay, timed_out: bool, *, reaped: bool
) -> None:
    process_io.warn_about_relay(
        chat,
        relay,
        timed_out,
        reaped=reaped,
        warning=_relay_warning,
        emit=_emit,
    )


def _retire_relays(
    chat: int, relays: Sequence[_Relay], *, reaped: bool
) -> None:
    process_io.retire_relays(
        chat,
        relays,
        reaped=reaped,
        drain_timeout=RELAY_DRAIN_TIMEOUT,
        retire=_retire,
        warn=_warn_about_relay,
    )


def _kill_launch(
    process: subprocess.Popen[bytes],
    chat: int,
    relays: Sequence[_Relay] = (),
) -> None:
    process_io.kill_launch(
        process,
        chat,
        list(relays),
        cleanup=cleanup,
        retire_relays=_retire_relays,
    )


def _record_status(
    chat: int,
    speedup: float,
    *,
    verified: bool,
    error: str | None = None,
    measured_rates: dict[int, float] | None = None,
) -> None:
    try:
        outcome = reg.record_sitl_status(
            chat,
            speedup=speedup,
            verified=verified,
            error=error,
            sitl_pid=os.getpid(),
            measured_rates=measured_rates,
        )
        if outcome != "recorded":
            _emit(
                f"SITL chat {chat}: launch status not recorded ({outcome})",
                err=True,
            )
    except Exception as exc:
        _emit(
            f"SITL chat {chat}: could not record launch status ({exc})",
            err=True,
        )


def _retract_sitl(chat: int) -> None:
    try:
        outcome = reg.retract_sitl(chat, sitl_pid=os.getpid())
        _emit(f"SITL chat {chat}: registry retract={outcome}", err=True)
    except Exception as exc:
        _emit(
            f"SITL chat {chat}: could not retract registry SITL state ({exc})",
            err=True,
        )


def main() -> None:
    args = runner.build_parser(DEFAULT_SPEEDUP).parse_args()
    wsl.ensure_experimental_paths(args.sitl_root, args.sitl_binary)
    ensure_fork_supports_companion_udp()
    if args.defaults is not None:
        # `is not None`, not truthiness: --defaults "" would otherwise be
        # accepted and then silently ignored by launch_command.
        wsl.ensure_defaults_file(args.defaults)
    chat = _resolve_chat(args.chat, eval_mode=args.eval)
    try:
        reg.begin_sitl_launch(
            chat,
            supervisor_pid=os.getpid(),
            launch_token=args.launch_token,
        )
    except SitlSupervisorActive as exc:
        sys.exit(
            f"swarm_run: {exc}. Stop it first: "
            f"python scripts/gcs_stop.py --chat {chat}"
        )
    except Exception as exc:
        print(
            f"SITL chat {chat}: could not record launch ownership ({exc})",
            file=sys.stderr,
            flush=True,
        )
    launch = runner.LaunchServices(
        _start_launcher,
        _verify_swarm,
        _verification_passed,
        _describe_failure,
        _speed_is_the_fault,
        _force_speedup,
        _kill_launch,
    )
    runtime = runner.RuntimeServices(
        reg,
        atexit,
        cleanup,
        _emit,
        _record_status,
        _retract_sitl,
        _retire_relays,
        _launch_command,
    )
    runner.run_supervisor(
        chat,
        args,
        launch=launch,
        runtime=runtime,
        max_attempts=1 if args.single_boot else MAX_LAUNCH_ATTEMPTS,
    )


if __name__ == "__main__":
    main()
