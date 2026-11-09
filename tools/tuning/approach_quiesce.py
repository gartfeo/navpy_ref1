"""Observation and shutdown of one approach run (hitl_final_approach.md section 9, step 16).

Observation: navpy is the router's reader client (mav_router.py --reader): while the harness flies the fixed search
lane, navpy may read and report but none of its writes to the aircraft reaches the board (observation_allowed).

Quiesce: one idempotent step, the same for every end of the run: navpy's terminal record, the scenario's deadline,
a crash (a role exited, a runner error), an abort (Ctrl+C, SIGTERM) and a floor breach. Its first call stops navpy
and closes navpy's router endpoint (NavpyHandoff, navpy_handoff.py); later calls return the same record and only add
their reason. The runner quiesces before any RTL of its own and before the board's stop.

After the run, audit() reads the router's write log at the board boundary: navpy's writes outside the observation
allow list, and navpy's writes after the first drain ack (navpy's own barrier or the runner's), must both be 0.
outcome() takes the outcome only from a navpy terminal record the runner saw before its deadline; a deadline, a
stop or a crash never yields DOCKED.
"""
from __future__ import annotations

import signal
import time
from collections import Counter
from pathlib import Path
from typing import Callable, Optional

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from navpy.app.flow.confirm_record import parse_confirm_record
from navpy.app.router_barrier import RouterBarrier
from navpy.app.terminal_handoff import TRIGGER_TERMINAL, parse_terminal_record
from tools.tuning import approach_chain as chain
from tools.tuning.approach_procs import ProcessGroup
from tools.tuning.board import DEFAULT_BAUD
from tools.tuning.mav_router_policy import COMMAND_MESSAGES, OBSERVATION_COMMANDS, OBSERVATION_MESSAGES
from tools.tuning.navpy_handoff import NavpyHandoff, direct_rtl, end_process, stop_navpy
from tools.tuning.vehicle import Vehicle

TERMINAL_POLL_S = 1.0           # one HEARTBEAT wait per check of navpy's output and the owned processes
NAVPY = "navpy"
ENDED_BY_TERMINAL, ENDED_BY_DEADLINE = "terminal record", "deadline"
# The reasons a quiesce is asked for (the first one is the run's; EXIT is the runner's last, always a repeat after a
# clean flight).
TERMINAL, DEADLINE, CRASH, ABORT, FLOOR, EXIT = "terminal", "deadline", "crash", "abort", "floor", "exit"
GIMBAL_STOP_OK = ("sent", "no_gimbal")
_ALLOWED_IDS = frozenset(cls.id for cls in mavlink2.mavlink_map.values() if cls.msgname in OBSERVATION_MESSAGES)
_COMMAND_IDS = frozenset(cls.id for cls in mavlink2.mavlink_map.values() if cls.msgname in COMMAND_MESSAGES)


# SIGHUP (POSIX: the WSL session or terminal went away) would otherwise end the runner at once, with no stop at all.
ABORT_SIGNALS = tuple(getattr(signal, name) for name in ("SIGTERM", "SIGHUP") if hasattr(signal, name))


class Aborted(BaseException):
    """SIGTERM or SIGHUP during a run: ends it like Ctrl+C (an abort), so the quiesce and every stop still run."""


def raise_aborted(signum: int, frame: object) -> None:
    """The runner's SIGTERM and SIGHUP handler while a run is in progress."""
    raise Aborted(f"signal {signum}")


def _install(handler: object, signums: tuple, saved: dict) -> None:
    """Each signal's first handler goes into saved before it is replaced, so an interruption loses none."""
    for signum in signums:
        saved.setdefault(signum, signal.getsignal(signum))
        signal.signal(signum, handler)


def arm(saved: dict) -> None:
    """While a run is in progress SIGTERM and SIGHUP raise Aborted; called inside the run's try, so a signal even
    during arm() is an abort."""
    _install(raise_aborted, ABORT_SIGNALS, saved)


def shield(saved: dict) -> None:
    """Ignore Ctrl+C, SIGTERM and SIGHUP while the run's cleanup runs, so a second signal cannot cut its stops short.
    The signals are blocked meanwhile (POSIX); one that was already on its way when shield() began is retried past,
    and once all are ignored none can raise (CPython runs no handler for an ignored signal). The caller retries the
    call itself too, for a signal at its very start."""
    mask = None
    while True:
        try:
            signums, block = (signal.SIGINT, *ABORT_SIGNALS), getattr(signal, "pthread_sigmask", None)
            if block:
                held = block(signal.SIG_BLOCK, signums)
                mask = held if mask is None else mask
            _install(signal.SIG_IGN, signums, saved)
            break
        except (Aborted, KeyboardInterrupt):
            continue
    if mask is not None:
        block(signal.SIG_SETMASK, mask)         # a signal held meanwhile now meets SIG_IGN


def restore(saved: dict) -> None:
    for signum, handler in saved.items():
        signal.signal(signum, handler)


def _handed_off(group: ProcessGroup, role: str, result: dict, clean: Optional[str]) -> dict:
    """The handoff stopped this owned role itself: recorded as stopped, ended after stop only by its own stop."""
    group.mark_stopped(role, result.get("returncode") if clean is not None and result.get("stop") == clean else None)
    return result


class Quiesce:
    """The run's one quiesce (module notes). run(reason) returns NavpyHandoff's record; it never raises."""

    def __init__(self, handoff: NavpyHandoff) -> None:
        self._handoff = handoff
        self.reasons: list[str] = []

    @classmethod
    def of(cls, group: ProcessGroup) -> Quiesce:
        """navpy's stop, the barrier on the router's control port, and the no-ack path, for this group's roles."""
        return cls(NavpyHandoff(
            lambda: _handed_off(group, NAVPY, stop_navpy(group.handle(NAVPY).process), "requested"),
            RouterBarrier(("127.0.0.1", chain.CONTROL_PORT), NAVPY),
            lambda: _handed_off(group, "router", end_process(group.handle("router").process), None),
            lambda: direct_rtl(f"tcp:127.0.0.1:{chain.SITL_PORT}", DEFAULT_BAUD)))

    @property
    def record(self) -> Optional[dict]:
        return self._handoff.record

    def end(self, group: ProcessGroup, reason: str) -> Optional[dict]:
        """The runner's last quiesce, before the board's stop: only once navpy has been started."""
        if not any(owned.spec.role == NAVPY for owned in group.owned):
            return None
        return self.run(reason)

    def run(self, reason: str) -> dict:
        self.reasons.append(reason)
        try:
            return self._handoff.before_rtl()
        except Exception as exc:     # noqa: BLE001 - NavpyHandoff records its own errors; this is a last guard
            return {"error": f"{type(exc).__name__}: {exc}", "failed": "quiesce raised"}

    def describe(self) -> dict:
        return {"reason": self.reasons[0] if self.reasons else None, "repeats": self.reasons[1:],
                "handoff": self.record}


def observe(board: Vehicle, group: ProcessGroup, navpy_out: Path, seconds: float,
            on_confirm: Optional[Callable[[dict], None]] = None) -> str:
    """Drain the link (the floor's hook sees every position) until the deadline, or until navpy's complete terminal
    record is read before it: a record read after the deadline, or a line not yet ended, never ends the observation.
    Each complete confirm record (navpy's confirm releasing NAV, step 23b) goes to on_confirm (the floor's release)."""
    deadline, partial = time.monotonic() + seconds, ""
    with open(navpy_out, encoding="utf-8", errors="replace") as out:
        while time.monotonic() < deadline:
            board.wait_message("HEARTBEAT", TERMINAL_POLL_S)
            group.check()
            lines = (partial + out.read()).split("\n")
            partial = lines.pop()
            for confirm in filter(None, map(parse_confirm_record, lines)):
                if on_confirm is not None:
                    on_confirm(confirm)
            if any(parse_terminal_record(line) for line in lines) and time.monotonic() < deadline:
                return ENDED_BY_TERMINAL
    return ENDED_BY_DEADLINE


def navpy_records(navpy_out: Path) -> list[dict]:
    """Every NAVPY_TERMINAL_V1 record navpy printed (at most one: its handoff runs once)."""
    if not navpy_out.exists():
        return []
    lines = navpy_out.read_text(encoding="utf-8", errors="replace").splitlines()
    return [record for record in map(parse_terminal_record, lines) if record is not None]


def outcome(records: list[dict], ended_by: Optional[str]) -> dict:
    """The run's outcome: navpy's terminal outcome only when its record ended the observation; else None."""
    first = records[0] if records else None
    counted = (first is not None and ended_by == ENDED_BY_TERMINAL and first.get("trigger", TRIGGER_TERMINAL)
               == TRIGGER_TERMINAL)
    problems = []
    if first is not None and first.get("barrier") != "acked":
        problems.append(f"navpy's own barrier failed: {first.get('barrier_error')}")
    if first is not None and first.get("gimbal_stop") not in GIMBAL_STOP_OK:
        problems.append(f"navpy's gimbal stop: {first.get('gimbal_stop')}")
    return {"outcome": first.get("outcome") if counted else None, "ended_by": ended_by,
            "navpy_record": first, "problems": problems}


def _drain_acks(handoff: Optional[dict], records: list[dict]) -> list[float]:
    acks = [((handoff or {}).get("barrier") or {}).get("ack")] + [r.get("router_ack") for r in records]
    return [float(ack["barrier_mono_s"]) for ack in acks if isinstance(ack, dict) and "barrier_mono_s" in ack]


def allowed(msg_id: int, command: Optional[int]) -> bool:
    """The write log's view of mav_router_policy.observation_allowed."""
    if msg_id in _COMMAND_IDS:
        return command in OBSERVATION_COMMANDS
    return msg_id in _ALLOWED_IDS


def audit(write_log: Path, handoff: Optional[dict], records: list[dict], command_mode: bool = False) -> dict:
    """navpy's writes at the board boundary (router monotonic seconds): those after the drain and, in observation,
    forbidden ones. In command mode (step 27) navpy's commands reach the board by design until the drain ack; none
    after it, as in observation."""
    acks = _drain_acks(handoff, records)
    drain = min(acks) if acks else None
    writes, forbidden, after = 0, Counter(), 0
    if write_log.exists():
        for line in write_log.read_text(encoding="utf-8").splitlines():
            stamp, source, msg_id, command = (line.split(",") + [""])[:4]
            if source != NAVPY:
                continue
            writes += 1
            cmd = int(command) if command else None
            if not command_mode and not allowed(int(msg_id), cmd):
                forbidden[f"{msg_id}/{command}" if command else msg_id] += 1
            if drain is not None and float(stamp) > drain:
                after += 1
    ok = write_log.exists() and not forbidden and drain is not None and after == 0
    return {"authority": "command" if command_mode else "observation", "write_log": write_log.exists(),
            "navpy_writes": writes, "forbidden": sum(forbidden.values()),
            "forbidden_ids": dict(forbidden), "drain_ack_mono_s": drain,
            "after_drain": None if drain is None else after, "ok": ok}
