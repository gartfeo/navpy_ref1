"""The HITL terminal handoff (docs/design/hitl_final_approach.md sections 2-3; only with --router-control).

On the approach's first terminal outcome (DOCKED, MISSED or FAILED: episode_latch.py) navpy
1. fences its MAVLink link: every later write is dropped and counted (LinkFence wraps the connection's ``write``,
   where every pymavlink send ends), and its SIYI gimbal link: every later command except queries and stops is
   dropped and counted (SiyiCommandFence wraps the SDK's ``sendMsg``); then waits until no write is in progress and,
   bounded, until no gimbal send is, and sends every gimbal one zero rate and zoom hold;
2. closes its router endpoint and waits, bounded, for the ack (router_barrier.py), so nothing it sent before the
   fence reaches the board after the record;
3. prints one flushed record, ``NAVPY_TERMINAL_V1 {json}``: trigger, outcome, task, dock, episode, aircraft time,
   fenced writes, the gimbal stop and fenced gimbal commands, and the barrier's ack, or its error (then the run counts
   as failed; a failed gimbal stop too).
Then it holds: the fences stay closed until the process stops (RESET's AUTO restart never reaches the board, and
no gimbal or zoom command reaches the gimbal).

The same quiesce, once per process, runs when navpy stops without an outcome (hitl_final_approach.md step 16): first
in its cleanup, trigger ``stop`` after a stop request (the runner's deadline or abort) and ``shutdown`` after an error
or an interrupt, so the fences close before navigation, vision and the vehicle stop and the gimbal gets its stop while
its SDK is still open. Such a record has no outcome, and an outcome latched after the quiesce began is never reported:
a deadline never becomes DOCKED. Whichever trigger comes first is the record's.
"""

from __future__ import annotations

import json
from argparse import ArgumentParser, Namespace
from collections.abc import Callable

from pymavlink import mavutil

from navpy.app.router_barrier import BARRIER_TIMEOUT_S, RouterBarrier, parse_address
from navpy.core.approach.vision_nav.episode_latch import FinalApproachTerminal, TerminalObserver
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import DockContactIdentity
from navpy.plugins.gimbal.siyi.hardware.command_fence import GIMBAL_STOP_TIMEOUT_S, SiyiCommandFence
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus, MavBusOpener

NAVPY_TERMINAL_MARKER = "NAVPY_TERMINAL_V1"
DEFAULT_ROUTER_CLIENT = "navpy"
TRIGGER_TERMINAL, TRIGGER_STOP, TRIGGER_SHUTDOWN = "terminal", "stop", "shutdown"
# The cleanup's wait for the record: the handoff's own bounds (gimbal stop, then barrier) and a second for the rest
# (the write in progress, the print). Past it the cleanup fails, navpy prints no stop ack and the runner ends it.
HANDOFF_JOIN_S = GIMBAL_STOP_TIMEOUT_S + BARRIER_TIMEOUT_S + 1.0

TerminalPublisher = Callable[[dict[str, object]], None]


class LinkFence:
    """Drops every MAVLink write once closed. close() is safe under any lock; wait_idle() takes the send lock."""

    def __init__(self) -> None:
        self._closed = rt.Event()
        self._connection: mavutil.mavfile | None = None
        self._send_lock: rt.LockPort | None = None
        self.dropped = 0

    def install(self, connection: mavutil.mavfile, send_lock: rt.LockPort) -> None:
        write = connection.write

        def fenced_write(buf: bytes) -> object:
            if self._closed.is_set():
                self.dropped += 1
                return None
            return write(buf)

        connection.write = fenced_write
        self._connection, self._send_lock = connection, send_lock

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def close(self) -> None:
        self._closed.set()

    def wait_idle(self) -> None:
        """Return once a write that passed the fence before close() has returned."""
        if self._send_lock is not None:
            with self._send_lock:
                pass

    def autopilot_uptime_s(self) -> float | None:
        """The autopilot's time since boot of the last message carrying it (pymavlink's ``uptime``)."""
        uptime = getattr(self._connection, "uptime", None)
        return float(uptime) if uptime else None


def fenced_opener(open_bus: MavBusOpener, handoff: TerminalHandoff | None) -> MavBusOpener:
    """open_bus itself without a handoff; else it also installs the handoff's fence on the bus it opens."""
    if handoff is None:
        return open_bus
    fence = handoff.fence

    def open_fenced(*args: object, **kwargs: object) -> MavBus:
        bus = open_bus(*args, **kwargs)
        fence.install(bus.conn, bus.send_lock)
        return bus

    return open_fenced


def siyi_fence(handoff: TerminalHandoff | None) -> SiyiCommandFence | None:
    return None if handoff is None else handoff.gimbal_fence


def terminal_observer(handoff: TerminalHandoff | None) -> TerminalObserver | None:
    return None if handoff is None else handoff.on_terminal


def own_quiesce(own_quiescence: Callable[[str, Callable[[], object]], None], handoff: TerminalHandoff | None,
                stop_requested: Callable[[], bool] | None) -> None:
    """Owned last, so navpy's cleanup quiesces first (module notes): RuntimeResourceStack.own_quiescence."""
    if handoff is None:
        return

    def quiesce() -> bool:
        stopped = stop_requested is not None and stop_requested()
        return handoff.finish(TRIGGER_STOP if stopped else TRIGGER_SHUTDOWN)

    own_quiescence("terminal handoff", quiesce)


def print_terminal_record(record: dict[str, object]) -> None:
    encoded = json.dumps(record, sort_keys=True, separators=(",", ":"))
    print(f"{NAVPY_TERMINAL_MARKER} {encoded}", flush=True)


def parse_terminal_record(line: str) -> dict[str, object] | None:
    prefix = f"{NAVPY_TERMINAL_MARKER} "
    if not isinstance(line, str) or not line.startswith(prefix):
        return None
    try:
        record = json.loads(line[len(prefix):])
    except ValueError:
        return None
    return record if isinstance(record, dict) else None


class TerminalHandoff:
    """Fence, barrier, record: once per process, on the first terminal outcome or quiesce (module notes)."""

    def __init__(self, fence: LinkFence, barrier: RouterBarrier,
                 publish: TerminalPublisher = print_terminal_record,
                 gimbal_fence: SiyiCommandFence | None = None) -> None:
        self.fence = fence
        self.gimbal_fence = SiyiCommandFence() if gimbal_fence is None else gimbal_fence
        self._barrier = barrier
        self._publish = publish
        self._lock = rt.Lock()
        self._thread = None

    def on_terminal(self, outcome: FinalApproachTerminal, identity: DockContactIdentity | None) -> None:
        """Any thread, under the final-approach lock: closes the fence at once; the rest runs on its own thread."""
        self._begin(TRIGGER_TERMINAL, outcome, identity)

    def quiesce(self, trigger: str) -> None:
        """The same handoff without an outcome (TRIGGER_STOP or TRIGGER_SHUTDOWN); nothing once one has begun."""
        self._begin(trigger, None, None)

    def finish(self, trigger: str, timeout_s: float = HANDOFF_JOIN_S) -> bool:
        """navpy's cleanup: quiesce unless already begun, then wait, bounded, for the record (False: not out)."""
        self.quiesce(trigger)
        return self.join(timeout_s)

    def _begin(self, trigger: str, outcome: FinalApproachTerminal | None,
               identity: DockContactIdentity | None) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self.fence.close()
            self.gimbal_fence.close()
            self._thread = rt.Thread(target=self._handoff, args=(trigger, outcome, identity),
                                     name="navpy-terminal-handoff")
            self._thread.start()

    def join(self, timeout_s: float | None = None) -> bool:
        """True once the record is out (False before any outcome or while the barrier waits)."""
        with self._lock:
            thread = self._thread
        if thread is None:
            return False
        thread.join(timeout_s)
        return not thread.is_alive()

    def _handoff(self, trigger: str, outcome: FinalApproachTerminal | None,
                 identity: DockContactIdentity | None) -> None:
        self.fence.wait_idle()
        gimbal_stop = self.gimbal_fence.quiesce_and_stop()
        barrier = self._barrier.close_endpoint()
        self._publish({
            "trigger": trigger,
            "outcome": None if outcome is None else outcome.value,
            "task": None if identity is None else identity.task_id,
            "dock": None if identity is None else identity.dock_uid,
            "episode": None if identity is None else identity.episode,
            "aircraft_time_s": self.fence.autopilot_uptime_s(),
            "fenced_writes": self.fence.dropped,
            "gimbal_stop": gimbal_stop,
            "fenced_gimbal_commands": self.gimbal_fence.dropped,
            "barrier": "acked" if barrier.ok else "failed",
            "router_ack": barrier.ack,
            "barrier_error": barrier.error,
        })


def build_terminal_handoff(args: Namespace) -> TerminalHandoff | None:
    if getattr(args, "router_control", None) is None:
        return None
    barrier = RouterBarrier(parse_address(args.router_control), args.router_client)
    return TerminalHandoff(LinkFence(), barrier)


def add_args(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--router-control", metavar="HOST:PORT", default=None,
        help=(
            "HITL: the board link router's control port. On the first terminal outcome navpy fences its "
            f"MAVLink link, closes its router endpoint and prints {NAVPY_TERMINAL_MARKER}"
        ),
    )
    parser.add_argument(
        "--router-client", metavar="NAME", default=DEFAULT_ROUTER_CLIENT,
        help="HITL: navpy's endpoint name at the router (default %(default)s)",
    )


__all__ = [
    "DEFAULT_ROUTER_CLIENT", "HANDOFF_JOIN_S", "LinkFence", "NAVPY_TERMINAL_MARKER", "TRIGGER_SHUTDOWN",
    "TRIGGER_STOP", "TRIGGER_TERMINAL", "TerminalHandoff", "add_args", "build_terminal_handoff", "fenced_opener",
    "own_quiesce", "parse_terminal_record", "print_terminal_record", "siyi_fence", "terminal_observer",
]
