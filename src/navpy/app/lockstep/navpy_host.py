"""NavPy inside its participant: composed and run on the bootstrap thread.

Construction (the managed bootstrap thread, at the admission tick, under the
baton; ``lockstep_slice.md``, "NavPy participant" and "Participant runtime"):

- process identity: readiness names ``{ordinal, generation}`` and is a
  ``runtime_ready`` event in the tick's ``DONE``, which the coordinator turns
  into the lifecycle ``ready`` record (never stdout);
- the navigation log lands in the run-relative :data:`LOG_DIR` (the
  participant's working directory, the run directory, as wall mode's
  ``uav_<sys>_*`` files do), names no host path (run tokens instead), and
  every line also reaches ``DONE.log_lines``;
- its device (``lockstep:serial0.<sysid>``) is opened through the link (``create_vehicle``'s
  ``open_bus``); the scheduler cadence is :class:`IdentitySchedulerCadence`
  (nothing writes ``SIM_SPEEDUP``: a navigation speedup is refused);
- the stop is a :class:`LifecycleStopRequest` (``--stop-on-lifecycle`` is
  required): a lifecycle ``stop`` or the drain ``end`` ends the run loop, the
  existing cleanup joins every producer and drains the log worker on ticks,
  and the ack is a ``stopped{id}`` event after the strict pose-cadence flush.

A failure of the composition or the run is kept, not raised: the end's
flushes report it, so the participant ends with ``flushed: false`` (a stop
then answers ``missed``).
"""

from __future__ import annotations

import logging
import sys
import traceback
from argparse import Namespace
from collections.abc import Callable, Sequence
from pathlib import Path

from navpy.app import runtime_ready
from navpy.app.navpy_argparse import make_parser
from navpy.app.network_args import NetworkType
from navpy.app.runtime_composition import RuntimePorts, compose_runtime
from navpy.app.runtime_stop import LifecycleStopRequest
from navpy.core.kernel.logging import logger_factory
from navpy.core.kernel.logging.logger_api import ILogger
from navpy.core.kernel.logging.log_output_latch import close_strictly
from navpy.core.kernel.scheduler_cadence import IdentitySchedulerCadence
from navpy.sim.lockstep.coordinator_tick import READY_EVENT
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.log_capture import HostPathScrubbingFormatter, OutboxLogHandler
from navpy.sim.lockstep.part_wire import Event, RequestId
from navpy.sim.lockstep.participant_hooks import OutboxBuffer, ParticipantContext

#: The participant's run-relative log directory: its working directory.
LOG_DIR = "."
#: The acknowledgement of a participant stop, in the ending ``DONE``.
STOPPED_EVENT = "stopped"
#: The peer transports a lockstep NavPy may run: the vehicle bus
#: (``NetworkMavlink``, its NAVLINK on the companion device) or none.
#: ``NetworkWifi`` and ``NetworkSerial`` open host sockets and ports, so they
#: stay wall-only (``lockstep_fleet.md``, "Remaining packages" item 8).
LOCKSTEP_NETWORK_TYPES = frozenset({NetworkType.MAV, NetworkType.NONE})
#: The navigation file log's line format (``logger_factory.initialize_logger``).
LINE_FORMAT = "%(asctime)s %(levelname)s %(message)s"


def parse_lockstep_args(argv: Sequence[str], device: str) -> Namespace:
    """NavPy's arguments, checked against the lockstep contract."""
    args = make_parser(description="NavPy Control System").parse_args(list(argv))
    if args.connection != device:
        raise ValueError(f"lockstep NavPy connects to {device}, not {args.connection!r}")
    if not args.stop_on_lifecycle or args.stop_on_stdin:
        raise ValueError("lockstep NavPy stops on its lifecycle record only (--stop-on-lifecycle)")
    if args.nav_sim_speedup > 0.0:
        raise ValueError("lockstep NavPy never writes SIM_SPEEDUP (-gsu)")
    if args.network_type not in LOCKSTEP_NETWORK_TYPES:
        raise ValueError(f"lockstep NavPy's peer transport is -nt mav or none, not "
                         f"{args.network_type.value!r}: a host socket or port is wall-only")
    return args


class NavpyHost:
    def __init__(self, context: ParticipantContext, link: LockstepLink, out: OutboxBuffer, *,
                 held_stop_id: RequestId | None = None, held_end: bool = False) -> None:
        self._out = out
        self._link = link
        self._args = parse_lockstep_args(context.config.get("argv") or (), link.device)
        runtime_ready.set_process_identity(context.participant.identity)
        runtime_ready.set_ready_publisher(self._publish_ready)
        logger_factory.bind_run_log_dir(LOG_DIR)
        self.stop = LifecycleStopRequest(self._publish_stopped, held_stop_id=held_stop_id,
                                         held_end=held_end)
        self._logger: ILogger | None = None
        self._failure: BaseException | None = None

    def _publish_ready(self, ready: dict[str, object]) -> None:
        self._out.events.append(Event(READY_EVENT, {"role": "navpy", **ready}))

    def _publish_stopped(self, stop_id: RequestId) -> None:
        self._out.events.append(Event(STOPPED_EVENT, {"id": list(stop_id)}))

    def _observe_logger(self, logger: ILogger) -> None:
        """Every navigation line also into ``DONE``; the file log names no host path."""
        self._logger = logger
        navigation = logging.getLogger(f"vehicle_logger_{self._args.source_system}")
        for handler in navigation.handlers:
            if isinstance(handler, logging.FileHandler):
                handler.setFormatter(HostPathScrubbingFormatter(handler.formatter, Path.cwd()))
        navigation.addHandler(OutboxLogHandler(self._out, Path.cwd(), level=logging.DEBUG,
                                               line_format=LINE_FORMAT))

    def run(self) -> None:
        """Compose, then run until the stop; a failure is kept for the flushes."""
        ports = RuntimePorts(
            open_bus=self._link.open,
            scheduler_cadence=lambda _vehicle, _bus: IdentitySchedulerCadence(),
            observe_logger=self._observe_logger,
            lifecycle_stop=self.stop,
        )
        try:
            compose_runtime(self._args, ports).run()
        except Exception as error:
            # The kind reaches the journal through the unflushed end; the text
            # (host paths) goes to stderr, the coordinator's host log.
            traceback.print_exception(error, file=sys.stderr)
            self._failure = error

    def flushes(self) -> Sequence[Callable[[], object]]:
        """The end's strict flushes: the run's outcome, then the log worker."""
        return (self._raise_if_failed, self._close_log)

    def _raise_if_failed(self) -> None:
        if self._failure is not None:
            raise RuntimeError("NavPy's run failed") from self._failure

    def _close_log(self) -> None:
        if self._logger is not None:
            close_strictly(self._logger)


__all__ = ["LINE_FORMAT", "LOCKSTEP_NETWORK_TYPES", "LOG_DIR", "NavpyHost", "STOPPED_EVENT", "parse_lockstep_args"]
