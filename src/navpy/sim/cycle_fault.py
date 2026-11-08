"""SITL-only NAV cycle fault injection, for the recovery checks (docs/design/navpy_recovery.md, step 9 (b), (c)).

``--sim-cycle-fault-file PATH`` wraps NAV's cycle at composition; without the flag nothing is wrapped and the cycle
path is unchanged. While PATH exists, each NAV cycle fails before it runs, as its content says:

- ``hang``: the cycle blocks until the file is gone or says otherwise: a stuck NAV loop, which the in-process
  watchdog (step 6) must see and hold for;
- anything else (``raise``, or an empty file): the cycle raises, a failed cycle that NAV recovery (step 4) counts.

Deleting the file heals it. Remove it before stopping NavPy: a hang holds the NAV loop, and so its shutdown.

It is refused, with an error, on a vehicle that is not a simulated autopilot, and by the lockstep host
(``navpy/app/lockstep/navpy_host.py``), so it can never reach a real aircraft or a scored lockstep run. Arming it and
each change of its mode are recovery reports (source ``sim_fault``) in the recovery journal, so a wall eval run with
it scores at best "failed" by the clean-run gate, never clean.
"""

from __future__ import annotations

from argparse import ArgumentParser
from collections.abc import Callable
from pathlib import Path
from typing import Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.logger_api import ILogger

SIM_CYCLE_FAULT_FLAG = "--sim-cycle-fault-file"
HANG = "hang"
RAISE = "raise"
SOURCE = "sim_fault"
#: How often a hung cycle looks at the file again: test tooling, healed by a person or a test script.
HANG_CHECK_S = 0.05


class SimCycleFaultRefused(ValueError):
    """The hook was asked for where it must never run."""


class SimCycleFaultInjected(RuntimeError):
    """The injected failure of one NAV cycle."""


class _Cycle(Protocol):
    def run(self) -> None: ...


class _SimulatedVehicle(Protocol):
    def is_simulated_autopilot(self) -> bool: ...


def add_args(parser: ArgumentParser) -> None:
    parser.add_argument(
        SIM_CYCLE_FAULT_FLAG, dest="sim_cycle_fault_file", default=None, metavar="PATH",
        help="SITL only: while PATH exists, each NAV cycle fails ('hang' in it: blocks; else: raises). "
             "Refused on a real autopilot and in lockstep.",
    )


class FaultingCycle:
    def __init__(
        self,
        cycle: _Cycle,
        path: Path,
        report: Callable[[str], None],
        sleep: Callable[[float], None] = rt.sleep,
    ) -> None:
        self._cycle = cycle
        self._path = path
        self._report = report
        self._sleep = sleep
        self._mode: str | None = None       # the NAV thread's only

    def run(self) -> None:
        mode = self._read()
        while mode == HANG:
            self._sleep(HANG_CHECK_S)
            mode = self._read()
        if mode is not None:
            raise SimCycleFaultInjected(f"sim cycle fault injected ({self._path})")
        self._cycle.run()

    def _read(self) -> str | None:
        try:
            text = self._path.read_text(encoding="utf-8").strip().lower()
        except FileNotFoundError:
            mode = None
        else:
            mode = HANG if text == HANG else RAISE
        if mode != self._mode:
            self._mode = mode
            self._report(f"sim cycle fault: {mode or 'healed'}")
        return mode


def sim_cycle_fault(cycle: _Cycle, path: object, vehicle: _SimulatedVehicle, logger: ILogger) -> _Cycle:
    """``cycle`` itself without the flag (NavArgs holds None, or the flag's string); else the faulting wrapper,
    refused unless the autopilot is simulated."""
    if not isinstance(path, str):
        return cycle
    if not vehicle.is_simulated_autopilot():
        raise SimCycleFaultRefused(f"{SIM_CYCLE_FAULT_FLAG} is SITL only: this autopilot is not simulated")
    logger.recovery(f"sim cycle fault hook armed: {path}", SOURCE)
    return FaultingCycle(cycle, Path(path), lambda message: logger.recovery(message, SOURCE))


__all__ = [
    "FaultingCycle", "HANG", "RAISE", "SIM_CYCLE_FAULT_FLAG", "SimCycleFaultInjected", "SimCycleFaultRefused",
    "add_args", "sim_cycle_fault",
]
