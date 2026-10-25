"""A participant process: ``python -m navpy.sim.lockstep.participant_main``.

1. It runs nothing before ``go``: its first stdin line must be ``go``,
   written by the coordinator only after it recorded this child under the
   registry lock. Anything else, or EOF (the coordinator died or refused it),
   exits at once, so an unrecorded child never runs.
2. Bootstrap steps 1-2: only the standard library, ``runtime_time`` and
   ``navpy.sim.lockstep`` are imported, then the runtime is bound.
3. The hooks factory named in the environment record is loaded (it must not
   import the product; ``construct`` does, at the admission tick), the loop
   connects to the coordinator and registers. Its handoff caps come from the
   role config's participant models (``participant_budget``), if any, and so
   do the sim camera's frame schedules (``camera_frames``), which the hooks get.
"""

from __future__ import annotations

import importlib
import os
import random
import sys
import time
from collections.abc import Callable

from navpy.sim.lockstep.camera_frames import camera_frame_factory
from navpy.sim.lockstep.part_socket import PartSocket
from navpy.sim.lockstep.participant import bootstrap
from navpy.sim.lockstep.participant_budget import SIM_MODELS_KEY, TickBudget
from navpy.sim.lockstep.participant_env import ENVIRONMENT_VARIABLE, EnvironmentRecord
from navpy.sim.lockstep.participant_hooks import ParticipantContext, ParticipantHooks, RequestIds
from navpy.sim.lockstep.participant_loop import ParticipantLoop

#: Exit code of a child that never got ``go``.
EXIT_NO_GO = 4
#: Wall budget for the loopback connect to the coordinator (host supervision).
CONNECT_WALL_BUDGET_S = 30.0
#: Acceptance (c): a host-only seed of injected delays before each ``DONE``
#: (set by the spawner; never part of the environment record or the journal).
HOST_DELAY_VARIABLE = "NAVPY_LOCKSTEP_HOST_DELAY_SEED"
#: Upper bound of one injected participant host delay (seconds).
MAX_HOST_DELAY_S = 0.004


def host_delay() -> Callable[[], None]:
    seed = int(os.environ.get(HOST_DELAY_VARIABLE, "0") or 0)
    if not seed:
        return lambda: None
    delays = random.Random(seed)
    return lambda: time.sleep(delays.random() * MAX_HOST_DELAY_S)


def wait_for_go() -> bool:
    return sys.stdin.readline().strip() == "go"


def load_factory(path: str) -> Callable[[ParticipantContext], ParticipantHooks]:
    module, _, attribute = path.partition(":")
    return getattr(importlib.import_module(module), attribute)


def main() -> int:
    if not wait_for_go():
        return EXIT_NO_GO
    record = EnvironmentRecord.from_json(os.environ[ENVIRONMENT_VARIABLE])
    participant = bootstrap(record.spec())
    factory = load_factory(record.hooks)
    models = record.config.get(SIM_MODELS_KEY, {})
    camera_frames = camera_frame_factory(models, record.seed, record.ordinal, record.generation)
    hooks = factory(ParticipantContext(participant, record.config, RequestIds(record.ordinal), camera_frames))
    budget = TickBudget(models, record.seed, record.ordinal, record.generation)
    link = PartSocket.connect(record.coordinator, timeout_s=CONNECT_WALL_BUDGET_S)
    try:
        return ParticipantLoop(participant, hooks, link, host_delay=host_delay(),
                               handoffs=budget.handoffs).run(record.register())
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
