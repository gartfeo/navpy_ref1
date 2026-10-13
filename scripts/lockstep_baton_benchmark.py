"""Reproducible cost of the lockstep baton vs a synchronous tick step.

Prints median and range over repeated samples:
- a 200-item mission upload, per tick (baton runs the real MissionUploader;
  synchronous runs a hand-written state machine);
- N managed threads that each wake every tick (the baton's idle cost).

Run: python scripts/lockstep_baton_benchmark.py
"""

from __future__ import annotations

import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "src"):
    sys.path.insert(0, str(path))

import pytest  # noqa: E402

from navpy.sim.lockstep.baton import BatonRuntime  # noqa: E402
from navpy.sim.lockstep.baton_queue import BatonQueue  # noqa: E402
from tests.modules.lockstep import test_mission_upload_lockstep as upload  # noqa: E402

SAMPLES = 7
TICK_US = 20_000


def _stats(values: list[float]) -> str:
    return (f"median {statistics.median(values):.0f} us "
            f"(min {min(values):.0f}, max {max(values):.0f}, n={len(values)})")


def upload_per_tick(model: str) -> list[float]:
    patch = pytest.MonkeyPatch()
    samples = []
    for _ in range(SAMPLES):
        start = time.perf_counter()
        journal = (upload.run_baton_model(patch, 200)[1] if model == "baton"
                   else upload.run_sync_model(200)[1])
        samples.append((time.perf_counter() - start) / len(journal) * 1e6)
    patch.undo()
    return samples


def idle_per_tick(threads: int, ticks: int = 300) -> list[float]:
    """Each worker is woken once per tick by a coordinator pulse."""
    samples = []
    for _ in range(SAMPLES):
        runtime = BatonRuntime()
        queues = [BatonQueue(runtime) for _ in range(threads)]
        wakes = [0] * threads

        def worker(index: int) -> None:
            while True:
                queues[index].get()
                wakes[index] += 1

        for index in range(threads):
            runtime.spawn(lambda i=index: worker(i), name=f"w{index}")
        runtime.run_until_quiescent()

        def pulse() -> None:
            for queue in queues:
                queue.deliver(None)

        start = time.perf_counter()
        for k in range(1, ticks + 1):
            runtime.tick(k * TICK_US, pulse)
        samples.append((time.perf_counter() - start) / ticks * 1e6)
        runtime.abort()
        if wakes != [ticks] * threads:
            raise RuntimeError(f"a worker missed a tick: {wakes}")
    return samples


def main() -> int:
    print(f"python {sys.version.split()[0]} on {sys.platform}")
    print("mission upload, baton:      ", _stats(upload_per_tick("baton")))
    print("mission upload, synchronous:", _stats(upload_per_tick("sync")))
    for threads in (10, 40):
        print(f"{threads} threads waking every tick:", _stats(idle_per_tick(threads)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
