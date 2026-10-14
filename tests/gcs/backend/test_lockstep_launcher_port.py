"""The launcher port names each companion generation apart (``launcher_port``).

Driven as the backend drives it: manager calls on a managed thread, the
coordinator's lifecycle records delivered in the tick's input phase. A
replacement's pid differs from its predecessor's, and a stop with the old
generation's pid is refused without sending any stop request.
"""

from __future__ import annotations

from gcs.backend.lockstep.launcher_port import LockstepLauncher, generation_pid
from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.part_wire import Lifecycle
from navpy.sim.lockstep.participant_hooks import OutboxBuffer, RequestIds

ORDINAL, SYS_ID = 1, 1
TICK_US = 20_000


def test_a_stale_generation_pid_never_stops_the_replacement(monkeypatch):
    baton, out = BatonRuntime(), OutboxBuffer()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(baton, epoch_us=0, seed=1, ordinal=2))
    monkeypatch.setattr(rt, "_binding", binding)
    launcher = LockstepLauncher({SYS_ID: {"role": "navpy", "ordinal": ORDINAL, "device": "lockstep:serial0"}},
                                out, RequestIds(2))
    seen = {}
    commits = iter(range(1, 100))

    def record(kind: str, request, generation: int, result: str = "ok") -> Lifecycle:
        return Lifecycle(kind, (0, next(commits)), request, ORDINAL, generation, result)

    def manager() -> None:
        seen["first"] = launcher.start(SYS_ID, "ignored").pid
        launcher.stop(SYS_ID)
        seen["second"] = launcher.start(SYS_ID, "ignored").pid
        seen["stale"] = launcher.stop_generation(SYS_ID, seen["first"])
        seen["status"] = launcher.get_status(SYS_ID)

    baton.spawn(manager, name="executor-1")
    baton.run_until_quiescent()
    k = 0
    for generation in (1, 2):
        k += 1
        [spawn] = out.spawn_requests[-1:]
        baton.tick(k * TICK_US, lambda s=spawn, g=generation: launcher.deliver(record("spawn", s.id, g)))
        if generation == 1:
            k += 1
            [stop] = out.stop_requests
            baton.tick(k * TICK_US, lambda s=stop: launcher.deliver(record("stop_result", s.id, 1, "acked")))
    assert (seen["first"], seen["second"]) == (generation_pid(ORDINAL, 1), generation_pid(ORDINAL, 2))
    assert seen["stale"] is False
    assert [(s.ordinal, s.generation) for s in out.stop_requests] == [(ORDINAL, 1)]  # none for the stale pid
    assert seen["status"]["pid"] == seen["second"] and seen["status"]["running"]
    assert baton.threads.alive() == [] and not baton.threads.errors


def test_a_refused_process_manager_cannot_be_built(monkeypatch):
    """Lockstep fails closed: the subprocess manager is never constructed."""
    import pytest

    from gcs.backend.navpy_process_manager import NavpyProcessManager

    monkeypatch.setattr(NavpyProcessManager, "refused", "lockstep companions are coordinator participants")
    with pytest.raises(RuntimeError, match="refused"):
        NavpyProcessManager()
