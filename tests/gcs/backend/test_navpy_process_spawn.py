"""NavPy starts suspended inside its job (bounded_cpu_share.md); Windows only."""

from __future__ import annotations

import ctypes
import os
import subprocess
import sys
import threading
from pathlib import Path
from unittest.mock import MagicMock

import pytest

pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="Windows job objects")

from gcs.backend import navpy_process_instance as instance  # noqa: E402
from gcs.backend import navpy_process_spawn as spawn  # noqa: E402
from gcs.backend.cpu_share_native import (  # noqa: E402
    ABOVE_NORMAL_PRIORITY_CLASS,
    BELOW_NORMAL_PRIORITY_CLASS,
    WindowsScheduling,
    cpus_of,
    mask_of,
)

SRC = Path(__file__).resolve().parents[3] / "src"
FLAGS = subprocess.CREATE_NEW_PROCESS_GROUP


class FakeScheduling:
    def __init__(self, events: list, holding: bool, demotes: bool = True) -> None:
        self.events, self.holding, self.demotes = events, holding, demotes

    def give_up(self, reason: str) -> bool:
        self.events.append(("give_up", reason))
        # A failed demotion keeps the share held.
        self.holding = self.holding and not self.demotes
        return not self.holding


@pytest.fixture
def events(monkeypatch):
    log: list = []
    monkeypatch.setattr(spawn, "_create_job", lambda: log.append("create_job") or 77)
    monkeypatch.setattr(spawn, "_assign_job", lambda job, proc: log.append(("assign", job, proc.n)))
    monkeypatch.setattr(spawn, "_close_job", lambda job: log.append(("close_job", job)))
    monkeypatch.setattr(spawn, "_resume", lambda proc: log.append(("resume", proc.n)))
    return log


def _popen(events: list):
    count = iter(range(1, 100))

    def factory(command, **kwargs):
        proc = MagicMock(n=next(count))
        proc.kill.side_effect = lambda: events.append(("kill", proc.n))
        events.append(("popen", proc.n, kwargs["creationflags"]))
        return proc

    return factory


def _spawn(events, holding=False, demotes=True):
    return spawn.spawn_in_job(["navpy"], {"creationflags": FLAGS}, popen_factory=_popen(events),
                              sys_id=1, logger=MagicMock(),
                              scheduling=FakeScheduling(events, holding, demotes))


def test_navpy_starts_suspended_joins_its_job_then_resumes(events):
    proc, job = _spawn(events)
    assert (proc.n, job) == (1, 77)
    assert events == ["create_job", ("popen", 1, FLAGS | BELOW_NORMAL_PRIORITY_CLASS | spawn.CREATE_SUSPENDED),
                      ("assign", 77, 1), ("resume", 1)]


def test_a_holders_navpy_starts_at_normal_and_raises_itself(events):
    _spawn(events, holding=True)
    assert events[1] == ("popen", 1, FLAGS | spawn.CREATE_SUSPENDED)


def test_a_holder_without_a_job_gives_up_its_share_before_spawning(events, monkeypatch):
    def refused():
        raise OSError("no job")

    monkeypatch.setattr(spawn, "_create_job", refused)
    proc, job = _spawn(events, holding=True)
    assert job is None
    assert events == [("give_up", "job creation failed"),
                      ("popen", 1, FLAGS | BELOW_NORMAL_PRIORITY_CLASS)]


def test_a_holders_unassigned_child_is_ended_before_it_runs(events, monkeypatch):
    def refused(_job, _proc):
        raise OSError("access denied")

    monkeypatch.setattr(spawn, "_assign_job", refused)
    proc, job = _spawn(events, holding=True)
    assert (proc.n, job) == (2, None)
    # The suspended child never resumes; the share ends before the background NavPy starts.
    assert events == ["create_job", ("popen", 1, FLAGS | spawn.CREATE_SUSPENDED),
                      ("close_job", 77), ("kill", 1), ("give_up", "job assignment failed"),
                      ("popen", 2, FLAGS | BELOW_NORMAL_PRIORITY_CLASS)]


def test_a_holder_that_cannot_be_demoted_starts_no_navpy_without_a_job(events, monkeypatch):
    def refused():
        raise OSError("no job")

    monkeypatch.setattr(spawn, "_create_job", refused)
    with pytest.raises(OSError, match="could not be given up"):
        _spawn(events, holding=True, demotes=False)
    assert events == [("give_up", "job creation failed")]


def test_a_holder_that_cannot_be_demoted_ends_its_unassigned_child(events, monkeypatch):
    def refused(_job, _proc):
        raise OSError("access denied")

    monkeypatch.setattr(spawn, "_assign_job", refused)
    with pytest.raises(OSError, match="could not be given up"):
        _spawn(events, holding=True, demotes=False)
    assert events[-3:] == [("close_job", 77), ("kill", 1), ("give_up", "job assignment failed")]


def test_without_a_share_an_unassigned_child_runs_on(events, monkeypatch):
    def refused(_job, _proc):
        raise OSError("access denied")

    monkeypatch.setattr(spawn, "_assign_job", refused)
    proc, job = _spawn(events)
    assert (proc.n, job) == (1, None)
    assert events[-2:] == [("close_job", 77), ("resume", 1)]


def test_a_failed_resume_ends_the_child_and_closes_its_job(events, monkeypatch):
    def refused(_proc):
        raise OSError("NtResumeProcess failed")

    monkeypatch.setattr(spawn, "_resume", refused)
    with pytest.raises(OSError, match="NtResumeProcess"):
        _spawn(events)
    assert events[-2:] == [("kill", 1), ("close_job", 77)]


def test_a_demotion_during_a_spawn_waits_for_it(events, monkeypatch):
    """The expiry timer's demotion takes the spawn's lock, so it runs after the
    resume and reaches the new job."""
    inside, go = threading.Event(), threading.Event()
    factory = _popen(events)

    def blocking(command, **kwargs):
        inside.set()
        assert go.wait(10.0)
        return factory(command, **kwargs)

    monkeypatch.setitem(instance._open_jobs, 77, 77)  # what _create_job registers

    def demote():
        with instance.JOBS_LOCK:
            events.append(("demote", sorted(instance._open_jobs)))

    spawner = threading.Thread(target=lambda: spawn.spawn_in_job(
        ["navpy"], {"creationflags": FLAGS}, popen_factory=blocking, sys_id=1,
        logger=MagicMock(), scheduling=FakeScheduling(events, True)))
    spawner.start()
    assert inside.wait(10.0)
    demoter = threading.Thread(target=demote)
    demoter.start()
    go.set()
    spawner.join(10.0)
    demoter.join(10.0)
    assert events[-2:] == [("resume", 1), ("demote", [77])]


# Real processes through the venv launcher.

_REPORT = """
import ctypes, os, sys
from ctypes import wintypes
k = ctypes.WinDLL("kernel32")
me = wintypes.HANDLE(-1)  # GetCurrentProcess()
mask, system = ctypes.c_size_t(), ctypes.c_size_t()
k.GetProcessAffinityMask(me, ctypes.byref(mask), ctypes.byref(system))
print(os.getpid(), k.GetPriorityClass(me), mask.value, flush=True)
"""


def _child_env() -> dict[str, str]:
    return {**os.environ, "PYTHONPATH": str(SRC)}


def _query(pid: int, job: int) -> tuple[int, int, bool]:
    """Priority class, affinity and membership in *job* of *pid*, read from outside."""
    from ctypes import wintypes

    k = ctypes.WinDLL("kernel32", use_last_error=True)
    k.OpenProcess.restype = wintypes.HANDLE
    k.IsProcessInJob.argtypes = [wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)]
    handle = k.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
    assert handle, ctypes.get_last_error()
    try:
        mask, system, in_job = ctypes.c_size_t(), ctypes.c_size_t(), wintypes.BOOL()
        assert k.GetProcessAffinityMask(wintypes.HANDLE(handle), ctypes.byref(mask), ctypes.byref(system))
        assert k.IsProcessInJob(wintypes.HANDLE(handle), wintypes.HANDLE(job), ctypes.byref(in_job))
        return k.GetPriorityClass(wintypes.HANDLE(handle)), mask.value, bool(in_job.value)
    finally:
        k.CloseHandle(wintypes.HANDLE(handle))


#: What a holder's NavPy does first (``raise_process_qos``).
_RAISE_ABOVE = (
    "import ctypes\nfrom ctypes import wintypes\n"
    "assert ctypes.WinDLL('kernel32').SetPriorityClass(wintypes.HANDLE(-1), 0x8000)\n"
)


def test_a_real_venv_child_runs_inside_its_job_and_the_job_demotes_it():
    proc, job = spawn.spawn_in_job(
        [sys.executable, "-c", _RAISE_ABOVE + _REPORT + "sys.stdin.readline()\n"],
        {"stdin": subprocess.PIPE, "stdout": subprocess.PIPE, "creationflags": FLAGS},
        popen_factory=subprocess.Popen, sys_id=1, logger=MagicMock(),
        scheduling=FakeScheduling([], holding=True))
    try:
        assert job is not None
        pid = int(proc.stdout.readline().split()[0])  # the real interpreter, past the launcher
        priority, mask, in_job = _query(pid, job)
        assert in_job and priority == ABOVE_NORMAL_PRIORITY_CLASS
        one_cpu = mask_of(cpus_of(mask)[:1])
        assert instance.demote_open_jobs(BELOW_NORMAL_PRIORITY_CLASS, one_cpu) >= 1
        assert _query(pid, job) == (BELOW_NORMAL_PRIORITY_CLASS, one_cpu, True)
    finally:
        instance._terminate_job(job)
        proc.wait(10)
        proc.stdin.close()
        proc.stdout.close()


def test_a_real_navpy_beside_the_shares_starts_below_normal_from_a_normal_backend():
    """The creation flag, not inheritance: the backend here is NORMAL, as when
    its background priority was refused (pytest itself runs as background)."""
    parent = (
        "import ctypes, subprocess, sys\n"
        "from ctypes import wintypes\n"
        "from unittest.mock import MagicMock\n"
        "from gcs.backend import navpy_process_instance as instance\n"
        "from gcs.backend import navpy_process_spawn as spawn\n"
        "k = ctypes.WinDLL('kernel32')\n"
        "k.GetPriorityClass.argtypes = [wintypes.HANDLE]\n"
        "assert k.SetPriorityClass(wintypes.HANDLE(-1), 0x20)\n"  # NORMAL
        "class Beside:\n"
        "    holding = False\n"
        "    def give_up(self, reason): return True\n"
        f"proc, job = spawn.spawn_in_job([sys.executable, '-c', {_REPORT!r}],"
        f" {{'stdout': subprocess.PIPE, 'creationflags': {FLAGS}}}, popen_factory=subprocess.Popen,"
        " sys_id=1, logger=MagicMock(), scheduling=Beside())\n"
        "child = proc.stdout.read().decode().split()\n"
        "proc.wait(10)\n"
        "instance._terminate_job(job)\n"
        "print(k.GetPriorityClass(wintypes.HANDLE(-1)), child[1])\n"
    )
    out = subprocess.run([sys.executable, "-c", parent], capture_output=True, text=True,
                         env=_child_env(), timeout=60, check=True).stdout
    assert tuple(map(int, out.split())) == (0x20, BELOW_NORMAL_PRIORITY_CLASS)


def test_a_real_grandchild_inherits_below_normal_and_the_affinity():
    """Background is set once on a process; its children, through the venv
    launcher and a new process group, start with both."""
    two_cpus = mask_of(cpus_of(WindowsScheduling().own()[1])[:2])
    parent = (
        "import subprocess, sys\n"
        "from gcs.backend.cpu_share_native import BELOW_NORMAL_PRIORITY_CLASS, WindowsScheduling\n"
        f"WindowsScheduling().set_own(BELOW_NORMAL_PRIORITY_CLASS, {two_cpus})\n"
        f"print(subprocess.run([sys.executable, '-c', {_REPORT!r}], capture_output=True, text=True,"
        f" creationflags={FLAGS}).stdout, end='')\n"
    )
    out = subprocess.run([sys.executable, "-c", parent], capture_output=True, text=True,
                         env=_child_env(), timeout=60, check=True).stdout
    _pid, priority, affinity = map(int, out.split())
    assert (priority, affinity) == (BELOW_NORMAL_PRIORITY_CLASS, two_cpus)
