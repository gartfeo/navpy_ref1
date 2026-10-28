"""Background and holder scheduling (bounded_cpu_share.md), on a fake kernel and timer."""

from __future__ import annotations

import pytest

from gcs.backend import cpu_share
from gcs.backend.cpu_share import ShareRecord
from gcs.backend.cpu_share_background import BackgroundScheduling
from gcs.backend.cpu_share_holder import CPU_SHARE_ENV, StackScheduling
from gcs.backend.cpu_share_native import ABOVE_NORMAL_PRIORITY_CLASS as ABOVE
from gcs.backend.cpu_share_native import BELOW_NORMAL_PRIORITY_CLASS as BELOW
from gcs.backend.cpu_share_native import mask_of

NORMAL = 0x20
P_CORES = (0, 1, 6, 7, 8, 9, 18, 19)
HYBRID = {cpu: int(cpu in P_CORES) for cpu in range(20)}
SYSTEM = (1 << 20) - 1


class Clock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


class FakeNative:
    def __init__(self, events: list, system: int = SYSTEM, fail: bool = False,
                 affinity_fails: bool = False) -> None:
        self.events, self.system, self.fail, self.affinity_fails = events, system, fail, affinity_fails
        self.priority = NORMAL

    def efficiency_classes(self) -> dict[int, int]:
        return dict(HYBRID)

    def system_mask(self) -> int:
        return self.system

    def own(self) -> tuple[int, int]:
        return self.priority, self.system

    def set_own(self, priority: int, mask: int) -> None:
        self.events.append(("set_own", priority, mask))
        if self.fail:
            raise OSError("denied")
        self.priority = priority  # before the affinity, as WindowsScheduling sets them
        if self.affinity_fails:
            raise OSError("affinity denied")


class FakeTimer:
    def __init__(self, interval: float, function, args=()) -> None:
        self.interval, self.function, self.args = interval, function, args
        self.daemon = self.started = self.cancelled = False

    def start(self) -> None:
        self.started = True

    def cancel(self) -> None:
        self.cancelled = True

    def fire(self) -> None:
        self.function(*self.args)


@pytest.fixture(autouse=True)
def own_start(monkeypatch):
    monkeypatch.setattr(cpu_share.runtime, "pid_start_time", lambda _pid: 1234)


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def record(tmp_path, clock):
    return ShareRecord(tmp_path / "cpu_shares.json", alive=lambda _pid, _start: True, clock=clock)


@pytest.fixture
def timers():
    made: list[FakeTimer] = []

    def factory(*args):
        made.append(FakeTimer(*args))
        return made[-1]

    factory.made = made
    return factory


# Background.

def test_background_avoids_valid_shares_and_recomputes_at_each_expiry(record, clock, timers):
    first, second = record.claim("a", 60.0, HYBRID), record.claim("b", 30.0, HYBRID)
    events: list = []
    background = BackgroundScheduling(record, FakeNative(events), timer=timers)

    assert background.apply() is True
    assert events[-1] == ("set_own", BELOW, SYSTEM & ~mask_of(first.cpus + second.cpus))
    assert (timers.made[-1].interval, timers.made[-1].daemon, timers.made[-1].started) == (30.0, True, True)

    clock.now = 1030.0  # the second share expires
    timers.made[-1].fire()
    assert events[-1] == ("set_own", BELOW, SYSTEM & ~mask_of(first.cpus))
    assert timers.made[-1].interval == 30.0

    clock.now = 1060.0
    timers.made[-1].fire()
    assert events[-1] == ("set_own", BELOW, SYSTEM)
    assert len(timers.made) == 2  # nothing left to wait for


def test_background_keeps_priority_only_when_every_cpu_is_shared(record, timers):
    share = record.claim("a", 60.0, HYBRID)
    events: list = []
    only_shared = mask_of(share.cpus)
    assert BackgroundScheduling(record, FakeNative(events, only_shared), timer=timers).apply()
    assert events == [("set_own", BELOW, only_shared)]


def test_background_is_a_no_op_without_cpus_and_reports_a_refused_priority(record, timers, caplog):
    events: list = []
    assert BackgroundScheduling(record, FakeNative(events, system=0), timer=timers).apply() is False
    assert events == []
    assert BackgroundScheduling(record, FakeNative(events, fail=True), timer=timers).apply() is False
    assert timers.made == []
    assert [r.levelname for r in caplog.records] == ["ERROR"]


def test_background_priority_without_its_cpus_is_still_background(record, timers):
    record.claim("a", 60.0, HYBRID)
    events: list = []
    native = FakeNative(events, affinity_fails=True)
    assert BackgroundScheduling(record, native, timer=timers).apply() is True
    assert native.priority == BELOW and timers.made[-1].interval == 60.0


# Holder: every row of the design's scheduling table.

def _stack(record, events, timers, native=None, demote=None, background=True):
    def demote_jobs(priority, affinity):
        events.append(("demote_jobs", priority, affinity, [s.id for s, _ in record.listing()]))
        return 3

    return StackScheduling(
        record, native or FakeNative(events), timer=timers,
        background=lambda: events.append("background") or background,
        demote_jobs=demote or demote_jobs,
    )


def test_a_granted_share_runs_the_backend_above_normal_on_its_cpus(record, timers):
    # Level with its NavPys: at NORMAL their threads starved its reader.
    events: list = []
    stack = _stack(record, events, timers)
    assert stack.start(60.0, "eval") == "holder"
    (share, state), = record.listing()
    assert events == [("set_own", ABOVE, mask_of(share.cpus))]
    assert stack.holding and state == "held, 60 s left"
    assert (timers.made[0].interval, timers.made[0].args, timers.made[0].daemon) == (60.0, ("expired",), True)


def test_a_refused_share_runs_as_background(record, timers):
    record.claim("a", 60.0, HYBRID), record.claim("b", 60.0, HYBRID)
    events: list = []
    stack = _stack(record, events, timers)
    assert stack.start(60.0, "eval") == "background"
    assert events == ["background"] and not stack.holding and timers.made == []


def test_no_request_runs_as_background(record, timers):
    events: list = []
    assert _stack(record, events, timers).start(None, "live test") == "background"
    assert events == ["background"] and not record.path.exists()


def test_a_stack_that_cannot_run_as_background_says_so(record, timers):
    assert _stack(record, [], timers, background=False).start(None, "live test") == "normal"


def test_expiry_demotes_the_stack_then_releases_the_share(record, timers):
    events: list = []
    stack = _stack(record, events, timers)
    stack.start(60.0, "eval")
    (share, _state), = record.listing()
    timers.made[0].fire()
    # Demoted while the record still holds the share; released only then.
    assert events[1:] == [("set_own", BELOW, SYSTEM), ("demote_jobs", BELOW, SYSTEM, [share.id])]
    assert record.listing() == [] and not stack.holding


def test_a_failed_demotion_keeps_the_cpus_held(record, timers):
    events: list = []

    def refused(_priority, _affinity):
        raise OSError("job limit refused")

    stack = _stack(record, events, timers, demote=refused)
    stack.start(60.0, "eval")
    assert stack.give_up("expired") is False
    assert stack.holding and len(record.listing()) == 1


def test_a_share_that_cannot_be_applied_is_released_and_the_stack_runs_as_background(record, timers):
    events: list = []
    stack = _stack(record, events, timers, native=FakeNative(events, fail=True))
    assert stack.start(60.0, "eval") == "background"
    assert events[1:] == ["background"] and record.listing() == [] and not stack.holding


def test_a_share_neither_applied_nor_backed_out_stays_held(record, timers):
    # It may run ABOVE_NORMAL on every CPU: no other holder gets these CPUs.
    events: list = []
    stack = _stack(record, events, timers, native=FakeNative(events, affinity_fails=True),
                   background=False)
    assert stack.start(60.0, "eval") == "normal"
    assert len(record.listing()) == 1 and not stack.holding


def test_shutdown_before_expiry_cancels_the_timer_and_releases(record, timers):
    events: list = []
    stack = _stack(record, events, timers)
    stack.start(600.0, "eval")
    assert stack.give_up("backend shutdown") is True
    assert timers.made[0].cancelled and record.listing() == []
    assert stack.give_up("again") is True  # nothing held any more


@pytest.mark.parametrize(("raw", "outcome"), (("90", "holder"), ("abc", "background"), ("", "background")))
def test_the_request_comes_from_the_launchers_environment(record, timers, monkeypatch, raw, outcome):
    monkeypatch.setenv(CPU_SHARE_ENV, raw)
    stack = _stack(record, [], timers)
    assert stack.start_from_env() == outcome
    if outcome == "holder":
        assert timers.made[0].interval == 90.0
