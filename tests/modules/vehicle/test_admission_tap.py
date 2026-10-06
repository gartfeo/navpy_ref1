"""D1: the admission tap, through the real router, its owner and VehicleMav.

Every message the router rules on, autopilot telemetry from its target system,
reaches the tap as a numbered ruling after the verdict and before anything is
published. A subscription is owed exactly ``first``..``end``, and a ruling it
did not get is a fault counted on it. None of this may change what the router
does: its verdicts, its store and its callbacks are the same with a subscriber
as without one.
"""

from __future__ import annotations

import dataclasses
import gc
import sys
import threading
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.modules.vehicle.admission_tap import (
    AdmissionRuling,
    AdmissionSubscription,
    AdmissionTap,
)
from navpy.modules.vehicle.inbound_router import (
    REJECT_FOREIGN_COMPONENT,
    REJECT_STALE_BOOT,
    InboundMessageRouter,
    message_boot_time_ms,
)
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.vehicle_runtime_interfaces import VehicleMessaging

TARGET = 42
# Bounds a FAILING run only: every wait below is released by the test itself.
WAIT_S = 10.0
# The types the router stores in the sequence below.
STORED = ("ATTITUDE", "VFR_HUD", "SYS_STATUS")
IDENTITIES = pytest.mark.parametrize(
    "mav_type", [MAV_TYPE_ONBOARD_CONTROLLER, MAV_TYPE_GCS], ids=["onboard", "gcs"]
)


def _router(
    mav_type: int = MAV_TYPE_ONBOARD_CONTROLLER,
) -> tuple[InboundMessageRouter, MessageStore, CallbackRegistry]:
    store = MessageStore()
    callbacks = CallbackRegistry()
    router = InboundMessageRouter(
        VehicleIdentity(TARGET, TARGET, MAV_COMP_ID_ONBOARD_COMPUTER, mav_type),
        store,
        callbacks,
        ParameterRepository(),
        HeartbeatState(TARGET, 3.0),
        PacketLossTracker(),
        MissionInbox(),
        SimpleNamespace(value=MagicMock()),
    )
    return router, store, callbacks


def _message(
    message_type: str,
    *,
    system: int = TARGET,
    component: int = MAV_COMP_ID_AUTOPILOT1,
    boot_ms: Any = None,
) -> MagicMock:
    message = MagicMock()
    message.get_msgId.return_value = 0
    message.get_srcSystem.return_value = system
    message.get_srcComponent.return_value = component
    message.get_seq.return_value = 0
    message.get_type.return_value = message_type
    message.time_boot_ms = boot_ms
    return message


def _sequence() -> list[MagicMock]:
    """One of each thing the router does with a message."""
    return [
        _message("ATTITUDE", boot_ms=10_000),  # admitted
        _message("ATTITUDE", boot_ms=10_000),  # a repeated stamp, admitted
        _message("ATTITUDE", boot_ms=9_900),  # a stale stamp, rejected
        _message(  # another component, rejected
            "ATTITUDE", component=MAV_COMP_ID_ONBOARD_COMPUTER, boot_ms=20_000
        ),
        _message("VFR_HUD"),  # no stamp, admitted
        _message("SYS_STATUS", boot_ms=5),  # not ruled on
        _message("ATTITUDE", system=TARGET + 1, boot_ms=30_000),  # not ruled on
        _message("ATTITUDE", boot_ms=10_020),  # admitted
    ]


def _run(
    mav_type: int, subscriber: Callable[[AdmissionRuling], None] | None
) -> tuple[list[int], dict[str, list[tuple[int, int | None]]]]:
    """What the router did with the sequence: every callback, and every
    stored sample with its stamp, each by the message's place in it."""
    router, store, callbacks = _router(mav_type)
    messages = _sequence()
    called: list[Any] = []
    callbacks.subscribe("*", called.append)
    if subscriber is not None:
        for message_type in ("ATTITUDE", "VFR_HUD"):
            router.on_admission(message_type, subscriber)
    for message in messages:
        router.ingest(message)
    place = {id(message): index for index, message in enumerate(messages)}
    stored = {
        message_type: [
            (place[id(sample.message)], sample.boot_time_ms)
            for sample in store.recent(message_type, len(messages))
        ]
        for message_type in STORED
    }
    return [place[id(message)] for message in called], stored


def _escaped(call: Callable[[], object]) -> BaseException | None:
    """What ``call`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt that escaped would end the whole pytest
    session instead of failing one test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts
        return escaped
    return None


def _reachable(root: object) -> list[object]:
    """Everything within two steps of ``root`` through the interpreter's own
    referents, classes aside: a ruling's fields, and its __dict__ if any."""
    found: list[object] = []
    frontier = [root]
    for _ in range(2):
        frontier = [
            referent
            for item in frontier
            for referent in gc.get_referents(item)
            if not isinstance(referent, type)
        ]
        found.extend(frontier)
    return found


class _FailsWhenArmed:
    """A real lock whose acquisition raises while ``armed``."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.armed = False

    def __enter__(self) -> bool:
        if self.armed:
            raise RuntimeError("the tap's lock will not open")
        return self._lock.__enter__()

    def __exit__(self, *exc: object) -> None:
        self._lock.__exit__(*exc)


def _construction_bus() -> MagicMock:
    """The fake bus of test_vehicle_mav_identity.py: nothing reads from it."""
    connection = SimpleNamespace(mav=MagicMock())
    return MagicMock(conn=connection, send_lock=threading.RLock(), heartbeats={})


@IDENTITIES
@pytest.mark.parametrize("raises", [False, True], ids=["listening", "raising"])
def test_a_subscriber_changes_no_verdict_store_or_callback(
    mav_type: int, raises: bool
) -> None:
    """Record-only: with a subscriber, even one that raises on every ruling,
    the router admits, stores and dispatches exactly what it does without."""
    heard: list[AdmissionRuling] = []

    def subscriber(ruling: AdmissionRuling) -> None:
        heard.append(ruling)
        if raises:
            raise RuntimeError("the subscriber failed")

    without = _run(mav_type, None)
    with_subscriber = _run(mav_type, subscriber)

    assert heard, "the subscriber heard nothing, so nothing was compared"
    assert with_subscriber == without


def test_every_ruling_reaches_the_tap_before_anything_is_published() -> None:
    """After the verdict, before the store and the callbacks: when a ruling
    arrives, the store still holds the message before it."""
    router, store, callbacks = _router()
    events: list[tuple] = []
    callbacks.subscribe(
        "ATTITUDE",
        lambda message: events.append(("callback", message.time_boot_ms)),
    )

    def heard(ruling: AdmissionRuling) -> None:
        latest = store.latest("ATTITUDE")
        events.append((
            "ruling",
            ruling.ruling,
            ruling.accepted,
            None if latest is None else latest.boot_time_ms,
        ))

    router.on_admission("ATTITUDE", heard)
    for boot_ms in (10_000, 9_900, 10_020):
        router.ingest(_message("ATTITUDE", boot_ms=boot_ms))

    assert events == [
        ("ruling", 1, True, None),
        ("callback", 10_000),
        ("ruling", 2, False, 10_000),
        ("ruling", 3, True, 10_000),
        ("callback", 10_020),
    ]


def test_each_ruling_carries_its_number_stamp_verdict_and_reason() -> None:
    """Admitted, a repeated stamp, stale, another component, and a stampless
    admission: numbered per type, and nothing for what is not ruled on."""
    router, _, _ = _router()
    heard: dict[str, list[AdmissionRuling]] = {"ATTITUDE": [], "VFR_HUD": []}
    for message_type, rulings in heard.items():
        router.on_admission(message_type, rulings.append)

    for message in _sequence():
        router.ingest(message)

    assert heard["ATTITUDE"] == [
        AdmissionRuling("ATTITUDE", 1, 10_000, True, None),
        AdmissionRuling("ATTITUDE", 2, 10_000, True, None),
        AdmissionRuling("ATTITUDE", 3, 9_900, False, REJECT_STALE_BOOT),
        AdmissionRuling("ATTITUDE", 4, 20_000, False, REJECT_FOREIGN_COMPONENT),
        AdmissionRuling("ATTITUDE", 5, 10_020, True, None),
    ]
    assert heard["VFR_HUD"] == [AdmissionRuling("VFR_HUD", 1, None, True, None)]


def test_a_ruling_carries_the_stamp_as_the_router_reads_it() -> None:
    """The masked uint32, and None for what is no stamp at all."""
    router, _, _ = _router()
    heard: list[AdmissionRuling] = []
    router.on_admission("ATTITUDE", heard.append)

    router.ingest(_message("ATTITUDE", boot_ms=2**32 + 7))
    router.ingest(_message("ATTITUDE", boot_ms=True))

    assert [(r.ruling, r.time_boot_ms, r.accepted) for r in heard] == [
        (1, 7, True),
        (2, None, True),
    ]


@IDENTITIES
def test_no_ruling_for_a_message_the_router_does_not_rule_on(
    mav_type: int,
) -> None:
    """Another system's telemetry, and a type that is not autopilot
    telemetry. A GCS router publishes the other system's ATTITUDE unruled; a
    subscription to a type never ruled on would be owed nothing, so it is
    refused."""
    router, store, _ = _router(mav_type)
    heard: list[AdmissionRuling] = []
    router.on_admission("ATTITUDE", heard.append)

    router.ingest(_message("ATTITUDE", system=TARGET + 1, boot_ms=1_000))
    router.ingest(_message("SYS_STATUS", boot_ms=1_000))

    assert heard == []
    assert (store.latest("ATTITUDE") is not None) == (mav_type == MAV_TYPE_GCS)
    assert store.latest("SYS_STATUS") is not None
    with pytest.raises(ValueError):
        router.on_admission("SYS_STATUS", heard.append)


@pytest.mark.parametrize(
    "failure",
    [RuntimeError("no stamp"), SystemExit(3)],
    ids=["error", "interrupt"],
)
def test_a_ruling_that_cannot_be_built_is_a_fault_on_every_subscription(
    failure: BaseException,
) -> None:
    """Containment, the build: a fault on every subscription owed the
    ruling. An error stays in the tap and is counted; an interrupt is
    flagged, not counted, and passed on."""
    def unreadable(message: object) -> int | None:
        raise failure

    tap = AdmissionTap(unreadable, {"ATTITUDE"})
    heard: list[AdmissionRuling] = []
    owed = [tap.subscribe("ATTITUDE", heard.append) for _ in range(2)]

    escaped = _escaped(lambda: tap.rule("ATTITUDE", object(), True, None))

    error = isinstance(failure, Exception)
    assert escaped is (None if error else failure)
    assert heard == []
    assert [(s.faults, s.faulted) for s in owed] == [(int(error), True)] * 2


def test_a_failed_delivery_is_a_fault_on_that_subscription_alone() -> None:
    """Containment, one delivery: the next subscriber still hears the ruling,
    and the router publishes the message as it would have."""
    router, store, _ = _router()
    heard: list[AdmissionRuling] = []

    def failing(ruling: AdmissionRuling) -> None:
        raise RuntimeError("the subscriber failed")

    broken = router.on_admission("ATTITUDE", failing)
    healthy = router.on_admission("ATTITUDE", heard.append)

    router.ingest(_message("ATTITUDE", boot_ms=10_000))

    assert (broken.faults, broken.faulted) == (1, True)
    assert (healthy.faults, healthy.faulted) == (0, False)
    assert [r.ruling for r in heard] == [1]
    assert store.latest("ATTITUDE").boot_time_ms == 10_000


def test_an_interrupt_in_a_delivery_flags_all_it_cost_and_is_passed_on(
) -> None:
    """Containment, an interrupt: never swallowed, and every subscription it
    kept from the ruling, its own and the ones after it, is flagged. Not
    counted: an interrupt's path takes no lock, as D5's latches nothing."""
    tap = AdmissionTap(message_boot_time_ms, {"ATTITUDE"})
    interrupt = SystemExit(3)
    heard: list[AdmissionRuling] = []

    def interrupting(ruling: AdmissionRuling) -> None:
        raise interrupt

    before = tap.subscribe("ATTITUDE", heard.append)
    stopped = tap.subscribe("ATTITUDE", interrupting)
    after = tap.subscribe("ATTITUDE", heard.append)

    escaped = _escaped(lambda: tap.rule(
        "ATTITUDE", _message("ATTITUDE", boot_ms=1), True, None
    ))

    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    assert [r.ruling for r in heard] == [1]
    assert [(s.faults, s.faulted) for s in (before, stopped, after)] == [
        (0, False),
        (0, True),
        (0, True),
    ]


def test_a_fault_whose_count_fails_still_marks_its_subscription() -> None:
    """Containment, the count itself: the flag is set first, a plain store,
    so a count lost to a lock that will not open never reads as no fault."""
    tap = AdmissionTap(message_boot_time_ms, {"ATTITUDE"})
    lock = tap._lock = _FailsWhenArmed()

    def failing(ruling: AdmissionRuling) -> None:
        lock.armed = True
        raise RuntimeError("the subscriber failed")

    subscription = tap.subscribe("ATTITUDE", failing)

    tap.rule("ATTITUDE", _message("ATTITUDE", boot_ms=1), True, None)
    lock.armed = False

    assert subscription.faults == 0, "the count did not fail: nothing tested"
    assert subscription.faulted, "a fault whose count failed read as none"


class _FailingNumbers(dict):
    """The tap's numbering, whose every store raises: a failed allocation."""

    def __setitem__(self, key: str, value: int) -> None:
        raise MemoryError("the tap could not number the ruling")


@pytest.mark.parametrize("subscribed", [False, True], ids=["untraced", "traced"])
def test_a_tap_error_outside_build_and_delivery_never_costs_the_message(
    subscribed: bool,
) -> None:
    """Containment, the numbering (review of the whole): an ordinary error
    outside the build and the deliveries stays in the tap and flags every
    subscription it kept from the ruling, and the router still publishes
    and dispatches the message, with no subscriber too. It escaped, and the
    admitted ATTITUDE reached neither the store nor a callback."""
    router, store, callbacks = _router()
    called: list[Any] = []
    callbacks.subscribe("*", called.append)
    heard: list[AdmissionRuling] = []
    subscription = (
        router.on_admission("ATTITUDE", heard.append) if subscribed else None
    )
    router.ingest(_message("ATTITUDE", boot_ms=10_000))
    router._tap._numbers = _FailingNumbers(router._tap._numbers)

    escaped = _escaped(lambda: router.ingest(_message("ATTITUDE", boot_ms=10_020)))

    assert escaped is None, f"{escaped!r} escaped into the router"
    assert store.latest("ATTITUDE").boot_time_ms == 10_020
    assert [message.time_boot_ms for message in called] == [10_000, 10_020]
    if subscription is not None:
        assert [ruling.ruling for ruling in heard] == [1]
        assert (subscription.faults, subscription.faulted) == (0, True)


class _LandAt:
    """A trace function that counts the lines run in the tap's module and
    raises ``interrupt`` on line ``landing`` of them, 0 on none: an
    interrupt landing on exactly that line. ``where`` is the function and
    line it landed on. Python stops tracing once a trace function raises."""

    def __init__(self, landing: int, interrupt: BaseException) -> None:
        self.landing = landing
        self.interrupt = interrupt
        self.lines = 0
        self.where = ""
        self._tap_file = sys.modules[AdmissionTap.__module__].__file__

    def __call__(self, frame: Any, event: str, arg: object) -> _LandAt | None:
        if frame.f_code.co_filename != self._tap_file:
            return None
        if event == "line":
            self.lines += 1
            if self.lines == self.landing:
                self.where = f"{frame.f_code.co_name}:{frame.f_lineno}"
                raise self.interrupt
        return self


def _one_ruling(
    lost: str, trace: _LandAt
) -> tuple[BaseException | None, bool, list[tuple[bool, int, bool]]] | None:
    """One ruling on a fresh tap, on a thread traced by ``trace``, owed to
    three subscriptions: the middle one's delivery fails, or the ruling
    cannot be built. None if the thread did not finish. Otherwise what
    escaped, whether the ruling was numbered, and for each subscription
    whether it got the ruling, its faults and its flag, all read without
    the tap's lock: an exception a trace function raises on a with
    statement's exit line skips __exit__ and leaves the lock held."""

    def stamp_of(message: object) -> int | None:
        if lost == "build":
            raise RuntimeError("no stamp")
        return 1

    def failing(ruling: AdmissionRuling) -> None:
        raise RuntimeError("the subscriber failed")

    tap = AdmissionTap(stamp_of, {"ATTITUDE"})
    got: list[int] = []
    owed = [
        tap.subscribe("ATTITUDE", lambda ruling: got.append(0)),
        tap.subscribe("ATTITUDE", failing),
        tap.subscribe("ATTITUDE", lambda ruling: got.append(2)),
    ]
    escaped: list[BaseException | None] = []

    def run() -> None:
        sys.settrace(trace)
        try:
            escaped.append(
                _escaped(lambda: tap.rule("ATTITUDE", object(), True, None))
            )
        finally:
            sys.settrace(None)

    runner = threading.Thread(target=run, daemon=True)
    runner.start()
    runner.join(WAIT_S)
    if runner.is_alive():
        return None
    return escaped[0], tap._numbers.get("ATTITUDE") == 1, [
        (index in got, subscription._faults, subscription.faulted)
        for index, subscription in enumerate(owed)
    ]


@pytest.mark.parametrize("lost", ["delivery", "build"])
def test_an_interrupt_on_any_line_of_a_ruling_hides_no_lost_one(
    lost: str,
) -> None:
    """Containment, an interrupt landing in the tap itself: one ruling for
    each line the tap runs, the interrupt raised on that line. The tap never
    hangs; once the ruling is numbered, every subscription got it or is
    flagged; none counts it twice; the interrupt is passed on. The review
    found one landing in a failed delivery's count, which left the
    subscription after it without the ruling and without a flag."""
    clean = _LandAt(0, SystemExit("never raised"))
    outcome = _one_ruling(lost, clean)
    assert outcome is not None and outcome[0] is None
    assert clean.lines, "the trace saw none of the tap's lines"

    wrong: list[str] = []
    numbered_landings = 0
    for landing in range(1, clean.lines + 1):
        interrupt = SystemExit(f"an interrupt on line {landing}")
        trace = _LandAt(landing, interrupt)
        outcome = _one_ruling(lost, trace)
        if outcome is None:
            wrong.append(f"{landing} ({trace.where}): the tap hung")
            continue
        escaped, numbered, owed = outcome
        numbered_landings += numbered
        if escaped is not interrupt:
            wrong.append(f"{landing} ({trace.where}): {escaped!r} came out")
        if numbered and not all(got or flag for got, _, flag in owed):
            wrong.append(f"{landing} ({trace.where}): lost unflagged: {owed}")
        if any(
            faults > 1 or (faults and not flag) for _, faults, flag in owed
        ):
            wrong.append(f"{landing} ({trace.where}): miscounted: {owed}")
    assert numbered_landings, "no landing came after the numbering"
    assert not wrong, "\n".join(wrong)


LAST = {"admitted": 10_020, "rejected": 9_900, "repeated_stamp": 10_000}


@pytest.mark.parametrize("last", sorted(LAST))
def test_a_lost_last_ruling_before_cancel_is_counted(last: str) -> None:
    """The ruling owed last is the easiest to lose unseen: no later one shows
    the gap. Owed 1..2, handed 1, one fault, whatever the verdict on the lost
    ruling; this is what D8.6 will read."""
    router, _, _ = _router()
    heard: list[AdmissionRuling] = []
    witnessed: list[AdmissionRuling] = []
    losing: list[bool] = []

    def subscriber(ruling: AdmissionRuling) -> None:
        if losing:
            raise RuntimeError("lost")
        heard.append(ruling)

    subscription = router.on_admission("ATTITUDE", subscriber)
    witness = router.on_admission("ATTITUDE", witnessed.append)
    router.ingest(_message("ATTITUDE", boot_ms=10_000))
    losing.append(True)
    router.ingest(_message("ATTITUDE", boot_ms=LAST[last]))
    subscription.cancel()
    router.ingest(_message("ATTITUDE", boot_ms=10_040))
    subscription.cancel()

    assert [(r.accepted, r.time_boot_ms) for r in witnessed[1:2]] == [
        (last != "rejected", LAST[last])
    ], "the lost ruling was not the one this case is about"
    assert (subscription.first, subscription.end) == (1, 2)
    assert [r.ruling for r in heard] == [1]
    assert (subscription.faults, subscription.faulted) == (1, True)
    assert (witness.faults, len(witnessed)) == (0, 3)


def test_subscribe_and_cancel_racing_the_rulings_never_misassign_one() -> None:
    """Numbering and the snapshot are one acquisition, and so are subscribe
    and cancel, so every subscription hears exactly first..end, in order,
    while rulings are made on another thread. The switch interval is cut so
    the two threads interleave finely."""
    claims_wanted = 500
    tap = AdmissionTap(message_boot_time_ms, {"ATTITUDE"})
    message = _message("ATTITUDE", boot_ms=1)
    done = threading.Event()
    claims: list[tuple[AdmissionSubscription, list[int]]] = []

    def rule_until_done() -> None:
        while not done.is_set():
            tap.rule("ATTITUDE", message, True, None)

    def churn() -> None:
        try:
            for _ in range(claims_wanted):
                heard: list[int] = []
                arrived = threading.Event()

                def subscriber(
                    ruling: AdmissionRuling,
                    heard: list[int] = heard,
                    arrived: threading.Event = arrived,
                ) -> None:
                    heard.append(ruling.ruling)
                    arrived.set()

                subscription = tap.subscribe("ATTITUDE", subscriber)
                if not arrived.wait(WAIT_S):
                    return
                subscription.cancel()
                claims.append((subscription, heard))
        finally:
            done.set()

    interval = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    try:
        ruler = threading.Thread(target=rule_until_done)
        churner = threading.Thread(target=churn)
        ruler.start()
        churner.start()
        churner.join(WAIT_S * 3)
        done.set()
        ruler.join(WAIT_S)
    finally:
        sys.setswitchinterval(interval)

    assert not churner.is_alive() and not ruler.is_alive()
    assert len(claims) == claims_wanted, "a subscription never heard a ruling"
    for subscription, heard in claims:
        assert heard == list(range(subscription.first, subscription.end + 1)), (
            f"owed {subscription.first}..{subscription.end}, heard {heard}"
        )


class _HookOnRelease:
    """A real lock that runs ``on_release`` once, on its first release after
    that is set: the moment between one acquisition and the next."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.on_release: Callable[[], object] | None = None

    def __enter__(self) -> bool:
        return self._lock.__enter__()

    def __exit__(self, *exc: object) -> None:
        self._lock.__exit__(*exc)
        hook, self.on_release = self.on_release, None
        if hook is not None:
            hook()


@pytest.mark.parametrize("act", ["subscribe", "cancel"])
def test_a_claim_changed_as_a_ruling_is_numbered_is_owed_exactly_its_own(
    act: str,
) -> None:
    """Numbering a ruling and taking the subscriptions owed it are one
    acquisition, so a subscribe or a cancel that runs the moment it ends is
    ordered after that ruling, never between its number and its snapshot.
    The race test above can only show this by chance; this shows it every
    time."""
    tap = AdmissionTap(message_boot_time_ms, {"ATTITUDE"})
    lock = tap._lock = _HookOnRelease()
    message = _message("ATTITUDE", boot_ms=1)
    heard: list[int] = []

    def listen(ruling: AdmissionRuling) -> None:
        heard.append(ruling.ruling)

    if act == "subscribe":
        claims: list[AdmissionSubscription] = []
        lock.on_release = lambda: claims.append(tap.subscribe("ATTITUDE", listen))
        tap.rule("ATTITUDE", message, True, None)
        tap.rule("ATTITUDE", message, True, None)
        (claim,) = claims
        claim.cancel()
        assert (claim.first, claim.end, heard) == (2, 2, [2])
    else:
        claim = tap.subscribe("ATTITUDE", listen)
        lock.on_release = claim.cancel
        tap.rule("ATTITUDE", message, True, None)
        tap.rule("ATTITUDE", message, True, None)
        assert (claim.first, claim.end, heard) == (1, 1, [1])


def test_with_no_subscriber_a_ruling_is_numbered_and_nothing_is_built() -> None:
    """The cost with no subscriber: a number, and not even a stamp read. The
    number still counts, so a later subscription starts after it."""
    reads: list[object] = []

    def stamp_of(message: object) -> int | None:
        reads.append(message)
        return 1

    tap = AdmissionTap(stamp_of, {"ATTITUDE"})
    tap.rule("ATTITUDE", object(), True, None)
    subscription = tap.subscribe("ATTITUDE", lambda ruling: None)

    assert reads == []
    assert subscription.first == 2


def test_the_tap_lock_is_free_while_a_ruling_is_built_and_delivered() -> None:
    """A leaf: never held while a ruling is built or delivered, so neither
    the stamp reader nor a subscriber runs under it."""
    held: list[bool] = []

    def stamp_of(message: object) -> int | None:
        held.append(tap._lock.locked())
        return 1

    tap = AdmissionTap(stamp_of, {"ATTITUDE"})
    tap.subscribe("ATTITUDE", lambda ruling: held.append(tap._lock.locked()))

    tap.rule("ATTITUDE", object(), True, None)

    assert held == [False, False]


def test_a_rejection_is_still_recorded_for_the_cadence_log(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The verdict now carries its reason to the tap too. The cadence log
    still records each rejection, with the same reason strings its readers
    parse."""
    recorded: list[tuple[int, str, str]] = []
    module = "navpy.modules.vehicle.pose_cadence_debug"
    monkeypatch.setattr(f"{module}.ENABLED", True)
    monkeypatch.setattr(
        f"{module}.record_telemetry_reject",
        lambda system, wall_s, message_type, reason: recorded.append(
            (system, message_type, reason)
        ),
    )
    for name in ("record_attitude_arrival", "record_position_arrival"):
        monkeypatch.setattr(f"{module}.{name}", lambda *args: None)
    router, _, _ = _router()
    router.on_admission("ATTITUDE", lambda ruling: None)

    for message in _sequence():
        router.ingest(message)

    assert recorded == [
        (TARGET, "ATTITUDE", "stale_boot"),
        (TARGET, "ATTITUDE", "foreign_component"),
    ]


def test_a_subscriber_gets_plain_values_and_never_the_message() -> None:
    router, _, _ = _router()
    heard: list[AdmissionRuling] = []
    router.on_admission("ATTITUDE", heard.append)
    message = _message("ATTITUDE", boot_ms=10_000)

    router.ingest(message)

    (ruling,) = heard
    assert vars(ruling) == {
        "message_type": "ATTITUDE",
        "ruling": 1,
        "time_boot_ms": 10_000,
        "accepted": True,
        "reason": None,
    }
    assert not any(value is message for value in _reachable(ruling))
    with pytest.raises(dataclasses.FrozenInstanceError):
        ruling.accepted = False  # type: ignore[misc]


def test_the_runtime_contract_default_is_none_for_every_other_vehicle() -> None:
    class _OtherVehicle(VehicleMessaging):
        def register_rc_channel(self, rc_channel, rc_channel_receiver):
            return MagicMock()

        def on_message(self, message_name, callback):
            return MagicMock()

        def send_mavlink_message(self, mav_msg, *, source_component=None):
            return None

        def send_status_text(self, msg):
            return None

    assert "on_admission" not in VehicleMessaging.__abstractmethods__
    assert _OtherVehicle().on_admission("ATTITUDE", lambda ruling: None) is None


def test_vehicle_mav_hands_out_a_live_subscription_numbered_from_first() -> None:
    """Through the real boundary, as the review asked. A contract default
    that shadowed the facet would hand out None here, which a test of the
    owner or the router alone could not see."""
    vehicle = VehicleMav(
        "unused",
        target_system=7,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        send_heartbeat=False,
        bus=_construction_bus(),
    )
    try:
        heard: list[AdmissionRuling] = []
        vehicle.feed_message(_message("ATTITUDE", system=7, boot_ms=10_000))
        subscription = vehicle.on_admission("ATTITUDE", heard.append)

        assert isinstance(subscription, AdmissionSubscription)
        assert (subscription.first, subscription.end) == (2, None)
        vehicle.feed_message(_message("ATTITUDE", system=7, boot_ms=10_020))
        subscription.cancel()
        vehicle.feed_message(_message("ATTITUDE", system=7, boot_ms=10_040))

        assert heard == [AdmissionRuling("ATTITUDE", 2, 10_020, True, None)]
        assert subscription.end == 2
    finally:
        vehicle.close()
