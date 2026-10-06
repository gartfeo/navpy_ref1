"""D2: the ATTITUDE admission ledger, and the worker pass that samples it (D3).

Every ruling the router makes on ATTITUDE reaches ``append`` through its
admission tap, admissions and rejections alike. The entries are held in order
and bounded, and every figure a reader uses is derived from them, so none can
disagree with the entries it describes. The one-section rule every store
keeps (D5) is pinned for the ledger in test_determinism_seal.py.
"""

from __future__ import annotations

import dataclasses
from types import SimpleNamespace

import pytest

from navpy.modules.vehicle.admission_tap import AdmissionRuling, AdmissionTap
from navpy.modules.vehicle.inbound_router import (
    AUTOPILOT_TELEMETRY_TYPES,
    REJECT_FOREIGN_COMPONENT,
    REJECT_STALE_BOOT,
    message_boot_time_ms,
)
from navpy.modules.vision.sim.determinism_admission_ledger import (
    AdmissionLedger,
    PassObserver,
)
from navpy.modules.vision.sim.determinism_command_log import CommandLoopLog


def _ruling(
    number: int,
    stamp: int | None,
    *,
    accepted: bool = True,
    reason: str | None = None,
) -> AdmissionRuling:
    return AdmissionRuling("ATTITUDE", number, stamp, accepted, reason)


def _stale(number: int, stamp: int | None) -> AdmissionRuling:
    return _ruling(number, stamp, accepted=False, reason=REJECT_STALE_BOOT)


def test_every_ruling_is_held_in_order_as_plain_values() -> None:
    """Admissions and rejections alike, each as (ruling, stamp, accepted,
    reason), converted: a reader joins rows to these numbers."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 1_000))
    ledger.append(_stale(2, 980))
    ledger.append(SimpleNamespace(
        ruling=3.0, time_boot_ms=1_020.0, accepted=1, reason=None
    ))

    capture = ledger.capture()
    assert capture.entries == (
        (1, 1_000, True, None),
        (2, 980, False, REJECT_STALE_BOOT),
        (3, 1_020, True, None),
    )
    for number, stamp, accepted, _ in capture.entries:
        assert (type(number), type(stamp), type(accepted)) == (int, int, bool)
    assert (capture.observed, capture.admitted) == (3, 3)
    assert not capture.incomplete


def test_the_cutoff_names_the_latest_ruling_observed_and_admitted() -> None:
    """The two live figures a reader on another thread needs while the leg
    runs, from one acquisition: None until the ledger holds such a ruling."""
    ledger = AdmissionLedger(8)
    assert ledger.cutoff() == (None, None)
    ledger.append(_stale(1, 980))
    assert ledger.cutoff() == (1, None)
    ledger.append(_ruling(2, 1_000))
    assert ledger.cutoff() == (2, 2)
    ledger.append(_ruling(3, None, accepted=False,
                          reason=REJECT_FOREIGN_COMPONENT))
    assert ledger.cutoff() == (3, 2)
    ledger.append(_ruling(4, 1_020))
    assert ledger.cutoff() == (4, 4)


def test_an_overflow_drops_entries_and_never_the_rulings_made() -> None:
    """Bounded like the other stores: the EARLIEST entries are kept, the
    rest counted, and the live figures still advance past the budget."""
    ledger = AdmissionLedger(2)
    for number in (1, 2, 3, 4):
        ledger.append(_ruling(number, 1_000 + 20 * number))

    capture = ledger.capture()
    assert [entry[0] for entry in capture.entries] == [1, 2]
    assert capture.dropped == 2
    assert capture.overflowed and capture.incomplete
    assert (capture.observed, capture.admitted) == (4, 4)
    assert ledger.cutoff() == (4, 4)


def test_a_zero_capacity_ledger_keeps_only_its_cutoff() -> None:
    ledger = AdmissionLedger(0)
    ledger.append(_ruling(1, 1_000))

    capture = ledger.capture()
    assert (capture.entries, capture.dropped, capture.admitted) == ((), 1, 1)


def test_a_ruling_missing_between_two_held_is_a_gap_and_a_hole() -> None:
    """The tap numbers every ruling, so a number skipped between two the
    ledger holds is one it was owed and never got."""
    ledger = AdmissionLedger(8)
    for number in (1, 2, 4):
        ledger.append(_ruling(number, 1_000 + 20 * number))

    capture = ledger.capture()
    assert capture.gapped and capture.incomplete
    assert not (capture.overflowed or capture.failed)


def test_rejections_and_stampless_admissions_are_read_off_the_entries() -> (
    None
):
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 1_000))
    ledger.append(_ruling(2, None))
    ledger.append(_stale(3, 990))
    ledger.append(_ruling(4, None, accepted=False,
                          reason=REJECT_FOREIGN_COMPONENT))
    ledger.append(_stale(5, 995))

    capture = ledger.capture()
    assert capture.rejections == {
        REJECT_STALE_BOOT: 2, REJECT_FOREIGN_COMPONENT: 1
    }
    # Admitted with no stamp: no clock label can place it. A rejection
    # without one is not counted, since nothing acted on it.
    assert capture.stampless == 1


def test_the_stamps_a_label_reads_come_from_admitted_entries_only() -> None:
    """A rejected ruling never reached the store, so its stamp labels
    nothing, however high it is."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 1_000))
    ledger.append(_ruling(2, 5_000, accepted=False,
                          reason=REJECT_FOREIGN_COMPONENT))
    ledger.append(_ruling(3, 1_020))
    ledger.append(_ruling(4, 9_000, accepted=False,
                          reason=REJECT_FOREIGN_COMPONENT))

    capture = ledger.capture()
    assert (capture.admitted_stamp, capture.max_admitted_stamp) == (
        1_020, 1_020
    ), "the latest ruling was rejected, so ruling 3's stamp is the label"

    ledger.append(_ruling(5, None))
    capture = ledger.capture()
    assert capture.admitted_stamp is None, "the latest admitted had no stamp"
    assert capture.max_admitted_stamp == 1_020


def test_an_admitted_step_back_is_a_discontinuity_and_not_a_hole() -> None:
    """D9: the router admits a lower stamp only as a reboot. A rejected
    lower stamp is no step back, since the router refused it."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 70_000))
    ledger.append(_stale(2, 69_990))
    assert ledger.capture().discontinuous is False

    ledger.append(_ruling(3, 1_000))

    capture = ledger.capture()
    assert capture.discontinuous is True
    assert (capture.admitted_stamp, capture.max_admitted_stamp) == (
        1_000, 70_000
    )
    assert not capture.incomplete, "a reboot is a fact about the run"


def test_a_ruling_that_cannot_be_read_is_a_fault_and_changes_nothing() -> (
    None
):
    """Built whole before anything moves, so neither entry nor cutoff
    changes, and the ledger is failed: the tap never hears of it."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 1_000))

    ledger.append(SimpleNamespace(
        ruling="two", time_boot_ms=1_020, accepted=True, reason=None
    ))
    ledger.append(SimpleNamespace(ruling=3, time_boot_ms=1_040, accepted=True))

    capture = ledger.capture()
    assert capture.entries == ((1, 1_000, True, None),)
    assert (capture.observed, capture.admitted) == (1, 1)
    assert capture.failed and capture.incomplete


def test_a_capture_is_frozen_and_the_ledger_takes_no_new_attribute() -> None:
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1, 1_000))
    capture = ledger.capture()

    with pytest.raises(dataclasses.FrozenInstanceError):
        capture.dropped = 5  # type: ignore[misc]
    ledger.append(_ruling(2, 1_020))
    assert len(capture.entries) == 1, "a capture changed after it was taken"
    assert not hasattr(ledger, "__dict__")
    with pytest.raises(AttributeError):
        ledger.extra = object()  # type: ignore[attr-defined]


def test_every_attitude_ruling_through_a_real_tap_reaches_the_ledger() -> (
    None
):
    """As the router feeds it: from the first ruling after the subscription,
    admissions and rejections alike, other types not at all, and the boot
    stamp as the router reads it."""
    ledger = AdmissionLedger(8)
    tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)
    tap.rule("ATTITUDE", SimpleNamespace(time_boot_ms=900), True, None)
    subscription = tap.subscribe("ATTITUDE", ledger.append)

    tap.rule("ATTITUDE", SimpleNamespace(time_boot_ms=1_000), True, None)
    tap.rule("VFR_HUD", SimpleNamespace(), True, None)
    tap.rule(
        "ATTITUDE", SimpleNamespace(time_boot_ms=980), False,
        REJECT_STALE_BOOT,
    )
    tap.rule(
        "ATTITUDE", SimpleNamespace(time_boot_ms=2**32 + 1_040), True, None
    )
    subscription.cancel()

    capture = ledger.capture()
    assert capture.entries == (
        (2, 1_000, True, None),
        (3, 980, False, REJECT_STALE_BOOT),
        (4, 1_040, True, None),
    )
    assert (
        subscription.first, subscription.end,
        subscription.faults, subscription.faulted,
    ) == (2, 4, 0, False)
    assert not capture.incomplete


class _WatchedPasses:
    """A command log stand-in: each pass, with whether the ledger's lock was
    held when it arrived."""

    def __init__(self, ledger: AdmissionLedger) -> None:
        self._ledger = ledger
        self.passes: list[tuple] = []
        self.commands: list[tuple] = []

    def note_pass(
        self, iteration: int, observed: int | None, admitted: int | None
    ) -> None:
        self.passes.append(
            (iteration, observed, admitted, self._ledger._lock.locked())
        )

    def note_command(
        self, iteration: int, command: object, raised: bool = False
    ) -> None:
        self.commands.append((iteration, command, raised))


def test_a_pass_samples_the_cutoff_and_releases_the_ledger_first() -> None:
    """D3: ONE cutoff, released before the command log is called, so the
    two locks are never held together and the sample crosses as values."""
    ledger = AdmissionLedger(8)
    log = _WatchedPasses(ledger)
    observer = PassObserver(ledger, log)

    observer.note_iteration(1)
    ledger.append(_ruling(1, 1_000))
    ledger.append(_stale(2, 980))
    observer.note_iteration(2)
    observer.note_command(2, "command", True)

    assert log.passes == [(1, None, None, False), (2, 2, 1, False)]
    assert log.commands == [(2, "command", True)]
    assert not hasattr(observer, "__dict__")


def test_the_rulings_between_two_passes_are_read_off_their_samples() -> None:
    """What the samples are for: a pass could act only on rulings made by
    the time it began, so the rulings new to it are those after the last
    pass's observed cutoff, up to its own. A rejection and an admission
    landing between two passes are placed exactly, and the pass's label is
    the latest admitted when it began (Q3)."""
    ledger = AdmissionLedger(16)
    log = CommandLoopLog(16)
    observer = PassObserver(ledger, log)
    ledger.append(_ruling(1, 1_000))
    observer.note_iteration(1)
    ledger.append(_stale(2, 990))
    ledger.append(_ruling(3, 1_020))
    observer.note_iteration(2)
    observer.note_iteration(3)

    passes = log.capture().passes
    assert passes == ((1, 1, 1), (2, 3, 3), (3, 3, 3))
    entries = {entry[0]: entry for entry in ledger.capture().entries}
    assert [
        entries[number] for number in range(passes[0][1] + 1, passes[1][1] + 1)
    ] == [(2, 990, False, REJECT_STALE_BOOT), (3, 1_020, True, None)]
    assert passes[2][1] == passes[1][1], "no ruling came between these two"
