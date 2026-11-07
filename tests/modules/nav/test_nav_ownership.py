"""NavPy owns the flight mode only after a fresh GUIDED heartbeat newer than its own request
(docs/design/navpy_recovery.md, step 3): each acquire and revoke path, and no automatic mode
request without ownership."""
from __future__ import annotations

import pytest

from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership, Ownership, OwnershipEnd
from navpy.core.ports.flight_mode import FlightMode

G, L, A = FlightMode.GUIDED, FlightMode.LOITER, FlightMode.AUTO


def hb(count, mode, *, armed=True, fresh=True):
    return ModeObservation(heartbeat_count=count, mode=mode, armed=armed, fresh=fresh)


def owned():
    """Acquired: NavPy asked for GUIDED at heartbeat 10, heartbeat 11 shows it."""
    record = NavOwnership()
    record.requested(G, 10)
    record.observe(hb(11, G))
    assert record.state is Ownership.OWNED
    return record


def test_not_owned_without_any_request():
    record = NavOwnership()
    record.observe(hb(1, G))   # GUIDED set by someone else (GCS)
    assert record.state is Ownership.NOT_OWNED
    assert not record.may_request_mode()


def test_acquired_on_a_guided_heartbeat_newer_than_the_request():
    record = NavOwnership()
    record.requested(G, 10)
    record.observe(hb(10, G))  # not newer than the request: proves nothing
    assert record.state is Ownership.NOT_OWNED
    record.observe(hb(11, G))
    assert record.state is Ownership.OWNED
    assert record.may_request_mode()


@pytest.mark.parametrize("observation", [hb(11, A), hb(11, G, armed=False), hb(11, G, fresh=False)])
def test_not_acquired_without_fresh_armed_guided(observation):
    record = NavOwnership()
    record.requested(G, 10)
    record.observe(observation)
    assert record.state is Ownership.NOT_OWNED


@pytest.mark.parametrize("mode", [FlightMode.MANUAL, FlightMode.FBWA, FlightMode.RTL, L, A, None])
def test_revoked_by_a_mode_navpy_did_not_ask_for(mode):
    record = owned()
    record.observe(hb(12, mode))
    assert record.state is Ownership.NOT_OWNED
    assert record.ended is OwnershipEnd.REVOKED_MODE
    assert not record.may_request_mode()


def test_revoked_by_disarm():
    record = owned()
    record.observe(hb(12, G, armed=False))
    assert record.ended is OwnershipEnd.REVOKED_DISARM
    assert not record.may_request_mode()


def test_hold_loiter_after_navpys_request_stays_owned():
    record = owned()
    record.requested(L, 12)
    record.observe(hb(12, G))   # the request not taken yet
    record.observe(hb(13, L))
    assert record.state is Ownership.OWNED


def test_loiter_shown_before_navpys_request_is_someone_elses():
    record = owned()
    record.observe(hb(12, L))   # processed late: the pilot's LOITER came first
    record.requested(L, 12)
    assert record.state is Ownership.NOT_OWNED
    assert record.ended is OwnershipEnd.REVOKED_MODE


def test_handed_over_on_auto_newer_than_navpys_auto_request():
    record = owned()
    record.requested(L, 12)
    record.observe(hb(13, L))
    record.requested(A, 13)
    record.observe(hb(14, A))
    assert record.state is Ownership.NOT_OWNED
    assert record.ended is OwnershipEnd.HANDED_OVER
    assert not record.may_request_mode()


def test_stale_heartbeat_makes_ownership_unknown_then_fresh_decides():
    record = owned()
    record.observe(hb(11, G, fresh=False))
    assert record.state is Ownership.UNKNOWN
    assert not record.may_request_mode()
    record.observe(hb(12, G))
    assert record.state is Ownership.OWNED

    record.observe(hb(12, G, fresh=False))
    record.observe(hb(13, FlightMode.MANUAL))   # the pilot took over during the outage
    assert record.ended is OwnershipEnd.REVOKED_MODE


def test_no_newer_heartbeat_changes_nothing():
    record = owned()
    record.observe(hb(11, FlightMode.MANUAL))   # same count as the acquiring heartbeat: already seen
    assert record.state is Ownership.OWNED


def test_ending_needs_a_new_guided_request_to_own_again():
    record = owned()
    record.observe(hb(12, FlightMode.MANUAL))
    record.observe(hb(13, G))   # the pilot gives GUIDED back: not NavPy's request
    assert record.state is Ownership.NOT_OWNED
    record.requested(G, 13)
    record.observe(hb(14, G))
    assert record.state is Ownership.OWNED
    assert record.ended is None


@pytest.mark.parametrize("lapse", [hb(11, FlightMode.FBWA), hb(11, A, armed=False)])
def test_a_guided_request_lapses_on_another_mode_or_a_disarm(lapse):
    """Codex finding 5 on steps 1-4: NavPy asked for GUIDED from AUTO; the pilot set FBWA (or disarmed); the
    operator's later GUIDED is not NavPy's."""
    record = NavOwnership()
    record.requested(G, 10, A)
    record.observe(lapse)
    record.observe(hb(15, G))
    assert record.state is Ownership.NOT_OWNED


def test_a_guided_request_survives_a_heartbeat_still_showing_the_old_mode():
    """The heartbeat after the request may still show the mode NavPy asked from: that is no lapse."""
    record = NavOwnership()
    record.requested(G, 10, A)
    record.observe(hb(11, A))
    record.observe(hb(12, G))
    assert record.state is Ownership.OWNED


def test_a_withdrawn_guided_request_does_not_claim_a_later_guided():
    record = NavOwnership()
    record.requested(G, 10, A)
    record.withdraw(G)   # NavPy stopped waiting (GUIDED not accepted, task cleared)
    record.observe(hb(15, G))
    assert record.state is Ownership.NOT_OWNED


def test_withdraw_leaves_an_owned_mode_owned():
    record = owned()
    record.withdraw(G)
    record.observe(hb(12, G))
    assert record.state is Ownership.OWNED
