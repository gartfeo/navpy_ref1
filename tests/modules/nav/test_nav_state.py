from __future__ import annotations

from unittest.mock import Mock

from navpy.modules.nav.detection_snapshot import DetectionSnapshot


def test_acknowledging_event_lease_preserves_later_append() -> None:
    snapshot = DetectionSnapshot()
    old_event = Mock(name="old_event")
    new_event = Mock(name="new_event")
    snapshot.replace([], None, [old_event], append_events=False)
    lease = snapshot.lease_events()

    snapshot.replace([], None, [new_event], append_events=True)

    assert snapshot.ack_events(lease)
    assert snapshot.events() == [new_event]


def test_stale_event_lease_cannot_ack_replaced_generation() -> None:
    snapshot = DetectionSnapshot()
    old_event = Mock(name="old_event")
    new_event = Mock(name="new_event")
    snapshot.replace([], None, [old_event], append_events=False)
    lease = snapshot.lease_events()

    snapshot.replace([], None, [new_event], append_events=False)

    assert not snapshot.ack_events(lease)
    assert snapshot.events() == [new_event]


def test_selection_is_an_immutable_generation_consistent_view() -> None:
    snapshot = DetectionSnapshot()
    first = Mock(name="first_target")
    second = Mock(name="second_target")
    snapshot.replace_selection([first], first)

    selection = snapshot.selection()
    snapshot.replace_selection([second], second)

    assert selection.targets == (first,)
    assert selection.primary_target is first
    assert snapshot.selection().targets == (second,)
    assert snapshot.selection().primary_target is second
