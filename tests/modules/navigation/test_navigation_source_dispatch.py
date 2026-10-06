"""Single-owner source callback tests."""

from __future__ import annotations

from typing import get_type_hints

import pytest

from navpy.modules.navigation.navigation_composition import NavigationComposition
from navpy.modules.navigation.navigation_source_dispatch import (
    BindSourceDispatch,
    NavigationSourceDispatchSlot,
)


def test_the_composition_names_the_binder_it_holds() -> None:
    """Review of the whole: the field's annotation named a type its module
    never imported, so resolving NavigationComposition's hints raised
    NameError."""
    hints = get_type_hints(NavigationComposition)

    assert hints["bind_source_dispatch"] is BindSourceDispatch


def test_unbound_source_slot_has_no_work() -> None:
    assert NavigationSourceDispatchSlot().dispatch_available() is False


def test_bound_source_callback_is_forwarded() -> None:
    calls: list[str] = []
    slot = NavigationSourceDispatchSlot()
    slot.bind(lambda: calls.append("source") or True)

    assert slot.dispatch_available() is True
    assert calls == ["source"]


def test_source_slot_rejects_a_second_owner() -> None:
    slot = NavigationSourceDispatchSlot()
    slot.bind(lambda: False)

    with pytest.raises(RuntimeError, match="already bound"):
        slot.bind(lambda: True)


def test_an_unbound_slot_swallows_the_observer_calls() -> None:
    """The worker publishes every pass whether a source bound one or not."""
    slot = NavigationSourceDispatchSlot()

    slot.note_iteration(1)
    slot.note_command(1, object(), True)


def test_a_bound_observer_receives_the_iteration_and_the_command() -> None:
    seen: list[tuple] = []

    class _Observer:
        def note_iteration(self, iteration: int) -> None:
            seen.append(("iteration", iteration))

        def note_command(self, iteration, command, raised=False) -> None:
            seen.append(("command", iteration, command, raised))

    slot = NavigationSourceDispatchSlot()
    slot.bind(lambda: True, _Observer())

    slot.note_iteration(7)
    slot.note_command(7, "cmd")
    slot.note_command(8, None, True)

    assert seen == [
        ("iteration", 7),
        ("command", 7, "cmd", False),
        ("command", 8, None, True),
    ]


def test_the_observer_is_bound_with_the_callback_and_not_apart_from_it() -> None:
    """One owner, one bind: an observer cannot outlive the source it came with."""
    observer = object()
    slot = NavigationSourceDispatchSlot()
    slot.bind(lambda: False, observer)

    with pytest.raises(RuntimeError, match="already bound"):
        slot.bind(lambda: True, object())
