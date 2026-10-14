"""MessageStore keeps history only for the types its policy declares.

docs/design/message_history.md: a 1024-deep history for every received type
grew the live heap until full garbage collections froze every thread for
90-220 ms. Only the types read through `wait_after`/`recent` keep history.
"""
from __future__ import annotations

import ast
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from navpy.core.ports.pose_stream_rate import ATTITUDE_HISTORY_COUNT
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import (
    MESSAGE_HISTORY_POLICY,
    RESPONSE_HISTORY_DEPTH,
)

HISTORY_READERS = frozenset({"wait_after", "recent"})
REPO_ROOT = Path(__file__).resolve().parents[3]
# Everything that can obtain a vehicle's MessageStore.
SCAN_ROOTS = ("src", "scripts")
# Reviewed uses of the same method names on objects that are not a
# MessageStore: (file, receiver). MissionInbox.wait_after(cursor, ...) takes no
# message type.
NOT_MESSAGE_STORE = frozenset({
    ("src/navpy/plugins/vehicle/ardupilot/mission_upload.py", "self._inbox"),
})


def _fill(store: MessageStore, message_type: str, count: int) -> None:
    for index in range(count):
        store.publish(
            message_type,
            SimpleNamespace(index=index),
            receipt_time_s=float(index),
        )


def test_vehicle_policy_declares_response_types_and_attitude_depths():
    assert dict(MESSAGE_HISTORY_POLICY) == {
        "PARAM_VALUE": RESPONSE_HISTORY_DEPTH,
        "MISSION_COUNT": RESPONSE_HISTORY_DEPTH,
        "MISSION_ITEM": RESPONSE_HISTORY_DEPTH,
        "MISSION_ITEM_INT": RESPONSE_HISTORY_DEPTH,
        "HOME_POSITION": RESPONSE_HISTORY_DEPTH,
        "TERRAIN_REPORT": RESPONSE_HISTORY_DEPTH,
        "ATTITUDE": ATTITUDE_HISTORY_COUNT,
    }


@pytest.mark.parametrize("message_type", sorted(MESSAGE_HISTORY_POLICY))
def test_declared_type_keeps_exactly_its_declared_depth(message_type):
    store = MessageStore(MESSAGE_HISTORY_POLICY)
    depth = MESSAGE_HISTORY_POLICY[message_type]
    _fill(store, message_type, depth + 5)

    kept = store.recent(message_type, depth + 5)

    assert len(kept) == depth
    assert [sample.message.index for sample in kept] == list(
        range(5, depth + 5)
    )


def test_undeclared_high_rate_type_keeps_only_its_latest_sample():
    store = MessageStore(MESSAGE_HISTORY_POLICY)
    _fill(store, "SIM_STATE", 500)

    assert store.latest("SIM_STATE").message.index == 499
    assert store.message("SIM_STATE").index == 499
    assert store.cursor("SIM_STATE") == store.cursor()
    with pytest.raises(ValueError, match="SIM_STATE"):
        store.recent("SIM_STATE", 10)


def test_unknown_type_still_publishes_and_advances_the_generation():
    store = MessageStore({})
    first = store.publish("NEW_TYPE", object(), receipt_time_s=1.0)
    second = store.publish("NEW_TYPE", object(), receipt_time_s=2.0)

    assert second.generation == first.generation + 1
    assert store.latest("NEW_TYPE") is second


def test_recent_on_undeclared_type_raises_even_for_zero_count():
    store = MessageStore({"ATTITUDE": 4})

    with pytest.raises(ValueError, match="VFR_HUD"):
        store.recent("VFR_HUD", 0)


@pytest.mark.parametrize(
    "message_types",
    ["VFR_HUD", ("VFR_HUD",), ("PARAM_VALUE", "VFR_HUD")],
    ids=["single", "tuple", "mixed"],
)
def test_wait_after_on_undeclared_type_raises_at_the_call(message_types):
    store = MessageStore({"PARAM_VALUE": 8})
    store.publish("VFR_HUD", object(), receipt_time_s=1.0)
    started_s = time.monotonic()

    with pytest.raises(ValueError, match="VFR_HUD"):
        store.wait_after(
            message_types,
            0,
            lambda _message: True,
            deadline=time.monotonic() + 5.0,
        )

    # Raised before waiting, not after the deadline.
    assert time.monotonic() - started_s < 1.0


def test_policy_is_copied_at_construction():
    policy = {"PARAM_VALUE": 2}
    store = MessageStore(policy)
    policy["PARAM_VALUE"] = 100
    policy["VFR_HUD"] = 100
    _fill(store, "PARAM_VALUE", 5)

    assert len(store.recent("PARAM_VALUE", 10)) == 2
    with pytest.raises(ValueError):
        store.recent("VFR_HUD", 1)


@pytest.mark.parametrize(
    "policy",
    [{"PARAM_VALUE": 0}, {"PARAM_VALUE": -1}, {"PARAM_VALUE": True},
     {"PARAM_VALUE": 2.0}, {"": 4}],
)
def test_invalid_policy_is_rejected(policy):
    with pytest.raises(ValueError):
        MessageStore(policy)


def test_overrun_compares_against_each_types_own_depth():
    from navpy.plugins.vehicle.ardupilot.message_store import (
        MessageHistoryOverrun,
    )

    store = MessageStore({"SHALLOW": 2, "DEEP": 8})
    cursor = store.cursor()
    _fill(store, "SHALLOW", 3)
    _fill(store, "DEEP", 3)

    with pytest.raises(MessageHistoryOverrun):
        store.wait_after(
            "SHALLOW", cursor, lambda _m: False, deadline=time.monotonic()
        )
    assert store.wait_after(
        "DEEP", cursor, lambda _m: False, deadline=time.monotonic()
    ) is None


def _literal_types(node: ast.expr) -> tuple[str, ...] | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return (node.value,)
    if isinstance(node, (ast.Tuple, ast.List)) and all(
        isinstance(item, ast.Constant) and isinstance(item.value, str)
        for item in node.elts
    ):
        return tuple(item.value for item in node.elts)
    return None


def _scan_history_reads(
    source: str, where: str
) -> tuple[dict[str, list[str]], list[str], set[tuple[str, str]]]:
    """Fail-closed scan of one module's `recent`/`wait_after` uses.

    Returns the literal message types read (type -> places), every use the
    scan cannot resolve to literal types (an alias, `getattr`, a non-literal
    type argument), and the reviewed exclusions it matched. Any receiver
    counts: the scan does not guess which objects are a MessageStore.
    """
    tree = ast.parse(source)
    call_of = {
        id(node.func): node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
    }
    reads: dict[str, list[str]] = {}
    unresolved: list[str] = []
    excluded: set[tuple[str, str]] = set()
    for node in ast.walk(tree):
        place = f"{where}:{getattr(node, 'lineno', '?')}"
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "getattr"
            and len(node.args) >= 2
            and isinstance(node.args[1], ast.Constant)
            and node.args[1].value in HISTORY_READERS
        ):
            unresolved.append(f"{place} {ast.unparse(node)}")
            continue
        if not (
            isinstance(node, ast.Attribute) and node.attr in HISTORY_READERS
        ):
            continue
        exclusion = (where, ast.unparse(node.value))
        if exclusion in NOT_MESSAGE_STORE:
            excluded.add(exclusion)
            continue
        call = call_of.get(id(node))
        types = (
            _literal_types(call.args[0])
            if call is not None and call.args else None
        )
        if types is None:
            unresolved.append(f"{place} {ast.unparse(node)}")
            continue
        for message_type in types:
            reads.setdefault(message_type, []).append(place)
    return reads, unresolved, excluded


def _scan_repository() -> tuple[
    dict[str, list[str]], list[str], set[tuple[str, str]], set[str]
]:
    reads: dict[str, list[str]] = {}
    unresolved: list[str] = []
    excluded: set[tuple[str, str]] = set()
    scanned: set[str] = set()
    for scan_root in SCAN_ROOTS:
        for path in sorted((REPO_ROOT / scan_root).rglob("*.py")):
            where = path.relative_to(REPO_ROOT).as_posix()
            scanned.add(where)
            file_reads, file_unresolved, file_excluded = _scan_history_reads(
                path.read_text(encoding="utf-8"), where
            )
            for message_type, places in file_reads.items():
                reads.setdefault(message_type, []).extend(places)
            unresolved.extend(file_unresolved)
            excluded |= file_excluded
    return reads, unresolved, excluded, scanned


def test_every_history_read_in_src_and_scripts_is_declared():
    reads, unresolved, excluded, scanned = _scan_repository()

    # Coverage: nested packages and scripts that can reach the store.
    assert {
        "src/navpy/plugins/vehicle/ardupilot/pose_telemetry.py",
        "src/navpy/plugins/vehicle/ardupilot/mission_download.py",
        "scripts/scratch_plant_id_child.py",
    } <= scanned
    # The scan really finds the known consumers (guards the guard).
    assert {"PARAM_VALUE", "MISSION_COUNT", "ATTITUDE", "TERRAIN_REPORT"} <= (
        set(reads)
    )
    assert unresolved == []
    assert {
        message_type: places
        for message_type, places in reads.items()
        if message_type not in MESSAGE_HISTORY_POLICY
    } == {}
    # Every reviewed exclusion still names a real use; drop stale ones.
    assert excluded == set(NOT_MESSAGE_STORE)


@pytest.mark.parametrize(
    "source",
    [
        "cache.recent(kind, 4)",
        "read = self._messages.recent\nread('ATTITUDE', 4)",
        "read = self._messages.wait_after\nread(types, 0, p, deadline=1.0)",
        "reader = getattr(store, 'recent')",
        "self._messages.wait_after(types, cursor, p, deadline=1.0)",
        "self._inbox.wait_after(cursor, p, deadline=1.0)",
    ],
    ids=[
        "other-name", "alias-recent", "alias-wait", "getattr",
        "non-literal-types", "exclusion-is-per-file",
    ],
)
def test_guard_fails_closed_on_unresolvable_history_reads(source):
    reads, unresolved, excluded = _scan_history_reads(source, "synthetic.py")

    assert unresolved
    assert reads == {}
    assert excluded == set()


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        ("cache.recent('VFR_HUD', 4)", {"VFR_HUD"}),
        (
            "self._messages.wait_after(('PARAM_VALUE', 'X'), c, p, deadline=1)",
            {"PARAM_VALUE", "X"},
        ),
    ],
    ids=["any-receiver", "tuple"],
)
def test_guard_reads_literal_types_from_any_receiver(source, expected):
    reads, unresolved, _excluded = _scan_history_reads(source, "synthetic.py")

    assert unresolved == []
    assert set(reads) == expected
