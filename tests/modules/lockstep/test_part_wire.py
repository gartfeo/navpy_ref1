"""NVPART01 golden vectors: every message and the listed protocol cases.

Each vector is the 11-byte header (magic ``NVPART01``, version 1, type) and
the canonical JSON body, spelled out so a change to either is a visible diff.
"""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.part_wire import (
    Done,
    Event,
    Lifecycle,
    PartWireError,
    Register,
    Request,
    Response,
    SpawnRequest,
    StopRequest,
    Tick,
    check_tick,
    decode,
    encode,
)

REGISTER_HEADER = bytes.fromhex("4e56504152543031" "0100" "01")
DONE_HEADER = bytes.fromhex("4e56504152543031" "0100" "02")
TICK_HEADER = bytes.fromhex("4e56504152543031" "0100" "03")

GOLDEN = {
    "register": (
        Register("9f86d081", "rover", 1, (2, 3), 1),
        REGISTER_HEADER + b'{"capability":"9f86d081","generation":1,"ordinal":1,"role":"rover",'
        b'"spawn_id":[2,3]}',
    ),
    "tick": (
        Tick(7, 140000, (("lockstep:serial0", b"\xfe\x01"),), (Request((0, 4), {"op": "echo"}),),
             (Lifecycle("spawn", (0, 9), (2, 3), 1, 1, "ok", None),)),
        TICK_HEADER + b'{"cancels":[],"end":false,"k":7,"lifecycle":[{"generation":1,"id":[0,9],'
        b'"kind":"spawn","ordinal":1,"payload":null,"request":[2,3],"result":"ok"}],"ports":'
        b'[["lockstep:serial0",'
        b'"fe01"]],"requests":[{"id":[0,4],"payload":{"op":"echo"}}],"t_us":140000}',
    ),
    "done": (
        Done(7, (("lockstep:serial0", b"\x02"),), (Response((0, 4), {"ok": True}),),
             (Event("runtime_ready", {"items": 4}),), (SpawnRequest((2, 5), "rover", 1),),
             (StopRequest((2, 6), 1, 1),), ("SNAP(VISION-NAV) at 7",)),
        DONE_HEADER + b'{"ended":null,"events":[{"kind":"runtime_ready","payload":{"items":4}}],'
        b'"faults":[],"k":7,"log_lines":["SNAP(VISION-NAV) at 7"],"ports":[["lockstep:serial0",'
        b'"02"]],"responses":[{"id":[0,4],"payload":{"ok":true}}],"spawn_requests":[{"id":[2,5],'
        b'"ordinal":1,"role":"rover"}],"stop_requests":[{"generation":1,"id":[2,6],"ordinal":1}]}',
    ),
    "ending done": (
        Done(9, events=(Event("stopped", {"id": [2, 6]}),), ended=True),
        DONE_HEADER + b'{"ended":{"flushed":true},"events":[{"kind":"stopped","payload":{"id":'
        b'[2,6]}}],"faults":[],"k":9,"log_lines":[],"ports":[],"responses":[],"spawn_requests":[],'
        b'"stop_requests":[]}',
    ),
    "pending multi-tick request": (
        # (0, 4) arrived at tick 7 and is still pending; only (0, 5) is new.
        Tick(8, 160000, requests=(Request((0, 5), {"op": "echo"}),)),
        TICK_HEADER + b'{"cancels":[],"end":false,"k":8,"lifecycle":[],"ports":[],"requests":'
        b'[{"id":[0,5],"payload":{"op":"echo"}}],"t_us":160000}',
    ),
    "cancel": (
        Tick(9, 180000, cancels=((0, 4),)),
        TICK_HEADER + b'{"cancels":[[0,4]],"end":false,"k":9,"lifecycle":[],"ports":[],'
        b'"requests":[],"t_us":180000}',
    ),
    "stale stop": (
        Tick(10, 200000, lifecycle=(Lifecycle("stop_result", (0, 12), (2, 7), 1, 0, "error",
                                              "stale generation"),)),
        TICK_HEADER + b'{"cancels":[],"end":false,"k":10,"lifecycle":[{"generation":0,"id":[0,12],'
        b'"kind":"stop_result","ordinal":1,"payload":"stale generation","request":[2,7],'
        b'"result":"error"}],'
        b'"ports":[],"requests":[],"t_us":200000}',
    ),
    "lifecycle in commit order": (
        # A stop's result and the target's retirement, committed in that order at one tick.
        Tick(12, 240000, lifecycle=(Lifecycle("exited", (0, 14), (2, 1), 1, 1, None, {"failed": False}),
                                    Lifecycle("stop_result", (0, 15), (2, 6), 1, 1, "acked"))),
        TICK_HEADER + b'{"cancels":[],"end":false,"k":12,"lifecycle":[{"generation":1,"id":[0,14],'
        b'"kind":"exited","ordinal":1,"payload":{"failed":false},"request":[2,1],"result":null},'
        b'{"generation":1,"id":[0,15],"kind":"stop_result","ordinal":1,"payload":null,'
        b'"request":[2,6],"result":"acked"}],"ports":[],"requests":[],"t_us":240000}',
    ),
    "drain end": (
        Tick(11, 220000, end=True),
        TICK_HEADER + b'{"cancels":[],"end":true,"k":11,"lifecycle":[],"ports":[],"requests":[],'
        b'"t_us":220000}',
    ),
}


@pytest.mark.parametrize("name", sorted(GOLDEN))
def test_each_message_matches_its_golden_bytes(name):
    message, data = GOLDEN[name]
    assert encode(message) == data
    assert decode(data) == message


@pytest.mark.parametrize(
    ("data", "message"),
    [
        (b"NVPART02" + REGISTER_HEADER[8:] + GOLDEN["register"][1][11:], "bad magic"),
        (REGISTER_HEADER[:8] + b"\x02\x00\x01" + GOLDEN["register"][1][11:], "version"),
        (REGISTER_HEADER[:10] + b"\x09" + GOLDEN["register"][1][11:], "unknown message type"),
        (REGISTER_HEADER[:5], "truncated"),
        (GOLDEN["register"][1][:-1], "malformed"),
        (GOLDEN["register"][1].replace(b'"generation":1,', b'"generation":1, '), "canonical"),
        (GOLDEN["register"][1].replace(b'[2,3]}', b'[2,3],"x":1}'), "keys"),
        (GOLDEN["drain end"][1].replace(b'"end":true', b'"end":1'), "boolean"),
        (GOLDEN["tick"][1].replace(b'"kind":"spawn"', b'"kind":"spawned"'), "lifecycle kind"),
    ],
)
def test_malformed_messages_are_protocol_errors(data, message):
    with pytest.raises(PartWireError, match=message):
        decode(data)


def test_a_failed_registration_is_a_protocol_error():
    """Ordinal 0 is the operator's: no participant may register as it."""
    data = REGISTER_HEADER + (b'{"capability":"9f86d081","generation":1,"ordinal":0,'
                              b'"role":"rover","spawn_id":[2,3]}')
    with pytest.raises(PartWireError, match="ordinal"):
        decode(data)


def test_a_duplicate_or_a_missing_tick_is_a_protocol_error():
    check_tick(decode(GOLDEN["tick"][1]).k, 7)
    with pytest.raises(PartWireError, match="missing or duplicate"):
        check_tick(decode(GOLDEN["tick"][1]).k, 8)  # tick 7 again: a duplicate
    with pytest.raises(PartWireError, match="missing or duplicate"):
        check_tick(decode(GOLDEN["cancel"][1]).k, 8)  # tick 9 after 7: 8 is missing


def test_every_id_list_is_in_strict_id_order():
    swapped = GOLDEN["pending multi-tick request"][1].replace(
        b'[{"id":[0,5]', b'[{"id":[0,6],"payload":null},{"id":[0,5]')
    with pytest.raises(PartWireError, match="id order"):
        decode(swapped)
    duplicate = GOLDEN["cancel"][1].replace(b"[[0,4]]", b"[[0,4],[0,4]]")
    with pytest.raises(PartWireError, match="id order"):
        decode(duplicate)
    with pytest.raises(PartWireError, match="round-trip|id order"):
        encode(Tick(1, 0, cancels=((0, 5), (0, 4))))


def test_lifecycle_records_are_in_commit_id_order():
    data = GOLDEN["lifecycle in commit order"][1]
    swapped = data.replace(b'"id":[0,14]', b'"id":[0,99]')
    with pytest.raises(PartWireError, match="lifecycle not in strict id order"):
        decode(swapped)
    with pytest.raises(PartWireError, match="id"):
        decode(data.replace(b'"id":[0,14]', b'"id":null'))  # every record has a commit id


@pytest.mark.parametrize("spelling", ["FE01", "fe 01", " fe01", "fe01 ", "Fe01"])
def test_port_bytes_have_one_hex_spelling(spelling):
    data = GOLDEN["tick"][1].replace(b'"fe01"', f'"{spelling}"'.encode())
    with pytest.raises(PartWireError, match="hex"):
        decode(data)
