"""Router parity: the model against the real ``/usr/bin/mavlink-routerd -s 255`` (step 10b gate).

``lockstep_fleet.md``, Router parity (pinned, not claimed). The fixture
(``tests/fixtures/lockstep/router_parity.json``, recorded offline by
``scripts/router_parity.py --record``, never inside a run) holds the
binary's sha256 and, per case, the datagrams sent into its endpoints and
the bytes each endpoint received, the case's ``BlockMsgIdOut`` and ``BlockSrcCompOut`` filters
and the binary's ``msg_to_unknown`` count. The model (the router entry
table, the framer and
:class:`~navpy.sim.lockstep.router_model.ReferenceRouter`) must deliver the
same bytes to the same endpoints and count the same. ``NAVPY_ROUTER_PARITY=1``
also re-runs the binary in WSL and re-extracts its entry table.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.router_entries import TABLE_FILE, EntryTable, load_entry_table
from navpy.sim.lockstep.router_framer import RouterFramer
from navpy.sim.lockstep.router_model import SITL_FILTER, EndpointFilter
from scripts import router_parity
from scripts.router_parity import (ENDPOINTS, FILTERED_CASES, FIXTURE, SITL_FILTERS, filters_from_json,
                                   replay_model)

CASES = ("targeted", "broadcast", "sniffer", "unknown_target", "targeted_navlink", "boot_text",
         "split_packet", "ap_echo", *FILTERED_CASES)


def _fixture() -> dict:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _steps(case: dict) -> list:
    return [(endpoint, bytes.fromhex(data), fence) for endpoint, data, fence in case["steps"]]


def _received(case: dict) -> dict[str, bytes]:
    return {name: bytes.fromhex(data) for name, data in case["received"].items()}


def _filters(case: dict) -> dict[str, EndpointFilter]:
    return filters_from_json(case["filters"])


def test_the_fixture_and_the_entry_table_name_the_same_binary():
    fixture, table = _fixture(), load_entry_table()
    assert fixture["binary"] == "/usr/bin/mavlink-routerd" and fixture["version"] == "d59ef94"
    assert fixture["binary_sha256"] == table.binary_sha256
    assert len(fixture["binary_sha256"]) == 64 and table.version == fixture["version"]
    assert set(fixture["cases"]) == set(CASES) and fixture["endpoints"] == list(ENDPOINTS)
    for name, case in fixture["cases"].items():
        assert _filters(case) == (SITL_FILTERS if name in FILTERED_CASES else {})


@pytest.mark.parametrize("name", CASES)
def test_the_model_delivers_the_binarys_bytes_to_the_binarys_endpoints(name):
    case = _fixture()["cases"][name]
    received = _received(case)
    assert replay_model(_steps(case), load_entry_table(), filters=_filters(case)) == (
        received, case["unknown"])
    fences = {data for _endpoint, data, fence in _steps(case) if fence}
    payload = {endpoint: [s.data for s in RouterFramer(load_entry_table()).feed(data)
                          if s.packet is not None and s.data not in fences]
               for endpoint, data in received.items()}
    assert any(payload.values())  # every case routes more than its fences


def test_targets_come_from_the_entry_table_not_the_dialect():
    table = load_entry_table()
    # The deployed binary was built with the NAVLINK dialect: a targeted
    # NAVLINK message is routed by its target system (offset in the table).
    assign = table.get(mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST)
    assert (assign.flags, assign.target_system_ofs) == (1, 34)
    # Every id both know agrees on its CRC extra (a rebuilt dialect would show here).
    shared = [msgid for msgid in table.entries if msgid in mavlink.mavlink_map]
    assert len(shared) > 300
    assert all(table.get(msgid).crc_extra == mavlink.mavlink_map[msgid].crc_extra for msgid in shared)


def test_the_cases_tell_a_wrong_model_apart():
    """Each rule matters: a model without it disagrees with the binary somewhere."""
    fixture, table = _fixture(), load_entry_table()
    cases = {name: (_steps(case), _filters(case), (_received(case), case["unknown"]))
             for name, case in fixture["cases"].items()}

    def failing(**variant) -> set[str]:
        return {name for name, (steps, filters, recorded) in cases.items()
                if replay_model(steps, variant.get("table", table), variant.get("sniffer", 255),
                                variant.get("filters", filters)) != recorded}

    assert "sniffer" in failing(sniffer=0)
    no_targets = EntryTable(table.binary_sha256, table.version, {
        msgid: dataclasses.replace(entry, flags=0) for msgid, entry in table.entries.items()})
    assert {"targeted", "targeted_navlink"} <= failing(table=no_targets)
    assert "boot_text" in failing(table=EntryTable(table.binary_sha256, table.version, {}))
    # Without the filter, or with it on the backend too, every filtered case disagrees.
    assert set(FILTERED_CASES) <= failing(filters={})
    assert set(FILTERED_CASES) <= failing(filters={name: SITL_FILTER for name in ENDPOINTS})
    # Each key matters: without the id block the image case disagrees, without the component
    # block every other filtered case.
    no_ids = EndpointFilter(block_src_comp_out=SITL_FILTER.block_src_comp_out)
    no_comps = EndpointFilter(block_msg_id_out=SITL_FILTER.block_msg_id_out)
    assert "filtered_image" in failing(filters={name: no_ids for name in SITL_FILTERS})
    assert set(FILTERED_CASES) - {"filtered_image"} <= failing(
        filters={name: no_comps for name in SITL_FILTERS})


@pytest.mark.skipif(os.environ.get("NAVPY_ROUTER_PARITY") != "1",
                    reason="runs the real mavlink-routerd in WSL: set NAVPY_ROUTER_PARITY=1")
def test_the_deployed_binary_still_gives_the_recorded_bytes():
    fixture = _fixture()
    image = router_parity.binary_bytes()
    assert hashlib.sha256(image).hexdigest() == fixture["binary_sha256"]
    table = json.loads(TABLE_FILE.read_text(encoding="utf-8"))
    assert [list(row) for row in router_parity.extract_entry_table(image)] == table["entries"]
    cases = {name: _steps(case) for name, case in fixture["cases"].items()}
    live = router_parity.run_binary(cases)
    assert live == {name: (_received(case), case["unknown"]) for name, case in fixture["cases"].items()}
