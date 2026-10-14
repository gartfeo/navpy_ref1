"""The router entry table: the message entries mavlink-router was built with.

mavlink-router reads a message's target from its own message-entry table
(``mavlink_get_msg_entry``: CRC extra, target system and component offsets),
compiled from the dialect the deployed binary was built with, never from
NavPy's pymavlink dialect (``lockstep_fleet.md``, Router parity). An id
outside the table has no target, so the router broadcasts it, and its CRC is
not checked.

``router_entries.json`` is that table, extracted once from the deployed
``/usr/bin/mavlink-routerd`` by ``scripts/router_parity.py --record``; the
binary's sha256 is pinned beside it, and ``test_router_parity`` replays the
recorded cases against it.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

TABLE_FILE = Path(__file__).with_name("router_entries.json")
#: ``mavlink_msg_entry_t.flags`` (``MAV_MSG_ENTRY_FLAG_HAVE_TARGET_*``).
FLAG_TARGET_SYSTEM = 1
FLAG_TARGET_COMPONENT = 2
#: The entry's fields, in the struct's (and the JSON rows') order.
FIELDS = ("msgid", "crc_extra", "min_len", "max_len", "flags", "target_system_ofs",
          "target_component_ofs")


@dataclass(frozen=True)
class RouterEntry:
    msgid: int
    crc_extra: int
    min_len: int
    max_len: int
    flags: int
    target_system_ofs: int
    target_component_ofs: int


@dataclass(frozen=True)
class EntryTable:
    #: The binary the table was extracted from.
    binary_sha256: str
    version: str
    entries: Mapping[int, RouterEntry]

    def get(self, msgid: int) -> RouterEntry | None:
        return self.entries.get(msgid)


def parse_table(data: Mapping[str, object]) -> EntryTable:
    if list(data["fields"]) != list(FIELDS):
        raise ValueError(f"router entry fields {data['fields']}, expected {list(FIELDS)}")
    entries = {row[0]: RouterEntry(*row) for row in data["entries"]}
    return EntryTable(str(data["binary_sha256"]), str(data["version"]), entries)


@lru_cache(maxsize=None)
def load_entry_table(path: Path = TABLE_FILE) -> EntryTable:
    return parse_table(json.loads(path.read_text(encoding="utf-8")))


__all__ = [
    "EntryTable", "FIELDS", "FLAG_TARGET_COMPONENT", "FLAG_TARGET_SYSTEM", "RouterEntry",
    "TABLE_FILE", "load_entry_table", "parse_table",
]
