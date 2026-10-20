"""Router parity: the coordinator's router model against the real mavlink-routerd.

``lockstep_fleet.md``, Router parity (pinned, not claimed). The cases are
recorded streams of UDP datagrams into the router's four endpoints (SITL
1-3 serial1 and the backend): targeted, broadcast, sniffer, an unknown
target, a targeted NAVLINK, boot text, a split packet and an AP echo, and
cases with the SITL endpoints' ``BlockSrcCompOut`` filter
(``radio_budget.md`` item 2): filtered broadcasts, filtered targeted
messages, an unknown id, the message classes the fleet uses, and mixed loop,
filter and target rejection.

``--record`` (offline, never inside a run) reads the deployed binary from
WSL, extracts its message-entry table into
``src/navpy/sim/lockstep/router_entries.json``, replays every case through
the binary (``router_parity_wsl.py``) and writes what each endpoint
received and its ``msg_to_unknown`` count, with the binary's sha256, to
``tests/fixtures/lockstep/router_parity.json``. ``test_router_parity``
replays the same datagrams through the model (:func:`replay_model`) and
requires the same bytes at every endpoint and the same count; with
``NAVPY_ROUTER_PARITY=1`` it also re-runs the binary.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "src"):
    sys.path.insert(0, str(directory))

from navpy.sim.lockstep.router_entries import FIELDS, TABLE_FILE, EntryTable
from navpy.sim.lockstep.router_framer import RouterFramer
from navpy.sim.lockstep.router_model import SITL_FILTER, SNIFFER_SYSID, EndpointFilter, ReferenceRouter
from scripts.router_parity_cases import ENDPOINTS, FILTERED_CASES, Step, build_cases

FIXTURE = ROOT / "tests" / "fixtures" / "lockstep" / "router_parity.json"
WSL_HARNESS = ROOT / "scripts" / "router_parity_wsl.py"
BINARY = "/usr/bin/mavlink-routerd"
#: ``mavlink_msg_entry_t`` (12 bytes with its padding) and HEARTBEAT's entry,
#: the table's first row: where each copy of the table starts in the binary.
ENTRY = struct.Struct("<IBBBBBBxx")
HEARTBEAT_ENTRY = ENTRY.pack(0, 50, 9, 9, 0, 0, 0)
#: The filter ``run_swarm.sh`` and the coordinator give each SITL endpoint.
SITL_FILTERS = {name: SITL_FILTER for name in ENDPOINTS if name.startswith("sitl")}

Filters = dict[str, EndpointFilter]


def filters_json(filters: Filters) -> dict[str, dict[str, list[int]]]:
    """Each endpoint's filters by their mavlink-router conf keys (the fixture's and the job's
    form); an empty ``AllowMsgIdIn`` is no incoming filter."""
    return {name: {"AllowMsgIdIn": sorted(f.allow_msg_id_in or ()),
                   "BlockMsgIdOut": list(f.block_msg_id_out), "BlockSrcCompOut": list(f.block_src_comp_out)}
            for name, f in filters.items()}


def filters_from_json(data: dict[str, dict[str, list[int]]]) -> Filters:
    return {name: EndpointFilter(frozenset(keys["AllowMsgIdIn"]) or None, tuple(keys["BlockMsgIdOut"]),
                                 tuple(keys["BlockSrcCompOut"]))
            for name, keys in data.items()}


def extract_entry_table(binary: bytes) -> list[tuple[int, ...]]:
    """Every copy of the binary's entry table (one per translation unit) must agree."""
    tables, start = [], binary.find(HEARTBEAT_ENTRY)
    while start >= 0:
        rows, offset = [], start
        while not rows or ENTRY.unpack_from(binary, offset)[0] > rows[-1][0]:
            rows.append(ENTRY.unpack_from(binary, offset))
            offset += ENTRY.size
        tables.append(rows)
        start = binary.find(HEARTBEAT_ENTRY, start + 1)
    if not tables or any(table != tables[0] for table in tables):
        raise ValueError(f"{len(tables)} entry tables found; they must exist and agree")
    return tables[0]


def table_json(rows: list[tuple[int, ...]], sha256: str, version: str) -> str:
    lines = ",\n".join("    " + json.dumps(list(row)) for row in rows)
    head = json.dumps({"source": BINARY, "version": version, "binary_sha256": sha256,
                       "fields": list(FIELDS)}, indent=2)[:-2]
    return f'{head},\n  "entries": [\n{lines}\n  ]\n}}\n'


def case_filters(name: str) -> Filters:
    return dict(SITL_FILTERS) if name in FILTERED_CASES else {}


def replay_model(steps: list[Step], table: EntryTable, sniffer_sysid: int = SNIFFER_SYSID,
                 filters: Filters | None = None) -> tuple[dict[str, bytes], int]:
    """The model's view of a case: what each endpoint receives, in order, and its unknown count."""
    router = ReferenceRouter(list(ENDPOINTS), sniffer_sysid, filters)
    framers = {name: RouterFramer(table) for name in ENDPOINTS}
    received = {name: bytearray() for name in ENDPOINTS}
    for endpoint, data, _fence in steps:
        for segment in framers[endpoint].feed(data):
            if segment.packet is not None:
                for destination in router.route(endpoint, segment.packet).destinations:
                    received[destination] += segment.data
    return {name: bytes(data) for name, data in received.items()}, router.dropped_unknown_target


def run_binary(cases: dict[str, list[Step]],
               binary: str = BINARY) -> dict[str, tuple[dict[str, bytes], int]]:
    """Each case's bytes per endpoint and ``msg_to_unknown`` count from the binary."""
    job = {"binary": binary, "endpoints": list(ENDPOINTS),
           "cases": {name: [[e, d.hex(), f] for e, d, f in steps] for name, steps in cases.items()},
           "filters": {name: filters_json(case_filters(name)) for name in cases}}
    harness = subprocess.run(["wsl.exe", "-e", "wslpath", "-a", str(WSL_HARNESS)],
                             capture_output=True, text=True, check=True).stdout.strip()
    result = subprocess.run(["wsl.exe", "-e", "python3", harness], input=json.dumps(job),
                            capture_output=True, text=True, timeout=300)
    if result.returncode != 0:
        raise RuntimeError(f"router parity harness failed: {result.stderr.strip()}")
    return {case: ({name: bytes.fromhex(data) for name, data in out["received"].items()}, out["unknown"])
            for case, out in json.loads(result.stdout).items()}


def binary_bytes(binary: str = BINARY) -> bytes:
    return subprocess.run(["wsl.exe", "-e", "cat", binary], capture_output=True, check=True,
                          timeout=60).stdout


def binary_version(binary: str = BINARY) -> str:
    out = subprocess.run(["wsl.exe", "-e", binary, "--version"], capture_output=True, text=True,
                         check=True, timeout=60).stdout
    return out.split()[-1]


def record() -> None:
    image = binary_bytes()
    sha256, version = hashlib.sha256(image).hexdigest(), binary_version()
    TABLE_FILE.write_text(table_json(extract_entry_table(image), sha256, version), encoding="utf-8")
    cases = build_cases()
    outputs = run_binary(cases)
    fixture = {"binary": BINARY, "version": version, "binary_sha256": sha256,
               "endpoints": list(ENDPOINTS), "cases": {
                   name: {"steps": [[e, d.hex(), f] for e, d, f in steps],
                          "filters": filters_json(case_filters(name)),
                          "received": {n: outputs[name][0][n].hex() for n in ENDPOINTS},
                          "unknown": outputs[name][1]}
                   for name, steps in cases.items()}}
    FIXTURE.write_text(json.dumps(fixture, indent=1) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--record", action="store_true", help="re-record the table and the fixture")
    if parser.parse_args().record:
        record()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
