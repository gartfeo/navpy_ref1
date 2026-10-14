"""Router parity: the coordinator's router model against the real mavlink-routerd.

``lockstep_fleet.md``, Router parity (pinned, not claimed). The cases are
recorded streams of UDP datagrams into the router's four endpoints (SITL
1-3 serial1 and the backend): targeted, broadcast, sniffer, an unknown
target, a targeted NAVLINK, boot text, a split packet and an AP echo.

``--record`` (offline, never inside a run) reads the deployed binary from
WSL, extracts its message-entry table into
``src/navpy/sim/lockstep/router_entries.json``, replays every case through
the binary (``router_parity_wsl.py``) and writes what each endpoint
received, with the binary's sha256, to ``tests/fixtures/lockstep/router_parity.json``.
``test_router_parity`` replays the same datagrams through the model
(:func:`replay_model`) and requires the same bytes at every endpoint; with
``NAVPY_ROUTER_PARITY=1`` it also re-runs the binary.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import subprocess
from collections.abc import Callable
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import encoder
from navpy.sim.lockstep.router_entries import FIELDS, TABLE_FILE, EntryTable
from navpy.sim.lockstep.router_framer import RouterFramer
from navpy.sim.lockstep.router_model import SNIFFER_SYSID, ReferenceRouter

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "lockstep" / "router_parity.json"
WSL_HARNESS = ROOT / "scripts" / "router_parity_wsl.py"
BINARY = "/usr/bin/mavlink-routerd"
ENDPOINTS = ("sitl1", "sitl2", "sitl3", "backend")
#: ``mavlink_msg_entry_t`` (12 bytes with its padding) and HEARTBEAT's entry,
#: the table's first row: where each copy of the table starts in the binary.
ENTRY = struct.Struct("<IBBBBBBxx")
HEARTBEAT_ENTRY = ENTRY.pack(0, 50, 9, 9, 0, 0, 0)
#: A fence packet's source: a sysid per sending endpoint, used by nothing else.
FENCE_SYSID = 200
FENCE_COMPID = 199

Step = tuple[str, bytes, bool]


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


class _Sources:
    """One encoder per MAVLink source, so sequence numbers run as on a real link."""

    def __init__(self) -> None:
        self._links: dict[tuple[int, int], tuple[mavlink.MAVLink, object]] = {}

    def __call__(self, system: int, component: int, send: str, *args: object) -> bytes:
        if (system, component) not in self._links:
            self._links[(system, component)] = encoder(system, component)
        link, sink = self._links[(system, component)]
        getattr(link, f"{send}_send")(*args)
        return sink.drain()


def _case(steps: list[tuple[str, bytes]]) -> list[Step]:
    """Each step from endpoint E, then E's fence."""
    fences = _Sources()
    out: list[Step] = []
    for endpoint, data in steps:
        out.append((endpoint, data, False))
        index = ENDPOINTS.index(endpoint)
        fence = fences(FENCE_SYSID + index, FENCE_COMPID, "heartbeat", 6, 8, 0, 0, 0)
        out.append((endpoint, fence, True))
    return out


def _unknown_navlink(system: int, component: int) -> bytes:
    """A MAVLink 2 frame whose id (25500) is in neither the entry table nor the dialect."""
    payload = bytes(range(1, 9))
    header = bytes((0xFD, len(payload), 0, 0, 0, system, component)) + (25500).to_bytes(3, "little")
    return header + payload + b"\x12\x34"


def build_cases() -> dict[str, list[Step]]:
    m = _Sources()
    hb = lambda system, component: m(system, component, "heartbeat", 1, 3, 0, 0, 0)  # noqa: E731
    param = lambda src, system, component: m(  # noqa: E731
        *src, "param_request_read", system, component, b"X", -1)
    learn = [("sitl1", hb(1, 1)), ("sitl2", hb(2, 1)), ("sitl3", hb(3, 1)), ("backend", hb(255, 190))]
    assign = lambda src, target: m(  # noqa: E731
        *src, "task_assign_request", 7, 1, 1000, 5000, target, 4, 1, 2, 1, 2, 3)
    ack = lambda src, target: m(*src, "swarm_ack", 7, 2, 1000, 5000, target, 0, 0, 0, 0)  # noqa: E731
    heartbeat = hb(1, 1)
    corrupt = bytearray(hb(2, 1))
    corrupt[-1] ^= 0xFF
    v1 = bytes((0xFE, 9, 0, 3, 1, 0)) + bytes(9)
    return {
        "broadcast": _case([*learn, ("sitl1", hb(1, 1) + hb(1, 191)), ("sitl3", v1 + _checksum_v1(v1))]),
        "targeted": _case([*learn, ("backend", param((255, 190), 2, 1)), ("backend", param((255, 190), 2, 0)),
                           ("backend", param((255, 190), 2, 5)), ("sitl2", param((2, 1), 1, 1))]),
        "sniffer": _case([*learn, ("sitl1", param((1, 1), 2, 1)), ("sitl3", param((3, 1), 9, 1))]),
        "unknown_target": _case([("sitl1", hb(1, 1)), ("backend", param((255, 190), 9, 1)),
                                 ("backend", hb(255, 190)), ("sitl2", param((2, 1), 9, 1))]),
        "targeted_navlink": _case([*learn, ("sitl1", hb(1, 191)), ("sitl1", assign((1, 191), 2)),
                                   ("sitl2", ack((2, 191), 1)), ("sitl2", ack((2, 191), 0)),
                                   ("sitl1", assign((1, 191), 9)), ("sitl3", _unknown_navlink(3, 191))]),
        "boot_text": _case([("sitl1", b"Init ArduPlane V4.6.0\r\nFree RAM: 4096\r\n" + heartbeat),
                            ("sitl2", b"APM: no marker here\r\n"), ("sitl2", bytes(corrupt) + hb(2, 1)),
                            ("backend", hb(255, 190))]),
        "split_packet": _split_case(hb, assign),
        "ap_echo": _case([*learn, ("sitl1", hb(1, 191)), ("sitl2", hb(1, 191)),
                          ("sitl3", assign((1, 191), 2)), ("backend", hb(1, 191))]),
    }


def _split_case(hb: Callable[[int, int], bytes],
                assign: Callable[[tuple[int, int], int], bytes]) -> list[Step]:
    packet = assign((1, 191), 2)
    steps = _case([("sitl1", hb(1, 1)), ("sitl2", hb(2, 1)), ("backend", hb(255, 190))])
    first, second = packet[:7], packet[7:] + hb(1, 1)
    return [*steps, ("sitl1", first, False), *_case([("sitl1", second)])]


def _checksum_v1(frame: bytes) -> bytes:
    from pymavlink.generator.mavcrc import x25crc

    crc = x25crc(frame[1:])
    crc.accumulate(bytes((50,)))  # HEARTBEAT's CRC extra
    return crc.crc.to_bytes(2, "little")


def replay_model(steps: list[Step], table: EntryTable,
                 sniffer_sysid: int = SNIFFER_SYSID) -> dict[str, bytes]:
    """The model's view of a case: what each endpoint receives, in order."""
    router = ReferenceRouter(list(ENDPOINTS), sniffer_sysid)
    framers = {name: RouterFramer(table) for name in ENDPOINTS}
    received = {name: bytearray() for name in ENDPOINTS}
    for endpoint, data, _fence in steps:
        for segment in framers[endpoint].feed(data):
            if segment.packet is not None:
                for destination in router.route(endpoint, segment.packet):
                    received[destination] += segment.data
    return {name: bytes(data) for name, data in received.items()}


def run_binary(cases: dict[str, list[Step]], binary: str = BINARY) -> dict[str, dict[str, bytes]]:
    job = {"binary": binary, "endpoints": list(ENDPOINTS),
           "cases": {name: [[e, d.hex(), f] for e, d, f in steps] for name, steps in cases.items()}}
    harness = subprocess.run(["wsl.exe", "-e", "wslpath", "-a", str(WSL_HARNESS)],
                             capture_output=True, text=True, check=True).stdout.strip()
    result = subprocess.run(["wsl.exe", "-e", "python3", harness], input=json.dumps(job),
                            capture_output=True, text=True, timeout=300)
    if result.returncode != 0:
        raise RuntimeError(f"router parity harness failed: {result.stderr.strip()}")
    return {case: {name: bytes.fromhex(data) for name, data in received.items()}
            for case, received in json.loads(result.stdout).items()}


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
                          "received": {n: outputs[name][n].hex() for n in ENDPOINTS}}
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
