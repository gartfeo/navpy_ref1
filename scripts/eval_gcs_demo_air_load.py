"""A lockstep run's radio air load, from its journal (``radio_budget.md`` item 3).

Report only, no pass or fail: the owner sets a usable budget from a bench
test first. A node is a radio's sender: each SITL's serial1 output
(``sitl<i>``) and the backend's output to the router (``backend``). Its load
is framed bytes, the byte ranges of the packets it emitted (whole MAVLink
frames), counted at the tick that emitted them. Reported apart: bytes that
are no packet (``router_skipped_bytes``) and an incomplete frame a stream
still held at the end.

Sim time is cut into windows of :data:`WINDOW_US`; a node's rate is its
bytes per window second, a partial last window scaled by its length. Setup
(the ticks before the first NavPy spawn: connect, parameter writes, mission
uploads) is reported apart from the mission. SITL drops lost bytes before
the coordinator sees them, so only a run with ``SIM_UART_LOSS`` off measures
the offered load; a loss-on run reports what it observed.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from navpy.sim.lockstep.coordinator_mavlink import ROUTER_LINK, ROUTER_SKIPPED_BYTES
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE

#: One window of sim time, 1 s.
WINDOW_US = 1_000_000
BACKEND = "backend"


def _nodes(records: Sequence[Mapping[str, Any]]) -> dict[str, str]:
    """Each sending device's node name: ``sitl<i>`` for SITL i's serial1, and the backend."""
    grid = next(r["grid"] for r in records if "grid" in r)
    nodes = {serial1: f"sitl{index}" for index, (_serial0, serial1) in enumerate(grid["sitl_ports"], 1)}
    return {**nodes, ROUTER_DEVICE: BACKEND}


def _ticks(records: Sequence[Mapping[str, Any]]) -> list[tuple[int, list]]:
    """``(t_us, packet events)`` per tick; the parked record's at the last tick's time."""
    ticks, last = [], 0
    for record in records:
        if "k" in record and "t_us" in record:
            last = record["t_us"]
            ticks.append((last, record.get("packets", [])))
        elif "parked" in record:
            ticks.append((last, record["parked"].get("packets", [])))
    return ticks


def _first_spawn_us(records: Sequence[Mapping[str, Any]]) -> int | None:
    return next((record["t_us"] for record in records if "k" in record
                 for entry in record.get("lifecycle", ()) if entry["kind"] == "spawn"), None)


def _rates(samples: list[tuple[int, str, int]], names: Sequence[str], start_us: int,
           end_us: int) -> dict[str, Any]:
    """Bytes, mean and max bytes per second of each node and of all, over ``[start, end]``."""
    span_us = end_us - start_us
    if span_us <= 0:
        return {"seconds": 0.0, "nodes": {}}
    windows: dict[int, dict[str, int]] = {}
    for t_us, node, size in samples:
        bucket = windows.setdefault((t_us - start_us) // WINDOW_US, {})
        bucket[node] = bucket.get(node, 0) + size
    count = -(-span_us // WINDOW_US)  # windows, the last maybe partial

    def seconds(index: int) -> float:
        return min(WINDOW_US, span_us - index * WINDOW_US) / 1e6

    out = {}
    for name in [*names, "total"]:
        per = [sum(windows.get(i, {}).values()) if name == "total" else windows.get(i, {}).get(name, 0)
               for i in range(count)]
        out[name] = {"bytes": sum(per), "mean_bytes_per_s": round(sum(per) / (span_us / 1e6), 1),
                     "max_bytes_per_s": round(max(b / seconds(i) for i, b in enumerate(per)), 1)}
    return {"seconds": round(span_us / 1e6, 3), "nodes": out}


def air_load(records: Sequence[Mapping[str, Any]], uart_loss: bool) -> dict[str, Any]:
    """The report section; ``uart_loss`` says whether SITL dropped bytes before they were seen."""
    nodes = _nodes(records)
    names = sorted(set(nodes.values()))
    samples = []
    for t_us, events in _ticks(records):
        for event in events:
            if event[0] == "emitted" and event[1][2] in nodes:
                start, end = event[2]
                samples.append((t_us, nodes[event[1][2]], end - start))
    ticks = [t_us for t_us, _events in _ticks(records)]
    # Each tick covers the sim time up to the next; the last one its predecessor's length.
    first = ticks[0] if ticks else 0
    last = ticks[-1] + (ticks[-1] - ticks[-2]) if len(ticks) > 1 else first
    spawn = _first_spawn_us(records)
    split = spawn if spawn is not None else last
    closing = next((r for r in records if "mavlink_counts" in r), {})
    counts = closing.get("mavlink_counts", {})
    links = {**{device: node for device, node in nodes.items() if node != BACKEND}, ROUTER_LINK: BACKEND}
    return {
        "basis": "framed bytes of emitted packets per 1 s window of sim time",
        "measures": "observed after SITL byte loss" if uart_loss else "offered load",
        "setup": _rates([s for s in samples if s[0] < split], names, first, split),
        "mission": _rates([s for s in samples if s[0] >= split], names, split, last),
        "skipped_bytes": {node: counts.get(link, {}).get(ROUTER_SKIPPED_BYTES, {}).get("skipped_bytes", 0)
                          for link, node in sorted(links.items(), key=lambda item: item[1])},
        "held_bytes": {node: sum(held["bytes"] for key, held in closing.get("held", {}).items()
                                 if key.rsplit("/", 1)[-1] == device)
                       for device, node in sorted(nodes.items(), key=lambda item: item[1])},
    }


__all__ = ["BACKEND", "WINDOW_US", "air_load"]
