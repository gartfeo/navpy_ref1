"""A lockstep run's radio air load, from its journal (``radio_budget.md`` item 3).

Report only, no pass or fail: the owner sets a usable budget from a bench
test first. A node is a radio's sender: each SITL's serial1 output
(``sitl<i>``) and the backend's output to the router (``backend``). Its load
is framed bytes, the byte ranges of the packets it emitted (whole MAVLink
frames), counted at the tick that emitted them. Reported apart: bytes that
are no packet (``router_skipped_bytes``) and an incomplete frame a stream
still held at the end.

A tick's bytes were written over the sim time since the tick before, so
they count in the interval ``(t_prev, t]`` that ends at the tick's time
``t_us``; the first tick's starts at SITL boot (0). The parked frame k_F+1
counts at its own ``t_us``, and drain ticks at theirs; the measurement ends
at the last of them. Sim time is cut into windows of :data:`WINDOW_US`; a
node's rate is its bytes per window second, a partial last window scaled by
its length. Setup (up to the tick of the first NavPy spawn: connect,
parameter writes, mission uploads) is reported apart from the mission.
With radio byte loss (``radio_link``) the router frames what survives, so
only a run without it measures the offered load; a loss-on run reports the
emissions after radio loss.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from navpy.sim.lockstep.coordinator_mavlink import ROUTER_LINK, ROUTER_SKIPPED_BYTES
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE
from navpy.sim.lockstep.radio_link import journal_radio_loss_pct

#: One window of sim time, 1 s.
WINDOW_US = 1_000_000
BACKEND = "backend"


def _nodes(records: Sequence[Mapping[str, Any]]) -> dict[str, str]:
    """Each sending device's node name: ``sitl<i>`` for SITL i's serial1, and the backend."""
    grid = next(r["grid"] for r in records if "grid" in r)
    nodes = {serial1: f"sitl{index}" for index, (_serial0, serial1) in enumerate(grid["sitl_ports"], 1)}
    return {**nodes, ROUTER_DEVICE: BACKEND}


def _ticks(records: Sequence[Mapping[str, Any]]) -> list[tuple[int, list]]:
    """``(t_us, packet events)`` per tick, the parked frame's at its own sim time."""
    ticks = []
    for record in records:
        if "k" in record and "t_us" in record:
            ticks.append((record["t_us"], record.get("packets", [])))
        elif "parked" in record:
            ticks.append((record["parked"]["t_us"], record["parked"].get("packets", [])))
    return ticks


def _first_spawn_us(records: Sequence[Mapping[str, Any]]) -> int | None:
    return next((record["t_us"] for record in records if "k" in record
                 for entry in record.get("lifecycle", ()) if entry["kind"] == "spawn"), None)


def _rates(samples: list[tuple[int, str, int]], names: Sequence[str], start_us: int,
           end_us: int) -> dict[str, Any]:
    """Bytes, mean and max bytes per second of each node and of all, over ``(start, end]``."""
    span_us = end_us - start_us
    if span_us <= 0:
        return {"seconds": 0.0, "nodes": {}}
    windows: dict[int, dict[str, int]] = {}
    for t_us, node, size in samples:
        bucket = windows.setdefault((t_us - start_us - 1) // WINDOW_US, {})
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


def air_load(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """The report section; the journal's grid says whether the radio lost bytes before framing."""
    radio_loss = journal_radio_loss_pct(records) > 0.0
    nodes = _nodes(records)
    names = sorted(set(nodes.values()))
    samples = []
    for t_us, events in _ticks(records):
        for event in events:
            if event[0] == "emitted" and event[1][2] in nodes:
                start, end = event[2]
                samples.append((t_us, nodes[event[1][2]], end - start))
    end = max((t_us for t_us, _events in _ticks(records)), default=0)
    spawn = _first_spawn_us(records)
    split = spawn if spawn is not None else end
    closing = next((r for r in records if "mavlink_counts" in r), {})
    counts = closing.get("mavlink_counts", {})
    links = {**{device: node for device, node in nodes.items() if node != BACKEND}, ROUTER_LINK: BACKEND}
    return {
        "basis": "framed bytes of emitted packets per 1 s window of sim time",
        "measures": "observed after radio byte loss" if radio_loss else "offered load",
        "setup": _rates([s for s in samples if s[0] <= split], names, 0, split),
        "mission": _rates([s for s in samples if s[0] > split], names, split, end),
        "skipped_bytes": {node: counts.get(link, {}).get(ROUTER_SKIPPED_BYTES, {}).get("skipped_bytes", 0)
                          for link, node in sorted(links.items(), key=lambda item: item[1])},
        "held_bytes": {node: sum(held["bytes"] for key, held in closing.get("held", {}).items()
                                 if key.rsplit("/", 1)[-1] == device)
                       for device, node in sorted(nodes.items(), key=lambda item: item[1])},
    }


__all__ = ["BACKEND", "WINDOW_US", "air_load"]
