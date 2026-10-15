"""The 10f NAVLINK gate over a fleet run's traces (``navlink_trace``).

``scripts/eval_lockstep_fleet_backend.py --navpy`` writes the traces as
:func:`trace_rows` and gates the run on :func:`trace_summary`: every ordered
pair of NavPys has a complete trace, none is lost before stop-all's first
request (:func:`first_stop_tick`; packets in flight while stop-all retires
receivers may stop short) and none is ambiguous (:func:`navlink_ok`).
:func:`write_navlink` writes a run's traces and each NavPy's log lines.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path

from navpy.sim.lockstep.navlink_trace import NavlinkTrace, trace_navlink
from scripts.lockstep_fleet_actor import FleetLayout

TRACES_FILE = "navlink_traces.json"


def trace_rows(traces: list[NavlinkTrace]) -> list[dict]:
    """The traces as ``navlink_traces.json`` rows."""
    return [{"uid": list(t.uid), "msgid": t.msgid, "sender": t.sender, "receiver": t.receiver,
             "hops": [list(hop) for hop in t.hops], "complete": t.complete,
             "latency_ticks": t.latency_ticks, "ambiguous": t.ambiguous} for t in traces]


def first_stop_tick(records: list[dict]) -> int | None:
    """The tick of the run's first lifecycle stop request (stop-all's first)."""
    return next((record["k"] for record in records if "k" in record
                 for entry in record.get("lifecycle", ()) if entry["kind"] == "stop"), None)


def trace_summary(traces: list[NavlinkTrace], sys_ids: tuple[int, ...], stop_tick: int | None) -> dict:
    """Per ordered pair ``"i>j"``: traces, complete ones and their latency
    range; ``pairs_complete`` holds when every ordered pair of ``sys_ids`` has
    a complete trace; ``lost_before_stop`` counts the incomplete traces sent
    before ``stop_tick`` (any, with no stop) and ``ambiguous`` the traces of
    byte-identical resends, which no pairing can attribute. A complete trace
    kept its bytes through both autopilots (``navlink_trace`` matches
    re-emissions by them)."""
    pairs = {}
    for sender in sys_ids:
        for receiver in sys_ids:
            if sender == receiver:
                continue
            mine = [t for t in traces if (t.sender, t.receiver) == (sender, receiver)]
            latencies = [t.latency_ticks for t in mine if t.complete and not t.ambiguous]
            pairs[f"{sender}>{receiver}"] = {
                "traces": len(mine), "complete": len(latencies),
                "latency_ticks": [min(latencies), max(latencies)] if latencies else None}
    complete = [t for t in traces if t.complete and not t.ambiguous]
    lost = [t for t in traces if not t.complete and (stop_tick is None or t.hops[0][1] < stop_tick)]
    return {"pairs": pairs, "pairs_complete": all(p["complete"] > 0 for p in pairs.values()),
            "complete": len(complete),
            "stop_tick": stop_tick, "lost_before_stop": len(lost),
            "ambiguous": sum(t.ambiguous for t in traces)}


def navlink_ok(summary: dict) -> bool:
    """Every ordered pair complete, none lost before stop-all, none ambiguous."""
    return summary["pairs_complete"] and summary["lost_before_stop"] == 0 and summary["ambiguous"] == 0


def write_navlink(run_dir: Path, records: list[dict], log_lines: Mapping[int, list[str]], fleet: FleetLayout) -> dict:
    """Write each NavPy's log lines and the traces into ``run_dir``; return their summary."""
    for sys_id in fleet.sys_ids:
        lines = log_lines.get(fleet.navpy(sys_id), [])
        (run_dir / f"{fleet.companion_role(sys_id)}_log_lines.txt").write_text(
            "".join(f"{line}\n" for line in lines), encoding="utf-8")
    traces = trace_navlink(records, fleet.sys_ids)
    (run_dir / TRACES_FILE).write_text(json.dumps(trace_rows(traces), indent=1) + "\n", encoding="utf-8")
    return trace_summary(traces, fleet.sys_ids, first_stop_tick(records))


__all__ = ["TRACES_FILE", "first_stop_tick", "navlink_ok", "trace_rows", "trace_summary", "write_navlink"]
