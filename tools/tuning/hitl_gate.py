"""The HITL link-health gate: a run fails when its HIL link did not hold the contract.

The link-health gate reads the board log's HIL records (HIL link contract v2, fork
libraries/SITL/SIM_HIL.cpp write_log, 10 per second: the link state, counters that run from boot,
and the largest state age since the previous record) and the bridge's link log
(hitl_bridge/frame_loop.py LOG_COLUMNS, one row per frame). The counters run from boot, also while
no bridge has the port open (a board's USB port drops every frame until the PC reads it, so TxDrop
grows from boot): only their changes between consecutive records inside the armed flight count,
as the fork's Plane HIL autotest checks them (hil_check_link_health).
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from tools.tuning.hitl_bridge.board_clock import CONFLICT, CRASH, GAP, REINIT, RETRANSMIT
from tools.tuning.logdata import ARM_FIELDS, FlightLog, armed_windows, read_log

HIL_FIELDS = ("TimeUS", "LS", "Age", "AgeMx", "Stale", "Rx", "Rej", "Rpl", "Tx", "TxDrop", "CrcE", "ParE", "Oth",
              "Lp")
# HIL.LS, the link state: 0 no port, 1 waiting for the first state, 2 ok, 3 lost (the last state held)
HIL_LS_NO_PORT, HIL_LS_WAITING, HIL_LS_OK, HIL_LS_LOST = 0, 1, 2, 3
# Counters a healthy link never grows (contract v2 "HIL log record"): actuator frames not written whole,
# states rejected, frames with a bad CRC or a bad header. Rpl (a state replaced by a newer one in the
# same read) and Oth (other frames) are reported, not gated: both are normal on a jittery link.
HIL_MUST_NOT_GROW = {"TxDrop": "actuator frames dropped", "Rej": "states rejected",
                     "CrcE": "frames with a bad CRC", "ParE": "MAVLink header errors"}
# A stale frame hands the controllers a state one main loop (5 ms at 200 Hz) older; up to 1 in
# 20 is jitter, more makes that extra delay part of the plant the gains are tuned on.
HIL_STALE_FRACTION_MAX = 0.05
# 10 main loops at 200 Hz: a fresh state is one loop plus the round trip old (the bridge in WSL
# answered within about 10 ms at worst), so an older one means the link stalled.
HIL_AGE_MAX_US = 50_000
HIL_MIN_RECORDS = 2            # the gate needs a first and a last record: the counters run from boot
BRIDGE_EVENT, BRIDGE_ARMED = "event", "armed"      # hitl_bridge/frame_loop.py LOG_COLUMNS


def hil_link_from_log(log: FlightLog) -> dict:
    """The HIL link over the armed flight in a board log: frames, stale frames and their fraction,
    the largest state age, the link state, the receive and transmit counters, and the actuator
    frames against the completed main loops. The log also holds records from before arming and
    from just after the disarm, when the runner has already stopped the relay (a 78 ms age then
    failed a healthy run): only the records inside the ARM records' armed stretches count, and the
    counters only by their change between consecutive records there."""
    n = len(log.get("HIL", "TimeUS")) if log.has("HIL") else 0
    col = {f: log.get("HIL", f) for f in HIL_FIELDS} if n else {}
    missing = [f for f, values in col.items() if np.isnan(values).any()]
    if missing:
        return {"records": n, "missing_fields": missing}
    windows = [(col["TimeUS"] >= a) & (col["TimeUS"] <= b) for a, b in armed_windows(log)] if n else []
    inside = np.any(windows, axis=0) if windows else np.zeros(n, dtype=bool)
    if inside.sum() < HIL_MIN_RECORDS:
        return {"records": int(inside.sum()), "records_in_log": n}

    def steps(name: str) -> list[np.ndarray]:   # record-to-record changes inside each armed stretch
        return [np.diff(col[name][w]) for w in windows if w.sum() >= 2]

    def delta(name: str) -> int:
        return int(sum(d.sum() for d in steps(name)))

    sent = [tx + drop for tx, drop in zip(steps("Tx"), steps("TxDrop"))]
    mismatched = sum(int((frames != loops).sum()) for frames, loops in zip(sent, steps("Lp")))
    frames, stale = delta("Tx") + delta("TxDrop"), delta("Stale")
    states = col["LS"][inside].astype(int)
    return {"records": int(inside.sum()), "seconds": delta("TimeUS") / 1e6, "frames": frames,
            "loops": delta("Lp"), "frames_not_one_per_loop": mismatched,
            "stale": stale, "stale_fraction": stale / frames if frames > 0 else None,
            "age_max_us": int(col["AgeMx"][inside].max()), "age_median_us": float(np.median(col["Age"][inside])),
            "link_not_ok_records": int((states != HIL_LS_OK).sum()), "link_states": sorted(set(states.tolist())),
            "rx": delta("Rx"), "rejected": delta("Rej"), "replaced": delta("Rpl"), "tx_dropped": delta("TxDrop"),
            "crc_errors": delta("CrcE"), "parse_errors": delta("ParE"), "other_frames": delta("Oth"),
            "grown": {name: delta(name) for name in HIL_MUST_NOT_GROW}}


def hil_link_problems(name: str, link: dict) -> list[str]:
    """Why one log's HIL link fails the gate."""
    if "error" in link:
        return [f"{name}: unreadable: {link['error']}"]
    if link["records"] < HIL_MIN_RECORDS:
        return [f"{name}: {link['records']} HIL records inside the armed flight (ARM records), "
                f"{link.get('records_in_log', link['records'])} in the log; HIL firmware logs 10 per second"]
    if link.get("missing_fields"):
        return [f"{name}: HIL records lack {link['missing_fields']}"]
    problems = [] if link["frames"] > 0 else [f"{name}: no actuator frames between the first and last HIL record"]
    if link["link_not_ok_records"]:
        problems.append(f"{name}: link state not ok (LS {link['link_states']}) in {link['link_not_ok_records']} "
                        "records of the armed flight")
    if link["stale_fraction"] is not None and link["stale_fraction"] > HIL_STALE_FRACTION_MAX:
        problems.append(f"{name}: {link['stale']} of {link['frames']} frames stale ({link['stale_fraction']:.1%}), "
                        f"limit {HIL_STALE_FRACTION_MAX:.0%}")
    if link["age_max_us"] > HIL_AGE_MAX_US:
        problems.append(f"{name}: state age reached {link['age_max_us']} us, limit {HIL_AGE_MAX_US} us")
    for counter, what in HIL_MUST_NOT_GROW.items():
        if link["grown"][counter]:
            problems.append(f"{name}: {link['grown'][counter]} {what} ({counter}) during the armed flight")
    if link["frames_not_one_per_loop"]:
        problems.append(f"{name}: {link['frames']} actuator frames for {link['loops']} main loops "
                        f"({link['frames_not_one_per_loop']} records apart): not one frame per loop (Tx+TxDrop vs Lp)")
    return problems


def bridge_resets(link_csv: Path) -> dict:
    """Resets of the aircraft to the origin in the bridge's link log, and how many came at or
    after the first armed frame (a mid-flight reset teleports the aircraft and re-fires the
    catapult). Gaps (lost frames stepped across) and retransmitted frames (lockstep SITL only)
    are counted for the record; the board's state age covers them. Conflicting frames (other
    content at the same board time: contract v2 says board time strictly increases) are counted
    to fail the run, and so are crashes of the simulated aircraft (the plant left the envelope: the board
    then reads it at rest where it crashed)."""
    frames = resets = after_armed = gaps = retransmits = conflicts = crashes = 0
    armed_seen = False
    with open(link_csv, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        absent = {BRIDGE_EVENT, BRIDGE_ARMED} - set(reader.fieldnames or [])
        if absent:
            raise KeyError(f"columns {sorted(absent)} missing")
        for row in reader:
            frames += 1
            armed_seen = armed_seen or row[BRIDGE_ARMED] == "1"
            if row[BRIDGE_EVENT] == REINIT:
                resets += 1
                after_armed += 1 if armed_seen else 0
            gaps += 1 if row[BRIDGE_EVENT] == GAP else 0
            retransmits += 1 if row[BRIDGE_EVENT] == RETRANSMIT else 0
            conflicts += 1 if row[BRIDGE_EVENT] == CONFLICT else 0
            crashes += 1 if row[BRIDGE_EVENT] == CRASH else 0
    return {"frames": frames, "resets": resets, "resets_after_armed": after_armed, "gaps": gaps,
            "retransmits": retransmits, "conflicts": conflicts, "crashes": crashes, "armed_seen": armed_seen}


def link_health(logs: list[Path], link_csv: Path) -> dict:
    """The HITL run's link-health gate over the board logs' HIL records and the bridge's link log."""
    per_log: dict[str, dict] = {}
    problems: list[str] = []
    for path in logs:
        try:
            per_log[path.name] = hil_link_from_log(read_log(str(path), {"HIL": HIL_FIELDS, "ARM": ARM_FIELDS}))
        except Exception as exc:     # noqa: BLE001 - a broken log fails the gate; the record is kept
            per_log[path.name] = {"error": str(exc)}
        problems += hil_link_problems(path.name, per_log[path.name])
    if not logs:
        problems.append("no board log with HIL records (--log-fetch none: copy the logs from the SD card, "
                        "then run: python -m tools.tuning.hitl link-check)")
    bridge: dict = {"link_csv": str(link_csv)}
    try:
        bridge.update(bridge_resets(link_csv))
    except (OSError, KeyError, csv.Error) as exc:
        reason = getattr(exc, "strerror", None) or (exc.args[0] if exc.args else repr(exc))
        problems.append(f"bridge link log {link_csv}: {reason}")
    else:
        if bridge["resets_after_armed"]:
            problems.append(f"the bridge reset the aircraft to the origin {bridge['resets_after_armed']} time(s) "
                            "after the first armed frame")
        if bridge["conflicts"]:
            problems.append(f"the board sent {bridge['conflicts']} conflicting frame(s): other content at a board "
                            "time already answered (HIL link contract v2: board time strictly increases)")
        if bridge["crashes"]:
            problems.append(f"the simulated aircraft crashed {bridge['crashes']} time(s) (bridge event "
                            f"'{CRASH}': it left the physical envelope; see the bridge's output)")
    return {"stale_fraction_max": HIL_STALE_FRACTION_MAX, "age_limit_us": HIL_AGE_MAX_US, "logs": per_log,
            "bridge": bridge, "problems": problems, "ok": not problems}
