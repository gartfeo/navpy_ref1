"""The board's one MAVLink port shared by the runner, navpy and Mission Planner (HITL final approach).

The board has one USB link (COM15). This router owns it in its own process and serves each client on its own
loopback UDP port; a client opens udpin:127.0.0.1:PORT and answers whoever sent to it (as Mission Planner does
with mp_mirror.py). Rules (routing, stream-rate claims): mav_router_policy.py. Design:
docs/design/hitl_final_approach.md, section 2.

- One reader thread reads the board and only parses and sends: a UDP send on loopback never waits on a client,
  so a slow client cannot hold up the board's packets to the others.
- One writer thread drains a single queue that each client's receiver thread fills, so the clients' packets reach
  the board in arrival order and neither client can block the other or the reader.
- A USB board's port vanishes when it reboots: the reader reopens it, retried every OPEN_RETRY_S (the OS gives no
  event when the port comes back); client packets are dropped and counted while it is closed, and packets queued
  under an earlier link are dropped, never replayed into a new boot. A failed write counts as a lost link. After each
  (re)open the first board HEARTBEAT re-sends the claimed intervals (a new boot starts from its stream rates).
- An observer client (Mission Planner) cannot fly or change rates: mp_mirror.blocked() packets are dropped.
- One owner of the board handle: writes, opens and closes run under one lock, so a reboot (the reader closing the
  port) can never meet a write in progress.
- Endpoint barrier (the terminal handoff, design section 2): a JSON request on the loopback control port closes a
  client's endpoint; from then on its packets are refused, those still queued are dropped, any write in progress has
  returned, and the reply says how many were dropped and when that client's last packet was written. "open" re-opens
  it (the next navpy process). With --write-log every packet written to the board is logged with its source.

Run: python -m tools.tuning.mav_router --board COM15 --client runner=9641 --client navpy=9642 [--observer mp=9640]
     [--control 9643] [--write-log FILE] [--stats FILE] [--exit-on-stdin-eof]
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
from pathlib import Path
from typing import Optional

from tools.tuning.hitl_bridge.process_priority import raise_bridge_priority
from tools.tuning.hitl_bridge.transport import make_transport
from tools.tuning.mav_router_core import ROUTER_SOURCE, MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mp_mirror import reserved_band


def parse_endpoint(text: str) -> tuple[str, int]:
    name, sep, port = text.partition("=")
    if not sep or not name or not port.isdigit():
        raise argparse.ArgumentTypeError(f"expected NAME=PORT, got {text!r}")
    if name == ROUTER_SOURCE:
        raise argparse.ArgumentTypeError(f"{name!r} is the router's own source name")
    port_no = int(port)
    band = reserved_band(port_no)
    if band is not None:
        raise argparse.ArgumentTypeError(f"port {port_no} is in the {band} band")
    return name, port_no


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m tools.tuning.mav_router", description=__doc__.split("\n\n")[0])
    p.add_argument("--board", required=True, help="COMn, /dev/ttyACMn, or tcp:HOST:PORT (a SITL serial port)")
    p.add_argument("--client", type=parse_endpoint, action="append", default=[], metavar="NAME=PORT",
                   help="a client with full authority (the runner, navpy)")
    p.add_argument("--observer", type=parse_endpoint, action="append", default=[], metavar="NAME=PORT",
                   help="a watching client (Mission Planner): no RC, no stream rates, no SYSID_MYGCS")
    p.add_argument("--control", type=int, metavar="PORT",
                   help="loopback UDP control port for the endpoint barrier (JSON {op: close|open, client, id})")
    p.add_argument("--write-log", type=Path, help="log every packet written to the board: monotonic s, source, msgid")
    p.add_argument("--stats", type=Path, help="write the counters here as JSON on exit")
    p.add_argument("--exit-on-stdin-eof", action="store_true",
                   help="exit when stdin reaches end-of-file (when started by another program)")
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    endpoints = [(n, p, False) for n, p in args.client] + [(n, p, True) for n, p in args.observer]
    names = [n for n, _, _ in endpoints]
    ports = [p for _, p, _ in endpoints]
    if not endpoints or len(set(names)) != len(names) or len(set(ports)) != len(ports):
        print("[router] need at least one client, with distinct names and ports", file=sys.stderr)
        return 2
    print(f"[router] {raise_bridge_priority()}", flush=True)
    write_log = args.write_log.open("w", encoding="utf-8", buffering=1) if args.write_log else None
    router = MavRouter(make_transport(args.board), [Client(n, p, o) for n, p, o in endpoints],
                       control_port=args.control, write_log=write_log).start()
    print(f"[router] {args.board} <-> " + ", ".join(f"{n} udp {p}" for n, p, _ in endpoints), flush=True)
    try:
        if args.exit_on_stdin_eof:
            while sys.stdin.read(1):
                pass
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        pass
    finally:
        router.close()
        if write_log is not None:
            write_log.close()
        report = router.report()
        if args.stats is not None:
            args.stats.write_text(json.dumps(report, indent=1), encoding="utf-8")
        print(f"[router] {json.dumps(report)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
