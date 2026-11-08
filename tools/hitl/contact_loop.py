"""The contact service's process loop (hitl_guidance.md step 26): two loopback UDP sockets, one thread.

navpy's socket (the link's service endpoint) takes arm/disarm datagrams only from navpy's endpoint and sends contacts
there; the pose socket takes the bridge's vehicle_state copy. Everything runs on the calling thread, so the judge
needs no lock; the loop ends on an interrupt (the runner's stop) and prints counts, never a pose.
"""
from __future__ import annotations

import json
import selectors
import socket
from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from navpy.core.kernel.location import Location
from navpy.plugins.contact import hitl_contact_wire as wire
from navpy.plugins.contact.hitl_contact_link import ContactLinkConfig

_RECEIVE_BYTES = 65535
READY_LINE = "contact service: serving"     # printed once both sockets are bound


class EvidenceLog:
    """The service's private JSONL evidence: one new file per run, a line per record, flushed at once."""

    def __init__(self, path: Path) -> None:
        self._file = open(path, "x", encoding="utf-8")

    def __call__(self, record: Mapping[str, Any]) -> None:
        line = {"utc": datetime.now(timezone.utc).isoformat(), **record}
        self._file.write(json.dumps(line, sort_keys=True, default=str) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def serve(config: ContactLinkConfig, dock: Location, poses: tuple[str, int], evidence: Path,
          judge_type: Callable[..., Any]) -> int:
    log = EvidenceLog(evidence)
    navpy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        navpy_sock.bind(config.service)
        pose_sock.bind(poses)
        judge = judge_type(config, dock, lambda datagram: navpy_sock.sendto(datagram, config.navpy), log)
        log({"event": "serving", "run_id": config.run_id})
        print(f"{READY_LINE} run {config.run_id}", flush=True)     # the runner's ready line (no pose)
        with selectors.DefaultSelector() as selector:
            selector.register(navpy_sock, selectors.EVENT_READ, "navpy")
            selector.register(pose_sock, selectors.EVENT_READ, "pose")
            try:
                while True:
                    for key, _ in selector.select():
                        _read(key.fileobj, key.data, config, judge)
            except KeyboardInterrupt:
                pass
        print(json.dumps({"contacts_sent": judge.sent, "refusals": dict(judge.refusals)}, sort_keys=True))
        return 0
    finally:
        navpy_sock.close()
        pose_sock.close()
        log.close()


def _read(sock: Any, role: str, config: ContactLinkConfig, judge: Any) -> None:
    try:
        datagram, source = sock.recvfrom(_RECEIVE_BYTES)
    except OSError:
        return      # Windows reports an earlier send to a closed port here
    if role == "pose":
        judge.pose(datagram, source)
    elif source != config.navpy:
        judge.refusals["foreign source"] += 1
    elif len(datagram) <= wire.MAX_DATAGRAM_BYTES:
        judge.from_navpy(datagram)
    else:
        judge.refusals["oversize"] += 1


__all__ = ["EvidenceLog", "READY_LINE", "serve"]
