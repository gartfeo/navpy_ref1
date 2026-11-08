"""The HITL contact channel's link file and navpy's UDP end (hitl_final_approach.md step 26).

The link file is written per run by ``tools/hitl/contact_service.py link`` into the run's private folder: the run id
and mode it belongs to, the run's random key and the two loopback endpoints. navpy and the service both read it; it
holds no pose. ``UdpContactLink`` binds navpy's endpoint, sends to the service's, and hands every datagram that came
from the service's address to one receiver on its own thread; ``close`` wakes and joins that thread.
"""
from __future__ import annotations

import ipaddress
import json
import logging
import socket
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.contact import hitl_contact_wire as wire

_LOG = logging.getLogger(__name__)

LINK_SCHEMA = "navpy.hitl.contact_link.v1"
# Stage 1 runs the service beside SITL on the PC: both ends stay on the loopback (stage 2 step 5 moves navpy).
LOOPBACK_ONLY = True
LINK_FIELDS = frozenset({"schema", "run_id", "mode", "key_hex", "service", "navpy"})
_RECEIVE_BYTES = wire.MAX_DATAGRAM_BYTES + 1    # one byte over the limit, so an oversize datagram is seen as such
# A bounded join: close never hangs the stop. A receiver still busy past it (a contact waiting on the navigation
# lock) is logged and the link stays open for another close; it ends as soon as the receiver returns.
JOIN_S = 5.0


@dataclass(frozen=True)
class ContactLinkConfig:
    run_id: str
    mode: str
    key: bytes
    service: tuple[str, int]
    navpy: tuple[str, int]

    def record(self) -> dict[str, Any]:
        return {"schema": LINK_SCHEMA, "run_id": self.run_id, "mode": self.mode, "key_hex": self.key.hex(),
                "service": f"{self.service[0]}:{self.service[1]}", "navpy": f"{self.navpy[0]}:{self.navpy[1]}"}


def parse_endpoint(text: Any) -> tuple[str, int]:
    """``IPv4:PORT`` on the loopback; ValueError otherwise (no name lookups)."""
    host, sep, port = str(text).rpartition(":")
    if not sep or not port.isdigit() or not 0 < int(port) < 65536:
        raise ValueError(f"{text!r}: expected IPv4:PORT")
    try:
        address = ipaddress.IPv4Address(host)
    except ValueError:
        raise ValueError(f"{text!r}: expected IPv4:PORT") from None
    if LOOPBACK_ONLY and not address.is_loopback:
        raise ValueError(f"{text!r}: the stage 1 contact link stays on the loopback")
    return str(address), int(port)


def config_from_record(record: Any) -> ContactLinkConfig:
    if not isinstance(record, dict) or set(record) != LINK_FIELDS or record["schema"] != LINK_SCHEMA:
        raise ValueError("not a contact link record")
    if not isinstance(record["run_id"], str) or not wire.RUN_ID.fullmatch(record["run_id"]):
        raise ValueError("contact link run_id is not a run id")
    if not isinstance(record["mode"], str):
        raise ValueError("contact link mode is not text")
    try:
        key = wire.valid_key(bytes.fromhex(record["key_hex"]))
    except (TypeError, ValueError):
        raise ValueError(f"contact link key is not {wire.KEY_BYTES} bytes of hex") from None
    service, navpy = parse_endpoint(record["service"]), parse_endpoint(record["navpy"])
    if service == navpy:
        raise ValueError("contact link endpoints must differ")
    return ContactLinkConfig(record["run_id"], record["mode"], key, service, navpy)


def load_link(path: str | Path) -> ContactLinkConfig:
    with open(path, encoding="utf-8") as handle:
        record = json.load(handle)
    try:
        return config_from_record(record)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from None


class UdpContactLink:
    """navpy's end: bind, send to the service, deliver the service's datagrams on one thread."""

    def __init__(self, config: ContactLinkConfig, receiver: Callable[[bytes], object],
                 join_s: float = JOIN_S) -> None:
        self._join_s = join_s
        self._service = config.service
        self._receiver = receiver
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._sock.bind(config.navpy)
        except OSError:
            self._sock.close()
            raise
        self._address = self._sock.getsockname()
        self._stopping = rt.Event()
        self.foreign = 0       # datagrams from any other address: dropped unread
        self._thread = rt.Thread(target=self._run, name="hitl-contact-link", daemon=True)
        self._thread.start()

    def send(self, datagram: bytes) -> None:
        self._sock.sendto(datagram, self._service)

    def close(self) -> bool:
        """Wake the reader with one datagram to itself and join it; close the socket once it has ended.

        True once the thread is gone. A reader still inside the receiver past the bounded join is logged and the
        socket stays open; a later close joins again. Safe to call any number of times.
        """
        if not self._stopping.is_set():
            self._stopping.set()
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as waker:
                    waker.sendto(b"", self._address)
            except OSError:
                _LOG.exception("HITL contact link wake failed")
        self._thread.join(self._join_s)
        if self._thread.is_alive():
            _LOG.error("HITL contact link: the receiver is still busy after %.1f s", self._join_s)
            return False
        self._sock.close()
        return True

    @property
    def running(self) -> bool:
        return self._thread.is_alive()

    def _run(self) -> None:
        while not self._stopping.is_set():
            try:
                datagram, source = self._sock.recvfrom(_RECEIVE_BYTES)
            except OSError:
                # Windows reports an earlier send to a closed port on the next receive; keep reading.
                continue
            if self._stopping.is_set():
                return
            if source != self._service:
                self.foreign += 1
                continue
            try:
                self._receiver(datagram)
            except Exception:  # noqa: BLE001 - one bad datagram never stops the link
                _LOG.exception("HITL contact receiver failed")


__all__ = ["ContactLinkConfig", "LINK_SCHEMA", "UdpContactLink", "config_from_record", "load_link",
           "parse_endpoint"]
