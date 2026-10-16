"""One client endpoint of the board link router and the packing helpers it shares (mav_router.py)."""
from __future__ import annotations

import select
import socket
from collections import Counter
from typing import Iterable, Optional

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tools.tuning.mav_router_policy import SET_MESSAGE_INTERVAL, Identity
from tools.tuning.mp_mirror import LOOPBACK, MAX_DATAGRAM

SEQ_MODULO = 256               # MAVLink packet sequence numbers wrap at 256


def pack(encoder: mavlink2.MAVLink, msg: mavlink2.MAVLink_message) -> bytes:
    """msg's bytes from this sender, with the sender's next sequence number (pack() alone does not advance it)."""
    buf = msg.pack(encoder)
    encoder.seq = (encoder.seq + 1) % SEQ_MODULO
    return buf


def msg_id(buf: bytes) -> int:
    """The message id of one packed MAVLink packet (v1 or v2), -1 when it is neither."""
    if len(buf) >= 10 and buf[0] == 0xFD:
        return buf[7] | (buf[8] << 8) | (buf[9] << 16)
    if len(buf) >= 6 and buf[0] == 0xFE:
        return buf[5]
    return -1


def new_parser() -> mavlink2.MAVLink:
    """A fresh MAVLink parser that skips bad bytes instead of raising."""
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    return parser


def interval_command(encoder: mavlink2.MAVLink, target: Identity, message_id: int, interval_us: int) -> bytes:
    """SET_MESSAGE_INTERVAL for one message, packed by this sender."""
    return pack(encoder, encoder.command_long_encode(target.system, target.component, SET_MESSAGE_INTERVAL, 0,
                                                     message_id, interval_us, 0, 0, 0, 0, 0))


def wake_all(clients: Iterable[Client]) -> None:
    """After a transition (link up or down, barrier, reopen): each receiver takes the router's new state at once."""
    for client in clients:
        client.wake()


class Client:
    """One client's UDP endpoint: the router sends from an ephemeral port to 127.0.0.1:port and takes packets only
    from that port."""

    def __init__(self, name: str, port: int, observer: bool) -> None:
        self.name = name
        self.port = port
        self.observer = observer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((LOOPBACK, 0))
        self.parser = new_parser()
        self.identities: set[Identity] = set()
        self.encoders: dict[Identity, mavlink2.MAVLink] = {}
        self.closed = False                          # endpoint barrier passed: packets refused (router lock)
        self.epoch = 0                               # bumped by every barrier: older queued packets never go out
        self.queued = 0                              # its packets in the board queue (router lock)
        self.last_write_s: Optional[float] = None    # time.monotonic() of its last packet written (io lock)

    def close_barrier(self) -> tuple[bool, int]:
        """Refuse its packets from now on; every packet queued or received before now is dropped (caller holds the
        router lock). Returns (already closed, packets dropped from the queue)."""
        already, dropped = self.closed, self.queued
        self.closed, self.epoch, self.queued = True, self.epoch + 1, 0
        return already, dropped

    def reopen(self) -> bool:
        """Take packets again, none received while closed (caller holds the router lock). Returns already open; an
        open endpoint keeps its epoch and queue as they are."""
        if not self.closed:
            return True
        self.closed, self.epoch = False, self.epoch + 1
        self.identities.clear()                      # a new process announces itself again
        return False

    def drain(self) -> int:
        """Discard every datagram already waiting (a transition's backlog); returns how many carried data. Only the
        receiver thread calls it, so a ready socket never blocks."""
        dropped = 0
        while select.select([self.sock], [], [], 0)[0]:
            try:
                data, _ = self.sock.recvfrom(MAX_DATAGRAM)
            except ConnectionResetError:
                continue
            except OSError:
                break
            dropped += bool(data)
        return dropped

    def wake(self) -> None:
        """An empty datagram to its own receiver: it returns from recvfrom and takes the router's state anew."""
        try:
            self.sock.sendto(b"", self.sock.getsockname())
        except OSError:
            pass

    def encoder(self, identity: Identity) -> mavlink2.MAVLink:
        """Packs a command in this client's name (the board then acknowledges to the client)."""
        if identity not in self.encoders:
            self.encoders[identity] = mavlink2.MAVLink(None, srcSystem=identity.system, srcComponent=identity.component)
        return self.encoders[identity]


class SeqGaps:
    """Per board source (system/component): packets the router never saw (USB loss), from MAVLink sequence gaps."""

    def __init__(self) -> None:
        self.gaps: Counter = Counter()
        self._last: dict[tuple[int, int], int] = {}

    def restart(self) -> None:
        """A new link: the next packet of each source starts its count again."""
        self._last.clear()

    def count(self, msg: mavlink2.MAVLink_message) -> None:
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        last = self._last.get(source)
        if last is not None:
            gap = (msg.get_seq() - last - 1) % SEQ_MODULO
            if gap:
                self.gaps[f"{source[0]}/{source[1]}"] += gap
        self._last[source] = msg.get_seq()
