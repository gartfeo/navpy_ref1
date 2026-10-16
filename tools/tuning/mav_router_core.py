"""The board link router: one board reader, one board writer, one receiver per client (rules: mav_router.py)."""
from __future__ import annotations

import queue
import threading
import time
from collections import Counter
from typing import Callable, Optional, TextIO

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tools.tuning.hitl_bridge.transport import Transport
from tools.tuning.mav_router_control import ControlServer
from tools.tuning.mav_router_endpoint import Client, SeqGaps, interval_command, msg_id, new_parser, wake_all
from tools.tuning.mav_router_policy import (ROUTER_COMPONENT, ROUTER_SYSTEM, Identity,
                                            StreamClaims, all_streams_rate_hz, destinations, interval_request,
                                            is_router_target)
from tools.tuning.mp_mirror import LOOPBACK, MAX_DATAGRAM, UNPARSED, blocked

OPEN_RETRY_S = 0.5             # board.OPEN_RETRY_S: the OS gives no event when a USB port comes back
ROUTER_SOURCE = "router"       # the source name of the router's own packets in the queue and the write log


class MavRouter:
    """The board link shared by several UDP clients (module notes)."""

    def __init__(self, board: Transport, clients: list[Client],
                 announce: Callable[[str], None] = lambda line: print(f"[router] {line}", flush=True),
                 control_port: Optional[int] = None, write_log: Optional[TextIO] = None) -> None:
        self.board = board
        self.clients = {c.name: c for c in clients}
        self.claims = StreamClaims()
        self.stats: Counter = Counter()
        self.seq_gaps = SeqGaps()                   # per board source: packets the router never saw (USB loss)
        self._announce = announce
        self._lock = threading.Lock()               # identities, claims, stats, endpoint state
        self._io = threading.Lock()                 # the board handle: open, write, close
        self._write_log = write_log
        self._to_board: queue.Queue[Optional[tuple[str, int, int, bytes]]] = queue.Queue()
        self._link_gen = 0                          # bumped at every (re)open: older queued packets never go out
        self._write_failed = threading.Event()
        self._parser = new_parser()
        self._own = mavlink2.MAVLink(None, srcSystem=ROUTER_SYSTEM, srcComponent=ROUTER_COMPONENT)
        self._link_up = threading.Event()
        self._reapply_pending = False
        self._closed = threading.Event()
        self._threads = [threading.Thread(target=self._read_board, name="router-board-reader", daemon=True),
                         threading.Thread(target=self._write_board, name="router-board-writer", daemon=True)]
        self._threads += [threading.Thread(target=self._receive, args=(c,), name=f"router-{c.name}", daemon=True)
                          for c in clients]
        self.control: Optional[ControlServer] = None
        if control_port is not None:
            self.control = ControlServer(control_port, set(self.clients), self.close_endpoint, self.open_endpoint)
            self._threads.append(self.control.thread)

    def start(self) -> MavRouter:
        for t in self._threads:
            t.start()
        return self

    # ---- the board's side ----------------------------------------------------------------
    def _open_board(self) -> bool:
        while not self._closed.is_set():
            try:
                with self._io:
                    self.board.open()
            except OSError:
                with self._io:                      # an open can fail after taking the handle: release it
                    self.board.close()
                self._closed.wait(OPEN_RETRY_S)
                continue
            with self._lock:
                self._link_gen += 1
                self._write_failed.clear()
                self._parser = new_parser()
                self.seq_gaps.restart()
                self._reapply_pending = True
                self.stats["board_opened"] += 1
                self._link_up.set()
            wake_all(self.clients.values())
            return True
        return False

    def _read_board(self) -> None:
        while self._open_board():
            try:
                while not self._closed.is_set():
                    if self._write_failed.is_set():
                        raise OSError("board write failed")
                    data = self.board.read()
                    if data:
                        for msg in self._parser.parse_buffer(data) or []:
                            self._from_board(msg)
            except OSError:
                with self._lock:
                    self.stats["board_lost"] += 1
            with self._lock:
                self._link_up.clear()
            wake_all(self.clients.values())
            with self._io:                          # never while a write is in progress
                self.board.close()
            if not self._closed.is_set():
                self._announce("board link lost, reopening")

    def _from_board(self, msg: mavlink2.MAVLink_message) -> None:
        if msg.get_type() == UNPARSED:
            with self._lock:
                self.stats["board_unparsed"] += 1
            return
        buf = msg.get_msgbuf()
        with self._lock:
            self.seq_gaps.count(msg)
            if is_router_target(msg):
                self.stats["router_replies"] += 1
                if msg.get_type() == "COMMAND_ACK" and msg.result != mavlink2.MAV_RESULT_ACCEPTED:
                    self.stats["router_interval_refused"] += 1
                return
            names = destinations(msg, {n: c.identities for n, c in self.clients.items()})
            if not names:
                self.stats["board_unrouted"] += 1
            if self._reapply_pending and msg.get_type() == "HEARTBEAT":
                self._reapply_pending = False
                self._queue_claims(Identity(msg.get_srcSystem(), msg.get_srcComponent()))
        for name in names:
            client = self.clients[name]
            try:
                client.sock.sendto(buf, (LOOPBACK, client.port))
                self.stats[f"to_{name}"] += 1
            except OSError:
                with self._lock:
                    self.stats[f"send_errors_{name}"] += 1

    def _write_board(self) -> None:
        while True:
            item = self._to_board.get()
            if item is None:
                return
            source, epoch, link_gen, buf = item
            with self._io:
                with self._lock:
                    client = self.clients.get(source)
                    if client is not None:
                        if epoch != client.epoch or client.closed:  # its barrier already counted it as dropped
                            self.stats[f"dropped_closed_{source}"] += 1
                            continue
                        client.queued -= 1
                    if not self._link_up.is_set() or link_gen != self._link_gen:
                        self.stats["dropped_link_down"] += 1
                        continue
                try:
                    self.board.write(buf)
                except OSError:
                    with self._lock:
                        self.stats["dropped_write_error"] += 1
                        self._link_up.clear()               # the reader closes and reopens the port
                    self._write_failed.set()
                    continue
                now = time.monotonic()
                if client is not None:
                    client.last_write_s = now
                if self._write_log is not None:
                    self._write_log.write(f"{now:.6f},{source},{msg_id(buf)}\n")

    def _link_state(self) -> int:
        """The link generation while the board is open, -1 while it is closed (caller holds the router lock)."""
        return self._link_gen if self._link_up.is_set() else -1

    def _enqueue(self, source: str, buf: bytes) -> None:
        """Queue one packet for the board (caller holds the router lock); dropped at once while the link is down."""
        if not self._link_up.is_set():
            self.stats["dropped_link_down"] += 1
            return
        client = self.clients.get(source)
        if client is not None:
            client.queued += 1
        self._to_board.put((source, client.epoch if client is not None else 0, self._link_gen, buf))

    # ---- the clients' side --------------------------------------------------------------
    def _receive(self, client: Client) -> None:
        """Fail closed: a datagram read across a transition (reboot, barrier, reopen) is dropped with all waiting behind
        it, so nothing from before it reaches the board after it; a partial packet (datagrams are whole) is dropped."""
        with self._lock:
            accepting = (self._link_state(), client.epoch)
        while not self._closed.is_set():
            try:
                data, (_, sender_port) = client.sock.recvfrom(MAX_DATAGRAM)
            except ConnectionResetError:    # Windows: an earlier send found no client listening yet
                continue
            except OSError:                 # closed
                return
            if self._closed.is_set():       # close()'s wake-up
                return
            with self._lock:
                now = (self._link_state(), client.epoch)
                changed, accepting = now != accepting, now
                if not changed and sender_port != client.port:  # an empty one is a wake-up with no transition since
                    self.stats[f"foreign_{client.name}"] += bool(data)
                    continue
            if changed:
                stale = bool(data) + client.drain()
                with self._lock:
                    self.stats[f"dropped_stale_{client.name}"] += stale
                continue
            for msg in client.parser.parse_buffer(data) or []:
                self._from_client(client, msg, *accepting)
            if client.parser.buf_len():     # never joined to the next datagram
                client.parser = new_parser()
                with self._lock:
                    self.stats[f"partial_{client.name}"] += 1

    def _from_client(self, client: Client, msg: mavlink2.MAVLink_message, link_gen: int, epoch: int) -> None:
        kind = msg.get_type()
        with self._lock:
            if kind == UNPARSED:
                self.stats[f"unparsed_{client.name}"] += 1
                return
            if client.closed or epoch != client.epoch:
                self.stats[f"refused_closed_{client.name}"] += 1
                return
            if link_gen < 0 or link_gen != self._link_state():  # link down or an earlier link: no claim, no write
                self.stats["dropped_link_down"] += 1
                return
            if client.observer and blocked(msg):
                self.stats[f"blocked_{client.name}_{kind}"] += 1
                return
            identity = Identity(msg.get_srcSystem(), msg.get_srcComponent())
            client.identities.add(identity)
            self.stats[f"from_{client.name}"] += 1
            request = interval_request(msg)
            if request is not None:
                buf = self._claim(client, identity, msg, *request)
            else:
                buf = msg.get_msgbuf()
            rate_hz = all_streams_rate_hz(msg)
            self._enqueue(client.name, buf)
            if rate_hz is not None:
                self.claims.stream_rate(rate_hz)
            if kind == "REQUEST_DATA_STREAM":    # any such request re-sets intervals of its streams: claims after it
                self._queue_claims(Identity(msg.target_system, msg.target_component))

    def _claim(self, client: Client, identity: Identity, msg: mavlink2.MAVLink_message, message_id: int,
               interval_us: int) -> bytes:
        """The client's SET_MESSAGE_INTERVAL as the board must get it: unchanged when it asks for the effective
        interval, else repacked in the client's name with the effective one (the board acknowledges to it)."""
        effective = self.claims.claim(client.name, message_id, interval_us)
        if effective == interval_us:
            return msg.get_msgbuf()
        self.stats["interval_rewritten"] += 1
        return interval_command(client.encoder(identity), Identity(msg.target_system, msg.target_component),
                                message_id, effective)

    def _queue_claims(self, autopilot: Identity) -> None:
        """Re-send every claimed interval to the autopilot in the router's name (caller holds the lock)."""
        for message_id, interval_us in self.claims.claimed().items():
            self._enqueue(ROUTER_SOURCE, interval_command(self._own, autopilot, message_id, interval_us))
            self.stats["router_intervals_sent"] += 1

    # ---- endpoint barrier ------------------------------------------------------------------
    def close_endpoint(self, name: str) -> dict:
        """Refuse the client's packets from now on, drop those queued, and return once no write of its packets is in
        progress (design section 2: the barrier before the terminal record and before the runner's RTL)."""
        client = self.clients[name]
        with self._lock:
            already, dropped = client.close_barrier()
            self.stats[f"endpoint_closed_{name}"] += not already
        client.wake()
        with self._io:                              # a write in progress has returned; later ones see `closed`
            last = client.last_write_s
        return {"op": "closed", "client": name, "dropped_queued": dropped, "already_closed": already,
                "last_write_mono_s": last, "barrier_mono_s": time.monotonic()}

    def open_endpoint(self, name: str) -> dict:
        client = self.clients[name]
        with self._lock:
            already = client.reopen()
        client.wake()
        return {"op": "opened", "client": name, "already_open": already}

    # ---- lifetime --------------------------------------------------------------------------
    def close(self) -> None:
        self._closed.set()
        self._to_board.put(None)
        if self.control is not None:
            self.control.wake()
        wake_all(self.clients.values())
        for t in self._threads:
            t.join()
        for client in self.clients.values():
            client.sock.close()
        if self.control is not None:
            self.control.close()
        with self._io:
            self.board.shutdown()

    def report(self) -> dict:
        with self._lock:
            return {"stats": dict(self.stats), "board_seq_gaps": dict(self.seq_gaps.gaps),
                    "claims": {str(k): v for k, v in self.claims.claimed().items()},
                    "clients": {n: sorted([i.system, i.component] for i in c.identities)
                                for n, c in self.clients.items()}}
