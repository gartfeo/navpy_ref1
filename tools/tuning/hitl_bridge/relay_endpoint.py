"""One side of the Windows relay (relay.py): a transport, its state, and when a connection counts.

A connection counts as up ("confirmed") once a byte arrived on it or it lasted
CONFIRM_AFTER_S. Before that it is only tentative: it is not logged as open, it does not end
an outage (the relay's bridge-gone timer keeps running), and when it closes without data the
next attempt waits the retry interval. Reason: WSL's localhost forwarding keeps accepting on
Windows 127.0.0.1:<port> after the WSL listener is gone and closes each connection at once
(review: 42 open/lost pairs, and the relay ended 6 s after the bridge instead of 2 s).
Bytes written into a tentative connection that then closed are counted as lost, not forwarded.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from tools.tuning.hitl_bridge.transport import Transport

# A phantom connection closes within milliseconds; a real bridge keeps its connection for
# the whole run and answers the first board frame within milliseconds, so one second
# without data and without a close tells them apart with a wide margin on both sides.
CONFIRM_AFTER_S = 1.0


@dataclass
class DirectionStats:
    bytes: int = 0
    chunks: int = 0
    dropped_bytes: int = 0          # arrived while the other side was down


class Endpoint:
    """One side of the relay. Its own thread opens, reads and closes it; the other
    thread only writes to it (under the lock, so a close never cuts a write in half)."""

    def __init__(self, name: str, transport: Transport, retry_interval_s: float, log: Callable[[str], None],
                 clock: Callable[[], float] = time.monotonic, confirm_after_s: float = CONFIRM_AFTER_S) -> None:
        self.name = name
        self.retry_interval_s = retry_interval_s
        self._transport = transport
        self._log = log
        self._clock = clock
        self._confirm_after_s = confirm_after_s
        self._lock = threading.Lock()
        self.up = False                 # a connection is open (confirmed or tentative)
        self.failed = False             # a write failed: the owner thread must reopen
        self.confirmed = False
        self._opened_at = 0.0
        self._tentative_bytes = 0       # written into the current connection before it was confirmed
        self.down_since: Optional[float] = None     # since the last confirmed connection ended
        self.closed_without_data = False            # the last connection closed while tentative
        self.opens = 0                  # confirmed connections
        self.losses = 0                 # confirmed connections that ended
        self.tentative_closes = 0       # connections that closed before they were confirmed
        self.lost_bytes = 0             # written into connections that closed before they were confirmed

    @property
    def ever_up(self) -> bool:
        return self.opens > 0

    def try_open(self) -> Optional[OSError]:
        """None when a (tentative) connection is open, else the error."""
        try:
            self._transport.open()
        except OSError as exc:
            return exc
        with self._lock:
            self.up = True
            self.failed = False
            self.confirmed = False
            self._tentative_bytes = 0
        self._opened_at = self._clock()
        return None

    def note_read(self, got_data: bool) -> None:
        """Owner thread, after each read: confirm on the first byte or after CONFIRM_AFTER_S."""
        if self.confirmed or not (got_data or self._clock() - self._opened_at >= self._confirm_after_s):
            return
        with self._lock:
            self.confirmed = True
            self._tentative_bytes = 0
        self.opens += 1
        self.down_since = None
        self.closed_without_data = False
        self._log(f"relay: {self.name} open (#{self.opens})")

    def read(self) -> bytes:
        return self._transport.read()

    def write(self, data: bytes) -> bool:
        with self._lock:
            if not self.up or self.failed:
                return False
            try:
                self._transport.write(data)
            except OSError:
                self.failed = True
                return False
            if not self.confirmed:
                self._tentative_bytes += len(data)
        return True

    def drop(self, reason: object) -> None:
        with self._lock:
            self._transport.close()
            was_confirmed, tentative_bytes = self.confirmed, self._tentative_bytes
            self.up = self.failed = self.confirmed = False
            self._tentative_bytes = 0
        if was_confirmed:
            self.losses += 1
            self.down_since = self._clock()
            self._log(f"relay: {self.name} lost ({reason})")
            return
        self.tentative_closes += 1
        self.lost_bytes += tentative_bytes
        self.closed_without_data = True

    def shutdown(self) -> None:
        with self._lock:
            self._transport.shutdown()
            self.up = False
