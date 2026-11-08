"""Per-UAV messages history for the GCS Messages window.

Like Mission Planner's Messages tab: every STATUSTEXT the UAV's own autopilot
or companion sends, plus each change of a NAMED_VALUE_FLOAT / NAMED_VALUE_INT
(for example the autopilot watchdog's "NAVWD"). The latest value of every
named value is kept apart, with its arrival time, so the operator can see one
that stopped arriving.

Attribution is by MAVLink SOURCE system: over the shared radio link every
UAV's traffic reaches every vehicle connection, so a message counts for this
UAV only when its source system is this UAV's system id. The component tells
autopilot, companion and camera apart.

Every entry carries the log's ``epoch``, new for each log instance (each
connection of the vehicle), so the browser can tell a restarted log, whose
``seq`` starts again at 1, from an overlap of entries it already has.
"""
from __future__ import annotations

import collections
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from gcs.backend.companion_identity import (
    COMPANION_COMPONENT_ID,
    COMPANION_TELEMETRY_COMPONENT_IDS,
    source_ids,
)

# Entries kept per UAV. Bounded so a chatty UAV cannot grow the backend
# without limit; about an hour of a busy flight's texts and value changes.
MESSAGE_LOG_CAPACITY = 500

# Distinct named values kept per UAV; past it the one heard least recently is
# dropped. A script naming values by a counter must not grow every snapshot.
NAMED_VALUE_CAPACITY = 32

# STATUSTEXT carries 50 characters; a longer text is sent as chunks sharing a
# non-zero ``id`` with ``chunk_seq`` 0, 1, ..., ended by a chunk shorter than
# 50 characters (MAVLink common.xml). The sender emits the chunks back to back,
# so a text still unfinished this long after its first chunk lost a chunk; it
# is then shown as it stands, marked incomplete.
STATUSTEXT_CHUNK_LEN = 50
STATUSTEXT_CHUNK_WAIT_S = 2.0
# Chunked texts assembled at once per UAV. A sender has one or two in flight;
# past this the oldest is shown as it stands, marked incomplete, so a burst of
# unfinished ids bounds both memory and the work each arrival does.
STATUSTEXT_PENDING_CAPACITY = 8

_AUTOPILOT_COMPONENT_ID = 1

SEVERITY_LABELS = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR",
                   4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
_INFO = 6


def source_label(component: Optional[int]) -> str:
    """Operator name of the sending component."""
    if component is None:
        return "unknown"
    if component == _AUTOPILOT_COMPONENT_ID:
        return "autopilot"
    if component == COMPANION_COMPONENT_ID:
        return "companion"
    if component in COMPANION_TELEMETRY_COMPONENT_IDS:
        return "camera"
    return f"comp {component}"


def _decode(raw) -> str:
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="replace")
    return str(raw or "")


def _text(raw) -> str:
    return _decode(raw).rstrip("\x00").strip()


def _chunk_body(raw: str) -> str:
    """A chunk's text up to its null terminator."""
    return raw.split("\x00", 1)[0]


class _PendingText:
    """The chunks of one chunked STATUSTEXT received so far."""

    def __init__(self, component: Optional[int], severity: int, started: float) -> None:
        self.component = component
        self.severity = severity
        self.started = started
        self.chunks: dict[int, str] = {}
        self.last: Optional[int] = None  # chunk_seq of the terminating chunk

    def conflicts(self, chunk_seq: int, raw: str) -> bool:
        """True for a different chunk under a number already held (a reused id).

        The same chunk again is a duplicate packet, not a conflict.
        """
        held = self.chunks.get(chunk_seq)
        return held is not None and held != _chunk_body(raw)

    def duplicate(self, chunk_seq: int, raw: str) -> bool:
        return self.chunks.get(chunk_seq) == _chunk_body(raw)

    def add(self, chunk_seq: int, raw: str) -> bool:
        """Store a chunk; True once the text is complete."""
        terminated = "\x00" in raw or len(raw) < STATUSTEXT_CHUNK_LEN
        self.chunks[chunk_seq] = _chunk_body(raw)
        if terminated:
            self.last = chunk_seq
        return self.last is not None and all(i in self.chunks for i in range(self.last + 1))

    def text(self) -> str:
        return "".join(self.chunks[i] for i in sorted(self.chunks)).strip()


class VehicleMessageLog:
    """One UAV's messages history and latest named values (thread-safe).

    MAVLink callbacks append from the link thread; the telemetry broadcast
    drains new entries and the REST route reads the backlog.
    """

    def __init__(
        self,
        sys_id: int,
        capacity: int = MESSAGE_LOG_CAPACITY,
        named_capacity: int = NAMED_VALUE_CAPACITY,
        pending_capacity: int = STATUSTEXT_PENDING_CAPACITY,
    ) -> None:
        self.sys_id = int(sys_id)
        self.epoch = rt.uuid4().hex[:12]
        self._entries: collections.deque = collections.deque(maxlen=capacity)
        self._named: dict[str, dict] = {}
        self._named_capacity = named_capacity
        # (component, id) -> chunks of a STATUSTEXT still being received.
        self._pending: dict[tuple[Optional[int], int], _PendingText] = {}
        self._pending_capacity = pending_capacity
        self._seq = 0
        self._drained_seq = 0
        self._lock = rt.Lock()

    def _own_source(self, msg) -> tuple[bool, Optional[int]]:
        ids = source_ids(msg)
        if ids is None:
            # Test / non-MAVLink paths carry no source: accept as this UAV's.
            return True, None
        system, component = ids
        return system == self.sys_id, component

    def _append(self, entry: dict) -> None:
        self._seq += 1
        entry["seq"] = self._seq
        entry["epoch"] = self.epoch
        self._entries.append(entry)

    def _append_text(self, component: Optional[int], severity: int, text: str,
                     ts: float, incomplete: bool = False) -> None:
        entry = {
            "kind": "text",
            "ts": ts,
            "source": source_label(component),
            "severity": severity,
            "label": SEVERITY_LABELS.get(severity, "INFO"),
            "text": text,
        }
        if incomplete:
            entry["incomplete"] = True
        self._append(entry)

    def _retire_locked(self, key: tuple[Optional[int], int]) -> None:
        """Show an unfinished text as it stands, marked incomplete."""
        pending = self._pending.pop(key)
        self._append_text(pending.component, pending.severity, pending.text(),
                          pending.started, incomplete=True)

    def _flush_stale_locked(self, now: float) -> None:
        """Show texts whose missing chunk did not come as incomplete."""
        for key, pending in list(self._pending.items()):
            if now - pending.started >= STATUSTEXT_CHUNK_WAIT_S:
                self._retire_locked(key)

    def on_statustext(self, msg) -> None:
        own, component = self._own_source(msg)
        if not own:
            return
        severity = getattr(msg, "severity", _INFO)
        text_id = int(getattr(msg, "id", 0) or 0)
        now = rt.time()
        with self._lock:
            self._flush_stale_locked(now)
            if text_id == 0:
                self._append_text(component, severity, _text(getattr(msg, "text", "")), now)
                return
            key = (component, text_id)
            chunk_seq = int(getattr(msg, "chunk_seq", 0) or 0)
            raw = _decode(getattr(msg, "text", ""))
            pending = self._pending.get(key)
            if pending is not None and pending.duplicate(chunk_seq, raw):
                return
            if pending is not None and pending.conflicts(chunk_seq, raw):
                # A chunk number seen again with other text: the sender reused
                # the id for a new text, so the old one will never finish.
                self._retire_locked(key)
                pending = None
            if pending is None:
                while len(self._pending) >= self._pending_capacity:
                    # Insertion order: the first key started longest ago.
                    self._retire_locked(next(iter(self._pending)))
                pending = self._pending[key] = _PendingText(component, severity, now)
            if pending.add(chunk_seq, raw):
                del self._pending[key]
                self._append_text(component, pending.severity, pending.text(), pending.started)

    def on_named_value(self, msg) -> None:
        own, component = self._own_source(msg)
        if not own:
            return
        name = _text(getattr(msg, "name", ""))
        if not name:
            return
        value = getattr(msg, "value", None)
        source = source_label(component)
        now = rt.time()
        with self._lock:
            previous = self._named.pop(name, None)
            # Insertion order is recency: the first key was heard longest ago.
            self._named[name] = {"value": value, "source": source, "ts": now}
            while len(self._named) > self._named_capacity:
                del self._named[next(iter(self._named))]
            # The history records a value only when it changes: a 1 Hz
            # watchdog report would otherwise bury every text.
            if previous is None or previous["value"] != value:
                self._append({
                    "kind": "value",
                    "ts": now,
                    "source": source,
                    "severity": _INFO,
                    "label": "VALUE",
                    "name": name,
                    "value": value,
                    "text": f"{name} = {value:g}" if isinstance(value, (int, float)) else f"{name} = {value}",
                })

    def drain_new(self) -> list[dict]:
        """Entries added since the last drain; only the broadcast drains."""
        with self._lock:
            self._flush_stale_locked(rt.time())
            new = [dict(e) for e in self._entries if e["seq"] > self._drained_seq]
            self._drained_seq = self._seq
            return new

    def backlog(self, after: int = 0) -> dict:
        """Every kept entry with ``seq > after`` and the latest named values."""
        with self._lock:
            self._flush_stale_locked(rt.time())
            return {
                "sys_id": self.sys_id,
                "epoch": self.epoch,
                "entries": [dict(e) for e in self._entries if e["seq"] > after],
                "named_values": self.named_values_locked(),
                "last_seq": self._seq,
            }

    def named_values(self) -> dict[str, dict]:
        with self._lock:
            return self.named_values_locked()

    def named_values_locked(self) -> dict[str, dict]:
        return {name: dict(v) for name, v in self._named.items()}
