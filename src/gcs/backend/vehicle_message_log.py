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


def _text(raw) -> str:
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="replace")
    return str(raw or "").rstrip("\x00").strip()


class VehicleMessageLog:
    """One UAV's messages history and latest named values (thread-safe).

    MAVLink callbacks append from the link thread; the telemetry broadcast
    drains new entries and the REST route reads the backlog.
    """

    def __init__(self, sys_id: int, capacity: int = MESSAGE_LOG_CAPACITY) -> None:
        self.sys_id = int(sys_id)
        self._entries: collections.deque = collections.deque(maxlen=capacity)
        self._named: dict[str, dict] = {}
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
        self._entries.append(entry)

    def on_statustext(self, msg) -> None:
        own, component = self._own_source(msg)
        if not own:
            return
        severity = getattr(msg, "severity", _INFO)
        text = _text(getattr(msg, "text", ""))
        with self._lock:
            self._append({
                "kind": "text",
                "ts": rt.time(),
                "source": source_label(component),
                "severity": severity,
                "label": SEVERITY_LABELS.get(severity, "INFO"),
                "text": text,
            })

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
            previous = self._named.get(name)
            self._named[name] = {"value": value, "source": source, "ts": now}
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
            new = [dict(e) for e in self._entries if e["seq"] > self._drained_seq]
            self._drained_seq = self._seq
            return new

    def backlog(self, after: int = 0) -> dict:
        """Every kept entry with ``seq > after`` and the latest named values."""
        with self._lock:
            return {
                "sys_id": self.sys_id,
                "entries": [dict(e) for e in self._entries if e["seq"] > after],
                "named_values": self.named_values_locked(),
                "last_seq": self._seq,
            }

    def named_values(self) -> dict[str, dict]:
        with self._lock:
            return self.named_values_locked()

    def named_values_locked(self) -> dict[str, dict]:
        return {name: dict(v) for name, v in self._named.items()}
