"""The run journal: one canonical record per tick, hash-chained.

Each record is canonical JSON (``part_wire_codec.dumps``) of run facts only:
the tick, phase and ``t_k``; hashes of the raw SITL frame and reply and of
every participant's raw ``TICK`` and ``DONE``; discarded egress per link;
the lifecycle records committed; drain egress; the parked frame; outcomes.
Host facts (PIDs, ports, paths, exit codes, wall time) never enter it.
The chain is ``sha256(previous chain + record)``.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

from navpy.sim.lockstep.part_wire import Lifecycle
from navpy.sim.lockstep.part_wire_codec import dumps


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def lifecycle_record(entry: Lifecycle, recipient: int) -> dict[str, Any]:
    return {
        "kind": entry.kind, "id": list(entry.id),
        "request": None if entry.request is None else list(entry.request),
        "ordinal": entry.ordinal, "generation": entry.generation, "result": entry.result,
        "payload": entry.payload, "to": recipient,
    }


class RunJournal:
    def __init__(self) -> None:
        self._chain = bytes(32)
        self.records: list[dict[str, Any]] = []

    def add(self, record: dict[str, Any]) -> str:
        data = dumps(record)
        self._chain = hashlib.sha256(self._chain + data).digest()
        self.records.append(record)
        return self._chain.hex()

    @property
    def digest(self) -> str:
        return self._chain.hex()

    def write(self, path: Path) -> None:
        """One canonical JSON record per line, then the digest in a sibling file."""
        path.write_bytes(b"".join(dumps(record) + b"\n" for record in self.records))
        path.with_name(path.stem + "_digest.txt").write_text(self.digest + "\n", encoding="ascii")


class TickRecord:
    """Builds one tick's journal record while the tick runs."""

    def __init__(self, k: int, phase: str, t_us: int) -> None:
        self.data: dict[str, Any] = {
            "k": k, "phase": phase, "t_us": t_us, "frame": None, "reply": None,
            "discarded": {}, "members": [], "lifecycle": [], "drain_egress": [],
        }

    def discard(self, device: str, count: int) -> None:
        if count:
            self.data["discarded"][device] = self.data["discarded"].get(device, 0) + count

    def member(self, ordinal: int, tick: bytes, done: bytes, generation: int | None = None) -> None:
        entry: dict[str, Any] = {"ordinal": ordinal, "tick": sha256(tick), "done": sha256(done)}
        if generation is not None:  # a routed fleet's records (a delivery names its receiver)
            entry["generation"] = generation
        self.data["members"].append(entry)

    def lifecycle(self, entry: Lifecycle, recipient: int) -> None:
        self.data["lifecycle"].append(lifecycle_record(entry, recipient))

    def drain_egress(self, ordinal: int, device: str, data: bytes) -> None:
        if data:
            self.data["drain_egress"].append([ordinal, device, len(data), sha256(data)])

    def note(self, key: str, value: Any) -> None:
        self.data[key] = value


__all__ = ["RunJournal", "TickRecord", "lifecycle_record", "sha256"]
