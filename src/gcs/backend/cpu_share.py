"""The machine-wide CPU share record (``bounded_cpu_share.md``).

A wall run's stack holds a share: a few performance cores for a limited
time, while everything else runs as background on the other cores at lower
priority. ``cpu_shares.json``, next to the instance registry, lists the
shares.

An entry holds its CPUs while its owner (pid and exact creation time) may be
alive and has not released it; the next transaction drops an entry whose
owner is known dead: no running process has the pid, or the process has
another creation time. An identity that cannot be read keeps its entry
(fail closed). Background work avoids an entry only before its
``expires_at``. Every transaction, reads included, takes the record's named
mutex: on Windows an open reader makes the atomic replace fail. A
transaction holds it for milliseconds, so each waits for it at most the
registry's lock timeout. A claim that finds no free CPUs is refused at once:
nothing waits for a share. There is no renewal.
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

from gcs.backend import instance_registry_runtime as runtime
from gcs.backend.cpu_share_native import NamedMutex, pid_gone
from gcs.backend.instance_registry_store import LOCK_TIMEOUT_S, registry_path

#: Logical CPUs per share: one per NavPy of a 3-UAV run plus its backend
#: (m1: NavPy about 1.0 core each, p95 1.3; backend 0.4).
SHARE_CPUS = 4
#: Two shares cover the 8 performance cores of this PC's CPU and leave the
#: efficiency cores to everyone else.
MAX_SHARES = 2


@dataclass(frozen=True)
class Share:
    id: str
    pid: int
    start: int | None
    label: str
    cpus: tuple[int, ...]
    expires_at: float

    @classmethod
    def from_json(cls, raw: dict) -> Share:
        return cls(str(raw["id"]), int(raw["pid"]), raw.get("start"), str(raw["label"]),
                   tuple(int(cpu) for cpu in raw["cpus"]), float(raw["expires_at"]))


def shares_path() -> Path:
    return registry_path().with_name("cpu_shares.json")


def owner_alive(pid: int, start: int | None) -> bool:
    """False only for an owner known dead: its pid is gone, or now names a
    process with another creation time (a reused pid)."""
    if pid_gone(pid):
        return False
    current = runtime.pid_start_time(pid)
    return current is None or start is None or current == start


def pick_cpus(classes: dict[int, int], taken: set[int], count: int = SHARE_CPUS) -> list[int]:
    """*count* of the fastest CPUs outside *taken*, or fewer when none are left.

    On a CPU without efficiency classes every CPU is in the fastest class.
    """
    if not classes:
        return []
    fastest = max(classes.values())
    free = sorted(cpu for cpu, cls in classes.items() if cls == fastest and cpu not in taken)
    return free[:count]


class ShareRecord:
    """Claims, releases and reads of the record; the clock and owner checks are ports."""

    def __init__(
        self,
        path: Path | None = None,
        *,
        alive: Callable[[int, int | None], bool] = owner_alive,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.path = path or shares_path()
        self._alive = alive
        self._clock = self.clock = clock
        digest = hashlib.sha1(os.path.normcase(str(self.path.resolve())).encode()).hexdigest()
        # Without its lock the record is unusable: claims are refused and
        # background avoids nothing.
        self._mutex: NamedMutex | None = None
        self._mutex_error = ""
        try:
            self._mutex = NamedMutex(f"navpy-cpu-shares-{digest[:16]}")
        except OSError as error:
            self._mutex_error = str(error)

    def claim(self, label: str, seconds: float, classes: dict[int, int]) -> Share | str:
        """A new share for this process, or why it was refused."""
        if not classes:
            return "no CPU information on this platform"
        pid = os.getpid()
        start = runtime.pid_start_time(pid)
        if start is None:
            return "this process's start time is unreadable"
        if self._mutex is None:
            return f"the share record's lock is unavailable: {self._mutex_error}"
        if not self._mutex.acquire(LOCK_TIMEOUT_S):
            return "the share record stayed busy"
        try:
            held = self._held(self._load())
            if len(held) >= MAX_SHARES:
                return f"{len(held)} shares are held (at most {MAX_SHARES})"
            cpus = pick_cpus(classes, {cpu for share in held for cpu in share.cpus})
            if len(cpus) < SHARE_CPUS:
                return "no free performance cores"
            share = Share(os.urandom(8).hex(), pid, start, label, tuple(cpus),
                          self._clock() + float(seconds))
            self._save(held + [share])
            return share
        except (OSError, ValueError) as error:
            return f"the share record is unusable: {error}"
        finally:
            self._mutex.release()

    def release(self, share: Share) -> bool:
        """Remove *share*; False when the record stayed busy or unusable (its
        CPUs stay held until this process exits)."""
        if self._mutex is None or not self._mutex.acquire(LOCK_TIMEOUT_S):
            return False
        try:
            self._save([held for held in self._held(self._load()) if held.id != share.id])
            return True
        except (OSError, ValueError):
            return False
        finally:
            self._mutex.release()

    def avoided(self) -> tuple[set[int], float | None]:
        """CPUs background work avoids now, and the earliest of their expiries.

        A busy or unreadable record avoids nothing; priority still ranks the
        shares first.
        """
        try:
            shares = self._locked_load()
        except ValueError:
            return set(), None
        now = self._clock()
        valid = [share for share in self._held(shares) if now < share.expires_at]
        if not valid:
            return set(), None
        return {cpu for share in valid for cpu in share.cpus}, min(s.expires_at for s in valid)

    def listing(self) -> list[tuple[Share, str]]:
        """Every entry with its state; it changes nothing."""
        shares = self._locked_load()
        now = self._clock()
        rows = []
        for share in shares:
            if not self._alive(share.pid, share.start):
                state = "owner gone"
            elif now < share.expires_at:
                state = f"held, {share.expires_at - now:.0f} s left"
            else:
                state = "expired, held until released"
            rows.append((share, state))
        return rows

    def _held(self, shares: list[Share]) -> list[Share]:
        return [share for share in shares if self._alive(share.pid, share.start)]

    def _locked_load(self) -> list[Share]:
        if self._mutex is None:
            raise ValueError(f"the share record's lock is unavailable: {self._mutex_error}")
        if not self._mutex.acquire(LOCK_TIMEOUT_S):
            raise ValueError("the share record stayed busy")
        try:
            return self._load()
        finally:
            self._mutex.release()

    def _load(self) -> list[Share]:
        """The entries; ValueError when the record exists but cannot be read."""
        try:
            text = self.path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return []
        except OSError as error:
            raise ValueError(str(error)) from error
        try:
            return [Share.from_json(entry) for entry in json.loads(text)["shares"]]
        except (KeyError, TypeError, ValueError) as error:  # JSONDecodeError is a ValueError
            raise ValueError(f"{type(error).__name__}: {error}") from error

    def _save(self, shares: list[Share]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(f".{os.getpid()}.tmp")
        temporary.write_text(json.dumps({"shares": [asdict(s) for s in shares]}, indent=2),
                             encoding="utf-8")
        temporary.replace(self.path)


__all__ = ["MAX_SHARES", "SHARE_CPUS", "Share", "ShareRecord", "owner_alive", "pick_cpus",
           "shares_path"]
