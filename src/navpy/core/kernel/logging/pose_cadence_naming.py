"""File naming and writes for the pose-cadence CSVs.

``pid`` (default) keys each file by process id, so generations never share
one; ``producer`` keys it by producer role (``navpy`` or ``gcs``), so equal
runs write equal names and a later generation of a producer appends.
"""

from __future__ import annotations

import os
from collections.abc import Iterable


NAMING_ENV = "NAVPY_POSE_CADENCE_NAMING"


def resolve_naming() -> str:
    value = os.environ.get(NAMING_ENV, "").strip().lower() or "pid"
    if value not in {"pid", "producer"}:
        raise ValueError(f"{NAMING_ENV} must be 'pid' or 'producer', got {value!r}")
    return value


def validate_producer(name: str) -> str:
    if not name.isalpha() or not name.islower():
        raise ValueError(f"producer must be lowercase letters, got {name!r}")
    return name


def file_owner(naming: str, producer: str) -> str:
    return producer if naming == "producer" else f"pid_{os.getpid()}"


def starts_fresh(naming: str, path: str, first_write: bool) -> bool:
    """Whether this write truncates and writes the header.

    pid naming: the process's first write. Producer naming: only when no
    earlier generation of the producer started the file.
    """
    if not first_write:
        return False
    return naming == "pid" or not os.path.exists(path) or os.path.getsize(path) == 0


def write_rows(path: str, header: str | None, lines: Iterable[str]) -> None:
    """Write a fresh file (``header`` given) or append; all or nothing.

    A failed write is cut back to the length it started from (zero for a
    fresh file), so the retry neither duplicates rows nor extends a partial one.
    """
    start = 0 if header is not None or not os.path.exists(path) else os.path.getsize(path)
    try:
        with open(path, "w" if header is not None else "a",
                  encoding="utf-8", newline="") as fh:
            if header is not None:
                fh.write(header)
            for line in lines:
                fh.write(line)
    except OSError:
        try:
            with open(path, "r+b") as fh:
                fh.truncate(start)
        except OSError:
            pass
        raise


__all__ = [
    "NAMING_ENV",
    "file_owner",
    "resolve_naming",
    "starts_fresh",
    "validate_producer",
    "write_rows",
]
