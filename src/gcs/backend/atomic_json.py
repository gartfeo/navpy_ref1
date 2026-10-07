"""Atomic replacement of small JSON/text documents.

Same temp-file + replace pattern as ``instance_registry_store.save``: the new
content goes to a sibling temp file, is flushed to disk, then renamed over the
target. A process that dies mid-write leaves the previous file untouched
(never truncated), and readers only ever see the old or the new document.
Callers own any locking needed to serialize read-modify-write cycles.
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any


def atomic_write_text(path: Path | str, text: str) -> None:
    """Atomically replace *path* with *text* (UTF-8, platform newlines)."""
    target = Path(path)
    # Unique per process/thread so two writers never share a temp file; a
    # sibling of the target so the rename stays on one filesystem.
    temporary = target.with_name(
        f".{target.name}.{os.getpid()}.{threading.get_ident()}.tmp"
    )
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, target)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path | str, data: Any) -> None:
    """Atomically replace *path* with ``data`` as indented JSON."""
    atomic_write_text(path, json.dumps(data, indent=2) + "\n")
