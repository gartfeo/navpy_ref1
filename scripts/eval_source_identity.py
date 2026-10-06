"""Which code a sweep actually flew, as one hash.

A sweep is only comparable if every case flew the SAME code, and this worktree
is routinely dirty. The 20260814-001114 sweep had to be discarded entirely
because the navigation constant changed partway through it and nothing in the
artefacts recorded that.

The set of files that matters was hand-maintained, and that is exactly how it
went stale: the flight child grew an import of `pixel_pn_terminal_speed`, which
decides whether the scored leg runs at the requested clock, and which in turn
reads `CLOCK_RATE_TOLERANCE` out of `swarm_run_verification_model`. Neither was
in the list, so editing either changed how a case flew without changing its
recorded SHA.

So the list is derived rather than written down. Starting from the child, this
follows top-level imports and keeps every one that resolves to a module in the
same scripts directory, transitively. That is deliberately narrow: `navpy` is
hashed as a tree by the caller, and everything outside those two places
(pymavlink, the GCS package, the standard library) is a dependency of the
harness rather than part of the code under test, and would make the identity so
brittle that unrelated edits aborted sweeps.

Function-local imports are not followed, which is correct here rather than a
gap: a module the child never executes cannot change how the child flies.
"""

from __future__ import annotations

import ast
import hashlib
from pathlib import Path


def _imported_names(source: str) -> set[str]:
    """Top-level module names imported by a file, `scripts.` prefix stripped."""
    names: set[str] = set()
    for node in ast.parse(source).body:
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
            names.add(node.module)
            if node.module == "scripts":
                # `from scripts import eval_x as y` carries its scripts-local
                # modules in the ALIASES; node.module alone loses them, which
                # is how the three-UAV module escaped the fleet closure.
                names.update(alias.name for alias in node.names)
    return {
        name.split(".", 1)[1] if name.startswith("scripts.") else name
        for name in names
    }


def script_import_closure(entry: Path, scripts_dir: Path) -> tuple[Path, ...]:
    """`entry` plus every scripts-local module it reaches, sorted and unique."""
    found: set[Path] = set()
    pending = [entry.resolve()]
    while pending:
        current = pending.pop()
        if current in found or not current.is_file():
            continue
        found.add(current)
        for name in _imported_names(current.read_text(encoding="utf-8")):
            candidate = (scripts_dir / f"{name.split('.')[0]}.py").resolve()
            if candidate.is_file() and candidate not in found:
                pending.append(candidate)
    return tuple(sorted(found))


def source_identity(roots: tuple[Path, ...], relative_to: Path) -> dict[str, object]:
    """Content hash of the code that decides how a run flies."""
    digest = hashlib.sha256()
    counted = 0
    for root in roots:
        paths = sorted(root.rglob("*.py")) if root.is_dir() else [root]
        for path in paths:
            digest.update(str(path.relative_to(relative_to)).encode())
            digest.update(path.read_bytes())
            counted += 1
    return {"sha256": digest.hexdigest(), "files": counted}


__all__ = ["script_import_closure", "source_identity"]
