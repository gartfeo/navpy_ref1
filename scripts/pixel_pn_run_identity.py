"""D7b: which code a traced case flew, and which case it was told it was.

The harness writes ``case.json`` beside the result the child publishes
(``pixel_pn_case_manifest``): its own source identity, the case's name, and
the sha256 of the case's definition. The child records the three AS GIVEN,
and one it could not read as absent, with the reason. It checks none of them
and makes none up, so a reader (D8.13) sees exactly what the harness said.

What the child measures itself is the code on disk it imports from: the
files of the navpy tree it imported and of its own scripts-local import
closure, hashed the way ``eval_source_identity`` hashes them, once as the
run starts, before anything is built, and again at teardown. Files that
differ between those two instants show as two different hashes, which D8.13
refuses. Equal hashes say only that the hashed files held the same contents
at the two instants: not that nothing changed in between, since an edit
made and undone while the case flew is in neither hash, nor which code was
loaded, since an edit between the imports and run()'s start is in both
(delivery step 6's review, rounds 1 and 2). A hash that could not be taken
is recorded as its error, never as a hash.

The rest describes the run -- its options, pid, a random run id, the wall
clock at the start and the Python version -- and nothing compares it: wall
time may say when, never what. A description that could not be made is
recorded as its error too: the identity starts the run, and must never stop
it (delivery step 6's review).

Gathered only while the determinism trace is on (``determinism_trace.ENABLED``,
read when called), so with tracing off nothing is read or hashed.
"""

from __future__ import annotations

import json
import os
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Importable as ``pixel_pn_run_identity`` (how the child loads it) and as
# ``scripts.pixel_pn_run_identity`` (how the tests import it). Only the first
# puts this directory on the path, and the sibling imports below need it.
SCRIPTS = str(Path(__file__).resolve().parent)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import navpy  # noqa: E402
from navpy.modules.vision.sim import determinism_trace  # noqa: E402
from eval_source_identity import (  # noqa: E402
    script_import_closure,
    source_identity,
)
from pixel_pn_case_manifest import MANIFEST_NAME as CASE_MANIFEST  # noqa: E402
from pixel_pn_determinism_summary import artifact_repr  # noqa: E402

# The navpy tree this process imported, and the worktree its paths are named
# relative to. Read when called, so a test can point both at a tree of its own.
NAVPY_ROOT = Path(navpy.__path__[0]).resolve()
WORKTREE = Path(__file__).resolve().parent.parent
# What the harness gives in case.json, by key.
_GIVEN = ("source_identity", "case_name", "case_definition_sha256")


def start_identity(options: Any, entry: Path) -> RunIdentity | None:
    """The run's identity, started; None with tracing off.

    ``entry`` is the child's own file, where its import closure starts.
    Nothing here raises but an interrupt.
    """
    if not determinism_trace.ENABLED:
        return None
    given, error = _given(options)
    return RunIdentity(entry, given, error, _endpoint(entry), _run(options))


class RunIdentity:
    """What the harness gave, and the code's hash at the start, kept until
    the teardown finishes the identity."""

    __slots__ = ("_entry", "_given", "_error", "_start", "_run")

    def __init__(
        self,
        entry: Path,
        given: dict[str, Any],
        error: str | None,
        start: dict[str, Any],
        run: dict[str, Any],
    ) -> None:
        self._entry = entry
        self._given = given
        self._error = error
        self._start = start
        self._run = run

    def finish(self) -> dict[str, Any]:
        """The identity, with the code's files hashed again. Nothing here
        raises but an interrupt, which the teardown's step keeps."""
        return {
            "harness": self._given["source_identity"],
            "case": {
                "name": self._given["case_name"],
                "definition_sha256": self._given["case_definition_sha256"],
            },
            "given_error": self._error,
            "endpoints": {
                "start": self._start,
                "teardown": _endpoint(self._entry),
            },
            "run": self._run,
        }


def _given(options: Any) -> tuple[dict[str, Any], str | None]:
    """What case.json, beside the result, says: each key, None when absent,
    and the error that stopped it being read, if one did."""
    try:
        path = Path(options.result).parent / CASE_MANIFEST
        case = json.loads(path.read_text(encoding="utf-8"))
        return {name: case.get(name) for name in _GIVEN}, None
    except Exception as exc:  # noqa: BLE001 - recorded, and the case flies
        return dict.fromkeys(_GIVEN), artifact_repr(exc)


def _endpoint(entry: Path) -> dict[str, Any]:
    """The hash of the code's files, or the error that stopped it being
    taken."""
    try:
        closure = script_import_closure(entry, Path(entry).parent)
        return source_identity((NAVPY_ROOT, *closure), WORKTREE)
    except Exception as exc:  # noqa: BLE001 - recorded, never as a hash
        return {"error": artifact_repr(exc)}


def _run(options: Any) -> dict[str, Any]:
    """A description of the run, and nothing compares it; or the error that
    stopped it being made, so that it never stops the case."""
    try:
        return {
            "options": _described(options),
            "pid": os.getpid(),
            "run_id": uuid.uuid4().hex,
            "wall_start": datetime.now(timezone.utc).isoformat(),
            "python": sys.version,
        }
    except Exception as exc:  # noqa: BLE001 - recorded, and the case flies
        return {"error": artifact_repr(exc)}


def _described(options: Any) -> dict[str, Any] | str:
    """The child's options by name, a path as its text."""
    try:
        return {
            name: str(value) if isinstance(value, Path) else value
            for name, value in sorted(vars(options).items())
        }
    except Exception as exc:  # noqa: BLE001 - a description, nothing more
        return artifact_repr(exc)


__all__ = ["NAVPY_ROOT", "WORKTREE", "RunIdentity", "start_identity"]
