"""The determinism summary, and the text and the writes all the evidence shares.

Split from ``direct_pixel_pn_child`` because that child sits near its 300-line
module budget, and the reasoning below is not worth cutting to fit.

WHY THIS EXISTS. The trace is a BOUNDED IN-MEMORY RING. Landing 1 wired the
recorder and the post-leg reduction and tested both, but nothing outside the
tests ever called ``summarise`` -- so a traced leg recorded every decision
perfectly and then threw all of it away when the process exited.
``DirectTargetPixelSource.determinism_trace`` describes itself as "the decision
trace for the owner to drain AFTER the scored leg"; no owner ever did. This is
the owner doing it.

That is the instrument's own recurring defect wearing a new hat: something that
reports clean by not reporting at all.

THE SUMMARY is ``{"finalization": ..., "summary": ..., "error": ...}``. The
finalization block says what it can and cannot vouch for, and only the
teardown knows that (``pixel_pn_child_teardown``). It is written even when
there is no summary: an error on its own says nothing about which leg ended or
what else failed on the way down. Since delivery step 6 it is one of three
files made from one sealed capture, and ``pixel_pn_determinism_evidence``
writes them, with the rule for what is raised and when.

EVERY FILE IS WRITTEN BESIDE ITSELF, THEN MOVED INTO PLACE (``write_replacing``),
so a file a reader finds is whole. An interrupt mid-write leaves a half-written
copy under ``STAGING_SUFFIX`` and the file untouched. Written in place, as the
summary was until step 6, a truncated file stood where the summary should be.
"""

from __future__ import annotations

import json
import os
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, TypeVar

from navpy.modules.vision.sim.determinism_evidence import SUMMARY_NAME
from navpy.modules.vision.sim.determinism_trace import TraceCapture
from navpy.modules.vision.sim.determinism_trace_summary import (
    summarise_capture,
)

_T = TypeVar("_T")

# The name a file is written under before it is moved into place.
STAGING_SUFFIX = ".tmp"
UNWRITABLE_MARKER = "DETERMINISM_EVIDENCE_UNWRITABLE"
# Why there is no summary when close() named no leg. Spelled out because the
# tempting alternative is silent: ``summarise(epoch=None)`` means EVERY leg,
# and would label the cruise and scored legs merged as this case.
UNKNOWN_EPOCH_ERROR = "ending epoch unknown: close() did not return one"
# What a file holds where a value's text could not be made: its repr() raised.
REPR_FAILED = "<repr() failed>"


def summary_payload(
    capture: TraceCapture | None,
    *,
    epoch: int | None,
    finalization: Mapping[str, Any],
    error: str | None,
    interrupts: list[BaseException],
) -> dict[str, Any]:
    """The summary file: the ended leg reduced, or why it is not.

    ``capture`` is the ONE sealed capture all the evidence is made from, and
    None when it could not be taken; ``error`` then says why.

    ``epoch`` is REQUIRED and is the one ``close()`` returned. Resolving it from
    the rows named the wrong leg -- a dispatch that takes its frame after
    close() records at the NEXT epoch -- or no leg, since a boundary with both
    slots empty recorded nothing (review round 1, F1 and F3). Its own LIFECYCLE
    row, since Landing 2, can still be lost to an overflow or a recorder fault.
    ``None`` means close() returned no epoch, and gives an error rather than a
    summary of every leg.

    A summary that fails is its error here, never raised. An interrupt is
    held in ``interrupts``, for the writer to raise once every file was
    attempted.
    """
    payload: dict[str, Any] = {
        "finalization": {**finalization, "epoch": epoch},
        "summary": None,
        "error": error,
    }
    if capture is None:
        return payload
    if epoch is None:
        payload["error"] = UNKNOWN_EPOCH_ERROR
        return payload
    payload["summary"], payload["error"] = held(
        lambda: summarise_capture(capture, epoch=epoch), interrupts
    )
    return payload


def held(
    make: Callable[[], _T], interrupts: list[BaseException]
) -> tuple[_T | None, str | None]:
    """``make()`` and None, or None and the text of what it raised.

    Nothing is swallowed: an interrupt is appended to ``interrupts``, for its
    owner to raise once its files were attempted, and so is one that lands
    while the text is made (``deferred_repr``).
    """
    try:
        return make(), None
    except Exception as exc:  # noqa: BLE001 - written where the result goes
        return None, deferred_repr(exc, interrupts)
    except BaseException as exc:
        interrupts.append(exc)
        return None, deferred_repr(exc, interrupts)


def write_replacing(path: Path, make: Callable[[], bytes]) -> str | None:
    """Write ``make()`` beside ``path``, then move it into place.

    None once the file is in place. An OSError costs that file alone: it is
    announced on stdout, which the harness captures, and its text returned.
    Anything else, the announcement's own failure included, is raised, for
    the owner to hold (``pixel_pn_determinism_evidence``).
    """
    staging = path.with_name(path.name + STAGING_SUFFIX)
    try:
        staging.write_bytes(make())
        os.replace(staging, path)
    except OSError as exc:
        text = artifact_repr(exc)
        print(f"{UNWRITABLE_MARKER} {path}: {text}", flush=True)
        return text
    return None


def artifact_json(payload: Mapping[str, Any]) -> bytes:
    """``payload`` as the bytes of a JSON file.

    ``artifact_repr`` stands in for a value ``json`` cannot encode, so one
    unexpected value degrades to readable text instead of losing the whole
    file to a serialiser error.
    """
    return json.dumps(payload, indent=2, default=artifact_repr).encode("utf-8")


def artifact_repr(value: object) -> str:
    """``repr(value)``, or ``REPR_FAILED`` when that raises.

    The one way a value becomes text in the evidence: an error the
    teardown, the ledger subscription or the writer caught, and a value
    ``json`` cannot encode. Any of them can be a foreign object whose repr
    raises, and a repr that raised where nothing caught it cost the
    artifact it was made for. Only an interrupt passes.
    """
    try:
        return repr(value)
    except Exception:  # noqa: BLE001 - the file is still written
        return REPR_FAILED


def deferred_repr(value: object, interrupts: list[BaseException]) -> str:
    """``artifact_repr(value)``, with an interrupt held for its owner.

    For the owner of a file that is still to be written. An interrupt that
    lands while the text is made is appended to ``interrupts`` and the text
    is ``REPR_FAILED``; the owner raises it once the write has been
    attempted. Raised where it landed, it skipped that write (delivery step
    5, review round 2). Nothing is swallowed.
    """
    try:
        return artifact_repr(value)
    except BaseException as interrupted:  # noqa: BLE001 - the owner raises it
        interrupts.append(interrupted)
        return REPR_FAILED


__all__ = [
    "REPR_FAILED",
    "STAGING_SUFFIX",
    "SUMMARY_NAME",
    "UNKNOWN_EPOCH_ERROR",
    "UNWRITABLE_MARKER",
    "artifact_json",
    "artifact_repr",
    "deferred_repr",
    "held",
    "summary_payload",
    "write_replacing",
]
