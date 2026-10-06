"""BIN binding policy for the SIM_CPA cross-check.

Binding is identity-based, never discovery-based: LASTLOG.TXT in the
slot's logs directory names the last log number, and it advances exactly
when a boot starts logging (at arm), so the case's BIN is the recorded
successor of the pre-flight number.  Target matching in the BIN certifies
the CONTENT afterwards; it never chooses the file.  Wrapped numbering is
real on this host (slot 121 holds 500 BINs with LASTLOG at 371), so a
stale higher-numbered file always exists -- the successor rule plus the
mtime-within-case check are what keep binding sound, and the wrap
boundary itself is refused outright (R3/R13).

The BIN is copied only after the slot's SITL process is confirmed gone
(eval_sim_cpa_slot_io.wait_for_sitl_exit); a live BIN is never parsed.
The WSL IO primitives themselves live in eval_sim_cpa_slot_io.py.
"""

from __future__ import annotations

import hashlib
import sys
import time
from pathlib import Path
from typing import Any

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_sim_cpa_block import (  # noqa: E402
    ARTIFACT_ACQUIRED, ARTIFACT_AMBIGUOUS, ARTIFACT_COPY_FAILED,
    ARTIFACT_NO_BIN, ARTIFACT_ROLLOVER, default_artifact,
)
from eval_sim_cpa_slot_io import (  # noqa: E402
    bin_path_for, copy_slot_file, count_bins, read_lastlog,
    resolve_logs_dir, stat_size_mtime,
)

# ArduPilot's file backend wraps log numbering at 500; near the boundary
# the successor rule stops being a simple +1, so binding refuses instead
# of guessing (R13).
ROLLOVER_GUARD_MIN = 499
# WSL and Windows share the wall clock on one host; the slack only covers
# filesystem timestamp granularity and the copy happening seconds later.
MTIME_SLACK_S = 60.0
COPIED_BIN_NAME = "sim_cpa.BIN"


def binding_pre_flight(sysid: int) -> dict[str, Any]:
    """Establish the successor contract before the flight arms.

    Returned dict is the ``binding`` record the verdict carries; its
    ``status`` is "armed" only when a unique successor is established.
    """
    record: dict[str, Any] = {
        "mode": "lastlog_successor",
        "status": "armed",
        "sysid": sysid,
        "logs_dir": None,
        "lastlog_pre": None,
        "expected_number": None,
        "error": None,
        "recorded_wall_time_s": time.time(),
    }
    logs_dir, error = resolve_logs_dir(sysid)
    if logs_dir is None:
        record.update({"status": "unavailable", "error": error})
        return record
    record["logs_dir"] = logs_dir
    number, read_error = read_lastlog(logs_dir)
    if number is None:
        bins = count_bins(logs_dir)
        if bins == 0:
            # A virgin slot: the first log of the first boot is number 1.
            record.update({"lastlog_pre": 0, "expected_number": 1})
            return record
        record.update({
            "status": "ambiguous",
            "error": (
                "LASTLOG.TXT is absent but the directory holds "
                f"{bins if bins is not None else 'an unknown number of'} "
                f"BIN(s) ({read_error}); no successor can be established"
            ),
        })
        return record
    if number >= ROLLOVER_GUARD_MIN:
        record.update({
            "status": "rollover",
            "lastlog_pre": number,
            "error": (
                f"LASTLOG {number} is at the log-number wrap boundary; "
                "successor binding refuses rather than guessing "
                "(clear the slot's logs directory)"
            ),
        })
        return record
    record.update({"lastlog_pre": number, "expected_number": number + 1})
    return record


def acquire_bin(
    binding: dict[str, Any],
    case_dir: Path,
    *,
    case_start_wall_s: float,
) -> dict[str, Any]:
    """Verify the successor after confirmed exit and copy it case-local."""
    artifact = default_artifact()
    artifact["binding"] = binding
    if binding.get("status") != "armed":
        artifact["status"] = (
            ARTIFACT_ROLLOVER if binding.get("status") == "rollover"
            else ARTIFACT_AMBIGUOUS
        )
        artifact["error"] = binding.get("error")
        return artifact
    logs_dir = binding["logs_dir"]
    expected = binding["expected_number"]
    number, read_error = read_lastlog(logs_dir)
    binding["lastlog_post"] = number
    if number is None:
        artifact["status"] = ARTIFACT_NO_BIN
        artifact["error"] = f"LASTLOG unreadable after the flight: {read_error}"
        return artifact
    if number == binding["lastlog_pre"]:
        artifact["status"] = ARTIFACT_NO_BIN
        artifact["error"] = (
            "LASTLOG never advanced: the flight did not start logging "
            "(never armed?)"
        )
        return artifact
    if number != expected:
        artifact["status"] = ARTIFACT_AMBIGUOUS
        artifact["error"] = (
            f"LASTLOG moved {binding['lastlog_pre']} -> {number}, expected "
            f"exactly {expected}; more than one logging session means no "
            "single BIN is this case's evidence"
        )
        return artifact
    source = bin_path_for(logs_dir, number)
    artifact["source_path"] = source
    stat, stat_error = stat_size_mtime(source)
    if stat is None:
        artifact["status"] = ARTIFACT_NO_BIN
        artifact["error"] = f"bound BIN missing: {stat_error}"
        return artifact
    size, mtime = stat
    binding["source_size_bytes"] = size
    binding["source_mtime_epoch_s"] = mtime
    if mtime < case_start_wall_s - MTIME_SLACK_S:
        artifact["status"] = ARTIFACT_AMBIGUOUS
        artifact["error"] = (
            f"bound BIN mtime {mtime} predates the case start "
            f"{case_start_wall_s:.0f}; a stale wrapped file is not this "
            "case's evidence"
        )
        return artifact
    if mtime > time.time() + MTIME_SLACK_S:
        artifact["status"] = ARTIFACT_AMBIGUOUS
        artifact["error"] = (
            f"bound BIN mtime {mtime} is in the future; a skewed clock "
            "cannot certify which flight wrote this file"
        )
        return artifact
    # Copy to a staging name and only rename after verification: an
    # interrupted cp must never leave a partial file under the
    # authoritative name (90_review finding 5; Path.replace is atomic
    # within one directory).
    destination = case_dir / COPIED_BIN_NAME
    staging = case_dir / f"{COPIED_BIN_NAME}.partial"
    copied, copy_error = copy_slot_file(source, staging)
    if not copied:
        artifact["status"] = ARTIFACT_COPY_FAILED
        artifact["error"] = copy_error
        return artifact
    try:
        copied_bytes = staging.read_bytes()
    except OSError as exc:
        artifact["status"] = ARTIFACT_COPY_FAILED
        artifact["error"] = f"copied BIN unreadable: {exc}"
        return artifact
    if len(copied_bytes) != size:
        artifact["status"] = ARTIFACT_COPY_FAILED
        artifact["error"] = (
            f"copied {len(copied_bytes)} bytes but the source stat said "
            f"{size}; the copy is not the bound artifact"
        )
        try:
            staging.unlink()
        except OSError:
            pass
        return artifact
    try:
        staging.replace(destination)
    except OSError as exc:
        artifact["status"] = ARTIFACT_COPY_FAILED
        artifact["error"] = f"verified copy could not be renamed: {exc}"
        return artifact
    artifact.update({
        "status": ARTIFACT_ACQUIRED,
        "copied_path": str(destination),
        "size_bytes": size,
        "sha256": hashlib.sha256(copied_bytes).hexdigest(),
    })
    return artifact


__all__ = [
    "COPIED_BIN_NAME",
    "MTIME_SLACK_S",
    "ROLLOVER_GUARD_MIN",
    "acquire_bin",
    "binding_pre_flight",
]
