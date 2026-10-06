"""SCPC/SCPA decoding and evidence certification for the SIM_CPA module.

The IO half turns a closed DataFlash BIN into plain row dicts; everything
after that is pure logic on those rows so every rejection state is unit
testable without a BIN.  Certification follows the module's interval
contract exactly (SIM_CPA.h): SCPC EpUS is the first interval boundary,
Seq starts at 0 and advances even when a log write is dropped, interval N
spans [EpUS + N*20000, EpUS + (N+1)*20000), a normal row's TimeUS is the
exact end boundary, and partial rows close early with flag bits.

Fail-closed selection: exactly
one epoch may match the expected POI bit-for-bit -- ambiguity is
rejected, never resolved by recency -- and a FAULT anywhere in the
selected epoch rejects the whole epoch.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from eval_sim_cpa_block import (  # noqa: E402
    EVIDENCE_ACCEPTED, EVIDENCE_AMBIGUOUS_EPOCH, EVIDENCE_BAD_ARITHMETIC,
    EVIDENCE_FAULT, EVIDENCE_NO_ROWS, EVIDENCE_NO_SCPC,
    EVIDENCE_PARSE_FAILED, EVIDENCE_POI_MISMATCH,
)

INTERVAL_US = 20_000

EV_ENABLE = 1
EV_DISABLE = 2
EV_REPOI = 3
EV_FAULT = 4
EV_INVALID_CONFIG = 5

FLAG_PARTIAL = 1
FLAG_FINAL = 2


def parse_sim_cpa_records(
    bin_path: Path,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """All SCPC and SCPA rows from a CLOSED BIN, in log order.

    Only ever call this on the case-local copy: parsing a live BIN reads a
    file the simulator is still buffering into.
    """
    from pymavlink import mavutil

    reader = mavutil.mavlink_connection(str(bin_path))
    scpc: list[dict[str, Any]] = []
    scpa: list[dict[str, Any]] = []
    while True:
        message = reader.recv_match(type=["SCPC", "SCPA"], blocking=False)
        if message is None:
            break
        row = message.to_dict()
        (scpc if message.get_type() == "SCPC" else scpa).append(row)
    return scpc, scpa


@dataclass(frozen=True)
class EpochEvidence:
    """One selected epoch's certified rows, or the reason there are none."""

    status: str
    errors: tuple[str, ...] = ()
    epoch: int | None = None
    epoch_us: int | None = None
    rows: tuple[dict[str, Any], ...] = ()
    first_seq: int | None = None
    last_seq: int | None = None
    close_time_us: int | None = None
    missing_seqs: tuple[int, ...] = field(default=())


def _rejected(status: str, *errors: str) -> EpochEvidence:
    return EpochEvidence(status=status, errors=tuple(errors))


def certify_epoch(
    scpc: list[dict[str, Any]],
    scpa: list[dict[str, Any]],
    expected_poi: tuple[int, int, int],
) -> EpochEvidence:
    """Select and certify the case's epoch from parsed rows.

    Internal Seq gaps do NOT reject here: whether a gap matters depends on
    whether the comparison episode needs the missing interval, which only
    the comparison layer knows.  The gap list rides along; duplicate or
    out-of-order Seq and interval-arithmetic violations reject outright
    because they mean the contract itself was broken.
    """
    if not scpc:
        return _rejected(
            EVIDENCE_NO_SCPC, "no SCPC configuration record in the BIN"
        )
    open_events = [
        row for row in scpc if row.get("Ev") in (EV_ENABLE, EV_REPOI)
    ]
    matching = [
        row for row in open_events
        if (row.get("LatE7"), row.get("LngE7"), row.get("AltCM"))
        == expected_poi
    ]
    if not matching:
        observed = [
            (row.get("LatE7"), row.get("LngE7"), row.get("AltCM"))
            for row in open_events
        ]
        return _rejected(
            EVIDENCE_POI_MISMATCH,
            f"no epoch matches POI {expected_poi}; observed {observed}",
        )
    if len(matching) > 1:
        return _rejected(
            EVIDENCE_AMBIGUOUS_EPOCH,
            f"{len(matching)} epochs match the POI bit-for-bit; "
            "recency never disambiguates evidence",
        )
    selected = matching[0]
    epoch = selected.get("Ep")
    epoch_us = selected.get("EpUS")
    if not isinstance(epoch, int) or not isinstance(epoch_us, int):
        return _rejected(
            EVIDENCE_PARSE_FAILED,
            f"SCPC row carries non-integer Ep/EpUS: {selected!r}",
        )
    epoch_events = [row for row in scpc if row.get("Ep") == epoch]
    if any(row.get("Ev") == EV_FAULT for row in epoch_events):
        return _rejected(
            EVIDENCE_FAULT,
            f"epoch {epoch} carries a FAULT event; the whole epoch is "
            "rejected (unknown corruption window)",
        )
    close_time_us = None
    for row in epoch_events:
        if row.get("Ev") in (EV_DISABLE, EV_INVALID_CONFIG) or (
            row.get("Ev") == EV_REPOI and row is not selected
        ):
            time_us = row.get("TimeUS")
            if isinstance(time_us, int) and (
                close_time_us is None or time_us < close_time_us
            ):
                close_time_us = time_us
    # Physical log order is part of the contract: the module appends rows
    # in Seq order, so sorting before validation would silently repair the
    # very corruption this check exists to reject (90_review finding 2).
    rows = [row for row in scpa if row.get("Ep") == epoch]
    if not rows:
        return _rejected(
            EVIDENCE_NO_ROWS, f"no SCPA interval rows in epoch {epoch}"
        )
    sequences = [row.get("Seq") for row in rows]
    if any(not isinstance(seq, int) or seq < 0 for seq in sequences):
        return _rejected(
            EVIDENCE_PARSE_FAILED, f"non-integer Seq values: {sequences!r}"
        )
    missing: list[int] = []
    for previous, current in zip(sequences, sequences[1:]):
        if current == previous:
            return _rejected(
                EVIDENCE_BAD_ARITHMETIC,
                "duplicate Seq values break the interval contract "
                f"(Seq {current} logged twice)",
            )
        if current < previous:
            return _rejected(
                EVIDENCE_BAD_ARITHMETIC,
                f"out-of-order Seq in physical log order ({previous} then "
                f"{current}); the append-only interval contract is broken",
            )
        missing.extend(range(previous + 1, current))
    arithmetic = _interval_arithmetic_errors(rows, epoch_us)
    if arithmetic:
        return _rejected(EVIDENCE_BAD_ARITHMETIC, *arithmetic)
    return EpochEvidence(
        status=EVIDENCE_ACCEPTED,
        epoch=epoch,
        epoch_us=epoch_us,
        rows=tuple(rows),
        first_seq=sequences[0],
        last_seq=sequences[-1],
        close_time_us=close_time_us,
        missing_seqs=tuple(missing),
    )


# Distance fields the comparison consumes: each must be a finite,
# non-negative number.  A NaN here would otherwise flow through every
# threshold check as agreement (90_review finding 1).
_DISTANCE_FIELDS = ("D3", "DH", "DV", "GD3", "GDH", "GDV")


def _interval_arithmetic_errors(
    rows: list[dict[str, Any]], epoch_us: int
) -> list[str]:
    """Violations of the exact 20 ms interval contract."""
    errors: list[str] = []
    for row in rows:
        seq = row["Seq"]
        flags = row.get("Fl", 0)
        for name in _DISTANCE_FIELDS:
            value = row.get(name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0.0
            ):
                errors.append(
                    f"Seq {seq}: {name} is not a finite non-negative "
                    f"number: {value!r}"
                )
        time_us = row.get("TimeUS")
        if not isinstance(time_us, int):
            errors.append(f"Seq {seq}: non-integer TimeUS {time_us!r}")
            continue
        start_us = epoch_us + seq * INTERVAL_US
        boundary_us = start_us + INTERVAL_US
        if flags & FLAG_PARTIAL:
            if not start_us < time_us <= boundary_us:
                errors.append(
                    f"Seq {seq}: partial close time {time_us} outside "
                    f"({start_us}, {boundary_us}]"
                )
        elif time_us != boundary_us:
            errors.append(
                f"Seq {seq}: TimeUS {time_us} is not the exact interval "
                f"boundary {boundary_us}"
            )
        cpa_us = row.get("CpaUS")
        if not isinstance(cpa_us, int) or not (
            start_us <= cpa_us <= (
                time_us if flags & FLAG_PARTIAL else boundary_us
            )
        ):
            errors.append(
                f"Seq {seq}: interval minimum time {cpa_us!r} outside its "
                "own interval"
            )
    return errors


def raw_epoch_global(rows: tuple[dict[str, Any], ...]) -> dict[str, Any]:
    """The last row's running epoch-global minimum, as raw diagnostics.

    Never the comparison value: the epoch spans climb, loiter and approach,
    and a leading nonzero Seq means it can include accumulation from before
    logging started (the episode-mismatch blocker).
    """
    last = rows[-1]
    return {
        "d3_m": last.get("GD3"),
        "dh_m": last.get("GDH"),
        "dv_m": last.get("GDV"),
        "cpa_time_s": (
            last["GCpUS"] / 1e6 if isinstance(last.get("GCpUS"), int) else None
        ),
        "final_flagged": bool(last.get("Fl", 0) & FLAG_FINAL),
    }


__all__ = [
    "EV_DISABLE",
    "EV_ENABLE",
    "EV_FAULT",
    "EV_INVALID_CONFIG",
    "EV_REPOI",
    "EpochEvidence",
    "FLAG_FINAL",
    "FLAG_PARTIAL",
    "INTERVAL_US",
    "certify_epoch",
    "parse_sim_cpa_records",
    "raw_epoch_global",
]
