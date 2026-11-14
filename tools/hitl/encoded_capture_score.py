"""Scores of the encoded capture's live check; hitl_guidance.md section 9, stage 1 step 9."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

RTP_TICK_NS = 11_112        # 1 / 90 kHz, rounded up
INTERVAL_TOLERANCE_NS = 2 * RTP_TICK_NS     # one quantisation per side
ANCHOR_WINDOW = 200


@dataclass(frozen=True)
class IntervalScore:
    decoded: int
    sender_in_window: int
    unmatched_decoded: int
    missing_sender: int
    repeated_pts: int
    worst_error_ns: int

    @property
    def ok(self) -> bool:
        return self.decoded > 0 and not (self.unmatched_decoded or self.missing_sender or self.repeated_pts)


def _find_anchor(decoded: Sequence[int], sender: Sequence[int]) -> int | None:
    """The sender frame the decoded start lines up with; one bad frame must not hide the alignment of the rest."""
    best, best_hits = None, 0
    window = decoded[:ANCHOR_WINDOW]
    for anchor in range(len(sender)):
        position, hits = anchor, 0
        for pts in window:
            offset = pts - window[0]
            while position < len(sender) and sender[position] - sender[anchor] < offset - INTERVAL_TOLERANCE_NS:
                position += 1
            if position < len(sender) and abs(sender[position] - sender[anchor] - offset) <= INTERVAL_TOLERANCE_NS:
                hits += 1
                position += 1
        if hits > best_hits:
            best, best_hits = anchor, hits
        if hits == len(window):
            break
    return best if best_hits * 2 > len(window) else None


def score_intervals(decoded_pts: Sequence[int], sender_pts: Sequence[int]) -> IntervalScore:
    """Compares PTS by delta from a common frame: the two sides have different zero points."""
    decoded = list(decoded_pts)
    sender = sorted(sender_pts)
    repeated = len(decoded) - len(set(decoded))
    if not decoded:
        return IntervalScore(0, 0, 0, 0, 0, 0)
    anchor = _find_anchor(decoded, sender)
    if anchor is None:
        return IntervalScore(len(decoded), 0, len(decoded), 0, repeated, 0)
    position, last_matched, unmatched, used, worst = anchor, anchor, 0, 0, 0
    for pts in decoded:
        offset = pts - decoded[0]
        while position < len(sender) and sender[position] - sender[anchor] < offset - INTERVAL_TOLERANCE_NS:
            position += 1
        error = None if position == len(sender) else abs(sender[position] - sender[anchor] - offset)
        if error is None or error > INTERVAL_TOLERANCE_NS:
            unmatched += 1
            continue
        worst = max(worst, error)
        used += 1
        last_matched = position
        position += 1       # one sender frame matches one decoded frame
    in_window = last_matched - anchor + 1
    # A repeated PTS uses a sender frame twice, so it cannot hide a missing one: both counts fail.
    return IntervalScore(len(decoded), in_window, unmatched, max(0, in_window - used), repeated, worst)


def score_los(rows: Iterable[tuple[int, float]]) -> dict[str, int]:
    """Measurements delivered, and how many repeated a picture or a measurement time."""
    rows = list(rows)
    return {
        "los_updates": len(rows),
        "duplicate_pictures": len(rows) - len({picture for picture, _ in rows}),
        "duplicate_times": len(rows) - len({at for _, at in rows}),
    }


def read_sender_pts(publisher_run: Path) -> list[int]:
    pts: list[int] = []
    for path in sorted(publisher_run.glob("encoded-*.csv")):
        with path.open(newline="", encoding="utf-8") as file:
            # The publisher may still be writing: a row without its last column is not finished.
            pts.extend(int(row["pts_ns"]) for row in csv.DictReader(file) if row.get("vcl_sha256"))
    return pts


def rescore(out: Path, publisher_run: Path) -> dict[str, Any]:
    """Scores the intervals again once the publisher has ended: its last rows reach the disk only then."""
    result = json.loads((out / "result.json").read_text(encoding="utf-8"))
    with (out / "frames.csv").open(newline="", encoding="utf-8") as file:
        decoded = [int(row["pts_ns"]) for row in csv.DictReader(file)]
    intervals = score_intervals(decoded, read_sender_pts(publisher_run))
    result["intervals"] = asdict(intervals)
    result["checks"]["intervals_kept"] = intervals.ok
    result["pass"] = all(result["checks"].values())
    return result
