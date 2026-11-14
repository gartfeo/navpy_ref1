"""The publisher's video numbers of one approach run (hitl_guidance.md section 9, step 8's done check, repeated
by step 27's simple checks): every frame accounted for, a drained stream, bounded queues, the unique frame rate within
2 % of the profile's rate and the mean bitrate within 10 % of its target.

The rate and bitrate come from the publisher's own encoded log (``encoded-NNN.csv``: one row per access unit, its
zero-based PTS and its bytes), over the stream's PTS span; the profile (``publisher.json``) gives the targets.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Optional

UNIQUE_RATE_TOLERANCE = 0.02    # step 8: unique rate within 2 %
BITRATE_TOLERANCE = 0.10        # step 8: mean bitrate within 10 %


def _status(run_dir: Path) -> dict:
    return json.loads((Path(run_dir) / "publisher" / "publisher.json").read_text(encoding="utf-8"))


def encoded_stats(run_dir: Path) -> Optional[dict[str, Any]]:
    """Units, PTS span, unique rate (frames per second of PTS) and mean bitrate (kbit/s, each unit lasting one
    interval); None without two units."""
    rows = []
    for path in sorted((Path(run_dir) / "publisher").glob("encoded-*.csv")):
        with open(path, encoding="utf-8", newline="") as handle:
            rows += [(int(row["pts_ns"]), int(row["bytes"])) for row in csv.DictReader(handle)]
    if len(rows) < 2:
        return None
    pts = sorted(p for p, _ in rows)
    span_s = (pts[-1] - pts[0]) * 1e-9
    if span_s <= 0:
        return None
    rate = (len(rows) - 1) / span_s
    return {"units": len(rows), "span_s": span_s, "unique_fps": rate,
            "bitrate_kbps": sum(b for _, b in rows) * 8e-3 / (span_s + 1.0 / rate)}


def video_check(run: dict, run_dir: Path) -> list[str]:
    try:
        status = _status(run_dir)
        stats = encoded_stats(run_dir)
    except (OSError, ValueError, KeyError) as exc:
        return [f"publisher logs unreadable: {exc}"]
    publisher, bound = status.get("publisher") or {}, (status.get("mediamtx") or {}).get("write_queue_size")
    profile = status.get("profile") or {}
    found = [] if publisher.get("accounted") is True and publisher.get("failed") is False else [
        "the publisher did not account for every frame or failed"]
    if not (status.get("drain") or {}).get("ok"):
        found.append("the publisher did not drain")
    if not isinstance(bound, int) or not isinstance(publisher.get("max_queue_level"), int) or \
            publisher["max_queue_level"] > bound:
        found.append(f"queue level {publisher.get('max_queue_level')} over its bound {bound}")
    if (status.get("receiver") or {}).get("frames_dropped") != 0:
        found.append(f"the receiver dropped {(status.get('receiver') or {}).get('frames_dropped')} frames")
    fps, kbps = profile.get("fps"), profile.get("bitrate_kbps")
    if stats is None or not isinstance(fps, (int, float)) or fps <= 0:
        found.append("no encoded stream or profile rate to check")
    elif abs(stats["unique_fps"] / fps - 1.0) > UNIQUE_RATE_TOLERANCE:
        found.append(f"unique rate {stats['unique_fps']:.2f} fps, not within {UNIQUE_RATE_TOLERANCE:.0%} of {fps}")
    if stats is not None and (not isinstance(kbps, (int, float)) or kbps <= 0
                              or abs(stats["bitrate_kbps"] / kbps - 1.0) > BITRATE_TOLERANCE):
        found.append(f"mean bitrate {stats['bitrate_kbps']:.0f} kbit/s, not within {BITRATE_TOLERANCE:.0%} of {kbps}")
    return found


def video_numbers(run_dir: Path) -> dict:
    """The numbers reported beside the score (publisher, receiver and the encoded stream)."""
    try:
        status, stats = _status(run_dir), encoded_stats(run_dir)
    except (OSError, ValueError, KeyError):
        return {}
    publisher, receiver = status.get("publisher") or {}, status.get("receiver") or {}
    return {**{key: publisher.get(key) for key in ("frames_in", "pushed", "queue_full", "missed_slots",
                                                   "max_queue_level")},
            **{key: receiver.get(key) for key in ("frames_dropped", "source_gaps")}, "encoded": stats}


__all__ = ["BITRATE_TOLERANCE", "UNIQUE_RATE_TOLERANCE", "encoded_stats", "video_check", "video_numbers"]
