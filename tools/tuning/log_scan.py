"""List flight logs in a folder whose vehicle SYSID is in a range, with their flight time.

DataFlash logs (.bin) give the SYSID from the SYSID_THISMAV parameter and the flight time
from STAT.isFlying, ArduPlane's own flying estimate. Mission Planner telemetry logs (.tlog)
give the SYSID from the autopilot's HEARTBEAT source system and only the armed time, since
they carry no STAT record. File names are not trusted: a log named after one airframe can
carry another's SYSID.

    python -m tools.tuning.log_scan <log folder> --sysid 151-170 --min-minutes 10
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from pymavlink import mavutil

BIN_SUFFIXES = {".bin"}
TLOG_SUFFIXES = {".tlog"}
MAX_SAMPLE_GAP_S = 5.0   # a longer gap between samples (log stopped or dropped) is not counted
GCS_AUTOPILOT = mavutil.mavlink.MAV_AUTOPILOT_INVALID


@dataclass
class LogInfo:
    path: Path
    kind: str                 # "bin" or "tlog"
    sysid: int | None
    minutes: float            # flying (bin) or armed (tlog) time
    time_basis: str           # "flying" or "armed"
    span_minutes: float       # first to last time stamp
    error: str = ""


class OnTimer:
    """Adds up the time a sampled flag is on, skipping gaps longer than MAX_SAMPLE_GAP_S."""

    def __init__(self) -> None:
        self.total_s = 0.0
        self._last: tuple[float, bool] | None = None

    def sample(self, t_s: float, on: bool) -> None:
        if self._last is not None and self._last[1] and 0 < t_s - self._last[0] <= MAX_SAMPLE_GAP_S:
            self.total_s += t_s - self._last[0]
        self._last = (t_s, on)


def parse_range(text: str) -> tuple[int, int]:
    """'157' -> (157, 157); '151-170' -> (151, 170); '161-' -> (161, 255)."""
    lo, sep, hi = text.partition("-")
    low = int(lo) if lo else 0
    high = (int(hi) if hi else 255) if sep else low
    if low > high:
        raise argparse.ArgumentTypeError(f"empty SYSID range {text!r}")
    return low, high


def scan_bin(path: Path, sysids: tuple[int, int]) -> LogInfo:
    conn = mavutil.mavlink_connection(str(path), robust_parsing=True)
    sysid = None
    flying = OnTimer()
    first = last = None
    while (msg := conn.recv_match(type=["PARM", "STAT"])) is not None:
        t = msg.TimeUS / 1e6
        first = t if first is None else first
        last = t
        if msg.get_type() == "PARM":
            if msg.Name == "SYSID_THISMAV":
                sysid = int(msg.Value)
                if not sysids[0] <= sysid <= sysids[1]:
                    # outside the range: the rest of the file is not read, so no times
                    return LogInfo(path, "bin", sysid, float("nan"), "skip", float("nan"))
        else:
            flying.sample(t, bool(msg.isFlying))
    span = (last - first) if first is not None else 0.0
    return LogInfo(path, "bin", sysid, flying.total_s / 60, "flying", span / 60)


def scan_tlog(path: Path) -> LogInfo:
    conn = mavutil.mavlink_connection(str(path), robust_parsing=True)
    sysid = None
    armed = OnTimer()
    first = last = None
    while (msg := conn.recv_match(type="HEARTBEAT")) is not None:
        if msg.autopilot == GCS_AUTOPILOT:
            continue              # the ground station's own heartbeat
        src = msg.get_srcSystem()
        if sysid is None:
            sysid = src
        if src != sysid:
            continue
        t = msg._timestamp
        first = t if first is None else first
        last = t
        armed.sample(t, bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
    span = (last - first) if first is not None else 0.0
    return LogInfo(path, "tlog", sysid, armed.total_s / 60, "armed", span / 60)


def scan_file(path: Path, sysids: tuple[int, int]) -> LogInfo:
    try:
        if path.suffix.lower() in TLOG_SUFFIXES:
            return scan_tlog(path)
        return scan_bin(path, sysids)
    except Exception as exc:  # a damaged file is reported, not fatal to the scan
        return LogInfo(path, path.suffix.lower().lstrip("."), None, 0.0, "", 0.0, str(exc))


def find_logs(folder: Path, recursive: bool, tlogs: bool) -> list[Path]:
    suffixes = BIN_SUFFIXES | (TLOG_SUFFIXES if tlogs else set())
    pattern = "**/*" if recursive else "*"
    return sorted(p for p in folder.glob(pattern) if p.is_file() and p.suffix.lower() in suffixes)


def select(infos: Iterable[LogInfo], sysids: tuple[int, int], min_minutes: float) -> list[LogInfo]:
    return [i for i in infos if i.sysid is not None and sysids[0] <= i.sysid <= sysids[1]
            and i.minutes >= min_minutes]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", type=Path)
    ap.add_argument("--sysid", type=parse_range, default=(0, 255),
                    help="SYSID or range: 157, 151-170, 161- (default: all)")
    ap.add_argument("--min-minutes", type=float, default=0.0, help="minimum flight time in minutes")
    ap.add_argument("--no-recursive", action="store_true", help="do not look in subfolders")
    ap.add_argument("--tlogs", action="store_true", help="also read Mission Planner .tlog files (armed time)")
    ap.add_argument("--csv", type=Path, help="also write the selected logs to this CSV file")
    ap.add_argument("--all", action="store_true", help="also list the logs that did not match, and errors")
    args = ap.parse_args(argv)

    if not args.folder.is_dir():
        ap.error(f"not a folder: {args.folder}")
    paths = find_logs(args.folder, not args.no_recursive, args.tlogs)
    infos = []
    for n, p in enumerate(paths, 1):
        print(f"[{n}/{len(paths)}] {p.name}", file=sys.stderr, flush=True)
        infos.append(scan_file(p, args.sysid))
    chosen = select(infos, args.sysid, args.min_minutes)

    rows = infos if args.all else chosen
    print(f"{'SYSID':>5}  {'min':>6}  {'basis':<6}  {'span':>6}  file")
    for i in sorted(rows, key=lambda i: (i.sysid if i.sysid is not None else -1, str(i.path))):
        mark = "" if i in chosen else "   (not selected" + (f": {i.error})" if i.error else ")")
        sysid = "?" if i.sysid is None else str(i.sysid)
        print(f"{sysid:>5}  {i.minutes:6.1f}  {i.time_basis:<6}  {i.span_minutes:6.1f}  "
              f"{i.path.relative_to(args.folder)}{mark}")
    print(f"\n{len(chosen)} of {len(infos)} logs: SYSID {args.sysid[0]}-{args.sysid[1]}, "
          f">= {args.min_minutes:g} min")

    if args.csv:
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["sysid", "minutes", "time_basis", "span_minutes", "kind", "path"])
            for i in chosen:
                w.writerow([i.sysid, f"{i.minutes:.2f}", i.time_basis, f"{i.span_minutes:.2f}", i.kind, i.path])
    return 0


if __name__ == "__main__":
    sys.exit(main())
