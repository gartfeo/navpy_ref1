"""DataFlash log download over MAVLink and the flight tuner's timing summary.

ArduPilot answers LOG_REQUEST_* only while disarmed (AP_Logger_MAVLinkLogTransfer.cpp:40-47).
It serves one LOG_REQUEST_DATA range at a time, 90 bytes per LOG_DATA, and a packet with
fewer bytes than asked ends the transfer (:330). List entries count from the oldest log on
the card (AP_Logger_Backend.cpp:626-639), so they shift down when old logs are deleted.
The HITL link-health gate over the downloaded logs is hitl_gate.py.
"""
from __future__ import annotations

from pathlib import Path

from tools.tuning.logdata import FlightLog, read_log
from tools.tuning.vehicle import Vehicle

LOG_LIST_TIMEOUT_S = 10.0      # between LOG_ENTRY replies of one listing
LOG_LIST_ATTEMPTS = 3          # listings asked for before giving up
LOG_DATA_STALL_S = 3.0         # no LOG_DATA for this long ends a request; the gap is asked for again
LOG_MAX_STALLS = 20            # requests in a row that bring no new byte before the download fails
LOG_SIZE_CHECKS = 2            # listings after the read that may still report a larger size
LOG_CHUNK = 90                 # MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN
ALL_ENTRIES = 0xFFFF
FLIGHT_TUNE_COMPUTE_US = 600   # budget per compute call (ArduPlane/flight_tune.cpp:9)
FLIGHT_TUNE_SLOT_US = 700      # scheduler slot of flight_tune_compute (ArduPlane/ArduPlane.cpp:146)
TIMING_TYPES = {"FTPR": ("TimeUS", "PcUs", "CaUs"), "FTD2": ("TimeUS", "PcUs", "CaUs"),
                "PM": ("TimeUS", "NLon", "NL", "MaxT")}


def list_logs(vehicle: Vehicle) -> dict[int, tuple[int, int]]:
    """List entry -> (size in bytes, time_utc) for every log on the vehicle."""
    conn = vehicle.mav
    entries: dict[int, tuple[int, int]] = {}
    for _ in range(LOG_LIST_ATTEMPTS):
        conn.mav.log_request_list_send(conn.target_system, conn.target_component, 0, ALL_ENTRIES)
        num = None
        while num is None or len(entries) < num:
            m = vehicle.wait_message("LOG_ENTRY", LOG_LIST_TIMEOUT_S)
            if m is None:
                break
            if m.num_logs == 0:
                return {}
            num = m.num_logs
            entries[m.id] = (m.size, m.time_utc)
        if num is not None and len(entries) >= num:
            return entries
    raise RuntimeError(f"log listing incomplete after {LOG_LIST_ATTEMPTS} attempts: {sorted(entries)}")


def new_log_entries(before: dict[int, tuple[int, int]], after: dict[int, tuple[int, int]]) -> list[int]:
    """Entries of the logs written after the listing `before`, found behind its newest log
    (same size and time) because entry numbers shift when old logs are deleted."""
    if not before:
        return sorted(after)
    anchor = before[max(before)]
    matches = [i for i, info in after.items() if info == anchor]
    if not matches:
        raise RuntimeError(f"the newest log before the run ({anchor}) is no longer listed; copy the log by hand")
    return sorted(i for i in after if i > max(matches))


class LogBuffer:
    """Bytes received for one log and the merged ranges [start, end) they cover."""

    def __init__(self) -> None:
        self.data = bytearray()
        self.ranges: list[list[int]] = []

    def covered(self) -> int:
        return sum(end - start for start, end in self.ranges)

    def add(self, ofs: int, chunk: bytes) -> bool:
        """Store a chunk; True if it covered bytes not received before."""
        end = ofs + len(chunk)
        if end > len(self.data):
            self.data.extend(bytes(end - len(self.data)))
        self.data[ofs:end] = chunk
        before = self.covered()
        last = self.ranges[-1] if self.ranges else None
        if last is not None and last[0] <= ofs <= last[1]:       # in order: extend the last range
            last[1] = max(last[1], end)
        else:
            merged: list[list[int]] = []
            for start, stop in sorted(self.ranges + [[ofs, end]]):
                if merged and start <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], stop)
                else:
                    merged.append([start, stop])
            self.ranges = merged
        return self.covered() > before

    def missing(self, size: int) -> list[tuple[int, int]]:
        gaps, pos = [], 0
        for start, end in self.ranges:
            if start > pos:
                gaps.append((pos, min(start, size)))
            pos = max(pos, end)
        if pos < size:
            gaps.append((pos, size))
        return [(s, e) for s, e in gaps if s < e]


def _receive(vehicle: Vehicle, entry: int, buf: LogBuffer, end: int) -> tuple[bool, int | None]:
    """LOG_DATA of one request until it reaches `end` or stalls: (new bytes?, end of file if reported)."""
    new = False
    while True:
        m = vehicle.wait_message("LOG_DATA", LOG_DATA_STALL_S, lambda d: d.id == entry)
        if m is None:
            return new, None
        if m.count:
            new = buf.add(m.ofs, bytes(m.data[:m.count])) or new
        if m.count < LOG_CHUNK and m.ofs + m.count < end:      # short before the asked end: file ends here
            return new, m.ofs + m.count
        if m.ofs + m.count >= end:
            return new, None


def fetch_log(vehicle: Vehicle, entry: int, size: int) -> bytes:
    """One log's bytes. Gaps are asked for again until a listing taken after the read agrees on
    the size: reading closes a log still being written (AP_Logger_File.cpp:610), which can grow it."""
    conn = vehicle.mav
    buf, stalls, checks = LogBuffer(), 0, 0
    while True:
        gaps = buf.missing(size)
        if not gaps:
            listed = list_logs(vehicle).get(entry, (size, 0))[0]
            if listed <= size:
                return bytes(buf.data[:size])
            checks += 1
            if checks > LOG_SIZE_CHECKS:
                raise RuntimeError(f"log {entry} keeps growing ({listed} bytes); is the vehicle disarmed?")
            size = listed
            continue
        start, end = gaps[0]
        conn.mav.log_request_data_send(conn.target_system, conn.target_component, entry, start, end - start)
        new, eof = _receive(vehicle, entry, buf, end)
        if eof is not None:
            size = eof
        stalls = 0 if new else stalls + 1
        if stalls >= LOG_MAX_STALLS:
            raise RuntimeError(f"log {entry}: no data for {stalls} requests at offset {start} of {size}")


def download_logs(vehicle: Vehicle, before: dict[int, tuple[int, int]], dest: Path) -> list[Path]:
    """Download the logs written since the listing `before` into dest/<entry:08d>.BIN."""
    conn = vehicle.mav
    paths = []
    try:
        after = list_logs(vehicle)
        dest.mkdir(parents=True, exist_ok=True)
        for entry in new_log_entries(before, after):
            path = dest / f"{entry:08d}.BIN"
            path.write_bytes(fetch_log(vehicle, entry, after[entry][0]))
            paths.append(path)
    finally:
        conn.mav.log_request_end_send(conn.target_system, conn.target_component)   # logging resumes
    return paths


def timing_from_log(log: FlightLog) -> dict:
    """Largest tuner job (PcUs) and compute call (CaUs) times from FTPR/FTD2 against the budget
    and the slot, and the scheduler's long-loop counts and longest loop (PM, microseconds)."""
    out: dict = {"budget_us": FLIGHT_TUNE_COMPUTE_US, "slot_us": FLIGHT_TUNE_SLOT_US}
    for mtype in ("FTPR", "FTD2"):
        if not log.has(mtype):
            out[mtype] = {"records": 0}
            continue
        call = float(log.get(mtype, "CaUs").max())
        out[mtype] = {"records": len(log.get(mtype, "TimeUS")), "max_job_us": float(log.get(mtype, "PcUs").max()),
                      "max_call_us": call, "call_over_budget": call > FLIGHT_TUNE_COMPUTE_US,
                      "call_over_slot": call > FLIGHT_TUNE_SLOT_US}
    if log.has("PM"):
        out["PM"] = {"records": len(log.get("PM", "TimeUS")), "long_loops": int(log.get("PM", "NLon").sum()),
                     "loops": int(log.get("PM", "NL").sum()), "max_loop_us": float(log.get("PM", "MaxT").max())}
    else:
        out["PM"] = {"records": 0}
    return out


def timing_summary(path: Path) -> dict:
    return timing_from_log(read_log(str(path), TIMING_TYPES))
