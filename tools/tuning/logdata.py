"""Read the DataFlash messages the tuning tools need into numpy arrays."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pymavlink import DFReader

PID_FIELDS = ("TimeUS", "Tar", "Act", "Err", "P", "I", "D", "FF", "DFF", "Dmod", "SRate", "Flags")
ARM_FIELDS = ("TimeUS", "ArmState")      # the ARM record: ArmState 1 armed, 0 disarmed


@dataclass
class FlightLog:
    path: str
    series: dict            # message type -> {field: np.ndarray}
    params: dict            # name -> last value in the log
    param_history: list     # (TimeUS, name, value) in log order

    def get(self, mtype: str, field: str) -> np.ndarray:
        return self.series[mtype][field]

    def has(self, mtype: str) -> bool:
        return mtype in self.series and len(self.series[mtype]["TimeUS"]) > 0

    def param_at(self, name: str, time_us: float, default: float | None = None) -> float | None:
        """Value of a parameter in force at time_us (last PARM record at or before it)."""
        value = default
        for t, n, v in self.param_history:
            if n != name:
                continue
            if t <= time_us or value is default:
                value = v
            if t > time_us:
                break
        return value


def read_log(path: str, types: dict[str, tuple[str, ...]]) -> FlightLog:
    """types maps message type to the fields to keep. PARM is always read.

    Multi-instance messages (IMU, BAT, ...) keep all instances; callers filter on I.
    """
    reader = DFReader.DFReader_binary(path, zero_time_base=False)
    rows: dict[str, list] = {t: [] for t in types}
    params, history = {}, []
    wanted = list(types) + ["PARM"]
    while True:
        m = reader.recv_match(type=wanted)
        if m is None:
            break
        t = m.get_type()
        if t == "PARM":
            params[m.Name] = m.Value
            history.append((m.TimeUS, m.Name, m.Value))
            continue
        rows[t].append(tuple(getattr(m, f, np.nan) for f in types[t]))
    series = {}
    for t, fields in types.items():
        arr = np.array(rows[t], dtype=float) if rows[t] else np.zeros((0, len(fields)))
        series[t] = {f: arr[:, i] for i, f in enumerate(fields)}
    return FlightLog(path=path, series=series, params=params, param_history=history)


def armed_windows(log: FlightLog) -> list[tuple[float, float]]:
    """(arm, disarm) TimeUS of every armed stretch in the log's ARM records (read with ARM_FIELDS);
    a stretch still armed at the end of the log runs to +inf. No ARM records: no stretch."""
    rows = zip(log.get("ARM", "TimeUS"), log.get("ARM", "ArmState")) if log.has("ARM") else []
    windows, start = [], None
    for t, armed in rows:
        if armed and start is None:
            start = t
        elif not armed and start is not None:
            windows, start = windows + [(float(start), float(t))], None
    return windows + ([(float(start), np.inf)] if start is not None else [])


def standard_log(path: str) -> FlightLog:
    """PIDR/PIDP/ATT at full rate plus CTUN airspeed and the tuning script's own messages."""
    return read_log(path, {
        "PIDR": PID_FIELDS,
        "PIDP": PID_FIELDS,
        "ATT": ("TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch"),
        "CTUN": ("TimeUS", "As", "E2T", "ThO"),
        "STAT": ("TimeUS", "isFlying"),
        "MODE": ("TimeUS", "ModeNum"),
        "TFLS": ("TimeUS", "St", "Ax", "Seg", "Dem", "Ang", "Rate", "As", "Alt"),
    })
