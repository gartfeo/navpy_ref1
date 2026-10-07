"""Structured event rows for navigation CSV logs.

Two row shapes:
  - SNAP, written to the compact CSV: byte-stable legacy special case. Column layout:
      {ts},SNAP({ALG}),{snap_str},{kp_str},,,,,,,
    The existing format is the source of truth and must not change.
  - Standard debug events, written to the navigation debug CSV (CONFIG,
    SNAP_COMPONENTS, LAW_SWITCH, GATE_FLIP, VALIDITY_FLIP, ABORT,
    FINAL_APPROACH_CMD, FINAL_APPROACH_PITCH_STATE, FINAL_APPROACH_PREDICTION,
    FINAL_APPROACH_ATTITUDE_DIAG, FINAL_APPROACH_RESPONSE_STATE,
    FINAL_APPROACH_BEARING_RATE_DIAG, FINAL_APPROACH_VERTICAL_FLOW_DIAG,
    FINAL_APPROACH_PREVIEW_STATE):
      {ts},EVENT:{NAME},{k=v;k=v},,,,,,,,,
    12-column CSV-parseable. Payload uses sanitized k=v pairs.
"""
from __future__ import annotations

import re
from enum import Enum
from typing import Iterable, Tuple


_PRIMARY_COLUMNS = 12
_UNSAFE = re.compile(r"[;=,\r\n]")
STANDARD_EVENT_HEADER = "ts,event,payload" + "," * (_PRIMARY_COLUMNS - 3)


def _sanitize(token: str) -> str:
    if not token:
        return ""
    return _UNSAFE.sub("_", token)


class LogEvent(Enum):
    CONFIG = "CONFIG"
    SNAP = "SNAP"
    SNAP_COMPONENTS = "SNAP_COMPONENTS"
    LAW_SWITCH = "LAW_SWITCH"
    GATE_FLIP = "GATE_FLIP"
    VALIDITY_FLIP = "VALIDITY_FLIP"
    ABORT = "ABORT"
    FINAL_APPROACH_CMD = "FINAL_APPROACH_CMD"
    FINAL_APPROACH_PITCH_STATE = "FINAL_APPROACH_PITCH_STATE"
    FINAL_APPROACH_PREDICTION = "FINAL_APPROACH_PREDICTION"
    FINAL_APPROACH_ATTITUDE_DIAG = "FINAL_APPROACH_ATTITUDE_DIAG"
    FINAL_APPROACH_RESPONSE_STATE = "FINAL_APPROACH_RESPONSE_STATE"
    FINAL_APPROACH_BEARING_RATE_DIAG = "FINAL_APPROACH_BEARING_RATE_DIAG"
    FINAL_APPROACH_VERTICAL_FLOW_DIAG = "FINAL_APPROACH_VERTICAL_FLOW_DIAG"
    FINAL_APPROACH_PREVIEW_STATE = "FINAL_APPROACH_PREVIEW_STATE"


def format_snap_event_row(
    ts: str,
    algorithm: str | None,
    snap_str: str,
    kp_str: str,
) -> str:
    """Byte-identical to the legacy SNAP row format.

    Legacy source of truth (do not "normalize" this to 12 columns via
    the standard event template):
        {ts},SNAP({ALG}),{snap_str},{kp_str},,,,,,,\\n
    """
    alg = algorithm.upper() if algorithm else ""
    return f"{ts},SNAP({alg}),{snap_str},{kp_str},,,,,,,\n"


def format_standard_event_row(
    ts: str,
    event: LogEvent,
    payload_pairs: Iterable[Tuple[str, str]],
) -> str:
    """Standardised event row for non-SNAP events.

    Column layout (12 total):
      0: ts
      1: EVENT:NAME
      2: k=v;k=v;... (sanitised)
      3..11: empty
    """
    if event == LogEvent.SNAP:
        raise ValueError(
            "format_standard_event_row does not handle SNAP; "
            "use format_snap_event_row instead."
        )
    safe_pairs = [
        f"{_sanitize(str(k))}={_sanitize(str(v))}" for k, v in payload_pairs
    ]
    body = ";".join(safe_pairs)
    populated = 3
    trailing_commas = "," * (_PRIMARY_COLUMNS - populated)
    return f"{ts},EVENT:{event.value},{body}{trailing_commas}\n"
