"""Task-assignment lines of the independent GCS backend log, in either format.

Legacy logs (recorded before docs/design/swarm-task-assignment-ack.md) hold
one request and one response line per assignment. Acked logs end every
request and response line with ``uid=B:S``, one line per repeated copy, and
add a ``Task assign ack:`` line for each owner APPLIED of a helper's
accepted response (gcs/backend/task_assign_listener.py writes them).
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum
from typing import Optional


_NUMBER = r"-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
REQUEST = re.compile(
    rf"Task assign request: sender=(?P<sender>\d+) "
    rf"receiver=(?P<receiver>\d+) task_id=(?P<task>\d+) "
    rf"at \((?P<lat>{_NUMBER}),\s*(?P<lon>{_NUMBER})\)"
)
RESPONSE = re.compile(
    r"Task assign response: sender=(?P<sender>\d+) "
    r"receiver=(?P<receiver>\d+) task_id=(?P<task>\d+) "
    r"accepted=(?P<accepted>True|False)"
)
ACK = re.compile(
    r"Task assign ack: owner=(?P<owner>\d+) helper=(?P<helper>\d+) "
    r"task_id=(?P<task>\d+) status=APPLIED "
    r"ref=(?P<ref_boot>\d+):(?P<ref_seq>\d+) uid=(?P<boot>\d+):(?P<seq>\d+)"
)
_UID = re.compile(r" uid=(?P<boot>\d+):(?P<seq>\d+)\b")

# (boot_id, msg_seq) of a message within its known sender.
Uid = tuple[int, int]


class LogFormat(Enum):
    LEGACY = "legacy"
    ACKED = "acked"


@dataclass(frozen=True)
class RequestLine:
    sender: int
    receiver: int
    task: int
    lat: float
    lon: float
    uid: Optional[Uid]


@dataclass(frozen=True)
class ResponseLine:
    sender: int
    receiver: int
    task: int
    accepted: bool
    uid: Optional[Uid]


@dataclass(frozen=True)
class AckLine:
    owner: int
    helper: int
    task: int
    ref: Uid
    uid: Uid


@dataclass(frozen=True)
class AssignmentLines:
    log_format: LogFormat
    requests: tuple[RequestLine, ...]
    responses: tuple[ResponseLine, ...]
    acks: tuple[AckLine, ...]


def parse_assignment_lines(backend_log: str) -> AssignmentLines:
    """Parse every assignment line; a malformed one or mixed formats raise."""
    if type(backend_log) is not str:
        raise TypeError("GCS backend assignment evidence must be text")
    requests: list[RequestLine] = []
    responses: list[ResponseLine] = []
    acks: list[AckLine] = []
    for line in backend_log.splitlines():
        if "Task assign request:" in line:
            match, uid = _marker(REQUEST, line, "request")
            requests.append(RequestLine(
                int(match["sender"]), int(match["receiver"]), int(match["task"]),
                float(match["lat"]), float(match["lon"]), uid,
            ))
        elif "Task assign response:" in line:
            match, uid = _marker(RESPONSE, line, "response")
            responses.append(ResponseLine(
                int(match["sender"]), int(match["receiver"]), int(match["task"]),
                match["accepted"] == "True", uid,
            ))
        elif "Task assign ack:" in line:
            match = ACK.search(line)
            if match is None:
                raise ValueError(f"malformed task assignment ack marker: {line}")
            acks.append(AckLine(
                int(match["owner"]), int(match["helper"]), int(match["task"]),
                (int(match["ref_boot"]), int(match["ref_seq"])),
                (int(match["boot"]), int(match["seq"])),
            ))
    return AssignmentLines(
        _log_format(requests, responses, acks),
        tuple(requests), tuple(responses), tuple(acks),
    )


def _marker(
    pattern: re.Pattern[str],
    line: str,
    kind: str,
) -> tuple[re.Match[str], Optional[Uid]]:
    match = pattern.search(line)
    if match is None:
        raise ValueError(f"malformed task assignment {kind} marker: {line}")
    if " uid=" not in line:
        return match, None
    uid = _UID.match(line, match.end())
    if uid is None:
        raise ValueError(f"malformed task assignment {kind} uid: {line}")
    return match, (int(uid["boot"]), int(uid["seq"]))


def _log_format(
    requests: list[RequestLine],
    responses: list[ResponseLine],
    acks: list[AckLine],
) -> LogFormat:
    lines = [*requests, *responses]
    legacy = sum(line.uid is None for line in lines)
    acked = len(lines) - legacy + len(acks)
    if legacy and acked:
        raise ValueError(
            f"mixed assignment log formats: {legacy} lines without uid, "
            f"{acked} acked lines"
        )
    return LogFormat.ACKED if acked else LogFormat.LEGACY


__all__ = [
    "ACK",
    "REQUEST",
    "RESPONSE",
    "AckLine",
    "AssignmentLines",
    "LogFormat",
    "RequestLine",
    "ResponseLine",
    "Uid",
    "parse_assignment_lines",
]
