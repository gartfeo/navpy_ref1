"""The logs step 12 joins (hitl_final_approach.md section 9, stage 1 step 12), read into plain records.

- navpy's integration log (JSONL from step 11's ``TimingLog``): ``association`` records and the receiver-local
  ``attitude_receipt`` / ``gimbal_receipt`` histories (integration_association.py), all on the receiver's
  ``rt.perf_counter``.
- The aas-hitl publisher's frame logs (``frames-NNN.csv``): every pushed frame's index, board time and PTS.
- Unreal's ``NavPyCaptureSlot`` lines (tools/unreal/cadence_log.py): the board time and gimbal bracket each sent
  frame was rendered from.
- The ZR10 emulator's JSONL (tools/hitl/zr10_emulator.py): its ``step`` records (the rendered true states) and
  ``siyi_out`` records (each SIYI frame's wire sequence and the plant time a 0x0D frame reports).

A record that does not parse is an error, not a skipped row: a check that drops what it cannot read could pass on
half a log.
"""
from __future__ import annotations

import csv
import json
import math
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path

from navpy.core.perception.pipeline.integration_log import (
    EVENT_ASSOCIATION,
    EVENT_ATTITUDE_RECEIPT,
    EVENT_GIMBAL_RECEIPT,
)
from navpy.plugins.gimbal.siyi.siyi_message import COMMAND
from tools.unreal.cadence_log import parse_log

PUBLISHER_PUSHED = "pushed"       # aas-hitl publisher_core PUSH outcome
UNREAL_SENT = "sent"              # NavPyCaptureLedger outcome of a frame that went out


class TimingInputError(ValueError):
    """A log that cannot be read as the format it claims."""


def _number(record: dict, name: str, where: str) -> float:
    value = record.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TimingInputError(f"{where}: {name}={value!r} is not a finite number")
    return float(value)


def _reject_constant(name: str) -> float:
    raise TimingInputError(f"non-finite number {name} in JSON")


def json_record(line: str, where: str) -> object:
    """One JSON value; NaN and Infinity are refused, since a NaN skew or bound makes every comparison false."""
    try:
        return json.loads(line, parse_constant=_reject_constant)
    except json.JSONDecodeError as exc:
        raise TimingInputError(f"{where}: not JSON ({exc})") from exc
    except TimingInputError as exc:
        raise TimingInputError(f"{where}: {exc}") from exc


def _integer(record: dict, name: str, where: str) -> int:
    value = record.get(name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise TimingInputError(f"{where}: {name}={value!r} is not an integer")
    return value


# Association fields the check reads: each a finite number or null.
_ASSOCIATION_NUMBERS = ("read_s", "mapped_s", "attitude_lookup_s", "attitude_time_boot_s", "attitude_weight",
                        "gimbal_lookup_s", "gimbal_weight")
_ASSOCIATION_LIMITS = ("now_s", "attitude_bracket_max_s", "seam_tolerance_s")     # required finite numbers


def _check_association(record: dict, at: str) -> None:
    if not isinstance(record.get("status"), str):
        raise TimingInputError(f"{at}: association without a status")
    for name in _ASSOCIATION_LIMITS:
        _number(record, name, at)
    for name in _ASSOCIATION_NUMBERS:
        if record.get(name) is not None:
            _number(record, name, at)
    for name in ("frame_sequence", "pts_ns"):
        if record.get(name) is not None:
            _integer(record, name, at)
    receipts = record.get("gimbal_receipts_s")
    if receipts is not None:
        if not isinstance(receipts, list) or len(receipts) != 2:
            raise TimingInputError(f"{at}: gimbal_receipts_s is not a pair")
        for index in range(2):
            _number({"value": receipts[index]}, "value", f"{at} gimbal_receipts_s[{index}]")
    snapshot = record.get("gimbal_history")
    if snapshot is not None:
        if not isinstance(snapshot, dict):
            raise TimingInputError(f"{at}: gimbal_history is not an object")
        _integer(snapshot, "count", f"{at} gimbal_history")
        if snapshot["count"]:
            _integer(snapshot, "session", f"{at} gimbal_history")
            for name in ("first_receipt_s", "last_receipt_s"):
                _number(snapshot, name, f"{at} gimbal_history")


@dataclass(frozen=True)
class AttitudeReceipt:
    time_boot_s: float | None     # None: a sample without board time, which breaks adjacency (a seam)
    receipt_s: float


@dataclass(frozen=True)
class GimbalReceipt:
    session: int
    seq: int                      # 16-bit SIYI wire sequence
    receipt_s: float
    order: int                    # position in the navpy log: what an association written later could have seen


@dataclass(frozen=True)
class Association:
    record: dict
    order: int                    # position in the navpy log

    @property
    def status(self) -> str:
        return str(self.record.get("status"))

    def get(self, name: str) -> object:
        return self.record.get(name)


@dataclass
class NavpyLog:
    associations: list[Association] = field(default_factory=list)
    attitude: list[AttitudeReceipt] = field(default_factory=list)
    gimbal: list[GimbalReceipt] = field(default_factory=list)
    run_ids: set[str] = field(default_factory=set)


def read_navpy_log(lines: Iterable[str], where: str = "navpy log") -> NavpyLog:
    log = NavpyLog()
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        at = f"{where}:{number}"
        record = json_record(line, at)
        if not isinstance(record, dict):
            raise TimingInputError(f"{at}: not a JSON object")
        run_id = record.get("run_id")
        if not isinstance(run_id, str) or not run_id:
            raise TimingInputError(f"{at}: no run_id")
        log.run_ids.add(run_id)
        event = record.get("event")
        if event == EVENT_ASSOCIATION:
            _check_association(record, at)
            log.associations.append(Association(record, number))
        elif event == EVENT_ATTITUDE_RECEIPT:
            boot = None if record.get("time_boot_s") is None else _number(record, "time_boot_s", at)
            log.attitude.append(AttitudeReceipt(boot, _number(record, "capture_receipt_s", at)))
        elif event == EVENT_GIMBAL_RECEIPT:
            log.gimbal.append(GimbalReceipt(_integer(record, "session", at), _integer(record, "seq", at),
                                            _number(record, "receipt_s", at), number))
        else:
            raise TimingInputError(f"{at}: unknown event {event!r}")
    return log


@dataclass(frozen=True)
class PublisherFrame:
    frame_index: int
    board_time_us: int
    pts_ns: int


def read_publisher_frames(paths: Iterable[Path]) -> list[PublisherFrame]:
    """The pushed frames of one publisher run, in file and row order."""
    frames: list[PublisherFrame] = []
    for path in paths:
        with open(path, newline="", encoding="utf-8") as file:
            for number, row in enumerate(csv.DictReader(file), 2):
                if row.get("outcome") != PUBLISHER_PUSHED:
                    continue
                try:
                    frames.append(PublisherFrame(int(row["frame_index"]), int(row["board_time_us"]),
                                                 int(row["pts_ns"])))
                except (KeyError, TypeError, ValueError) as exc:
                    raise TimingInputError(f"{path}:{number}: pushed row unreadable ({exc})") from exc
    return frames


@dataclass(frozen=True)
class RenderedSlot:
    sys_id: int
    run: str
    gen: int
    frame: int
    board_us: int
    gimbal_session: str
    gimbal_older: int
    gimbal_newer: int
    gimbal_weight: float
    gimbal_valid: bool
    pose_valid: bool
    clock_mapped: bool


def read_unreal_slots(lines: Iterable[str], where: str = "Unreal log") -> list[RenderedSlot]:
    """The sent frames' render stamps (outcome=sent NavPyCaptureSlot lines)."""
    slots: list[RenderedSlot] = []
    for fields in parse_log(lines).slots:
        if fields.get("outcome") != UNREAL_SENT:
            continue
        try:
            older, newer = (int(value) for value in fields["gimbal_seq"].split(","))
            slots.append(RenderedSlot(int(fields["sys_id"]), fields["run"], int(fields["gen"]),
                                      int(fields["frame"]), int(fields["board_us"]), fields["gimbal_session"],
                                      older, newer, float(fields["gimbal_w"]), fields["gimbal_valid"] == "1",
                                      fields["pose_valid"] == "1", fields["mapped"] == "1"))
        except (KeyError, ValueError) as exc:
            raise TimingInputError(f"{where}: sent slot unreadable ({exc}): {fields}") from exc
    return slots


@dataclass(frozen=True)
class SiyiOut:
    seq: int
    cmd: str
    dest: tuple[str, int]
    sent: bool
    report_t: float | None        # 0x0D: the represented plant time, device clock


@dataclass
class EmulatorLog:
    session: str = ""
    step_device_t: dict[int, float] = field(default_factory=dict)    # gimbal_state sequence -> device time
    siyi_out: list[SiyiOut] = field(default_factory=list)            # in send order


def read_emulator_log(lines: Iterable[str], where: str = "emulator log") -> EmulatorLog:
    log = EmulatorLog()
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        at = f"{where}:{number}"
        record = json_record(line, at)
        if not isinstance(record, dict):
            raise TimingInputError(f"{at}: not a JSON object")
        event = record.get("event")
        if event == "start":
            if log.session:
                raise TimingInputError(f"{at}: a second emulator session in one log")
            log.session = str(record.get("session_id", ""))
        elif event == "step":
            seq = _integer(record, "seq", at)
            if seq in log.step_device_t:
                raise TimingInputError(f"{at}: step sequence {seq} twice")
            log.step_device_t[seq] = _number(record, "device_t", at)
        elif event == "siyi_out":
            cmd = record.get("cmd")
            dest = record.get("dest")
            if not isinstance(cmd, str) or not isinstance(dest, list) or len(dest) != 2:
                raise TimingInputError(f"{at}: siyi_out without cmd/dest")
            report_t = None
            if cmd == COMMAND.ACQUIRE_GIMBAL_ATT:
                report_t = _number(record, "report_t", at)
            log.siyi_out.append(SiyiOut(_integer(record, "seq", at), cmd, (str(dest[0]), int(dest[1])),
                                        record.get("sent") is True, report_t))
    if not log.session:
        raise TimingInputError(f"{where}: no start record (no session)")
    return log


__all__ = [
    "Association",
    "AttitudeReceipt",
    "EmulatorLog",
    "GimbalReceipt",
    "NavpyLog",
    "PublisherFrame",
    "RenderedSlot",
    "SiyiOut",
    "TimingInputError",
    "json_record",
    "read_emulator_log",
    "read_navpy_log",
    "read_publisher_frames",
    "read_unreal_slots",
]
