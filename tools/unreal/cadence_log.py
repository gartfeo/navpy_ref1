"""Parsing for the capture-cadence check: the Unreal log lines of the camera and the receiver
(NavPyCaptureSlot, NavPyPoseAdmit, NavPyRenderClock, NavPyCameraMount, NavPyBoardPose) and the ZR10
emulator's step log."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Iterable, Optional

from tools.hitl.zr10_device import mode_name
from tools.unreal.unreal_math import Quat, Vec


# --- Unreal log ----------------------------------------------------------------------------------------------------
_KV = re.compile(r"(\w+)=(\S+)")


def _fields(text: str) -> dict[str, str]:
    return dict(_KV.findall(text))


def parse_vec(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split(","))


@dataclass(frozen=True)
class Mount:
    receiver_port: int
    pivot_cm: Vec
    camera_in_pitch_cm: Vec
    camera_in_pitch_quat: Quat
    stabilize_roll: bool
    min_pitch: float
    max_pitch: float
    fps: float


@dataclass(frozen=True)
class BoardPose:
    location: Vec
    quat: Quat
    width_cm: float
    height_cm: float


@dataclass(frozen=True)
class Admit:
    """One NavPyPoseAdmit line: a pose the receiver gave its interpolator, in admission order."""
    port: int
    gen: int
    admitted: bool
    board_us: int
    receipt_qpc: float
    location: Vec
    quat: Quat


@dataclass
class UnrealLog:
    slots: list[dict] = field(default_factory=list)
    admits: list[Admit] = field(default_factory=list)
    stream_ends: dict[tuple[int, int], list[int]] = field(default_factory=dict)    # (port, gen) -> next_slot
    generations: set[tuple[int, int]] = field(default_factory=set)                 # (port, gen) started
    mounts: dict[str, Mount] = field(default_factory=dict)                          # by camera run
    board: Optional[BoardPose] = None


def parse_log(lines: Iterable[str]) -> UnrealLog:
    log = UnrealLog()
    for line in lines:
        if (at := line.find("NavPyCaptureSlot ")) >= 0:
            log.slots.append(_fields(line[at:]))
        elif (at := line.find("NavPyPoseAdmit ")) >= 0:
            f = _fields(line[at:])
            log.admits.append(Admit(int(f["port"]), int(f["gen"]), f["admitted"] == "1", int(f["board_us"]),
                                    float(f["receipt_qpc"]), parse_vec(f["loc"]), parse_vec(f["quat"])))
        elif (at := line.find("NavPyRenderClock ")) >= 0:
            f = _fields(line[at:])
            if f.get("event") == "stream_end":
                log.stream_ends.setdefault((int(f["port"]), int(f["gen"])), []).append(int(f["next_slot"]))
            elif f.get("event") == "generation":
                log.generations.add((int(f["port"]), int(f["gen"])))
        elif (at := line.find("NavPyCameraMount ")) >= 0:
            f = _fields(line[at:])
            log.mounts[f["run"]] = Mount(int(f["receiver_port"]), parse_vec(f["pivot_cm"]), parse_vec(f["camera_in_pitch_cm"]),
                                         parse_vec(f["camera_in_pitch_quat"]), f["stabilize_roll"] == "1",
                                         float(f["min_pitch"]), float(f["max_pitch"]), float(f["fps"]))
        elif (at := line.find("NavPyBoardPose ")) >= 0:
            f = _fields(line[at:])
            log.board = BoardPose(parse_vec(f["loc"]), parse_vec(f["quat"]), float(f["width_cm"]), float(f["height_cm"]))
    return log


# --- emulator log --------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class GimbalStep:
    t: float                  # sender_time_s: scheduled perf_counter of the step
    seq: int
    yaw: float
    pitch: float
    roll: float
    roll_stabilized: bool
    zoom: float = 1.0


def parse_emulator_log(lines: Iterable[str]) -> list[GimbalStep]:
    steps = []
    for line in lines:
        record = json.loads(line)
        if record.get("event") == "step":
            true = record["true"]
            steps.append(GimbalStep(float(record["t"]), int(record["seq"]), float(true["yaw"]), float(true["pitch"]),
                                    float(true["roll"]), mode_name(int(true["motion_mode"])) != "fpv",
                                    float(true["zoom"])))
    return steps
