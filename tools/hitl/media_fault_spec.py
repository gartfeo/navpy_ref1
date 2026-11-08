"""The commissioning fault of one HITL run (hitl_guidance.md section 9, stage 1 step 15).

The runner's explicit ``--commission-fault`` flag writes it into the run record's ``commissioning`` section; the
injector (media_fault_injector.py) reads it back from there, so the record names the fault the run had. Spellings:

- ``none``: no injector (every ordinary run);
- ``delay:N[@K]``: after K decoded frames pass through, each frame is published N frames late;
- ``loss[@K]``: after K decoded frames pass through, no frame is published again (total loss).

Only SITL modes may commission (COMMISSIONING_MODES), and a run with a fault is never admitted to a ten-run series,
a fit, a held-out timing check or Qualification (admission_problems).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Mapping

from tools.hitl import json_checks as jc

NONE, DELAY, LOSS = "none", "delay", "loss"
KINDS = (NONE, DELAY, LOSS)
# The SITL run modes (tools/hitl/approach_records.MODE_SUBSTITUTIONS) a fault may be injected in. Flight and
# Qualification are never in it; later SITL stages add their mode here deliberately.
COMMISSIONING_MODES = frozenset({"pc-sitl"})
# My values, not the plan's. A delay holds at most this many frames (the plan's fault is 1 frame) ...
MAX_DELAY_FRAMES = 8
# ... and a held frame older than this on its capture stamp is discarded, never published late: the same 0.5 s as
# integration timing's max_frame_age_s, past which navpy refuses the frame anyway.
MAX_HOLD_S = 0.5

_SPELLING = re.compile(r"^(?:(none)|(delay):([0-9]+)|(loss))(?:@([0-9]+))?$")


@dataclass(frozen=True)
class FaultSpec:
    kind: str = NONE
    delay_frames: int = 0
    after_frames: int = 0
    max_hold_s: float = MAX_HOLD_S

    @property
    def active(self) -> bool:
        return self.kind != NONE

    def problems(self) -> list[str]:
        if self.kind not in KINDS:
            return [f"fault kind {self.kind!r} is not one of {', '.join(KINDS)}"]
        found = []
        for name in ("delay_frames", "after_frames"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                found.append(f"{name} must be a whole number >= 0")
        hold = self.max_hold_s
        if isinstance(hold, bool) or not isinstance(hold, (int, float)) or not math.isfinite(hold) or hold <= 0:
            found.append("max_hold_s must be a positive finite number")
        elif hold > MAX_HOLD_S:
            found.append(f"max_hold_s {hold} is over the bound {MAX_HOLD_S}")
        if found:
            return found
        if self.kind == DELAY and not 1 <= self.delay_frames <= MAX_DELAY_FRAMES:
            found.append(f"a delay holds 1 to {MAX_DELAY_FRAMES} frames, not {self.delay_frames}")
        if self.kind != DELAY and self.delay_frames:
            found.append(f"fault {self.kind} has no delay_frames")
        if self.kind == NONE and self.after_frames:
            found.append("no fault has no after_frames")
        return found

    def record(self) -> dict[str, Any]:
        return {"fault": self.kind, "delay_frames": self.delay_frames, "after_frames": self.after_frames,
                "max_hold_s": self.max_hold_s}

    def describe(self) -> str:
        if self.kind == DELAY:
            return f"delay {self.delay_frames} frame(s) after {self.after_frames}"
        if self.kind == LOSS:
            return f"total loss after {self.after_frames} frame(s)"
        return "none"


NO_FAULT = FaultSpec()


def parse(text: str) -> FaultSpec:
    """A spelling (module docstring) as a checked spec; ValueError names what is wrong."""
    match = _SPELLING.match(text.strip())
    if match is None:
        raise ValueError(f"commissioning fault {text!r} is not none, delay:N[@K] or loss[@K]")
    none, delay, frames, loss, after = match.groups()
    spec = FaultSpec(kind=none or delay or loss, delay_frames=int(frames or 0), after_frames=int(after or 0))
    problems = spec.problems()
    if problems:
        raise ValueError(f"commissioning fault {text!r}: " + "; ".join(problems))
    return spec


def from_record(section: Mapping[str, Any]) -> FaultSpec:
    return FaultSpec(kind=section["fault"], delay_frames=section["delay_frames"],
                     after_frames=section["after_frames"], max_hold_s=section["max_hold_s"])


# The run record's "commissioning" section (approach_records.RECORD_SPEC).
RECORD_SPEC: Mapping[str, jc.Check] = {"fault": jc.choice(KINDS), "delay_frames": jc.index, "after_frames": jc.index,
                                       "max_hold_s": jc.positive}


def mode_problems(spec: FaultSpec, mode: str) -> list[str]:
    """An active fault outside a SITL commissioning mode."""
    if spec.active and mode not in COMMISSIONING_MODES:
        return [f"commissioning fault {spec.describe()} in mode {mode}: only {', '.join(sorted(COMMISSIONING_MODES))}"]
    return []


def record_problems(record: Mapping[str, Any]) -> list[str]:
    """A run record's malformed fault, or one outside a SITL commissioning mode."""
    spec = from_record(record["commissioning"])
    problems = [*spec.problems(), *mode_problems(spec, record["run"]["mode"])]
    return [f"record.commissioning: {problem}" for problem in problems]


def admission_problems(record: Mapping[str, Any]) -> list[str]:
    """Why a valid record's run cannot count toward a ten-run series, a timing fit or check, or Qualification: an
    injected fault."""
    spec = from_record(record["commissioning"])
    if spec.active:
        return [f"commissioning run (injected fault: {spec.describe()}): never admitted to a ten-run series, "
                "a timing fit or check, or Qualification"]
    return []


__all__ = ["COMMISSIONING_MODES", "DELAY", "FaultSpec", "KINDS", "LOSS", "MAX_DELAY_FRAMES", "MAX_HOLD_S", "NONE",
           "NO_FAULT", "RECORD_SPEC", "admission_problems", "from_record", "mode_problems", "parse", "record_problems"]
