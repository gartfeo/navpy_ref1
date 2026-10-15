"""The slice's confirmation timing, from the lockstep journal (``lockstep_slice.md``).

Named events and where the journal holds them:

- e1, k1: NavPy logs ``CONFIRMING: P`` (its line's tick);
- e2, k2: the backend's ``task_confirm_request`` broadcast reaches the
  operator's event port (the delivery tick of the operator's ``received``);
- e3, k3: the operator's receive returns it (the tick of that ``received``);
- e4, k4: the operator submits the approval (its ``sent`` POST);
- e5, k5: the backend's first ``TASK_CONFIRM_RESPONSE`` into the owner SITL's
  serial1 (``navlink``, the router's copy admitted to a reply);
- e6, k6: the response completes after the resend schedule (the operator's
  ``answered``), at the last send's tick.

Relations: k1 ≤ k2 ≤ k3; the delay runs from consumption, and each of the
loop's two waits per pass overshoots its float deadline by at most Δ_max, so
``d ≤ t(k4) - t(k3) ≤ d + 2·poll + 2·Δ_max`` (15.00 to 15.10 + 2Δ_max for the
owner's 15 s); k5 = k4 + 1; the schedule's sends all fall in (k4, k6] with
the last at k6, and each resend waits its own tick-rounded gap after the
previous send (the route sleeps ``gap`` after every send but the last), so
every consecutive interval is ``gap ≤ t(s_{i+1}) - t(s_i) ≤ gap + Δ_max``; the
audited ``actual_delay_s`` is ``t(k6) - t(k3)``. A send is any
``TASK_CONFIRM_RESPONSE`` admitted to the owner SITL's serial1 (``to_sitl``
on the serial1 the journal's grid pairs with vehicle ``sys_id``'s serial0), whatever its
source (the assignment gate checks identities). Times are compared in
integer microseconds, so no float rounding moves a bound.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from gcs.backend.task_confirm_resend import (
    TASK_CONFIRM_RESPONSE_RESEND_GAP_S,
    TASK_CONFIRM_RESPONSE_SENDS,
)
from navpy.sim.lockstep.coordinator_mavlink import message_id
from navpy.sim.lockstep.coordinator_types import serial0_device
from scripts.eval_gcs_demo_confirmation import CONFIRMATION_POLL_S

CONFIRM_REQUEST_EVENT = "task_confirm_request"
APPROVAL_TARGET = "/api/control/task_confirm"
RESPONSE_DIRECTION = "to_sitl"
RESPONSE_ID = mavlink.MAVLINK_MSG_ID_TASK_CONFIRM_RESPONSE
#: The audited delay is float seconds from the baton clock; the journal's
#: times are integer microseconds.
ACTUAL_DELAY_TOLERANCE_S = 1e-6


def _us(seconds: float) -> int:
    return round(seconds * 1_000_000)


@dataclass
class ConfirmationTicks:
    k: dict[str, int | None] = field(default_factory=lambda: dict.fromkeys(
        ("k1", "k2", "k3", "k4", "k5", "k6")))
    sends: list[int] = field(default_factory=list)


def tick_times(records: Iterable[Mapping]) -> dict[int, int]:
    return {r["k"]: r["t_us"] for r in records if "k" in r and "t_us" in r}


def max_frame_gap_us(records: Sequence[Mapping]) -> int:
    for record in records:
        if "grid" in record:
            return int(record["grid"]["max_frame_gap_us"])
    raise ValueError("the journal records no tick grid")


def response_link(records: Sequence[Mapping], sys_id: int) -> str:
    """The serial1 device of vehicle ``sys_id``'s SITL: the journal grid's SITL
    whose serial0 is that vehicle's companion link (``serial0_device``)."""
    for record in records:
        if "grid" in record:
            companion = serial0_device(sys_id)
            ports = [p for p in record["grid"]["sitl_ports"] if p[0] == companion]
            if not ports:
                raise ValueError(f"the journal's grid names no SITL with {companion}")
            return str(ports[0][1])
    raise ValueError("the journal records no tick grid")


def _first_line_tick(lines: Iterable[tuple[int, str]], marker: str) -> int | None:
    return next((k for k, line in lines if marker in line), None)


def confirmation_ticks(records: Sequence[Mapping], lines: Sequence[tuple[int, str]],
                       poi: int, sys_id: int) -> ConfirmationTicks:
    ticks = ConfirmationTicks()
    ticks.k["k1"] = _first_line_tick(lines, f"CONFIRMING: P{poi}")
    approval_id = None
    for record in records:
        operator = record.get("operator", {})
        for received in operator.get("received", ()):  # evidence appends the identity
            if ticks.k["k3"] is None and received[1] == CONFIRM_REQUEST_EVENT:
                ticks.k["k2"], ticks.k["k3"] = received[0], record["k"]
        for sent in operator.get("sent", ()):
            if approval_id is None and sent[2] == "POST" and sent[3] == APPROVAL_TARGET:
                approval_id, ticks.k["k4"] = sent[:2], record["k"]
        for answered in operator.get("answered", ()):
            if approval_id is not None and ticks.k["k6"] is None and answered[:2] == approval_id:
                ticks.k["k6"] = record["k"]
    if ticks.k["k4"] is None or ticks.k["k6"] is None:
        return ticks
    link = response_link(records, sys_id)
    for record in records:
        if ticks.k["k4"] < record.get("k", -1) <= ticks.k["k6"]:
            sends = record.get("navlink", {}).get(link, {}).get(RESPONSE_DIRECTION, {})
            count = sum(n for key, n in sends.items() if message_id(key) == RESPONSE_ID)
            ticks.sends.extend([record["k"]] * count)
    ticks.k["k5"] = ticks.sends[0] if ticks.sends else None
    return ticks


def confirmation_timing_errors(records: Sequence[Mapping], lines: Sequence[tuple[int, str]],
                               approval: Mapping, poi: int, sys_id: int) -> list[str]:
    ticks = confirmation_ticks(records, lines, poi, sys_id)
    missing = [name for name, value in ticks.k.items() if value is None]
    if missing:
        return [f"confirmation timing: the journal holds no {', '.join(missing)}"]
    k = ticks.k
    t = tick_times(records)
    gap = max_frame_gap_us(records)
    errors = []
    if not k["k1"] <= k["k2"] <= k["k3"]:
        errors.append(f"confirmation timing: not k1 <= k2 <= k3 ({k['k1']}, {k['k2']}, {k['k3']})")
    delay = _us(float(approval["requested_delay_s"]))
    waited = t[k["k4"]] - t[k["k3"]]
    upper = delay + 2 * _us(CONFIRMATION_POLL_S) + 2 * gap
    if not delay <= waited <= upper:
        errors.append(f"confirmation timing: t(k4) - t(k3) = {waited} us outside [{delay}, {upper}]")
    if k["k5"] != k["k4"] + 1:
        errors.append(f"confirmation timing: first response send at k{k['k5']}, not k4 + 1 = {k['k4'] + 1}")
    if len(ticks.sends) != TASK_CONFIRM_RESPONSE_SENDS or ticks.sends[-1] != k["k6"]:
        errors.append(f"confirmation timing: response sends at ticks {ticks.sends}, expected "
                      f"{TASK_CONFIRM_RESPONSE_SENDS} ending at k6 = {k['k6']}")
    low = _us(TASK_CONFIRM_RESPONSE_RESEND_GAP_S)
    high = low + gap
    for before, after in zip(ticks.sends, ticks.sends[1:]):
        interval = t[after] - t[before]
        if not low <= interval <= high:
            errors.append(f"confirmation timing: resend interval t({after}) - t({before}) = "
                          f"{interval} us outside [{low}, {high}]")
    audited = float(approval["actual_delay_s"])
    expected = (t[k["k6"]] - t[k["k3"]]) / 1_000_000
    if abs(audited - expected) > ACTUAL_DELAY_TOLERANCE_S:
        errors.append(f"confirmation timing: audited actual_delay_s {audited} != t(k6) - t(k3) = {expected}")
    return errors


__all__ = [
    "APPROVAL_TARGET", "CONFIRM_REQUEST_EVENT", "RESPONSE_DIRECTION", "RESPONSE_ID", "ConfirmationTicks", "confirmation_ticks",
    "confirmation_timing_errors", "max_frame_gap_us", "response_link", "tick_times",
]
