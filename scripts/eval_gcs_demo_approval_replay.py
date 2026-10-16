"""The approval gate's replay of the operator's polling schedule
(``lockstep_fleet.md``, Journal gates, "Approval timing").

The confirmation loop (``run_confirmation_workflow``) starts polling once
(its ``polling`` record: the start tick and the loop's deadline) and then
runs passes of one receive, one due check, the due approvals' POSTs and one
sleep. With the operator's ``evidence`` (``operator_journal``) the journal
holds each of them with its ticks, and this gate replays the loop from the
deliveries, the deadline and the tick grid alone. Every wait must be
explained; the first that is not fails the gate:

- **Start:** the first receive starts at the polling tick; each pass starts
  before the deadline (else the loop would have ended).
- **Receive (FIFO):** only the loop's own stream (its subscribe is the last
  one at or before the polling tick). A receive that starts with a delivery
  queued returns it at its start tick. With an empty queue it returns at the
  next delivery tick, or times out at the first tick at or after start + w,
  w = min(poll, deadline - start); a delivery on the timeout tick wins. It
  consumes the oldest queued delivery (its type and confirmation identity).
- **Due check:** at the receive's return tick, ``now`` that tick's time;
  its due list is every pending request (deduplicated by identity, pending
  from its consumption tick k3) with ``now - t(k3) >= delay``, in
  consumption order.
- **Sends:** the due approvals' POSTs in that order, the first at the due
  check's tick, each later one at its predecessor's answer tick.
- **Sleep:** from the pass's last answer tick (the due check's if nothing
  was sent) to the first tick at or after start + w; the next receive
  starts at the wake tick. The loop's last pass (its stop) has no sleep:
  the loop returns, and its stream's unsubscribe (the first after the
  polling start) is sent at that pass's end.
- **Complete:** every consumption on the stream and every approval POST
  belongs to a replayed pass.
- **Audit:** the approvals are the POSTs, in order, each with the requested
  delay and ``actual_delay_s`` = t(answer) - t(k3).

Times follow the baton's float clock (``t_us / 1e6``) and the loop's float
deadline, so a bound falls on the same tick as the run's timer. k1 and the
backend's response sends stay with the confirmation timing
(``eval_gcs_demo_confirmation_timing``).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from scripts.eval_gcs_demo_confirmation import CONFIRMATION_POLL_S
from scripts.eval_gcs_demo_confirmation_timing import (
    ACTUAL_DELAY_TOLERANCE_S,
    APPROVAL_TARGET,
    CONFIRM_REQUEST_EVENT,
    tick_times,
)
from scripts.eval_gcs_demo_ports import JsonValue

SUBSCRIBE = "subscribe"
UNSUBSCRIBE = "unsubscribe"


class ReplayError(ValueError):
    """The first wait or record the replay cannot explain."""


@dataclass(frozen=True)
class _Entry:
    k: int
    value: list


_KEYS = ("polling", "receive", "due_check", "sleep", "sent", "answered", "delivered", "received")


def _journal(records: Sequence[Mapping]) -> dict[str, list[_Entry]]:
    """The operator's evidence records by key, each with its tick, in journal order."""
    found: dict[str, list[_Entry]] = {key: [] for key in _KEYS}
    for record in records:
        operator = record.get("operator", {})
        for key in _KEYS:
            found[key].extend(_Entry(record["k"], list(value)) for value in operator.get(key, ()))
    return found


class _Grid:
    """The tick grid and the loop's deadline, on the baton's float clock."""

    def __init__(self, records: Sequence[Mapping], deadline_s: float) -> None:
        self.t = tick_times(records)
        self.ticks = sorted(self.t)
        self.deadline_s = deadline_s

    def seconds(self, k: int) -> float:
        return self.t[k] / 1_000_000

    def wait(self, start: int) -> float:
        """The loop's wait from ``start``: ``min(poll, max(0, deadline - now))``."""
        return min(CONFIRMATION_POLL_S, max(0.0, self.deadline_s - self.seconds(start)))

    def sleep_wake(self, start: int) -> int:
        """A sleep's timer (``baton_sync.sleep``): due at start + w."""
        due = self.seconds(start) + self.wait(start)
        wake = self._first(start, lambda now: now >= due)
        if wake is None:
            raise ReplayError(f"the journal ends before the sleep from tick {start} wakes")
        return wake

    def receive_timeout(self, start: int) -> int | None:
        """A receive's timeout (``BatonQueue.get``): its queue deadline is
        start + w, its timer due at start + (queue deadline - start), and it
        ends once ``now >= queue deadline``. None: after the journal's end."""
        now = self.seconds(start)
        until = now + self.wait(start)
        timer = now + (until - now)
        return self._first(start, lambda t: t >= timer and t >= until)

    def _first(self, start: int, due: Callable[[float], bool]) -> int | None:
        return next((k for k in self.ticks if k >= start and due(self.seconds(k))), None)


def _delay(approvals: Sequence[Mapping[str, JsonValue]]) -> float:
    delays = {float(row["requested_delay_s"]) for row in approvals}
    if len(delays) != 1:
        raise ReplayError(f"the approvals carry requested delays {sorted(delays)}, not one")
    return delays.pop()


@dataclass
class _Loop:
    """The confirmation loop's records, selected by its polling start and stream."""

    start: int
    receives: list[_Entry]
    checks: list[_Entry]
    sleeps: list[_Entry]
    deliveries: list[_Entry]
    consumed: list[_Entry]
    posts: list[_Entry]
    answers: dict[tuple, int]
    #: The tick of the stream's unsubscribe: the loop's return.
    end: int | None


def _loop(journal: dict[str, list[_Entry]]) -> tuple[_Loop, float]:
    if not journal["due_check"]:
        raise ReplayError("the journal holds no due checks (operator evidence off)")
    if len(journal["polling"]) != 1:
        raise ReplayError(f"{len(journal['polling'])} polling starts, not one")
    (polling,) = journal["polling"]
    start, deadline_s = polling.value
    subscribes = [e.value[:2] for e in journal["sent"] if e.k <= start and e.value[2] == SUBSCRIBE]
    if not subscribes:
        raise ReplayError(f"no subscribe before the polling start at tick {start}")
    stream = subscribes[-1]
    checks = journal["due_check"]
    last_check = checks[-1].k
    loop = _Loop(
        start=start,
        receives=[e for e in journal["receive"] if e.value[3:5] == stream],
        checks=checks,
        # The loop's sleeps: one before it wakes by the polling start; one
        # after it starts at or after the last due check.
        sleeps=[e for e in journal["sleep"] if start <= e.value[0] < last_check],
        deliveries=[e for e in journal["delivered"] if e.value[:2] == stream],
        consumed=[e for e in journal["received"] if e.value[5:7] == stream],
        posts=[e for e in journal["sent"] if e.value[2] == "POST" and e.value[3] == APPROVAL_TARGET],
        answers={tuple(e.value[:2]): e.k for e in journal["answered"]},
        end=next((e.k for e in journal["sent"] if e.k >= start and e.value[2] == UNSUBSCRIBE), None),
    )
    if len(loop.receives) != len(checks):
        raise ReplayError(f"{len(loop.receives)} receives on the loop's stream for {len(checks)} due checks")
    if len(loop.sleeps) != len(checks) - 1:
        raise ReplayError(f"{len(loop.sleeps)} sleeps between {len(checks)} passes")
    return loop, float(deadline_s)


def _replay(records: Sequence[Mapping], approvals: Sequence[Mapping[str, JsonValue]]) -> None:
    loop, deadline_s = _loop(_journal(records))
    grid, delay = _Grid(records, deadline_s), _delay(approvals)
    pending: dict[tuple, int] = {}
    seen: set[tuple] = set()
    head = sent = 0
    answered_at: dict[tuple, tuple[int, int]] = {}
    start = loop.start
    for n, (receive, check) in enumerate(zip(loop.receives, loop.checks)):
        begun, returned, ended = receive.value[:3]
        if begun != start:
            raise ReplayError(f"pass {n}: receive starts at tick {begun}, not {start}")
        if grid.seconds(begun) >= deadline_s:
            raise ReplayError(f"pass {n}: starts at tick {begun}, at or after the deadline {deadline_s}")
        if head < len(loop.deliveries) and loop.deliveries[head].k <= begun:
            expected = (begun, "event")
        else:
            timeout = grid.receive_timeout(begun)
            nxt = loop.deliveries[head].k if head < len(loop.deliveries) else None
            if nxt is not None and (timeout is None or nxt <= timeout):
                expected = (nxt, "event")
            elif timeout is None:
                raise ReplayError(f"pass {n}: the journal ends before the receive from tick {begun} times out")
            else:
                expected = (timeout, "timeout")
        if (returned, ended) != expected or receive.k != returned:
            raise ReplayError(f"pass {n}: receive from tick {begun} returned at {returned} "
                              f"({ended}), expected {expected[0]} ({expected[1]})")
        if ended == "event":
            if head >= len(loop.consumed):
                raise ReplayError(f"pass {n}: the receive at tick {returned} consumed nothing")
            got, oldest = loop.consumed[head], loop.deliveries[head]
            if (got.k, got.value[0], got.value[1:5]) != (returned, oldest.k, oldest.value[2:6]):
                raise ReplayError(f"pass {n}: consumed {got.value[1:5]} delivered at {got.value[0]} "
                                  f"on tick {got.k}, not the oldest queued ({oldest.value[2:6]} "
                                  f"delivered at {oldest.k})")
            head += 1
            identity = tuple(got.value[2:5])
            if got.value[1] == CONFIRM_REQUEST_EVENT and identity not in seen:
                seen.add(identity)
                pending[identity] = returned
        k, now_us, due = check.value
        if (check.k, k, now_us) != (returned, returned, grid.t[returned]):
            raise ReplayError(f"pass {n}: due check at tick {k} ({now_us} us), not at the "
                              f"receive's return tick {returned} ({grid.t[returned]} us)")
        now = grid.seconds(returned)
        expected_due = [i for i, k3 in pending.items() if now - grid.seconds(k3) >= delay]
        if [tuple(item) for item in due] != expected_due:
            raise ReplayError(f"pass {n}: due list {due} at tick {returned}, expected "
                              f"{[list(i) for i in expected_due]}")
        last = returned
        for identity in expected_due:
            if sent >= len(loop.posts):
                raise ReplayError(f"pass {n}: no approval POST for {list(identity)}")
            post = loop.posts[sent]
            if (post.k, tuple(post.value[4:7])) != (last, identity):
                raise ReplayError(f"pass {n}: approval POST for {post.value[4:7]} at tick {post.k}, "
                                  f"expected {list(identity)} at tick {last}")
            answer = loop.answers.get(tuple(post.value[:2]))
            if answer is None:
                raise ReplayError(f"pass {n}: the approval POST {post.value[:2]} has no answer")
            answered_at[identity] = (pending.pop(identity), answer)
            last, sent = answer, sent + 1
        if n == len(loop.checks) - 1:
            if loop.end != last:
                raise ReplayError(f"the loop's last replayed pass ends at tick {last}, but its stream "
                                  f"is unsubscribed at tick {loop.end}")
            break
        sleep = loop.sleeps[n]
        wake = grid.sleep_wake(last)
        if sleep.value != [last, wake] or sleep.k != wake:
            raise ReplayError(f"pass {n}: sleep {sleep.value}, expected [{last}, {wake}]")
        start = wake
    if head != len(loop.consumed):
        raise ReplayError(f"{len(loop.consumed) - head} consumptions outside the replayed passes")
    if sent != len(loop.posts):
        raise ReplayError(f"{len(loop.posts) - sent} approval POSTs outside the replayed passes")
    _audit(approvals, answered_at, grid)


def _audit(approvals: Sequence[Mapping[str, JsonValue]], answered_at: dict[tuple, tuple[int, int]],
           grid: _Grid) -> None:
    rows = [(row["sys_id"], row["task_id"], row["round_uid"]) for row in approvals]
    if rows != list(answered_at):
        raise ReplayError(f"approvals {[list(r) for r in rows]} are not the journal's POSTs "
                          f"{[list(i) for i in answered_at]}")
    for row, identity in zip(approvals, rows):
        k3, answer = answered_at[identity]
        journaled = (grid.t[answer] - grid.t[k3]) / 1_000_000
        if abs(float(row["actual_delay_s"]) - journaled) > ACTUAL_DELAY_TOLERANCE_S:
            raise ReplayError(f"approval {list(identity)}: audited actual_delay_s "
                              f"{row['actual_delay_s']} != t({answer}) - t({k3}) = {journaled}")


def approval_replay_errors(records: Sequence[Mapping],
                           approvals: Sequence[Mapping[str, JsonValue]]) -> list[str]:
    """The first unexplained wait or record of the operator's polling schedule."""
    try:
        _replay(records, approvals)
    except ReplayError as error:
        return [f"approval replay: {error}"]
    return []


__all__ = ["approval_replay_errors"]
