"""The operator's ``TailPort`` over a participant's ``DONE.log_lines``.

The coordinator hands each tick's log lines to the operator
(``OperatorInbox.log_lines``); the operator's journal keeps them per
ordinal. A tail reads one participant's lines as text: ``read_new`` returns
what arrived since the last read, ``text`` everything so far.
"""

from __future__ import annotations

from navpy.sim.lockstep.operator_baton import OperatorBaton


class LockstepLogTail:
    def __init__(self, operator: OperatorBaton, ordinal: int) -> None:
        self._operator = operator
        self._ordinal = ordinal
        self._read = 0

    def _lines(self) -> list[str]:
        return self._operator.journal.log_lines.get(self._ordinal, [])

    def read_new(self) -> str:
        lines = self._lines()
        new, self._read = lines[self._read:], len(lines)
        return "".join(f"{line}\n" for line in new)

    @property
    def text(self) -> str:
        return "".join(f"{line}\n" for line in self._lines())


__all__ = ["LockstepLogTail"]
