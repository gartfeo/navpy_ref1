"""Composable log schemas for compact CSV rows.

Step 3 introduces one built-in schema, PRIMARY_SCHEMA, that exactly
reproduces the existing 12-column navigation CSV. Later steps compose
new per-law or per-signal-source schemas by concatenating fields;
there is no central dispatch.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional, Tuple


@dataclass(frozen=True)
class LogField:
    """A single CSV column with its formatter.

    The formatter receives the raw value (possibly None) and returns the
    exact string that will appear in the CSV cell. Returning "" for None
    preserves the legacy empty-cell convention.
    """

    name: str
    formatter: Callable[[Any], str]


def _format_timestamp(value: Any) -> str:
    return str(value) if value is not None else ""


def _format_float_1(value: Optional[float]) -> str:
    if value is None:
        return ""
    return f"{float(value):.1f}"


def _format_int_round(value: Optional[float]) -> str:
    """Round-to-int cell (matches the legacy %.0f for x_err / y_err)."""
    if value is None:
        return ""
    return f"{float(value):.0f}"


def _format_source_time(value: Optional[float]) -> str:
    """Seconds on the FLIGHT/source clock, at millisecond resolution.

    Empty when the writer had no source clock, deliberately. The wall ``ts``
    column is always present and would be a tempting fallback, but writing wall
    time into a column named for the source clock is how sim-time cadence came to
    be analysed against wall time in the first place -- the exact confusion this
    column was added to end. An empty cell says "this row cannot answer that
    question", which is true and safe.
    """
    if value is None:
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if number != number or number in (float("inf"), float("-inf")):
        return ""
    return f"{number:.3f}"


@dataclass(frozen=True)
class LogSchema:
    """A named, ordered tuple of LogFields that format into a CSV row."""

    name: str
    fields: Tuple[LogField, ...]

    @property
    def column_names(self) -> Tuple[str, ...]:
        return tuple(field.name for field in self.fields)

    def header_line(self) -> str:
        return ",".join(self.column_names)

    def format_row(self, values: Mapping[str, Any]) -> str:
        return ",".join(
            field.formatter(values.get(field.name)) for field in self.fields
        )


#: The legacy 12 columns, in their original order and with their original
#: formatters. Kept as a named constant because it is a compatibility contract,
#: not just the first slice of a tuple: every reader that indexes compact rows
#: positionally (``row[0]`` for the timestamp, ``row[4]`` for cmd_r) depends on
#: nothing being inserted into or before it.
LEGACY_PRIMARY_FIELDS: Tuple[LogField, ...] = (
    LogField("ts", _format_timestamp),
    LogField("dist", _format_float_1),
    LogField("h_dist", _format_float_1),
    LogField("v_dist", _format_float_1),
    LogField("cmd_r", _format_float_1),
    LogField("cmd_p", _format_float_1),
    LogField("yaw_err", _format_float_1),
    LogField("pitch_err", _format_float_1),
    LogField("act_r", _format_float_1),
    LogField("act_p", _format_float_1),
    LogField("x_err", _format_int_round),
    LogField("y_err", _format_int_round),
)

#: The FLIGHT/source clock the row was throttled on, appended after the legacy
#: block. ``ts`` is a wall-clock time of day, so at SIM_SPEEDUP>1 any cadence
#: computed from it is in the wrong time domain -- rows throttled at a sim-time
#: interval look unevenly spaced on the wall, and analyses read that as jitter
#: that does not exist. Appended rather than inserted so positional readers of
#: the legacy block keep working unchanged.
SOURCE_TIME_FIELD = LogField("src_t", _format_source_time)

# The legacy navigation CSV columns, byte-stable, plus the appended source clock.
PRIMARY_SCHEMA = LogSchema(
    name="primary",
    fields=LEGACY_PRIMARY_FIELDS + (SOURCE_TIME_FIELD,),
)

#: The pre-``src_t`` shape, for reading logs written before the column existed.
LEGACY_PRIMARY_SCHEMA = LogSchema(
    name="primary-legacy",
    fields=LEGACY_PRIMARY_FIELDS,
)
