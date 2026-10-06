"""Step 3: LogSchema/LogField composition and formatting tests."""
from __future__ import annotations

import unittest

from navpy.logger.log_schema import (
    PRIMARY_SCHEMA,
    LogField,
    LogSchema,
)


class LogFieldTests(unittest.TestCase):
    def test_field_formatter_invoked(self):
        f = LogField("x", lambda v: "Z" if v is None else f"!{v}!")
        self.assertEqual(f.formatter(None), "Z")
        self.assertEqual(f.formatter(42), "!42!")


class LogSchemaHeaderTests(unittest.TestCase):
    def test_header_line_joins_names_with_commas(self):
        schema = LogSchema(
            "demo",
            (
                LogField("a", lambda v: str(v)),
                LogField("b", lambda v: str(v)),
                LogField("c", lambda v: str(v)),
            ),
        )
        self.assertEqual(schema.header_line(), "a,b,c")

    def test_column_names_ordered(self):
        schema = LogSchema(
            "demo",
            (
                LogField("x", str),
                LogField("y", str),
            ),
        )
        self.assertEqual(schema.column_names, ("x", "y"))


class LogSchemaFormatRowTests(unittest.TestCase):
    def test_empty_string_for_missing_key(self):
        """Missing keys must not raise; field formatter receives None."""
        schema = LogSchema(
            "demo",
            (
                LogField("a", lambda v: "" if v is None else str(v)),
                LogField("b", lambda v: "" if v is None else str(v)),
            ),
        )
        self.assertEqual(schema.format_row({"a": 1}), "1,")

    def test_row_respects_field_order(self):
        schema = LogSchema(
            "demo",
            (
                LogField("z", str),
                LogField("a", str),
            ),
        )
        self.assertEqual(schema.format_row({"z": "first", "a": "second"}), "first,second")

    def test_compose_by_concatenation_of_fields(self):
        """New schemas are built by concatenating LogField tuples; no
        central switch in this module."""
        base = LogSchema("b", (LogField("x", str), LogField("y", str)))
        extra = LogSchema("e", (LogField("z", str),))
        composed = LogSchema("b+e", base.fields + extra.fields)
        self.assertEqual(composed.header_line(), "x,y,z")


class PrimarySchemaCompatibilityTests(unittest.TestCase):
    """PRIMARY_SCHEMA extends the legacy header; it never rewrites it.

    ``src_t`` was appended so sim-time row throttling stops being analysed
    against the wall-clock ``ts`` column. The compatibility contract that
    matters is POSITIONAL: readers index compact rows (``row[0]`` timestamp,
    ``row[4]`` cmd_r), so the legacy block must stay first, in order, with its
    original formatters, and anything new must come after it.
    """

    _LEGACY_HEADER = (
        "ts,dist,h_dist,v_dist,cmd_r,cmd_p,"
        "yaw_err,pitch_err,act_r,act_p,x_err,y_err"
    )

    def test_legacy_columns_are_still_the_leading_block(self):
        self.assertTrue(
            PRIMARY_SCHEMA.header_line().startswith(self._LEGACY_HEADER + ","),
            PRIMARY_SCHEMA.header_line(),
        )

    def test_source_time_column_is_appended_last(self):
        self.assertEqual(PRIMARY_SCHEMA.column_names[-1], "src_t")
        self.assertEqual(
            PRIMARY_SCHEMA.column_names[:12],
            tuple(self._LEGACY_HEADER.split(",")),
        )

    def test_column_count(self):
        self.assertEqual(len(PRIMARY_SCHEMA.fields), 13)

    def test_source_time_is_blank_rather_than_wall_time_when_absent(self):
        # A source-clock column filled from the wall clock is the confusion the
        # column exists to end, so "no source clock" must render as empty.
        self.assertEqual(PRIMARY_SCHEMA.fields[-1].formatter(None), "")
        self.assertEqual(PRIMARY_SCHEMA.fields[-1].formatter(float("nan")), "")
        self.assertEqual(PRIMARY_SCHEMA.fields[-1].formatter(12.3456), "12.346")

    def test_primary_row_legacy_bytes(self):
        """Format a row using legacy input values and compare against the
        legacy f-string exactly."""
        values = {
            "ts": "12:34:56.789",
            "dist": 123.456,
            "h_dist": 120.5,
            "v_dist": 10.25,
            "cmd_r": 5.4,
            "cmd_p": -7.8,
            "yaw_err": 1.1,
            "pitch_err": -2.2,
            "act_r": 3.3,
            "act_p": -4.4,
            "x_err": 42.7,
            "y_err": -11.2,
        }
        row = PRIMARY_SCHEMA.format_row(values)
        legacy = (
            "12:34:56.789,123.5,120.5,10.2,5.4,-7.8,"
            "1.1,-2.2,3.3,-4.4,43,-11"
        )
        # Legacy cells byte-for-byte, then the appended source-clock cell.
        self.assertEqual(row, legacy + ",")
        values["src_t"] = 987.6543
        self.assertEqual(PRIMARY_SCHEMA.format_row(values), legacy + ",987.654")

    def test_missing_cmd_r_becomes_empty_cell(self):
        values = {
            "ts": "00:00:00.000",
            "dist": 0.0,
            "h_dist": 0.0,
            "v_dist": 0.0,
            "cmd_r": None,
            "cmd_p": None,
            "yaw_err": 0.0,
            "pitch_err": 0.0,
            "act_r": 0.0,
            "act_p": 0.0,
            "x_err": None,
            "y_err": None,
        }
        row = PRIMARY_SCHEMA.format_row(values)
        self.assertEqual(
            row,
            "00:00:00.000,0.0,0.0,0.0,,,0.0,0.0,0.0,0.0,,,",
        )


if __name__ == "__main__":
    unittest.main()
