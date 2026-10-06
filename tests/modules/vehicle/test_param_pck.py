"""Tests for the clean-room @PARAM/param.pck parser.

Synthetic fixtures only; real SITL-captured fixture validation lives in
Step 8 of the full-param-edit feature.
"""
from __future__ import annotations

import struct
import unittest

from navpy.modules.vehicle._mavftp.param_pck import (
    AP_TYPE_FLOAT,
    AP_TYPE_INT8,
    AP_TYPE_INT16,
    AP_TYPE_INT32,
    AP_TYPE_NONE,
    FLAG_HAS_DEFAULT,
    PCK_MAGIC,
    PCK_MAGIC_WITH_DEFAULTS,
    ParamPck,
    ParamPckError,
    ParamRecord,
    parse_param_pck,
)


# ---------------------------------------------------------------------
# Synthetic fixture builder
# ---------------------------------------------------------------------
def _pack_record(
    *,
    ap_type: int,
    flags: int,
    common_len: int,
    suffix: bytes,
    value_bytes: bytes,
    default_bytes: bytes = b"",
) -> bytes:
    name_len_field = len(suffix) - 1
    if not (0 <= ap_type <= 0xF) or not (0 <= flags <= 0xF):
        raise ValueError("type/flags must fit in 4 bits")
    if not (0 <= common_len <= 0xF) or not (0 <= name_len_field <= 0xF):
        raise ValueError("common_len/name_len must fit in 4 bits")
    byte0 = (flags << 4) | ap_type
    byte1 = (name_len_field << 4) | common_len
    return bytes([byte0, byte1]) + suffix + value_bytes + default_bytes


def _pack_pck(records_payload: bytes, *, num_params: int,
              total_params: int | None = None,
              magic: int = PCK_MAGIC) -> bytes:
    if total_params is None:
        total_params = num_params
    return struct.pack("<HHH", magic, num_params, total_params) + records_payload


def _value_bytes(ap_type: int, value) -> bytes:
    if ap_type == AP_TYPE_INT8:
        return struct.pack("<b", value)
    if ap_type == AP_TYPE_INT16:
        return struct.pack("<h", value)
    if ap_type == AP_TYPE_INT32:
        return struct.pack("<i", value)
    if ap_type == AP_TYPE_FLOAT:
        return struct.pack("<f", value)
    raise AssertionError(f"unsupported ap_type {ap_type}")


# ---------------------------------------------------------------------
# Header / empty
# ---------------------------------------------------------------------
class TestHeader(unittest.TestCase):
    def test_truncated_header(self):
        with self.assertRaises(ParamPckError):
            parse_param_pck(b"\x1b\x67\x00")

    def test_bad_magic(self):
        blob = struct.pack("<HHH", 0xDEAD, 0, 0)
        with self.assertRaises(ParamPckError):
            parse_param_pck(blob)

    def test_empty_pck(self):
        result = parse_param_pck(_pack_pck(b"", num_params=0))
        self.assertEqual(result.records, ())
        self.assertEqual(result.num_params, 0)
        self.assertEqual(result.total_params, 0)
        self.assertEqual(result.magic, PCK_MAGIC)

    def test_empty_pck_with_defaults_magic(self):
        result = parse_param_pck(_pack_pck(
            b"", num_params=0, magic=PCK_MAGIC_WITH_DEFAULTS))
        self.assertEqual(result.records, ())
        self.assertEqual(result.magic, PCK_MAGIC_WITH_DEFAULTS)

    def test_num_exceeds_total(self):
        # num_params=2, total_params=1 — inconsistent.
        rec = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                           suffix=b"X", value_bytes=b"\x01")
        blob = _pack_pck(rec * 2, num_params=2, total_params=1)
        with self.assertRaises(ParamPckError):
            parse_param_pck(blob)


# ---------------------------------------------------------------------
# Single-record per type
# ---------------------------------------------------------------------
class TestTypeWidths(unittest.TestCase):
    def _single(self, ap_type, value):
        rec = _pack_record(
            ap_type=ap_type, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(ap_type, value),
        )
        return parse_param_pck(_pack_pck(rec, num_params=1)).records[0]

    def test_int8(self):
        r = self._single(AP_TYPE_INT8, -42)
        self.assertEqual(r.value, -42)
        self.assertIsInstance(r.value, int)
        self.assertEqual(r.ap_type, AP_TYPE_INT8)

    def test_int16(self):
        r = self._single(AP_TYPE_INT16, -32000)
        self.assertEqual(r.value, -32000)
        self.assertIsInstance(r.value, int)

    def test_int32(self):
        r = self._single(AP_TYPE_INT32, 0x7FFFFFFF)
        self.assertEqual(r.value, 0x7FFFFFFF)
        self.assertIsInstance(r.value, int)

    def test_float(self):
        r = self._single(AP_TYPE_FLOAT, 3.14)
        self.assertAlmostEqual(r.value, 3.14, places=5)
        self.assertIsInstance(r.value, float)


# ---------------------------------------------------------------------
# Common-prefix delta encoding
# ---------------------------------------------------------------------
class TestDeltaCompression(unittest.TestCase):
    def test_two_records_share_prefix(self):
        # ARMING_REQUIRE then ARMING_CHECK.
        r1 = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=0,
            suffix=b"ARMING_REQUIRE",
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
        )
        # common_len = len("ARMING_") = 7, suffix = "CHECK"
        r2 = _pack_record(
            ap_type=AP_TYPE_INT16, flags=0, common_len=7,
            suffix=b"CHECK",
            value_bytes=_value_bytes(AP_TYPE_INT16, 0x40),
        )
        result = parse_param_pck(_pack_pck(r1 + r2, num_params=2))
        names = [r.name for r in result.records]
        self.assertEqual(names, ["ARMING_REQUIRE", "ARMING_CHECK"])
        self.assertEqual(result.records[1].value, 0x40)

    def test_first_record_with_common_len_raises(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=3,
            suffix=b"X", value_bytes=b"\x01",
        )
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=1))

    def test_common_len_exceeds_prev_name(self):
        # First record name "AB" (len 2), second claims common_len=5.
        r1 = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                          suffix=b"AB",
                          value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        r2 = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=5,
                          suffix=b"X",
                          value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(r1 + r2, num_params=2))

    def test_max_suffix_length(self):
        # name_len_field = 15 → suffix_len = 16.
        suffix = b"A" * 16
        rec = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=0, suffix=suffix,
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1))
        self.assertEqual(result.records[0].name, "A" * 16)


# ---------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------
class TestDefaults(unittest.TestCase):
    def test_default_present_in_record(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT16, flags=FLAG_HAS_DEFAULT, common_len=0,
            suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT16, 5),
            default_bytes=_value_bytes(AP_TYPE_INT16, 1),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1),
                                 defaults_requested=True)
        r = result.records[0]
        self.assertEqual(r.value, 5)
        self.assertEqual(r.default, 1)
        self.assertTrue(r.default_known)
        self.assertEqual(r.flags & FLAG_HAS_DEFAULT, FLAG_HAS_DEFAULT)

    def test_default_absent_when_not_requested(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT16, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT16, 5),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1),
                                 defaults_requested=False)
        r = result.records[0]
        self.assertIsNone(r.default)
        self.assertFalse(r.default_known)

    def test_default_inferred_when_requested_and_flag_absent(self):
        """If caller used ?withdefaults=1 but a record's flag bit is absent,
        the default equals the value (server omits matching defaults)."""
        rec = _pack_record(
            ap_type=AP_TYPE_FLOAT, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_FLOAT, 2.5),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1),
                                 defaults_requested=True)
        r = result.records[0]
        self.assertAlmostEqual(r.default, 2.5, places=5)
        self.assertTrue(r.default_known)

    def test_defaults_magic_with_explicit_default(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT16, flags=FLAG_HAS_DEFAULT, common_len=0,
            suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT16, 5),
            default_bytes=_value_bytes(AP_TYPE_INT16, 1),
        )
        result = parse_param_pck(
            _pack_pck(rec, num_params=1, magic=PCK_MAGIC_WITH_DEFAULTS),
            defaults_requested=False,
        )
        r = result.records[0]
        self.assertEqual(r.value, 5)
        self.assertEqual(r.default, 1)
        self.assertTrue(r.default_known)

    def test_defaults_magic_infers_default_when_flag_absent(self):
        rec = _pack_record(
            ap_type=AP_TYPE_FLOAT, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_FLOAT, 2.5),
        )
        result = parse_param_pck(
            _pack_pck(rec, num_params=1, magic=PCK_MAGIC_WITH_DEFAULTS),
            defaults_requested=False,
        )
        r = result.records[0]
        self.assertAlmostEqual(r.value, 2.5, places=5)
        self.assertAlmostEqual(r.default, 2.5, places=5)
        self.assertTrue(r.default_known)

    def test_default_flag_with_truncated_default_bytes(self):
        # FLAG_HAS_DEFAULT set but only the value present, default truncated.
        rec_bytes = bytes([FLAG_HAS_DEFAULT << 4 | AP_TYPE_INT32,
                           0 << 4 | (1 - 1)]) + b"P" \
            + _value_bytes(AP_TYPE_INT32, 7)  # missing default 4 bytes
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec_bytes, num_params=1),
                            defaults_requested=True)

    def test_flag_reserved_bits_preserved(self):
        # Set FLAG_HAS_DEFAULT plus a reserved bit; parser must not reject.
        rec = _pack_record(
            ap_type=AP_TYPE_INT8, flags=FLAG_HAS_DEFAULT | 0x4,
            common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
            default_bytes=_value_bytes(AP_TYPE_INT8, 0),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1),
                                 defaults_requested=True)
        self.assertEqual(result.records[0].flags & 0x4, 0x4)


# ---------------------------------------------------------------------
# Type code edge cases
# ---------------------------------------------------------------------
class TestTypeCodes(unittest.TestCase):
    def test_type_zero_in_record_raises(self):
        # Crafted record whose first byte is non-zero (so padding skip
        # doesn't swallow it) but whose type nibble is zero. Set flags=1.
        rec = bytes([(1 << 4) | AP_TYPE_NONE, (1 - 1) << 4 | 0]) + b"P" + b"\x00"
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=1))

    def test_unknown_type_raises(self):
        # ap_type = 5 (Vector3F) not supported in Step 1b.
        rec = bytes([(0 << 4) | 5, (1 - 1) << 4 | 0]) + b"P" + b"\x00" * 4
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=1))


# ---------------------------------------------------------------------
# Truncation / trailing bytes
# ---------------------------------------------------------------------
class TestTruncation(unittest.TestCase):
    def test_mid_record_truncated_value(self):
        rec_partial = bytes([AP_TYPE_INT32, (1 - 1) << 4]) + b"P" + b"\x01\x02"
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec_partial, num_params=1))

    def test_truncated_name_suffix(self):
        # Header claims suffix_len=5 but only 2 bytes of suffix follow.
        bad = bytes([AP_TYPE_INT8, (5 - 1) << 4]) + b"AB"
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(bad, num_params=1))

    def test_record_count_exceeds_data(self):
        # num_params=3 but only 1 record's bytes are present.
        rec = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                           suffix=b"P",
                           value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=3))

    def test_trailing_zero_padding_ignored(self):
        rec = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                           suffix=b"P",
                           value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        blob = _pack_pck(rec, num_params=1) + b"\x00" * 16
        result = parse_param_pck(blob)
        self.assertEqual(len(result.records), 1)

    def test_trailing_nonzero_bytes_raise(self):
        rec = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                           suffix=b"P",
                           value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        blob = _pack_pck(rec, num_params=1) + b"\x00\x42"  # stray nonzero
        with self.assertRaises(ParamPckError):
            parse_param_pck(blob)


# ---------------------------------------------------------------------
# Inter-record padding (server pads chunks to keep records intact)
# ---------------------------------------------------------------------
class TestInterRecordPadding(unittest.TestCase):
    def test_zero_padding_between_records_skipped(self):
        r1 = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                          suffix=b"AA",
                          value_bytes=_value_bytes(AP_TYPE_INT8, 1))
        r2 = _pack_record(ap_type=AP_TYPE_INT8, flags=0, common_len=0,
                          suffix=b"BB",
                          value_bytes=_value_bytes(AP_TYPE_INT8, 2))
        blob = _pack_pck(r1 + b"\x00" * 7 + r2, num_params=2)
        result = parse_param_pck(blob)
        self.assertEqual([r.name for r in result.records], ["AA", "BB"])


# ---------------------------------------------------------------------
# Name decoding
# ---------------------------------------------------------------------
class TestNameDecoding(unittest.TestCase):
    def test_invalid_ascii_raises(self):
        # 0xC3 is non-ASCII; parser must raise rather than silently
        # mis-reconstruct under codepoint vs byte counting.
        rec = bytes([AP_TYPE_INT8, (1 - 1) << 4]) + b"\xC3" + b"\x01"
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=1))

    def test_valid_multibyte_utf8_also_raises(self):
        # Even a fully-formed UTF-8 multi-byte sequence (Latin-1 'é' = 0xC3 0xA9)
        # must raise — names are ASCII per the format documentation.
        rec = bytes([AP_TYPE_INT8, (2 - 1) << 4]) + b"\xC3\xA9" + b"\x01"
        with self.assertRaises(ParamPckError):
            parse_param_pck(_pack_pck(rec, num_params=1))


class TestAdditionalEdgeCases(unittest.TestCase):
    """Codex post-step finding suggestions."""

    def test_int32_negative_boundary(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT32, flags=0, common_len=0, suffix=b"P",
            value_bytes=struct.pack("<i", -2147483648),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1))
        self.assertEqual(result.records[0].value, -2147483648)

    def test_leading_zero_padding_before_first_record(self):
        rec = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
        )
        # Inject 5 zero bytes between header and first record.
        blob = struct.pack("<HHH", PCK_MAGIC, 1, 1) + b"\x00" * 5 + rec
        result = parse_param_pck(blob)
        self.assertEqual(result.records[0].name, "P")

    def test_common_len_equals_prev_name_length(self):
        # First record "AAA"; second record common_len=3, suffix="X" → name "AAAX"
        r1 = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=0, suffix=b"AAA",
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
        )
        r2 = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=3, suffix=b"X",
            value_bytes=_value_bytes(AP_TYPE_INT8, 2),
        )
        result = parse_param_pck(_pack_pck(r1 + r2, num_params=2))
        self.assertEqual([r.name for r in result.records], ["AAA", "AAAX"])

    def test_name_len_field_zero_is_one_suffix_byte(self):
        # name_len_field == 0 → suffix_len = 1.
        rec = _pack_record(
            ap_type=AP_TYPE_INT8, flags=0, common_len=0, suffix=b"P",
            value_bytes=_value_bytes(AP_TYPE_INT8, 1),
        )
        result = parse_param_pck(_pack_pck(rec, num_params=1))
        self.assertEqual(result.records[0].name, "P")
        self.assertEqual(len(result.records[0].name), 1)

    def test_padding_only_before_eof_with_remaining_count_raises(self):
        # Header claims 1 record but body is all zeros — truncated.
        blob = struct.pack("<HHH", PCK_MAGIC, 1, 1) + b"\x00" * 8
        with self.assertRaises(ParamPckError):
            parse_param_pck(blob)


# ---------------------------------------------------------------------
# Nibble decode helper (byte-exact)
# ---------------------------------------------------------------------
class TestDecodeNibbles(unittest.TestCase):
    def test_nibble_split(self):
        from navpy.modules.vehicle._mavftp.param_pck import _decode_nibbles
        # byte = 0xAB → low=0xB, high=0xA
        self.assertEqual(_decode_nibbles(0xAB), (0xB, 0xA))
        self.assertEqual(_decode_nibbles(0x00), (0, 0))
        self.assertEqual(_decode_nibbles(0xFF), (0xF, 0xF))

    def test_record_byte_assembly_round_trip(self):
        # Build a record byte0 = (flags << 4) | type with flags=0x3, type=0x4
        byte0 = (0x3 << 4) | 0x4
        from navpy.modules.vehicle._mavftp.param_pck import _decode_nibbles
        ap_type, flags = _decode_nibbles(byte0)
        self.assertEqual(ap_type, 0x4)
        self.assertEqual(flags, 0x3)


if __name__ == "__main__":
    unittest.main()
