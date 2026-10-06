"""Requested pre-boot values must be proven live, not assumed from the file.

Handing run_swarm.sh a DEFAULTS file selects what the binary loads; it does not
decide what the vehicle flies. ArduPilot applies a defaults entry only when the
parameter is not already configured in storage, and each instance's eeprom is
cloned from a template, so a saved value outranks the file without saying so.
"""
import importlib.util
import math
import pathlib
import sys
import unittest
from unittest.mock import MagicMock, patch

_ROOT = pathlib.Path(__file__).resolve().parents[2]


def _load():
    # The repo ROOT is required as well as scripts/ and src/: the module
    # reaches eval_navigation_vehicle_config, which does `from scripts import
    # eval_certificate`. Without it, direct execution fails to import.
    for entry in (str(_ROOT), str(_ROOT / "src"), str(_ROOT / "scripts")):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    spec = importlib.util.spec_from_file_location(
        "preboot_params", _ROOT / "scripts" / "eval_preboot_params.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestParsing(unittest.TestCase):
    def setUp(self) -> None:
        self.m = _load()

    def test_comments_and_blank_lines_are_ignored(self) -> None:
        parsed = self.m.parse_parm(
            "# a comment\n\nEK3_HGT_DELAY 120\nSIM_BARO_RND 0.2  # trailing\n")
        # Values come back as float32 -- what the vehicle stores. 0.2 is not
        # representable there, so the stored value is 0.20000000298023224.
        self.assertEqual(parsed, {"EK3_HGT_DELAY": 120.0,
                                  "SIM_BARO_RND": self.m._f32(0.2)})

    def test_a_comma_separated_row_still_parses(self) -> None:
        self.assertEqual(self.m.parse_parm("EK3_HGT_DELAY,120"),
                         {"EK3_HGT_DELAY": 120.0})

    def test_the_equals_forms_parse(self) -> None:
        # ArduPilot accepts these (AP_Param.cpp); parsing them to nothing would
        # empty the override set and skip verification silently.
        for spelling in ("EK3_HGT_DELAY=120", "EK3_HGT_DELAY = 120"):
            self.assertEqual(self.m.parse_parm(spelling),
                             {"EK3_HGT_DELAY": 120.0}, spelling)

    def test_a_row_without_a_value_is_skipped_not_fatal(self) -> None:
        self.assertEqual(self.m.parse_parm("EK3_HGT_DELAY\nSIM_X 1"),
                         {"SIM_X": 1.0})


    def test_a_row_longer_than_the_firmware_reads_is_refused(self) -> None:
        # fgets takes at most 98 bytes per call (AP_Filesystem.cpp:270), so a
        # longer row is CUT and its tail read as the next row. Reading the
        # whole row here would certify a value the vehicle never saw: this
        # one is 1.0000001192092896 read whole and 1.0 read as the firmware
        # reads it, and nothing in the run's output shows the difference.
        prefix = "SIM_BARO_RND 1.000000059604644775390625"
        row = prefix + "0" * (98 - len(prefix)) + "1"
        self.assertEqual(len(row), 99)
        with self.assertRaises(ValueError) as caught:
            self.m.parse_parm(row)
        self.assertIn("98", str(caught.exception))

    def test_a_row_at_exactly_the_firmware_limit_still_parses(self) -> None:
        row = "A 1" + " " * (98 - len("A 1"))
        self.assertEqual(len(row), 98)
        self.assertEqual(self.m.parse_parm(row), {"A": 1.0})

    def test_the_limit_counts_bytes_not_characters(self) -> None:
        # The firmware reads one byte at a time, so a multi-byte character
        # costs more than one of its 98.
        row = "# " + "\u00e9" * 49
        self.assertEqual(len(row), 51)
        self.assertEqual(len(row.encode("utf-8")), 100)
        with self.assertRaises(ValueError):
            self.m.parse_parm(row)

    def test_a_value_ardupilot_would_not_apply_is_refused(self) -> None:
        # set_float returns early on nan/inf (AP_Param.cpp:2181), so the
        # vehicle keeps what it had. Accepting the row would certify a value
        # never applied: infinity clamps to the type limit, matching a vehicle
        # sitting at that limit for unrelated reasons. Both spellings must be
        # refused -- one overflows float32 only, the other overflows binary64.
        for token in ("1e39", "1e999"):
            with self.assertRaises(ValueError, msg=token) as caught:
                self.m.parse_parm("EK3_HGT_DELAY " + token)
            self.assertIn("not finite", str(caught.exception), token)

    def test_a_name_is_matched_case_insensitively(self) -> None:
        self.assertEqual(self.m.parse_parm("ek3_hgt_delay 120"),
                         {"EK3_HGT_DELAY": 120.0})

    def test_a_name_repeated_with_the_same_value_is_harmless(self) -> None:
        # Nothing is ambiguous when both rows ask for the same thing.
        self.assertEqual(self.m.parse_parm("A 1\nA 1.0"), {"A": 1.0})

    def test_a_name_repeated_with_a_DIFFERENT_value_is_refused(self) -> None:
        # ArduPilot has no single answer for a duplicated row: set_float is
        # called for each one in turn, so RAM ends up holding the last, while
        # the override lookup returns the FIRST match (AP_Param.cpp:2560).
        # Picking one here would mean verifying a value that may not be the
        # one that flew.
        with self.assertRaises(ValueError) as caught:
            self.m.parse_parm("A 1\nA 2")
        self.assertIn("twice", str(caught.exception))

    def test_an_unreadable_value_is_refused_rather_than_dropped(self) -> None:
        # Silently skipping the row would leave a real override unverified,
        # which is the failure this whole module exists to prevent.
        with self.assertRaises(ValueError) as caught:
            self.m.parse_parm("EK3_HGT_DELAY oops")
        self.assertIn("cannot read value", str(caught.exception))


class TestFileRead(unittest.TestCase):
    """A file that could not be read must not read as an empty one."""

    def setUp(self) -> None:
        self.m = _load()

    def test_a_failed_read_raises_rather_than_returning_nothing(self) -> None:
        # Empty text parses to no overrides, and no overrides means nothing
        # is verified -- the run proceeds looking clean.
        failed = MagicMock(returncode=1, stdout=b"")
        with patch.object(self.m.subprocess, "run", return_value=failed):
            with self.assertRaises(RuntimeError) as caught:
                self.m._read_wsl_file("/home/gart/x.parm")
        self.assertIn("/home/gart/x.parm", str(caught.exception))

    def test_the_requested_path_is_the_one_read(self) -> None:
        ok = MagicMock(returncode=0, stdout=b"A 1\n")
        with patch.object(self.m.subprocess, "run", return_value=ok) as run:
            self.m._read_wsl_file("/home/gart/x.parm")
        self.assertIn("/home/gart/x.parm",
                      " ".join(str(a) for a in run.call_args[0][0]))


class TestStrtofSemantics(unittest.TestCase):
    """Values are read as ArduPilot's strtof reads them, not as float() does.

    Every case here is one where Python's float() answers differently or
    refuses outright, so together they pin the parser to C semantics.
    """

    def setUp(self) -> None:
        self.m = _load()

    def test_hex_parses(self) -> None:
        # Four of the 75 files under Tools/autotest/default_params use hex;
        # float() raises on it, which would refuse a shipped defaults file.
        self.assertEqual(self.m.strtof("0xF0"), 240.0)

    def test_a_hex_fraction_parses(self) -> None:
        # libc strtof("0x.8p1") is 1.0. A form the pattern misses is not
        # skipped -- it reads as something else, or costs the file its parse.
        self.assertEqual(self.m.strtof("0x.8p1"), 1.0)

    def test_conversion_stops_at_the_first_unusable_character(self) -> None:
        self.assertEqual(self.m.strtof("120;note"), 120.0)

    def test_an_underscore_terminates_the_number(self) -> None:
        # strtof halts at the underscore; Python float() reads 1_20 as 120.
        self.assertEqual(self.m.strtof("1_20"), 1.0)

    def test_a_token_with_no_numeric_prefix_reads_as_nothing(self) -> None:
        self.assertIsNone(self.m.strtof("oops"))

    def test_a_negative_hex_value_keeps_its_sign(self) -> None:
        self.assertEqual(self.m.strtof("-0xF0"), -240.0)

    def test_a_leading_plus_is_accepted(self) -> None:
        self.assertEqual(self.m.strtof("+120"), 120.0)

    def test_the_hex_prefix_and_exponents_are_case_insensitive(self) -> None:
        # strtof accepts either case. Rejecting one reads the value as
        # something else entirely -- 0X10 becomes 0, 1E2 becomes 1.
        self.assertEqual(self.m.strtof("0X10"), 16.0)
        self.assertEqual(self.m.strtof("0x1P2"), 4.0)
        self.assertEqual(self.m.strtof("1E2"), 100.0)

    def test_the_result_is_the_float32_a_vehicle_stores(self) -> None:
        # Not binary64: 0.2 has no float32 representation, and the value the
        # vehicle holds is the one this check has to compare against.
        self.assertEqual(self.m.strtof("0.2"), self.m._f32(0.2))
        self.assertNotEqual(self.m.strtof("0.2"), 0.2)

    def test_an_exact_midpoint_rounds_to_even(self) -> None:
        # A decimal exactly between two float32 values. glibc -- the C library
        # the SITL binary links -- rounds half to even; each expectation here
        # was read out of a compiled strtof under WSL, not derived from this
        # code. (Windows UCRT answers differently at these points, which is
        # why the oracle has to be the one the vehicle actually uses.)
        for token, expected in (
                ("116634.78515625", 116634.78125),
                ("-116634.78515625", -116634.78125),
                ("46252.025390625", 46252.0234375),
                ("35256.275390625", 35256.2734375)):
            self.assertEqual(self.m.strtof(token), expected, token)

    def test_the_overflow_boundary_is_decided_exactly(self) -> None:
        # 2**128 - 2**103 is where IEEE sends a result to infinity instead of
        # the largest finite float32. One below still rounds down to it --
        # but as a double it rounds UP onto the boundary and then overflows,
        # so deciding this from a binary64 approximation gets it wrong.
        boundary = 2 ** 128 - 2 ** 103
        largest = 3.4028234663852886e38
        self.assertEqual(self.m.strtof(str(boundary - 1)), largest)
        self.assertEqual(self.m.strtof(str(-(boundary - 1))), -largest)
        self.assertEqual(self.m.strtof(str(boundary)), float("inf"))
        self.assertEqual(self.m.strtof(str(-boundary)), float("-inf"))
        # The same case in the short hex form the pattern also accepts.
        self.assertEqual(self.m.strtof("0x1.fffffeffffffffffffp127"), largest)

    def test_an_exponent_too_large_to_compute_still_converts(self) -> None:
        # The pattern accepts any run of exponent digits, and Decimal refuses
        # to build a number from most of them. The answer is never in doubt:
        # it is zero or infinity, and the sign survives.
        self.assertEqual(self.m.strtof("1e-9999999999999999999"), 0.0)
        self.assertEqual(self.m.strtof("1e9999999999999999999"), float("inf"))
        self.assertEqual(self.m.strtof("0e9999999999999999999"), 0.0)
        self.assertEqual(self.m.strtof("0x1p-99999999999999"), 0.0)
        self.assertEqual(self.m.strtof("0x1p99999999999999"), float("inf"))
        self.assertEqual(
            math.copysign(1.0, self.m.strtof("-1e-9999999999999999999")), -1.0)

    def test_an_in_range_exponent_is_never_clamped(self) -> None:
        # The bound that makes absurd exponents computable must not touch a
        # real one. A binary exponent runs to -149 on this type, far past the
        # decimal bound, so sharing one margin between the two bases silently
        # converts 0x1p-100 as 0x1p-64 -- a wrong answer, not a refusal.
        self.assertEqual(self.m.strtof("0x1p-100"), 2.0 ** -100)
        self.assertEqual(self.m.strtof("0x1p-149"), 2.0 ** -149)
        self.assertEqual(self.m.strtof("0x1p120"), 2.0 ** 120)
        # 1e-45 is below the smallest float32 subnormal and rounds up to it;
        # the point is that it survives as a number rather than clamping to 0.
        self.assertEqual(self.m.strtof("1e-45"), 2.0 ** -149)
        self.assertEqual(self.m.strtof("1e38"),
                         self.m.strtof("1" + "0" * 38))

    def test_a_signed_zero_keeps_its_sign(self) -> None:
        # C strtof("-0") is -0.0. Nothing here compares the two differently,
        # so this costs no verdict -- but the conversion claims C semantics.
        for token in ("-0", "-0.0", "-0x0p0", "-1e-46"):
            self.assertEqual(math.copysign(1.0, self.m.strtof(token)), -1.0,
                             token)
        self.assertEqual(math.copysign(1.0, self.m.strtof("0")), 1.0)

    def test_a_value_past_float32_saturates_rather_than_raising(self) -> None:
        # strtof returns HUGE_VALF; the caller refuses it by name. Raising
        # out of struct or float.fromhex instead would be a traceback with
        # no parameter in it.
        for token in ("1e39", "1e999", "0x1p1024"):
            self.assertEqual(self.m.strtof(token), float("inf"), token)
        self.assertEqual(self.m.strtof("-1e999"), float("-inf"))

    def test_parsing_uses_these_semantics_not_python_float(self) -> None:
        self.assertEqual(
            self.m.parse_parm("A 0xF0\nB 1_20\nC 120;note\n"),
            {"A": 240.0, "B": 1.0, "C": 120.0})


class TestValueComparison(unittest.TestCase):
    """One absolute threshold was wrong in both directions; these are the cases."""

    def setUp(self) -> None:
        self.m = _load()

    def test_a_correctly_stored_large_float_is_not_a_mismatch(self) -> None:
        # float32 cannot hold 12345.678 exactly; the readback is the truth.
        self.assertTrue(self.m.same_value(12345.678, 12345.677734375))

    def test_a_small_request_that_read_back_as_zero_is_a_mismatch(self) -> None:
        self.assertFalse(self.m.same_value(0.00009, 0.0))

    def test_two_close_but_distinct_values_are_not_equal(self) -> None:
        self.assertFalse(self.m.same_value(0.007265, 0.0073))

    def test_zero_matches_zero(self) -> None:
        self.assertTrue(self.m.same_value(0.0, 0.0))

    def test_adjacent_bitmask_integers_are_not_equal(self) -> None:
        # ARMING_CHECK bit 20 (OSD) against bit 0 (All): both exactly
        # representable, and a relative tolerance called them the same.
        self.assertFalse(self.m.same_value(1048577, 1048576))

    def test_distinct_large_integers_are_not_equal(self) -> None:
        self.assertFalse(self.m.same_value(16777218, 16777216))

    def test_one_ulp_apart_is_not_equal(self) -> None:
        # The smallest difference float32 can hold above 1.0 is 2**-23. The
        # value here was previously 1.0000009536743164, which is eight ULP --
        # a far weaker claim than the name made.
        self.assertFalse(self.m.same_value(1.0 + 2.0 ** -23, 1.0))

    def test_an_integer_parameter_rounds_the_request_as_ardupilot_does(self) -> None:
        # AP_Param adds 0.01 then truncates, so a file saying 119.99999 stores
        # 120. EK3_HGT_DELAY is AP_Int16; rejecting that would be a false alarm.
        for requested in (120.00001, 119.99999):
            self.assertTrue(self.m.same_value(requested, 120, 4), requested)

    def test_a_bitmask_difference_survives_the_integer_rounding(self) -> None:
        self.assertFalse(self.m.same_value(1048577, 1048576, 6))

    def test_a_float_parameter_gets_no_integer_rounding(self) -> None:
        self.assertFalse(self.m.same_value(119.99999, 120, 9))

    def test_the_rounding_addition_happens_in_float32(self) -> None:
        # AP_Param assigns the sum to a float before truncating. In double
        # precision 120.99 stays under 121 and truncates to 120, which would
        # accept a stale 120 and reject the correct 121.
        # There is one rounding now, not two: the clamp was applied to the
        # cast result instead, so this assertion fails the moment the float32
        # round is lost.
        self.assertEqual(self.m.as_stored(120.99, 4), 121.0)
        self.assertEqual(self.m.as_stored(-120.99, 4), -121.0)

    def test_a_request_outside_the_type_range_stores_the_limit(self) -> None:
        # constrain_float runs before the cast (AP_Param.cpp), so a correct
        # readback of the limit must not read as a mismatch.
        self.assertEqual(self.m.as_stored(128, 2), 127.0)
        self.assertEqual(self.m.as_stored(-129, 2), -128.0)
        self.assertEqual(self.m.as_stored(40000, 4), 32767.0)
        self.assertEqual(self.m.as_stored(-40000, 4), -32768.0)

    def test_an_untyped_comparison_does_no_integer_rounding(self) -> None:
        self.assertFalse(self.m.same_value(119.99999, 120))

    def test_a_non_finite_request_has_no_stored_value(self) -> None:
        # set_float returns early on nan/inf, so there is nothing to report.
        # Clamping infinity to the type limit would invent a match with a
        # vehicle sitting at that limit for unrelated reasons.
        for bad in (float("inf"), float("-inf"), float("nan")):
            for param_type in (4, 9, None):
                with self.assertRaises(ValueError, msg=(bad, param_type)) as e:
                    self.m.as_stored(bad, param_type)
                # int(nan) raises ValueError by itself, so without this the
                # NaN case passes even with the guard removed.
                self.assertIn("not finite", str(e.exception))

    def test_a_request_cannot_escape_an_int32(self) -> None:
        # constrain_float converts INT32_MAX to float too, giving
        # 2147483648.0f -- one past the type. Returning that would report a
        # stored value the parameter cannot hold.
        self.assertEqual(self.m.as_stored(3e9, 6), 2147483647.0)
        self.assertEqual(self.m.as_stored(-3e9, 6), -2147483648.0)

    def test_an_in_range_int32_is_untouched_by_that_clamp(self) -> None:
        self.assertEqual(self.m.as_stored(1048577, 6), 1048577.0)

    def test_a_type_no_vehicle_reports_is_not_treated_as_an_integer(self) -> None:
        # mav_param_type emits only 2/4/6 for integers and REAL32 for
        # everything else (GCS_MAVLink.cpp:101), and set_float has branches
        # for exactly those. Rounding for a code ArduPilot cannot send would
        # model a conversion it never performs.
        for unreachable in (1, 3, 5, 7, 8):
            self.assertEqual(self.m.as_stored(119.99999, unreachable),
                             self.m.as_stored(119.99999), unreachable)


class TestOverrides(unittest.TestCase):
    def setUp(self) -> None:
        self.m = _load()

    SUPPLIED = "/home/gart/x.parm"

    def _with_files(self, supplied: str, template: str):
        # Answer by PATH, not by call order. A mock that pops a queue passes
        # even if the code reads the template where it meant to read the
        # supplied file -- which in production would compare the template
        # against itself and find no overrides to verify at all.
        contents = {self.SUPPLIED: supplied, self.m.LAUNCHER_TEMPLATE: template}

        def answer(path: str) -> str:
            for known, text in contents.items():
                if known in path:
                    return text
            raise AssertionError(f"read an unexpected path: {path!r}")

        return patch.object(self.m, "_read_wsl_file", side_effect=answer)

    def test_the_supplied_file_is_the_one_that_is_read(self) -> None:
        # Reading the template in its place yields {} -- no overrides, so
        # nothing is verified and the run proceeds unchecked.
        seen = []
        with self._with_files("A 250\n", "A 60\n") as read:
            self.m.overrides(self.SUPPLIED)
            seen = [call.args[0] for call in read.call_args_list]
        self.assertTrue(any(self.SUPPLIED in path for path in seen), seen)
        self.assertTrue(
            any(self.m.LAUNCHER_TEMPLATE in path for path in seen), seen)

    def test_a_double_rounded_value_does_not_lose_its_override(self) -> None:
        # This decimal sits a hair above the float32 midpoint above 1.0, so C
        # strtof answers 1.0000001192092896. Converting through binary64 first
        # lands exactly ON that midpoint and rounds to 1.0 -- equal to the
        # template, so the override vanished and was never verified.
        with self._with_files("A 1.0000000596046448\n", "A 1\n"):
            self.assertEqual(self.m.overrides(self.SUPPLIED),
                             {"A": 1.0000001192092896})

    def test_a_supplied_path_is_shell_quoted(self) -> None:
        # The path reaches a shell inside WSL. Unquoted, a space or a
        # semicolon changes which file is read, or what else runs.
        seen = []

        def answer(path: str) -> str:
            seen.append(path)
            return "A 1\n"

        with patch.object(self.m, "_read_wsl_file", side_effect=answer):
            self.m.overrides("/home/gart/my defaults.parm")
        self.assertTrue(
            any("'" in path or chr(92) in path for path in seen), seen)

    def test_a_caller_supplied_template_is_the_one_compared(self) -> None:
        seen = []

        def answer(path: str) -> str:
            seen.append(path)
            return "A 1\n"

        with patch.object(self.m, "_read_wsl_file", side_effect=answer):
            self.m.overrides(self.SUPPLIED, template="/home/gart/other.parm")
        self.assertTrue(any("other.parm" in path for path in seen), seen)

    def test_a_difference_below_float32_is_correctly_skipped(self) -> None:
        # The other side of the same coin: these two decimals differ as text
        # but both store as 1.0, so there is nothing for the vehicle to prove.
        with self._with_files("A 1.0000000596046447\n", "A 1\n"):
            self.assertEqual(self.m.overrides(self.SUPPLIED), {})

    def test_only_the_differing_entries_are_returned(self) -> None:
        with self._with_files("A 1\nB 2\nC 9\n", "A 1\nB 2\n"):
            self.assertEqual(self.m.overrides("/home/gart/x.parm"), {"C": 9.0})

    def test_a_changed_value_counts_as_an_override(self) -> None:
        with self._with_files("A 250\n", "A 60\n"):
            self.assertEqual(self.m.overrides("/home/gart/x.parm"), {"A": 250.0})

    def test_an_identical_file_asks_for_nothing(self) -> None:
        with self._with_files("A 1\nB 2\n", "A 1\nB 2\n"):
            self.assertEqual(self.m.overrides("/home/gart/x.parm"), {})


class TestTypedReader(unittest.TestCase):
    """The reader must keep param_type; every verify test mocks it away."""

    def setUp(self) -> None:
        self.m = _load()

    def _master(self, message):
        master = MagicMock()
        master.target_system = 121
        master.recv_match.return_value = message
        return master

    def _reply(self, name="EK3_HGT_DELAY", value=120.0, ptype=4):
        message = MagicMock()
        message.param_id = name.encode("ascii")
        message.param_value = value
        message.param_type = ptype
        return message

    def test_the_storage_type_is_returned_with_the_value(self) -> None:
        with patch.object(self.m, "message_from_poi", return_value=True):
            reading = self.m.read_param_typed(
                self._master(self._reply()), "EK3_HGT_DELAY")
        self.assertEqual(reading, (120.0, 4))

    def test_a_reply_for_another_parameter_is_not_accepted(self) -> None:
        with patch.object(self.m, "message_from_poi", return_value=True):
            reading = self.m.read_param_typed(
                self._master(self._reply(name="SIM_BARO_RND")),
                "EK3_HGT_DELAY", timeout_s=0.05)
        self.assertIsNone(reading)

    def test_silence_reads_as_unreadable_rather_than_a_value(self) -> None:
        reading = self.m.read_param_typed(
            self._master(None), "EK3_HGT_DELAY", timeout_s=0.05)
        self.assertIsNone(reading)

    def test_the_parameter_is_actually_requested_by_name(self) -> None:
        # Without the request the vehicle sends nothing, and the read would
        # time out against a live vehicle however well the mock answers.
        master = self._master(self._reply())
        with patch.object(self.m, "message_from_poi", return_value=True):
            self.m.read_param_typed(master, "EK3_HGT_DELAY")
        master.mav.param_request_read_send.assert_called_once()
        args = master.mav.param_request_read_send.call_args.args
        self.assertEqual(args[0], 121)
        self.assertEqual(args[2], b"EK3_HGT_DELAY")

    def test_the_request_asks_by_name_not_by_index(self) -> None:
        # Index -1 tells the firmware to use the name; index 0 makes it
        # answer with whichever parameter happens to be first
        # (GCS_Param.cpp:390), and that reply would be judged as this one.
        master = self._master(self._reply())
        with patch.object(self.m, "message_from_poi", return_value=True):
            self.m.read_param_typed(master, "EK3_HGT_DELAY")
        self.assertEqual(master.mav.param_request_read_send.call_args.args[3], -1)

    def test_the_request_carries_the_selected_component(self) -> None:
        master = self._master(self._reply())
        master.target_component = 7
        with patch.object(self.m, "message_from_poi", return_value=True):
            self.m.read_param_typed(master, "EK3_HGT_DELAY")
        self.assertEqual(
            master.mav.param_request_read_send.call_args.args[1], 7)

    def test_only_param_value_replies_are_waited_for(self) -> None:
        master = self._master(self._reply())
        with patch.object(self.m, "message_from_poi", return_value=True):
            self.m.read_param_typed(master, "EK3_HGT_DELAY")
        kwargs = master.recv_match.call_args.kwargs
        self.assertEqual(kwargs.get("type"), "PARAM_VALUE")
        self.assertTrue(kwargs.get("blocking"))

    def test_the_filter_is_asked_about_the_vehicle_being_read(self) -> None:
        # Passing a fixed system id would filter against the wrong aircraft
        # while still looking like a filter.
        master = self._master(self._reply())
        master.target_system = 123
        seen = []
        with patch.object(self.m, "message_from_poi",
                          side_effect=lambda _m, s: (seen.append(s), True)[1]):
            self.m.read_param_typed(master, "EK3_HGT_DELAY")
        self.assertEqual(seen, [123])

    def test_a_malformed_value_reads_as_unreadable(self) -> None:
        broken = self._reply()
        broken.param_value = "not a number"
        with patch.object(self.m, "message_from_poi", return_value=True):
            reading = self.m.read_param_typed(
                self._master(broken), "EK3_HGT_DELAY", timeout_s=0.05)
        self.assertIsNone(reading)

    def test_a_reply_from_another_vehicle_is_not_accepted(self) -> None:
        # Every vehicle answers on the shared link. Taking the first
        # PARAM_VALUE would let one aircraft vouch for another.
        with patch.object(self.m, "message_from_poi", return_value=False):
            reading = self.m.read_param_typed(
                self._master(self._reply()), "EK3_HGT_DELAY", timeout_s=0.05)
        self.assertIsNone(reading)

    def test_a_name_padded_to_the_field_width_still_matches(self) -> None:
        # MAVLink pads param_id to 16 bytes with NULs; comparing the raw
        # field would reject every real reply.
        padded = self._reply()
        padded.param_id = b"EK3_HGT_DELAY" + bytes(3)
        with patch.object(self.m, "message_from_poi", return_value=True):
            reading = self.m.read_param_typed(
                self._master(padded), "EK3_HGT_DELAY")
        self.assertEqual(reading, (120.0, 4))

    def test_the_reported_type_is_carried_through_whatever_it_is(self) -> None:
        # A reader that returned a constant 4 would satisfy a single-type
        # test while losing the distinction verify depends on.
        for reported in (2, 4, 6, 9):
            with patch.object(self.m, "message_from_poi", return_value=True):
                reading = self.m.read_param_typed(
                    self._master(self._reply(ptype=reported)), "EK3_HGT_DELAY")
            self.assertEqual(reading, (120.0, reported))

    def test_a_reply_without_a_type_field_reads_as_untyped(self) -> None:
        bare = self._reply()
        del bare.param_type
        with patch.object(self.m, "message_from_poi", return_value=True):
            reading = self.m.read_param_typed(self._master(bare), "EK3_HGT_DELAY")
        self.assertEqual(reading, (120.0, 0))


class TestFleetGate(unittest.TestCase):
    """Both halves of the gate must actually run at the call site."""

    def setUp(self) -> None:
        _load()          # puts scripts/ and src/ on the path
        import importlib
        self.fleet = importlib.import_module("scripts.eval_direct_pixel_pn_fleet")

    def _args(self):
        return MagicMock()

    def test_nothing_runs_without_a_defaults_file(self) -> None:
        with patch.object(self.fleet.preboot, "overrides") as reads, \
                patch.object(self.fleet.preboot, "verify") as checks:
            self.fleet.gate_preboot_defaults(
                MagicMock(), [121], self._args(), None)
        reads.assert_not_called()
        checks.assert_not_called()

    def test_the_readback_and_the_conflict_gate_both_run(self) -> None:
        wanted = {"EK3_HGT_DELAY": 120.0}
        master = MagicMock()
        with patch.object(self.fleet, "sim_parameters", return_value=()), \
                patch.object(self.fleet.preboot, "overrides",
                             return_value=wanted), \
                patch.object(self.fleet.preboot,
                             "reject_postboot_conflicts") as clash, \
                patch.object(self.fleet.preboot, "verify") as checks:
            self.fleet.gate_preboot_defaults(
                master, [121, 122], self._args(), "/home/gart/x.parm")
        clash.assert_called_once()
        self.assertEqual(clash.call_args.args[0], wanted)
        checks.assert_called_once_with(master, [121, 122], wanted)

    def test_run_fleet_actually_calls_the_gate(self) -> None:
        # The tests below exercise the helper. This one checks the call site,
        # because deleting it removes every protection here and the helper
        # tests would still pass. Structural on purpose: reaching this line
        # for real needs a launched fleet.
        import ast
        import inspect
        tree = ast.parse(inspect.getsource(self.fleet.run_fleet))
        called = [node.func.id for node in ast.walk(tree)
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Name)]
        self.assertIn("gate_preboot_defaults", called)

    def test_the_defaults_path_the_caller_asked_for_is_the_one_read(self) -> None:
        with patch.object(self.fleet, "sim_parameters", return_value=()), \
                patch.object(self.fleet.preboot, "overrides",
                             return_value={}) as reads, \
                patch.object(self.fleet.preboot, "reject_postboot_conflicts"), \
                patch.object(self.fleet.preboot, "verify"):
            self.fleet.gate_preboot_defaults(
                MagicMock(), [121], self._args(), "/home/gart/chosen.parm")
        reads.assert_called_once_with("/home/gart/chosen.parm")

    def test_the_ordinary_sim_parameters_reach_the_conflict_set(self) -> None:
        # Substituting an empty tuple, as the other tests do, cannot show
        # that these arrive -- and they are the commonest way a pre-boot
        # value gets overwritten.
        with patch.object(self.fleet, "sim_parameters",
                          return_value=(("ARMING_CHECK", 0.0),
                                        ("SIM_RATE_HZ", 1000.0))), \
                patch.object(self.fleet.preboot, "overrides", return_value={}), \
                patch.object(self.fleet.preboot,
                             "reject_postboot_conflicts") as clash, \
                patch.object(self.fleet.preboot, "verify"):
            self.fleet.gate_preboot_defaults(
                MagicMock(), [121], self._args(), "/home/gart/x.parm")
        names = {name for name, _ in clash.call_args.args[1]}
        self.assertIn("ARMING_CHECK", names)
        self.assertIn("SIM_RATE_HZ", names)

    def test_the_later_writers_are_included_in_the_conflict_set(self) -> None:
        # sim_parameters() alone misses the SIM_CPA POI and the final_approach
        # step-down clock, which are written by other paths after boot.
        with patch.object(self.fleet, "sim_parameters", return_value=()), \
                patch.object(self.fleet.preboot, "overrides", return_value={}), \
                patch.object(self.fleet.preboot,
                             "reject_postboot_conflicts") as clash, \
                patch.object(self.fleet.preboot, "verify"):
            self.fleet.gate_preboot_defaults(
                MagicMock(), [121], self._args(), "/home/gart/x.parm")
        names = {name for name, _ in clash.call_args.args[1]}
        # Every one of them, not a representative sample: a name missing from
        # this set is a parameter that can silently replace a proven value.
        self.assertLessEqual(
            {"SIM_CPA_LAT_HI", "SIM_CPA_LAT_LO", "SIM_CPA_LNG_HI",
             "SIM_CPA_LNG_LO", "SIM_CPA_ALT_CM", "SIM_CPA_ENABLE",
             "SIM_SPEEDUP"},
            names)


class TestPostBootConflicts(unittest.TestCase):
    """A proven boot value is worth nothing if something overwrites it."""

    def setUp(self) -> None:
        self.m = _load()

    LATER = (("ARMING_CHECK", 0.0), ("SIM_WIND_SPD", 3.0), ("SIM_RATE_HZ", 1000.0))

    def test_unrelated_post_boot_writes_are_fine(self) -> None:
        self.m.reject_postboot_conflicts({"EK3_HGT_DELAY": 120.0}, self.LATER)

    def test_a_name_written_after_boot_is_named_and_raises(self) -> None:
        # verify() reads before prepare_aircraft writes, so this run would
        # report a clean pre-boot check and then fly the later value.
        with self.assertRaises(RuntimeError) as caught:
            self.m.reject_postboot_conflicts(
                {"SIM_RATE_HZ": 400.0}, self.LATER)
        message = str(caught.exception)
        self.assertIn("SIM_RATE_HZ", message)
        self.assertIn("overwritten after boot", message)

    def test_every_clashing_name_is_reported_not_just_the_first(self) -> None:
        with self.assertRaises(RuntimeError) as caught:
            self.m.reject_postboot_conflicts(
                {"SIM_RATE_HZ": 400.0, "ARMING_CHECK": 1.0}, self.LATER)
        message = str(caught.exception)
        self.assertIn("ARMING_CHECK", message)
        self.assertIn("SIM_RATE_HZ", message)

    def test_nothing_requested_cannot_clash(self) -> None:
        self.m.reject_postboot_conflicts({}, self.LATER)


class TestVerify(unittest.TestCase):
    def setUp(self) -> None:
        self.m = _load()

    def _master(self):
        master = MagicMock()
        master.target_system = 0
        return master

    def test_nothing_requested_makes_no_round_trips(self) -> None:
        with patch.object(self.m, "read_param_typed") as read:
            self.m.verify(self._master(), [121], {})
        read.assert_not_called()

    def test_a_matching_value_passes(self) -> None:
        with patch.object(self.m, "read_param_typed", return_value=(120.0, 4)):
            self.m.verify(self._master(), [121, 122], {"EK3_HGT_DELAY": 120.0})

    def test_a_stale_eeprom_value_is_named_and_raises(self) -> None:
        with patch.object(self.m, "read_param_typed", return_value=(60.0, 4)):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(self._master(), [121], {"EK3_HGT_DELAY": 120.0})
        message = str(caught.exception)
        self.assertIn("EK3_HGT_DELAY is 60 on 121, asked 120", message)
        self.assertIn("eeprom", message)

    def test_the_storage_type_decides_whether_a_readback_matches(self) -> None:
        # Same request, same readback, different storage class. An AP_Int16
        # rounds 119.99999 to 120; a float parameter does not. Dropping the
        # type from the comparison loses exactly this distinction.
        for param_type, should_raise in ((4, False), (9, True)):
            with patch.object(self.m, "read_param_typed",
                              return_value=(120.0, param_type)):
                if should_raise:
                    with self.assertRaises(RuntimeError):
                        self.m.verify(self._master(), [121],
                                      {"EK3_HGT_DELAY": 119.99999})
                else:
                    self.m.verify(self._master(), [121],
                                  {"EK3_HGT_DELAY": 119.99999})

    def test_every_requested_parameter_is_read_by_name(self) -> None:
        # Checking only the first would leave later overrides unverified,
        # and a wrong name would verify a parameter nobody asked about.
        asked = []
        with patch.object(
                self.m, "read_param_typed",
                side_effect=lambda m, n: (asked.append(n), (120.0, 4))[1]):
            self.m.verify(self._master(), [121],
                          {"EK3_HGT_DELAY": 120.0, "SIM_BARO_RND": 120.0})
        self.assertEqual(sorted(asked), ["EK3_HGT_DELAY", "SIM_BARO_RND"])

    def test_a_mismatch_on_a_LATER_parameter_is_still_caught(self) -> None:
        values = {"EK3_HGT_DELAY": (120.0, 4), "SIM_BARO_RND": (60.0, 4)}
        with patch.object(self.m, "read_param_typed",
                          side_effect=lambda m, n: values[n]):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(self._master(), [121],
                              {"EK3_HGT_DELAY": 120.0, "SIM_BARO_RND": 120.0})
        self.assertIn("SIM_BARO_RND is 60", str(caught.exception))

    def test_a_request_the_type_cannot_hold_is_not_certified(self) -> None:
        # The vehicle clamps silently, so the readback CAN match -- and the
        # check would pass on a flight that never tested the request. At the
        # AP_Int32 bound the C conversion is undefined as well.
        with patch.object(self.m, "read_param_typed",
                          return_value=(2147483648.0, 6)):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(self._master(), [121], {"BIG": 3e9})
        message = str(caught.exception)
        self.assertIn("outside the", message)
        self.assertIn("2147483647", message)

    def test_an_in_range_request_is_unaffected_by_that_rule(self) -> None:
        with patch.object(self.m, "read_param_typed",
                          return_value=(1048577.0, 6)):
            self.m.verify(self._master(), [121], {"MASK": 1048577.0})

    def test_both_bounds_of_every_integer_type_are_refused(self) -> None:
        # Not the AP_Int32 upper bound alone: the vehicle clamps every
        # integer type, so a readback can match a request it never honoured
        # whichever end it fell off.
        for param_type, over, under in ((2, 128, -129),
                                        (4, 32768, -32769),
                                        (6, 3e9, -3e9)):
            for request in (over, under):
                with patch.object(self.m, "read_param_typed",
                                  return_value=(0.0, param_type)):
                    with self.assertRaises(RuntimeError,
                                           msg=(param_type, request)):
                        self.m.verify(self._master(), [121], {"P": request})

    def test_the_endpoints_themselves_are_accepted(self) -> None:
        # Refusing these would reject a legitimate request. AP_Int32's upper
        # endpoint is absent on purpose -- see the test below.
        for param_type, low, high in ((2, -128, 127),
                                      (4, -32768, 32767),
                                      (6, -2147483648, 1073741824)):
            for request in (low, high):
                with patch.object(self.m, "read_param_typed",
                                  return_value=(float(request), param_type)):
                    self.m.verify(self._master(), [121], {"P": float(request)})

    def test_int32_max_cannot_be_requested_over_a_float32_field(self) -> None:
        # 2147483647 has no float32 representation; the nearest is
        # 2147483648, one past the type. A PARAM_VALUE field carries float32,
        # so this request cannot reach the vehicle intact whatever the
        # firmware then does with it -- and the C cast of that value is
        # undefined besides. Refusing is the only honest answer.
        with patch.object(self.m, "read_param_typed",
                          return_value=(2147483648.0, 6)):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(self._master(), [121], {"P": 2147483647.0})
        self.assertIn("arrives as 2147483648", str(caught.exception))

    def test_the_rule_reads_the_converted_request(self) -> None:
        # 127.0000001 stores as 127 on an AP_Int8, so it is in range and
        # verifies -- the rule is about what the vehicle would hold, not
        # about the arithmetic value of the text.
        with patch.object(self.m, "read_param_typed", return_value=(127.0, 2)):
            self.m.verify(self._master(), [121], {"P": 127.0000001})

    def test_the_autopilot_component_is_selected_before_each_read(self) -> None:
        # An inherited component from an earlier exchange would send the
        # request somewhere that never answers.
        seen = []
        with patch.object(
                self.m, "read_param_typed",
                side_effect=lambda m, n: (
                    seen.append((m.target_system, m.target_component)),
                    (120.0, 4))[1]):
            self.m.verify(self._master(), [121, 122], {"P": 120.0})
        self.assertEqual(seen, [(121, 1), (122, 1)])

    def test_an_unreadable_parameter_raises(self) -> None:
        with patch.object(self.m, "read_param_typed", return_value=None):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(self._master(), [121], {"EK3_HGT_DELAY": 120.0})
        self.assertIn("unreadable", str(caught.exception))

    def test_every_vehicle_is_selected_before_it_is_read(self) -> None:
        # A call count alone passes even if the target is never switched, so
        # capture which vehicle was actually selected at each read.
        master = self._master()
        seen = []
        with patch.object(
                self.m, "read_param_typed",
                side_effect=lambda m, n: (
                    seen.append(m.target_system), (120.0, 4))[1]):
            self.m.verify(master, [121, 122, 123], {"EK3_HGT_DELAY": 120.0})
        self.assertEqual(seen, [121, 122, 123])

    def test_a_mismatch_on_a_LATER_vehicle_is_still_caught(self) -> None:
        master = self._master()
        values = {121: 120.0, 122: 120.0, 123: 60.0}
        with patch.object(
                self.m, "read_param_typed",
                side_effect=lambda m, n: (values[m.target_system], 4)):
            with self.assertRaises(RuntimeError) as caught:
                self.m.verify(master, [121, 122, 123], {"EK3_HGT_DELAY": 120.0})
        message = str(caught.exception)
        self.assertIn("EK3_HGT_DELAY is 60 on 123, asked 120", message)
        self.assertNotIn("unreadable", message)
        self.assertNotIn("on 121", message)


if __name__ == "__main__":
    unittest.main()
