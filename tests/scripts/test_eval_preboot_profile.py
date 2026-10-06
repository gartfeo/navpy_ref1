"""The bench profile must reach the aircraft, and must reach it unambiguously.

``run_swarm.sh`` takes one DEFAULTS path and its ``[[ -f ]]`` preflight rejects
a comma list, so the profile can only be applied by building a file. Two
things can go wrong quietly there: the delta can fail to arrive at all, and it
can arrive TWICE -- once from the launcher template and once from the profile
-- which ArduPilot has no single answer for (``set_float`` runs per row so RAM
holds the last, while the override lookup returns the first,
AP_Param.cpp:2560). Neither is visible in a run's own output.
"""
import ast
import importlib.util
import inspect
import pathlib
import sys
import unittest
from unittest.mock import MagicMock, patch

_ROOT = pathlib.Path(__file__).resolve().parents[2]

# The delta the bench actually ships. Named here rather than recomputed so a
# test cannot agree with a mistake by making it twice.
TIMING = {"GPS1_DELAY_MS": 80.0, "EK3_HGT_DELAY": 0.0}
# Every category bit in SITL.h's NoiseCategory today, so every noise-bearing
# SIM parameter is gated. This is a literal, and nothing here reads the
# firmware enum: widening the enum will NOT fail this test. What it catches is
# the profile changing to a narrower mask than the bench asked for.
NOISE_CATEGORIES = 13
MASK = {"SIM_NOISE_OFF": float((1 << NOISE_CATEGORIES) - 1)}
NOISE_OFF = (
    "SIM_DRIFT_SPEED", "SIM_DRIFT_TIME",
    "SIM_BARO_RND", "SIM_BAR2_RND", "SIM_BAR3_RND",
    "SIM_ARSPD_RND", "SIM_ARSPD2_RND", "SIM_FLOW_RND",
    # These four only exist in our fork: upstream applies both the IMU noise
    # floor and the airframe disturbance as hard-coded literals. Without them
    # the run flies with noise nothing can turn off -- 0.01 m/s^2 and
    # 0.04 deg/s on every sample, plus a throttle-scaled 0.3 m/s^2 and
    # 0.1 deg/s on the airframe's own state.
    "SIM_GYR_RND_MIN", "SIM_ACC_RND_MIN",
    "SIM_DYN_GYR_RND", "SIM_DYN_ACC_RND",
)


def _load(name, filename):
    for entry in (str(_ROOT), str(_ROOT / "src"), str(_ROOT / "scripts")):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    spec = importlib.util.spec_from_file_location(
        name, _ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    # Keep a private module for each test without replacing the module that
    # other collected tests have already imported functions from.
    previous = sys.modules.get(spec.name)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        if previous is None:
            sys.modules.pop(spec.name, None)
        else:
            sys.modules[spec.name] = previous
    return module


class TestShippedProfile(unittest.TestCase):
    """What the file says is what the investigation concluded."""

    def setUp(self) -> None:
        self.preboot = _load("eval_preboot_params", "eval_preboot_params.py")
        self.m = _load("eval_preboot_profile", "eval_preboot_profile.py")
        self.parsed = self.preboot.parse_parm(
            self.m.PROFILE.read_text(encoding="utf-8"))

    def test_the_timing_pair_carries_the_measured_values(self) -> None:
        # These two are the whole reason the profile is a default. 80 ms is
        # SITL's effective GPS lag against ArduPilot's assumed 120, and 0
        # matches EK3_HGT_DELAY to a barometer SITL does not delay. Either one
        # alone leaves a channel differential, which is what displaces the
        # published position at closest approach.
        for name, value in TIMING.items():
            self.assertEqual(self.parsed.get(name), value, name)

    def test_every_noise_source_is_zeroed(self) -> None:
        for name in NOISE_OFF:
            self.assertEqual(self.parsed.get(name), 0.0, name)

    def test_the_master_switch_disables_every_category(self) -> None:
        # A SET bit disables that category, so every bit set is all noise off.
        # The explicit rows above do not name IMU vibration, sonar, GPS byte
        # loss, vicon or loop-time jitter; this is what turns those off.
        for name, value in MASK.items():
            self.assertEqual(self.parsed.get(name), value, name)

    def test_the_paired_calibrations_are_left_alone(self) -> None:
        # SIM_MAG*_OFS pairs with COMPASS_OFS and SIM_ARSPD_OFS with the
        # airspeed calibration. Zeroing one side of a pair INTRODUCES an
        # error, so "turn the noise off" must not reach them.
        for name in self.parsed:
            self.assertFalse(name.endswith("_OFS"), name)

    def test_nothing_in_the_profile_is_written_again_after_boot(self) -> None:
        # reject_postboot_conflicts raises on an overlap, and every name here
        # is now pre-boot on every run, so one clash would fail every launch
        # after the span was already claimed. Checked against the real sets,
        # not a sample.
        from types import SimpleNamespace
        preboot = _load("eval_preboot_params", "eval_preboot_params.py")
        profile = _load("eval_preboot_profile", "eval_preboot_profile.py")
        wanted = preboot.parse_parm(
            profile.PROFILE.read_text(encoding="utf-8"))
        args = SimpleNamespace(wind_speed=0.0, wind_dir=0.0, sitl_param=[])
        from scripts.eval_sim_parameters import sim_parameters
        later = sim_parameters(args) + preboot.LATER_WRITERS
        preboot.reject_postboot_conflicts(wanted, later)

    def test_no_row_exceeds_what_the_firmware_reads_at_once(self) -> None:
        # Including comment rows: the firmware takes 98 bytes per fgets
        # (AP_Filesystem.cpp:270) and only treats '#' as a comment at line[0]
        # (AP_Param.cpp:2263), so a cut comment tail is read as a parameter.
        for number, raw in enumerate(
                self.m.PROFILE.read_text(encoding="utf-8").splitlines(), 1):
            self.assertLessEqual(len(raw.encode("utf-8")), 98, f"row {number}")


class TestCombining(unittest.TestCase):
    def setUp(self) -> None:
        self.preboot = _load("eval_preboot_params", "eval_preboot_params.py")
        self.m = _load("eval_preboot_profile", "eval_preboot_profile.py")

    def _combined(self, template_text, profile_text, tmp=None):
        path = pathlib.Path(tmp or self._tmp())
        path.write_text(profile_text, encoding="utf-8")
        with patch.object(self.m, "_read_template",
                          return_value=template_text):
            return self.m.combined_text(path, "~/template.parm")

    def _tmp(self):
        import tempfile
        handle = tempfile.NamedTemporaryFile(
            suffix=".parm", delete=False, mode="w")
        handle.close()
        self.addCleanup(lambda: pathlib.Path(handle.name).unlink(missing_ok=True))
        return handle.name

    def test_the_template_and_the_profile_both_arrive(self) -> None:
        text = self._combined("ARSPD_FBW_MIN 12\n", "EK3_HGT_DELAY 0\n")
        self.assertEqual(
            self.preboot.parse_parm(text),
            {"ARSPD_FBW_MIN": 12.0, "EK3_HGT_DELAY": 0.0},
        )

    def test_an_overridden_name_appears_once_with_the_profile_value(self) -> None:
        # The ambiguity guard. Appending would leave the name twice, and which
        # value ArduPilot flies is then not well defined.
        text = self._combined("EK3_HGT_DELAY 60\nTRIM_ARSPD_CM 2200\n",
                              "EK3_HGT_DELAY 0\n")
        self.assertEqual(text.count("EK3_HGT_DELAY"), 1)
        parsed = self.preboot.parse_parm(text)
        self.assertEqual(parsed["EK3_HGT_DELAY"], 0.0)
        self.assertEqual(parsed["TRIM_ARSPD_CM"], 2200.0)

    def test_an_overridden_name_is_matched_whatever_separator_it_used(self) -> None:
        # ArduPilot accepts NAME=VALUE and NAME,VALUE as well as whitespace,
        # so a template written either way must still be recognised as the
        # same parameter -- otherwise the duplicate slips through.
        for row in ("EK3_HGT_DELAY=60", "EK3_HGT_DELAY,60"):
            with self.subTest(row=row):
                text = self._combined(row + "\n", "EK3_HGT_DELAY 0\n")
                self.assertEqual(
                    self.preboot.parse_parm(text), {"EK3_HGT_DELAY": 0.0})

    def test_a_trailing_hash_does_not_hide_a_duplicate(self) -> None:
        # Found by review. The firmware honours a '#' only at line[0]
        # (AP_Param.cpp:2263); anywhere else the row still tokenises, so
        # "GPS1_DELAY_MS #comment" IS a parameter row to it and strtof
        # reads the comment as 0. Treating it as a comment here left the
        # row beside the profile's own -- exactly the duplicate this
        # module exists to prevent.
        text = self._combined("GPS1_DELAY_MS #comment\n",
                              "GPS1_DELAY_MS 80\n")
        self.assertEqual(text.count("GPS1_DELAY_MS"), 1)
        self.assertEqual(
            self.preboot.parse_parm(text), {"GPS1_DELAY_MS": 80.0})

    def test_a_template_comment_naming_the_parameter_is_kept(self) -> None:
        # A comment has no parameter name, so dropping it would be a bug: the
        # template's own annotations should survive into what is booted.
        text = self._combined("# EK3_HGT_DELAY is set below\nEK3_HGT_DELAY 60\n",
                              "EK3_HGT_DELAY 0\n")
        self.assertIn("# EK3_HGT_DELAY is set below", text)
        self.assertEqual(self.preboot.parse_parm(text), {"EK3_HGT_DELAY": 0.0})

    def test_an_empty_profile_is_refused(self) -> None:
        # Returning the bare template would boot a stock aircraft while every
        # caller believed the bench configuration was in force.
        with self.assertRaises(RuntimeError):
            self._combined("ARSPD_FBW_MIN 12\n", "# nothing here\n")

    def test_an_unparsable_result_is_refused_before_launch(self) -> None:
        # The check is on the merged bytes, so a bad template row is caught
        # too -- not only a bad profile row.
        with self.assertRaises(ValueError):
            self._combined("ARSPD_FBW_MIN not-a-number\n", "EK3_HGT_DELAY 0\n")

    def test_the_real_profile_combines_cleanly(self) -> None:
        text = self._combined("ARSPD_FBW_MIN 12\n",
                              self.m.PROFILE.read_text(encoding="utf-8"),
                              tmp=self._tmp())
        parsed = self.preboot.parse_parm(text)
        for name, value in TIMING.items():
            self.assertEqual(parsed.get(name), value, name)
        self.assertEqual(parsed["ARSPD_FBW_MIN"], 12.0)


class TestMaterialise(unittest.TestCase):
    def setUp(self) -> None:
        self.preboot = _load("eval_preboot_params", "eval_preboot_params.py")
        self.m = _load("eval_preboot_profile", "eval_preboot_profile.py")

    def test_the_template_is_decoded_as_utf8(self) -> None:
        # Found by review. subprocess text mode uses the console codepage --
        # cp1252 on this host -- so a UTF-8 template came back mojibaked. The
        # damage is not cosmetic: a valid 62-byte comment re-encodes to 122
        # and then trips the firmware's 98-byte row guard, turning a good
        # template into a refused launch.
        import subprocess
        comment = "# " + 'Ã©' * 30
        payload = (comment + '\n' + "ARSPD_FBW_MIN 12" + '\n')
        emitted = payload.encode("utf-8")
        self.assertGreater(len(emitted), len(payload))

        class _Probe:
            returncode = 0
            stdout = emitted
            stderr = b""

        with patch.object(self.m, "_wsl", return_value="wsl.exe"), \
                patch.object(self.m, "normalize_wsl_path",
                             return_value=("/home/x/plane.parm", "")), \
                patch.object(subprocess, "run", return_value=_Probe()):
            text = self.m._read_template("~/plane.parm")
        self.assertEqual(text, payload)
        # And the round trip must not inflate the row past what the firmware
        # reads at once, which is what the mis-decode did.
        self.assertEqual(len(text.splitlines()[0].encode("utf-8")),
                         len(comment.encode("utf-8")))

    def test_a_failed_template_read_names_the_cause(self) -> None:
        class _Probe:
            returncode = 1
            stdout = b""
            stderr = "cat: no such file".encode("utf-8")

        import subprocess
        with patch.object(self.m, "_wsl", return_value="wsl.exe"), \
                patch.object(self.m, "normalize_wsl_path",
                             return_value=("/home/x/plane.parm", "")), \
                patch.object(subprocess, "run", return_value=_Probe()):
            with self.assertRaises(RuntimeError) as caught:
                self.m._read_template("~/plane.parm")
        self.assertIn("no such file", str(caught.exception))

    def test_a_hung_template_read_is_bounded(self) -> None:
        # Unbounded, the launch would sit forever with no message.  Asserting
        # only that TimeoutExpired becomes RuntimeError would stay green if the
        # timeout argument were deleted -- subprocess would then never raise it
        # -- so check the bound is actually REQUESTED as well as handled.
        import subprocess
        asked = {}

        def _hang(*args, **kwargs):
            asked.update(kwargs)
            raise subprocess.TimeoutExpired("cat", 30)

        with patch.object(self.m, "_wsl", return_value="wsl.exe"), \
                patch.object(self.m, "normalize_wsl_path",
                             return_value=("/home/x/plane.parm", "")), \
                patch.object(subprocess, "run", side_effect=_hang):
            with self.assertRaises(RuntimeError):
                self.m._read_template("~/plane.parm")
        self.assertIsNotNone(asked.get("timeout"), "the read was not bounded")
        self.assertGreater(asked["timeout"], 0)

    def test_the_file_is_written_and_its_wsl_path_returned(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            destination = pathlib.Path(directory) / "preboot.parm"
            with patch.object(self.m, "wsl_executable", return_value="wsl.exe"), \
                    patch.object(self.m, "run_text",
                                 return_value=("/mnt/c/x/preboot.parm", "")), \
                    patch.object(self.m, "_read_template",
                                 return_value="ARSPD_FBW_MIN 12\n"):
                path = self.m.materialise(destination)
            self.assertEqual(path, "/mnt/c/x/preboot.parm")
            parsed = self.preboot.parse_parm(
                destination.read_text(encoding="utf-8"))
            self.assertEqual(parsed["EK3_HGT_DELAY"], 0.0)

    def test_a_path_wsl_cannot_express_is_refused(self) -> None:
        # Launching anyway would hand run_swarm.sh a path it cannot open, and
        # the SITL binary panics rather than falling back to the template.
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(self.m, "wsl_executable", return_value="wsl.exe"), \
                    patch.object(self.m, "run_text",
                                 return_value=(None, "wslpath exited 1")), \
                    patch.object(self.m, "_read_template",
                                 return_value="ARSPD_FBW_MIN 12\n"):
                with self.assertRaises(RuntimeError) as caught:
                    self.m.materialise(pathlib.Path(directory) / "preboot.parm")
        self.assertIn("wslpath exited 1", str(caught.exception))


class TestFleetDefault(unittest.TestCase):
    """The profile has to be the default, and the gate has to see that file."""

    def setUp(self) -> None:
        _load("eval_preboot_params", "eval_preboot_params.py")
        import importlib
        self.fleet = importlib.import_module(
            "scripts.eval_direct_pixel_pn_fleet")
        # The module object the HARNESS holds, not a second copy loaded here:
        # patching the wrong one would leave the harness calling the real
        # thing and the test passing on nothing.
        self.m = self.fleet.profile

    def _args(self, **fields):
        from types import SimpleNamespace
        fields.setdefault("sitl_defaults", None)
        fields.setdefault("no_preboot_profile", False)
        return SimpleNamespace(**fields)

    def test_a_plain_run_builds_the_profile(self) -> None:
        with patch.object(self.m, "materialise",
                          return_value="/mnt/c/x/preboot.parm") as built:
            chosen, owned = self.fleet.profile.defaults_for(
                self._args(), pathlib.Path("case"))
        self.assertEqual(chosen, "/mnt/c/x/preboot.parm")
        self.assertEqual(built.call_args.args[0], pathlib.Path("case/preboot.parm"))
        # The gate must be handed every profile parameter, not only the ones
        # that differ from the template.
        self.assertEqual(owned, self.m.owned())
        # Derived from the named sets rather than a literal, so adding a row
        # to one of them cannot leave this silently checking a stale count.
        self.assertEqual(len(owned),
                         len(TIMING) + len(NOISE_OFF) + len(MASK))

    def test_an_explicit_file_replaces_the_profile_rather_than_adding(self) -> None:
        with patch.object(self.m, "materialise") as built:
            chosen, owned = self.fleet.profile.defaults_for(
                self._args(sitl_defaults="/home/gart/mine.parm"),
                pathlib.Path("case"))
        self.assertEqual(chosen, "/home/gart/mine.parm")
        # Nothing is force-verified: the profile is not in force, so its
        # parameters are not claims this run is making.
        self.assertEqual(owned, {})
        built.assert_not_called()

    def test_the_opt_out_boots_the_launcher_template(self) -> None:
        with patch.object(self.m, "materialise") as built:
            chosen, owned = self.fleet.profile.defaults_for(
                self._args(no_preboot_profile=True), pathlib.Path("case"))
        self.assertIsNone(chosen)
        self.assertEqual(owned, {})
        built.assert_not_called()

    def test_the_default_is_on_in_the_parser(self) -> None:
        # A flag that defaulted to True would silently disable the profile for
        # every run while the resolver above still passed its own tests.
        parsed = self.fleet._parser().parse_args([])
        self.assertFalse(parsed.no_preboot_profile)
        self.assertIsNone(parsed.sitl_defaults)

    def test_the_profile_is_hashed_into_the_fleet_identity(self) -> None:
        # Found by review. The identity closure follows Python imports and
        # source_identity globs *.py, so this .parm -- which decides every
        # aircraft's GPS and height-delay timing -- sat outside the hash. An
        # edit between repetitions would change the experiment while
        # require_same_source still passed.
        from scripts.eval_fleet_setup import (
            FLEET_IDENTITY_ROOTS, fleet_source_identity,
        )
        self.assertIn(self.m.PROFILE, FLEET_IDENTITY_ROOTS)
        before = fleet_source_identity()
        original = self.m.PROFILE.read_bytes()
        try:
            self.m.PROFILE.write_bytes(original + b"\n# stray edit\n")
            self.assertNotEqual(fleet_source_identity(), before)
        finally:
            self.m.PROFILE.write_bytes(original)
        self.assertEqual(fleet_source_identity(), before)

    def test_a_value_the_template_shares_is_still_verified(self) -> None:
        # Found by review. overrides() diffs against the launcher template, so
        # a profile value the template happens to match drops out of the
        # readback -- while the cloned eeprom outranks BOTH and could fly
        # something else entirely, unchecked.
        owned = {"GPS1_DELAY_MS": 80.0}
        master = MagicMock()
        with patch.object(self.fleet, "sim_parameters", return_value=()), \
                patch.object(self.fleet.preboot, "overrides",
                             return_value={}), \
                patch.object(self.fleet.preboot,
                             "reject_postboot_conflicts") as clash, \
                patch.object(self.fleet.preboot, "verify") as checks:
            self.fleet.gate_preboot_defaults(
                master, [121], self._args(), "/mnt/c/x/preboot.parm", owned)
        checks.assert_called_once_with(master, [121], owned)
        # and it must reach the conflict check too, or a post-boot write of
        # that same name would go unnoticed.
        self.assertEqual(clash.call_args.args[0], owned)

    def test_the_file_launched_is_the_file_verified(self) -> None:
        # Structural, like the gate's own call-site test: reaching this for
        # real needs a launched fleet. Two call sites taking the same name is
        # what stops the gate proving one file while another one boots.
        tree = ast.parse(inspect.getsource(self.fleet.run_fleet))
        launched = verified = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            name = getattr(node.func, "attr", getattr(node.func, "id", None))
            if name == "_start_swarm":
                launched = [kw.value.id for kw in node.keywords
                            if kw.arg == "sitl_defaults"
                            and isinstance(kw.value, ast.Name)]
            elif name == "gate_preboot_defaults":
                verified = [arg.id for arg in node.args
                            if isinstance(arg, ast.Name)]
        self.assertEqual(launched, ["defaults"])
        self.assertIn("defaults", verified or [])


if __name__ == "__main__":
    unittest.main()
