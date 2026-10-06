"""A pre-boot parameter file reaches run_swarm.sh through DEFAULTS.

Values the EKF reads at buffer-allocation time, and the ones the barometer
reads at calibration time, are consumed before the harness pushes any
parameters: `eval_sim_parameters.py` runs after SITL has booted.  The only
lever left is run_swarm.sh's own `${DEFAULTS:-.../models/plane.parm}`, so
`launch_command` has to be able to override it per launch -- editing the
shared template instead would change every concurrent session's SITL.
"""
import importlib.util
import pathlib
import sys
import unittest

_ROOT = pathlib.Path(__file__).resolve().parents[2]


def _load(name: str):
    # swarm_run_runner imports its siblings by bare name, the way swarm_run.py
    # puts scripts/ on the path before importing it.
    scripts = str(_ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location(
        f"preboot_{name}", _ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TestPrebootDefaults(unittest.TestCase):
    def setUp(self) -> None:
        self.wsl = _load("swarm_run_wsl")

    def _command(self, **kwargs) -> str:
        return self.wsl.launch_command(1.0, 0, "1,2,3", 3, 0, **kwargs)

    def test_defaults_absent_leaves_the_shared_template_in_force(self) -> None:
        self.assertNotIn("DEFAULTS=", self._command())

    def test_defaults_path_is_exported_before_the_script(self) -> None:
        command = self._command(defaults_path="/home/gart/eval.parm")
        self.assertIn("DEFAULTS=/home/gart/eval.parm", command)
        # run_swarm.sh only sees it as an inline environment assignment, which
        # must precede the script it is meant to configure.
        self.assertLess(command.index("DEFAULTS="), command.index("run_swarm.sh"))

    def test_a_path_carrying_shell_syntax_cannot_run_as_a_command(self) -> None:
        # The string is handed to `bash -lc`, so an unquoted `;` would execute.
        payload = "/tmp/a b;touch /tmp/pwned"
        command = self._command(defaults_path=payload)
        # The whole payload must appear inside ONE single-quoted token; any
        # split would let the shell read past the `;`.
        self.assertIn(f"DEFAULTS='{payload}'", command)
        # And nothing unquoted may follow the separator.
        self.assertNotIn(f"DEFAULTS={payload}", command)


class TestSupervisorContract(unittest.TestCase):
    def test_parser_accepts_defaults_and_defaults_to_none(self) -> None:
        runner = _load("swarm_run_runner")
        parser = runner.build_parser(1.0)
        self.assertIsNone(parser.parse_args([]).defaults)
        parsed = parser.parse_args(["--defaults", "/home/gart/eval.parm"])
        self.assertEqual(parsed.defaults, "/home/gart/eval.parm")



class TestDefaultsPreflight(unittest.TestCase):
    """An unusable path must be refused early, and for the right reason.

    run_swarm.sh does reject a missing file, and SITL panics if it cannot open
    one, so these checks do not prevent a silent fallback -- they move the
    refusal ahead of the registry slot and the launch, and name the cause.
    """

    def setUp(self) -> None:
        self.wsl = _load("swarm_run_wsl")

    def test_a_windows_rewritten_path_is_refused_by_name(self) -> None:
        with self.assertRaises(SystemExit) as caught:
            self.wsl.ensure_defaults_file(
                "C:/Program Files/Git/home/gart/eval.parm")
        self.assertIn("MSYS_NO_PATHCONV", str(caught.exception))

    def test_an_unreadable_wsl_path_is_refused(self) -> None:
        from unittest.mock import MagicMock, patch
        with patch.object(self.wsl.subprocess, "run",
                          return_value=MagicMock(returncode=1)):
            with self.assertRaises(SystemExit) as caught:
                self.wsl.ensure_defaults_file("/home/gart/missing.parm")
        self.assertIn("readable file", str(caught.exception))

    def test_a_readable_wsl_path_passes(self) -> None:
        from unittest.mock import MagicMock, patch
        with patch.object(self.wsl.subprocess, "run",
                          return_value=MagicMock(returncode=0)):
            self.assertIsNone(
                self.wsl.ensure_defaults_file("/home/gart/eval.parm"))


class TestPredicateEdges(unittest.TestCase):
    """Cases the launcher would reject later, or misread entirely."""

    def setUp(self) -> None:
        self.wsl = _load("swarm_run_wsl")

    def test_a_relative_path_is_refused(self) -> None:
        with self.assertRaises(SystemExit) as caught:
            self.wsl.ensure_defaults_file("models/plane.parm")
        self.assertIn("absolute", str(caught.exception))

    def test_an_empty_path_is_refused_rather_than_ignored(self) -> None:
        with self.assertRaises(SystemExit):
            self.wsl.ensure_defaults_file("")

    def test_a_comma_is_refused_because_ardupilot_splits_on_it(self) -> None:
        with self.assertRaises(SystemExit) as caught:
            self.wsl.ensure_defaults_file("/home/gart/a,b.parm")
        self.assertIn("comma", str(caught.exception))

    def test_a_directory_is_refused(self) -> None:
        # Real WSL call: `test -r` alone passes for a directory, which would
        # reach the launcher before being rejected there.
        #
        # Prove WSL answers first. Without this the test also passes when WSL
        # is unreachable, which is a refusal for entirely the wrong reason --
        # observed on a machine where the probe returned E_ACCESSDENIED.
        import subprocess
        reachable = subprocess.run(
            self.wsl.wsl_argv("test -f /etc/hostname"), capture_output=True)
        if reachable.returncode != 0:
            self.skipTest(
                "WSL did not answer a control probe "
                f"(rc={reachable.returncode}); a refusal here would prove "
                "nothing about directories")
        with self.assertRaises(SystemExit):
            self.wsl.ensure_defaults_file("/tmp")

    def test_the_probe_requires_a_regular_file_AND_readability(self) -> None:
        # Joined with "or", a readable directory or an unreadable regular
        # file passes the preflight and fails later, after a slot is claimed.
        from unittest.mock import MagicMock, patch
        with patch.object(self.wsl.subprocess, "run",
                          return_value=MagicMock(returncode=0)) as run:
            self.wsl.ensure_defaults_file("/home/gart/eval.parm")
        sent = " ".join(str(a) for a in run.call_args[0][0])
        self.assertIn("test -f", sent)
        self.assertIn("test -r", sent)
        self.assertIn("&&", sent)
        self.assertNotIn("||", sent)

    def test_the_probe_quotes_the_path(self) -> None:
        # BOTH tests take the path. Asserting the quoted form appears once
        # passes while the other half is still bare, and a bare path with a
        # space in it tests two different files.
        from unittest.mock import MagicMock, patch
        with patch.object(self.wsl.subprocess, "run",
                          return_value=MagicMock(returncode=0)) as run:
            self.wsl.ensure_defaults_file("/home/gart/my defaults.parm")
        sent = " ".join(str(a) for a in run.call_args[0][0])
        self.assertEqual(sent.count("'/home/gart/my defaults.parm'"), 2, sent)

    def test_the_probe_tests_for_a_regular_file(self) -> None:
        from unittest.mock import MagicMock, patch
        with patch.object(self.wsl.subprocess, "run",
                          return_value=MagicMock(returncode=0)) as run:
            self.wsl.ensure_defaults_file("/home/gart/eval.parm")
        sent = " ".join(str(a) for a in run.call_args[0][0])
        self.assertIn("test -f", sent)
        self.assertIn("/home/gart/eval.parm", sent)


if __name__ == "__main__":
    unittest.main()
