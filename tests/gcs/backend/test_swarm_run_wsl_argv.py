"""Every WSL bash-script invocation in swarm_run must reach bash as written.

``wsl <command>`` does not run <command>: it hands its command line to the
distro's DEFAULT SHELL, which re-parses it and only then reaches the ``bash -lc``
we asked for. A ``;`` splits the line into separate commands and a ``$(...)`` is
expanded before bash parses, so the string bash executes is not the string this
file built. ``--exec`` skips that shell.

Nothing in the tree depends on the difference TODAY -- the launch command and the
fork probe have no shell metacharacters, and ``cleanup()``'s pkill fragments
happen to survive the round trip -- which is exactly why it needs pinning: the
next ``$(...)`` added to one of those strings would change meaning silently.
"""
import ast
import functools
import importlib.util
import pathlib
import subprocess
import sys
import unittest
from unittest.mock import MagicMock, patch

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SWARM_RUN = _ROOT / "scripts" / "swarm_run.py"
_SWARM_RUN_MODULES = sorted((_ROOT / "scripts").glob("swarm_run*.py"))


def _load_swarm():
    spec = importlib.util.spec_from_file_location("swarm_run_wsl_argv", _SWARM_RUN)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class TestWslArgv(unittest.TestCase):
    def test_the_builder_asks_wsl_not_to_use_the_default_shell(self):
        m = _load_swarm()
        self.assertEqual(m._wsl_argv("echo hi"),
                         ["wsl", "--exec", "bash", "-lc", "echo hi"])

    def test_exec_comes_before_bash(self):
        # `wsl bash --exec ...` would be an argument to bash, not to wsl.exe.
        m = _load_swarm()
        argv = m._wsl_argv("echo hi")
        self.assertLess(argv.index("--exec"), argv.index("bash"))

    def test_the_script_is_one_argument_and_comes_last(self):
        # Splitting it would hand bash a -c string of "echo" with the rest as
        # positional parameters, which is the failure this whole file is about.
        m = _load_swarm()
        self.assertEqual(m._wsl_argv("echo a; echo b")[-1], "echo a; echo b")
        self.assertEqual(len(m._wsl_argv("echo a; echo b")), 5)


class TestEveryCallSiteUsesIt(unittest.TestCase):
    """One builder, or the sites drift on the one detail that decides meaning."""

    def test_cleanup_goes_through_the_builder(self):
        m = _load_swarm()
        with patch.object(m.subprocess, "run") as run:
            m._wsl("pkill -f -- 'x' || true; sleep 2")
        self.assertEqual(run.call_args.args[0],
                         m._wsl_argv("pkill -f -- 'x' || true; sleep 2"))

    def test_the_fork_probe_goes_through_the_builder(self):
        m = _load_swarm()
        with patch.object(m.subprocess, "run",
                          return_value=MagicMock(returncode=0)) as run:
            m.ensure_fork_supports_companion_udp()
        argv = run.call_args.args[0]
        self.assertEqual(argv[:4], list(m.WSL_BASH_ARGV))
        self.assertIn("COMPANION_UDP", argv[4])

    def test_the_launcher_goes_through_the_builder(self):
        m = _load_swarm()
        with patch.object(m, "_is_regular_file", lambda fd: False), \
             patch.object(m.subprocess, "Popen") as popen:
            m._start_launcher("run_swarm.sh pi -n 3")
        self.assertEqual(popen.call_args.args[0],
                         m._wsl_argv("run_swarm.sh pi -n 3"))

    def test_no_call_site_bypasses_the_builder(self):
        """A site added later must not hand-write the argv and get the reparse.

        Read from the syntax tree rather than the text: it does not care about
        quoting style or formatting, and it sees ``"wsl.exe"`` and single quotes
        too. It only catches a LITERAL argv handed to run/Popen — an argv
        assembled in a variable would slip past, which is a deliberate limit
        rather than an oversight.
        """
        offenders = []
        for path in _SWARM_RUN_MODULES:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or not node.args:
                    continue
                func = node.func
                name = (func.attr if isinstance(func, ast.Attribute)
                        else getattr(func, "id", ""))
                if name not in ("run", "Popen"):
                    continue
                first = node.args[0]
                if not isinstance(first, (ast.List, ast.Tuple)) or not first.elts:
                    continue
                head = first.elts[0]
                if isinstance(head, ast.Constant) and \
                        str(head.value).lower() in ("wsl", "wsl.exe"):
                    offenders.append(f"{path.name}: {ast.unparse(first)}")
        self.assertEqual(offenders, [], f"argv built by hand: {offenders}")


@functools.lru_cache(maxsize=1)
def _wsl_available() -> bool:
    try:
        return subprocess.run(["wsl", "--exec", "true"],
                              capture_output=True, timeout=30).returncode == 0
    except Exception:
        return False


@unittest.skipUnless(sys.platform == "win32", "wsl.exe is Windows-only")
class RealWslArgvSemantics(unittest.TestCase):
    """What bash actually receives, from the real wsl.exe on this machine.

    Deterministic, unlike the write-storm regression: the default shell either
    re-parses the command line or it does not.
    """

    @classmethod
    def setUpClass(cls):
        if not _wsl_available():
            raise unittest.SkipTest("wsl.exe is not available on this machine")

    #: Its output is only correct if ONE bash both expanded the substitution and
    #: ran the loop. Under the default-shell reparse the substitution happens
    #: first and bash is handed multi-line text it cannot parse.
    SCRIPT = "for i in $(seq 1 3); do echo X$i; done"
    EXPECTED = ["X1", "X2", "X3"]

    def _run(self, argv):
        # Short: WSL is already warm by here (_wsl_available probed it), so a
        # script that takes longer than this is wedged, not slow.
        return subprocess.run(argv, capture_output=True, text=True, timeout=15)

    def test_the_production_argv_runs_the_script_as_written(self):
        m = _load_swarm()
        done = self._run(m._wsl_argv(self.SCRIPT))
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual(done.stdout.split(), self.EXPECTED)

    def test_without_exec_the_same_script_is_mangled(self):
        # The control. If this ever passes, wsl.exe changed and the --exec
        # requirement should be re-examined deliberately rather than assumed.
        done = self._run(["wsl", "bash", "-lc", self.SCRIPT])
        self.assertNotEqual(done.stdout.split(), self.EXPECTED,
                            "plain `wsl bash -lc` no longer re-parses; "
                            "re-evaluate WSL_BASH_ARGV")

    def test_bash_receives_the_string_byte_for_byte(self):
        # BASH_EXECUTION_STRING is bash's own record of its -c argument, so this
        # reads the contract directly rather than inferring it from behaviour.
        m = _load_swarm()
        script = 'echo hi; printf "%s" "$BASH_EXECUTION_STRING"'
        done = self._run(m._wsl_argv(script))
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual(done.stdout, f"hi\n{script}")

    def test_a_semicolon_no_longer_splits_the_command(self):
        m = _load_swarm()
        done = self._run(m._wsl_argv("cd /tmp; pwd"))
        self.assertEqual(done.stdout.strip(), "/tmp")


if __name__ == "__main__":
    unittest.main()
