"""swarm_run launches SITL with COMPANION_UDP=1 and refuses an outdated fork.

The companion transport is a dedicated per-vehicle UDP link: SITL serial0 runs
as a udpclient toward the Windows-side companion bind. run_swarm.sh gates that
on COMPANION_UDP so older branches (which never set it) keep their TCP serial0.
Version skew in the other direction — this branch against a fork script that
predates the gate — would strand every companion on a silent UDP port, so the
launch must abort loudly instead.
"""
import importlib.util
import pathlib
import sys
import unittest
from unittest.mock import MagicMock, patch

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load_swarm():
    spec = importlib.util.spec_from_file_location(
        "swarm_run_companion_udp", _ROOT / "scripts" / "swarm_run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class TestForkCapabilityPreflight(unittest.TestCase):
    def test_aborts_when_fork_script_lacks_companion_udp(self):
        m = _load_swarm()
        with patch.object(m.subprocess, "run",
                          return_value=MagicMock(returncode=1)):
            with self.assertRaises(SystemExit) as ctx:
                m.ensure_fork_supports_companion_udp()
        self.assertIn("COMPANION_UDP", str(ctx.exception))

    def test_passes_when_fork_script_knows_companion_udp(self):
        m = _load_swarm()
        with patch.object(m.subprocess, "run",
                          return_value=MagicMock(returncode=0)):
            m.ensure_fork_supports_companion_udp()  # must not raise

    def test_preflight_runs_before_slot_claim(self):
        # Exiting after the claim leaks the slot (no atexit yet) and lets the
        # eval harness read the registry entry as launch success — the check
        # must come first. Assert the ordering structurally.
        source = (_ROOT / "scripts" / "swarm_run.py").read_text(encoding="utf-8")
        body = source[source.index("def main()"):]
        self.assertLess(body.index("ensure_fork_supports_companion_udp()"),
                        body.index("_resolve_chat("))


class TestLaunchCommandTransport(unittest.TestCase):
    def test_swarm_command_sets_companion_udp(self):
        # The gate must ride the same inline env prefix as SPEEDUP/SWARM_OFFSET
        # (run_swarm.sh reads its env from the bash command string, not the
        # Windows process environment).
        #
        # Asserts on the command the launcher actually builds, not on the source
        # text: a literal match broke the moment the builder was refactored, while
        # saying nothing about whether the gate still reaches run_swarm.sh.
        m = _load_swarm()
        cmd = m._launch_command(10, 0, "14550,15550,16550", 3, 50)
        prefix = cmd.split("~/ardupilot")[0]
        self.assertIn("COMPANION_UDP=1", prefix)
        self.assertIn("SPEEDUP=10", prefix)
        self.assertIn("SWARM_OFFSET=0", prefix)

    def test_keeping_instance_state_is_opt_in(self):
        # CLONE_FROM_TEMPLATE=0 defeats run_swarm.sh's anti-drift sync, so it must
        # appear only when a caller asks for it — see _launch_command.
        m = _load_swarm()
        self.assertNotIn("CLONE_FROM_TEMPLATE",
                         m._launch_command(1, 0, "p", 3, 50))
        self.assertIn("CLONE_FROM_TEMPLATE=0",
                      m._launch_command(1, 0, "p", 3, 50,
                                        keep_instance_state=True))

    def test_fractional_request_boots_at_supported_floor_for_live_healing(self):
        m = _load_swarm()

        command = m._launch_command(0.5, 0, "p", 1, 50)

        self.assertIn("SPEEDUP=1 ", command)

    def test_home_coords_reach_run_swarm_as_inline_env(self):
        m = _load_swarm()

        command = m._launch_command(
            1, 0, "p", 1, 50, home_coords="40.3120025,44.4554112,1294.86,0.0"
        )

        prefix = command.split("~/ardupilot")[0]
        self.assertIn("HOME_COORDS=40.3120025,44.4554112,1294.86,0.0", prefix)

    def test_home_coords_cannot_smuggle_a_second_command(self):
        # The command string is handed to `bash -lc`, so an unquoted value
        # carrying `;` would run rather than assign.
        m = _load_swarm()

        command = m._launch_command(
            1, 0, "p", 1, 50, home_coords="x; echo INJECTED"
        )

        # The hazard is an unquoted `;` ending the assignment; inside single
        # quotes the same characters are just data.
        self.assertNotIn("HOME_COORDS=x;", command)
        self.assertIn("HOME_COORDS='x; echo INJECTED'", command)

    def test_home_argument_refuses_a_non_coordinate(self):
        import scripts.swarm_run_runner as runner

        parser = runner.build_parser(1.0)
        for bad in ("x; echo INJECTED", "1,2,3", "a,b,c,d", "999,0,0,0"):
            with self.subTest(home=bad):
                with self.assertRaises(SystemExit):
                    parser.parse_args(["--home", bad])

    def test_home_argument_canonicalises_a_valid_coordinate(self):
        import scripts.swarm_run_runner as runner

        parsed = runner.build_parser(1.0).parse_args(
            ["--home", "40.3120025,44.4554112,1294.86,0.0"]
        )

        self.assertEqual(parsed.home, "40.3120025,44.4554112,1294.86,0.0")


if __name__ == "__main__":
    unittest.main()
