"""gcs_launch --sitl / --no-sitl: the launcher spawns the SITL swarm by default
and skips it with --no-sitl (backend + frontend still start either way)."""
import importlib.util
import pathlib
import sys
import unittest
from unittest.mock import MagicMock, patch

_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _load():
    spec = importlib.util.spec_from_file_location("gcs_launch", _ROOT / "scripts" / "gcs_launch.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _fake_entry(n=5):
    return {"chat_index": n, "frontend": 3000 + n, "backend": 8000 + n,
            "monitor": 15550 + n, "mission_planner": 14550 + n,
            "sysids": [3 * n + 1, 3 * n + 2, 3 * n + 3]}


def _run(argv, mod=None):
    m = mod or _load()
    spawn = MagicMock()
    with patch.object(m, "_spawn", spawn), \
         patch.object(m.reg, "claim", return_value=_fake_entry()), \
         patch.object(m.reg, "registered_pids", return_value=set()), \
         patch.object(m.reg, "reap_port", return_value=[]), \
         patch.object(m.reg, "reap_companion_ports", return_value=[]), \
         patch.object(m.swarm_run, "ensure_fork_supports_companion_udp"), \
         patch.object(m.reg, "record_pids"), patch.object(m.reg, "release"), \
         patch.object(m.reg, "owner_for", return_value="session:test"), \
         patch.object(m.webbrowser, "open"), \
         patch.object(sys, "argv", ["gcs_launch.py", *argv]):
        rc = m.main()
    return rc, spawn


def _swarm_cmd(spawn):
    """The argv gcs_launch used to spawn swarm_run.py, or None."""
    for c in spawn.call_args_list:
        cmd = c.args[0] if c.args else c.kwargs.get("cmd")
        if isinstance(cmd, list) and any("swarm_run.py" in str(x) for x in cmd):
            return [str(x) for x in cmd]
    return None


def _spawned_swarm(spawn):
    return _swarm_cmd(spawn) is not None


class TestGcsLaunchSitlFlag(unittest.TestCase):
    def test_default_starts_sitl(self):
        rc, spawn = _run(["--chat", "5", "--no-browser"])
        self.assertEqual(rc, 0)
        self.assertTrue(_spawned_swarm(spawn), "default should spawn swarm_run.py")

    def test_no_sitl_skips_swarm_but_keeps_gcs(self):
        rc, spawn = _run(["--chat", "5", "--no-browser", "--no-sitl"])
        self.assertEqual(rc, 0)
        self.assertFalse(_spawned_swarm(spawn), "--no-sitl must not spawn swarm_run.py")
        # backend + frontend still launched
        self.assertGreaterEqual(spawn.call_count, 2)


class TestGcsLaunchSpeedup(unittest.TestCase):
    """--speedup exists so a speed-sensitive run needs ONE command. Without it,
    gcs_launch always got swarm_run's default, which forced a two-step launch
    dance (gcs_launch --no-sitl, then swarm_run --speedup N) — and that dance is
    what produced a swarm silently running at 10x while 1x was requested."""

    def test_default_forwards_swarm_runs_own_default(self):
        m = _load()
        # A non-default value proves the parser really reads the constant. Against
        # the real 10 this would pass even with the default hard-coded here.
        with patch.object(m.swarm_run, "DEFAULT_SPEEDUP", 7):
            rc, spawn = _run(["--chat", "5", "--no-browser"], mod=m)
        cmd = _swarm_cmd(spawn)
        self.assertIn("--speedup", cmd)
        # One source of truth: the two launchers must not drift apart.
        self.assertEqual(cmd[cmd.index("--speedup") + 1], "7")

    def test_explicit_speedup_is_forwarded(self):
        rc, spawn = _run(["--chat", "5", "--no-browser", "--speedup", "1"])
        cmd = _swarm_cmd(spawn)
        self.assertEqual(cmd[cmd.index("--speedup") + 1], "1")

    def test_launch_token_is_forwarded_to_swarm(self):
        rc, spawn = _run([
            "--chat", "5", "--no-browser", "--launch-token", "exact-token",
        ])
        self.assertEqual(rc, 0)
        cmd = _swarm_cmd(spawn)
        self.assertEqual(cmd[cmd.index("--launch-token") + 1], "exact-token")

    def test_no_sitl_does_not_advertise_a_speed_it_is_not_launching(self):
        m = _load()
        with patch("builtins.print") as out:
            _run(["--chat", "5", "--no-browser", "--no-sitl", "--speedup", "1"])
        printed = " ".join(str(c.args[0]) for c in out.call_args_list if c.args)
        self.assertIn("--no-sitl", printed)
        self.assertNotIn("launching swarm", printed)

    def test_banner_states_the_requested_speed(self):
        with patch("builtins.print") as out:
            _run(["--chat", "5", "--no-browser", "--speedup", "3"])
        printed = " ".join(str(c.args[0]) for c in out.call_args_list if c.args)
        self.assertIn("at speedup 3", printed)

    def test_banner_names_only_the_sysids_that_actually_fly(self):
        """A partial launch must not advertise vehicles it never started."""
        with patch("builtins.print") as out:
            _run(["--chat", "5", "--no-browser", "--instances", "1"])
        printed = " ".join(str(c.args[0]) for c in out.call_args_list if c.args)
        self.assertIn("launching 1 at speedup", printed)
        self.assertIn("slot reserves", printed)

    def test_instance_count_reaches_the_swarm_launch(self):
        _rc, spawn = _run(["--chat", "5", "--no-browser", "--instances", "2"])
        cmd = _swarm_cmd(spawn)
        self.assertIsNotNone(cmd, "swarm_run was not spawned")
        self.assertIn("--instances", cmd)
        self.assertEqual(cmd[cmd.index("--instances") + 1], "2")

    def test_home_reaches_the_swarm_launch(self):
        """Home is fixed at SITL start; a mission upload cannot move it."""
        _rc, spawn = _run([
            "--chat", "5", "--no-browser",
            "--home", "40.3120025,44.4554112,1294.86,0.0",
        ])
        cmd = _swarm_cmd(spawn)
        self.assertIsNotNone(cmd, "swarm_run was not spawned")
        self.assertIn("--home", cmd)
        self.assertEqual(
            cmd[cmd.index("--home") + 1], "40.3120025,44.4554112,1294.86,0.0"
        )

    def test_home_is_omitted_when_not_requested(self):
        _rc, spawn = _run(["--chat", "5", "--no-browser"])
        cmd = _swarm_cmd(spawn)
        self.assertIsNotNone(cmd, "swarm_run was not spawned")
        self.assertNotIn("--home", cmd)

    def test_instance_count_out_of_range_is_refused(self):
        for bad in ("0", "4"):
            with self.subTest(instances=bad):
                rc, spawn = _run(["--chat", "5", "--no-browser", "--instances", bad])
                self.assertEqual(rc, 2)
                self.assertIsNone(_swarm_cmd(spawn))


class TestSitlStatusRendering(unittest.TestCase):
    """--list is where a failed swarm launch survives: swarm_run's console closes
    with it, taking the abort message along."""

    def _render(self, **fields):
        m = _load()
        return m._sitl_status({"sitl": True, **fields})

    def test_no_sitl(self):
        m = _load()
        self.assertEqual(m._sitl_status({"sitl": False}), "no")

    def test_pending_while_verifying(self):
        self.assertEqual(self._render(sitl_verified=None), "yes (verifying)")

    def test_legacy_unknown_is_not_shown_as_in_flight(self):
        # "we never looked" must not read as "we are looking" — the point of the
        # field is durable evidence.
        self.assertNotEqual(self._render(), self._render(sitl_verified=None))
        self.assertIn("unknown", self._render())

    def test_verified_shows_the_achieved_speed(self):
        self.assertEqual(self._render(sitl_verified=True, sitl_speedup=1),
                         "yes speedup=1 verified")

    def test_failure_is_loud_and_carries_the_reason(self):
        text = self._render(sitl_verified=False, sitl_speedup=1,
                            sitl_error="SIM_SPEEDUP mismatch (requested 1): sys_id 1=10")
        self.assertIn("FAILED VERIFICATION", text)
        self.assertIn("sys_id 1=10", text)

    def test_retracted_failure_still_names_the_reason(self):
        # retract_sitl clears the flag but keeps the verdict. If --list rendered
        # a bare "no" here, the only durable record of the failure would be gone
        # — swarm_run's console already closed with the abort message.
        m = _load()
        text = m._sitl_status({"sitl": False, "sitl_verified": False,
                               "sitl_speedup": 1,
                               "sitl_error": "sys_ids [1] never streamed"})
        self.assertIn("FAILED VERIFICATION", text)
        self.assertIn("never streamed", text)
        self.assertFalse(text.startswith("yes"))  # and not as a live swarm

    def test_retracted_verified_launch_is_not_launch_history(self):
        # --list is a liveness view; a swarm that came up fine and is now gone
        # reads as plain "no". Only the failure earns a line.
        m = _load()
        self.assertEqual(
            m._sitl_status({"sitl": False, "sitl_verified": True, "sitl_speedup": 1}),
            "no")

    def test_legacy_entry_without_the_fields_still_renders(self):
        # Entries written before this evidence existed must not crash --list or
        # be reported as failed.
        self.assertEqual(self._render(), "yes (verification unknown - legacy entry)")


class TestGcsLaunchPidRecording(unittest.TestCase):
    def test_backend_pid_recorded_before_frontend_spawn(self):
        # If the launcher dies between the backend and frontend spawns, the
        # launch guard can only see the backend process if its pid was already
        # persisted — so the record must happen immediately after the spawn.
        m = _load()
        tracker = MagicMock()
        with patch.object(m, "_spawn", tracker.spawn), \
             patch.object(m.reg, "claim", return_value=_fake_entry()), \
             patch.object(m.reg, "registered_pids", return_value=set()), \
             patch.object(m.reg, "reap_port", return_value=[]), \
             patch.object(m.reg, "reap_companion_ports", return_value=[]), \
             patch.object(m.reg, "record_pids", tracker.record_pids), \
             patch.object(m.reg, "release"), \
             patch.object(m.reg, "owner_for", return_value="session:test"), \
             patch.object(m.webbrowser, "open"), \
             patch.object(sys, "argv", ["gcs_launch.py", "--no-browser", "--no-sitl"]):
            rc = m.main()
        self.assertEqual(rc, 0)
        calls = tracker.mock_calls
        backend_recorded_at = next(
            i for i, c in enumerate(calls)
            if c[0] == "record_pids" and "backend_pid" in c[2])
        # --no-sitl: first spawn is the backend, second is the frontend.
        frontend_spawned_at = [i for i, c in enumerate(calls) if c[0] == "spawn"][1]
        self.assertLess(backend_recorded_at, frontend_spawned_at)


class TestGcsLaunchCompanionUdp(unittest.TestCase):
    def test_reaps_companion_udp_ports_before_spawning(self):
        m = _load()
        with patch.object(m, "_spawn", MagicMock()), \
             patch.object(m.reg, "claim", return_value=_fake_entry()), \
             patch.object(m.reg, "registered_pids", return_value={9}), \
             patch.object(m.reg, "reap_port", return_value=[]), \
             patch.object(m.reg, "reap_companion_ports",
                          return_value=[]) as mcomp, \
             patch.object(m.swarm_run, "ensure_fork_supports_companion_udp"), \
             patch.object(m.reg, "record_pids"), patch.object(m.reg, "release"), \
             patch.object(m.reg, "owner_for", return_value="session:test"), \
             patch.object(m.webbrowser, "open"), \
             patch.object(sys, "argv",
                          ["gcs_launch.py", "--chat", "5", "--no-browser"]):
            self.assertEqual(m.main(), 0)
        mcomp.assert_called_once_with(5, exclude={9})

    def test_fork_skew_aborts_launch_and_releases_slot(self):
        # An outdated fork script must fail the launch LOUDLY in the
        # launcher's own console (swarm_run's new console vanishes on exit)
        # and must not leave a phantom slot reservation behind.
        m = _load()
        spawn = MagicMock()
        with patch.object(m, "_spawn", spawn), \
             patch.object(m.reg, "claim", return_value=_fake_entry()), \
             patch.object(m.reg, "registered_pids", return_value=set()), \
             patch.object(m.reg, "reap_port", return_value=[]), \
             patch.object(m.reg, "reap_companion_ports", return_value=[]), \
             patch.object(m.swarm_run, "ensure_fork_supports_companion_udp",
                          side_effect=SystemExit("fork predates COMPANION_UDP")), \
             patch.object(m.reg, "record_pids"), \
             patch.object(m.reg, "release") as release, \
             patch.object(m.reg, "owner_for", return_value="session:test"), \
             patch.object(m.webbrowser, "open"), \
             patch.object(sys, "argv",
                          ["gcs_launch.py", "--chat", "5", "--no-browser"]):
            with self.assertRaises(SystemExit):
                m.main()
        spawn.assert_not_called()
        release.assert_called_once_with(5)


class TestGcsLaunchGuard(unittest.TestCase):
    def test_busy_slot_refuses_to_spawn_a_duplicate_stack(self):
        # claim() raising SlotBusyError = this directory's previous launch is
        # still alive/spawning. The launcher must exit cleanly WITHOUT spawning
        # anything (duplicate stacks double-bind the monitor port and duel over
        # SITL serial0 — vehicles drop and companions can't start).
        m = _load()
        spawn = MagicMock()
        with patch.object(m, "_spawn", spawn), \
             patch.object(m.reg, "claim",
                          side_effect=m.reg.SlotBusyError(_fake_entry())), \
             patch.object(m.reg, "owner_for", return_value="session:test"), \
             patch.object(m.webbrowser, "open") as browser, \
             patch.object(sys, "argv", ["gcs_launch.py", "--no-browser"]):
            rc = m.main()
        self.assertEqual(rc, 0)
        spawn.assert_not_called()
        browser.assert_not_called()

    def test_busy_slot_still_opens_browser_without_no_browser(self):
        m = _load()
        entry = _fake_entry()
        with patch.object(m, "_spawn", MagicMock()) as spawn, \
             patch.object(m.reg, "claim", side_effect=m.reg.SlotBusyError(entry)), \
             patch.object(m.reg, "owner_for", return_value="session:test"), \
             patch.object(m.webbrowser, "open") as browser, \
             patch.object(sys, "argv", ["gcs_launch.py"]):
            rc = m.main()
        self.assertEqual(rc, 0)
        spawn.assert_not_called()
        browser.assert_called_once_with(f"http://localhost:{entry['frontend']}")


if __name__ == "__main__":
    unittest.main()
