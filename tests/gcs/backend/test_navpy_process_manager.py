"""Tests for NavpyProcessManager."""
import json
import subprocess
import sys
import unittest
from contextlib import redirect_stdout
from collections import deque
from io import StringIO
from unittest.mock import MagicMock, patch

from gcs.backend.navpy_process_manager import (
    NavpyInstance,
    NavpyProcessManager,
    _build_env,
    _foreign_udp_holders,
    _normalize_log_level,
)
from gcs.backend.navpy_process_instance import terminate_navpy_instance
from navpy.runtime_ready import NAVPY_RUNTIME_READY_MARKER, emit_runtime_ready


class TestNavpyProcessManager(unittest.TestCase):
    """Unit tests for NavpyProcessManager with mocked subprocess."""

    def setUp(self):
        self.mgr = NavpyProcessManager()

    def test_runtime_marker_includes_emitting_python_pid(self):
        output = StringIO()
        with patch("navpy.runtime_ready.os.getpid", return_value=4321), \
             redirect_stdout(output):
            emit_runtime_ready({"mission_items": 10})

        self.assertEqual(
            output.getvalue(),
            f'{NAVPY_RUNTIME_READY_MARKER} '
            '{"mission_items":10,"process_pid":4321}\n',
        )

    def test_exact_runtime_marker_sets_generation_readiness(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        proc.stdout.readline.side_effect = [
            (
                f'{NAVPY_RUNTIME_READY_MARKER} '
                '{"mission_items":10,"targ_wps":7,"nav_last_wp":2}\n'
            ).encode(),
            b"",
        ]
        inst = NavpyInstance(1, "tcp:127.0.0.1:14550", proc)
        self.mgr._instances[1] = inst

        self.mgr._read_output(inst)

        self.assertEqual(
            self.mgr.wait_ready(1, 1001, 0),
            {"mission_items": 10, "targ_wps": 7, "nav_last_wp": 2},
        )
        self.assertTrue(self.mgr.get_status(1)["ready"])

    def test_near_match_marker_does_not_set_readiness(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        proc.stdout.readline.side_effect = [
            f'{NAVPY_RUNTIME_READY_MARKER}_OLD {{}}\n'.encode(),
            b"",
        ]
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        self.mgr._instances[1] = inst

        self.mgr._read_output(inst)

        with self.assertRaisesRegex(RuntimeError, "before readiness"):
            self.mgr.wait_ready(1, 1001, 0)
        self.assertFalse(self.mgr.get_status(1)["ready"])

    def test_exited_process_never_snapshots_ready(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = 1
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        inst.ready_payload = {"mission_items": 10}
        inst._ready_or_exit.set()
        self.mgr._instances[1] = inst

        self.assertFalse(self.mgr.get_status(1)["ready"])
        with self.assertRaisesRegex(RuntimeError, "exited after readiness"):
            self.mgr.wait_ready(1, 1001, 0)

    def _mock_start(
            self,
            sys_id,
            connection="udp:0.0.0.0:14550",
            detector_debug_show=False,
            vision_profile=None,
    ):
        """Start an instance with a mocked Popen.

        The mocked process has a fake _handle so _assign_job fails
        gracefully, leaving _job_handle=None and the terminate path
        falling through to proc.terminate().
        """
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen, \
             patch("gcs.backend.navpy_process_manager._foreign_udp_holders",
                   return_value=[]):
            proc = MagicMock()
            proc.poll.return_value = None  # running
            proc.pid = 1000 + sys_id
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""  # empty → reader exits
            mock_popen.return_value = proc
            inst = self.mgr.start(
                sys_id,
                connection,
                detector_debug_show=detector_debug_show,
                vision_profile=vision_profile,
            )
            # Job assignment fails on mocked handle → falls back to terminate()
            self.assertIsNone(inst._job_handle)
            return inst, proc

    def test_start_creates_tracked_instance(self):
        inst, proc = self._mock_start(1)
        self.assertEqual(inst.sys_id, 1)
        self.assertEqual(inst.process, proc)
        status = self.mgr.get_status(1)
        self.assertIsNotNone(status)
        self.assertEqual(status["sys_id"], 1)
        self.assertTrue(status["running"])

    def test_start_refuses_udp_port_held_by_foreign_pid(self):
        # SO_REUSEADDR means a double-bind would not error — the two
        # companions would silently split the stream. Must refuse instead.
        with patch("gcs.backend.navpy_process_manager._foreign_udp_holders",
                   return_value=[4242]):
            with self.assertRaises(ValueError) as ctx:
                self.mgr.start(1, "udp:0.0.0.0:5760")
        self.assertIn("4242", str(ctx.exception))
        self.assertIsNone(self.mgr.get_status(1))

    def test_foreign_udp_holders_only_checks_local_udp_binds(self):
        # Non-bind connections (serial, tcp, udpout) have no local UDP port
        # to collide on and must never trigger a probe.
        with patch("gcs.backend.instance_registry.udp_port_held") as probe:
            self.assertEqual(_foreign_udp_holders("tcp:127.0.0.1:5760"), [])
            self.assertEqual(_foreign_udp_holders("udpout:10.0.0.5:5760"), [])
            self.assertEqual(_foreign_udp_holders("COM7"), [])
            probe.assert_not_called()

    def test_foreign_udp_holders_matches_udpin_and_any_host(self):
        # pymavlink "udp:"/"udpin:" are ALWAYS local server binds regardless
        # of the host part — the guard must cover them all, or a preset like
        # udpin:0.0.0.0:14560 silently skips squatter detection.
        with patch("gcs.backend.instance_registry.udp_port_held",
                   return_value=True), \
             patch("gcs.backend.instance_registry.listeners_on_port",
                   return_value=[555]):
            self.assertEqual(_foreign_udp_holders("udpin:0.0.0.0:14560"), [555])
            self.assertEqual(_foreign_udp_holders("udp:192.168.1.7:5760"), [555])

    def test_foreign_udp_holders_skips_netstat_when_port_free(self):
        # The common path must stay a microsecond bind probe — netstat runs
        # only to NAME an actual holder.
        with patch("gcs.backend.instance_registry.udp_port_held",
                   return_value=False), \
             patch("gcs.backend.instance_registry.listeners_on_port") as netstat:
            self.assertEqual(_foreign_udp_holders("udp:0.0.0.0:5760"), [])
            netstat.assert_not_called()

    def test_foreign_udp_holders_excludes_own_pid(self):
        import os
        with patch("gcs.backend.instance_registry.udp_port_held",
                   return_value=True), \
             patch("gcs.backend.instance_registry.listeners_on_port",
                   return_value=[os.getpid(), 555]):
            self.assertEqual(_foreign_udp_holders("udp:0.0.0.0:5760"), [555])

    def test_foreign_udp_holders_reports_unnamed_holder(self):
        # Probe sees a holder but netstat can't name it (race / non-Windows):
        # still refuse rather than silently double-bind.
        with patch("gcs.backend.instance_registry.udp_port_held",
                   return_value=True), \
             patch("gcs.backend.instance_registry.listeners_on_port",
                   return_value=[]):
            self.assertEqual(_foreign_udp_holders("udp:0.0.0.0:5760"), [-1])

    def test_foreign_udp_holders_accepts_port_released_during_pid_lookup(self):
        # The bind probe and netstat are not one atomic operation. A companion
        # may finish releasing the port between them; recheck before turning an
        # empty netstat result into the synthetic unknown PID.
        with patch(
            "gcs.backend.instance_registry.udp_port_held",
            side_effect=[True, False],
        ) as probe, patch(
            "gcs.backend.instance_registry.listeners_on_port",
            return_value=[],
        ):
            self.assertEqual(_foreign_udp_holders("udp:0.0.0.0:5760"), [])

        self.assertEqual(probe.call_count, 2)

    def test_foreign_udp_holders_accepts_named_holder_released_during_lookup(self):
        with patch(
            "gcs.backend.instance_registry.udp_port_held",
            side_effect=[True, False],
        ) as probe, patch(
            "gcs.backend.instance_registry.listeners_on_port",
            return_value=[555],
        ):
            self.assertEqual(_foreign_udp_holders("udp:0.0.0.0:5760"), [])

        self.assertEqual(probe.call_count, 2)

    def test_ready_udp_collision_terminates_generation_and_rejects_readiness(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        proc.stdout.readline.side_effect = [
            (
                f'{NAVPY_RUNTIME_READY_MARKER} '
                '{"mission_items":10,"targ_wps":7,"nav_last_wp":2,'
                '"process_pid":2002}\n'
            ).encode(),
            b"",
        ]
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        self.mgr._instances[1] = inst

        with patch(
            "gcs.backend.instance_registry.listeners_on_port",
            return_value=[2002, 4242],
        ):
            self.mgr._read_output(inst)

        proc.terminate.assert_called_once_with()
        self.assertIsNone(inst.ready_payload)
        with self.assertRaisesRegex(RuntimeError, "shared.*4242"):
            self.mgr.wait_ready(1, 1001, 0)

    def test_ready_udp_accepts_runtime_child_of_launcher_as_sole_owner(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        proc.stdout.readline.side_effect = [
            (
                f'{NAVPY_RUNTIME_READY_MARKER} '
                '{"mission_items":10,"targ_wps":7,"nav_last_wp":2,'
                '"process_pid":2002}\n'
            ).encode(),
            b"",
        ]
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        self.mgr._instances[1] = inst

        with patch(
            "gcs.backend.instance_registry.listeners_on_port",
            return_value=[2002],
        ):
            self.mgr._read_output(inst)

        self.assertEqual(self.mgr.wait_ready(1, 1001, 0)["mission_items"], 10)
        proc.terminate.assert_not_called()

    def test_ready_udp_rejects_missing_or_invalid_runtime_pid(self):
        invalid_payloads = ({}, {"process_pid": True}, {"process_pid": 0},
                            {"process_pid": -1})
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                proc = MagicMock()
                proc.pid = 1001
                proc.poll.return_value = None
                proc.stdout.readline.side_effect = [
                    (
                        f"{NAVPY_RUNTIME_READY_MARKER} "
                        f"{json.dumps(payload)}\n"
                    ).encode(),
                    b"",
                ]
                inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
                self.mgr._instances[1] = inst

                self.mgr._read_output(inst)

                proc.terminate.assert_called_once_with()
                with self.assertRaisesRegex(RuntimeError, "valid process_pid"):
                    self.mgr.wait_ready(1, 1001, 0)

    def test_start_omits_detector_debug_flag_by_default(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550")

            cmd = mock_popen.call_args.args[0]
            self.assertNotIn("--detector-debug-show", cmd)

    def test_start_passes_only_the_plain_sysid_identity(self):
        """The companion shares its aircraft's sysid (component 191 tells them
        apart), so -ss is the only identity argument -- the removed
        --mav-source-system flag must not be resurrected."""
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1121
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(121, "tcp:127.0.0.1:6960")

            cmd = mock_popen.call_args.args[0]
            self.assertNotIn("--mav-source-system", cmd)
            idx = cmd.index("-ss")
            self.assertEqual(cmd[idx + 1], "121")

    def test_start_leaves_navigation_speedup_disabled_by_default(self):
        """The GCS must preserve NavArgs' disabled default unless opted in."""
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550")

            cmd = mock_popen.call_args.args[0]
            self.assertNotIn("-gsu", cmd)

    def test_start_adds_positive_navigation_speedup_override(self):
        with patch(
            "gcs.backend.navpy_process_manager._GCS_SIM_NAVIGATION_SPEEDUP", 2.0,
        ), patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550")

            cmd = mock_popen.call_args.args[0]
            idx = cmd.index("-gsu")
            self.assertEqual(cmd[idx + 1], "2.0")

    def test_start_omits_vision_profile_when_unset(self):
        for vision_profile in (None, "", "   "):
            with self.subTest(vision_profile=vision_profile):
                mgr = NavpyProcessManager()
                with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
                    proc = MagicMock()
                    proc.poll.return_value = None
                    proc.pid = 1001
                    proc.stdout = MagicMock()
                    proc.stdout.readline.return_value = b""
                    mock_popen.return_value = proc

                    mgr.start(
                        1,
                        "udp:0.0.0.0:14550",
                        vision_profile=vision_profile,
                    )

                    cmd = mock_popen.call_args.args[0]
                    self.assertNotIn("--vision-profile", cmd)

    def test_start_adds_vision_profile_when_selected(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(
                1,
                "udp:0.0.0.0:14550",
                vision_profile="  novoxy_dual  ",
            )

            cmd = mock_popen.call_args.args[0]
            idx = cmd.index("--vision-profile")
            self.assertEqual(cmd[idx + 1], "novoxy_dual")

    def test_start_adds_detector_debug_flag_when_enabled(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(
                1,
                "udp:0.0.0.0:14550",
                detector_debug_show=True,
            )

            cmd = mock_popen.call_args.args[0]
            self.assertIn("--detector-debug-show", cmd)

    def test_start_omits_log_level_by_default(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550")

            cmd = mock_popen.call_args.args[0]
            self.assertNotIn("--log-level", cmd)

    def test_start_forwards_normalized_log_level(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550", log_level="debug")

            cmd = mock_popen.call_args.args[0]
            idx = cmd.index("--log-level")
            # Case-normalized to the canonical CacheLogLevel name.
            self.assertEqual(cmd[idx + 1], "DEBUG")

    def test_start_ignores_invalid_log_level(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            # Invalid level must not block launch — it is dropped, not forwarded.
            self.mgr.start(1, "udp:0.0.0.0:14550", log_level="bogus")

            cmd = mock_popen.call_args.args[0]
            self.assertNotIn("--log-level", cmd)

    def test_start_duplicate_raises(self):
        self._mock_start(1)
        with self.assertRaises(ValueError):
            self._mock_start(1)

    def test_start_replaces_exited_instance(self):
        inst1, proc1 = self._mock_start(1)
        # Simulate the process having exited
        proc1.poll.return_value = 0
        # Starting again should succeed
        inst2, proc2 = self._mock_start(1)
        self.assertIsNot(inst1, inst2)

    def test_stop_terminates_process(self):
        inst, proc = self._mock_start(1)
        result = self.mgr.stop(1)
        self.assertTrue(result)
        proc.terminate.assert_called_once()
        self.assertIsNone(self.mgr.get_status(1))

    def test_stop_failure_keeps_generation_tracked_for_retry(self):
        inst, proc = self._mock_start(1)
        failure = OSError("terminate failed")
        proc.terminate.side_effect = failure

        with self.assertRaises(OSError) as raised:
            self.mgr.stop(1)

        self.assertIs(raised.exception, failure)
        self.assertIs(self.mgr._instances[1], inst)

    def test_stop_does_not_remove_concurrently_installed_generation(self):
        old, _proc = self._mock_start(1)
        replacement_proc = MagicMock()
        replacement_proc.pid = 2001
        replacement_proc.poll.return_value = None
        replacement = NavpyInstance(
            1,
            "udp:0.0.0.0:14550",
            replacement_proc,
        )

        def terminate(_inst):
            self.assertIs(_inst, old)
            self.mgr._instances[1] = replacement

        with patch.object(self.mgr, "_terminate", side_effect=terminate):
            self.assertTrue(self.mgr.stop(1))

        self.assertIs(self.mgr._instances[1], replacement)

    def test_stop_generation_requires_exact_pid(self):
        inst, proc = self._mock_start(1)
        runtime_pid = proc.pid + 1
        inst.ready_payload = {"process_pid": runtime_pid}

        self.assertFalse(self.mgr.stop_generation(1, runtime_pid))
        proc.terminate.assert_not_called()
        self.assertIs(self.mgr._instances[1], inst)

        self.assertTrue(self.mgr.stop_generation(1, proc.pid))
        proc.terminate.assert_called_once_with()
        self.assertNotIn(1, self.mgr._instances)

    def test_output_reader_error_is_reported_as_startup_error(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        proc.stdout.readline.side_effect = OSError("pipe failed")
        inst = NavpyInstance(1, "tcp:127.0.0.1:14550", proc)
        self.mgr._instances[1] = inst

        self.mgr._read_output(inst)

        with self.assertRaisesRegex(RuntimeError, "output reader failed.*pipe failed"):
            self.mgr.wait_ready(1, 1001, 0)
        self.assertIn("pipe failed", self.mgr.get_status(1)["startup_error"])

    def test_reader_start_failure_terminates_unregistered_process(self):
        proc = MagicMock()
        proc.poll.return_value = None
        proc.pid = 1001
        proc.stdout = MagicMock()
        thread = MagicMock()
        failure = RuntimeError("reader start failed")
        thread.start.side_effect = failure

        with patch(
            "gcs.backend.navpy_process_manager.subprocess.Popen",
            return_value=proc,
        ), patch(
            "gcs.backend.navpy_process_manager._foreign_udp_holders",
            return_value=[],
        ), patch(
            "gcs.backend.navpy_process_launch.threading.Thread",
            return_value=thread,
        ):
            with self.assertRaises(RuntimeError) as raised:
                self.mgr.start(1, "udp:0.0.0.0:14550")

        self.assertIs(raised.exception, failure)
        proc.terminate.assert_called_once_with()
        proc.stdout.close.assert_called_once_with()
        self.assertIsNone(self.mgr.get_status(1))

    def test_reader_thread_construction_failure_cleans_unregistered_process(self):
        proc = MagicMock()
        proc.poll.return_value = None
        proc.pid = 1001
        proc.stdout = MagicMock()
        failure = RuntimeError("reader construction failed")

        with patch(
            "gcs.backend.navpy_process_manager.subprocess.Popen",
            return_value=proc,
        ), patch(
            "gcs.backend.navpy_process_manager._foreign_udp_holders",
            return_value=[],
        ), patch(
            "gcs.backend.navpy_process_launch.threading.Thread",
            side_effect=failure,
        ):
            with self.assertRaises(RuntimeError) as raised:
                self.mgr.start(1, "udp:0.0.0.0:14550")

        self.assertIs(raised.exception, failure)
        proc.terminate.assert_called_once_with()
        proc.stdout.close.assert_called_once_with()
        self.assertIsNone(self.mgr.get_status(1))

    @unittest.skipUnless(sys.platform == "win32", "Windows Job Object only")
    def test_job_timeout_kills_then_reaps_process(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.wait.side_effect = [
            subprocess.TimeoutExpired("navpy", 5),
            None,
        ]
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        inst._job_handle = 77

        with patch(
            "gcs.backend.navpy_process_instance._terminate_job",
        ) as terminate_job:
            terminate_navpy_instance(inst, MagicMock())

        terminate_job.assert_called_once_with(77)
        self.assertIsNone(inst._job_handle)
        self.assertEqual(
            proc.wait.call_args_list,
            [unittest.mock.call(timeout=5), unittest.mock.call(timeout=2)],
        )
        proc.kill.assert_called_once_with()

    @unittest.skipUnless(sys.platform == "win32", "Windows Job Object only")
    def test_job_termination_failure_retains_handle_and_generation(self):
        proc = MagicMock()
        proc.pid = 1001
        proc.poll.return_value = None
        inst = NavpyInstance(1, "udp:0.0.0.0:14550", proc)
        inst._job_handle = 77
        self.mgr._instances[1] = inst
        failure = OSError("job termination failed")

        with patch(
            "gcs.backend.navpy_process_instance._terminate_job",
            side_effect=failure,
        ):
            with self.assertRaises(OSError) as raised:
                self.mgr.stop(1)

        self.assertIs(raised.exception, failure)
        self.assertEqual(inst._job_handle, 77)
        self.assertIs(self.mgr._instances[1], inst)

    @unittest.skipUnless(sys.platform == "win32", "Windows Job Object only")
    def test_spawn_failure_closes_precreated_job_handle(self):
        failure = OSError("spawn failed")
        with patch(
            "gcs.backend.navpy_process_manager.subprocess.Popen",
            side_effect=failure,
        ), patch(
            "gcs.backend.navpy_process_manager._foreign_udp_holders",
            return_value=[],
        ), patch(
            "gcs.backend.navpy_process_launch._create_job",
            return_value=77,
        ), patch(
            "gcs.backend.navpy_process_launch._close_job",
        ) as close_job:
            with self.assertRaises(OSError) as raised:
                self.mgr.start(1, "udp:0.0.0.0:14550")

        self.assertIs(raised.exception, failure)
        close_job.assert_called_once_with(77)
        self.assertIsNone(self.mgr.get_status(1))

    def test_stop_nonexistent_returns_false(self):
        result = self.mgr.stop(99)
        self.assertFalse(result)

    def test_stop_all_terminates_all(self):
        _, proc1 = self._mock_start(1)
        _, proc2 = self._mock_start(2)
        self.mgr.stop_all()
        proc1.terminate.assert_called_once()
        proc2.terminate.assert_called_once()
        self.assertEqual(self.mgr.get_all_status(), [])

    def test_stop_all_continues_and_retains_only_failed_instance(self):
        # A termination that raises partway must not abort the loop: every
        # remaining instance is still terminated. The successfully-stopped one is
        # dropped; the one whose termination raised is kept tracked (it may still
        # be alive) so status stays truthful rather than falsely reporting it gone.
        _, proc1 = self._mock_start(1)
        _, proc2 = self._mock_start(2)
        proc1.terminate.side_effect = OSError("terminate failed")

        # Must not propagate the failure.
        self.mgr.stop_all()

        proc1.terminate.assert_called_once()
        proc2.terminate.assert_called_once()
        # sys_id 1 failed -> retained; sys_id 2 stopped -> dropped.
        statuses = self.mgr.get_all_status()
        self.assertEqual([s["sys_id"] for s in statuses], [1])

    def test_get_all_status_returns_snapshots(self):
        self._mock_start(1)
        self._mock_start(2)
        statuses = self.mgr.get_all_status()
        self.assertEqual(len(statuses), 2)
        ids = {s["sys_id"] for s in statuses}
        self.assertEqual(ids, {1, 2})

    def test_get_status_nonexistent_returns_none(self):
        self.assertIsNone(self.mgr.get_status(42))

    def test_build_env_enables_utf8_mode(self):
        # PYTHONUTF8=1 makes the spawned NavPy emit UTF-8 on stdout/stderr so
        # log lines with non-cp1252 chars ('→') don't crash the console handler.
        env = _build_env()
        self.assertEqual(env.get("PYTHONUTF8"), "1")

    def test_start_passes_utf8_env_to_subprocess(self):
        with patch("gcs.backend.navpy_process_manager.subprocess.Popen") as mock_popen:
            proc = MagicMock()
            proc.poll.return_value = None
            proc.pid = 1001
            proc.stdout = MagicMock()
            proc.stdout.readline.return_value = b""
            mock_popen.return_value = proc

            self.mgr.start(1, "udp:0.0.0.0:14550")

            env = mock_popen.call_args.kwargs["env"]
            self.assertEqual(env.get("PYTHONUTF8"), "1")


class TestNormalizeLogLevel(unittest.TestCase):
    """Unit tests for the _normalize_log_level helper."""

    def test_maps_case_insensitive_name_to_canonical(self):
        self.assertEqual(_normalize_log_level("debug"), "DEBUG")
        self.assertEqual(_normalize_log_level("  Info  "), "INFO")

    def test_none_and_blank_return_none(self):
        self.assertIsNone(_normalize_log_level(None))
        self.assertIsNone(_normalize_log_level(""))
        self.assertIsNone(_normalize_log_level("   "))

    def test_invalid_name_returns_none(self):
        self.assertIsNone(_normalize_log_level("bogus"))


if __name__ == "__main__":
    unittest.main()
