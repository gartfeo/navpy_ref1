import threading
import unittest
from unittest.mock import Mock, patch

from navpy.core.kernel.restart_budget import RestartBudget
from navpy.core.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.core.swarm.swarm_presence import SwarmPresence


class _StuckThread:
    def __init__(self, *, target, daemon):
        self.target = target
        self.daemon = daemon
        self.join_timeout = None

    def start(self):
        return None

    def join(self, timeout=None):
        self.join_timeout = timeout

    def is_alive(self):
        return True


class SwarmHeartbeatRuntimeTest(unittest.TestCase):
    @staticmethod
    def _runtime():
        return SwarmHeartbeatRuntime(Mock(spec=SwarmPresence))

    def test_restart_uses_a_new_stop_generation_without_reviving_old_loop(self):
        runtime = self._runtime()
        self.assertTrue(runtime.start())
        retired_stop = runtime._stop

        runtime.stop()
        self.assertTrue(runtime.start())
        try:
            self.assertIsNot(runtime._stop, retired_stop)
            self.assertTrue(retired_stop.is_set())
        finally:
            runtime.stop()

    def test_stuck_stop_fails_loudly_and_retains_thread_for_inspection(self):
        runtime = self._runtime()
        stuck_thread = _StuckThread(target=lambda: None, daemon=True)

        with patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.rt.Thread",
            return_value=stuck_thread,
        ):
            runtime.start()
        self.assertTrue(runtime._launch.enter())

        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            runtime.stop()

        self.assertIs(runtime.heartbeat_thread(), stuck_thread)

    def test_thread_start_failure_restores_restartable_state(self):
        runtime = self._runtime()
        thread = Mock()
        thread.ident = None
        failure = RuntimeError("thread start failed")
        thread.start.side_effect = failure
        thread.is_alive.return_value = False

        with patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.rt.Thread",
            return_value=thread,
        ):
            with self.assertRaises(RuntimeError) as raised:
                runtime.start()

        self.assertIs(raised.exception, failure)
        self.assertFalse(runtime.is_started())
        self.assertIsNone(runtime.heartbeat_thread())
        runtime.start()
        runtime.stop()

    def test_prelaunch_baseexception_is_cancelled_without_retaining_owner(self):
        runtime = self._runtime()
        thread = Mock()
        thread.ident = None
        failure = KeyboardInterrupt()
        thread.start.side_effect = failure
        thread.is_alive.return_value = False

        with patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.rt.Thread",
            return_value=thread,
        ):
            with self.assertRaises(KeyboardInterrupt) as raised:
                runtime.start()

        self.assertIs(raised.exception, failure)
        self.assertFalse(runtime.is_started())
        self.assertIsNone(runtime.heartbeat_thread())
        runtime.stop()
        thread.join.assert_not_called()

    def test_stop_cancels_delayed_bootstrap_without_join_error(self):
        runtime = self._runtime()
        runtime._heartbeat_loop = Mock()
        release = threading.Event()
        original_start = threading.Thread.start
        delayed_thread = None
        launcher = None

        def delayed_start(thread):
            nonlocal delayed_thread, launcher
            delayed_thread = thread

            def launch_later():
                self.assertTrue(release.wait(timeout=1.0))
                original_start(thread)

            launcher = threading.Thread(target=launch_later, daemon=True)
            original_start(launcher)
            return None

        try:
            with patch.object(threading.Thread, "start", delayed_start):
                runtime.start()
                runtime.stop()
            self.assertIsNone(runtime.heartbeat_thread())
        finally:
            release.set()
            launcher.join(timeout=1.0)
            delayed_thread.join(timeout=1.0)

        runtime._heartbeat_loop.assert_not_called()

    def test_heartbeat_failure_is_persistent_and_prevents_silent_restart(self):
        presence = Mock(spec=SwarmPresence)
        failure = RuntimeError("heartbeat encoder failed")
        attempted = threading.Event()

        def fail_heartbeat():
            attempted.set()
            raise failure

        presence.heartbeat.side_effect = fail_heartbeat
        runtime = SwarmHeartbeatRuntime(presence)

        with patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            0.0,
        ):
            runtime.start()
            self.assertTrue(attempted.wait(1.0))
            thread = runtime.heartbeat_thread()
            self.assertIsNotNone(thread)
            thread.join(1.0)

        self.assertFalse(thread.is_alive())
        self.assertFalse(runtime.is_started())
        runtime.stop()
        for _ in range(2):
            with self.assertRaises(RuntimeError) as raised:
                runtime.raise_if_failed()
            self.assertIs(raised.exception, failure)
        with self.assertRaises(RuntimeError) as raised:
            runtime.start()
        self.assertIs(raised.exception, failure)

    def test_recoverable_heartbeat_status_does_not_poison_health(self):
        presence = Mock(spec=SwarmPresence)
        attempted = threading.Event()

        def unavailable():
            attempted.set()
            return None  # the send failed and was reported

        presence.heartbeat.side_effect = unavailable
        runtime = SwarmHeartbeatRuntime(presence)
        with patch(
            "navpy.core.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            0.01,
        ):
            runtime.start()
            self.assertTrue(attempted.wait(1.0))
            runtime.raise_if_failed()
            self.assertTrue(runtime.is_started())
            runtime.stop()


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class SwarmHeartbeatRestartTest(unittest.TestCase):
    """Recovery step 5c: a failed generation restarts fresh, within its tries (docs/design/navpy_recovery.md)."""

    def setUp(self):
        patcher = patch("navpy.core.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S", 0.0)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.clock = _Clock()
        self.reports = []
        self.presence = Mock(spec=SwarmPresence)
        self.runtime = SwarmHeartbeatRuntime(
            self.presence, self.reports.append, RestartBudget(clock=self.clock),
        )
        self.addCleanup(self.runtime.stop)

    def _fail_once(self, failure):
        """The running generation dies on its next heartbeat; wait until it has left."""
        failed = threading.Event()

        def fail():
            failed.set()
            raise failure

        self.presence.heartbeat.side_effect = fail
        self.assertTrue(failed.wait(1.0))
        thread = self.runtime.heartbeat_thread()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        self.presence.heartbeat.side_effect = None
        return thread

    def test_failed_generation_restarts_fresh_and_heartbeats_again(self):
        self.runtime.start()
        failure = RuntimeError("send failed")
        old = self._fail_once(failure)

        self.runtime.raise_if_failed()      # first try is immediate: no raise

        self.assertTrue(self.runtime.is_started())
        self.assertIsNot(self.runtime.heartbeat_thread(), old)
        beat = threading.Event()
        self.presence.heartbeat.side_effect = lambda: beat.set()
        self.assertTrue(beat.wait(1.0))
        self.runtime.raise_if_failed()
        self.assertEqual(len(self.reports), 1)
        self.assertIn("restarted (try 1)", self.reports[0])
        self.assertIn("send failed", self.reports[0])

    def test_tries_back_off_then_the_failure_reaches_the_health_check(self):
        self.runtime.start()
        for try_number, wait_s in ((1, 0.0), (2, 1.0), (3, 4.0)):
            self._fail_once(RuntimeError(f"failure {try_number}"))
            self.clock.now += wait_s - 0.01 if wait_s else 0.0
            if wait_s:
                self.runtime.raise_if_failed()       # back-off not over: waits quietly
                self.assertFalse(self.runtime.is_started())
                self.clock.now += 0.01
            self.runtime.raise_if_failed()
            self.assertTrue(self.runtime.is_started())
        last = RuntimeError("failure 4")
        self._fail_once(last)                        # at once again: tries used up
        with self.assertRaises(RuntimeError) as raised:
            self.runtime.raise_if_failed()
        self.assertIs(raised.exception, last)

    def test_a_generation_that_ran_stably_gets_every_try_again(self):
        self.runtime.start()
        for _ in range(5):
            self._fail_once(RuntimeError("rare"))
            self.runtime.raise_if_failed()
            self.assertTrue(self.runtime.is_started())
            self.clock.now += 30.0

    def test_no_restart_after_the_owner_stopped_it(self):
        self.runtime.start()
        failure = RuntimeError("send failed")
        self._fail_once(failure)
        self.runtime.stop()
        with self.assertRaises(RuntimeError) as raised:
            self.runtime.raise_if_failed()
        self.assertIs(raised.exception, failure)
        self.assertEqual(self.reports, [])

    def test_base_exception_is_not_restarted(self):
        self.runtime.start()
        failure = KeyboardInterrupt()
        self._fail_once(failure)
        with self.assertRaises(KeyboardInterrupt):
            self.runtime.raise_if_failed()
        self.assertFalse(self.runtime.is_started())


if __name__ == "__main__":
    unittest.main()
