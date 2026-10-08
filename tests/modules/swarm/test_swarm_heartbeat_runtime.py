import threading
import unittest
from unittest.mock import Mock, patch

from navpy.modules.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.modules.swarm.swarm_presence import SwarmPresence


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
            "navpy.modules.swarm.swarm_heartbeat_runtime.threading.Thread",
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
            "navpy.modules.swarm.swarm_heartbeat_runtime.threading.Thread",
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
            "navpy.modules.swarm.swarm_heartbeat_runtime.threading.Thread",
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
            "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
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
            "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
            0.01,
        ):
            runtime.start()
            self.assertTrue(attempted.wait(1.0))
            runtime.raise_if_failed()
            self.assertTrue(runtime.is_started())
            runtime.stop()


if __name__ == "__main__":
    unittest.main()
