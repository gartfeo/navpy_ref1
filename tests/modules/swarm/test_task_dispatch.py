import unittest
from unittest.mock import Mock, patch

from navpy.modules.comm.messages.available_task_msg import TaskHandleMsgData, TaskMsgData
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import TaskDispatchStatus, TaskTypeMsgData
from navpy.modules.swarm.task_dispatch import TaskDispatch


class TaskDispatchTest(unittest.TestCase):
    def setUp(self):
        self.task = TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.SMALL,
            location=LocationMsgData(12.34, 56.78, 90.0)
        )
        self.task_dispatch = TaskDispatch(self.task)

    def test_initial_status(self):
        self.assertEqual(self.task_dispatch.status, TaskDispatchStatus.AVAILABLE)

    def test_peer_availability(self):
        # Arrange
        receiver_id = 100
        task_handle = TaskHandleMsgData(task_id=1, time_in_min=5.0)

        # Act
        self.task_dispatch.on_peer_available(receiver_id, task_handle)

        # Assert
        self.assertIn(receiver_id, self.task_dispatch.task_handle_by_peer)
        self.assertEqual(self.task_dispatch.task_handle_by_peer[receiver_id], task_handle)

    def test_peer_acceptance(self):
        # Arrange
        receiver_id = 100
        self.task_dispatch.peer_accept(receiver_id)

        # Assert
        self.assertEqual(self.task_dispatch.assigned_peer, receiver_id)
        self.assertEqual(self.task_dispatch.status, TaskDispatchStatus.CONFIRMED)

    def test_peer_rejection(self):
        # Arrange
        receiver_id = 100
        task_handle = TaskHandleMsgData(task_id=1, time_in_min=5.0)
        self.task_dispatch.on_peer_available(receiver_id, task_handle)

        # Act
        remaining_peers = self.task_dispatch.peer_reject(receiver_id)

        # Assert
        self.assertNotIn(receiver_id, self.task_dispatch.task_handle_by_peer)
        self.assertEqual(remaining_peers, 0)
        self.assertEqual(self.task_dispatch.status, TaskDispatchStatus.AVAILABLE)

    def test_timer_management(self):
        # Act
        self.task_dispatch.start_peer_select_timer(lambda x: x, 5.0)

        # Assert
        self.assertIsNotNone(self.task_dispatch._timer_event)

        # Cleanup
        self.task_dispatch.cancel_peer_select_timer()
        self.assertIsNone(self.task_dispatch._timer_event)

    def test_peer_select_timer_does_not_restart_while_active(self):
        class FakeTimer:
            def __init__(self, _interval, function, args=None):
                self.function = function
                self.args = args or ()
                self.cancelled = False
                self._alive = False

            def start(self):
                self._alive = True

            def cancel(self):
                self.cancelled = True
                self._alive = False

            def is_alive(self):
                return self._alive

            def fire(self):
                self._alive = False
                self.function(*self.args)

        timers = []

        def make_timer(*args, **kwargs):
            timer = FakeTimer(*args, **kwargs)
            timers.append(timer)
            return timer

        callback = Mock()

        def callback_after_clear(task_id):
            self.assertIsNone(self.task_dispatch._timer_event)
            callback(task_id)

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_dispatch.start_peer_select_timer(callback_after_clear, 5.0)
            self.task_dispatch.start_peer_select_timer(callback_after_clear, 5.0)

            self.assertEqual(len(timers), 1)
            self.assertFalse(timers[0].cancelled)

            timers[0].fire()

            callback.assert_called_once_with(1)
            self.assertIsNone(self.task_dispatch._timer_event)

            self.task_dispatch.start_peer_select_timer(callback_after_clear, 5.0)
            self.assertEqual(len(timers), 2)

    def test_rebroadcast_timer_start_cancel(self):
        # Act — start rebroadcast timer
        self.task_dispatch.start_rebroadcast(lambda x: x, 5.0)

        # Assert — timer is set
        self.assertIsNotNone(self.task_dispatch._rebroadcast_timer)
        self.assertTrue(self.task_dispatch.has_active_rebroadcast())

        # Act — cancel
        self.task_dispatch.cancel_rebroadcast()

        # Assert — timer cleared
        self.assertIsNone(self.task_dispatch._rebroadcast_timer)
        self.assertFalse(self.task_dispatch.has_active_rebroadcast())

    def test_rebroadcast_timer_clears_after_firing(self):
        class FakeTimer:
            def __init__(self, _interval, function, args=None):
                self.function = function
                self.args = args or ()
                self._alive = False

            def start(self):
                self._alive = True

            def cancel(self):
                self._alive = False

            def is_alive(self):
                return self._alive

            def fire(self):
                self._alive = False
                self.function(*self.args)

        timers = []

        def make_timer(*args, **kwargs):
            timer = FakeTimer(*args, **kwargs)
            timers.append(timer)
            return timer

        callback = Mock()

        def callback_after_clear(task_id):
            self.assertIsNone(self.task_dispatch._rebroadcast_timer)
            self.assertFalse(self.task_dispatch.has_active_rebroadcast())
            callback(task_id)

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_dispatch.start_rebroadcast(callback_after_clear, 5.0)
            self.assertTrue(self.task_dispatch.has_active_rebroadcast())

            timers[0].fire()

        callback.assert_called_once_with(1)
        self.assertIsNone(self.task_dispatch._rebroadcast_timer)
        self.assertFalse(self.task_dispatch.has_active_rebroadcast())

    def test_rebroadcast_timer_keeps_replacement_started_by_callback(self):
        class FakeTimer:
            def __init__(self, interval, function, args=None):
                self.interval = interval
                self.function = function
                self.args = args or ()
                self.cancelled = False
                self._alive = False

            def start(self):
                self._alive = True

            def cancel(self):
                self.cancelled = True
                self._alive = False

            def is_alive(self):
                return self._alive

            def fire(self):
                self._alive = False
                self.function(*self.args)

        timers = []

        def make_timer(*args, **kwargs):
            timer = FakeTimer(*args, **kwargs)
            timers.append(timer)
            return timer

        def callback(_task_id):
            self.task_dispatch.start_rebroadcast(lambda task_id: task_id, 2.0)

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_dispatch.start_rebroadcast(callback, 5.0)
            timers[0].fire()

        self.assertEqual(len(timers), 2)
        self.assertIs(self.task_dispatch._rebroadcast_timer, timers[1])
        self.assertEqual(timers[1].interval, 2.0)
        self.assertTrue(self.task_dispatch.has_active_rebroadcast())

        self.task_dispatch.cancel_rebroadcast()

    def test_peer_accept_cancels_rebroadcast(self):
        # Arrange — start a rebroadcast timer
        self.task_dispatch.start_rebroadcast(lambda x: x, 5.0)
        self.assertIsNotNone(self.task_dispatch._rebroadcast_timer)

        # Act — peer accepts
        self.task_dispatch.peer_accept(100)

        # Assert — rebroadcast cancelled
        self.assertIsNone(self.task_dispatch._rebroadcast_timer)
        self.assertEqual(self.task_dispatch.status, TaskDispatchStatus.CONFIRMED)

    def test_shutdown_cancels_rebroadcast(self):
        # Arrange
        self.task_dispatch.start_rebroadcast(lambda x: x, 5.0)
        self.assertIsNotNone(self.task_dispatch._rebroadcast_timer)

        # Act
        self.task_dispatch.shutdown()

        # Assert
        self.assertIsNone(self.task_dispatch._rebroadcast_timer)


if __name__ == '__main__':
    unittest.main()
