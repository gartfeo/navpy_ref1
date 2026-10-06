import threading
import unittest
from unittest.mock import Mock

from navpy.logger.cache_logger import ILogger
from navpy.modules.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.modules.swarm.swarm_presence import SwarmPresence
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.vehicle.vehicle_interface import IVehicle


class SwarmPresenceTest(unittest.TestCase):
    def _presence(self):
        vehicle = Mock(spec=IVehicle)
        sender = Mock(spec=TaskMessageSender)
        logger = Mock(spec=ILogger)
        return SwarmPresence(1, vehicle, sender, Mock(), logger), sender

    def test_committed_start_announces_checkin_once_per_generation(self):
        presence, sender = self._presence()
        announced = threading.Event()
        sender.checkin.side_effect = lambda: announced.set() or True

        presence.start()
        self.assertTrue(announced.wait(1.0))
        presence.start()
        self.assertEqual(sender.checkin.call_count, 1)
        presence.stop()

        announced.clear()
        presence.start()
        try:
            self.assertTrue(announced.wait(1.0))
            self.assertEqual(sender.checkin.call_count, 2)
        finally:
            presence.stop()

    def test_checkin_admits_only_foreign_peer_and_replies_with_heartbeat(self):
        vehicle = Mock(spec=IVehicle)
        sender = Mock(spec=TaskMessageSender)
        discovered = Mock()
        presence = SwarmPresence(
            1,
            vehicle,
            sender,
            discovered,
            Mock(spec=ILogger),
        )

        presence.on_checkin(Mock(sender_id=1))
        presence.on_checkin(Mock(sender_id=7))

        discovered.assert_called_once_with(7)
        sender.heartbeat.assert_called_once_with()

    def test_checkin_failure_stops_new_runtime_and_reraises_exact_failure(self):
        presence, sender = self._presence()
        heartbeat = Mock(spec=SwarmHeartbeatRuntime)
        heartbeat.start.return_value = True
        presence._heartbeat = heartbeat
        failure = RuntimeError("checkin failed")
        sender.checkin.side_effect = failure

        with self.assertRaises(RuntimeError) as raised:
            presence.start()

        self.assertIs(raised.exception, failure)
        heartbeat.stop.assert_called_once_with()

    def test_lifecycle_queries_delegate_to_heartbeat_runtime(self):
        presence, _sender = self._presence()
        heartbeat = Mock(spec=SwarmHeartbeatRuntime)
        thread = Mock(spec=threading.Thread)
        heartbeat.is_started.return_value = True
        heartbeat.heartbeat_thread.return_value = thread
        presence._heartbeat = heartbeat

        self.assertTrue(presence.is_started())
        self.assertIs(presence.heartbeat_thread(), thread)
        presence.raise_if_failed()
        presence.stop()

        heartbeat.raise_if_failed.assert_called_once_with()
        heartbeat.stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
