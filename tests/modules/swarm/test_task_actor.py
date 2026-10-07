import threading
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.messages.available_task_msg import (
    TaskAssignResponseMsg, TaskAssignRequestMsg, TaskAssignMsgData,
    TaskHandleMsgData, TaskMsgData,
    AvailableTaskResponseMsg, AvailableTaskRequestMsg,
)
from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.comm.messages.types import TaskDispatchStatus, TaskTypeMsgData
from navpy.modules.common.models.location import Location
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.vision.models.detect_data import DetectedObject, DetectionSizeClass
from tests.detection_factory import make_detected_poi
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.swarm.task_actor import TaskActor
from navpy.modules.swarm.task_auction_models import TaskRejectOutcome
from navpy.modules.swarm.task_dispatch import TaskDispatch
from navpy.modules.swarm.task_ports import MessageClockReset
from navpy.logger.cache_logger import ILogger


class _IdleTimer:
    """threading.Timer stand-in that never fires on its own."""

    def __init__(self, interval, function, args=None):
        self.interval = interval
        self.function = function
        self.args = args or ()
        self._alive = False

    def start(self):
        self._alive = True

    def cancel(self):
        self._alive = False

    def is_alive(self):
        return self._alive


class TaskActorTest(unittest.TestCase):
    def setUp(self):
        # No wall clock: auction timers are idle fakes (tests fire what they
        # need) and the periodic heartbeat never ticks, so no background
        # thread can broadcast while a test counts broadcasts.
        for target, value in (
            ("navpy.modules.swarm.task_dispatch.threading.Timer", _IdleTimer),
            (
                "navpy.modules.swarm.swarm_heartbeat_runtime.HEARTBEAT_INTERVAL_S",
                threading.TIMEOUT_MAX,
            ),
        ):
            patcher = patch(target, value)
            patcher.start()
            self.addCleanup(patcher.stop)

        # Mocking IDrone
        self.vehicle = Mock(spec=IVehicle)
        self.vehicle.source_system = 1
        self.vehicle.location.return_value = Location(12.34, 56.78, 90.0)

        # Mocking NetworkAbc
        self.network = Mock(spec=NetworkAbc)

        # Mocking ILogger
        self.logger = Mock(spec=ILogger)

        # TaskActor instance
        self.task_actor = TaskActor(vehicle=self.vehicle, network=self.network, logger=self.logger)
        self.task_actor.start()
        self.network.reset_mock()
        self.logger.reset_mock()

    def tearDown(self):
        self.task_actor.reset()

    def _register(self, task_or_dispatch):
        registered = (
            self.task_actor._auction_state.register_dispatch(task_or_dispatch)
            if isinstance(task_or_dispatch, TaskDispatch)
            else self.task_actor._auction_state.register(task_or_dispatch)
        )
        self.assertIsNotNone(registered)
        return registered[0]

    def _dispatch(self, task_id):
        return self.task_actor._auction_state.lookup(task_id)

    def _discover(self, *peer_ids):
        for peer_id in peer_ids:
            self.task_actor.on_message(SwarmHeartbeatMsg(sender_id=peer_id))

    def _rebroadcast_task(self, task_id):
        self.task_actor._rebroadcast._rebroadcast_task(
            task_id,
            self.task_actor._auction_state.current_generation(),
        )

    def _select_peer(self, task_id):
        self.task_actor._auction._select_peer_for_task(
            task_id,
            self.task_actor._auction_state.current_generation(),
        )

    def test_notify_pois(self):
        # Arrange
        gimbal_data = Mock(spec=GimbalData)
        vehicle_attitude = Mock(spec=Attitude)

        detected_pois = [
            make_detected_poi(
                obj_id=1,
                x_error=0.0,
                y_error=0.0,
                reference_height_m=1.0,
                k=1.0,
                g_data=gimbal_data,
                uas_att=vehicle_attitude,
                t_g_loc_debug=Location(12.34, 56.78, 90.0)
            ),
            make_detected_poi(
                obj_id=2,
                x_error=0.0,
                y_error=0.0,
                reference_height_m=1.0,
                k=1.0,
                g_data=gimbal_data,
                uas_att=vehicle_attitude,
                t_g_loc_debug=Location(23.45, 67.89, 100.0)
            ),
        ]

        # Manually set the size_class
        detected_pois[0].replace_classification(replace(
            detected_pois[0].classification,
            size_class=DetectionSizeClass.S,
        ))
        detected_pois[1].replace_classification(replace(
            detected_pois[1].classification,
            size_class=DetectionSizeClass.M,
        ))

        # Manually set the predicted POI location if not set
        detected_pois[0].set_p_t_g_loc(
            detected_pois[0].geo.truth_poi_location,
        )
        detected_pois[1].set_p_t_g_loc(
            detected_pois[1].geo.truth_poi_location,
        )

        # Act
        self.task_actor.notify_pois(detected_pois)

        # Assert
        self.assertEqual(len(self.task_actor._auction_state.task_ids()), 2)
        self.network.broadcast.assert_called()
        self.logger.info.assert_called()

    def test_notify_pois_uses_task_id_for_dispatch_identity(self):
        """POIs with the same local obj_id stay distinct when task IDs differ."""
        gimbal_data = Mock(spec=GimbalData)
        vehicle_attitude = Mock(spec=Attitude)

        detected_pois = [
            make_detected_poi(
                obj_id=1,
                task_id=101,
                x_error=0.0,
                y_error=0.0,
                reference_height_m=1.0,
                k=1.0,
                g_data=gimbal_data,
                uas_att=vehicle_attitude,
                t_g_loc_debug=Location(12.34, 56.78, 90.0),
            ),
            make_detected_poi(
                obj_id=1,
                task_id=202,
                x_error=0.0,
                y_error=0.0,
                reference_height_m=1.0,
                k=1.0,
                g_data=gimbal_data,
                uas_att=vehicle_attitude,
                t_g_loc_debug=Location(23.45, 67.89, 100.0),
            ),
        ]

        detected_pois[0].set_p_t_g_loc(
            detected_pois[0].geo.truth_poi_location,
        )
        detected_pois[1].set_p_t_g_loc(
            detected_pois[1].geo.truth_poi_location,
        )

        self.task_actor.notify_pois(detected_pois)

        self.assertCountEqual(
            self.task_actor._auction_state.task_ids(),
            [101, 202],
        )

    def test_on_message_with_wrong_receiver(self):
        # Arrange
        msg = CheckInMsg(sender_id=2)
        msg.receiver_id = 99  # Different receiver ID

        # Act
        self.task_actor.on_message(msg)

        # Assert
        # No methods should be called since receiver_id doesn't match
        self.logger.info.assert_not_called()

    def test_on_message_with_correct_receiver(self):
        # Arrange
        msg = CheckInMsg(sender_id=2)
        msg.receiver_id = None  # Broadcast message

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.logger.info.assert_called_with(f"Actor {msg.sender_id} checked in.")
        self.assertEqual(self.task_actor._rebroadcast.known_peers(), {2})
        heartbeat = self.network.broadcast.call_args.args[0]
        self.assertIsInstance(heartbeat, SwarmHeartbeatMsg)

    def test_on_task_available_request_busy(self):
        # Arrange
        self._discover(2)
        msg = AvailableTaskRequestMsg(sender_id=2, tasks=[])
        self.task_actor._selection.try_accept(Mock(), owner_id=3)

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.logger.info.assert_called_with("Already handling tasks. Can't handle more tasks.")
        self.network.broadcast.assert_not_called()

    def test_on_task_available_response(self):
        # Arrange
        self._discover(2)
        task_handle = TaskHandleMsgData(task_id=1, time_in_min=5.0)
        msg = AvailableTaskResponseMsg(sender_id=2, receiver_id=self.task_actor.id, tasks=[task_handle])
        dispatch = self._register(TaskDispatch(TaskMsgData(
            task_id=1, task_type=TaskTypeMsgData.DOCK, location=LocationMsgData(12.34, 56.78, 90.0)
        )))

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.logger.info.assert_called()
        self.assertIn(2, dispatch.task_handle_by_peer)

    def test_selection_deadline_waits_for_complete_three_uav_bid_matrix(self):
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

        task = TaskMsgData(
            task_id=10,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(12.34, 56.78, 90.0),
        )
        task_dispatch = TaskDispatch(task)
        self._register(task_dispatch)
        self._discover(5)

        msg = AvailableTaskResponseMsg(
            sender_id=5,
            receiver_id=self.task_actor.id,
            tasks=[TaskHandleMsgData(task_id=10, time_in_min=2.0)],
        )

        self.network.broadcast.reset_mock()

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_actor.on_message(msg)
            self.task_actor.on_message(msg)

            self.assertEqual(len(timers), 1)
            self.assertFalse(timers[0].cancelled)

            timers[0].fire()

            self.assertFalse(any(
                isinstance(call.args[0], TaskAssignRequestMsg)
                for call in self.network.broadcast.call_args_list
            ))
            self.assertEqual(task_dispatch.status, TaskDispatchStatus.AVAILABLE)
            self.assertIsNone(task_dispatch.assigned_peer)

            self._discover(6)
            self.network.broadcast.reset_mock()
            self.task_actor.on_message(AvailableTaskResponseMsg(
                sender_id=6,
                receiver_id=self.task_actor.id,
                tasks=[TaskHandleMsgData(task_id=10, time_in_min=3.0)],
            ))

        self.network.broadcast.assert_called_once()
        sent_msg = self.network.broadcast.call_args[0][0]
        self.assertIsInstance(sent_msg, TaskAssignRequestMsg)
        self.assertEqual(sent_msg.receiver_id, 5)
        self.assertEqual(task_dispatch.status, TaskDispatchStatus.CONFIRMING)
        self.assertEqual(task_dispatch.assigned_peer, 5)

    def test_complete_bid_matrix_assigns_before_selection_deadline(self):
        """Do not let a fast navigation task outrun a complete two-peer auction."""
        self._discover(2, 3)
        for task_id in (10, 20):
            self._register(TaskDispatch(TaskMsgData(
                task_id=task_id,
                task_type=TaskTypeMsgData.DOCK,
                location=LocationMsgData(12.34, 56.78, 90.0),
            )))

        responses = (
            AvailableTaskResponseMsg(
                sender_id=2,
                receiver_id=self.task_actor.id,
                tasks=[TaskHandleMsgData(task_id=10, time_in_min=1.0)],
            ),
            AvailableTaskResponseMsg(
                sender_id=2,
                receiver_id=self.task_actor.id,
                tasks=[TaskHandleMsgData(task_id=20, time_in_min=2.0)],
            ),
            AvailableTaskResponseMsg(
                sender_id=3,
                receiver_id=self.task_actor.id,
                tasks=[TaskHandleMsgData(task_id=10, time_in_min=2.0)],
            ),
            AvailableTaskResponseMsg(
                sender_id=3,
                receiver_id=self.task_actor.id,
                tasks=[TaskHandleMsgData(task_id=20, time_in_min=100.0)],
            ),
        )
        self.network.broadcast.reset_mock()

        with patch.object(
            TaskDispatch,
            "start_peer_select_timer",
            return_value=None,
        ):
            for response in responses[:-1]:
                self.task_actor.on_message(response)
                self.assertFalse(any(
                    isinstance(call.args[0], TaskAssignRequestMsg)
                    for call in self.network.broadcast.call_args_list
                ))
            self.task_actor.on_message(responses[-1])

        assignment_messages = [
            message
            for call in self.network.broadcast.call_args_list
            if isinstance((message := call.args[0]), TaskAssignRequestMsg)
        ]
        self.assertEqual(len(assignment_messages), 2)
        self.assertEqual(
            {(message.task.task_id, message.receiver_id) for message in assignment_messages},
            {(10, 3), (20, 2)},
        )

        self.task_actor.on_message(responses[-1])
        self.assertEqual(
            sum(
                isinstance(call.args[0], TaskAssignRequestMsg)
                for call in self.network.broadcast.call_args_list
            ),
            2,
        )
        self.task_actor.reset()
        self.assertEqual(self.task_actor._auction_state.task_ids(), set())

    def test_third_remote_peer_is_not_admitted_to_demo_auction(self):
        self._discover(2, 3, 4)
        dispatch = self._register(TaskDispatch(TaskMsgData(
            task_id=9,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(12.34, 56.78, 90.0),
        )))
        message = AvailableTaskResponseMsg(
            sender_id=4,
            receiver_id=self.task_actor.id,
            tasks=[TaskHandleMsgData(task_id=9, time_in_min=1.0)],
        )

        self.task_actor.on_message(message)

        self.assertEqual(self.task_actor._rebroadcast.known_peers(), {2, 3})
        self.assertNotIn(4, dispatch.task_handle_by_peer)
        self.logger.warning.assert_called_with(
            "Ignoring AVAILABLE_TASK_RESPONSE from unadmitted peer 4."
        )

    def test_third_remote_peer_cannot_advertise_tasks(self):
        self._discover(2, 3, 4)
        message = AvailableTaskRequestMsg(sender_id=4, tasks=[TaskMsgData(
            task_id=9,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(12.34, 56.78, 90.0),
        )])
        self.network.broadcast.reset_mock()

        self.task_actor.on_message(message)

        self.network.broadcast.assert_not_called()
        self.logger.warning.assert_called_with(
            "Ignoring AVAILABLE_TASK_REQUEST from unadmitted peer 4."
        )

    def test_third_remote_peer_cannot_assign_this_vehicle(self):
        self._discover(2, 3, 4)
        message = TaskAssignRequestMsg(
            sender_id=4,
            receiver_id=self.task_actor.id,
            task=TaskAssignMsgData(
                task_id=9,
                task_type=TaskTypeMsgData.DOCK,
                location=LocationMsgData(12.34, 56.78, 90.0),
            ),
        )
        self.network.broadcast.reset_mock()

        self.task_actor.on_message(message)

        self.assertIsNone(self.task_actor.selected_poi())
        self.network.broadcast.assert_not_called()
        self.logger.warning.assert_called_with(
            "Ignoring TASK_ASSIGN_REQUEST from unadmitted peer 4."
        )

    def test_select_peer(self):
        # Arrange
        self._discover(2, 3)
        task_dispatch = TaskDispatch(TaskMsgData(
            task_id=1, task_type=TaskTypeMsgData.DOCK, location=LocationMsgData(12.34, 56.78, 90.0)
        ))
        self._register(task_dispatch)
        task_handle1 = TaskHandleMsgData(task_id=1, time_in_min=5.0)
        task_handle2 = TaskHandleMsgData(task_id=1, time_in_min=3.0)
        task_dispatch.on_peer_available(2, task_handle1)
        task_dispatch.on_peer_available(3, task_handle2)

        # Act
        self._select_peer(1)

        # Assert
        self.network.broadcast.assert_called()
        args, kwargs = self.network.broadcast.call_args
        sent_msg = args[0]
        self.assertEqual(sent_msg.receiver_id, 3)  # Peer with ID 3 has lower time_in_min
        self.assertEqual(sent_msg.sender_id, self.task_actor.id)

    def test_on_task_assign_response_accept(self):
        # Arrange
        self._discover(2)
        task_dispatch = TaskDispatch(TaskMsgData(
            task_id=1, task_type=TaskTypeMsgData.DOCK, location=LocationMsgData(12.34, 56.78, 90.0)
        ))
        task_dispatch.assigned_peer = 2
        task_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(task_dispatch)
        msg = TaskAssignResponseMsg(
            sender_id=2,
            receiver_id=self.task_actor.id,
            task_id=1,
            is_accepted=True
        )

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.assertEqual(task_dispatch.status, TaskDispatchStatus.CONFIRMED)
        self.assertEqual(task_dispatch.assigned_peer, 2)
        self.logger.info.assert_called_with(f"Task {msg.task_id} accepted by {msg.sender_id}")

    def test_on_task_assign_response_reject(self):
        # Arrange
        self._discover(2)
        task_dispatch = TaskDispatch(TaskMsgData(
            task_id=1, task_type=TaskTypeMsgData.DOCK, location=LocationMsgData(12.34, 56.78, 90.0)
        ))
        task_dispatch.task_handle_by_peer = {2: TaskHandleMsgData(task_id=1, time_in_min=5.0)}
        task_dispatch.assigned_peer = 2
        task_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(task_dispatch)
        msg = TaskAssignResponseMsg(
            sender_id=2,
            receiver_id=self.task_actor.id,
            task_id=1,
            is_accepted=False
        )

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.assertEqual(task_dispatch.status, TaskDispatchStatus.AVAILABLE)
        self.logger.info.assert_called_with(f"Broadcast available tasks.")
        self.network.broadcast.assert_called()  # Should re-broadcast available tasks

    def test_rejection_reassigns_the_remaining_valid_bid(self):
        self._discover(2, 3)
        task_dispatch = TaskDispatch(TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(12.34, 56.78, 90.0),
        ))
        task_dispatch.task_handle_by_peer = {
            2: TaskHandleMsgData(task_id=1, time_in_min=2.0),
            3: TaskHandleMsgData(task_id=1, time_in_min=3.0),
        }
        task_dispatch.assigned_peer = 2
        task_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(task_dispatch)
        newer_dispatch = TaskDispatch(TaskMsgData(
            task_id=2,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(23.45, 67.89, 90.0),
        ))
        newer_dispatch.task_handle_by_peer = {
            3: TaskHandleMsgData(task_id=2, time_in_min=1.0),
        }
        self._register(newer_dispatch)
        self.network.broadcast.reset_mock()

        with patch.object(TaskDispatch, "start_rebroadcast", return_value=None):
            self.task_actor.on_message(TaskAssignResponseMsg(
                sender_id=2,
                receiver_id=self.task_actor.id,
                task_id=1,
                is_accepted=False,
            ))

        self.assertEqual(task_dispatch.status, TaskDispatchStatus.CONFIRMING)
        self.assertEqual(task_dispatch.assigned_peer, 3)
        self.network.broadcast.assert_called_once()
        assignment = self.network.broadcast.call_args.args[0]
        self.assertIsInstance(assignment, TaskAssignRequestMsg)
        self.assertEqual(assignment.receiver_id, 3)
        self.assertEqual(assignment.task.task_id, 1)
        self.assertEqual(newer_dispatch.status, TaskDispatchStatus.AVAILABLE)
        self.assertIsNone(newer_dispatch.assigned_peer)

    def test_stale_rejection_cannot_retry_a_replacement_generation(self) -> None:
        self._discover(2, 3)
        rejected_dispatch = TaskDispatch(TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(12.34, 56.78, 90.0),
        ))
        rejected_dispatch.task_handle_by_peer = {
            2: TaskHandleMsgData(task_id=1, time_in_min=2.0),
            3: TaskHandleMsgData(task_id=1, time_in_min=3.0),
        }
        rejected_dispatch.assigned_peer = 2
        rejected_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(rejected_dispatch)
        original_reject = self.task_actor._auction_state.reject
        replacements: list[TaskDispatch] = []

        def reject_then_replace(
            task_id: int,
            peer_id: int,
        ) -> TaskRejectOutcome:
            outcome = original_reject(task_id, peer_id)
            self.task_actor.reset()
            self.task_actor.start()
            self._discover(2, 3)
            replacement = TaskDispatch(TaskMsgData(
                task_id=1,
                task_type=TaskTypeMsgData.DOCK,
                location=LocationMsgData(23.45, 67.89, 90.0),
            ))
            replacement.task_handle_by_peer = {
                3: TaskHandleMsgData(task_id=1, time_in_min=1.0),
            }
            self._register(replacement)
            replacements.append(replacement)
            self.network.broadcast.reset_mock()
            return outcome

        with (
            patch.object(
                self.task_actor._auction_state,
                "reject",
                side_effect=reject_then_replace,
            ),
            patch.object(
                TaskDispatch,
                "start_rebroadcast",
                return_value=None,
            ) as start_rebroadcast,
        ):
            self.task_actor.on_message(TaskAssignResponseMsg(
                sender_id=2,
                receiver_id=self.task_actor.id,
                task_id=1,
                is_accepted=False,
            ))

        replacement = replacements[0]
        self.assertEqual(replacement.status, TaskDispatchStatus.AVAILABLE)
        self.assertIsNone(replacement.assigned_peer)
        self.assertFalse(any(
            isinstance(call.args[0], TaskAssignRequestMsg)
            for call in self.network.broadcast.call_args_list
        ))
        start_rebroadcast.assert_not_called()


    def test_swarm_heartbeat_tracks_peer(self):
        # Arrange
        msg = SwarmHeartbeatMsg(sender_id=5)

        # Act
        self.task_actor.on_message(msg)

        # Assert
        self.assertIn(5, self.task_actor._rebroadcast.known_peers())

    def test_swarm_heartbeat_ignores_self(self):
        # Arrange — sender_id matches actor id (the vehicle sysid)
        msg = SwarmHeartbeatMsg(sender_id=self.task_actor.id)

        # Act
        self.task_actor.on_message(msg)

        # Assert — own id NOT in known_peers
        self.assertNotIn(
            self.task_actor.id,
            self.task_actor._rebroadcast.known_peers(),
        )

    def test_actor_id_is_the_vehicle_source_system(self):
        # The companion shares its aircraft's sysid (component 191 tells them
        # apart), so the swarm actor identity is that plain sysid — which is
        # also the id peers and the GCS address replies to.
        self.assertEqual(self.task_actor.id, self.vehicle.source_system)

    def test_rebroadcast_when_peers_missing(self):
        # Arrange — register a known peer that has NOT responded
        self._discover(5, 6)
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        # Peer 5 responded, peer 6 did not
        td.on_peer_available(5, TaskHandleMsgData(task_id=10, time_in_min=2.0))
        self._register(td)

        self.network.broadcast.reset_mock()

        # Act — fire the rebroadcast callback
        self._rebroadcast_task(10)

        # Assert — broadcast called again for the missing peer
        self.network.broadcast.assert_called_once()
        self.logger.info.assert_any_call("Rebroadcast task 10, waiting for 1 peers")

    def test_rebroadcast_ignores_busy_assigned_peers(self):
        self._discover(5, 6)

        available_task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                                     location=LocationMsgData(1.0, 2.0, 3.0))
        available_dispatch = TaskDispatch(available_task)
        available_dispatch.on_peer_available(5, TaskHandleMsgData(task_id=10, time_in_min=2.0))
        self._register(available_dispatch)

        confirming_task = TaskMsgData(task_id=20, task_type=TaskTypeMsgData.DOCK,
                                      location=LocationMsgData(4.0, 5.0, 6.0))
        confirming_dispatch = TaskDispatch(confirming_task)
        confirming_dispatch.assigned_peer = 6
        confirming_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(confirming_dispatch)

        self.network.broadcast.reset_mock()

        self._rebroadcast_task(10)

        self.network.broadcast.assert_not_called()
        self.assertIsNone(available_dispatch._rebroadcast_timer)

    def test_rebroadcast_still_waits_for_unbusy_missing_peers(self):
        self._discover(5, 6)

        available_task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                                     location=LocationMsgData(1.0, 2.0, 3.0))
        available_dispatch = TaskDispatch(available_task)
        self._register(available_dispatch)

        busy_task = TaskMsgData(task_id=20, task_type=TaskTypeMsgData.DOCK,
                                location=LocationMsgData(4.0, 5.0, 6.0))
        busy_dispatch = TaskDispatch(busy_task)
        busy_dispatch.assigned_peer = 5
        busy_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(busy_dispatch)

        self.network.broadcast.reset_mock()

        self._rebroadcast_task(10)

        self.network.broadcast.assert_called_once()
        self.logger.info.assert_any_call("Rebroadcast task 10, waiting for 1 peers")

    def test_reject_restarts_available_rebroadcast_when_peer_becomes_free(self):
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

        self._discover(5, 6)

        available_task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                                     location=LocationMsgData(1.0, 2.0, 3.0))
        available_dispatch = TaskDispatch(available_task)
        available_dispatch.on_peer_available(5, TaskHandleMsgData(task_id=10, time_in_min=2.0))
        self._register(available_dispatch)

        busy_task = TaskMsgData(task_id=20, task_type=TaskTypeMsgData.DOCK,
                                location=LocationMsgData(4.0, 5.0, 6.0))
        busy_dispatch = TaskDispatch(busy_task)
        busy_dispatch.assigned_peer = 6
        busy_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(busy_dispatch)

        reject_msg = TaskAssignResponseMsg(
            sender_id=6,
            receiver_id=self.task_actor.id,
            task_id=20,
            is_accepted=False,
        )

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_actor.on_message(reject_msg)

        self.assertIs(available_dispatch._rebroadcast_timer, timers[0])
        self.assertEqual(timers[0].interval, 0.0)

        self.network.broadcast.reset_mock()
        timers[0].fire()

        sent = self.network.broadcast.call_args[0][0]
        self.assertIsInstance(sent, AvailableTaskRequestMsg)
        self.assertEqual(sent.tasks[0].task_id, 10)

    def test_rejected_task_restarts_when_remaining_peer_is_busy(self):
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

        self._discover(5, 6)

        rejected_task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                                    location=LocationMsgData(1.0, 2.0, 3.0))
        rejected_dispatch = TaskDispatch(rejected_task)
        rejected_dispatch.assigned_peer = 5
        rejected_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        rejected_dispatch.on_peer_available(5, TaskHandleMsgData(task_id=10, time_in_min=2.0))
        rejected_dispatch.on_peer_available(6, TaskHandleMsgData(task_id=10, time_in_min=3.0))
        self._register(rejected_dispatch)

        busy_task = TaskMsgData(task_id=20, task_type=TaskTypeMsgData.DOCK,
                                location=LocationMsgData(4.0, 5.0, 6.0))
        busy_dispatch = TaskDispatch(busy_task)
        busy_dispatch.assigned_peer = 6
        busy_dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(busy_dispatch)

        reject_msg = TaskAssignResponseMsg(
            sender_id=5,
            receiver_id=self.task_actor.id,
            task_id=10,
            is_accepted=False,
        )

        with patch("navpy.modules.swarm.task_dispatch.threading.Timer", side_effect=make_timer):
            self.task_actor.on_message(reject_msg)

        self.assertEqual(rejected_dispatch.status, TaskDispatchStatus.AVAILABLE)
        self.assertIsNone(rejected_dispatch.assigned_peer)
        self.assertEqual(rejected_dispatch.task_handle_by_peer, {
            6: TaskHandleMsgData(task_id=10, time_in_min=3.0),
        })
        self.assertIs(rejected_dispatch._rebroadcast_timer, timers[0])
        self.assertEqual(timers[0].interval, 0.0)

        self.network.broadcast.reset_mock()
        timers[0].fire()

        sent = self.network.broadcast.call_args[0][0]
        self.assertIsInstance(sent, AvailableTaskRequestMsg)
        self.assertEqual(sent.tasks[0].task_id, 10)

    def test_no_rebroadcast_when_all_responded(self):
        # Arrange — all known peers have responded
        self._discover(5)
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        td.on_peer_available(5, TaskHandleMsgData(task_id=10, time_in_min=2.0))
        self._register(td)

        self.network.broadcast.reset_mock()

        # Act
        self._rebroadcast_task(10)

        # Assert — no broadcast, rebroadcast cancelled
        self.network.broadcast.assert_not_called()
        self.assertIsNone(td._rebroadcast_timer)

    def test_rebroadcast_cancels_when_no_peers_known(self):
        """With no known peers, rebroadcast should cancel — nobody to send to."""
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        self._register(td)

        self.network.broadcast.reset_mock()

        # Act — fire rebroadcast with no known peers
        self._rebroadcast_task(10)

        # Assert — no broadcast, timer cancelled
        self.network.broadcast.assert_not_called()
        self.assertIsNone(td._rebroadcast_timer)

    def test_new_peer_triggers_rebroadcast_for_available_tasks(self):
        """When a new peer joins, rebroadcast should restart for available tasks."""
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        self._register(td)

        # Act — new peer arrives
        self.task_actor.on_message(SwarmHeartbeatMsg(sender_id=5))

        # Assert — rebroadcast timer started for the available task
        self.assertIsNotNone(td._rebroadcast_timer)
        self.assertEqual(td._rebroadcast_timer.interval, 0.0)
        self.assertTrue(td._rebroadcast_timer.is_alive())

    def test_existing_peer_heartbeat_does_not_restart_rebroadcast(self):
        """Heartbeat from already-known peer should not restart rebroadcast."""
        self._discover(5)
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        self._register(td)
        # No active rebroadcast timer
        self.assertIsNone(td._rebroadcast_timer)

        # Act — same peer heartbeat
        self.task_actor.on_message(SwarmHeartbeatMsg(sender_id=5))

        # Assert — timer NOT started (peer already known)
        self.assertIsNone(td._rebroadcast_timer)

    def test_rebroadcast_stops_on_assignment(self):
        # Arrange — task moves to CONFIRMING status
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        td = TaskDispatch(task)
        td.set_status(TaskDispatchStatus.CONFIRMING)
        self._register(td)
        self._discover(5)

        self.network.broadcast.reset_mock()

        # Act
        self._rebroadcast_task(10)

        # Assert — no broadcast since status is not AVAILABLE
        self.network.broadcast.assert_not_called()

    def test_reset_clears_all_state(self):
        """Reset must clear actor state AND clock offsets on the network filter."""
        # Use a real MessageFilter so we can verify offset clearing
        msg_filter = MessageFilter()
        self.network.message_filter = msg_filter
        self.task_actor._clock_reset = MessageClockReset(
            msg_filter.offset_estimator.clear
        )

        # Populate actor state
        self._discover(5, 6)
        self.task_actor._selection.try_accept(Mock(), owner_id=3)
        task = TaskMsgData(task_id=10, task_type=TaskTypeMsgData.DOCK,
                           location=LocationMsgData(1.0, 2.0, 3.0))
        self._register(task)

        # Populate clock offsets (simulating previous heartbeats)
        import time
        now_ms = int(time.time() * 1000)
        msg_filter.offset_estimator.update_offset(5, now_ms - 100)
        msg_filter.offset_estimator.update_offset(6, now_ms - 200)
        self.assertIsNotNone(msg_filter.offset_estimator.get_offset(5))

        # Act
        self.task_actor.reset()

        # Assert — actor state cleared
        self.assertEqual(len(self.task_actor._rebroadcast.known_peers()), 0)
        self.assertIsNone(self.task_actor.selected_poi())
        self.assertEqual(len(self.task_actor._auction_state.task_ids()), 0)
        self.assertFalse(self.task_actor._presence.is_started())

        # Assert — clock offsets cleared
        self.assertIsNone(msg_filter.offset_estimator.get_offset(5))
        self.assertIsNone(msg_filter.offset_estimator.get_offset(6))

    def test_fresh_actor_clears_stale_network_clock_offsets(self):
        msg_filter = MessageFilter()
        now_ms = 1_000_000
        msg_filter.offset_estimator.update_offset(5, now_ms - 100)
        self.assertIsNotNone(msg_filter.offset_estimator.get_offset(5))
        network = Mock(spec=NetworkAbc)
        network.message_filter = msg_filter

        actor = TaskActor(self.vehicle, network, self.logger)

        self.assertIsNone(msg_filter.offset_estimator.get_offset(5))
        actor.reset()

    def test_checkout_is_thin_presence_delegation(self):
        self.task_actor._presence.checkout = Mock()

        self.task_actor.checkout()

        self.task_actor._presence.checkout.assert_called_once_with()

    def test_raise_if_failed_is_thin_presence_delegation(self):
        self.task_actor._presence.raise_if_failed = Mock()

        self.task_actor.raise_if_failed()

        self.task_actor._presence.raise_if_failed.assert_called_once_with()

    def test_public_compatibility_surface_is_preserved(self):
        self.assertIs(self.task_actor.vehicle, self.vehicle)
        self.assertIs(self.task_actor.network, self.network)
        self.assertIs(self.task_actor.logger, self.logger)
        self.assertFalse(self.task_actor.has_selected_pois())

        selected = TaskAssignMsgData(
            task_id=7,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(1.0, 2.0, 3.0),
        )
        self.assertTrue(self.task_actor._selection.try_accept(selected, owner_id=3))
        self.assertTrue(self.task_actor.has_selected_pois())

    def test_shutdown_stops_auctions_when_checkout_raises(self):
        # _presence.stop is mocked below, so neither shutdown() nor tearDown's
        # reset() joins the real heartbeat daemon started in setUp; join it via
        # the class at cleanup so it does not outlive the test.
        presence = self.task_actor._presence
        self.addCleanup(lambda: type(presence).stop(presence))
        self.task_actor._presence.stop = Mock()
        self.task_actor.checkout = Mock(side_effect=OSError("checkout failed"))
        self.task_actor._auction_state.shutdown = Mock()

        with self.assertRaisesRegex(OSError, "checkout failed"):
            self.task_actor.shutdown()

        self.task_actor._auction_state.shutdown.assert_called_once_with(self.logger)

    def test_start_idempotent(self):
        """Calling start() twice does not create duplicate heartbeat threads."""
        # start() already called in setUp — call it again
        first_thread = self.task_actor._presence.heartbeat_thread()
        self.assertTrue(first_thread.is_alive())

        self.task_actor.start()  # second call

        # Same thread object — no new thread spawned
        self.assertIs(
            self.task_actor._presence.heartbeat_thread(),
            first_thread,
        )

    def test_full_cycle_detect_assign_navigate_reset_repeat(self):
        """
        Full realistic cycle:
        1. Detect POI → broadcast → peer responds → assign → peer accepts (navigates)
        2. Reset all
        3. Detect NEW POI → broadcast → peer responds → assign → peer accepts
        Verifies no stale state blocks the second cycle.
        """
        msg_filter = MessageFilter()
        self.network.message_filter = msg_filter

        gimbal_data = Mock(spec=GimbalData)
        vehicle_attitude = Mock(spec=Attitude)

        # ---- Phase 1: detect POI, peer navigates ----

        # Peer 5 announces via heartbeat
        self._discover(5, 6)
        self.assertEqual(self.task_actor._rebroadcast.known_peers(), {5, 6})

        # Detect POI (realistic entry point)
        poi1 = make_detected_poi(
            obj_id=1, x_error=0.0, y_error=0.0,
            reference_height_m=1.0, k=1.0,
            g_data=gimbal_data, uas_att=vehicle_attitude,
            t_g_loc_debug=Location(12.34, 56.78, 90.0),
        )
        poi1.replace_classification(replace(
            poi1.classification,
            size_class=DetectionSizeClass.S,
        ))
        poi1.set_p_t_g_loc(poi1.geo.truth_poi_location)

        self.task_actor.notify_pois([poi1])
        self.assertIn(1, self.task_actor._auction_state.task_ids())
        td1 = self._dispatch(1)
        td1.cancel_rebroadcast()  # prevent timer threads in test

        # Verify "Broadcast available tasks" log in cycle 1
        self.logger.info.assert_any_call("Broadcast available tasks.")

        # Peer 5 says it can handle the task
        td1.on_peer_available(5, TaskHandleMsgData(task_id=1, time_in_min=2.0))
        td1.on_peer_available(6, TaskHandleMsgData(task_id=1, time_in_min=3.0))

        # Assign peer
        self._select_peer(1)
        self.assertEqual(td1.status, TaskDispatchStatus.CONFIRMING)
        self.assertEqual(td1.assigned_peer, 5)

        # Peer accepts → navigates
        accept1 = TaskAssignResponseMsg(
            sender_id=5, receiver_id=1, task_id=1, is_accepted=True,
        )
        self.task_actor.on_message(accept1)
        self.assertEqual(td1.status, TaskDispatchStatus.CONFIRMED)

        # ---- Reset all ----
        self.task_actor.reset()

        # Verify clean slate
        self.assertEqual(len(self.task_actor._auction_state.task_ids()), 0)
        self.assertIsNone(self.task_actor.selected_poi())
        self.assertEqual(len(self.task_actor._rebroadcast.known_peers()), 0)
        self.assertFalse(self.task_actor._presence.is_started())
        self.assertIsNone(msg_filter.offset_estimator.get_offset(5))

        # ---- Phase 2: start fresh, detect new POI, assign again ----
        self.task_actor.start()

        # Detect NEW POI BEFORE any heartbeats arrive (the post-reset scenario)
        poi2 = make_detected_poi(
            obj_id=2, x_error=0.0, y_error=0.0,
            reference_height_m=1.0, k=1.0,
            g_data=gimbal_data, uas_att=vehicle_attitude,
            t_g_loc_debug=Location(23.45, 67.89, 100.0),
        )
        poi2.replace_classification(replace(
            poi2.classification,
            size_class=DetectionSizeClass.M,
        ))
        poi2.set_p_t_g_loc(poi2.geo.truth_poi_location)

        self.logger.info.reset_mock()
        self.network.broadcast.reset_mock()
        self.task_actor.notify_pois([poi2])
        self.assertIn(2, self.task_actor._auction_state.task_ids())
        td2 = self._dispatch(2)

        # Initial broadcast was sent even with no known peers
        self.network.broadcast.assert_called()
        # Verify "Broadcast available tasks" log in cycle 2
        self.logger.info.assert_any_call("Broadcast available tasks.")

        # No known peers — rebroadcast timer NOT started
        self.assertIsNone(td2._rebroadcast_timer)

        # Now peer 5 re-announces — should trigger rebroadcast for available task
        rebroadcast_timer = Mock()
        rebroadcast_timer.is_alive.return_value = True
        with patch(
            "navpy.modules.swarm.task_dispatch.threading.Timer",
            return_value=rebroadcast_timer,
        ) as timer_factory:
            self._discover(5, 6)
        self.assertEqual(self.task_actor._rebroadcast.known_peers(), {5, 6})
        timer_factory.assert_called_once()
        self.assertEqual(timer_factory.call_args.args[0], 0.0)
        rebroadcast_timer.start.assert_called_once_with()
        self.assertIs(td2._rebroadcast_timer, rebroadcast_timer)
        td2.cancel_rebroadcast()  # prevent timer threads in test

        # Peer 5 responds
        td2.on_peer_available(5, TaskHandleMsgData(task_id=2, time_in_min=3.0))
        td2.on_peer_available(6, TaskHandleMsgData(task_id=2, time_in_min=4.0))

        # Assign peer
        self.network.broadcast.reset_mock()
        self._select_peer(2)
        self.assertEqual(td2.status, TaskDispatchStatus.CONFIRMING)
        self.assertEqual(td2.assigned_peer, 5)

        # Verify assign request sent
        self.network.broadcast.assert_called_once()
        sent = self.network.broadcast.call_args[0][0]
        self.assertIsInstance(sent, TaskAssignRequestMsg)
        self.assertEqual(sent.receiver_id, 5)

        # Peer accepts again
        accept2 = TaskAssignResponseMsg(
            sender_id=5, receiver_id=1, task_id=2, is_accepted=True,
        )
        self.task_actor.on_message(accept2)
        self.assertEqual(td2.status, TaskDispatchStatus.CONFIRMED)
        self.assertEqual(td2.assigned_peer, 5)


if __name__ == '__main__':
    unittest.main()
