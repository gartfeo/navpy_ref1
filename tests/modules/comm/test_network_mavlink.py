"""Direct companion-system identity for MAVLink network messages."""

from unittest.mock import MagicMock

from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskResponseMsg,
    TaskAssignMsgData,
    TaskAssignRequestMsg,
    TaskConfirmResponseMsg,
    TaskHandleMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.comm.network_mavlink import NetworkMavlink


def _network(node_id: int):
    vehicle = MagicMock()
    network = NetworkMavlink(node_id=node_id, vehicle=vehicle, logger=MagicMock())
    return network, vehicle


def test_companion_poied_confirm_response_keeps_wire_id():
    network, _ = _network(101)
    listener = MagicMock()
    network.set_listener(listener)
    mav_msg = TaskConfirmResponseMsg(
        receiver_id=101,
        task_id=7,
        is_confirmed=True,
    ).to_mavlink()

    network._on_mavlink(mav_msg)

    delivered = listener.on_message.call_args.args[0]
    assert delivered.receiver_id == 101
    assert delivered.task_id == 7


def test_available_response_keeps_companion_sender_and_receiver_ids():
    network, _ = _network(101)
    listener = MagicMock()
    network.set_listener(listener)
    mav_msg = AvailableTaskResponseMsg(
        sender_id=102,
        receiver_id=101,
        tasks=[TaskHandleMsgData(task_id=7, time_in_min=1.5)],
    ).to_mavlink()
    mav_msg._header.srcSystem = 102

    network._on_mavlink(mav_msg)

    delivered = listener.on_message.call_args.args[0]
    assert delivered.sender_id == 102
    assert delivered.receiver_id == 101


def test_remove_listener_is_exact_and_idempotent():
    network, _ = _network(101)
    retained = MagicMock()
    removed = MagicMock()
    network.set_listener(retained)
    network.set_listener(removed)

    network.remove_listener(removed)
    network.remove_listener(removed)

    assert network.listeners == [retained]


def test_assignment_broadcast_targets_companion_id_directly():
    network, vehicle = _network(101)
    message = TaskAssignRequestMsg(
        sender_id=101,
        receiver_id=102,
        task=TaskAssignMsgData(
            task_id=7,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(40.0, 44.0, 1000.0),
        ),
    )

    network.broadcast(message)

    sent = vehicle.send_mavlink_message.call_args.args[0]
    assert sent.target_system == 102
    assert message.sender_id == 101
    assert message.receiver_id == 102


def test_close_cancels_vehicle_subscription_once_and_rejects_delivery():
    network, vehicle = _network(101)
    subscription = vehicle.on_message.return_value
    listener = MagicMock()
    network.set_listener(listener)
    mav_msg = TaskConfirmResponseMsg(
        receiver_id=101,
        task_id=7,
        is_confirmed=True,
    ).to_mavlink()

    network.close()
    network.close()
    network._on_mavlink(mav_msg)

    subscription.cancel.assert_called_once_with()
    listener.on_message.assert_not_called()
