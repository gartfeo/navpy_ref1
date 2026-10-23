"""NAVLINK listeners never run on, or wait on, the vehicle's MAVLink reader.

Todo 31: the GCS confirm response for a dock reached NetworkMavlink on the
vehicle reader thread while the confirmation round's image send held the
round's outbound fence (thumbnail, 132 chunks paced 5 ms apart, artifacts).
The response handler waited for the fence, so the reader read nothing for
0.56-0.71 s wall (5-7 s aircraft time at 10x). Final approach then started on
that buffered backlog, aircraft time ran ahead of the camera frames, and the
command failed ``source_command_stale``.
"""

import threading
from unittest.mock import MagicMock

import pytest

from navpy.core.link.task_confirmation_msg import TaskConfirmResponseMsg
from navpy.core.review.confirmation_inbound import ConfirmationInboundHandler
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink
from tests.detection_factory import make_detected_poi


@pytest.fixture
def make_network():
    networks: list[NetworkMavlink] = []

    def make(logger=None):
        vehicle = MagicMock()
        network = NetworkMavlink(
            node_id=101, vehicle=vehicle, logger=logger or MagicMock(),
        )
        networks.append(network)
        name, reader_callback = vehicle.on_message.call_args.args
        assert name == "NAVLINK"
        return network, reader_callback

    yield make
    for network in networks:
        network.close()


def _confirm_response(task_id: int):
    return TaskConfirmResponseMsg(
        receiver_id=101,
        task_id=task_id,
        is_confirmed=True,
    ).to_mavlink()


def test_reader_does_not_wait_for_a_response_fenced_by_the_image_send(
    make_network,
):
    network, reader_callback = make_network()
    state = ConfirmationManagerState()
    poi = make_detected_poi(obj_id=7, task_id=7, class_id=0)
    worker = state.start_review(poi, threading.Event)
    confirmation = state.begin_round(poi, worker, threading.Event)
    assert confirmation is not None
    network.set_listener(ConfirmationInboundHandler(
        sys_id=101, state=state, media=MagicMock(), logger=MagicMock(),
    ))
    send_entered = threading.Event()
    release_send = threading.Event()

    def image_send() -> None:
        send_entered.set()
        if not release_send.wait(5.0):
            raise TimeoutError("test did not release the image send")

    sender = threading.Thread(
        target=state.send_if_current, args=(confirmation, image_send),
    )
    sender.start()
    try:
        assert send_entered.wait(1.0)
        reader = threading.Thread(
            target=reader_callback, args=(_confirm_response(7),),
        )
        reader.start()
        reader.join(timeout=1.0)
        reader_returned = not reader.is_alive()
        resolved_during_send = confirmation.response_event.is_set()
    finally:
        release_send.set()
        sender.join(timeout=1.0)

    assert reader_returned, "the vehicle reader waited for the image send"
    # The round's fence still holds: the response resolves after the send.
    assert not resolved_during_send
    assert confirmation.response_event.wait(1.0)
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


def test_listeners_get_reader_messages_in_arrival_order_off_the_reader(
    make_network,
):
    network, reader_callback = make_network()
    delivered: list[tuple[int, bool]] = []
    all_delivered = threading.Event()
    reader_thread = threading.current_thread()

    class Listener:
        def on_message(self, message) -> None:
            delivered.append(
                (message.task_id, threading.current_thread() is reader_thread)
            )
            if len(delivered) == 3:
                all_delivered.set()

    network.set_listener(Listener())

    for task_id in (1, 2, 3):
        reader_callback(_confirm_response(task_id))

    assert all_delivered.wait(1.0)
    assert delivered == [(1, False), (2, False), (3, False)]


def test_close_stops_dispatch_and_drops_messages_queued_after_it(
    make_network,
):
    network, reader_callback = make_network()
    listener = MagicMock()
    network.set_listener(listener)

    network.close()
    reader_callback(_confirm_response(1))

    assert not network._dispatcher.is_alive()
    listener.on_message.assert_not_called()


def test_a_message_whose_dispatch_raises_does_not_end_delivery(make_network):
    logger = MagicMock()
    # The listener error is reported through the logger; a raising logger
    # lets the exception escape _on_mavlink into the dispatch thread.
    logger.error.side_effect = RuntimeError("log sink failed")
    network, reader_callback = make_network(logger)
    delivered: list[int] = []
    second_delivered = threading.Event()

    class Listener:
        def on_message(self, message) -> None:
            delivered.append(message.task_id)
            if message.task_id == 1:
                raise ValueError("listener failed")
            second_delivered.set()

    network.set_listener(Listener())

    reader_callback(_confirm_response(1))
    reader_callback(_confirm_response(2))

    assert second_delivered.wait(1.0)
    assert delivered == [1, 2]
    assert network._dispatcher.is_alive()


def test_a_listener_may_close_the_network(make_network):
    network, reader_callback = make_network()
    closed = threading.Event()

    class Listener:
        def on_message(self, _message) -> None:
            network.close()
            closed.set()

    network.set_listener(Listener())

    reader_callback(_confirm_response(1))

    assert closed.wait(1.0)
    network._dispatcher.join(timeout=1.0)
    assert not network._dispatcher.is_alive()


def test_failed_subscription_starts_no_dispatch_thread():
    vehicle = MagicMock()
    vehicle.on_message.side_effect = RuntimeError("bus closed")
    before = {thread.ident for thread in threading.enumerate()}

    with pytest.raises(RuntimeError):
        NetworkMavlink(node_id=101, vehicle=vehicle, logger=MagicMock())

    assert {thread.ident for thread in threading.enumerate()} == before
