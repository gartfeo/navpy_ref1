from unittest.mock import Mock, patch

import pytest

from navpy.modules.nav.nav_network import NavNetworkRuntime


@pytest.mark.parametrize("component_name", ["peer_dispatch", "task_actor"])
def test_network_runtime_persists_component_failure_after_detach(
    component_name: str,
) -> None:
    runtime = NavNetworkRuntime(
        Mock(),
        Mock(),
        Mock(),
        Mock(),
        Mock(),
        Mock(),
    )
    failure = RuntimeError(f"{component_name} failed")
    component = Mock()
    component.raise_if_failed.side_effect = failure
    setattr(runtime, component_name, component)

    with pytest.raises(RuntimeError) as first:
        runtime.raise_if_failed()
    assert first.value is failure

    setattr(runtime, component_name, None)
    for _ in range(2):
        with pytest.raises(RuntimeError) as raised:
            runtime.raise_if_failed()
        assert raised.value is failure


@pytest.mark.parametrize("component_name", ["peer_dispatch", "task_actor"])
def test_disconnect_harvests_component_failure_before_detach(
    component_name: str,
) -> None:
    confirmation_manager = Mock()
    runtime = NavNetworkRuntime(
        Mock(),
        Mock(),
        Mock(),
        confirmation_manager,
        Mock(),
        Mock(),
    )
    failure = RuntimeError(f"{component_name} failed during shutdown")
    component = Mock()
    component.raise_if_failed.side_effect = failure
    setattr(runtime, component_name, component)

    runtime.stop()

    assert getattr(runtime, component_name) is None
    with pytest.raises(RuntimeError) as raised:
        runtime.raise_if_failed()
    assert raised.value is failure


def test_disconnect_retains_dependencies_when_peer_worker_is_not_quiescent() -> None:
    confirmation_manager = Mock()
    runtime = NavNetworkRuntime(
        Mock(),
        Mock(),
        Mock(),
        confirmation_manager,
        Mock(),
        Mock(),
    )
    timeout = TimeoutError("peer dispatch is still active")
    worker = Mock()
    worker.stop.side_effect = timeout
    actor = Mock()
    network = Mock()
    listener = Mock()
    runtime.peer_dispatch = worker
    runtime.task_actor = actor
    runtime._network = network
    runtime._override_listener = listener

    with pytest.raises(ExceptionGroup) as raised:
        runtime.stop()

    assert raised.value.exceptions == (timeout,)
    network.remove_listener.assert_not_called()
    confirmation_manager.set_network.assert_not_called()
    actor.shutdown.assert_not_called()
    assert runtime.peer_dispatch is worker
    assert runtime.task_actor is actor
    assert runtime._network is network
    assert runtime._override_listener is listener


def test_set_network_owns_peer_worker_before_start_failure() -> None:
    confirmation_manager = Mock()
    runtime = NavNetworkRuntime(
        Mock(),
        Mock(),
        Mock(),
        confirmation_manager,
        Mock(),
        Mock(),
    )
    network = Mock()
    actor = Mock()
    listener = Mock()
    worker = Mock()
    failure = RuntimeError("peer worker launch failed")
    worker.start.side_effect = failure

    with patch(
        "navpy.modules.nav.nav_network.TaskActor",
        return_value=actor,
    ), patch(
        "navpy.modules.nav.nav_network.ConfirmOverrideListener",
        return_value=listener,
    ), patch(
        "navpy.modules.nav.nav_network.PeerPoiDispatchWorker",
        return_value=worker,
    ):
        with pytest.raises(RuntimeError) as raised:
            runtime.set_network(network)

    assert raised.value is failure
    assert runtime.peer_dispatch is worker
    assert runtime.task_actor is actor
    assert runtime._network is network

    worker.start.side_effect = None
    runtime.stop()
    worker.stop.assert_called_once_with()
    actor.shutdown.assert_called_once_with()
