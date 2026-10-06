"""Verified lease behavior for the simulator-only NAV speedup."""

from unittest.mock import Mock, call

from navpy.modules.nav.navigation_speedup import (
    NavigationSpeedupLease,
    NavigationSpeedupState,
    SimSpeedupParameterPort,
)


def _lease(vehicle, sync, logger) -> NavigationSpeedupLease:
    return NavigationSpeedupLease(
        parameter=SimSpeedupParameterPort(
            read=lambda: vehicle.get_parameter("SIM_SPEEDUP"),
            write=lambda value: vehicle.set_parameter("SIM_SPEEDUP", value),
        ),
        requested_speedup=lambda: 10.0,
        sync_scheduler_cadence=sync,
        logger=logger,
    )


def test_missing_baseline_is_retried_before_verified_apply():
    vehicle = Mock()
    vehicle.get_parameter.side_effect = [None, 1.0]
    vehicle.set_parameter.return_value = True
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is False
    vehicle.set_parameter.assert_not_called()
    sync.assert_not_called()
    assert lease.apply() is True

    assert lease.original == 1.0
    assert vehicle.get_parameter.call_count == 2
    vehicle.set_parameter.assert_called_once_with("SIM_SPEEDUP", 10.0)
    sync.assert_called_once_with()


def test_lost_request_echo_immediately_restores_mutated_autopilot():
    vehicle = Mock()
    autopilot = {"speedup": 1.0}
    observed_states = []

    def set_parameter(_name, value):
        observed_states.append(lease.state)
        autopilot["speedup"] = value
        return value == 1.0

    vehicle.get_parameter.side_effect = lambda _name: autopilot["speedup"]
    vehicle.set_parameter.side_effect = set_parameter
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is True

    assert autopilot["speedup"] == 1.0
    assert vehicle.set_parameter.call_args_list == [
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 1.0),
    ]
    assert observed_states == [
        NavigationSpeedupState.ROLLBACK_REQUIRED,
        NavigationSpeedupState.ROLLBACK_REQUIRED,
    ]
    sync.assert_called_once_with()
    assert lease.state is NavigationSpeedupState.REQUEST_SUPPRESSED
    assert lease.original == 1.0

    assert lease.restore(log=True) is True
    assert lease.state is NavigationSpeedupState.NO_LEASE
    assert lease.original is None


def test_unverified_request_and_rollback_are_retried_only_at_baseline():
    vehicle = Mock()
    vehicle.get_parameter.return_value = 1.0
    vehicle.set_parameter.side_effect = [False, False, True]
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is False
    assert lease.state is NavigationSpeedupState.ROLLBACK_REQUIRED
    assert lease.restore(log=True) is True

    assert vehicle.set_parameter.call_args_list == [
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 1.0),
        call("SIM_SPEEDUP", 1.0),
    ]
    assert call("SIM_SPEEDUP", 10.0) not in vehicle.set_parameter.call_args_list[1:]
    sync.assert_called_once_with()
    assert lease.state is NavigationSpeedupState.NO_LEASE
    assert lease.original is None


def test_apply_retries_pending_rollback_then_suppresses_request_for_episode():
    vehicle = Mock()
    vehicle.get_parameter.return_value = 1.0
    vehicle.set_parameter.side_effect = [False, False, True]
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is False
    assert lease.apply() is True
    assert lease.apply() is True

    assert vehicle.set_parameter.call_args_list == [
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 1.0),
        call("SIM_SPEEDUP", 1.0),
    ]
    assert lease.state is NavigationSpeedupState.REQUEST_SUPPRESSED
    sync.assert_called_once_with()


def test_failed_restore_keeps_baseline_and_retries_until_verified():
    vehicle = Mock()
    vehicle.get_parameter.return_value = 1.0
    vehicle.set_parameter.side_effect = [True, False, True]
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is True
    assert lease.restore(log=True) is False
    assert lease.original == 1.0
    assert lease.restore(log=True) is True
    assert lease.restore(log=True) is True

    assert vehicle.set_parameter.call_args_list == [
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 1.0),
        call("SIM_SPEEDUP", 1.0),
    ]
    assert sync.call_count == 2
    assert lease.original is None
    assert lease.state is NavigationSpeedupState.NO_LEASE


def test_each_new_lease_episode_captures_the_current_external_baseline():
    vehicle = Mock()
    vehicle.get_parameter.side_effect = [1.0, 3.0]
    vehicle.set_parameter.return_value = True
    sync = Mock()
    lease = _lease(vehicle, sync, Mock())

    assert lease.apply() is True
    assert lease.restore(log=False) is True
    assert lease.apply() is True
    assert lease.restore(log=False) is True

    assert vehicle.set_parameter.call_args_list == [
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 1.0),
        call("SIM_SPEEDUP", 10.0),
        call("SIM_SPEEDUP", 3.0),
    ]
    assert lease.original is None
    assert sync.call_count == 4


def test_disabled_speedup_performs_no_parameter_or_cadence_io():
    vehicle = Mock()
    sync = Mock()
    logger = Mock()
    lease = NavigationSpeedupLease(
        parameter=SimSpeedupParameterPort(
            read=lambda: vehicle.get_parameter("SIM_SPEEDUP"),
            write=lambda value: vehicle.set_parameter("SIM_SPEEDUP", value),
        ),
        requested_speedup=lambda: 0.0,
        sync_scheduler_cadence=sync,
        logger=logger,
    )

    assert lease.apply() is True

    vehicle.get_parameter.assert_not_called()
    vehicle.set_parameter.assert_not_called()
    sync.assert_not_called()
    logger.info.assert_not_called()
    logger.warning.assert_not_called()
    assert lease.state is NavigationSpeedupState.NO_LEASE
