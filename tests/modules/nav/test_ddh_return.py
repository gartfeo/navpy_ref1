"""A failed one-shot dock returns to the DDH (owner rulings 2026-11-09; docs/design/navpy_recovery.md step 10, item 3).

ArduPilot's RTL takes the UAV from NavPy, which then cannot park it. A healthy NavPy flies to the DDH itself, takes no
other work on the way (no peer task, no delivery dock, its swarm slot busy), circles the hub and parks there on the
operator's confirmation. The return is a latch in the task state (RecoveryHold.sortie_returning).
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.app.flow.nav_delivery_composition import _start_hub_return
from navpy.app.flow.nav_runtime_composition import _participating, _return_to_search
from navpy.core.flow.nav_recovery_ports import RecoveryHold
from navpy.core.flow.nav_state import NavState
from navpy.core.kernel.location import Location
from navpy.products.delivery.fallback_mission_navigation import FallbackMissionNavigation
from navpy.products.delivery.home_dock import HomeDockRules
from navpy.products.delivery.mission_catalog import MissionCatalog
from navpy.products.delivery.work_source import DeliveryWorkSource, HubReturnPorts

HUB = Location(47.0, 8.0, 0.0)
FAR = Location(47.01, 8.0, 0.0)


def _poi(location: Location) -> SimpleNamespace:
    return SimpleNamespace(location=location)


def _rules(*, homing: bool = False, returning: bool = False) -> HomeDockRules:
    focus = SimpleNamespace(dock_location=lambda poi: poi.location, assigned=lambda: False)
    return HomeDockRules(focus, lambda: HUB, lambda: homing, lambda: returning)


def test_returning_puts_the_home_dock_first():
    hub, delivery = _poi(HUB), _poi(FAR)
    assert _rules(returning=True).order([delivery, hub]) == [hub, delivery]
    assert _rules(homing=True).order([hub, delivery]) == [delivery, hub]      # today's mission-end return
    assert _rules().order([hub, delivery]) == [hub, delivery]


def _work_source(*, returning=True, can_return=True, rearmed=False, peer_pending=True):
    fallback = Mock()
    fallback.can_return.return_value = can_return
    fallback.should_nav_to_fallback.return_value = False
    peer_task = Mock()
    peer_task.pending.return_value = peer_pending
    peer_navigation, tasks = Mock(), Mock()
    rules = _rules(returning=returning)
    source = DeliveryWorkSource(
        SimpleNamespace(peer_navigation=False), peer_task, peer_navigation, fallback, tasks,
        HubReturnPorts(returning=lambda: returning, is_home_dock=rules.is_home_dock, rearmed=lambda: rearmed),
    )
    return source, fallback, peer_task, peer_navigation, tasks


def test_returning_takes_no_peer_task_and_no_delivery_dock_and_flies_to_the_hub():
    source, fallback, peer_task, peer_navigation, tasks = _work_source()
    source.handle_new_poi(_poi(FAR))
    peer_task.hand_off.assert_not_called()
    peer_navigation.setup.assert_not_called()
    tasks.try_start.assert_not_called()
    fallback.setup.assert_called_once_with()           # from wherever the mission is


def test_returning_starts_the_home_dock():
    source, fallback, _, peer_navigation, tasks = _work_source()
    hub = _poi(HUB)
    source.handle_new_poi(hub)
    tasks.try_start.assert_called_once_with(hub)
    fallback.setup.assert_not_called()
    peer_navigation.setup.assert_not_called()


@pytest.mark.parametrize("rearmed, setups", [(False, 0), (True, 1)])
def test_returning_flies_the_transit_again_only_after_a_hold_rearmed_it(rearmed, setups):
    source, fallback, *_ = _work_source(can_return=False, rearmed=rearmed)
    source.handle_new_poi(None)
    assert fallback.setup.call_count == setups


def test_not_returning_keeps_todays_order_of_work():
    source, fallback, peer_task, peer_navigation, _ = _work_source(returning=False)
    source.handle_new_poi(None)
    peer_navigation.setup.assert_called_once_with()    # the peer task outranks the hub, as before
    fallback.setup.assert_not_called()


def test_the_return_bypasses_the_last_waypoint_rule_but_not_a_hub_already_flown_to():
    catalog = MissionCatalog()
    ports = SimpleNamespace(next_waypoint=lambda: 1, mission_item_count=lambda: 10)
    fallback = FallbackMissionNavigation(ports, catalog, Mock(), Mock(), Mock(), Mock())
    assert not fallback.can_return()                    # no DDH in the mission
    catalog.default_delivery_hub = HUB
    assert fallback.can_return() and not fallback.should_nav_to_fallback()
    catalog.mark_fallback_active()
    assert not fallback.can_return()


def test_the_return_starts_only_with_a_ddh_in_the_mission():
    hold, catalog = RecoveryHold(), MissionCatalog()
    assert not _start_hub_return(hold, catalog) and not hold.sortie_returning
    catalog.default_delivery_hub = HUB
    assert _start_hub_return(hold, catalog) and hold.sortie_returning


@pytest.mark.parametrize("holding, returning, restarted", [
    (False, False, True), (True, False, False), (False, True, False),
])
def test_leaving_docked_on_the_return_never_restarts_the_auto_search(holding, returning, restarted):
    vehicle = Mock()
    hold = RecoveryHold()
    hold.holding = holding
    if returning:
        hold.start_sortie_return()
    _return_to_search(vehicle, SimpleNamespace(recovery_hold=hold))
    assert vehicle.restart_mission.called is restarted


def test_the_swarm_slot_stays_busy_while_returning():
    hold = RecoveryHold()
    assert not _participating(NavState.DETECT, hold)
    hold.start_sortie_return()
    assert _participating(NavState.DETECT, hold)
    hold.end_sortie_return()
    assert not _participating(NavState.DETECT, hold)
