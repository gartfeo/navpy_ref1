"""DOCKED final state (docs/design/docked_state.md), one mechanism per test."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.approach.logs.navigation_snap_types import ClosestSnap
from navpy.core.approach.nav_exit_cleanup import NavCleanupResult
from navpy.core.flow.nav_constants import ALT_HYST
from navpy.core.flow.nav_state import (
    ConfirmGateState,
    NavigationTaskState,
    NavPhaseState,
    NavState,
    PostDockClimb,
)
from navpy.core.flow.nav_transition import NavExitOutcome, NavTransition
from navpy.core.flow.navigation_decision import NavigationDecision
from navpy.core.flow.navigation_transition_handler import (
    NavigationTransitionHandler,
    TransitionPorts,
)
from navpy.core.flow.recovery import RecoveryAction, RecoveryPorts
from navpy.core.kernel.location import Location
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.swarm_ack_msg import ACK_STATUS_APPLIED
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.link.task_message_data import TaskAssignMsgData
from navpy.core.link.types import TaskTypeMsgData
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.core.swarm.task_actor_slots import (
    AckKind,
    RequestKind,
    SelectedTaskSlot,
    SlotState,
)
from navpy.core.swarm.task_msg_refs import MsgRef

DOCK = 4
IDENTITY = DockContactIdentity(vehicle=1, task_id=10, dock_uid=DOCK, episode=1)
MIN_ALT = 40.0


def _contact(identity=IDENTITY, closest_m: float = 0.05) -> DockContactEvent:
    return DockContactEvent(
        identity=identity,
        aircraft_time_s=12.0,
        closest_3d_m=closest_m,
        closest_h_m=closest_m,
        closest_v_m=0.0,
    )


# The episode keeps its latched contact for NAV exit


class _Sensor:
    def __init__(self):
        self.on_docked = None

    def arm(self, identity, on_docked):
        self.on_docked = on_docked

    def disarm(self, docked=False):
        pass


def _episode(*, latch=True):
    sensor = _Sensor()
    poi = SimpleNamespace(identity=SimpleNamespace(obj_id=DOCK, task_id=10))
    episode = FinalApproachDockContact(
        sensor, lambda: 1, lambda _identity, _s: True,
        lambda _event: latch, Mock(),
    )
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(
            "navpy.core.approach.dock_contact_episode.get_poi_task_id",
            lambda _poi: 10,
        )
        episode.arm(poi)
    return episode, sensor


def test_disarm_returns_the_latched_contact_of_the_armed_episode():
    episode, sensor = _episode()
    sensor.on_docked(_contact())

    assert episode.disarm() == _contact()
    assert episode.disarm() is None  # taken once


@pytest.mark.parametrize(
    "latch, event",
    [
        (False, _contact()),  # the latch refused (NAV_FAIL won the race)
        (True, _contact(DockContactIdentity(1, 10, DOCK, 2))),  # stale episode
    ],
)
def test_no_contact_for_a_refused_latch_or_another_episode(latch, event):
    episode, sensor = _episode(latch=latch)
    sensor.on_docked(event)

    assert episode.disarm() is None


def test_a_new_arm_drops_the_previous_contact():
    episode, sensor = _episode()
    sensor.on_docked(_contact())

    episode.arm(None)

    assert episode.disarm() is None


# NAV exit redirect


def _exit(
    docked=_contact(),
    *,
    requested=NavState.RESET,
    oneshot=False,
    completed=False,
    errors=(),
    snap=None,
):
    events: list = []
    navigation_task = NavigationTaskState(
        final_approach_navigation_active=True,
        final_approach_nav_completed=completed,
    )
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(
        snap=ClosestSnap(dist=4.4) if snap is None else snap,
        errors=tuple(errors),
        own_dock_contact=docked,
    )
    completion = Mock()
    completion.run.side_effect = lambda: events.append("oneshot") or ()
    logger = Mock()
    transition = NavTransition(
        Mock(), cleanup, Mock(), completion.run, navigation_task,
        lambda: oneshot, logger=logger,
    )
    return transition.exit(requested), events, logger


def test_an_own_dock_contact_redirects_to_docked_whatever_the_snap():
    # SITL: contact 0.07 m while the command-sampled SNAP read 4.4 m.
    outcome, events, logger = _exit(_contact(closest_m=0.07))

    assert outcome == NavExitOutcome(redirect=NavState.DOCKED)
    assert events == []
    message = logger.info.call_args.args[0]
    assert message.startswith("STATE DOCKED (sim): 3d=0.07m dock=4")


def test_no_contact_keeps_todays_flow_even_with_a_close_snap():
    outcome, _, logger = _exit(None, snap=ClosestSnap(dist=0.1))

    assert outcome == NavExitOutcome()
    logger.info.assert_not_called()


@pytest.mark.parametrize("requested", [NavState.ONHOLD, NavState.DETECT, None])
def test_only_a_nav_decision_ending_docks(requested):
    # Inactive guard (ONHOLD) or GUIDED departure (DETECT): the operator wins.
    outcome, _, _ = _exit(requested=requested)
    assert outcome == NavExitOutcome()


def test_one_shot_docked_disarms_even_without_a_completion_flag():
    outcome, events, _ = _exit(oneshot=True, completed=False)

    assert outcome == NavExitOutcome(
        redirect=NavState.DOCKED, oneshot_completed=True,
    )
    assert events == ["oneshot"]


def test_one_shot_completion_without_contact_keeps_onhold():
    outcome, events, _ = _exit(None, oneshot=True, completed=True)

    assert outcome == NavExitOutcome(
        redirect=NavState.ONHOLD, oneshot_completed=True,
    )
    assert events == ["oneshot"]


def test_a_cleanup_error_raises_and_never_docks():
    with pytest.raises(Exception, match="NAV exit failed"):
        _exit(oneshot=True, errors=(RuntimeError("source"),))


# Transition hooks


def _handler(*, oneshot: bool):
    climb = PostDockClimb()
    ports = TransitionPorts(
        decision_clock_s=lambda: 0.0,
        return_to_search=Mock(),
        enter_recovery=Mock(),
        start_task_actor=Mock(),
        is_oneshot=lambda: oneshot,
    )
    nav = Mock()
    nav.exit.return_value = NavExitOutcome(
        redirect=NavState.DOCKED, oneshot_completed=oneshot,
    )
    reset = Mock()
    handler = NavigationTransitionHandler(
        ports, ConfirmGateState(), nav, reset, Mock(), climb,
    )
    return handler, ports, nav, reset, climb


@pytest.mark.parametrize("oneshot, clears", [(False, 1), (True, 0)])
def test_docked_entry_runs_the_reset_transaction_once(oneshot, clears):
    # One-shot: OneShotCompletion.run already cleared at NAV exit.
    handler, ports, nav, reset, _ = _handler(oneshot=oneshot)

    outcome = handler.on_change(NavState.NAV, NavState.RESET)

    nav.exit.assert_called_once_with(NavState.RESET)
    assert outcome.effective_state is NavState.DOCKED
    assert reset.clear.call_count == clears
    ports.start_task_actor.assert_called_once_with()


def test_cyclic_climb_latch_lives_from_docked_to_any_recovery_exit():
    handler, ports, _, _, climb = _handler(oneshot=False)

    handler.on_change(NavState.DOCKED, NavState.RECOVERY)
    assert climb.active
    ports.enter_recovery.assert_called_once_with()

    handler.on_change(NavState.RECOVERY, NavState.ONHOLD)  # interrupted
    assert not climb.active
    ports.return_to_search.assert_not_called()


def test_leaving_docked_high_restarts_the_auto_mission():
    handler, ports, _, _, climb = _handler(oneshot=False)

    handler.on_change(NavState.DOCKED, NavState.DETECT)

    ports.return_to_search.assert_called_once_with()
    assert not climb.active


# Recovery after a cyclic dock


class _Recovery:
    def __init__(self, state: NavState, altitude, *, climb=False):
        self.altitude = altitude
        self.phase = NavPhaseState(state, state)
        self.climb = PostDockClimb(active=climb)
        self.ports = RecoveryPorts(
            final_approach_active=lambda: True,  # always so in vision builds
            detector_stop=Mock(),
            current_relative=lambda: (
                None if self.altitude is None
                else Location(0.0, 0.0, self.altitude)
            ),
            pause_navigation=Mock(),
            set_attitude=Mock(),
            max_pitch_deg=lambda: 10.0,
            min_alt=lambda: MIN_ALT,
            is_oneshot=lambda: False,
        )
        self.action = RecoveryAction(
            self.ports, self.phase, Mock(), Mock(), self.climb,
        )


@pytest.mark.parametrize(
    "altitude, expected",
    [
        (MIN_ALT - ALT_HYST - 1.0, NavState.RECOVERY),
        (MIN_ALT, NavState.DETECT),
    ],
)
def test_cyclic_docked_climbs_when_low_else_restarts(altitude, expected):
    recovery = _Recovery(NavState.DOCKED, altitude)

    assert recovery.action.advance() is True
    assert recovery.phase.current is expected


def test_cyclic_docked_waits_for_an_altitude():
    recovery = _Recovery(NavState.DOCKED, None)

    assert recovery.action.advance() is True
    assert recovery.phase.current is NavState.DOCKED


def test_post_dock_recovery_climbs_although_a_final_approach_is_active():
    recovery = _Recovery(NavState.RECOVERY, MIN_ALT - 1.0, climb=True)

    recovery.action.advance()
    assert recovery.phase.current is NavState.RECOVERY

    recovery.altitude = MIN_ALT + ALT_HYST
    recovery.action.advance()
    assert recovery.phase.current is NavState.DETECT


def test_any_other_recovery_keeps_todays_final_approach_behavior():
    recovery = _Recovery(NavState.RECOVERY, MIN_ALT - 1.0, climb=False)

    recovery.action.advance()

    assert recovery.phase.current is NavState.DETECT


# One-shot DOCKED is final


def _decision(*, oneshot: bool):
    phase = NavPhaseState(NavState.DOCKED, NavState.DOCKED)
    inactive, recovery, flow = Mock(), Mock(), Mock()
    for owner in (inactive, recovery):
        owner.apply.return_value = False
        owner.advance.return_value = False
    decision = NavigationDecision(
        SimpleNamespace(
            vehicle_mode=lambda: None,
            next_waypoint=lambda: 0,
            is_oneshot=lambda: oneshot,
        ),
        phase,
        inactive,
        recovery,
        flow,
    )
    return decision, (inactive, recovery, flow)


def test_one_shot_docked_runs_no_other_decision():
    # Not even the inactive guard: the disarm must not make it ONHOLD.
    decision, owners = _decision(oneshot=True)

    decision.decide()

    for owner in owners:
        # mock_calls also records a call of the flow Mock itself.
        assert owner.mock_calls == []


def test_cyclic_docked_lets_the_inactive_guard_decide_first():
    decision, (inactive, recovery, _) = _decision(oneshot=False)

    decision.decide()

    inactive.apply.assert_called_once()
    recovery.advance.assert_called_once_with()


# Swarm: a docked UAV takes no task


OWNER, BOOT = 1, 7
REQUEST = MsgRef(OWNER, BOOT, 50)
TASK = TaskAssignMsgData(
    task_id=10,
    task_type=TaskTypeMsgData.DOCK,
    location=LocationMsgData(1.0, 2.0, 3.0),
)


@pytest.fixture
def slot() -> SelectedTaskSlot:
    return SelectedTaskSlot(threading.RLock())


def test_a_docked_uav_is_busy_and_rejects_offers(slot):
    slot.set_participation(False, True)

    decision = slot.on_request(TASK, REQUEST, flyable=True)

    assert decision.kind is RequestKind.REJECT
    assert decision.reason == "docked"
    assert slot.node_state() is SwarmNodeState.BUSY


def test_nav_to_docked_is_one_update_with_no_free_window(slot):
    assert slot.set_participation(True, False) is None
    assert slot.set_participation(False, True) is None  # still busy: no drop
    assert slot.node_state() is SwarmNodeState.BUSY

    slot.set_participation(False, False)
    assert slot.node_state() is SwarmNodeState.FREE


def test_docking_drops_a_waiting_offer_and_its_late_ack(slot):
    held = slot.on_request(TASK, REQUEST, flyable=True).held
    reply = MsgRef(2, 9, 5)
    assert slot.record_reply(held.token, reply)

    assert slot.set_participation(False, True) == held
    assert slot.state() is SlotState.EMPTY
    kind, _ = slot.on_ack(reply, ACK_STATUS_APPLIED, MsgRef(OWNER, BOOT, 60))
    assert kind is AckKind.NONE


# Integrated: a contact dock after NAV_FAIL through the real dispatcher


def _flow(*, oneshot: bool):
    from navpy.core.approach.nav_exit_cleanup import (
        NavExitCleanup,
        NavExitCleanupPorts,
    )
    from navpy.products.delivery.nav_oneshot_completion import (
        OneShotCompletion,
        OneShotCompletionPorts,
    )
    from navpy.core.flow.nav_task_availability import is_approaching
    from navpy.core.flow.navigation_loop import StateActionDispatcher

    events: list = []
    slot = SelectedTaskSlot(threading.RLock())
    slot.set_participation(True, False)  # flying the approach

    def offer_during_cleanup():
        events.append(("offer", slot.on_request(TASK, REQUEST, True).reason))

    reset = Mock()
    reset.clear.side_effect = lambda: events.append("clear")
    speedup = Mock()
    speedup.restore.return_value = True
    cleanup = NavExitCleanup(
        NavExitCleanupPorts(
            disarm_dock_contact=lambda: events.append("contact off") or _contact(),
            close_final_approach_source=offer_during_cleanup,
            navigation_reset=lambda: events.append("fence") or ClosestSnap(dist=1.9),
            detector_stop=Mock(),
        ),
        speedup,
        Mock(),
    )
    completion = OneShotCompletion(
        OneShotCompletionPorts(
            is_simulated_autopilot=lambda: True,
            is_stopping=lambda: False,
            disarm=lambda: events.append("disarm"),
        ),
        reset,
        Mock(),
    )
    # NAV_FAIL without completion evidence: no completion flag.
    task = NavigationTaskState(final_approach_navigation_active=True)
    transition = NavTransition(
        Mock(), cleanup, Mock(), completion.run, task, lambda: oneshot,
        logger=Mock(),
    )
    climb = PostDockClimb()
    handler = NavigationTransitionHandler(
        TransitionPorts(
            decision_clock_s=lambda: 0.0,
            return_to_search=lambda: events.append("auto"),
            enter_recovery=Mock(),
            start_task_actor=Mock(),
            is_oneshot=lambda: oneshot,
        ),
        ConfirmGateState(),
        transition,
        reset,
        Mock(),
        climb,
    )

    def publish(committed):
        events.append(("publish", committed))
        slot.set_participation(
            is_approaching(committed), committed is NavState.DOCKED,
        )

    phase = NavPhaseState(NavState.NAV, NavState.NAV)
    dispatcher = StateActionDispatcher(
        phase,
        handler,
        {state: (lambda s=state: events.append(("act", s))) for state in NavState},
        publish,
    )
    return SimpleNamespace(
        events=events, slot=slot, phase=phase, act=dispatcher.act, climb=climb,
    )


@pytest.mark.parametrize("oneshot", [True, False])
def test_contact_dock_fences_cleans_once_then_commits_and_publishes_busy(oneshot):
    flow = _flow(oneshot=oneshot)
    flow.phase.request(NavState.RESET)

    flow.act()

    disarm = ["disarm"] if oneshot else []
    assert flow.events == [
        "contact off",
        ("offer", "flying a final approach"),  # still busy during cleanup
        "fence",
        *disarm,
        "clear",
        ("publish", NavState.DOCKED),
        ("act", NavState.DOCKED),
    ]
    assert flow.phase.current is NavState.DOCKED
    assert flow.phase.oneshot_completed is oneshot
    assert flow.slot.node_state() is SwarmNodeState.BUSY
    assert flow.slot.on_request(TASK, REQUEST, True).reason == "docked"


def test_cyclic_dock_climb_is_interruptible_and_the_slot_frees():
    flow = _flow(oneshot=False)
    flow.phase.request(NavState.RESET)
    flow.act()

    flow.phase.request(NavState.RECOVERY)
    flow.act()
    assert flow.climb.active
    assert flow.slot.node_state() is SwarmNodeState.FREE

    flow.phase.request(NavState.ONHOLD)  # disarm or manual mode
    flow.act()
    assert not flow.climb.active
    assert "auto" not in flow.events
