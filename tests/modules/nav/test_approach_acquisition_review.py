"""Lock and confirm gates (HITL step 25): the points of the independent review, rounds 1 and 2.

A cached GPS position cannot hold the orbit; no path writes CONFIRMED while the gates do not hold,
and a review under way is withdrawn when they stop, so a late operator answer cannot confirm; a
repeated frame does not keep the lock past the frame gap; a circle already flown cannot stall the
own ORBIT; and compass heading poisoned at the real final-approach confirmation and law boundary
changes nothing that the gates or the law decide.
"""

from __future__ import annotations

import threading

import pytest

from navpy.core.approach.vision_nav.confirmation import (
    FinalApproachConfirmation,
    FinalApproachConfirmationPorts,
)
from navpy.core.approach.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.core.approach.vision_nav.source_epoch import SourceEpochLedger
from navpy.core.flow.nav_state import NavState
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.ports.law_config import FinalApproachLawConfig, FixedFinalApproachLawConfigProvider
from navpy.core.review.confirmation_manager_state import ConfirmationManagerState
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.review.confirmation_round_transaction import ConfirmationResponseKind
from navpy.core.skills.confirm.acquisition_gate_settings import AcquisitionGateSettings
from navpy.core.skills.confirm.fresh_visual_lock import FreshVisualLock
from navpy.core.skills.confirm.orbit_established import (
    ORBIT_MAX_OBSERVATION_GAP_S,
    OrbitCommand,
    OrbitEstablished,
)
from navpy.plugins.laws.vision_pn.law import VisionNavLaw
from tests.modules.nav.acquisition_replay import GATES, OWN_CENTRE, AcquisitionRig, as_fix, frame, offset

SETTINGS = AcquisitionGateSettings(**GATES)
ON_CIRCLE = offset(OWN_CENTRE, 300.0, 0.0)


# --- P1: a cached GPS position cannot hold the orbit ----------------------------------------------------------------

def _orbit_with(fix_boot):
    """An orbit gate on the circle whose fix carries the boot time ``fix_boot(now)``."""
    now = [0.0]
    gate = OrbitEstablished(SETTINGS, lambda: as_fix(ON_CIRCLE, fix_boot(now[0])), lambda: now[0])
    gate.start(OrbitCommand(OWN_CENTRE, 300.0))
    return gate, now


def _hold(gate, now, start: float, end: float) -> bool:
    result = False
    for i in range(int(round((end - start) / 0.1)) + 1):
        now[0] = round(start + i * 0.1, 6)
        result = gate.established()
    return result


def test_a_cached_fix_cannot_hold_the_orbit() -> None:
    gate, now = _orbit_with(lambda _now: 5.0)  # the same GLOBAL_POSITION_INT on every tick
    assert not _hold(gate, now, 0.0, 30.0)


def test_new_fixes_hold_the_orbit_and_a_stalled_stream_restarts_it() -> None:
    stalled = {"at": None}
    gate, now = _orbit_with(lambda t: t if stalled["at"] is None else stalled["at"])
    assert _hold(gate, now, 0.0, 10.0)
    stalled["at"] = now[0]  # the stream stops; the cached fix stays on the circle
    assert _hold(gate, now, 10.1, 10.0 + ORBIT_MAX_OBSERVATION_GAP_S)
    assert not _hold(gate, now, 11.1, 11.2)
    stalled["at"] = None
    assert not _hold(gate, now, 11.3, 21.2)
    assert _hold(gate, now, 21.3, 21.3)


@pytest.mark.parametrize("fault", ["boot_gap", "boot_back"])
def test_a_broken_fix_clock_restarts_the_dwell(fault) -> None:
    shift = {"s": 0.0}
    gate, now = _orbit_with(lambda t: t + shift["s"])
    assert _hold(gate, now, 0.0, 10.0)
    shift["s"] = 5.0 if fault == "boot_gap" else -20.0
    assert not _hold(gate, now, 10.1, 10.1)
    assert not _hold(gate, now, 10.2, 20.0)
    assert _hold(gate, now, 20.1, 20.2)


def test_a_vehicle_without_stamped_fixes_never_establishes_the_orbit() -> None:
    rig = AcquisitionRig()
    type(rig.rig.vehicle).position_sample = property(lambda _self: None)
    _drive(rig, 0.0, 60.0)
    assert len(rig.orbit_calls()) == 1 and not rig.confirmed() and rig.phase is NavState.CONFIRM


# --- P1: a review under way is withdrawn when the gates stop holding ------------------------------------------------

def _drive(rig: AcquisitionRig, start: float, end: float, *, error: float = 0.5):
    for i in range(int(round((end - start) / 0.1)) + 1):
        ts = round(start + i * 0.1, 6)
        rig.world.t = ts
        rig.world.detections = [frame(ts, error, rig.now)]
        if rig.orbit_calls():
            rig.world.position = ON_CIRCLE
        rig.tick()
        if rig.phase is NavState.NAV:
            return ts
    return None


def _operator_rig() -> AcquisitionRig:
    """Both gates hold at 12.2 s; then an operator review is under way (no local confirm)."""
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 12.1) is None
    manager = rig.rig.confirmation_manager
    manager.update_status(manager.active_poi, ConfirmationStatus.CONFIRMING)
    return rig


def test_a_lost_lock_withdraws_the_review_under_way() -> None:
    rig = _operator_rig()
    manager = rig.rig.confirmation_manager
    assert _drive(rig, 12.2, 12.2, error=1.5) is None  # pointing off: the lock breaks
    assert manager.get_status(manager.active_poi) is None  # withdrawn: asked again once it holds


def test_the_orbit_keeps_its_dwell_while_the_operator_reviews() -> None:
    rig = _operator_rig()
    manager = rig.rig.confirmation_manager
    assert _drive(rig, 12.2, 15.0) is None  # three seconds under review, gates holding
    assert manager.get_status(manager.active_poi) is ConfirmationStatus.CONFIRMING
    manager.update_status(manager.active_poi, ConfirmationStatus.CONFIRMED)  # the operator's answer
    # The orbit was observed on every review tick, so it is still established: NAV on the next tick.
    assert _drive(rig, 15.1, 15.1) == pytest.approx(15.1)


def _review_under_way():
    state = ConfirmationManagerState()
    poi = frame(1.0, 0.5, lambda: 1.0)
    state.acquisition.publish(1, True)
    lease = state.start_review(poi, rt.Event)
    round_ = state.begin_round(poi, lease, rt.Event)
    assert round_ is not None and state.registry.get_status(poi) is ConfirmationStatus.CONFIRMING
    return state, poi, lease, round_


def test_an_answer_to_a_withdrawn_review_is_late() -> None:
    state, poi, lease, round_ = _review_under_way()
    assert state.acquisition.publish(1, False)  # the gates stop holding: withdrawn
    assert state.registry.get_status(poi) is None
    assert state.resolve_response(round_.poi_id, True) is ConfirmationResponseKind.LATE_OR_DUPLICATE
    assert not state.complete_round(round_, ConfirmationStatus.CONFIRMED)  # the worker's timeout path
    assert not state.set_worker_status(lease, ConfirmationStatus.CONFIRMED)  # its auto-confirm path
    assert state.registry.get_status(poi) is None
    assert not state.acquisition.publish(1, False)  # nothing under way any more


@pytest.mark.parametrize("path", ["operator", "timeout", "worker", "local"])
def test_no_path_confirms_while_the_gates_do_not_hold(path) -> None:
    """Round 2: an answer landing between the gate check and the withdrawal is refused too."""
    state, poi, lease, round_ = _review_under_way()
    state.registry.set_acquisition_held(1, False)  # published, the review not yet withdrawn
    if path == "operator":
        assert state.resolve_response(round_.poi_id, True) is ConfirmationResponseKind.LATE_OR_DUPLICATE
    elif path == "timeout":
        assert not state.complete_round(round_, ConfirmationStatus.CONFIRMED)
    elif path == "worker":
        assert not state.set_worker_status(lease, ConfirmationStatus.CONFIRMED)
    else:
        state.registry.update_status(poi, ConfirmationStatus.CONFIRMED)
    assert state.registry.get_status(poi) is None  # asked again once the gates hold


def test_an_answer_while_the_gates_hold_is_accepted() -> None:
    state, poi, _lease, round_ = _review_under_way()
    assert state.resolve_response(round_.poi_id, True) is ConfirmationResponseKind.CONFIRMED
    assert state.registry.get_status(poi) is ConfirmationStatus.CONFIRMED


def test_an_operator_confirmed_while_the_gates_fail_is_refused_in_the_graph() -> None:
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 5.0) is None  # locked and orbiting, the orbit not yet established
    manager = rig.rig.confirmation_manager
    manager.update_status(manager.active_poi, ConfirmationStatus.CONFIRMED)
    assert manager.get_status(manager.active_poi) is None


# --- P2 (round 2): a repeated frame does not keep the lock past the frame gap ----------------------------------------

def test_a_repeated_frame_breaks_the_lock_after_the_frame_gap() -> None:
    now = [0.0]
    lock = FreshVisualLock(SETTINGS, lambda _p: True, lambda: True, lambda: now[0])
    for i in range(26):
        now[0] = round(i * 0.1, 6)
        last = frame(now[0], 0.5, lambda: now[0])
        held = lock.locked(last)
    assert held
    now[0] = 2.5 + SETTINGS.lock_max_frame_gap_s
    assert lock.locked(last)  # the same frame, within the gap
    now[0] = 2.6 + SETTINGS.lock_max_frame_gap_s
    assert not lock.locked(last)  # no new frame for longer than the gap
    now[0] = 3.2
    assert not lock.locked(frame(3.2, 0.5, lambda: 3.2))  # a new frame restarts the dwell


# --- P2: a circle already flown cannot stall the own ORBIT ----------------------------------------------------------

def test_a_circle_already_flown_does_not_stall_the_own_orbit() -> None:
    rig = AcquisitionRig()
    rig.rig.navigation_task.orbit_radius_m = 250.0  # e.g. a peer's circle; the module keeps it
    entered = _drive(rig, 0.0, 60.0)
    assert rig.orbit_calls()[0].args[:2] == (OWN_CENTRE, 300.0)
    assert entered is not None


# --- P2: compass heading at the real final-approach confirmation and law boundary ------------------------------------

def _confirmation():
    return FinalApproachConfirmation(FinalApproachConfirmationPorts(
        lock=threading.RLock(),
        projector=FinalApproachFrameProjector(FinalApproachProjectionConfig("ZYX", True)),
        epochs=SourceEpochLedger(),
        law=VisionNavLaw(FixedFinalApproachLawConfigProvider(
            FinalApproachLawConfig(-55.0, 20.0, 45.0, None, 0.0),
        )),
    ))


def _law_outcome(error_deg: float, yaw_deg: float):
    confirmation = _confirmation()
    poi = frame(1.0, error_deg, lambda: 1.0, uas_att=Attitude(2.0, yaw_deg, 10.0),
                uas_body_rates_rad_s=(0.1, 0.2, 0.3), source_air_speed_mps=20.0)
    vision = confirmation._project(poi)
    plan = confirmation._ports.law.preview(vision)
    lock = FreshVisualLock(SETTINGS, lambda _p: True, lambda: True, lambda: 1.0)
    return (
        confirmation.can_confirm_detection(poi),
        (vision.control_x, vision.control_y, vision.control_z, vision.aircraft_yaw_rate_rad_s),
        None if plan is None else (plan.command, plan.within_limits),
        lock.locked(poi),
    )


@pytest.mark.parametrize("error_deg", [0.0, 0.5, 2.5])
def test_compass_heading_poisoned_at_the_law_boundary_changes_nothing(error_deg) -> None:
    clean = _law_outcome(error_deg, 0.0)
    assert clean[0] and clean[2] is not None
    for yaw_deg in (137.0, -91.0, 359.0):
        assert _law_outcome(error_deg, yaw_deg) == clean
