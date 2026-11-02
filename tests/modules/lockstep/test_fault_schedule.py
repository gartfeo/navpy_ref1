"""Which scheduled faults a run injects (``fault_schedule``, step 7)."""

from __future__ import annotations

import pytest

from navpy.core.swarm.task_peer_roster import PEER_SILENCE_EXPIRY_S
from navpy.sim.lockstep.coordinator_router import BACKEND_ENDPOINT, router_endpoints
from navpy.sim.lockstep.fault_schedule import (
    BACKEND_TARGET,
    FAULT_KINDS,
    FENCE_MARKER,
    LATE_HEARTBEAT,
    LAUNCH_MARKER,
    LINK_DROP,
    NAVPY_RESTART,
    PEER_RECOVERY_S,
    PRESENCE_MARKER,
    RADIO_PARTITION,
    RESTART_RECOVERY_S,
    FaultPlan,
    fault_plans,
)
from navpy.sim.lockstep.sim_profile import ModelDraw, Realization, load_profile, realize
from scripts.eval_gcs_demo_slice_gates import WORKFLOW_MARKERS

FLEET = 3


def _timed(kind: str, start_s: float = 12.0, duration_s: float = 3.0) -> ModelDraw:
    return ModelDraw(kind, True, {"start_s": start_s, "duration_s": duration_s})


def _realization(*draws: ModelDraw, seed: int = 1) -> Realization:
    return Realization("t", "0" * 64, seed, tuple(sorted(draws, key=lambda draw: draw.name)))


def test_the_constants_are_the_products_own_names():
    assert FENCE_MARKER == WORKFLOW_MARKERS["request"]
    assert BACKEND_TARGET == BACKEND_ENDPOINT
    assert PEER_RECOVERY_S == PEER_SILENCE_EXPIRY_S


def test_each_drawn_fault_is_planned_in_kind_order_with_its_drawn_times():
    realization = _realization(
        _timed(RADIO_PARTITION, 5.0, 2.5), _timed(LINK_DROP, 6.0, 1.5), _timed(LATE_HEARTBEAT, 7.0, 8.0),
        ModelDraw(NAVPY_RESTART, True, {"start_s": 8.0}))
    plans = fault_plans(realization, FLEET)
    assert tuple(plan.kind for plan in plans) == FAULT_KINDS
    assert [(plan.start_s, plan.duration_s) for plan in plans] == [
        (6.0, 1.5), (5.0, 2.5), (7.0, 8.0), (8.0, 0.0)]


def test_an_off_or_unlisted_fault_is_not_planned():
    realization = _realization(ModelDraw(LINK_DROP, False), ModelDraw("wind", True, {"speed_mps": 1.0}))
    assert fault_plans(realization, FLEET) == ()


def test_a_one_uav_fleet_has_no_fault():
    assert fault_plans(_realization(_timed(LINK_DROP)), 1) == ()


@pytest.mark.parametrize("profile", ["none", "quiet"])
def test_the_profiles_that_must_stay_clean_plan_no_fault(profile):
    assert all(fault_plans(realize(load_profile(profile), seed), FLEET) == () for seed in range(1, 41))


def test_a_link_drop_takes_any_router_endpoint_and_the_others_any_uav():
    seeds = range(1, 61)
    drops = {fault_plans(_realization(_timed(LINK_DROP), seed=s), FLEET)[0].target for s in seeds}
    assert drops == set(router_endpoints(FLEET))
    for kind in (RADIO_PARTITION, LATE_HEARTBEAT, NAVPY_RESTART):
        assert {fault_plans(_realization(_timed(kind), seed=s), FLEET)[0].target for s in seeds} == {
            f"uav{i}" for i in range(1, FLEET + 1)}


def test_the_target_has_its_own_stream():
    alone = fault_plans(_realization(_timed(LINK_DROP), seed=9), FLEET)
    beside = fault_plans(_realization(_timed(LINK_DROP), _timed(RADIO_PARTITION, 1.0, 2.0), seed=9), FLEET)
    assert alone[0].target == beside[0].target


@pytest.mark.parametrize(("kind", "target", "anchor", "recovery_s", "ordinal"), [
    (LINK_DROP, "sitl2", LAUNCH_MARKER, PEER_RECOVERY_S, 2),
    (RADIO_PARTITION, "uav3", PRESENCE_MARKER, PEER_RECOVERY_S, 3),
    (LATE_HEARTBEAT, "uav1", LAUNCH_MARKER, PEER_RECOVERY_S, 1),
    (NAVPY_RESTART, "uav2", LAUNCH_MARKER, RESTART_RECOVERY_S, 2),
])
def test_a_plan_names_its_anchor_recovery_and_ordinal(kind, target, anchor, recovery_s, ordinal):
    plan = FaultPlan(kind, target, 4.0, 2.0)
    assert (plan.id, plan.anchor, plan.recovery_s, plan.ordinal) == (f"{kind}:{target}", anchor, recovery_s, ordinal)
    assert plan.to_json() == {"id": plan.id, "kind": kind, "target": target, "anchor": anchor,
                              "start_s": 4.0, "duration_s": 2.0, "recovery_s": recovery_s}
