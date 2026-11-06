"""Lock and confirm gates before confirm and NAV (HITL step 25).

search -> own detection -> fresh visual lock -> ORBIT on the own-detected pose -> confirm -> NAV.
Unit tests of the two gates and the sequence, the composition, each gate missing, invalid vision,
truth and compass heading poisoned throughout, and 10 seeded replays through the real navigation
graph (``acquisition_replay.py``).
"""

from __future__ import annotations

import ast
import dataclasses
import math
from pathlib import Path
from unittest.mock import Mock

import pytest

from navpy.core.flow.nav_state import NavState
from navpy.core.kernel.location import Location
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.ports.pixel_observation import PixelProjectionKind
from navpy.core.review.confirmation_registry_state import ConfirmationStatus
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.skills.confirm.acquisition_gate_settings import (
    FINAL_APPROACH_GATES_KEY,
    AcquisitionGateSettings,
    acquisition_gate_settings,
)
from navpy.core.skills.confirm.acquisition_sequence import (
    OPEN_ACQUISITION,
    AcquisitionSequence,
    GatedOrbitStart,
)
from navpy.core.skills.confirm.fresh_visual_lock import FreshVisualLock, lock_sample
from navpy.core.skills.confirm.orbit_established import (
    ORBIT_MAX_OBSERVATION_GAP_S,
    OrbitCommand,
    OrbitEstablished,
)
from navpy.core.skills.geo_approach.self_detected_approach import SelfDetectedApproach
from tests.modules.nav.acquisition_replay import (
    GATES,
    OWN_CENTRE,
    AcquisitionRig,
    frame,
    offset,
    oracle_allows,
    run_replay,
)

SETTINGS = AcquisitionGateSettings(**GATES)
SEEDS = range(2501, 2511)
SRC = Path(__file__).resolve().parents[3] / "src" / "navpy"
GATE_MODULES = (
    "core/skills/confirm/acquisition_gate_settings.py",
    "core/skills/confirm/fresh_visual_lock.py",
    "core/skills/confirm/orbit_established.py",
    "core/skills/confirm/acquisition_sequence.py",
)


def _clock(start: float = 0.0):
    now = [start]
    return now, (lambda: now[0])


# --- settings ------------------------------------------------------------------------------------------------------

def test_only_the_hitl_profile_sets_the_gates() -> None:
    profiles = load_profiles()[0]
    assert acquisition_gate_settings(profiles["siyi_zr10_hitl_encoded"]) == SETTINGS
    for name, profile in profiles.items():
        if "hitl" not in name:
            assert acquisition_gate_settings(profile) is None, name


@pytest.mark.parametrize("block", [
    [1.0],
    {k: v for k, v in GATES.items() if k != "orbit_dwell_s"},
    {**GATES, "extra": 1.0},
    {**GATES, "lock_dwell_s": True},
    {**GATES, "lock_dwell_s": "2"},
    {**GATES, "lock_dwell_s": math.nan},
    {**GATES, "orbit_dwell_s": math.inf},
    {**GATES, "orbit_radial_tolerance_m": 0.0},
    {**GATES, "lock_max_frame_gap_s": -0.5},
    {**GATES, "lock_max_image_error_deg": 90.0},
])
def test_a_malformed_block_is_refused(block) -> None:
    with pytest.raises(ValueError):
        acquisition_gate_settings({FINAL_APPROACH_GATES_KEY: block})


# --- fresh visual lock ---------------------------------------------------------------------------------------------

def _lock(fresh=lambda _poi: True, armed=lambda: True) -> FreshVisualLock:
    return FreshVisualLock(SETTINGS, fresh, armed)


def _feed(lock: FreshVisualLock, start: float, end: float, error: float = 0.5, step: float = 0.1, **kw) -> bool:
    result = False
    for i in range(int(round((end - start) / step)) + 1):
        ts = round(start + i * step, 6)
        result = lock.locked(frame(ts, error, lambda: ts, **kw))
    return result


def test_lock_holds_after_the_dwell_and_not_before() -> None:
    lock = _lock()
    assert not _feed(lock, 0.0, 1.9)
    assert lock.locked(frame(2.0, 0.5, lambda: 2.0))


def test_image_error_is_the_angle_off_the_optical_axis() -> None:
    sample = lock_sample(frame(1.0, 0.9, lambda: 1.0))
    assert sample.image_error_deg == pytest.approx(0.9)
    assert sample.track == (1, 1) and sample.timestamp_s == 1.0


@pytest.mark.parametrize("break_at", ["error", "gap", "back", "track"])
def test_each_break_restarts_the_dwell(break_at) -> None:
    lock = _lock()
    assert _feed(lock, 0.0, 2.5)
    if break_at == "error":
        assert not lock.locked(frame(2.6, 1.2, lambda: 2.6))
        start = 2.7
    elif break_at == "gap":
        assert not lock.locked(frame(3.1, 0.5, lambda: 3.1))  # 0.6 s since the last new frame
        start = 3.2
    elif break_at == "back":
        assert not lock.locked(frame(2.0, 0.5, lambda: 2.6))
        start = 2.1
    else:
        assert not lock.locked(_other_track(2.6))
        start = 2.7
    assert not _feed(lock, start, start + 1.8)
    assert _feed(lock, start + 1.9, start + 2.2)


def _other_track(ts: float):
    poi = frame(ts, 0.5, lambda: ts)
    poi.reidentify(obj_id=7)
    return poi


def test_the_same_frame_again_adds_no_dwell() -> None:
    lock = _lock()
    first = frame(0.0, 0.5, lambda: 0.0)
    for _ in range(50):
        assert not lock.locked(first)


@pytest.mark.parametrize("invalid", ["stale", "unarmed", "not_atomic", "spherical", "nan", "none", "no_time"])
def test_invalid_vision_breaks_the_lock(invalid) -> None:
    fresh = {"ok": True}
    armed = {"ok": True}
    lock = _lock(lambda _poi: fresh["ok"], lambda: armed["ok"])
    assert _feed(lock, 0.0, 2.5)
    poi = frame(2.6, 0.5, lambda: 2.6)
    if invalid == "stale":
        fresh["ok"] = False
    elif invalid == "unarmed":
        armed["ok"] = False
    elif invalid == "not_atomic":
        poi = frame(2.6, 0.5, lambda: 2.6, atomic=False)
    elif invalid == "spherical":
        poi = frame(2.6, 0.5, lambda: 2.6, projection=PixelProjectionKind.SPHERICAL_EQUIANGULAR)
    elif invalid == "nan":
        poi = frame(math.nan, 0.5, lambda: 2.6)
    elif invalid == "no_time":
        poi = frame(2.6, 0.5, lambda: 2.6)
        poi.replace_pixel(dataclasses.replace(poi.pixel, source_timestamp_s=None))
    else:
        poi = None
    assert not lock.locked(poi)
    fresh["ok"] = armed["ok"] = True
    assert not _feed(lock, 2.7, 4.5)


# --- orbit established ---------------------------------------------------------------------------------------------

def _orbit(position):
    now, clock = _clock()
    gate = OrbitEstablished(SETTINGS, lambda: position[0], clock)
    return gate, now


def test_orbit_counts_only_after_the_command_and_the_dwell() -> None:
    position = [offset(OWN_CENTRE, 310.0, 0.0)]
    gate, now = _orbit(position)
    assert not gate.established()
    gate.start(OrbitCommand(OWN_CENTRE, 300.0))
    for step in range(100):
        now[0] = step * 0.1
        assert not gate.established()
    now[0] = 10.0
    assert gate.established()


@pytest.mark.parametrize("fault", ["outside", "gap", "none", "nan", "back"])
def test_orbit_faults_restart_the_dwell(fault) -> None:
    position = [offset(OWN_CENTRE, 0.0, 320.0)]
    gate, now = _orbit(position)
    gate.start(OrbitCommand(OWN_CENTRE, 300.0))
    for step in range(101):
        now[0] = step * 0.1
        gate.established()
    assert gate.established()
    if fault == "outside":
        position[0] = offset(OWN_CENTRE, 0.0, 326.0)
        now[0] = 10.1
    elif fault == "gap":
        now[0] = 10.0 + ORBIT_MAX_OBSERVATION_GAP_S + 0.1
        assert not gate.established()
        return
    elif fault == "none":
        position[0] = None
        now[0] = 10.1
    elif fault == "nan":
        position[0] = Location(math.nan, OWN_CENTRE.lng, 80.0)
        now[0] = 10.1
    else:
        now[0] = 9.0
    assert not gate.established()
    position[0] = offset(OWN_CENTRE, 0.0, 320.0)
    now[0] += 0.1
    assert not gate.established()


def test_orbit_ignores_the_heading_and_the_altitude() -> None:
    results = []
    for heading, alt in ((0.0, 80.0), (179.0, -5.0), (math.nan, 4000.0)):
        position = [offset(OWN_CENTRE, 0.0, 290.0, alt=alt, heading=heading)]
        gate, now = _orbit(position)
        gate.start(OrbitCommand(OWN_CENTRE, 300.0))
        for step in range(101):
            now[0] = step * 0.1
            last = gate.established()
        results.append(last)
    assert results == [True, True, True]


# --- sequence ------------------------------------------------------------------------------------------------------

class _Start:
    def __init__(self, command):
        self.calls = []
        self.command = command

    def prepare(self, poi):
        self.calls.append(poi)
        return self.command


def _sequence(command=OrbitCommand(OWN_CENTRE, 300.0)):
    position = [offset(OWN_CENTRE, 0.0, 300.0)]
    now, clock = _clock()
    start = _Start(command)
    sequence = AcquisitionSequence(
        FreshVisualLock(SETTINGS, lambda _p: True, lambda: True),
        OrbitEstablished(SETTINGS, lambda: position[0], clock),
        start, Mock(),
    )
    return sequence, start, now


def _run(sequence, now, start: float, end: float, error: float = 0.5) -> list:
    out = []
    for i in range(int(round((end - start) / 0.1)) + 1):
        ts = round(start + i * 0.1, 6)
        now[0] = ts
        out.append(sequence.ready(frame(ts, error, lambda: ts)))
    return out


def test_the_orbit_waits_for_the_lock_and_is_placed_once_from_that_frame() -> None:
    sequence, start, now = _sequence()
    poi = frame(0.0, 3.0, lambda: 0.0)
    GatedOrbitStart(sequence).prepare(poi)
    assert _run(sequence, now, 0.0, 5.0, error=3.0) == [False] * 51
    assert start.calls == []
    out = _run(sequence, now, 5.1, 7.1)  # the first frame within the error is at 5.1: locked at 7.1
    assert len(start.calls) == 1 and start.calls[0].pixel.source_timestamp_s == pytest.approx(7.1)
    assert not any(out)
    assert not any(_run(sequence, now, 7.2, 17.0))
    assert _run(sequence, now, 17.1, 17.1) == [True]
    assert len(start.calls) == 1


def test_a_vision_blackout_drops_the_lock_but_not_the_orbit_dwell() -> None:
    sequence, _start, now = _sequence()
    GatedOrbitStart(sequence).prepare(frame(0.0, 0.5, lambda: 0.0))
    assert _run(sequence, now, 0.0, 12.1)[-1] is True
    for i in range(15):  # 1.5 s with no detection, past ORBIT_MAX_OBSERVATION_GAP_S
        now[0] = round(12.2 + i * 0.1, 6)
        assert sequence.ready(None) is False
    assert sequence._orbit.established()  # watched every tick: still on the circle
    assert not any(_run(sequence, now, 13.7, 15.6))  # the lock dwells again
    assert _run(sequence, now, 15.7, 15.7) == [True]


def test_a_refused_orbit_is_retried_and_nothing_passes_meanwhile() -> None:
    sequence, start, now = _sequence(command=None)
    GatedOrbitStart(sequence).prepare(frame(0.0, 0.5, lambda: 0.0))
    assert not any(_run(sequence, now, 0.0, 30.0))
    assert len(start.calls) > 200  # every locked tick asks again


def test_another_task_or_no_start_is_never_ready() -> None:
    sequence, _start, now = _sequence()
    assert not any(_run(sequence, now, 0.0, 20.0))  # begin() never called
    other = frame(0.0, 0.5, lambda: 0.0, task_id=9)
    GatedOrbitStart(sequence).prepare(other)
    assert not any(_run(sequence, now, 20.1, 40.0))


def test_open_acquisition_passes_everything() -> None:
    assert OPEN_ACQUISITION.ready(None) is True


# --- composition ---------------------------------------------------------------------------------------------------

def test_the_composition_gates_only_with_the_profile() -> None:
    gated = AcquisitionRig()
    assert isinstance(gated.rig.navigation_task_action._self_approach, GatedOrbitStart)
    assert isinstance(gated.sequence, AcquisitionSequence)
    assert gated.rig.poi_status._confirmed._acquisition_ready.__code__.co_names  # bound to the sequence
    open_rig = AcquisitionRig(profile={})
    assert isinstance(open_rig.rig.navigation_task_action._self_approach, SelfDetectedApproach)
    assert open_rig.sequence is OPEN_ACQUISITION


def test_the_gates_need_the_orbit_approach_and_a_well_formed_block() -> None:
    with pytest.raises(ValueError, match="ORBIT"):
        AcquisitionRig(approach_kind=ApproachKind.OFFSET)
    with pytest.raises(ValueError):
        AcquisitionRig(profile={FINAL_APPROACH_GATES_KEY: {"lock_dwell_s": 2.0}})


# --- through the real navigation graph -----------------------------------------------------------------------------

def _drive(rig: AcquisitionRig, start: float, end: float, *, error=0.5, radial=0.0, atomic=True, orbit=True):
    for i in range(int(round((end - start) / 0.1)) + 1):
        ts = round(start + i * 0.1, 6)
        rig.world.t = ts
        rig.world.detections = [frame(ts, error, rig.now, atomic=atomic)]
        if orbit and rig.orbit_calls():
            rig.world.position = offset(OWN_CENTRE, 300.0 + radial, 0.0)
        rig.tick()
        if rig.phase is NavState.NAV:
            return ts
    return None


def test_without_a_lock_nothing_is_confirmed_orbited_or_flown() -> None:
    rig = AcquisitionRig()
    rig.world.position = offset(OWN_CENTRE, 300.0, 0.0)  # even on the circle already
    assert _drive(rig, 0.0, 60.0, error=1.5) is None
    assert rig.orbit_calls() == [] and not rig.confirmed()
    rig.navigation.final_approach.record_confirmed_detection.assert_not_called()


def test_without_an_established_orbit_nothing_is_confirmed_or_flown() -> None:
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 60.0, radial=40.0) is None
    assert len(rig.orbit_calls()) == 1 and not rig.confirmed()
    assert rig.phase is NavState.CONFIRM


def test_with_both_gates_it_confirms_then_flies() -> None:
    rig = AcquisitionRig()
    entered = _drive(rig, 0.0, 60.0)
    # The first CONFIRM tick is 0.1 s (DETECT starts the task at 0.0): locked and the orbit placed at
    # 2.1 s, on the circle from 2.2 s, established and confirmed at 12.2 s, NAV the tick after.
    assert entered == pytest.approx(12.3)
    assert rig.orbit_calls()[0].args[:2] == (OWN_CENTRE, 300.0)


def test_invalid_vision_blocks_both_confirm_and_nav() -> None:
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 12.1) is None
    assert _drive(rig, 12.2, 40.0, atomic=False) is None  # orbit established, vision invalid
    assert not rig.confirmed()
    # an operator confirmation does not open NAV while the vision is invalid
    active = rig.rig.confirmation_manager.active_poi
    rig.rig.confirmation_manager.update_status(active, ConfirmationStatus.CONFIRMED)
    assert _drive(rig, 40.1, 60.0, atomic=False) is None
    assert rig.phase is NavState.CONFIRM
    # valid vision again: the lock must dwell again before NAV
    assert _drive(rig, 60.1, 62.0) is None
    assert _drive(rig, 62.1, 63.0) == pytest.approx(62.1)


def test_a_lock_lost_after_confirmation_blocks_nav() -> None:
    rig = AcquisitionRig()
    assert _drive(rig, 0.0, 12.1) is None
    rig.world.t = 12.2
    rig.world.detections = [frame(12.2, 0.5, rig.now)]
    rig.tick()  # confirms on this tick (the decision runs before the action, so NAV waits one tick)
    assert rig.confirmed() and rig.phase is NavState.CONFIRM
    assert _drive(rig, 12.3, 20.0, error=1.5) is None  # pointing off: no NAV
    assert rig.phase is NavState.CONFIRM


# --- poisoned inputs -----------------------------------------------------------------------------------------------

def _trace(replay) -> list:
    return [(tick["t"], tick["phase"], tick["confirmed"]) for tick in replay.ticks]


@pytest.mark.parametrize("seed", [2501, 2506])
def test_truth_poisoned_throughout_changes_nothing(seed) -> None:
    clean = run_replay(seed)
    poisoned = run_replay(seed, truth=True)
    assert _trace(poisoned) == _trace(clean)
    assert poisoned.orbit_centre == OWN_CENTRE  # the own ground ray, not the truth 6 km away


@pytest.mark.parametrize("seed", [2502, 2509])
def test_compass_heading_poisoned_at_the_law_boundary_changes_nothing(seed) -> None:
    clean = run_replay(seed)
    poisoned = run_replay(seed, compass=True)
    assert _trace(poisoned) == _trace(clean)


def test_the_gate_modules_name_no_heading_truth_or_vertical_state() -> None:
    banned = {"yaw", "heading", "attitude", "truth_poi_location", "is_simulation", "ground_speed_ned",
              "velocity", "alt", "altitude", "relative_altitude", "range", "board_pose", "bbox_cxcywh"}
    hits = []
    for module in GATE_MODULES:
        tree = ast.parse((SRC / module).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            name = node.attr if isinstance(node, ast.Attribute) else node.id if isinstance(node, ast.Name) else None
            if name in banned:
                hits.append(f"{module}:{node.lineno} {name}")
    assert hits == []


# --- seeded replays ------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("seed", SEEDS)
def test_seeded_replay_keeps_the_order_with_no_premature_transition(seed) -> None:
    replay = run_replay(seed)
    events = replay.events
    order = ["detected", "locked", "orbit", "established", "confirmed", "nav"]
    assert all(stage in events for stage in order), events
    times = [events[stage] for stage in order]
    assert times == sorted(times) and events["detected"] > 0.0  # search came first
    assert replay.orbit_centre == OWN_CENTRE
    ticks = replay.ticks
    confirm_entries = [i for i, tick in enumerate(ticks)
                       if tick["confirmed"] and (i == 0 or not ticks[i - 1]["confirmed"])]
    nav_entries = [i for i, tick in enumerate(ticks) if tick["phase"] is NavState.NAV]
    assert len(confirm_entries) == 1 and len(nav_entries) == 1
    assert oracle_allows(ticks, confirm_entries[0]) and oracle_allows(ticks, nav_entries[0])
    # never before the oracle allowed it: no confirmation or NAV on any earlier tick
    first_allowed = next(i for i in range(len(ticks)) if oracle_allows(ticks, i))
    assert confirm_entries[0] >= first_allowed
