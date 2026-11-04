"""Fixed zoom policy (HITL stage 1 step 18): calibrated steps, settle evidence, containment hysteresis.

Done (plan section 9 item 18): 100% of commands select measured steps; 0 unsettled measurements update LOS; an
exceeded frozen age or gap limit aborts rather than extending freshness. Synthetic tables and plants only.
"""

from __future__ import annotations

import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from navpy.app.perception.fixed_zoom_profile import build_fixed_zoom
from navpy.core.perception.camera.reading_intrinsics import ReadingIntrinsicsTable
from navpy.core.perception.profile.vision_profile_types import freeze_profile_value
from navpy.core.perception.zoom.fixed_step_zoom_drive import FixedStepZoom, FixedStepZoomDrive
from navpy.core.perception.zoom.fixed_step_zoom_policy import (
    ContainmentMargins,
    FixedStepZoomPolicy,
    FrameBox,
    predict_box,
)
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps
from navpy.core.perception.zoom.zoom_settle_evidence import (
    ABORT_HISTORY_LOST,
    ABORT_READING_AGE,
    ABORT_READING_GAP,
    STATUS_ZOOM_ABORTED,
    STATUS_ZOOM_HISTORY_UNAVAILABLE,
    STATUS_ZOOM_SETTLED,
    STATUS_ZOOM_TRAVEL,
    STATUS_ZOOM_UNBRACKETED,
    STATUS_ZOOM_UNKNOWN,
    ZoomJudgement,
    ZoomSettleEvidence,
    ZoomSettleLimits,
)
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import (
    GimbalCaptureHistory,
    ZoomCommandSample,
    ZoomReadingSample,
)
from navpy.core.kernel.attitude import Attitude
from navpy.plugins.gimbal.siyi.hardware.commands import SiyiHardwareCommands
from navpy.plugins.gimbal.siyi.hardware.state import SiyiReadbackStore
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import GimbalZoomLimits, GimbalZoomPlant
from tests.modules.vision import test_integration_timing as timing

W, H = 1920, 1080
# (command, reading, fx, basis): 3.0 is interpolated, so it is not a step.
ROWS = ((2.0, 1.8, 3000.0, "measured"), (3.0, 2.8, 4500.0, "interpolated"), (4.0, 3.8, 6000.0, "measured"),
        (8.0, 7.7, 12000.0, "measured"))


def _section(width=W, height=H, rows=ROWS, wide_stop=True):
    out = [{"reading": reading, "set_by": {"absolute_zoom": command}, "basis": basis, "fx": fx, "fy": fx,
            "cx": width / 2.0, "cy": height / 2.0, "skew": 0.0, "dist": [0.0, 0.0, 0.0, 0.0, 0.0]}
           for command, reading, fx, basis in rows]
    if wide_stop:
        out.append({"reading": 1.0, "set_by": "wide_stop", "basis": "measured", "fx": 1700.0, "fy": 1700.0,
                    "cx": width / 2.0, "cy": height / 2.0, "skew": 0.0, "dist": [0.0] * 5})
    return {"source": "synthetic", "streams": [{"mode": "main", "image_width": width, "image_height": height,
                                                 "crop": [0, 0, width, height], "rows": out}]}


def _steps(width=W, height=H) -> FixedZoomSteps:
    table = ReadingIntrinsicsTable.from_profile(_section(width, height))
    return FixedZoomSteps.from_table(table, table.streams()[0])


STEPS = _steps()
WIDE, MID, NARROW = STEPS.steps


def _history(readings=(), commands=(), session=1):
    return GimbalCaptureHistory(
        session, (), GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        zoom_readings=tuple(ZoomReadingSample(session, seq, t, r) for seq, (t, r) in enumerate(readings)),
        zoom_commands=tuple(ZoomCommandSample(session, t, c) for t, c in commands),
    )


def _readings(start, end, value, period=0.05):
    count = int(round((end - start) / period))
    return [(start + i * period, value) for i in range(count + 1)]


# -- steps --------------------------------------------------------------------------------------------------------
def test_steps_are_the_measured_absolute_zoom_rows_widest_first():
    assert [step.command for step in STEPS.steps] == ["2.0", "4.0", "8.0"]
    assert [step.optics.reading for step in STEPS.steps] == ["1.8", "3.8", "7.7"]
    assert STEPS.widest is WIDE and STEPS.narrower(WIDE) is MID and STEPS.wider(WIDE) is None


def test_steps_refuse_an_empty_table_and_shared_readings():
    table = ReadingIntrinsicsTable.from_profile(_section(rows=((3.0, 2.8, 4500.0, "interpolated"),), wide_stop=True))
    with pytest.raises(ValueError, match="at least one"):
        FixedZoomSteps.from_table(table, table.streams()[0])
    with pytest.raises(ValueError, match="distinct readings"):
        FixedZoomSteps(STEPS.stream, (WIDE, replace(MID, reading_tenths=WIDE.reading_tenths)))


def test_profile_builder_uses_the_real_zr10_table_and_checks_the_command_table():
    camera = {"reading_intrinsics": _section(), "zoom_calibration": {"2.0": 1.8, "4.0": 3.8, "8.0": 7.7}}
    fixed = build_fixed_zoom(freeze_profile_value({"camera": camera}))
    assert [step.command for step in fixed.steps.steps] == ["2.0", "4.0", "8.0"]
    camera["zoom_calibration"]["4.0"] = 3.9
    with pytest.raises(ValueError, match="zoom_calibration reads 3.9"):
        build_fixed_zoom(freeze_profile_value({"camera": camera}))
    assert build_fixed_zoom(freeze_profile_value({"camera": {}})) is None


def test_the_shipped_zr10_profile_gives_measured_steps_that_agree_with_its_command_table():
    from navpy.core.perception.profile.vision_profile_loader import load_profiles

    profiles, _default, _path = load_profiles()
    cameras = [device["camera"] for device in profiles["siyi_zr10"]["devices"] if "camera" in device]
    fixed = build_fixed_zoom(freeze_profile_value({"camera": cameras[0]}))
    assert [step.command for step in fixed.steps.steps] == ["2.0", "3.0", "4.0", "6.0", "8.0", "9.0", "10.0"]
    assert all(step.optics.basis == "measured" for step in fixed.steps.steps)


# -- settle evidence: travel ----------------------------------------------------------------------------------------
def _travel_history(arrive_s=11.0, end_s=12.0):
    """1.8 until the command at 10.0 takes effect, travel through 2.x-3.7, 3.8 from arrive_s."""
    readings = _readings(9.0, 10.2, 1.8)
    travel = np.linspace(2.0, 3.7, int(round((arrive_s - 10.25) / 0.05)))
    readings += [(10.25 + i * 0.05, round(float(v), 1)) for i, v in enumerate(travel)]
    readings += _readings(arrive_s, end_s, 3.8)
    return _history(readings, [(10.0, 40)])


@pytest.mark.parametrize("lookup_s, status", [
    (9.5, STATUS_ZOOM_TRAVEL),          # before the command: earlier epochs are not kept
    (10.5, STATUS_ZOOM_TRAVEL),         # lens moving
    (11.1, STATUS_ZOOM_TRAVEL),         # reading at target, dwell not met
    (11.299, STATUS_ZOOM_TRAVEL),
    (11.3, STATUS_ZOOM_SETTLED),
    (12.0, STATUS_ZOOM_SETTLED),
    (12.01, STATUS_ZOOM_UNBRACKETED),   # no held reading after the lookup yet
])
def test_travel_frames_are_refused_until_the_reading_has_held_for_the_dwell(lookup_s, status):
    evidence = ZoomSettleEvidence(STEPS)
    judgement = evidence.judge(_travel_history(), lookup_s, 12.05)
    assert judgement.status == status
    assert judgement.step is MID
    if judgement.settled:
        assert judgement.arrival_s == pytest.approx(11.0) and judgement.held_until_s == pytest.approx(12.0)


def test_a_reading_that_leaves_the_target_before_the_dwell_restarts_the_wait():
    readings = _readings(10.0, 10.5, 1.8) + [(10.55, 3.8), (10.6, 3.8), (10.65, 3.9)] + _readings(10.7, 11.5, 3.8)
    evidence = ZoomSettleEvidence(STEPS)
    history = _history(readings, [(10.0, 40)])
    assert evidence.judge(history, 10.9, 11.5).status == STATUS_ZOOM_TRAVEL     # 10.7 + 0.3 not reached
    judgement = evidence.judge(history, 11.0, 11.5)
    assert judgement.settled and judgement.arrival_s == pytest.approx(10.7)


def test_fewer_than_the_minimum_held_readings_never_settle():
    readings = _readings(10.0, 10.5, 1.8) + [(10.6, 3.8), (10.95, 3.8)]
    evidence = ZoomSettleEvidence(STEPS, ZoomSettleLimits(min_held_readings=3))
    assert evidence.judge(_history(readings, [(10.0, 40)]), 10.92, 11.0).status == STATUS_ZOOM_TRAVEL


# -- settle evidence: unknown readings and settings ---------------------------------------------------------------
@pytest.mark.parametrize("commands, readings, status", [
    ((), _readings(10.0, 12.0, 1.8), STATUS_ZOOM_UNKNOWN),                        # nothing commanded yet
    (((10.0, 10),), _readings(10.0, 12.0, 1.0), STATUS_ZOOM_UNKNOWN),             # start-up zoom 1: no step
    (((10.0, 40), (10.5, None)), _readings(10.0, 12.0, 3.8), STATUS_ZOOM_UNKNOWN),  # a manual zoom after it
    (((10.0, 30),), _readings(10.0, 12.0, 2.8), STATUS_ZOOM_UNKNOWN),             # interpolated row: not a step
    (((10.0, 40),), _readings(10.0, 12.0, 3.85), STATUS_ZOOM_TRAVEL),             # off-grid reading never arrives
    (((10.0, 40),), _readings(10.0, 12.0, 3.7), STATUS_ZOOM_TRAVEL),              # wrong reading never arrives
])
def test_unknown_settings_and_readings_never_settle(commands, readings, status):
    judgement = ZoomSettleEvidence(STEPS).judge(_history(readings, commands), 11.9, 12.0)
    assert judgement.status == status and not judgement.settled


def test_a_lens_that_moves_after_settling_without_a_command_is_unknown_until_the_next_command():
    readings = _readings(10.0, 11.0, 3.8) + _readings(11.05, 12.0, 3.7)
    evidence = ZoomSettleEvidence(STEPS)
    assert evidence.judge(_history(_readings(10.0, 11.0, 3.8), [(9.9, 40)]), 10.8, 11.0).settled
    assert evidence.judge(_history(readings, [(9.9, 40)]), 10.8, 12.0).status == STATUS_ZOOM_UNKNOWN
    readings += _readings(12.05, 13.0, 1.8)
    assert evidence.judge(_history(readings, [(9.9, 40), (12.0, 20)]), 12.9, 13.0).step is WIDE


def test_no_history_is_its_own_refusal():
    assert ZoomSettleEvidence(STEPS).judge(None, 1.0, 1.0).status == STATUS_ZOOM_HISTORY_UNAVAILABLE


# -- settle evidence: frozen age and gaps abort, never extend freshness --------------------------------------------
def test_a_reading_gap_over_the_limit_aborts_for_the_session_even_after_readings_resume():
    readings = _readings(10.0, 11.0, 3.8) + _readings(11.6, 13.0, 3.8)          # 0.6 s gap
    evidence = ZoomSettleEvidence(STEPS)
    history = _history(readings, [(9.9, 40)])
    judgement = evidence.judge(history, 12.5, 13.0)
    assert judgement.status == STATUS_ZOOM_ABORTED and judgement.abort_reason == ABORT_READING_GAP
    more = _history(readings + _readings(13.05, 14.0, 3.8), [(9.9, 40), (13.0, 20)])
    assert evidence.judge(more, 13.9, 14.0).status == STATUS_ZOOM_ABORTED


def test_a_frozen_reading_older_than_the_age_limit_aborts():
    history = _history(_readings(10.0, 11.0, 3.8), [(9.9, 40)])
    evidence = ZoomSettleEvidence(STEPS)
    assert evidence.judge(history, 10.9, 11.4).settled
    judgement = evidence.judge(history, 10.9, 11.51)
    assert judgement.status == STATUS_ZOOM_ABORTED and judgement.abort_reason == ABORT_READING_AGE
    assert evidence.judge(history, 10.9, 11.0).status == STATUS_ZOOM_ABORTED      # latched


def test_history_lost_between_two_snapshots_aborts():
    evidence = ZoomSettleEvidence(STEPS)
    first = _history(_readings(10.0, 11.0, 3.8), [(9.9, 40)])
    assert evidence.judge(first, 10.9, 11.0).settled
    # The bounded history moved on past everything seen: whatever fell out between was never judged.
    later = GimbalCaptureHistory(1, (), first.data, zoom_readings=tuple(
        ZoomReadingSample(1, 100 + i, t, r) for i, (t, r) in enumerate(_readings(11.3, 11.5, 3.8))),
        zoom_commands=first.zoom_commands)
    judgement = evidence.judge(later, 11.4, 11.5)
    assert judgement.status == STATUS_ZOOM_ABORTED and judgement.abort_reason == ABORT_HISTORY_LOST


def test_a_new_session_starts_unknown_and_needs_its_own_command():
    evidence = ZoomSettleEvidence(STEPS)
    gap = _readings(10.0, 11.0, 3.8) + _readings(11.6, 12.0, 3.8)
    assert evidence.judge(_history(gap, [(9.9, 40)]), 11.9, 12.0).status == STATUS_ZOOM_ABORTED
    fresh = _history(_readings(20.0, 21.0, 3.8), (), session=2)
    assert evidence.judge(fresh, 20.9, 21.0).status == STATUS_ZOOM_UNKNOWN
    fresh = _history(_readings(20.0, 22.0, 3.8), [(21.05, 40)], session=2)
    assert evidence.judge(fresh, 21.9, 22.0).settled


# -- SIYI store and command log ---------------------------------------------------------------------------------
def test_siyi_store_keeps_newer_zoom_readings_and_logs_every_zoom_write():
    store = SiyiReadbackStore(GimbalData(att=Attitude(0.0, 0.0, 0.0)))
    assert store.accept_zoom((1, 1.8, 5.0), 6.0)
    assert not store.accept_zoom((1, 1.9, 5.5), 6.0)       # same sequence
    store.commit_poll(None, (2, 3.8, 5.6), 6.0)
    store.accept_zoom((0, 2.8, 5.7), 6.0)                   # older sequence: snapshot rule only
    session = SimpleNamespace(borrow=lambda: _Borrow(SimpleNamespace(
        requestAbsoluteZoom=lambda level: True, requestZoomIn=lambda: True, requestZoomHold=lambda: True)))
    commands = SiyiHardwareCommands(session, store, SimpleNamespace(warning=lambda message: None))
    assert commands.set_zoom("4.0") and commands.zoom_in() and commands.zoom_hold()
    history = store.capture_history()
    assert [(r.seq, r.reading) for r in history.zoom_readings] == [(1, 1.8), (2, 3.8)]
    assert [c.absolute_tenths for c in history.zoom_commands] == [40, None, None]
    store.reset_session()
    history = store.capture_history()
    assert history.zoom_readings == () and history.zoom_commands == ()


class _Borrow:
    def __init__(self, sdk):
        self._sdk = sdk

    def __enter__(self):
        return self._sdk

    def __exit__(self, *exc):
        return False


# -- policy --------------------------------------------------------------------------------------------------------
def _settled(step):
    return ZoomJudgement(STATUS_ZOOM_SETTLED, step=step, session=1)


def _box(cx, cy, w, h):
    return FrameBox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, W, H)


def test_zoom_in_needs_the_predicted_box_inside_the_in_margin_on_both_axes():
    policy = FixedStepZoomPolicy(STEPS)
    small = _box(W / 2, H / 2, 100, 60)
    assert policy.decide(_settled(WIDE), small) is MID
    # Wide enough horizontally, but at MID the box would be 2x taller: 0.6 H tall leaves a 0.2 gap < 0.25.
    tall = _box(W / 2, H / 2, 100, 0.3 * H)
    assert min(predict_box(tall, WIDE, MID).gaps()) < 0.25
    assert policy.decide(_settled(WIDE), tall) is None


def test_zoom_out_when_either_axis_is_near_the_edge():
    policy = FixedStepZoomPolicy(STEPS)
    assert policy.decide(_settled(MID), _box(W / 2, 0.08 * H, 80, 40)) is WIDE      # top edge
    assert policy.decide(_settled(MID), _box(0.95 * W, H / 2, 80, 40)) is WIDE      # right edge
    assert policy.decide(_settled(WIDE), _box(0.95 * W, H / 2, 80, 40)) is None     # nothing wider


def test_the_band_between_the_margins_holds_the_step():
    policy = FixedStepZoomPolicy(STEPS)
    # At MID: gaps 0.15 (above the 0.10 out margin); at NARROW it would be 2x off-centre, so no zoom in.
    box = _box(W / 2, H / 2, 0.70 * W, 0.70 * H)
    assert min(box.gaps()) == pytest.approx(0.15)
    assert policy.decide(_settled(MID), box) is None


def test_hysteresis_a_target_never_switches_back_and_forth():
    """A still target at any image position and size: once settled, no step is left and re-entered."""
    policy = FixedStepZoomPolicy(STEPS)
    rng = np.random.default_rng(3)
    for _ in range(300):
        # The target as a ray box (tangent half-extents and centre), the same for every step.
        tx, ty = rng.uniform(-0.3, 0.3), rng.uniform(-0.15, 0.15)
        hx, hy = rng.uniform(0.001, 0.05), rng.uniform(0.001, 0.05)
        step, visited = WIDE, [WIDE]
        for _ in range(10):
            f = step.optics.fx
            box = FrameBox(W / 2 + f * (tx - hx), H / 2 + f * (ty - hy), W / 2 + f * (tx + hx), H / 2 + f * (ty + hy),
                           W, H)
            chosen = policy.decide(_settled(step), box)
            if chosen is None:
                break
            step = chosen
            visited.append(step)
        assert len(visited) == len(set(visited)), visited          # no step re-entered


def test_loss_of_features_returns_to_the_widest_step_only_after_the_loss_count():
    policy = FixedStepZoomPolicy(STEPS)
    assert policy.decide(_settled(NARROW), None) is None
    assert policy.decide(ZoomJudgement(STATUS_ZOOM_TRAVEL, step=NARROW), None) is None   # travel does not count
    assert policy.decide(_settled(NARROW), None) is None
    assert policy.decide(_settled(NARROW), None) is WIDE
    assert policy.decide(_settled(WIDE), None) is None
    assert policy.decide(_settled(WIDE), None) is None
    assert policy.decide(_settled(WIDE), None) is None                                    # already widest


@pytest.mark.parametrize("status, expected", [
    (STATUS_ZOOM_UNKNOWN, "2.0"), (STATUS_ZOOM_TRAVEL, None), (STATUS_ZOOM_UNBRACKETED, None),
    (STATUS_ZOOM_ABORTED, None), (STATUS_ZOOM_HISTORY_UNAVAILABLE, None),
])
def test_only_unknown_commands_the_widest_step_and_travel_or_abort_commands_nothing(status, expected):
    step = FixedStepZoomPolicy(STEPS).decide(ZoomJudgement(status), _box(W / 2, H / 2, 50, 50))
    assert (None if step is None else step.command) == expected


def test_margins_must_keep_their_order():
    with pytest.raises(ValueError):
        ContainmentMargins(zoom_out=0.3, zoom_in=0.2)


# -- end to end: plant, readback, drive, association-side judgement -----------------------------------------------
class _ZoomRig:
    """An emulator-like lens: absolute zoom seeks at a finite speed, the reading is the 0.1-rounded level reported
    ``report_lag_s`` late, commands reach the lens ``link_s`` after they are logged."""

    def __init__(self, steps, speed=2.0, report_lag_s=0.03, link_s=0.02, start_level=1.0):
        self.plant = GimbalZoomPlant(start_level, GimbalZoomLimits(1.0, 10.0, speed, 1.0))
        self.readings_by_tenths = {step.command_tenths: step.reading_tenths / 10.0 for step in steps.steps}
        self.readings_by_tenths[10] = 1.0
        self.report_lag_s = report_lag_s
        self.link_s = link_s
        self.now = 0.0
        self.pending = []                       # (arrival time, level)
        self.levels = []                        # (time, level, target, setting)
        self.setting = None                     # command tenths the lens is set by, None for none
        self.readings = []
        self.commands = []
        self.writes = []

    def write(self, command: str) -> bool:
        tenths = round(float(command) * 10)
        self.writes.append(command)
        self.commands.append(ZoomCommandSample(1, self.now, tenths))
        self.pending.append((self.now + self.link_s, tenths))
        return True

    def history(self):
        return GimbalCaptureHistory(1, (), GimbalData(att=Attitude(0.0, 0.0, 0.0)),
                                    zoom_readings=tuple(self.readings[-64:]), zoom_commands=tuple(self.commands[-64:]))

    def advance(self, dt):
        self.now += dt
        for item in [p for p in self.pending if p[0] <= self.now]:
            self.pending.remove(item)
            self.plant.set_target(self.readings_by_tenths[item[1]])
            self.setting = item[1]
        self.plant.advance(dt)
        snapshot = self.plant.snapshot()
        self.levels.append((self.now, snapshot.level, snapshot.target, self.setting))

    def report(self, seq):
        past = [entry for entry in self.levels if entry[0] <= self.now - self.report_lag_s]
        if past:
            self.readings.append(ZoomReadingSample(1, seq, self.now, round(past[-1][1] * 10) / 10))

    def lens_at(self, t):
        return [entry for entry in self.levels if entry[0] <= t][-1]


@pytest.mark.parametrize("speed", [math.inf, 2.0, 0.5])
def test_end_to_end_every_command_is_a_step_and_no_unsettled_frame_is_admitted(speed, monkeypatch):
    rig = _ZoomRig(STEPS, speed=speed)
    rig.commands.append(ZoomCommandSample(1, 0.0, 10))      # navpy's start-up "zoom 1": no step
    rig.pending.append((rig.link_s, 10))
    fixed = FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS))
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: rig.now)
    drive = FixedStepZoomDrive(fixed, rig.history, rig.write, SimpleNamespace(info=lambda m: None))
    frame_evidence = ZoomSettleEvidence(STEPS)              # the association's own view, judged per frame
    admitted, refused, seq = 0, 0, 0
    dt = 1.0 / 600.0
    for tick in range(int(40.0 / dt)):
        rig.advance(dt)
        if tick % 30 == 0:                                   # 20 Hz readback
            seq += 1
            rig.report(seq)
        if tick % 20 != 0 or rig.now < 0.2:                  # 30 Hz frames
            continue
        exposure = rig.now - 0.1
        # The target swells and drifts: its ray box grows from 0.4 % to 12 % of the view, then shrinks.
        phase = rig.now / 40.0
        half = 0.002 + 0.06 * math.sin(math.pi * phase)
        centre = (0.05 * math.sin(rig.now / 3.0), 0.03 * math.cos(rig.now / 5.0))
        judgement = frame_evidence.judge(rig.history(), exposure + rig.report_lag_s, rig.now)
        if not judgement.settled:
            refused += 1
            drive.update_measurement(None, None) if judgement.status != STATUS_ZOOM_TRAVEL else drive.judge()
            continue
        _time, level, target, setting = rig.lens_at(exposure)
        step = judgement.step
        # 0 unsettled measurements: the lens was still, at the step's reading, set by the step's own command.
        assert target is None and setting == step.command_tenths, (rig.now, judgement)
        assert round(level * 10) == step.reading_tenths
        admitted += 1
        f = step.optics.fx
        box = FrameBox(W / 2 + f * (centre[0] - half), H / 2 + f * (centre[1] - half),
                       W / 2 + f * (centre[0] + half), H / 2 + f * (centre[1] + half), W, H)
        if min(box.gaps()) < 0.0:
            drive.update_measurement(None, step.command)      # out of the frame: lost
        else:
            drive.update_measurement(box, step.command)
    assert rig.writes and all(command in {s.command for s in STEPS.steps} for command in rig.writes)
    assert {"2.0", "4.0", "8.0"} <= set(rig.writes)           # it zoomed in and out through every step
    assert admitted > 300 and refused > 0
    # Hysteresis: commands are few compared with frames.
    assert len(rig.writes) < 20


def test_frozen_drive_holds_its_step_and_aborts_when_the_readback_freezes(monkeypatch):
    rig = _ZoomRig(STEPS, speed=math.inf)
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: rig.now)
    fixed = FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS))
    drive = FixedStepZoomDrive(fixed, rig.history, rig.write, SimpleNamespace(info=lambda m: None))
    seq = 0
    for tick in range(600):
        rig.advance(0.01)
        if tick % 5 == 0:
            seq += 1
            rig.report(seq)
        if tick == 10:
            assert drive.command_widest("freeze")
        drive.update_measurement(_box(W / 2, H / 2, 20, 20), "2.0", frozen=True)
    assert rig.writes == ["2.0"] and drive.is_stable                # held, no zoom-in while frozen
    # The readback stops: the frozen step is not extended, the evidence aborts and nothing more is written.
    for _ in range(60):
        rig.advance(0.01)
        drive.update_measurement(None, None, frozen=True)
    assert drive.judgement.status == STATUS_ZOOM_ABORTED and not drive.is_stable
    assert not drive.command_widest("reset") and rig.writes == ["2.0"]


def test_a_failed_write_is_retried_before_any_new_decision(monkeypatch):
    rig = _ZoomRig(STEPS)
    outcomes = iter([False, True])
    writes = []

    def write(command):
        writes.append(command)
        return next(outcomes)

    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: rig.now)
    drive = FixedStepZoomDrive(FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS)), rig.history, write,
                               SimpleNamespace(info=lambda m: None))
    drive.update_measurement(None, None)            # unknown -> widest, write fails
    drive.update_measurement(None, None)            # retried
    assert writes == ["2.0", "2.0"]


# -- association: 0 unsettled measurements update LOS -------------------------------------------------------------
def _fixed_4x4():
    steps = _steps(4, 4)
    return FixedStepZoom(steps, ZoomSettleEvidence(steps))


def _associate_with_zoom(readings_rel, commands_rel, fixed=None):
    frames = timing._trace(14.0)
    now_s = frames[-1][1] + 0.03
    gimbal = timing._gimbal_history(timing.T0_S - 1.0, now_s)
    zoom = _history([(now_s + t, r) for t, r in readings_rel], [(now_s + t, c) for t, c in commands_rel])
    gimbal = replace(gimbal, zoom_readings=zoom.zoom_readings, zoom_commands=zoom.zoom_commands)
    return timing._associate(mount=timing._mount(zoom="4.00"), gimbal=gimbal,
                             fixed_zoom=_fixed_4x4() if fixed is None else fixed)


SETTLED = (_readings(-3.0, -2.55, 1.8) + _readings(-2.5, 0.0, 3.8), [(-3.0, 40)])


def test_a_settled_zoom_frame_is_admitted_with_the_steps_measured_optics():
    association, records, _ = _associate_with_zoom(*SETTLED)
    assert association.pose_is_frame_atomic, association.pose_status
    mount = association.mount_state
    step = _steps(4, 4).by_command(40)
    assert mount.zoom_capture_relative and mount.zoom_command == "4.0"
    np.testing.assert_array_equal(mount.k, step.optics.k())
    np.testing.assert_array_equal(mount.dist, step.optics.dist_coeffs())
    [record] = [item for item in records if item["event"] == "association"]
    assert record["zoom_status"] == STATUS_ZOOM_SETTLED and record["zoom_step"] == "4.0"
    assert [item["event"] for item in records].count("zoom_config") == 1
    assert {"zoom_reading", "zoom_command"} <= {item["event"] for item in records}


@pytest.mark.parametrize("readings, commands, status", [
    (_readings(-3.0, -0.2, 1.8) + _readings(-0.15, 0.0, 3.8), [(-3.0, 40)], STATUS_ZOOM_TRAVEL),
    (_readings(-3.0, 0.0, 3.8), [(-3.0, None)], STATUS_ZOOM_UNKNOWN),
    (_readings(-3.0, 0.0, 3.8), [], STATUS_ZOOM_UNKNOWN),
    (_readings(-3.0, -1.8, 3.8) + _readings(-1.2, 0.0, 3.8), [(-3.5, 40)], STATUS_ZOOM_ABORTED),
    (_readings(-3.0, -0.6, 3.8), [(-3.5, 40)], STATUS_ZOOM_ABORTED),         # frozen: newest reading too old
])
def test_travel_unknown_and_aborted_zoom_frames_never_update_los(readings, commands, status):
    association, records, _ = _associate_with_zoom(readings, commands)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == status
    assert records[-1]["status"] == status


def test_a_frame_of_an_unmeasured_size_is_refused():
    steps = _steps(8, 8)
    association, _, _ = _associate_with_zoom(*SETTLED, fixed=FixedStepZoom(steps, ZoomSettleEvidence(steps)))
    assert association.pose_status == "real_frame_zoom_stream_unmeasured"


def test_without_fixed_zoom_a_zoom_mount_stays_refused():
    frames = timing._trace(14.0)
    association, _, _ = timing._associate(mount=timing._mount(zoom="4.00"))
    assert association.pose_status == "real_frame_zoom_unsettled"
    assert frames


# -- Codex round 1: session rollback, retries across a reconnect or freeze, fixed-only tracking ------------------
def test_an_older_session_snapshot_does_not_reset_an_aborted_newer_session():
    evidence = ZoomSettleEvidence(STEPS)
    gap = _readings(10.0, 11.0, 3.8) + _readings(11.6, 12.0, 3.8)
    assert evidence.judge(_history(gap, [(9.9, 40)], session=2), 11.9, 12.0).status == STATUS_ZOOM_ABORTED
    stale = _history(_readings(10.0, 12.0, 3.8), [(9.9, 40)], session=1)
    assert evidence.judge(stale, 11.9, 12.0).status == STATUS_ZOOM_HISTORY_UNAVAILABLE
    clean = _history(_readings(12.1, 13.0, 3.8), [(12.05, 40)], session=2)
    assert evidence.judge(clean, 12.9, 13.0).status == STATUS_ZOOM_ABORTED


def test_a_failed_write_is_not_retried_after_a_reconnect_or_into_a_freeze(monkeypatch):
    now = [11.0]
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: now[0])
    histories = [_history(_readings(10.0, 11.0, 1.8), [(9.9, 20)], session=1)]
    writes, outcomes = [], iter([False, True, True])

    def write(command):
        writes.append(command)
        return next(outcomes)

    drive = FixedStepZoomDrive(FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS)), lambda: histories[-1], write,
                               SimpleNamespace(info=lambda m: None))
    drive.update_measurement(_box(W / 2, H / 2, 40, 40), "2.0")            # zoom in to 4.0 fails
    assert writes == ["4.0"]
    histories.append(_history(_readings(10.5, 11.0, 3.8), (), session=2))  # reconnect: unknown
    drive.update_measurement(None, None, frozen=True)
    assert writes == ["4.0", "2.0"]                                           # widest, not the stale 4.0


def test_fixed_zoom_without_rate_tracking_still_starts_tracking_and_drives_zoom(monkeypatch):
    from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
    from tests.modules.navigation.test_gimbal_navigation import _make_mount, _make_poi

    mount, _gimbal = _make_mount()
    mount.gimbal_capture_history = lambda: _history(_readings(10.0, 11.0, 1.0), ())
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: 11.0)
    logger = SimpleNamespace(info=lambda m: None, warning=lambda m: None, debug=lambda m: None,
                             error=lambda m: None, is_enabled_for=lambda level: False)
    navigation = GimbalNavigation(mount, logger, tracking=None, zoom_config=None, rate_tracker=None,
                                  zoom_tracker=None, neutral_pitch_deg=-20.0,
                                  fixed_zoom=FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS)))
    navigation.start_tracking(7)
    mount.command_zoom.reset_mock()
    navigation.update(_make_poi(), now=11.0)
    mount.command_zoom.assert_called_with("2.0")          # unknown lens -> the widest step


def test_a_failed_zoom_in_does_not_survive_a_freeze_at_the_widest_step(monkeypatch):
    """Codex round 2: freeze at an already-widest lens, frozen updates without usable boxes, then unfreeze."""
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.rt.perf_counter", lambda: 11.0)
    history = _history(_readings(10.0, 11.0, 1.8), [(9.9, 20)])
    writes, outcomes = [], iter([False])

    def write(command):
        writes.append(command)
        return next(outcomes, True)

    drive = FixedStepZoomDrive(FixedStepZoom(STEPS, ZoomSettleEvidence(STEPS)), lambda: history, write,
                               SimpleNamespace(info=lambda m: None))
    drive.update_measurement(_box(W / 2, H / 2, 40, 40), "2.0")              # zoom in to 4.0 fails
    assert drive.command_widest("freeze") and writes == ["4.0"]              # already widest: nothing written
    poi = SimpleNamespace(optics=SimpleNamespace(frame_width_px=None, frame_height_px=None, zoom_command="2.0"))
    monkeypatch.setattr("navpy.core.perception.zoom.fixed_step_zoom_drive.extract_bbox", lambda p: None)
    drive.update(poi, frozen=True)
    drive.update(poi, frozen=False)
    assert writes == ["4.0"]                                                   # the stale zoom-in is gone
