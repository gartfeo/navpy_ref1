"""Explicit integration timing (HITL stage 1 step 11).

Synthetic encoded-video traces: a sender PTS clock with a rate error, a
receiver read time E + Dc + jitter (never negative), and slips of the video
delay. The map must stay within 1 ms of E + Dc over 10 minutes, abort on
every reset or slip over 1 ms, keep visual rates on PTS, and admit nothing as
certified. Association tests run the real publication store and builder.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.perception.camera.capture_lookup import CaptureLookupCalibration
from navpy.core.perception.camera.frame_publication import FramePublicationStore
from navpy.core.perception.camera.integration_timing import (
    AUTHORITY_INTEGRATION,
    STATUS_ABORTED,
    STATUS_MAPPED,
    STATUS_UNSETTLED,
    IntegrationTimingConfig,
    IntegrationTimingLimits,
    is_certified,
)
from navpy.core.perception.camera.pts_read_map import IntegrationTimingSource, PtsReadMap
from navpy.core.perception.pipeline.integration_association import (
    IntegrationAssociation,
    interpolate_gimbal_at,
)
from navpy.core.perception.pipeline.real_frame_association import FrameAssociationBuilder
from navpy.core.ports.frame import FinalApproachVisionFrame
from navpy.core.ports.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
    FrameSourceIdentity,
)
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.ports.gimbal_capture_history import (
    GimbalCaptureHistory,
    GimbalHistorySample,
)
from navpy.plugins.gimbal.siyi.hardware.state import SiyiReadbackStore
from navpy.plugins.laws.vision_pn.lateral_rate import LateralRateFilter
from navpy.plugins.laws.vision_pn.rate_filter import VerticalRateFilter

FPS = 30.0
DC_S = 0.080            # video delay of the trace (the envelope's floor)
T0_S = 500.0            # receiver capture clock at the first exposure
OPEN = "open-1"
SOURCE_ID = "rtsp://zr10/main"
LINK_ID = "udp:127.0.0.1:14550#1"
LIMITS = IntegrationTimingLimits()


def _config(**changes: object) -> IntegrationTimingConfig:
    values: dict[str, object] = dict(
        run_id="r001-pc-sitl", mode="pc-sitl", source_id=SOURCE_ID, link_id=LINK_ID,
        limits=LIMITS, attitude_correction_s=0.0, gimbal_correction_s=0.0,
    )
    values.update(changes)
    return IntegrationTimingConfig(**values)


def _stamp(pts_ns: int, read_s: float, sequence: int = 0, open_id: str = OPEN) -> CaptureStamp:
    return CaptureStamp(read_s, CaptureStampKind.READ, FrameSourceIdentity(open_id, sequence, pts_ns))


def _trace(seconds, ppm=0.0, jitter_mean_s=0.0, seed=0, slip_at_s=None, slip_s=0.0):
    """(pts_ns, read_s, ideal_s) per frame; ideal is E + DC_S, the pre-slip floor."""
    rng = np.random.default_rng(seed)
    frames = []
    for index in range(int(seconds * FPS)):
        exposure = T0_S + index / FPS
        pts_ns = round((index / FPS) * (1.0 + ppm * 1e-6) * 1e9) + 7_000_000_000
        delay = DC_S + (slip_s if slip_at_s is not None and index / FPS >= slip_at_s else 0.0)
        jitter = rng.exponential(jitter_mean_s) if jitter_mean_s > 0.0 else 0.0
        frames.append((pts_ns, exposure + delay + jitter, exposure + DC_S))
    return frames


def _run(frames, limits=LIMITS):
    """Feed every frame; returns (map, [(index, error_s, mapped)], first aborted index)."""
    pts_map = PtsReadMap(limits)
    admitted = []
    aborted_at = None
    for index, (pts_ns, read_s, ideal_s) in enumerate(frames):
        stamp = _stamp(pts_ns, read_s, index)
        status = pts_map.observe(stamp)
        if status == STATUS_ABORTED and aborted_at is None:
            aborted_at = index
        mapped, _ = pts_map.frame(stamp.source)
        if mapped is not None:
            admitted.append((index, mapped.mapped_s - ideal_s, mapped))
    return pts_map, admitted, aborted_at


# --- the map ------------------------------------------------------------------


@pytest.mark.parametrize("ppm", [300.0, -300.0, 50.0, -50.0])
def test_ten_minute_trace_with_jitter_and_clock_rate_error_maps_within_1_ms(ppm):
    frames = _trace(600.0, ppm=ppm, jitter_mean_s=0.002, seed=11)
    pts_map, admitted, aborted_at = _run(frames)
    assert aborted_at is None and pts_map.aborted is None
    errors = [abs(error) for _, error, _ in admitted]
    # Settles after the rate window + two slip windows (12 s), then every frame maps.
    assert len(admitted) >= len(frames) - int(12.1 * FPS)
    assert max(errors) <= 0.001


@pytest.mark.parametrize("ppm, jitter_s, slip_s", [
    (0.0, 0.0, 0.0012), (0.0, 0.0, -0.0012),
    (300.0, 0.001, 0.0015), (300.0, 0.001, -0.0015),
    (-300.0, 0.001, 0.0015), (-300.0, 0.001, -0.0015),
    (50.0, 0.002, 0.005), (-50.0, 0.002, -0.005),
])
def test_every_slip_over_1_ms_aborts_and_nothing_is_admitted_after(ppm, jitter_s, slip_s):
    frames = _trace(40.0, ppm=ppm, jitter_mean_s=jitter_s, seed=5, slip_at_s=20.0, slip_s=slip_s)
    pts_map, admitted, aborted_at = _run(frames)
    slip_index = int(20.0 * FPS)
    assert aborted_at is not None and aborted_at >= slip_index
    assert pts_map.aborted.startswith("slip")
    # Owner ruling 2026-11-02: frames of the short window before the abort
    # are admitted with up to the jump as error -- the stated gap.
    assert (aborted_at - slip_index) / FPS <= LIMITS.slip_window_s + 0.2
    after_slip = [(index, error) for index, error, _ in admitted if index >= slip_index]
    assert all(index < aborted_at for index, _ in after_slip)
    assert all(abs(error) <= abs(slip_s) + 0.001 for _, error in after_slip)


def test_a_new_open_or_a_backward_pts_aborts_and_a_repeated_pts_changes_nothing():
    frames = _trace(6.0)
    pts_map, admitted, _ = _run(frames)
    last_pts, last_read, _ = frames[-1]
    before = pts_map.frame(_stamp(last_pts, last_read).source)[0]
    assert pts_map.observe(_stamp(last_pts, last_read + 0.5)) == STATUS_MAPPED
    assert pts_map.frame(_stamp(last_pts, last_read).source)[0] == before
    assert pts_map.observe(_stamp(last_pts - 1, last_read + 0.1)) == STATUS_ABORTED
    assert pts_map.aborted == "reset: PTS went backward"
    assert pts_map.frame(_stamp(last_pts, last_read).source) == (None, STATUS_ABORTED)
    other = PtsReadMap(LIMITS)
    other.observe(_stamp(1, 1.0))
    assert other.observe(_stamp(2, 1.1, open_id="open-2")) == STATUS_ABORTED
    assert other.aborted == "reset: another open"


def test_unsettled_until_a_full_history():
    frames = _trace(30.0)
    pts_map = PtsReadMap(LIMITS)
    statuses = []
    for index, (pts_ns, read_s, _) in enumerate(frames[:400]):
        stamp = _stamp(pts_ns, read_s, index)
        pts_map.observe(stamp)
        statuses.append(pts_map.frame(stamp.source)[1])
    history_frames = int((LIMITS.rate_window_s + 2 * LIMITS.slip_window_s) * FPS)
    assert set(statuses[:history_frames]) == {STATUS_UNSETTLED}
    assert statuses[-1] == STATUS_MAPPED
    assert pts_map.aborted is None


def _gapped(frames, cut, gap_s, slip_s=0.0):
    resumed = [(pts, read + slip_s, ideal) for pts, read, ideal in frames[cut + int(gap_s * FPS):]]
    return frames[:cut] + resumed


@pytest.mark.parametrize("ppm, slip_s", [(300.0, 0.0), (-300.0, 0.0), (0.0, 0.00105), (0.0, -0.00105)])
@pytest.mark.parametrize("gap_s", [0.6, 30.0])
@pytest.mark.parametrize("first_admitted", [True, False])
def test_a_pts_gap_over_the_limit_aborts_as_a_reset(ppm, slip_s, gap_s, first_admitted):
    # Across a long gap the fitted rate's error (up to ~35 ppm under 2 ms
    # jitter) hides a 1 ms slip, so any gap over max_frame_gap_s ends the open.
    frames = _trace(gap_s + 40.0, ppm=ppm, jitter_mean_s=0.001, seed=4)
    _, admitted, _ = _run(frames)
    cut = admitted[0][0] + 1 if first_admitted else int(20.0 * FPS)
    pts_map, admitted, aborted_at = _run(_gapped(frames, cut, gap_s, slip_s))
    assert aborted_at == cut and pts_map.aborted == "reset: a PTS gap over max_frame_gap_s"
    assert not [index for index, _, _ in admitted if index >= cut]


@pytest.mark.parametrize("ppm", [300.0, -300.0])
def test_a_pts_gap_within_the_limit_keeps_mapping(ppm):
    frames = _trace(40.0, ppm=ppm, jitter_mean_s=0.002, seed=4)
    cut = int(20.0 * FPS)
    pts_map, admitted, aborted_at = _run(_gapped(frames, cut, 0.4))
    assert aborted_at is None and pts_map.aborted is None
    assert max(abs(error) for _, error, _ in admitted) <= 0.001
    assert [index for index, _, _ in admitted if index >= cut]


@pytest.mark.parametrize("slip_s", [0.0011, -0.0011])
@pytest.mark.parametrize("ppm", [300.0, 0.0, -300.0])
def test_a_slip_just_over_1_ms_in_a_short_gap_aborts_under_2_ms_jitter(slip_s, ppm):
    frames = _trace(40.0, ppm=ppm, jitter_mean_s=0.002, seed=401)
    cut = int(20.0 * FPS)
    pts_map, _, aborted_at = _run(_gapped(frames, cut, 0.4, slip_s))
    assert aborted_at is not None and pts_map.aborted.startswith("slip")


def test_visual_rate_intervals_are_pts_differences_while_the_offset_moves():
    frames = _trace(30.0, ppm=300.0, jitter_mean_s=0.002, seed=3)
    _, admitted, _ = _run(frames)
    mapped = [item for _, _, item in admitted]
    offsets = {item.offset_s for item in mapped}
    assert len(offsets) > 1                 # the mapping did update
    for earlier, later in zip(mapped, mapped[1:]):
        assert later.interval_s - earlier.interval_s == pytest.approx(
            (later.pts_ns - earlier.pts_ns) / 1e9, abs=1e-9)
    read_differences = {round(b.read_s - a.read_s, 9) for a, b in zip(mapped, mapped[1:])}
    pts_differences = {round(b.interval_s - a.interval_s, 9) for a, b in zip(mapped, mapped[1:])}
    assert len(read_differences) > len(pts_differences)


# --- admission ----------------------------------------------------------------


@pytest.mark.parametrize("changes, reason", [
    ({"mode": "flight"}, "may not use integration timing"),
    ({"mode": "qualification"}, "may not use integration timing"),
    ({"run_id": ""}, "run_id must be a non-empty string"),
    ({"link_id": " "}, "link_id must be a non-empty string"),
    ({"attitude_correction_s": math.nan}, "must be a finite number"),
    ({"limits": IntegrationTimingLimits(slip_limit_s=0.00005)}, "update_limit_s must be below"),
])
def test_integration_timing_is_refused_outside_a_valid_pc_sitl_run(changes, reason):
    with pytest.raises(ValueError, match=reason):
        IntegrationTimingSource(_config(**changes))


def test_integration_timing_is_never_certified_and_never_beside_a_calibration():
    assert not is_certified(AUTHORITY_INTEGRATION) and is_certified("certified")
    calibration = CaptureLookupCalibration(
        source_id=SOURCE_ID, link_id=LINK_ID, capture_lookup_bias_s=0.0,
        attitude_link_delay_s=0.0, residual_bound_s=0.006, camera_variation_bound_s=0.004,
        attitude_variation_bound_s=0.004, min_frame_interval_s=0.05,
    )
    integration = IntegrationAssociation(IntegrationTimingSource(_config()), lambda: None)
    with pytest.raises(ValueError, match="exclusive"):
        _builder(integration, calibration=calibration)


# Step 13: the one CLI path, the HITL runner's run record (--integration-timing), and the files wiring it.
INTEGRATION_CLI_FILES = {"vision_args.py", "integration_timing_input.py", "real_detector_factory.py",
                         "vision_controller_composition.py"}


def test_no_vision_profile_and_only_the_runner_cli_path_sets_integration_timing():
    root = Path(__file__).resolve().parents[3]
    for folder in ("src/navpy/app", "src/navpy/core/kernel/config", "src/navpy/products"):
        for path in (root / folder).rglob("*.py"):
            if folder == "src/navpy/app" and path.parent.name in {"app", "perception"}                     and path.name in INTEGRATION_CLI_FILES:
                continue
            assert "integration_timing" not in path.read_text(encoding="utf-8"), path
    for path in (root / "src").rglob("vision_profiles*.json"):
        assert "integration" not in path.read_text(encoding="utf-8"), path


# --- association --------------------------------------------------------------


def _attitude_history(start_s, end_s, period_s=0.02, yaw_deg=0.0):
    samples = []
    receipt = start_s
    while receipt <= end_s:
        samples.append(SimpleNamespace(
            attitude=Attitude(pitch=2.0, yaw=yaw_deg, roll=10.0 * (receipt - start_s)),
            body_rates_rad_s=(0.1, 0.0, 0.0),
            time_boot_s=receipt - start_s + 100.0,
            capture_receipt_s=receipt,
        ))
        receipt += period_s
    return tuple(samples)


def _mount(*, static=False, zoom=None):
    return CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(-30.0, 5.0, 0.0)),
        k=np.array([[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]]),
        dist=np.zeros(5),
        gimbal_timestamp_s=None,
        gimbal_is_static=static,
        zoom_command=zoom,
        zoom_sample_id="camera:fixed" if zoom is None else "7",
        zoom_sample_age_s=0.0,
    )


def _gimbal_history(start_s, end_s, period_s=0.05, session=1, lag_s=0.0):
    """A gimbal pitching 4 deg/s; each report reaches navpy lag_s after the plant held it."""
    samples = []
    seq = 65500                             # wraps inside the trace
    receipt = start_s
    while receipt <= end_s:
        samples.append(GimbalHistorySample(session, seq % 65536, receipt,
                                           Attitude(-30.0 + 4.0 * (receipt - lag_s - start_s), 5.0, 0.0)))
        seq += 1
        receipt += period_s
    return GimbalCaptureHistory(session, tuple(samples), GimbalData(att=samples[-1].att))


def _builder(integration, *, calibration=None, mount=None, history=(), link=LINK_ID):
    return FrameAssociationBuilder(
        attitude_sample_reader=lambda: history[-1] if history else None,
        mount_state_reader=lambda w, h: mount,
        location_reader=lambda: None,
        air_speed_reader=lambda: 18.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: link,
        calibration=calibration,
        source_id=SOURCE_ID,
        integration=integration,
    )


def _associate(*, mount=None, gimbal=..., attitude_yaw=0.0, link=LINK_ID, now_after_read_s=0.03,
               attitude_end_s=None, corrections=(0.0, 0.0), seconds=14.0, limits=LIMITS, fixed_zoom=None):
    """Publish a clean trace through the real store, then associate its last frame."""
    source = IntegrationTimingSource(_config(attitude_correction_s=corrections[0],
                                             gimbal_correction_s=corrections[1], limits=limits))
    store = FramePublicationStore(clock=lambda: 0.0, observer=source.observe)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frames = _trace(seconds)
    for index, (pts_ns, read_s, _) in enumerate(frames):
        store.publish(frame, _stamp(pts_ns, read_s, index))
    last_read = frames[-1][1]
    now_s = last_read + now_after_read_s
    history = _attitude_history(T0_S - 1.0, now_s if attitude_end_s is None else attitude_end_s,
                                yaw_deg=attitude_yaw)
    if gimbal is ...:
        gimbal = _gimbal_history(T0_S - 1.0, now_s)
    records = []
    integration = IntegrationAssociation(source, lambda: gimbal, records.append, fixed_zoom)
    builder = _builder(integration, mount=_mount() if mount is None else mount, history=history, link=link)
    with patch("navpy.core.perception.pipeline.real_frame_association.rt.perf_counter",
               return_value=now_s):
        association = builder.capture(store.snapshot(), frame)
    return association, records, frames[-1]


def test_dynamic_fixed_lens_gimbal_is_admitted_at_the_capture_instant_and_logged():
    association, records, (pts_ns, read_s, ideal_s) = _associate(corrections=(0.004, -0.003))
    assert association.pose_is_frame_atomic, association.pose_status
    capture = association.capture
    assert capture.authority == AUTHORITY_INTEGRATION and not is_certified(capture.authority)
    assert capture.interval_s == pts_ns / 1e9
    assert abs(capture.estimate_s - ideal_s) <= 0.001
    assert capture.lookup_s == pytest.approx(capture.estimate_s + 0.004)
    mount = association.mount_state
    gimbal_lookup = capture.estimate_s - 0.003
    assert mount.gimbal_capture_relative and mount.gimbal_timestamp_s == pytest.approx(gimbal_lookup)
    assert mount.gimbal_data.att.pitch == pytest.approx(-30.0 + 4.0 * (gimbal_lookup - (T0_S - 1.0)), abs=1e-6)
    [record] = [item for item in records if item["event"] == "association"]
    assert record is records[-1]     # the receipts it saw are logged before it
    assert record["status"] == "bracketed" and record["authority"] == AUTHORITY_INTEGRATION
    assert record["pts_ns"] == pts_ns and record["read_s"] == read_s
    assert record["attitude_time_boot_s"] == pytest.approx(association.attitude_boot_time_s)
    earlier_boot, later_boot = record["attitude_bracket_time_boot_s"]
    assert earlier_boot <= record["attitude_time_boot_s"] <= later_boot
    assert 0.0 <= record["attitude_weight"] <= 1.0 and 0.0 <= record["gimbal_weight"] <= 1.0
    assert record["gimbal_lookup_s"] == pytest.approx(gimbal_lookup)
    earlier_seq, later_seq = record["gimbal_seqs"]
    assert (later_seq - earlier_seq) % 65536 == 1 and record["gimbal_session"] == 1


def test_every_receipt_is_logged_once_before_the_first_record_that_could_see_it():
    source = IntegrationTimingSource(_config())
    store = FramePublicationStore(clock=lambda: 0.0, observer=source.observe)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frames = _trace(14.0)
    for index, (pts_ns, read_s, _) in enumerate(frames):
        store.publish(frame, _stamp(pts_ns, read_s, index))
    now_s = frames[-1][1] + 0.03
    attitude = _attitude_history(T0_S - 1.0, now_s)
    gimbal = _gimbal_history(T0_S - 1.0, now_s)
    views = [(attitude[:-5], GimbalCaptureHistory(1, gimbal.samples[:-3], gimbal.data)), (attitude, gimbal)]
    records = []
    current = {}
    integration = IntegrationAssociation(source, lambda: current["gimbal"], records.append)
    for attitude_view, gimbal_view in views:
        current["gimbal"] = gimbal_view
        builder = _builder(integration, mount=_mount(), history=attitude_view)
        with patch("navpy.core.perception.pipeline.real_frame_association.rt.perf_counter", return_value=now_s):
            builder.capture(store.snapshot(), frame)
    attitude_logged = [item["capture_receipt_s"] for item in records if item["event"] == "attitude_receipt"]
    gimbal_logged = [(item["session"], item["seq"]) for item in records if item["event"] == "gimbal_receipt"]
    # The association reads a window of attitude history: receipts are logged from the first window's start on.
    window_start = next(item for item in records if item["event"] == "association")["attitude_history"]
    assert attitude_logged == [sample.capture_receipt_s for sample in attitude
                               if sample.capture_receipt_s >= window_start["first_receipt_s"]]
    assert gimbal_logged == [(sample.session, sample.seq) for sample in gimbal.samples]
    first, second = [index for index, item in enumerate(records) if item["event"] == "association"]
    # The second record's new receipts come after the first record and before the second.
    late = [item for item in records[first + 1:second] if item["event"] != "association"]
    assert len(late) == 5 + 3
    for index, (attitude_view, gimbal_view) in zip((first, second), views):
        bounds = records[index]
        assert bounds["now_s"] == now_s
        assert bounds["attitude_history"]["last_receipt_s"] == attitude_view[-1].capture_receipt_s
        assert bounds["gimbal_history"]["count"] == len(gimbal_view.samples)
        assert bounds["gimbal_history"]["last_receipt_s"] == gimbal_view.samples[-1].receipt_s


@pytest.mark.parametrize("kwargs, status", [
    ({"mount": _mount(zoom="7")}, "real_frame_zoom_unsettled"),
    ({"gimbal": None}, "real_frame_gimbal_history_unavailable"),
    ({"link": "udp:127.0.0.1:14551#2"}, "real_frame_integration_context_mismatch"),
    # Older than max_frame_age_s, inside the gate's absolute 0.5 s ceiling.
    ({"now_after_read_s": 0.3, "limits": IntegrationTimingLimits(max_frame_age_s=0.2)},
     "real_frame_integration_stale"),
    ({"attitude_end_s": T0_S + 13.0}, "real_frame_integration_source_stale"),
    ({"seconds": 11.0}, "real_frame_integration_unsettled"),
])
def test_refusals_are_named_logged_and_never_frame_atomic(kwargs, status):
    association, records, _ = _associate(**kwargs)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == status
    assert records[-1]["status"] == status


def test_refused_frames_keep_the_pts_axis_for_trackers():
    association, _, (pts_ns, _, _) = _associate(seconds=11.0)
    assert association.pose_status == "real_frame_integration_unsettled"
    assert association.capture is None
    assert association.interval_timestamp_s == pts_ns / 1e9


def test_an_abort_during_association_refuses_the_frame():
    holder = {}

    def gimbal_then_reset():
        # The capture thread sees another open while this frame is associated.
        holder["source"].observe(_stamp(1, 1.0, open_id="open-2"))
        return holder["gimbal"]

    source = IntegrationTimingSource(_config())
    store = FramePublicationStore(clock=lambda: 0.0, observer=source.observe)
    frame = np.zeros((4, 4, 3), dtype=np.uint8)
    frames = _trace(14.0)
    for index, (pts_ns, read_s, _) in enumerate(frames):
        store.publish(frame, _stamp(pts_ns, read_s, index))
    now_s = frames[-1][1] + 0.03
    holder.update(source=source, gimbal=_gimbal_history(T0_S - 1.0, now_s))
    records = []
    integration = IntegrationAssociation(source, gimbal_then_reset, records.append)
    builder = _builder(integration, mount=_mount(), history=_attitude_history(T0_S - 1.0, now_s))
    with patch("navpy.core.perception.pipeline.real_frame_association.rt.perf_counter", return_value=now_s):
        association = builder.capture(store.snapshot(), frame)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == STATUS_ABORTED == records[-1]["status"]


REPORT_LAG_S = 0.04          # tests/tools/hitl/timing_scenario.py's report lag


@pytest.mark.parametrize("correction_s, error_deg", [(REPORT_LAG_S, 0.0), (0.0, -4.0 * REPORT_LAG_S)])
def test_a_known_gimbal_report_lag_is_removed_only_by_its_correction(correction_s, error_deg):
    """Step 17: the gimbal state navpy joins to a frame is the plant's at the capture instant once C_g holds the
    report lag; without it the join is one lag late, 4 deg/s * 40 ms here."""
    gimbal = _gimbal_history(T0_S - 1.0, T0_S + 15.0, lag_s=REPORT_LAG_S)
    association, records, _ = _associate(gimbal=gimbal, corrections=(0.0, correction_s), now_after_read_s=0.1)
    assert association.pose_is_frame_atomic, association.pose_status
    capture_s = association.capture.estimate_s
    plant_pitch = -30.0 + 4.0 * (capture_s - (T0_S - 1.0))
    assert association.mount_state.gimbal_data.att.pitch - plant_pitch == pytest.approx(error_deg, abs=1e-6)
    assert records[-1]["gimbal_lookup_s"] == pytest.approx(capture_s + correction_s)


def test_unbracketed_and_stale_gimbal_state_is_refused():
    association, _, _ = _associate(gimbal=_gimbal_history(T0_S - 1.0, T0_S + 13.0))
    assert association.pose_status == "real_frame_integration_source_stale"
    old = _gimbal_history(T0_S - 1.0, T0_S + 14.0)
    future_lookup = old.samples[-1].receipt_s + 0.01
    assert interpolate_gimbal_at(old, future_lookup, 0.1) == (None, "real_frame_gimbal_unbracketed")
    gapped = GimbalCaptureHistory(1, (old.samples[0], old.samples[5]), old.data)
    assert interpolate_gimbal_at(gapped, old.samples[2].receipt_s, 0.1) == (None, "real_frame_gimbal_gap")
    other_session = GimbalCaptureHistory(2, old.samples, old.data)
    assert interpolate_gimbal_at(other_session, old.samples[2].receipt_s, 0.1)[1] == "real_frame_gimbal_unbracketed"


def test_aircraft_heading_does_not_change_integration_timing():
    plain, _, _ = _associate()
    poisoned, _, _ = _associate(attitude_yaw=173.0)
    assert plain.capture == poisoned.capture
    assert plain.mount_state.gimbal_data.att == poisoned.mount_state.gimbal_data.att
    assert plain.pose_status == poisoned.pose_status == "real_frame_pose"


# --- gimbal history -------------------------------------------------------------


def test_gimbal_history_keeps_only_newer_sequences_per_session():
    store = SiyiReadbackStore(GimbalData(att=Attitude(0.0, 0.0, 0.0)))
    assert store.accept_attitude((65534, 1.00, 0.0, -10.0, 0.0))
    assert store.accept_attitude((65535, 1.05, 0.0, -11.0, 0.0))
    assert store.accept_attitude((0, 1.10, 0.0, -12.0, 0.0))          # wrap
    store.accept_attitude((65535, 1.15, 0.0, -11.0, 0.0))             # reordered, later receipt
    store.accept_attitude((65534, 1.20, 0.0, -10.0, 0.0))             # nonconsecutive duplicate
    history = store.capture_history()
    assert [sample.seq for sample in history.samples] == [65534, 65535, 0]
    store.reset_session()                                            # reconnect
    store.accept_attitude((7, 2.0, 0.0, -5.0, 0.0))
    history = store.capture_history()
    assert history.session == 1 and [sample.seq for sample in history.samples] == [7]


# --- the law's interval clock ----------------------------------------------------


def _frame(source_s, interval_s, elevation):
    z = math.sin(elevation)
    x = math.cos(elevation)
    return FinalApproachVisionFrame(
        source_name="zr10", source_generation=1, task_id=1, obj_id=1,
        source_timestamp_s=source_s, body_x=x, body_y=0.0, body_z=z,
        control_x=x, control_y=0.0, control_z=z, interval_timestamp_s=interval_s,
    )


def test_law_rates_difference_pts_not_mapped_or_read_time():
    first = _frame(1000.000, 7.0, 0.10)
    # Mapped time moved 50 ms (an offset update or jitter), PTS one frame.
    second = _frame(1000.050, 7.0 + 1.0 / FPS, 0.11)
    rates = VerticalRateFilter()
    rates.seed(first)
    plan = rates.plan(second, None)
    assert plan.next_state.timestamp_s == second.interval_timestamp_s
    lateral = LateralRateFilter()
    lateral.seed(first)
    assert lateral.plan(second).next_state.timestamp_s == second.interval_timestamp_s
    assert _frame(1.0, math.nan, 0.1).rate_time_s == 1.0
    with pytest.raises(ValueError, match="interval_timestamp_s"):
        _frame(1.0, math.inf, 0.1)
