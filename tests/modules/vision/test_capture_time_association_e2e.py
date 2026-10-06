"""The decisive asymmetric-delay end-to-end for capture-time association.

Decision doc §5.2-§5.3: attitude samples carry a SOURCE time and a LATER
receipt time (Da > 0); frames carry a true EXPOSURE time and a DIFFERENT
later readable time (Dc > 0, Dc != Da); nonzero fixed delay, asymmetric
jitter, and drift are all exercised through the REAL FramePublicationStore
and FrameAssociationBuilder. The associated attitude must match the
EXPOSURE-time truth — not receipt-time truth — and the pre-fix latest-sample
pairing, a wrong-sign bias, and an over-budget declaration must all fail.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMountFrameState
from navpy.modules.vision.capture_lookup import CaptureLookupCalibration
from navpy.modules.vision.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
)
from navpy.modules.vision.frame_publication import FramePublicationStore
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.real_frame_association import FrameAssociationBuilder

ROLL_RATE_DEG_S = 80.0
DA0_S = 0.012           # fixed ATTITUDE transport delay
DC0_S = 0.180           # fixed camera-path delay (a real latent camera)
ATTITUDE_PERIOD_S = 0.025
BASE_S = 1000.0


def _truth_roll(at_s: float) -> float:
    return ROLL_RATE_DEG_S * (at_s - BASE_S)


def _attitude_history(
    until_s: float,
    attitude_jitter=lambda index: 0.0,
) -> tuple[SimpleNamespace, ...]:
    """Samples generated at source times, stamped at their LATER receipts."""
    samples = []
    index = 0
    source = BASE_S
    while source <= until_s:
        receipt = source + DA0_S + attitude_jitter(index)
        samples.append(SimpleNamespace(
            attitude=Attitude(pitch=2.0, yaw=0.0, roll=_truth_roll(source)),
            body_rates_rad_s=(math.radians(ROLL_RATE_DEG_S), 0.0, 0.0),
            time_boot_s=source - BASE_S + 500.0,
            receipt_time_s=receipt,
        ))
        source += ATTITUDE_PERIOD_S
        index += 1
    return tuple(samples)


def _static_mount(width: int = 640, height: int = 480) -> CameraMountFrameState:
    # A truly FIXED camera: no zoom command at all. A zoom-CAPABLE camera is
    # not admitted for latent frames (association-time zoom cannot describe
    # a 180 ms-old exposure) -- covered by its own test below.
    return CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
        k=np.array(
            [[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]],
        ),
        dist=np.zeros(5),
        gimbal_timestamp_s=None,
        gimbal_is_static=True,
        zoom_command=None,
        zoom_sample_id="static:fixed",
        zoom_sample_age_s=0.0,
    )


def _calibration(**overrides: object) -> CaptureLookupCalibration:
    values: dict[str, object] = {
        "source_id": "rtsp-cam",
        "link_id": "udp:0.0.0.0:14560#1",
        "capture_lookup_bias_s": DC0_S - DA0_S,
        "attitude_link_delay_s": DA0_S,
        "residual_bound_s": 0.006,
        "camera_variation_bound_s": 0.004,
        "attitude_variation_bound_s": 0.004,
        "min_frame_interval_s": 0.050,
    }
    values.update(overrides)
    return CaptureLookupCalibration(**values)


def _associate(
    exposure_s: float,
    *,
    calibration: CaptureLookupCalibration,
    camera_jitter_s: float = 0.0,
    attitude_jitter=lambda index: 0.0,
    history=None,
):
    """One frame through the REAL publication store and association builder."""
    store = FramePublicationStore()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    read_s = exposure_s + DC0_S + camera_jitter_s
    store.publish(frame, CaptureStamp(read_s, CaptureStampKind.READ))
    snapshot = store.snapshot()
    assert snapshot.capture is not None and snapshot.published_at_s is not None

    if history is None:
        history = _attitude_history(exposure_s + 0.2, attitude_jitter)
    latest = history[-1] if history else None
    builder = FrameAssociationBuilder(
        attitude_sample_reader=lambda: latest,
        mount_state_reader=lambda w, h: _static_mount(w, h),
        location_reader=lambda: None,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=calibration,
        source_id="rtsp-cam",
    )
    associated_at = read_s + 0.005
    with patch(
        "navpy.modules.vision.real_frame_association.time.time",
        return_value=associated_at,
    ):
        return builder.capture(snapshot, frame)


def test_associated_attitude_matches_exposure_truth_not_receipt_truth() -> None:
    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=_calibration())

    assert association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_pose"
    truth = _truth_roll(exposure)
    error = abs(association.uas_att.roll - truth)
    # Tolerance: the roll the aircraft covers over the residual bound.
    assert error <= ROLL_RATE_DEG_S * 0.006 + 1e-6
    # Receipt-time truth is ~Dc0 newer and must NOT be what we matched:
    receipt_truth = _truth_roll(exposure + DC0_S)
    assert abs(association.uas_att.roll - receipt_truth) > 10.0
    # Geometry estimate is the exposure time to within the link delay:
    assert association.capture.estimate_s == exposure + DC0_S - (DC0_S - DA0_S) - DA0_S
    # Liveness stays publication-domain, later than the exposure:
    assert association.frame_timestamp_s > exposure + DC0_S


def test_latest_sample_pairing_fails_the_same_tolerance() -> None:
    # The PRE-FIX association: pair the newest sample. With a 180 ms latent
    # camera the newest attitude is ~180 ms newer than the light -- the
    # interpolation is load-bearing, not decorative.
    exposure = BASE_S + 1.0
    history = _attitude_history(exposure + 0.2)
    newest = history[-1]
    truth = _truth_roll(exposure)
    assert abs(newest.attitude.roll - truth) > ROLL_RATE_DEG_S * 0.006


def test_wrong_bias_is_caught_at_both_magnitudes() -> None:
    # Guards the ALGEBRA, not just the plumbing.
    # A SIGN-FLIPPED bias pushes the geometry estimate past association time
    # entirely: the negative-frame-age sanity check refuses the frame.
    exposure = BASE_S + 1.0
    flipped = _calibration(capture_lookup_bias_s=-(DC0_S - DA0_S))
    history = _attitude_history(exposure + 0.6)
    association = _associate(exposure, calibration=flipped, history=history)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_publication_stale"

    # A SUBTLY wrong bias (50 ms off) still brackets and stays "valid" --
    # only the truth comparison exposes it: the interpolated attitude misses
    # exposure truth by the full 50 ms of roll. This is what the lag-scan
    # certification exists to prevent.
    subtle = _calibration(capture_lookup_bias_s=DC0_S - DA0_S - 0.05)
    association = _associate(exposure, calibration=subtle, history=history)
    assert association.pose_is_frame_atomic
    error = abs(association.uas_att.roll - _truth_roll(exposure))
    assert error > ROLL_RATE_DEG_S * 0.006


def test_asymmetric_jitter_and_drift_stay_inside_certified_bounds() -> None:
    # Camera jitter one-sided (+4 ms), attitude delay drifting +-4 ms: both
    # inside the declared variation bounds; the association must stay within
    # the residual tolerance of exposure truth.
    for index, exposure in enumerate(BASE_S + 1.0 + np.arange(0.0, 0.5, 0.1)):
        association = _associate(
            float(exposure),
            calibration=_calibration(),
            camera_jitter_s=0.004 * ((index % 2)),
            attitude_jitter=lambda i: 0.004 * math.sin(i / 3.0),
        )
        assert association.pose_is_frame_atomic
        error = abs(association.uas_att.roll - _truth_roll(float(exposure)))
        # Jitter adds camera + attitude variation on top of the residual.
        assert error <= ROLL_RATE_DEG_S * (0.006 + 0.004 + 0.004) + 1e-6


def test_over_budget_declaration_fails_closed() -> None:
    exposure = BASE_S + 1.0
    over = _calibration(residual_bound_s=0.02)
    association = _associate(exposure, calibration=over)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_capture_uncalibrated"


def test_uncalibrated_read_source_fails_closed() -> None:
    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=None)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_capture_uncalibrated"


def test_calibrated_high_latency_passes_consistency_not_freshness() -> None:
    """§5.3: consistency and measurement-age admission are separate layers.

    A calibrated ~180 ms source PASSES association consistency; whether its
    measurement is fresh enough to act on is the detector freshness floor's
    decision, judged on the CAPTURE estimate. Both directions asserted, each
    naming the layer that decides.
    """
    from navpy.modules.vision.real_detector_state import FreshnessPolicy

    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=_calibration())
    assert association.pose_is_frame_atomic  # consistency layer: PASS

    policy = FreshnessPolicy(1.0 / 60.0)
    estimate = association.capture.estimate_s
    measurement = SimpleNamespace(timestamp=estimate)
    # ~184 ms measurement age: fresh under the 250 ms floor.
    assert policy.is_fresh(measurement, now_s=estimate + 0.184)
    # The same association is REJECTED by the freshness layer at 300 ms age.
    assert not policy.is_fresh(measurement, now_s=estimate + 0.300)


def test_exposure_kind_end_to_end_matches_truth_with_link_delay() -> None:
    # EXPOSURE algebra end-to-end: the stamp IS exposure time; the lookup
    # must add Da0 to land on the receipt axis.
    exposure = BASE_S + 1.0
    store = FramePublicationStore()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    store.publish(frame, CaptureStamp(exposure, CaptureStampKind.EXPOSURE))
    snapshot = store.snapshot()
    history = _attitude_history(exposure + 0.3)
    builder = FrameAssociationBuilder(
        attitude_sample_reader=lambda: history[-1],
        mount_state_reader=lambda w, h: _static_mount(w, h),
        location_reader=lambda: None,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=_calibration(),
        source_id="rtsp-cam",
    )
    with patch(
        "navpy.modules.vision.real_frame_association.time.time",
        return_value=exposure + 0.02,
    ):
        association = builder.capture(snapshot, frame)
    assert association.pose_is_frame_atomic
    error = abs(association.uas_att.roll - _truth_roll(exposure))
    assert error <= ROLL_RATE_DEG_S * 0.006 + 1e-6
    assert association.capture.estimate_s == exposure


def test_zoom_capable_camera_refused_for_latent_frames() -> None:
    # Post-review probe, pinned: static gimbal + zoom command + a 180 ms
    # latent READ frame must NOT be frame-atomic -- association-time zoom
    # cannot describe the optics at exposure.
    exposure = BASE_S + 1.0
    store = FramePublicationStore()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    read_s = exposure + DC0_S
    store.publish(frame, CaptureStamp(read_s, CaptureStampKind.READ))
    snapshot = store.snapshot()
    history = _attitude_history(exposure + 0.3)
    zooming_mount = CameraMountFrameState(
        gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
        k=np.array(
            [[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]],
        ),
        dist=np.zeros(5),
        gimbal_timestamp_s=None,
        gimbal_is_static=True,
        zoom_command="2.0",
        zoom_sample_id="static:2.0",
        zoom_sample_age_s=0.0,
    )
    builder = FrameAssociationBuilder(
        attitude_sample_reader=lambda: history[-1],
        mount_state_reader=lambda w, h: zooming_mount,
        location_reader=lambda: None,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=_calibration(),
        source_id="rtsp-cam",
    )
    with patch(
        "navpy.modules.vision.real_frame_association.time.time",
        return_value=read_s + 0.005,
    ):
        association = builder.capture(snapshot, frame)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_mount_dynamic_unsupported"

    # Round-3 probe, pinned: proximity proves nothing about optics -- a
    # YOUNG (20 ms) frame from the same zoom-capable camera is refused too.
    store = FramePublicationStore()
    store.publish(frame, CaptureStamp(exposure + 0.008, CaptureStampKind.READ))
    young = _calibration(capture_lookup_bias_s=0.008 - DA0_S)
    builder_young = FrameAssociationBuilder(
        attitude_sample_reader=lambda: history[-1],
        mount_state_reader=lambda w, h: zooming_mount,
        location_reader=lambda: None,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: history[-count:],
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=young,
        source_id="rtsp-cam",
    )
    with patch(
        "navpy.modules.vision.real_frame_association.time.time",
        return_value=exposure + 0.028,
    ):
        association = builder_young.capture(store.snapshot(), frame)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_mount_dynamic_unsupported"


def test_runtime_rate_tightens_the_bracket_ceiling() -> None:
    # Post-review probe, pinned: at an 80 Hz pose rate the two-period
    # ceiling is 25 ms; a 40 ms bracket admitted under the 40 Hz default
    # must be refused once the builder follows the runtime rate.
    from navpy.modules.vehicle.pose_streams import (
        pose_frame_association_max_skew_s,
    )

    exposure = BASE_S + 1.0
    lookup = exposure + DA0_S
    wide_history = (
        SimpleNamespace(
            attitude=Attitude(pitch=2.0, yaw=0.0, roll=0.0),
            body_rates_rad_s=None,
            time_boot_s=500.0,
            receipt_time_s=lookup - 0.02,
        ),
        SimpleNamespace(
            attitude=Attitude(pitch=2.0, yaw=0.0, roll=0.0),
            body_rates_rad_s=None,
            time_boot_s=500.04,
            receipt_time_s=lookup + 0.02,
        ),
    )
    association = _associate(
        exposure, calibration=_calibration(), history=wide_history,
    )
    assert association.pose_is_frame_atomic  # 40 ms bracket, 40 Hz bound

    store = FramePublicationStore()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    store.publish(
        frame, CaptureStamp(exposure + DC0_S, CaptureStampKind.READ),
    )
    builder = FrameAssociationBuilder(
        attitude_sample_reader=lambda: wide_history[-1],
        mount_state_reader=lambda w, h: _static_mount(w, h),
        location_reader=lambda: None,
        air_speed_reader=lambda: 25.0,
        attitude_history_reader=lambda count: wide_history,
        link_identity_reader=lambda: "udp:0.0.0.0:14560#1",
        calibration=_calibration(),
        source_id="rtsp-cam",
    )
    builder.set_maximum_skew(pose_frame_association_max_skew_s(100.0))
    with patch(
        "navpy.modules.vision.real_frame_association.time.time",
        return_value=exposure + DC0_S + 0.005,
    ):
        association = builder.capture(store.snapshot(), frame)
    assert not association.pose_is_frame_atomic
    assert association.pose_status == "real_frame_pose_gap"


def test_freshness_split_through_a_real_detected_poi() -> None:
    # Decision doc 5.3 through the REAL conversion: the DetectedObject
    # carries detection_timestamp_s = CAPTURE estimate, so the freshness
    # floor judges measurement age; a regression back to publication time in
    # _measurement_timestamp_s() fails this test.
    from navpy.modules.vision.detector import TrackedObject
    from navpy.modules.vision.real_detector_state import (
        ConfirmationFrameStore,
        FreshnessPolicy,
    )
    from navpy.modules.vision.real_detection_mapper import (
        DetectedObjectMapper,
        DetectionMappingConfig,
    )

    exposure = BASE_S + 1.0
    association = _associate(exposure, calibration=_calibration())
    assert association.pose_is_frame_atomic
    mapper = DetectedObjectMapper(
        DetectionMappingConfig(reference_height_m=2.0, output_mode="locked"),
        ConfirmationFrameStore(maximum_frames=4),
    )
    track = TrackedObject(
        id=7, cx=320.0, cy=240.0, w=80.0, h=40.0, confidence=0.9,
        class_id=3, age=1, hits=1, missed=0, is_confirmed=True,
        timestamp=exposure, vx=0.0, vy=0.0,
    )
    poi = mapper.convert([track], track, association)[0]
    assert (
        poi.timing.detection_timestamp_s == association.capture.estimate_s
    )
    assert (
        poi.timing.source_receipt_timestamp_s
        == association.frame_timestamp_s
    )
    policy = FreshnessPolicy(1.0 / 60.0)
    estimate = association.capture.estimate_s
    assert policy.is_fresh(poi, now_s=estimate + 0.184)
    assert not policy.is_fresh(poi, now_s=estimate + 0.300)
