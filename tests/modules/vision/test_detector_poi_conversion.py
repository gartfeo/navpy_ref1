import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.frame_projection import (
    FinalApproachFrameProjector,
    FinalApproachProjectionConfig,
)
from navpy.modules.vision.confirmation_frame import (
    should_replace_center_gated_confirmation_frame,
    should_replace_confirmation_frame,
)
from navpy.modules.vision.camera_mount import CameraMountFrameState
from navpy.modules.vision.detector import TrackedObject
from navpy.modules.vision.frame_provider import FrameSnapshot
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.real_detector_controls import (
    DetectionQuery,
    DetectorDebugConfig,
    DetectorDiagnostics,
)
from navpy.modules.vision.real_detector_lifecycle import PoseStreamRequester
from navpy.modules.vision.real_detector_state import (
    ConfirmationFrameStore,
    DetectionResultStore,
    DetectorRunState,
    FreshnessPolicy,
    OverlayStore,
    RuntimeMetrics,
)
from navpy.modules.vision.capture_lookup import CaptureLookupCalibration
from navpy.modules.vision.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
)
from navpy.modules.vision.real_frame_association import FrameAssociationBuilder
from navpy.modules.vision.real_detection_mapper import (
    DetectedObjectMapper,
    DetectionMappingConfig,
)
from tests.detection_factory import make_detected_poi


_UNSET_ATTITUDE_SAMPLE = object()
_UNSET = object()

# EXPOSURE with zero link delay and zero variation bounds: lookup == stamp ==
# geometry estimate, so expected values in these tests read directly off the
# constructed history. Exactly what a test artifact certifies (decision doc).
_TEST_CALIBRATION = CaptureLookupCalibration(
    source_id="test-cam",
    link_id="test-link",
    capture_lookup_bias_s=0.0,
    attitude_link_delay_s=0.0,
    residual_bound_s=0.0,
    camera_variation_bound_s=0.0,
    attitude_variation_bound_s=0.0,
    min_frame_interval_s=0.02,
)


class TestRequestPoseStreams(unittest.TestCase):
    def test_real_detector_requests_pose_at_ardupilot_mavlink_rate_limit(self):
        # The real detector must pin ATTITUDE + position to the pose rate on its
        # link so frame-time roll/pitch stays fresh at ArduPilot's
        # scheduler-derived 0.8 MAVLink limit -- same request the sim makes,
        # via the shared helper.
        from navpy.modules.vehicle.pose_streams import (
            MAV_CMD_SET_MESSAGE_INTERVAL,
            MAVLINK_MSG_ID_ATTITUDE,
            MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        )

        from navpy.modules.vehicle.pose_streams import (
            request_pose_streams,
            resolve_pose_stream_rate_hz,
        )

        vehicle = SimpleNamespace(
            send_command_long=Mock(),
            get_param_or_default=Mock(return_value=100.0),
        )
        logger = Mock()
        association_builder = Mock()
        requester = PoseStreamRequester(
            lambda: resolve_pose_stream_rate_hz(vehicle),
            lambda rate_hz: request_pose_streams(
                vehicle, logger, rate_hz=rate_hz,
            ),
            association_builder,
        )

        requester.request()

        interval_us = 12_500
        vehicle.send_command_long.assert_any_call(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            p1=MAVLINK_MSG_ID_ATTITUDE,
            p2=interval_us,
        )
        vehicle.send_command_long.assert_any_call(
            MAV_CMD_SET_MESSAGE_INTERVAL,
            p1=MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            p2=interval_us,
        )
        self.assertEqual(vehicle.send_command_long.call_count, 2)
        self.assertEqual(requester.rate_hz, 80.0)
        self.assertEqual(requester.maximum_skew_s, 0.025)
        association_builder.set_maximum_skew.assert_called_once_with(0.025)


class TestConfirmationFrameRanking(unittest.TestCase):
    def test_equal_size_more_centered_replaces_stored_frame(self):
        # Equal recognizable size (same w,h -> same diagonal); the more
        # centered candidate (lower score) wins the tie.
        replace = should_replace_confirmation_frame(
            candidate_bbox=(400.0, 220.0, 80.0, 60.0),
            candidate_center_score=0.10,
            stored_bbox=(320.0, 240.0, 80.0, 60.0),
            stored_center_score=0.20,
        )

        self.assertTrue(replace)

    def test_context_count_breaks_equal_size_and_center_tie(self):
        replace = should_replace_confirmation_frame(
            candidate_bbox=(320.0, 240.0, 80.0, 60.0),
            candidate_center_score=0.10,
            stored_bbox=(320.0, 240.0, 80.0, 60.0),
            stored_center_score=0.10,
            candidate_context_count=2,
            stored_context_count=1,
        )

        self.assertTrue(replace)

    def test_wider_candidate_outranks_equal_height_stored_frame(self):
        # The point of the diagonal measure: at equal height, a WIDER
        # POI (e.g. a wide detection) has a larger recognizable
        # size and wins — height-only ranking would have called it a tie.
        replace = should_replace_confirmation_frame(
            candidate_bbox=(320.0, 240.0, 140.0, 60.0),  # wider
            candidate_center_score=0.20,                  # less centered
            stored_bbox=(320.0, 240.0, 80.0, 60.0),
            stored_center_score=0.10,
        )

        self.assertTrue(replace)

    def test_center_gate_keeps_existing_centered_frame_over_off_center_candidate(self):
        replace = should_replace_center_gated_confirmation_frame(
            candidate_bbox=(900.0, 900.0, 120.0, 120.0),
            candidate_center_score=0.80,
            stored_bbox=(500.0, 500.0, 80.0, 80.0),
            stored_center_score=0.10,
        )

        self.assertFalse(replace)


class TestDetectorPoiConversion(unittest.TestCase):
    def _make_detector(self):
        vehicle = SimpleNamespace(
            attitude_sample=None,
            attitude_history=lambda count: (),
            link_identity="test-link",
            location=lambda _: None,
            air_speed=None,
        )
        k = np.array(
            [[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        # A truly FIXED camera: no zoom command. Zoom-capable cameras are
        # dynamic mounts under the capture-time association contract and are
        # never frame-atomic until capture-relative mount state exists.
        mount_state = CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
            k=k,
            dist=np.zeros(5, dtype=np.float32),
            gimbal_timestamp_s=None,
            gimbal_is_static=True,
            zoom_command=None,
            zoom_sample_id="static:fixed",
            zoom_sample_age_s=0.0,
        )
        mount = SimpleNamespace(
            frame_state=mount_state,
            capture_frame_state=None,
        )
        mount.capture_frame_state = lambda _w, _h: mount.frame_state
        confirmation_frames = ConfirmationFrameStore(maximum_frames=32)
        return SimpleNamespace(
            vehicle=vehicle,
            mount=mount,
            association=FrameAssociationBuilder(
                attitude_sample_reader=lambda: vehicle.attitude_sample,
                mount_state_reader=mount.capture_frame_state,
                location_reader=lambda: vehicle.location(False),
                air_speed_reader=lambda: vehicle.air_speed,
                attitude_history_reader=lambda count: vehicle.attitude_history(
                    count,
                ),
                link_identity_reader=lambda: vehicle.link_identity,
                calibration=_TEST_CALIBRATION,
                source_id="test-cam",
            ),
            mapper=DetectedObjectMapper(
                DetectionMappingConfig(reference_height_m=2.0, output_mode="locked"),
                confirmation_frames,
            ),
        )

    def _association(
            self,
            detector,
            *,
            frame=None,
            frame_ts=100.0,
            frame_sequence=1,
            attitude_sample=_UNSET_ATTITUDE_SAMPLE,
            associated_at_s=None,
            history=_UNSET,
    ):
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        if attitude_sample is _UNSET_ATTITUDE_SAMPLE:
            attitude_sample = SimpleNamespace(
                attitude=Attitude(3.0, 40.0, -5.0),
                time_boot_s=12.3,
                receipt_time_s=None if frame_ts is None else frame_ts - 0.01,
                body_rates_rad_s=(0.01, -0.02, 0.03),
            )
        detector.vehicle.attitude_sample = attitude_sample
        if history is _UNSET:
            # A tight, seam-free bracket around the lookup instant with the
            # SAME values on both ends, so the interpolated attitude equals
            # the sample the test constructed.
            if attitude_sample is None or frame_ts is None:
                history = ()
            else:
                boot = attitude_sample.time_boot_s or 0.0
                history = (
                    SimpleNamespace(
                        attitude=attitude_sample.attitude,
                        body_rates_rad_s=attitude_sample.body_rates_rad_s,
                        time_boot_s=boot - 0.01,
                        receipt_time_s=frame_ts - 0.01,
                    ),
                    SimpleNamespace(
                        attitude=attitude_sample.attitude,
                        body_rates_rad_s=attitude_sample.body_rates_rad_s,
                        time_boot_s=boot + 0.01,
                        receipt_time_s=frame_ts + 0.01,
                    ),
                )
        frozen_history = tuple(history)
        detector.vehicle.attitude_history = lambda count: frozen_history
        snapshot = FrameSnapshot(
            frame=frame,
            width=frame.shape[1],
            height=frame.shape[0],
            sequence=frame_sequence,
            published_at_s=frame_ts,
            capture=(
                None
                if frame_ts is None
                else CaptureStamp(frame_ts, CaptureStampKind.EXPOSURE)
            ),
        )
        if associated_at_s is None:
            associated_at_s = frame_ts + 0.01
        with patch(
                "navpy.modules.vision.real_frame_association.time.time",
                return_value=associated_at_s,
        ):
            return detector.association.capture(snapshot, frame)

    @staticmethod
    def _track(obj_id, cx, cy, w, h, timestamp):
        return TrackedObject(
            id=obj_id,
            cx=float(cx),
            cy=float(cy),
            w=float(w),
            h=float(h),
            confidence=0.92,
            class_id=3,
            age=1,
            hits=1,
            missed=0,
            is_confirmed=True,
            timestamp=float(timestamp),
            vx=5.0,
            vy=-2.0,
        )

    def test_convert_to_pois_keeps_live_bbox_separate_from_confirmation_bbox(self):
        detector = self._make_detector()
        first_frame = np.full((480, 640, 3), 10, dtype=np.uint8)

        centered = self._track(7, 320, 240, 80, 40, 1.0)
        first_association = self._association(
            detector,
            frame=first_frame,
            frame_ts=1.0,
            frame_sequence=10,
        )
        first = detector.mapper.convert(
            [centered], centered, first_association,
        )[0]
        self.assertEqual(first.tracking.bbox_cxcywh, (320.0, 240.0, 80.0, 40.0))
        self.assertEqual(first.confirmation.bbox_cxcywh, (320.0, 240.0, 80.0, 40.0))
        self.assertEqual(int(first.confirmation.frame[0, 0, 0]), 10)

        later_frame = np.full((480, 640, 3), 200, dtype=np.uint8)
        off_center = self._track(7, 560, 420, 100, 60, 2.0)
        later_association = self._association(
            detector,
            frame=later_frame,
            frame_ts=2.0,
            frame_sequence=11,
        )
        second = detector.mapper.convert(
            [off_center], off_center, later_association,
        )[0]

        self.assertEqual(second.tracking.bbox_cxcywh, (560.0, 420.0, 100.0, 60.0))
        self.assertEqual(second.confirmation.bbox_cxcywh, (560.0, 420.0, 100.0, 60.0))
        self.assertEqual(int(second.confirmation.frame[0, 0, 0]), 200)

    def test_frame_time_pose_populates_provenance_and_optional_diagnostics(self):
        # A frame-time attitude sample must ride along on the detection so it can
        # reach the certified body-fixed final-approach path: frame-time attitude (NOT
        # a conversion-time re-read), optional gyro diagnostics, atomic provenance,
        # per-stream timestamps.
        detector = self._make_detector()
        frame_att = Attitude(3.0, 40.0, -5.0)
        attitude_sample = SimpleNamespace(
            attitude=frame_att,
            time_boot_s=123.4,
            receipt_time_s=555.48,
            body_rates_rad_s=(0.01, -0.02, 0.03),
        )
        trk = self._track(7, 320, 240, 80, 40, timestamp=99.5)
        association = self._association(
            detector,
            frame_ts=555.5,
            frame_sequence=77,
            attitude_sample=attitude_sample,
            associated_at_s=555.51,
        )

        dt = detector.mapper.convert(
            [trk], trk, association,
        )[0]

        # CAPTURE-time attitude is used: interpolated at the frame's capture
        # instant, NOT the vehicle's live (conversion-time) sample.
        self.assertEqual(dt.pose.aircraft_attitude, frame_att)
        self.assertIsNot(dt.pose.aircraft_attitude, frame_att)
        self.assertEqual(dt.pose.body_rates_rad_s, (0.01, -0.02, 0.03))
        self.assertIs(dt.pose.is_frame_atomic, True)
        self.assertEqual(dt.pose.status, "real_frame_pose")
        self.assertAlmostEqual(dt.pose.pose_age_s, 0.01)
        self.assertFalse(dt.geo.is_simulation)
        self.assertEqual(dt.timing.camera_frame_timestamp_s, 555.5)
        self.assertEqual(dt.pose.gimbal_attitude_timestamp_s, 555.5)
        self.assertEqual(dt.timing.tracker_timestamp_s, 99.5)
        # The pose timestamp is the receipt-axis LOOKUP instant (== the
        # capture stamp for a zero-link-delay EXPOSURE calibration).
        self.assertEqual(dt.pose.pose_timestamp_s, 555.5)
        self.assertEqual(dt.pose.vehicle_attitude_timestamp_s, 555.5)
        # Receipt liveness keeps the PUBLICATION stamp, separate by design.
        self.assertEqual(dt.timing.source_receipt_timestamp_s, 555.5)
        self.assertEqual(dt.optics.camera_frame_sequence, 77)
        self.assertEqual(dt.optics.sample_id, "static:fixed")
        self.assertIsNone(dt.optics.zoom_command)
        self.assertEqual(dt.optics.frame_width_px, 640.0)
        self.assertEqual(dt.optics.frame_height_px, 480.0)
        # timestamp_now provider is the same wall clock as t.timestamp.
        self.assertIs(dt.timing.detection_now_s, time.time)

    def test_missing_attitude_sample_fails_closed_without_live_pose_fallback(self):
        # Missing frame-associated pose must never trigger a conversion-time
        # re-read that combines this frame with unrelated live state.
        detector = self._make_detector()
        trk = self._track(7, 320, 240, 80, 40, timestamp=42.0)
        association = self._association(
            detector,
            frame_ts=42.0,
            attitude_sample=None,
        )

        dt = detector.mapper.convert(
            [trk], trk, association,
        )[0]

        self.assertIsNone(dt.pose.aircraft_attitude)
        self.assertIsNone(dt.pose.body_rates_rad_s)
        self.assertIs(dt.pose.is_frame_atomic, False)
        # No retained samples can bracket the capture instant.
        self.assertEqual(dt.pose.status, "real_frame_pose_unbracketed")
        self.assertAlmostEqual(dt.pose.pose_age_s, 0.01)
        self.assertEqual(dt.timing.camera_frame_timestamp_s, 42.0)
        self.assertFalse(dt.geo.is_simulation)

    def test_missing_frame_timestamp_remains_visible_but_final_approach_rejects(self):
        detector = self._make_detector()
        trk = self._track(7, 320, 240, 80, 40, timestamp=42.0)
        association = self._association(
            detector,
            frame_ts=None,
            attitude_sample=None,
            associated_at_s=100.0,
        )

        dt = detector.mapper.convert([trk], trk, association)[0]

        self.assertIsNone(dt.pixel.source_timestamp_s)
        self.assertIs(dt.pose.is_frame_atomic, False)
        self.assertEqual(
            dt.pose.status, "real_frame_capture_timestamp_unavailable",
        )
        self.assertIsNone(
            FinalApproachFrameProjector(
                FinalApproachProjectionConfig("ZYX", True)
            ).project(dt.visual_detection(), 0)
        )

    def test_attitude_sample_without_rates_remains_frame_atomic(self):
        detector = self._make_detector()
        attitude_sample = SimpleNamespace(
            attitude=Attitude(1.0, 2.0, 3.0),
            time_boot_s=10.0,
            receipt_time_s=1.0,
            body_rates_rad_s=None,
        )
        trk = self._track(7, 320, 240, 80, 40, timestamp=5.0)
        association = self._association(
            detector,
            frame_ts=1.0,
            attitude_sample=attitude_sample,
        )

        dt = detector.mapper.convert(
            [trk], trk, association,
        )[0]

        self.assertIs(dt.pose.is_frame_atomic, True)
        self.assertEqual(dt.pose.status, "real_frame_pose")
        self.assertIsNone(dt.pose.body_rates_rad_s)

    def test_frame_a_conversion_never_reads_frame_b_pose_or_optics(self):
        detector = self._make_detector()
        frame_a = np.full((480, 640, 3), 10, dtype=np.uint8)
        pose_a = SimpleNamespace(
            attitude=Attitude(1.0, 2.0, 3.0),
            time_boot_s=1.0,
            receipt_time_s=100.0,
            body_rates_rad_s=(0.1, 0.2, 0.3),
        )
        association_a = self._association(
            detector,
            frame=frame_a,
            frame_ts=100.0,
            frame_sequence=41,
            attitude_sample=pose_a,
            associated_at_s=100.01,
        )

        # A later publication updates every mutable source before conversion.
        detector.vehicle.attitude_sample = SimpleNamespace(
            attitude=Attitude(91.0, 92.0, 93.0),
            time_boot_s=2.0,
            receipt_time_s=100.02,
            body_rates_rad_s=(9.1, 9.2, 9.3),
        )
        k_b = np.array(
            [[2000.0, 0.0, 320.0], [0.0, 2000.0, 240.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        detector.mount.frame_state = CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(20.0, 21.0, 22.0)),
            k=k_b,
            dist=np.zeros(5),
            gimbal_timestamp_s=None,
            gimbal_is_static=True,
            zoom_command="2.0",
            zoom_sample_id="static:2.0",
            zoom_sample_age_s=0.0,
        )
        detector.vehicle.location = Mock(
            side_effect=AssertionError("conversion re-read live vehicle state")
        )

        trk = self._track(7, 320, 240, 80, 40, timestamp=100.03)
        dt = detector.mapper.convert(
            [trk], trk, association_a,
        )[0]

        self.assertEqual(int(dt.confirmation.frame[0, 0, 0]), 10)
        self.assertEqual(dt.optics.camera_frame_sequence, 41)
        self.assertEqual(dt.pose.aircraft_attitude, pose_a.attitude)
        self.assertIsNot(dt.pose.aircraft_attitude, pose_a.attitude)
        self.assertEqual(dt.pose.body_rates_rad_s, (0.1, 0.2, 0.3))
        self.assertEqual(dt.pose.gimbal_data.att, Attitude(0, 0, 0))
        np.testing.assert_array_equal(
            dt.optics.camera_matrix(),
            [[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]],
        )
        self.assertEqual(dt.optics.sample_id, "static:fixed")

    def test_stale_pose_is_measured_and_rejected(self):
        # Every retained sample is OLDER than the capture instant: strict
        # bracketing refuses (no nearest-sample, no extrapolation), so the
        # stale pose can never de-rotate this frame.
        detector = self._make_detector()
        stale_pose = SimpleNamespace(
            attitude=Attitude(1.0, 2.0, 3.0),
            time_boot_s=5.0,
            receipt_time_s=99.8,
            body_rates_rad_s=(0.1, 0.2, 0.3),
        )
        stale_history = (
            SimpleNamespace(
                attitude=stale_pose.attitude,
                body_rates_rad_s=stale_pose.body_rates_rad_s,
                time_boot_s=4.95,
                receipt_time_s=99.75,
            ),
            SimpleNamespace(
                attitude=stale_pose.attitude,
                body_rates_rad_s=stale_pose.body_rates_rad_s,
                time_boot_s=5.0,
                receipt_time_s=99.8,
            ),
        )
        association = self._association(
            detector,
            frame_ts=100.0,
            attitude_sample=stale_pose,
            associated_at_s=100.01,
            history=stale_history,
        )
        trk = self._track(7, 320, 240, 80, 40, timestamp=100.02)

        dt = detector.mapper.convert(
            [trk], trk, association,
        )[0]

        self.assertIs(dt.pose.is_frame_atomic, False)
        self.assertEqual(dt.pose.status, "real_frame_pose_unbracketed")
        self.assertAlmostEqual(dt.pose.pose_age_s, 0.01)
        self.assertIsNone(
            FinalApproachFrameProjector(
                FinalApproachProjectionConfig("ZYX", True)
            ).project(dt.visual_detection(), 0)
        )

    def test_dynamic_mount_fails_closed_for_every_stamp_kind(self):
        """A dynamic mount cannot be frame-atomic in this change.

        Mount samples are association-time-relative and cannot describe the
        capture instant of a latent frame; an EXPOSURE stamp times the frame,
        not the mount. Fail closed until a capture-relative mount-state
        contract exists (capture-time association decision doc §3d) --
        previously a BOUNDED dynamic mount passed, which was unsound for any
        latent source.
        """
        detector = self._make_detector()
        detector.mount.frame_state = CameraMountFrameState(
            gimbal_data=GimbalData(
                att=Attitude(-20.0, 10.0, 5.0),
                timestamp_s=100.02,
            ),
            k=detector.mount.frame_state.k,
            dist=detector.mount.frame_state.dist,
            gimbal_timestamp_s=100.02,
            gimbal_is_static=False,
            zoom_command="2.0",
            zoom_sample_id="zoom:8",
            zoom_sample_age_s=0.03,
        )

        association = self._association(
            detector,
            frame_ts=100.0,
            associated_at_s=100.04,
        )

        self.assertFalse(association.pose_is_frame_atomic)
        self.assertEqual(
            association.pose_status, "real_frame_mount_dynamic_unsupported",
        )

    def test_packet_loss_gap_between_bracket_samples_is_rejected(self):
        # Adjacent retained samples CAN sit far apart after packet loss;
        # bracketing across such a gap would interpolate across unknown
        # motion. The bracket-width bound (two ATTITUDE periods, derived
        # from the stream's own cadence constants) refuses it.
        detector = self._make_detector()
        sample = SimpleNamespace(
            attitude=Attitude(3.0, 40.0, -5.0),
            time_boot_s=12.3,
            receipt_time_s=99.75,
            body_rates_rad_s=None,
        )
        gap_history = (
            SimpleNamespace(
                attitude=sample.attitude,
                body_rates_rad_s=None,
                time_boot_s=12.05,
                receipt_time_s=99.75,
            ),
            SimpleNamespace(
                attitude=sample.attitude,
                body_rates_rad_s=None,
                time_boot_s=12.55,
                receipt_time_s=100.25,
            ),
        )

        rejected = self._association(
            detector,
            frame_ts=100.0,
            associated_at_s=100.01,
            attitude_sample=sample,
            history=gap_history,
        )
        self.assertFalse(rejected.pose_is_frame_atomic)
        self.assertEqual(rejected.pose_status, "real_frame_pose_gap")

    def test_dynamic_gimbal_without_bounded_timestamp_fails_closed(self):
        detector = self._make_detector()
        detector.mount.frame_state = CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(-20.0, 10.0, 5.0)),
            k=detector.mount.frame_state.k,
            dist=detector.mount.frame_state.dist,
            gimbal_timestamp_s=None,
            gimbal_is_static=False,
            zoom_command="2.0",
            zoom_sample_id="zoom:8",
            zoom_sample_age_s=0.03,
        )

        association = self._association(
            detector,
            frame_ts=100.0,
            associated_at_s=100.04,
        )

        self.assertFalse(association.pose_is_frame_atomic)
        # Dynamic mounts are refused wholesale now; the missing gimbal
        # timestamp no longer needs its own branch to keep this closed.
        self.assertEqual(
            association.pose_status,
            "real_frame_mount_dynamic_unsupported",
        )

    def test_stale_zoom_readback_fails_closed(self):
        # Zoom skew stays guarded (defense in depth) even though the real
        # static-mount path stamps zoom age 0: a synthetic static state with
        # an old zoom sample must still be refused.
        detector = self._make_detector()
        detector.mount.frame_state = CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
            k=detector.mount.frame_state.k,
            dist=detector.mount.frame_state.dist,
            gimbal_timestamp_s=None,
            gimbal_is_static=True,
            zoom_command=None,
            zoom_sample_id="zoom:7",
            zoom_sample_age_s=0.2,
        )

        association = self._association(
            detector,
            frame_ts=100.0,
            associated_at_s=100.02,
        )

        self.assertFalse(association.pose_is_frame_atomic)
        self.assertEqual(association.pose_status, "real_frame_zoom_skew")
        self.assertAlmostEqual(association.pose_age_s, 0.2)

    def test_unproven_zoom_stays_visible_but_final_approach_rejects_it(self):
        detector = self._make_detector()
        detector.mount.frame_state = CameraMountFrameState(
            gimbal_data=GimbalData(att=Attitude(0, 0, 0)),
            k=detector.mount.frame_state.k,
            dist=detector.mount.frame_state.dist,
            gimbal_timestamp_s=None,
            gimbal_is_static=True,
            zoom_command=None,
            zoom_sample_id=None,
            zoom_sample_age_s=None,
        )
        association = self._association(
            detector,
            frame_ts=100.0,
            associated_at_s=100.02,
        )
        trk = self._track(7, 320, 240, 80, 40, timestamp=100.02)

        pois = detector.mapper.convert(
            [trk], trk, association,
        )

        self.assertEqual(len(pois), 1)
        self.assertFalse(pois[0].pose.is_frame_atomic)
        self.assertEqual(
            pois[0].pose.status,
            "real_frame_zoom_timestamp_unavailable",
        )
        self.assertIsNone(
            FinalApproachFrameProjector(
                FinalApproachProjectionConfig("ZYX", True)
            ).project(pois[0].visual_detection(), 0)
        )

    def test_get_detect_data_hides_stale_cached_poi(self):
        results = DetectionResultStore()
        stale = make_detected_poi(obj_id=7, task_id=7, timestamp=99.0)
        results.publish([stale], stale)
        query = DetectionQuery(
            results,
            FreshnessPolicy(1.0 / 60.0),
            Mock(),
            True,
        )

        with patch(
                "navpy.modules.vision.real_detector_controls.time.time",
                return_value=100.0,
        ):
            resp = query.get_detect_data(DetectRequest())

        self.assertEqual(resp.detected_pois, [])
        self.assertIsNone(resp.primary_poi)

    def test_overlay_tracks_hide_stale_cached_boxes(self):
        fresh = self._track(1, 320, 240, 80, 40, 99.9)
        stale = self._track(2, 300, 200, 80, 40, 99.0)
        overlays = OverlayStore()
        overlays.publish([fresh, stale], None)
        diagnostics = DetectorDiagnostics(
            DetectorRunState(),
            Mock(),
            overlays,
            FreshnessPolicy(1.0 / 60.0),
            RuntimeMetrics(),
            Mock(counters={}),
            Mock(),
            True,
            DetectorDebugConfig(),
            Mock(),
        )

        with patch(
                "navpy.modules.vision.real_detector_controls.time.time",
                return_value=100.0,
        ):
            tracks = diagnostics.get_overlay_tracks()

        self.assertEqual([track.id for track in tracks], [1])


if __name__ == "__main__":
    unittest.main()
