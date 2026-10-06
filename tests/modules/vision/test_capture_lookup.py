"""Units for the capture-lookup calibration contract and interpolation.

Implements §5.1 of the capture-time association decision doc (v7): every
refusal reachable, every bound boundary-tested, the review counterexamples
pinned as named regressions.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.capture_lookup import (
    BRACKET_MAX_WIDTH_S,
    CLOCK_SEAM_TOLERANCE_S,
    CaptureLookupCalibration,
    RATE_AMPLIFICATION_LIMIT,
    STATUS_CLOCK_SEAM,
    STATUS_GAP,
    STATUS_UNBRACKETED,
    TIMING_RESIDUAL_BUDGET_S,
    attitude_lookup_wall_s,
    capture_estimate_s,
    interpolate_attitude_at,
    quarantine_window_s,
    validate_calibration,
)
from navpy.modules.vision.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
)


def _calibration(**overrides: object) -> CaptureLookupCalibration:
    values: dict[str, object] = {
        "source_id": "cam",
        "link_id": "link#1",
        "capture_lookup_bias_s": 0.010,
        "attitude_link_delay_s": 0.004,
        "residual_bound_s": 0.004,
        "camera_variation_bound_s": 0.004,
        "attitude_variation_bound_s": 0.004,
        "min_frame_interval_s": 0.050,
    }
    values.update(overrides)
    return CaptureLookupCalibration(**values)


def _sample(receipt: float, boot: float, roll: float = 0.0) -> SimpleNamespace:
    return SimpleNamespace(
        attitude=Attitude(pitch=1.0, yaw=0.0, roll=roll),
        body_rates_rad_s=(0.0, 0.0, 0.0),
        time_boot_s=boot,
        receipt_time_s=receipt,
    )


class TestCalibrationValidation:
    def test_valid_artifact_admits(self) -> None:
        assert validate_calibration(_calibration(), "cam", "link#1") is None

    def test_absent_artifact_refuses(self) -> None:
        assert validate_calibration(None, "cam", "link#1") is not None

    def test_source_and_link_binding(self) -> None:
        assert validate_calibration(_calibration(), "other", "link#1")
        assert validate_calibration(_calibration(), "cam", "link#2")
        # A live identity that never materialized must not admit anything.
        assert validate_calibration(_calibration(), "cam", None)

    @pytest.mark.parametrize("field", [
        "capture_lookup_bias_s", "attitude_link_delay_s", "residual_bound_s",
        "camera_variation_bound_s", "attitude_variation_bound_s",
        "min_frame_interval_s",
    ])
    def test_non_finite_fields_refuse(self, field: str) -> None:
        artifact = _calibration(**{field: float("nan")})
        assert validate_calibration(artifact, "cam", "link#1") is not None

    def test_physical_ceilings(self) -> None:
        assert validate_calibration(
            _calibration(capture_lookup_bias_s=0.6), "cam", "link#1")
        assert validate_calibration(
            _calibration(attitude_link_delay_s=-0.001), "cam", "link#1")
        assert validate_calibration(
            _calibration(attitude_link_delay_s=0.6), "cam", "link#1")

    @pytest.mark.parametrize("field", [
        "residual_bound_s", "camera_variation_bound_s",
        "attitude_variation_bound_s",
    ])
    def test_declared_over_budget_bounds_refuse_at_runtime(
        self, field: str,
    ) -> None:
        # Interval large enough that the amplification inequality never
        # interferes with the budget bound under test.
        at_budget = _calibration(
            min_frame_interval_s=0.2, **{field: TIMING_RESIDUAL_BUDGET_S},
        )
        assert validate_calibration(at_budget, "cam", "link#1") is None
        over = _calibration(
            min_frame_interval_s=0.2,
            **{field: TIMING_RESIDUAL_BUDGET_S + 1e-6},
        )
        assert validate_calibration(over, "cam", "link#1") is not None

    def test_rate_amplification_admission_boundary(self) -> None:
        bound = 0.004
        required = (
            RATE_AMPLIFICATION_LIMIT / (RATE_AMPLIFICATION_LIMIT - 1.0)
            * 2.0 * bound
        )
        exact = _calibration(
            camera_variation_bound_s=bound, min_frame_interval_s=required,
        )
        assert validate_calibration(exact, "cam", "link#1") is None
        below = _calibration(
            camera_variation_bound_s=bound,
            min_frame_interval_s=required - 1e-6,
        )
        assert validate_calibration(below, "cam", "link#1") is not None
        zero_variation = _calibration(
            camera_variation_bound_s=0.0, min_frame_interval_s=1e-6,
        )
        # Sim/test artifacts with zero variation are unconstrained.
        assert validate_calibration(zero_variation, "cam", "link#1") is None


class TestLookupAlgebra:
    def test_read_lookup_subtracts_bias(self) -> None:
        stamp = CaptureStamp(100.0, CaptureStampKind.READ)
        assert attitude_lookup_wall_s(stamp, _calibration()) == 100.0 - 0.010

    def test_exposure_lookup_adds_link_delay(self) -> None:
        stamp = CaptureStamp(100.0, CaptureStampKind.EXPOSURE)
        assert attitude_lookup_wall_s(stamp, _calibration()) == 100.0 + 0.004

    def test_capture_estimate_removes_link_delay(self) -> None:
        assert capture_estimate_s(100.004, _calibration()) == 100.0

    def test_quarantine_windows_per_kind(self) -> None:
        artifact = _calibration()
        assert quarantine_window_s(
            CaptureStampKind.EXPOSURE, artifact) == 0.004
        assert quarantine_window_s(
            CaptureStampKind.READ, artifact) == 0.004 + 0.004


class TestInterpolation:
    def test_bracketed_interpolation_is_exact_at_fraction(self) -> None:
        history = (
            _sample(100.00, 50.00, roll=10.0),
            _sample(100.02, 50.02, roll=20.0),
        )
        result, status = interpolate_attitude_at(history, 100.015, 0.0)
        assert status == "bracketed"
        assert result.attitude.roll == pytest.approx(17.5)
        assert result.time_boot_s == pytest.approx(50.015)

    def test_shortest_arc_across_the_wrap(self) -> None:
        history = (
            _sample(100.00, 50.00, roll=179.0),
            _sample(100.02, 50.02, roll=-179.0),
        )
        result, status = interpolate_attitude_at(history, 100.01, 0.0)
        assert status == "bracketed"
        assert abs(abs(result.attitude.roll) - 180.0) == pytest.approx(0.0)

    def test_future_lookup_refused_even_by_a_hair(self) -> None:
        history = (_sample(100.00, 50.00), _sample(100.02, 50.02))
        result, status = interpolate_attitude_at(history, 100.021, 0.0)
        assert result is None and status == STATUS_UNBRACKETED

    def test_older_than_window_refused(self) -> None:
        history = (_sample(100.00, 50.00), _sample(100.02, 50.02))
        result, status = interpolate_attitude_at(history, 99.99, 0.0)
        assert result is None and status == STATUS_UNBRACKETED

    def test_packet_loss_gap_refused(self) -> None:
        history = (
            _sample(100.00, 50.00),
            _sample(100.00 + BRACKET_MAX_WIDTH_S + 0.01,
                    50.00 + BRACKET_MAX_WIDTH_S + 0.01),
        )
        result, status = interpolate_attitude_at(history, 100.03, 0.0)
        assert result is None and status == STATUS_GAP

    def test_boot_time_reboot_is_a_seam(self) -> None:
        history = (_sample(100.00, 50.00), _sample(100.02, 0.01))
        result, status = interpolate_attitude_at(history, 100.01, 0.0)
        assert result is None and status == STATUS_CLOCK_SEAM

    def test_receipt_boot_disagreement_boundary(self) -> None:
        # Boot progresses (no backwards step); only the |dr - db|
        # disagreement crosses the tolerance between the two cases.
        # dr = 0.04, tolerance = 0.025: db = 0.016 disagrees by 0.024 (in),
        # db = 0.014 disagrees by 0.026 (out).
        inside = (
            _sample(100.00, 50.00),
            _sample(100.04, 50.00 + 0.04 - CLOCK_SEAM_TOLERANCE_S + 0.001),
        )
        result, status = interpolate_attitude_at(inside, 100.01, 0.0)
        assert status == "bracketed"
        outside = (
            _sample(100.00, 50.00),
            _sample(100.04, 50.00 + 0.04 - CLOCK_SEAM_TOLERANCE_S - 0.001),
        )
        result, status = interpolate_attitude_at(outside, 100.01, 0.0)
        assert result is None and status == STATUS_CLOCK_SEAM

    def test_wall_clock_step_forward_is_a_seam(self) -> None:
        # Receipt jumps 1 s while boot advances 20 ms: |dr - db| >> tolerance.
        history = (
            _sample(100.00, 50.00),
            _sample(101.00, 50.02),
            _sample(101.02, 50.04),
        )
        result, status = interpolate_attitude_at(history, 101.01, 0.0)
        assert status == "bracketed"
        # ...but a bracket ACROSS the step is refused (also a gap; the seam
        # check fires first on the pair).
        result, status = interpolate_attitude_at(history, 100.5, 0.0)
        assert result is None and status == STATUS_CLOCK_SEAM

    def test_quarantine_refuses_frame_exposed_before_a_seam(self) -> None:
        """Review round-4 counterexample, pinned.

        Da0=40 ms, Dc0=20 ms, zero variation: bias=-20 ms, so a READ stamp at
        E+0.02 looks up at E+0.04. A seam 10 ms after exposure (30 ms before
        the lookup) sits OUTSIDE the retracted v4 window (Dc0+residual=20 ms)
        but INSIDE the correct one (Da0 + max positive dc = 40 ms).
        """
        exposure = 100.0
        lookup = exposure + 0.04
        seam_receipt = exposure + 0.01
        history = (
            _sample(seam_receipt - 0.02, 50.00),
            # boot jumps backwards: a seam lands at seam_receipt.
            _sample(seam_receipt, 10.00),
            _sample(lookup - 0.01, 10.01),
            _sample(lookup + 0.01, 10.03),
        )
        correct_window = 0.040  # attitude_link_delay + camera variation
        result, status = interpolate_attitude_at(
            history, lookup, correct_window,
        )
        assert result is None and status == STATUS_CLOCK_SEAM
        retracted_v4_window = 0.020
        result, status = interpolate_attitude_at(
            history, lookup, retracted_v4_window,
        )
        assert status == "bracketed"  # exactly why v4's formula was wrong


class TestImplementationChecks:
    """Decision doc §5.6 carried checks."""

    def test_opencv_backend_stamps_read_kind_at_grab(self) -> None:
        import numpy as np

        from navpy.modules.vision.frame_capture_opencv import (
            OpenCvFrameCapture,
        )

        class _Stub:
            def __init__(self) -> None:
                self.grabs = 0

            def grab(self) -> bool:
                self.grabs += 1
                return self.grabs <= 2

            def retrieve(self):
                return True, np.zeros((2, 2, 3), dtype=np.uint8)

            def release(self) -> None:
                pass

        published: list[CaptureStamp] = []
        import time as _time
        begin = _time.time()
        running = {"count": 0}

        def is_running() -> bool:
            # The run loop consults this three times per iteration (loop
            # head, pre-grab, post-grab); allow two full iterations.
            running["count"] += 1
            return running["count"] <= 6

        OpenCvFrameCapture(_Stub()).run(
            lambda frame, capture: published.append(capture), is_running,
        )
        assert len(published) == 2
        assert all(
            stamp.kind is CaptureStampKind.READ for stamp in published
        )
        assert begin <= published[0].captured_at_s <= published[1].captured_at_s
        assert published[1].captured_at_s <= _time.time()

    def test_message_store_recent_is_immutable_under_publish(self) -> None:
        import threading

        from navpy.modules.vehicle.message_store import MessageStore

        store = MessageStore()
        stop = threading.Event()

        def writer() -> None:
            index = 0
            while not stop.is_set():
                store.publish(
                    "ATTITUDE", SimpleNamespace(),
                    receipt_time_s=float(index), boot_time_ms=index,
                )
                index += 1

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            for _ in range(200):
                view = store.recent("ATTITUDE", 16)
                assert isinstance(view, tuple)
                receipts = [sample.receipt_time_s for sample in view]
                assert receipts == sorted(receipts)
                assert len(view) <= 16
        finally:
            stop.set()
            thread.join(timeout=2.0)
        assert store.recent("ATTITUDE", 0) == ()

    def test_link_change_invalidates_calibration_on_the_next_frame(self) -> None:
        # Reconnect/endpoint change: the live identity reader returns the NEW
        # link and the previously admitting artifact stops admitting.
        artifact = _calibration()
        assert validate_calibration(artifact, "cam", "link#1") is None
        assert validate_calibration(artifact, "cam", "link#9") is not None


class TestPostReviewHardening:
    """Post-implementation review round: reproduced probes, pinned."""

    def test_malformed_artifact_refuses_instead_of_raising(self) -> None:
        broken = SimpleNamespace(source_id="cam", link_id="link#1")
        assert validate_calibration(broken, "cam", "link#1") is not None

    def test_unrecognized_stamp_kind_never_rides_a_branch(self) -> None:
        bogus = SimpleNamespace(kind="jpeg", captured_at_s=100.0)
        with pytest.raises(ValueError):
            attitude_lookup_wall_s(bogus, _calibration())

    def test_seam_tolerance_boundary_is_strictly_greater(self) -> None:
        # Strictly-greater semantics at the boundary, to within a
        # microsecond (exact float equality at a decimal constant is
        # ill-defined; 1e-6 s is far below every physical effect here).
        just_inside = (
            _sample(100.00, 50.00),
            _sample(100.04, 50.00 + 0.04 - CLOCK_SEAM_TOLERANCE_S + 1e-6),
        )
        result, status = interpolate_attitude_at(just_inside, 100.01, 0.0)
        assert status == "bracketed"
        just_outside = (
            _sample(100.00, 50.00),
            _sample(100.04, 50.00 + 0.04 - CLOCK_SEAM_TOLERANCE_S - 1e-6),
        )
        result, status = interpolate_attitude_at(just_outside, 100.01, 0.0)
        assert result is None and status == STATUS_CLOCK_SEAM

    def test_wall_clock_backward_step_is_a_seam(self) -> None:
        history = (_sample(100.00, 50.00), _sample(99.90, 50.02))
        result, status = interpolate_attitude_at(history, 99.95, 0.0)
        assert result is None

    def test_runtime_rate_parameters_tighten_the_bounds(self) -> None:
        # An 80 Hz stream's two-period ceiling is 25 ms: a 40 ms bracket the
        # 40 Hz default admits must be refused with runtime-rate bounds.
        history = (
            _sample(100.00, 50.00),
            _sample(100.04, 50.04),
        )
        result, status = interpolate_attitude_at(history, 100.02, 0.0)
        assert status == "bracketed"
        result, status = interpolate_attitude_at(
            history, 100.02, 0.0,
            bracket_max_width_s=0.025, seam_tolerance_s=0.0125,
        )
        assert result is None and status == STATUS_GAP

    def test_exposure_quarantine_excludes_camera_variation(self) -> None:
        # EXPOSURE window = Da0 alone; READ adds camera variation. A seam
        # between the two windows discriminates the kinds.
        artifact = _calibration()
        exposure_window = quarantine_window_s(
            CaptureStampKind.EXPOSURE, artifact,
        )
        read_window = quarantine_window_s(CaptureStampKind.READ, artifact)
        lookup = 100.05
        seam_receipt = lookup - (exposure_window + read_window) / 2.0
        history = (
            _sample(seam_receipt - 0.02, 50.00),
            _sample(seam_receipt, 10.00),
            _sample(lookup - 0.004, 10.01),
            _sample(lookup + 0.004, 10.02),
        )
        result, status = interpolate_attitude_at(
            history, lookup, read_window,
        )
        assert result is None and status == STATUS_CLOCK_SEAM
        result, status = interpolate_attitude_at(
            history, lookup, exposure_window,
        )
        assert status == "bracketed"

    def test_link_identity_appears_only_after_heartbeat(self) -> None:
        from navpy.modules.vehicle.pose_telemetry import PoseTelemetry

        confirmed = {"heartbeat": False}

        def source() -> str | None:
            return "dev#1" if confirmed["heartbeat"] else None

        telemetry = PoseTelemetry(
            message_store=SimpleNamespace(),
            command_diagnostics=SimpleNamespace(),
            link_identity_source=source,
        )
        assert telemetry.link_identity is None
        assert validate_calibration(
            _calibration(link_id="dev#1"), "cam", telemetry.link_identity,
        ) is not None
        confirmed["heartbeat"] = True
        assert telemetry.link_identity == "dev#1"
        assert validate_calibration(
            _calibration(link_id="dev#1"), "cam", telemetry.link_identity,
        ) is None
