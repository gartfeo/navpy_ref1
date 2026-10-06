import weakref

import pytest

from navpy.exception_groups import (
    BaseExceptionGroup,
    ExceptionGroup,
    _FallbackBaseExceptionGroup,
    _FallbackExceptionGroup,
)
from navpy.modules.vision.models.detect_data import DetectionSizeClass, DetectedObject
from navpy.modules.vision.models.detection_components import (
    ConfirmationEvidence,
    DetectionClassification,
    DetectionGeoDiagnostics,
    DetectionIdentity,
    OpticsProvenance,
    PoseProvenance,
    SourceTiming,
    TrackingEvidence,
)
from navpy.modules.vision.models.pixel_observation import (
    CameraToBodyTransform,
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
)


def test_exception_group_contract_exposes_immutable_members():
    errors = (ValueError("first"), RuntimeError("second"))

    group = ExceptionGroup("cleanup failed", errors)

    assert isinstance(group, BaseExceptionGroup)
    assert group.exceptions == errors
    assert "cleanup failed" in str(group)
    with pytest.raises((AttributeError, TypeError)):
        group.exceptions[0] = RuntimeError("replacement")


def test_python_310_fallback_preserves_cleanup_group_catching_contract():
    errors = (ValueError("first"), RuntimeError("second"))

    group = _FallbackExceptionGroup("cleanup failed", errors)

    assert isinstance(group, _FallbackBaseExceptionGroup)
    assert isinstance(group, Exception)
    assert group.message == "cleanup failed"
    assert group.exceptions == errors
    with pytest.raises(AttributeError):
        group.exceptions = errors[:1]
    with pytest.raises(TypeError):
        _FallbackExceptionGroup("invalid", (KeyboardInterrupt(),))


def test_python_310_base_group_downcasts_normal_cleanup_failures():
    group = _FallbackBaseExceptionGroup("cleanup failed", (RuntimeError(),))

    assert isinstance(group, _FallbackExceptionGroup)
    assert isinstance(group, Exception)


def test_python_310_fallback_supports_base_exception_cleanup_failures():
    errors = (KeyboardInterrupt(), RuntimeError("cleanup"))

    group = _FallbackBaseExceptionGroup("cleanup failed", errors)

    assert isinstance(group, BaseException)
    assert not isinstance(group, Exception)
    assert group.exceptions == errors
    with pytest.raises(ValueError):
        _FallbackBaseExceptionGroup("empty", ())


def test_detected_poi_remains_weak_referenceable_without_python_311_slot_api():
    calibration = PixelCalibration(1.0, 1.0, 0.0, 0.0)
    poi = DetectedObject(
        DetectionIdentity(1, 2),
        DetectionClassification(DetectionSizeClass.S),
        PixelObservation(
            0.0,
            0.0,
            calibration,
            CameraToBodyTransform.from_matrix(
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
            ),
            PixelProjectionKind.PINHOLE,
            0.0,
            0.0,
            1.0,
            True,
            "test",
        ),
        TrackingEvidence(),
        ConfirmationEvidence(),
        PoseProvenance(None, None),
        OpticsProvenance(calibration),
        SourceTiming(),
        DetectionGeoDiagnostics(),
    )

    assert weakref.ref(poi)() is poi
    assert not hasattr(poi, "__dict__")
