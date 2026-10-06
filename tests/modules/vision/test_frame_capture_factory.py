"""Transactional ownership tests for frame-capture backend selection."""

from unittest.mock import Mock, PropertyMock, patch

import pytest

from navpy.modules.vision.frame_capture_factory import open_capture_backend
from navpy.modules.vision.frame_capture_opencv import OpenCvFrameCapture

from tests.modules.vision.test_frame_provider import _Logger


def test_opencv_wrapper_failure_releases_raw_capture() -> None:
    capture = Mock()
    capture.isOpened.return_value = True
    with patch(
        "navpy.modules.vision.frame_capture_factory.open_source_capture",
        return_value=capture,
    ):
        with patch(
            "navpy.modules.vision.frame_capture_factory.OpenCvFrameCapture",
            side_effect=RuntimeError("wrapper failed"),
        ):
            with pytest.raises(RuntimeError, match="wrapper failed"):
                open_capture_backend(0, _Logger())

    capture.release.assert_called_once_with()


def test_opencv_dimension_failure_releases_owned_capture() -> None:
    capture = Mock()
    capture.isOpened.return_value = True
    with patch(
        "navpy.modules.vision.frame_capture_factory.open_source_capture",
        return_value=capture,
    ):
        with patch.object(
            OpenCvFrameCapture,
            "dimensions",
            new_callable=PropertyMock,
            side_effect=RuntimeError("dimensions failed"),
        ):
            with pytest.raises(RuntimeError, match="dimensions failed"):
                open_capture_backend(0, _Logger())

    capture.release.assert_called_once_with()


def test_factory_recheck_false_releases_raw_capture() -> None:
    capture = Mock()
    capture.isOpened.return_value = False
    with patch(
        "navpy.modules.vision.frame_capture_factory.open_source_capture",
        return_value=capture,
    ):
        assert open_capture_backend(0, _Logger()) is None

    capture.release.assert_called_once_with()


def test_factory_recheck_error_releases_raw_capture() -> None:
    capture = Mock()
    capture.isOpened.side_effect = RuntimeError("recheck failed")
    with patch(
        "navpy.modules.vision.frame_capture_factory.open_source_capture",
        return_value=capture,
    ):
        with pytest.raises(RuntimeError, match="recheck failed"):
            open_capture_backend(0, _Logger())

    capture.release.assert_called_once_with()


def test_jetson_wrapper_failure_releases_raw_capture() -> None:
    capture = Mock()
    with patch(
        "navpy.modules.vision.frame_capture_factory._IS_JETSON",
        True,
    ):
        with patch(
            "navpy.modules.vision.frame_capture_factory.open_jetson_capture",
            return_value=capture,
        ):
            with patch(
                "navpy.modules.vision.frame_capture_factory.OpenCvFrameCapture",
                side_effect=RuntimeError("wrapper failed"),
            ):
                with pytest.raises(RuntimeError, match="wrapper failed"):
                    open_capture_backend("rtsp://camera/stream", _Logger())

    capture.release.assert_called_once_with()
