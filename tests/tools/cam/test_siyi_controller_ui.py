"""Tests for tools.cam.siyi_controller_ui."""

import importlib
import os
import sys
from unittest.mock import MagicMock, patch

_UI_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "..",
    "tools", "cam", "siyi_controller_ui.py",
))


def _import_module():
    """Import the UI module."""
    mod_name = "tools.cam.siyi_controller_ui"
    sys.modules.pop(mod_name, None)
    spec = importlib.util.spec_from_file_location(
        mod_name,
        _UI_FILE,
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _make_app(mod):
    """Create an app instance with _build_ui patched out, then stub the vars."""
    mock_root = MagicMock()
    with patch.object(mod.SiyiControllerApp, "_build_ui"):
        app = mod.SiyiControllerApp(mock_root, "127.0.0.1", 37260)
    # Stub the tk vars that would normally be created by _build_ui
    app.status_var = MagicMock()
    app.conn_btn = MagicMock()
    app.yaw_var = MagicMock()
    app.pitch_var = MagicMock()
    app.roll_var = MagicMock()
    app.zoom_display_var = MagicMock()
    app.zoom_var = MagicMock()
    app.zoom_label_var = MagicMock()
    app.rate_label_var = MagicMock()
    app.set_yaw_var = MagicMock()
    app.set_pitch_var = MagicMock()
    app.pad = MagicMock()
    app.pad_dot = "dot"
    app.camera_var = MagicMock()
    app.video_btn = MagicMock()
    app.video_canvas = MagicMock()
    app._canvas_image_id = "img"
    return app


# ---- Pure function tests ----

def test_clamp():
    mod = _import_module()
    assert mod._clamp(5, 0, 10) == 5
    assert mod._clamp(-1, 0, 10) == 0
    assert mod._clamp(15, 0, 10) == 10
    assert mod._clamp(0, 0, 0) == 0


def test_clamp_float():
    mod = _import_module()
    assert mod._clamp(1.5, 1.0, 30.0) == 1.5
    assert mod._clamp(0.5, 1.0, 30.0) == 1.0
    assert mod._clamp(35.0, 1.0, 30.0) == 30.0


def test_pad_size_constants():
    mod = _import_module()
    assert mod._PAD_SIZE > 0
    assert mod._PAD_HALF == mod._PAD_SIZE // 2
    assert mod._DEADZONE >= 0


# ---- App construction ----

def test_app_instantiation():
    mod = _import_module()
    app = _make_app(mod)
    assert app.ip == "127.0.0.1"
    assert app.port == 37260
    assert app.connected is False
    assert app.sdk is None


# ---- Zoom stepping ----

def test_step_zoom_up():
    mod = _import_module()
    app = _make_app(mod)
    app.zoom_var.get.return_value = 1.0
    app._step_zoom(1)
    app.zoom_var.set.assert_called_with(2.0)


def test_step_zoom_down_at_min():
    mod = _import_module()
    app = _make_app(mod)
    app.zoom_var.get.return_value = 1.0
    app._step_zoom(-1)
    app.zoom_var.set.assert_called_with(1.0)


def test_step_zoom_up_at_max():
    mod = _import_module()
    app = _make_app(mod)
    app.zoom_var.get.return_value = 30.0
    app._step_zoom(1)
    app.zoom_var.set.assert_called_with(30.0)


# ---- Rate control ----

def test_send_rate_no_sdk():
    mod = _import_module()
    app = _make_app(mod)
    app._send_rate(50, -50)  # No exception


def test_send_rate_with_sdk():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._send_rate(42, -17)
    app.sdk.requestGimbalSpeed.assert_called_once_with(42, -17)


# ---- Mode commands ----

def test_mode_fpv():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._mode_fpv()
    app.sdk.requestFPVMode.assert_called_once()


def test_mode_lock():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._mode_lock()
    app.sdk.requestLockMode.assert_called_once()


def test_mode_follow():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._mode_follow()
    app.sdk.requestFollowMode.assert_called_once()


def test_center():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._center()
    app.sdk.requestCenterGimbal.assert_called_once()


# ---- Camera actions ----

def test_take_photo():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._take_photo()
    app.sdk.requestPhoto.assert_called_once()


def test_toggle_record():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._toggle_record()
    app.sdk.requestRecording.assert_called_once()


def test_auto_focus():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app._auto_focus()
    app.sdk.requestAutoFocus.assert_called_once()


# ---- Set angles ----

def test_set_angles_valid():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app.set_yaw_var.get.return_value = "45.5"
    app.set_pitch_var.get.return_value = "-30"
    app._set_angles()
    app.sdk.requestSetAngles.assert_called_once_with(45.5, -30.0)


def test_set_angles_invalid():
    mod = _import_module()
    app = _make_app(mod)
    app.sdk = MagicMock()
    app.connected = True
    app.set_yaw_var.get.return_value = "abc"
    app.set_pitch_var.get.return_value = "0"
    app._set_angles()
    app.sdk.requestSetAngles.assert_not_called()


# ---- Disconnect ----

def test_disconnect_cleans_up():
    mod = _import_module()
    app = _make_app(mod)
    mock_sdk = MagicMock()
    app.sdk = mock_sdk
    app.connected = True

    app._disconnect()

    assert app.sdk is None
    assert app.connected is False
    mock_sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    mock_sdk.disconnect.assert_called_once()
    app.conn_btn.configure.assert_called_with(text="Connect")


# ---- Video ----

def test_start_video_empty_source():
    """Start video with empty source sets status, doesn't start thread."""
    mod = _import_module()
    app = _make_app(mod)
    app.camera_var.get.return_value = ""
    app._start_video()
    assert not app._video_running
    app.status_var.set.assert_called_with("No camera source")


def test_stop_video_resets_state():
    """Stop video cleans up capture and button state."""
    mod = _import_module()
    app = _make_app(mod)
    app._video_running = True
    mock_cap = MagicMock()
    app._video_cap = mock_cap

    app._stop_video()

    assert not app._video_running
    assert app._video_cap is None
    mock_cap.release.assert_called_once()
    app.video_btn.configure.assert_called_with(text="Start Video")


def test_toggle_video_starts_and_stops():
    """Toggle video starts when not running, stops when running."""
    mod = _import_module()
    app = _make_app(mod)
    app.camera_var.get.return_value = ""

    # Toggle on (will fail due to empty source, but exercises the path)
    app._video_running = False
    app._toggle_video()  # calls _start_video

    # Toggle off
    app._video_running = True
    app._toggle_video()  # calls _stop_video
    assert not app._video_running


def test_display_frame_sets_image():
    """_display_frame updates the video canvas."""
    mod = _import_module()
    app = _make_app(mod)
    app._video_running = True
    # Canvas reports small size so resize is called
    app.video_canvas.winfo_width.return_value = 640
    app.video_canvas.winfo_height.return_value = 360

    mock_pil = MagicMock()
    mock_resized = MagicMock()
    mock_pil.resize.return_value = mock_resized
    mock_photo = MagicMock()
    original_imgtk = mod.ImageTk
    try:
        mock_imgtk = MagicMock()
        mock_imgtk.PhotoImage.return_value = mock_photo
        mod.ImageTk = mock_imgtk
        app._display_frame(mock_pil)
    finally:
        mod.ImageTk = original_imgtk

    assert app._tk_image is mock_photo
    app.video_canvas.itemconfigure.assert_called_with("img", image=mock_photo)


def test_display_frame_skips_when_stopped():
    """_display_frame does nothing when video is not running."""
    mod = _import_module()
    app = _make_app(mod)
    app._video_running = False
    app._display_frame(MagicMock())
    app.video_canvas.itemconfigure.assert_not_called()


def test_video_loop_bad_source():
    """_video_loop with a bad source calls _stop_video via root.after."""
    mod = _import_module()
    app = _make_app(mod)

    mock_cap = MagicMock()
    mock_cap.isOpened.return_value = False
    with patch("tools.cam.siyi_controller_ui.cv2") as mock_cv2:
        mock_cv2.VideoCapture.return_value = mock_cap
        app._video_loop("bad_source")

    # Should have scheduled status + stop via root.after
    assert app.root.after.call_count >= 2


def test_camera_url_default():
    """Camera URL defaults to RTSP from IP when not provided."""
    mod = _import_module()
    mock_root = MagicMock()
    with patch.object(mod.SiyiControllerApp, "_build_ui"):
        app = mod.SiyiControllerApp(mock_root, "10.0.0.5", 37260)
    assert app.camera_url is None  # auto-derived in _build_ui


def test_camera_url_override():
    """Camera URL uses the provided override."""
    mod = _import_module()
    mock_root = MagicMock()
    with patch.object(mod.SiyiControllerApp, "_build_ui"):
        app = mod.SiyiControllerApp(mock_root, "10.0.0.5", 37260, camera="rtsp://custom:8554/stream")
    assert app.camera_url == "rtsp://custom:8554/stream"
