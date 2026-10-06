"""Public-contract and lifecycle regressions for the hardware SIYI adapter."""

from __future__ import annotations

import threading
import time
from unittest.mock import Mock, call, patch

import pytest

from navpy.exception_groups import ExceptionGroup
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.gimbal_siyi import GimbalSiyi
from navpy.modules.vision.peripheral.siyi.hardware.bounded_sdk import (
    BoundedSiyiSdk,
)


class _StrictLogger:
    """Logger seam that rejects stdlib-style formatting arguments."""

    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


class _Clock:
    def __init__(self, now_s: float) -> None:
        self.now_s = now_s

    def __call__(self) -> float:
        return self.now_s


class _RetainedPollThread:
    def __init__(self) -> None:
        self.alive = True
        self.join_calls = 0

    def is_alive(self) -> bool:
        return self.alive

    def join(self, timeout=None) -> None:
        self.join_calls += 1


def _sdk(
    *,
    connected: bool = True,
    attitude: tuple[int, float, float, float, float] = (
        7,
        1234.5,
        0.0,
        0.0,
        0.0,
    ),
    zoom: tuple[int, float, float] = (0, 0.0, 0.0),
) -> Mock:
    sdk = Mock()
    sdk.connect.return_value = connected
    sdk.isConnected.return_value = connected
    sdk.getAttitudeSample.return_value = attitude
    sdk.getCurrentZoomLevelSample.return_value = zoom
    for request_name in (
        "requestCenterGimbal",
        "requestLockMode",
        "requestFollowMode",
        "requestFPVMode",
        "requestAbsoluteZoom",
        "requestAutoFocus",
        "requestZoomIn",
        "requestZoomOut",
        "requestZoomHold",
        "requestSetAngles",
        "requestGimbalSpeed",
        "requestCurrentZoomLevel",
    ):
        getattr(sdk, request_name).return_value = True
    return sdk


def _gimbal(
    sdk: Mock,
    *,
    logger: _StrictLogger | None = None,
    clock=None,
) -> tuple[GimbalSiyi, _StrictLogger, Mock]:
    selected_logger = _StrictLogger() if logger is None else logger
    factory = Mock(return_value=sdk)
    data = GimbalData(
        att=Attitude(0.0, 0.0, 0.0),
        roll_stabilize=True,
        pitch_stabilize=True,
    )
    gimbal = GimbalSiyi(
        data,
        "192.168.144.25",
        37260,
        selected_logger,
        sdk_factory=factory,
        monotonic=time.monotonic if clock is None else clock,
    )
    return gimbal, selected_logger, factory


def test_facade_owns_only_composed_parts() -> None:
    gimbal, _, _ = _gimbal(_sdk())

    assert set(vars(gimbal)) == {"_parts"}


def test_constructed_attitude_is_available_before_start() -> None:
    data = GimbalData(att=Attitude(-15.0, 30.0, 5.0))
    gimbal = GimbalSiyi(
        data,
        "127.0.0.1",
        37260,
        _StrictLogger(),
        sdk_factory=Mock(return_value=_sdk()),
    )

    assert gimbal.get_data().att == Attitude(-15.0, 30.0, 5.0)


def test_zoom_readback_starts_unavailable() -> None:
    gimbal, _, _ = _gimbal(_sdk())

    assert gimbal.get_zoom_level() is None
    assert gimbal.get_zoom_level_age_s() == float("inf")
    assert gimbal.get_zoom_level_sample_id() is None
    assert gimbal.get_zoom_level_sample() is None
    assert gimbal.get_frame_state_sample()[1:] == (None, None, None)


@patch("navpy.modules.vision.peripheral.siyi.SIYISDK")
def test_existing_four_argument_constructor_uses_default_sdk(mock_sdk_cls) -> None:
    sdk = _sdk()
    mock_sdk_cls.return_value = sdk
    gimbal = GimbalSiyi(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        "10.0.0.2",
        5000,
        _StrictLogger(),
    )

    gimbal.start()
    try:
        mock_sdk_cls.assert_called_once_with(server_ip="10.0.0.2", port=5000)
    finally:
        gimbal.stop()


def test_start_is_idempotent_and_stop_releases_the_same_session_once() -> None:
    sdk = _sdk()
    gimbal, logger, factory = _gimbal(sdk)

    gimbal.start()
    gimbal.start()
    try:
        factory.assert_called_once_with(
            server_ip="192.168.144.25",
            port=37260,
        )
        sdk.connect.assert_called_once_with()
        sdk.requestCenterGimbal.assert_called_once_with()
        sdk.requestLockMode.assert_called_once_with()
        sdk.requestAbsoluteZoom.assert_called_once_with(1)
        sdk.requestAutoFocus.assert_called_once_with()
        assert gimbal.is_connected()
        assert "SIYI gimbal connected, Lock Mode enabled" in logger.infos
    finally:
        gimbal.stop()
        gimbal.stop()

    sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    sdk.disconnect.assert_called_once_with()
    assert not gimbal.is_connected()


def test_stop_retains_live_hardware_poll_worker_and_retries_quiescence() -> None:
    sdk = _sdk()
    gimbal, logger, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    retained = _RetainedPollThread()
    stop_event = threading.Event()
    lifecycle._sdk_owner._session.install(BoundedSiyiSdk(sdk))
    lifecycle._running = True
    lifecycle._poll_worker.ownership.stop_event = stop_event
    lifecycle._poll_worker.ownership.thread = retained  # type: ignore[assignment]

    assert gimbal.stop() is False
    assert stop_event.is_set()
    assert lifecycle._poll_worker.ownership.thread is retained
    assert gimbal.is_connected()
    sdk.disconnect.assert_not_called()
    assert logger.warnings == ["SIYI polling thread did not stop in time"]
    with pytest.raises(RuntimeError, match="shutdown is incomplete"):
        gimbal.start()

    retained.alive = False
    assert gimbal.stop() is True
    assert lifecycle._poll_worker.ownership.thread is None
    assert not gimbal.is_connected()
    sdk.disconnect.assert_called_once_with()


def test_stop_retains_sdk_ownership_when_disconnect_fails_for_retry() -> None:
    disconnect_error = OSError("disconnect failed")
    sdk = _sdk()
    sdk.disconnect.side_effect = [disconnect_error, None]
    gimbal, _, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    gimbal.start()

    with pytest.raises(OSError) as raised:
        gimbal.stop()
    assert raised.value is disconnect_error
    assert lifecycle._poll_worker.ownership.thread is None
    assert lifecycle._poll_worker.ownership.stop_event is None
    assert gimbal.is_connected()
    with pytest.raises(RuntimeError, match="shutdown is incomplete"):
        gimbal.start()

    assert gimbal.stop() is True
    assert lifecycle._poll_worker.ownership.thread is None
    assert lifecycle._poll_worker.ownership.stop_event is None
    assert not gimbal.is_connected()
    assert sdk.disconnect.call_count == 2


def test_failed_connection_is_cleaned_up_and_uses_ilogger_message_shape() -> None:
    sdk = _sdk(connected=False)
    gimbal, logger, _ = _gimbal(sdk)

    gimbal.start()
    gimbal.stop()

    sdk.disconnect.assert_called_once_with()
    sdk.requestLockMode.assert_not_called()
    assert logger.errors == [
        "Failed to connect to SIYI gimbal at 192.168.144.25:37260"
    ]


def test_failed_connection_is_compatible_with_real_console_logger(capsys) -> None:
    sdk = _sdk(connected=False)
    gimbal = GimbalSiyi(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        "192.168.144.25",
        37260,
        ConsoleLogger(),
        sdk_factory=Mock(return_value=sdk),
    )

    gimbal.start()

    assert (
        "Failed to connect to SIYI gimbal at 192.168.144.25:37260"
        in capsys.readouterr().out
    )


def test_expected_connect_io_error_is_reported_without_partial_session() -> None:
    sdk = _sdk()
    sdk.connect.side_effect = OSError("network unavailable")
    gimbal, logger, _ = _gimbal(sdk)

    gimbal.start()

    assert not gimbal.is_connected()
    sdk.disconnect.assert_called_once_with()
    assert logger.errors == [
        "Failed to connect to SIYI gimbal at "
        "192.168.144.25:37260: network unavailable"
    ]


def test_expected_sdk_creation_io_error_is_reported() -> None:
    logger = _StrictLogger()
    factory = Mock(side_effect=OSError("socket allocation failed"))
    gimbal = GimbalSiyi(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        "10.0.0.3",
        5001,
        logger,
        sdk_factory=factory,
    )

    gimbal.start()

    assert not gimbal.is_connected()
    assert logger.errors == [
        "Failed to create SIYI gimbal SDK for "
        "10.0.0.3:5001: socket allocation failed"
    ]


def test_partial_payload_setup_is_stopped_and_disconnected() -> None:
    sdk = _sdk()
    sdk.requestLockMode.side_effect = RuntimeError("setup contract failure")
    gimbal, _, _ = _gimbal(sdk)

    with pytest.raises(RuntimeError, match="setup contract failure"):
        gimbal.start()

    sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    sdk.disconnect.assert_called_once_with()
    assert not gimbal.is_connected()


def test_false_lock_ack_fails_setup_before_connected_log() -> None:
    sdk = _sdk()
    sdk.requestLockMode.return_value = False
    gimbal, logger, _ = _gimbal(sdk)

    with pytest.raises(RuntimeError, match="lock-mode.*not acknowledged"):
        gimbal.start()

    sdk.requestAbsoluteZoom.assert_not_called()
    sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    sdk.disconnect.assert_called_once_with()
    assert not any("Lock Mode enabled" in line for line in logger.infos)


def test_payload_shutdown_retries_zero_rate_before_disconnect() -> None:
    sdk = _sdk()
    sdk.requestGimbalSpeed.side_effect = [False, True]
    bounded = BoundedSiyiSdk(sdk, timeout_s=0.01)

    with pytest.raises(RuntimeError, match="zero-rate.*not acknowledged"):
        bounded.shutdown_payload()
    sdk.disconnect.assert_not_called()

    assert bounded.shutdown_payload() is True
    assert sdk.requestGimbalSpeed.call_count == 2
    sdk.disconnect.assert_called_once_with()


def test_sdk_disconnect_timeout_is_bounded_and_retryable() -> None:
    entered = threading.Event()
    release = threading.Event()
    sdk = _sdk()

    def block_disconnect():
        entered.set()
        assert release.wait(timeout=1.0)

    sdk.disconnect.side_effect = block_disconnect
    bounded = BoundedSiyiSdk(sdk, timeout_s=0.01)

    started_s = time.perf_counter()
    assert bounded.disconnect() is False
    assert time.perf_counter() - started_s < 0.2
    assert entered.is_set()

    release.set()
    assert bounded.disconnect() is True
    sdk.disconnect.assert_called_once_with()


def test_sdk_prelaunch_interrupt_allows_fresh_disconnect_retry() -> None:
    interrupt = KeyboardInterrupt()
    sdk = _sdk()
    bounded = BoundedSiyiSdk(sdk, timeout_s=0.01)

    with patch.object(threading.Thread, "start", side_effect=interrupt):
        with pytest.raises(KeyboardInterrupt) as raised:
            bounded.disconnect()

    assert raised.value is interrupt
    assert bounded.disconnect() is True
    sdk.disconnect.assert_called_once_with()


def test_sdk_delayed_normal_bootstrap_is_cancelled_before_retry() -> None:
    release = threading.Event()
    finished = threading.Event()
    original_thread = threading.Thread
    sdk = _sdk()
    bounded = BoundedSiyiSdk(sdk, timeout_s=0.01)

    class DelayedDisconnectThread:
        def __init__(self, target, args=(), **_kwargs):
            self._target = target
            self._args = args
            self._native = None

        def start(self):
            def bootstrap():
                try:
                    assert release.wait(timeout=1.0)
                    self._target(*self._args)
                finally:
                    finished.set()

            self._native = original_thread(target=bootstrap, daemon=True)
            self._native.start()

        def is_alive(self):
            return self._native is not None and self._native.is_alive()

        def join(self, timeout=None):
            if self._native is not None:
                self._native.join(timeout=timeout)

    def thread_factory(*args, **kwargs):
        if kwargs.get("name") == "siyi-sdk-disconnect":
            return DelayedDisconnectThread(*args, **kwargs)
        return original_thread(*args, **kwargs)

    try:
        with patch.object(threading, "Thread", side_effect=thread_factory):
            assert bounded.disconnect() is False
        sdk.disconnect.assert_not_called()
    finally:
        release.set()
        assert finished.wait(timeout=1.0)

    assert bounded.disconnect() is True
    sdk.disconnect.assert_called_once_with()


def test_disconnected_installed_sdk_remains_owned_until_stop() -> None:
    sdk = _sdk(connected=False)
    gimbal, _, factory = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    lifecycle._sdk_owner._session.install(BoundedSiyiSdk(sdk))

    assert lifecycle._sdk_owner.has_cleanup
    sdk.isConnected.assert_not_called()
    with pytest.raises(RuntimeError, match="shutdown is incomplete"):
        gimbal.start()
    factory.assert_not_called()

    assert gimbal.stop() is True
    sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    sdk.disconnect.assert_called_once_with()
    assert not lifecycle._sdk_owner.has_cleanup


def test_session_install_collision_cleans_new_sdk_and_retains_old() -> None:
    old_sdk = _sdk(connected=False)
    new_sdk = _sdk()
    gimbal, _, factory = _gimbal(new_sdk)
    lifecycle = gimbal._parts.lifecycle
    session = lifecycle._sdk_owner._session
    old_owner = BoundedSiyiSdk(old_sdk)

    def install_old_owner() -> bool:
        session.install(old_owner)
        return True

    new_sdk.requestAutoFocus.side_effect = install_old_owner

    with pytest.raises(RuntimeError, match="already installed"):
        gimbal.start()

    factory.assert_called_once_with(
        server_ip="192.168.144.25",
        port=37260,
    )
    new_sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    new_sdk.disconnect.assert_called_once_with()
    old_sdk.requestGimbalSpeed.assert_not_called()
    old_sdk.disconnect.assert_not_called()
    assert lifecycle._sdk_owner.has_cleanup

    assert gimbal.stop() is True
    old_sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    old_sdk.disconnect.assert_called_once_with()


def test_delayed_poll_bootstrap_is_cancelled_before_sdk_retirement() -> None:
    release = threading.Event()
    finished = threading.Event()
    original_thread = threading.Thread
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    lifecycle._poll_worker._poller.run = Mock()

    class DelayedPollThread:
        ident = None

        def __init__(self, target, args=(), **_kwargs):
            self._target = target
            self._args = args
            self._alive = False
            self._native = None

        def start(self):
            def bootstrap():
                try:
                    assert release.wait(timeout=1.0)
                    self._alive = True
                    self._target(*self._args)
                finally:
                    self._alive = False
                    finished.set()

            self._native = original_thread(target=bootstrap, daemon=True)
            self._native.start()
            raise KeyboardInterrupt("poll start interrupted")

        def is_alive(self):
            return self._alive

        def join(self, timeout=None):
            if self._native is not None:
                self._native.join(timeout=timeout)

    def thread_factory(*args, **kwargs):
        if kwargs.get("name") == "siyi-hardware-poll":
            return DelayedPollThread(*args, **kwargs)
        return original_thread(*args, **kwargs)

    try:
        with patch.object(threading, "Thread", side_effect=thread_factory):
            with pytest.raises(KeyboardInterrupt, match="poll start interrupted"):
                gimbal.start()

        sdk.disconnect.assert_called_once_with()
        assert not gimbal.is_connected()
        assert gimbal.stop() is True
    finally:
        release.set()
        assert finished.wait(timeout=1.0)

    lifecycle._poll_worker._poller.run.assert_not_called()


def test_poll_start_failure_after_launch_retains_owned_session() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    lifecycle._poll_worker._poller.run = Mock()
    original_start = threading.Thread.start

    def launch_then_fail(thread):
        original_start(thread)
        thread.join(timeout=1.0)
        raise RuntimeError("poll launch acknowledgement failed")

    with patch.object(threading.Thread, "start", launch_then_fail):
        with pytest.raises(RuntimeError, match="poll launch acknowledgement"):
            gimbal.start()

    assert lifecycle._poll_worker.ownership.thread is not None
    assert gimbal.is_connected()
    sdk.disconnect.assert_not_called()
    assert gimbal.stop() is True
    sdk.disconnect.assert_called_once_with()


def test_poll_failure_is_persistent_health_failure() -> None:
    failure = RuntimeError("poller failed")
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle
    lifecycle._poll_worker._poller.run = Mock(side_effect=failure)

    with patch(
        "navpy.modules.vision.peripheral.siyi.hardware.lifecycle."
        "ATTITUDE_READY_TIMEOUT_S",
        0.01,
    ):
        gimbal.start()

    for _ in range(2):
        with pytest.raises(RuntimeError) as raised:
            gimbal.raise_if_failed()
        assert raised.value is failure
    gimbal.stop()


def test_partial_setup_retains_sdk_when_disconnect_fails_for_stop_retry() -> None:
    setup_error = RuntimeError("setup contract failure")
    disconnect_error = OSError("disconnect failed")
    sdk = _sdk()
    sdk.requestLockMode.side_effect = setup_error
    sdk.disconnect.side_effect = [disconnect_error, None]
    gimbal, _, _ = _gimbal(sdk)
    lifecycle = gimbal._parts.lifecycle

    with pytest.raises(ExceptionGroup) as raised:
        gimbal.start()
    assert raised.value.exceptions == (setup_error, disconnect_error)
    assert lifecycle._sdk_owner._pending_sdk._sdk is sdk
    with pytest.raises(RuntimeError, match="shutdown is incomplete"):
        gimbal.start()

    assert gimbal.stop() is True
    assert lifecycle._sdk_owner._pending_sdk is None
    assert sdk.disconnect.call_count == 2


def test_valid_zero_roll_marks_attitude_ready_without_one_second_delay() -> None:
    gimbal, _, _ = _gimbal(
        _sdk(attitude=(4, 50.0, 10.0, -20.0, 0.0)),
    )

    started_s = time.perf_counter()
    gimbal.start()
    elapsed_s = time.perf_counter() - started_s
    try:
        assert elapsed_s < 0.75
        assert gimbal.get_data().att.roll == 0.0
    finally:
        gimbal.stop()


def test_attitude_and_zoom_preserve_sdk_receipt_instants() -> None:
    clock = _Clock(200.0)
    sdk = _sdk(
        attitude=(8, 123.456, 10.0, -20.0, 5.0),
        zoom=(10, 3.0, 100.0),
    )
    gimbal, _, _ = _gimbal(sdk, clock=clock)

    gimbal.start()
    try:
        data, zoom, zoom_age_s, sample_id = gimbal.get_frame_state_sample()
        assert data.att == Attitude(-20.0, 10.0, 5.0)
        assert data.timestamp_s == 123.456
        assert zoom == 3.0
        assert zoom_age_s == 100.0
        assert sample_id == 10
        assert gimbal.get_zoom_level_sample() == (3.0, 100.0, 10)
    finally:
        gimbal.stop()


def test_reconnect_resets_sequence_and_zoom_provenance() -> None:
    clock = _Clock(200.0)
    first_sdk = _sdk(
        attitude=(7, 10.0, 1.0, 2.0, 0.0),
        zoom=(11, 4.0, 190.0),
    )
    second_sdk = _sdk(
        attitude=(7, 20.0, 3.0, 4.0, 180.0),
        zoom=(0, 0.0, 0.0),
    )
    factory = Mock(side_effect=[first_sdk, second_sdk])
    logger = _StrictLogger()
    gimbal = GimbalSiyi(
        GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        "127.0.0.1",
        37260,
        logger,
        sdk_factory=factory,
        monotonic=clock,
    )

    gimbal.start()
    assert gimbal.get_data().timestamp_s == 10.0
    assert gimbal.get_zoom_level_sample_id() == 11
    gimbal.stop()

    clock.now_s = 300.0
    gimbal.start()
    try:
        assert gimbal.get_data().timestamp_s == 20.0
        assert gimbal.get_data().att == Attitude(4.0, 3.0, 180.0)
        assert gimbal.get_zoom_level() is None
        assert gimbal.get_zoom_level_sample_id() is None
        second_sdk.reset_mock()
        gimbal.set_rate(0.0, 10.0)
        second_sdk.requestGimbalSpeed.assert_called_once_with(0, -10)
    finally:
        gimbal.stop()

    assert factory.call_count == 2


def test_commands_preserve_public_mapping_and_rounding() -> None:
    sdk = _sdk()
    gimbal, logger, _ = _gimbal(sdk)
    gimbal.start()
    sdk.reset_mock()
    try:
        gimbal.set_att(Attitude(-30.0, 45.0, 0.0))
        gimbal.set_rate(25.5, -30.7)
        gimbal.set_rate(200.0, -150.0)
        assert gimbal.set_zoom("2")
        assert gimbal.zoom_in()
        assert gimbal.zoom_out()
        assert gimbal.zoom_hold()
        gimbal.request_autofocus()
        gimbal.set_motion_mode(0)
        gimbal.set_motion_mode(1)
        gimbal.set_motion_mode(2)
        gimbal.set_motion_mode(99)

        sdk.requestSetAngles.assert_called_once_with(45.0, -30.0)
        assert sdk.requestGimbalSpeed.call_args_list == [
            call(26, -31),
            call(100, -100),
        ]
        sdk.requestAbsoluteZoom.assert_called_once_with(2.0)
        sdk.requestZoomIn.assert_called_once_with()
        sdk.requestZoomOut.assert_called_once_with()
        sdk.requestZoomHold.assert_called_once_with()
        sdk.requestAutoFocus.assert_called_once_with()
        sdk.requestLockMode.assert_called_once_with()
        sdk.requestFollowMode.assert_called_once_with()
        sdk.requestFPVMode.assert_called_once_with()
        assert logger.warnings[-1] == "SIYI: ignoring unknown motion mode 99"
    finally:
        gimbal.stop()


def test_negative_yaw_rate_keeps_its_sign_and_uses_nearest_integer() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    sdk.reset_mock()
    try:
        gimbal.set_rate(-18.9, 12.4)
        sdk.requestGimbalSpeed.assert_called_once_with(-19, 12)
    finally:
        gimbal.stop()


def test_invalid_zoom_value_does_not_reach_sdk() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    sdk.reset_mock()
    try:
        assert not gimbal.set_zoom("not-a-number")
        sdk.requestAbsoluteZoom.assert_not_called()
    finally:
        gimbal.stop()


def test_zoom_send_failure_is_returned_to_the_caller() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    sdk.requestZoomIn.return_value = False
    sdk.requestZoomOut.return_value = False
    sdk.requestZoomHold.return_value = False
    try:
        assert not gimbal.zoom_in()
        assert not gimbal.zoom_out()
        assert not gimbal.zoom_hold()
    finally:
        gimbal.stop()


def test_truthy_non_boolean_zoom_reply_is_not_an_acknowledgement() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    sdk.requestAbsoluteZoom.return_value = 1
    sdk.requestZoomIn.return_value = "sent"
    try:
        assert not gimbal.set_zoom("2")
        assert not gimbal.zoom_in()
    finally:
        gimbal.stop()


def test_motion_mode_failure_remains_visible_to_navigation() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    sdk.requestLockMode.return_value = False
    try:
        with pytest.raises(RuntimeError, match="LOCK.*UDP send failed"):
            gimbal.set_motion_mode(0)
    finally:
        gimbal.stop()


def test_disconnected_commands_remain_quiet_noops() -> None:
    sdk = _sdk()
    gimbal, logger, _ = _gimbal(sdk)

    gimbal.set_att(Attitude(0.0, 0.0, 0.0))
    gimbal.set_rate(10.0, 10.0)
    gimbal.set_motion_mode(0)
    gimbal.set_motion_mode(99)
    assert not gimbal.zoom_in()
    assert not gimbal.zoom_out()
    assert not gimbal.zoom_hold()
    gimbal.request_autofocus()

    assert not gimbal.set_zoom("2")
    assert not gimbal.is_connected()
    assert logger.warnings == []
    sdk.assert_not_called()


def test_is_connected_reflects_live_sdk_connection_state() -> None:
    sdk = _sdk()
    gimbal, _, _ = _gimbal(sdk)
    gimbal.start()
    try:
        assert gimbal.is_connected()
        sdk.isConnected.return_value = False
        assert not gimbal.is_connected()
    finally:
        gimbal.stop()


def test_concurrent_start_and_stop_are_serialized_and_idempotent() -> None:
    sdk = _sdk()
    gimbal, _, factory = _gimbal(sdk)

    starters = [threading.Thread(target=gimbal.start) for _ in range(6)]
    for thread in starters:
        thread.start()
    for thread in starters:
        thread.join(timeout=2.0)
        assert not thread.is_alive()

    stoppers = [threading.Thread(target=gimbal.stop) for _ in range(6)]
    for thread in stoppers:
        thread.start()
    for thread in stoppers:
        thread.join(timeout=2.0)
        assert not thread.is_alive()

    factory.assert_called_once()
    sdk.connect.assert_called_once()
    sdk.requestGimbalSpeed.assert_called_once_with(0, 0)
    sdk.disconnect.assert_called_once()
