"""Production consumers stay on the explicit public vehicle contract."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import threading
import time
from unittest.mock import MagicMock, patch

from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.parameter_client import ParameterReader, ParameterWriter, SimAutopilotState
from navpy.modules.vehicle.parameter_repository import ParameterRepository

def test_operator_refresh_bypasses_cached_value_and_waits_for_fresh_echo():
    messages = MessageStore()
    repository = ParameterRepository()
    repository.update("TEST", 1.0)
    connection = SimpleNamespace(mav=MagicMock())
    fresh = MagicMock(param_id="TEST", param_value=2.0)
    fresh.get_srcSystem.return_value = 7
    connection.mav.param_request_read_send.side_effect = lambda *_args: (
        messages.publish(
            "PARAM_VALUE",
            fresh,
            receipt_time_s=time.time(),
        )
    )
    reader = ParameterReader(
        SimpleNamespace(target_system=7),
        MavTransport(connection, threading.RLock()),
        repository,
        messages,
        SimpleNamespace(value=MagicMock()),
    )

    assert reader.get("TEST") == 1.0
    assert reader.get_fresh("TEST", timeout=0.1) == 2.0
    connection.mav.param_request_read_send.assert_called_once()


def test_nonblocking_parameter_write_does_not_wait_for_busy_transport():
    send_lock = threading.RLock()
    lock_held = threading.Event()
    release_lock = threading.Event()

    def hold_transport():
        with send_lock:
            lock_held.set()
            release_lock.wait(timeout=1.0)

    holder = threading.Thread(target=hold_transport, daemon=True)
    holder.start()
    assert lock_held.wait(timeout=0.5)
    writer = ParameterWriter(
        SimpleNamespace(target_system=7),
        MavTransport(SimpleNamespace(mav=MagicMock()), send_lock),
        ParameterRepository(),
        MessageStore(),
        SimpleNamespace(value=MagicMock()),
        SimAutopilotState(),
    )
    started_s = time.monotonic()
    result = writer.send_unverified("TEST", 3.0)
    elapsed_s = time.monotonic() - started_s
    release_lock.set()
    holder.join(timeout=0.5)

    assert not result
    assert elapsed_s < 0.1


def test_manual_control_diagnostic_uses_transport_sender_identity():
    from gcs.backend.routes import telemetry

    vehicle = MagicMock(
        source_system=7,
        transport_source_system=255,
    )
    entry = SimpleNamespace(vehicle=vehicle)
    telemetry._mc_state.clear()
    try:
        with patch.object(
            telemetry.vehicle_mgr, "get_vehicle", return_value=entry,
        ), patch.object(telemetry, "_ensure_mc_loop"), patch.object(
            telemetry.log, "info",
        ) as info:
            telemetry._handle_manual_control({"sys_id": 7})

        info.assert_called_once_with(
            "manual_control started for vehicle %d (src_sys=%d)",
            7,
            255,
        )
    finally:
        telemetry._mc_state.clear()


def test_current_gcs_consumer_does_not_reach_vehicle_transport_or_cache():
    root = Path(__file__).resolve().parents[3]
    source = (root / "src/gcs/backend/routes/telemetry.py").read_text(
        encoding="utf-8"
    )
    assert "._conn" not in source
    assert "._latest" not in source
